// SPDX-License-Identifier: GPL-2.0
/*
 * V4L2 sensor driver for STMicroelectronics VB1940 image sensor.
 *
 * Standalone (non-tegracam) driver derived from the in-tree tegracam-based
 * "s56" driver (see s56_ctrl.c, s56_mode_tbls.h). The VB1940 is typically
 * deployed behind a MAX9295 serializer / MAX96712 deserializer GMSL2 link;
 * this driver only programs the sensor itself. Serializer/deserializer
 * routing is expected to be handled by the corresponding serdes drivers.
 *
 * Capabilities exposed:
 *   - One mode: 2560x1984 @ 30 fps, RAW10, 4-lane CSI-2
 *   - Controls: V4L2_CID_ANALOGUE_GAIN, V4L2_CID_EXPOSURE,
 *               V4L2_CID_PIXEL_RATE, V4L2_CID_LINK_FREQ
 */

#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/types.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "vb1940_regs.h"

#define VB1940_DRV_NAME			"vb1940"

/* Gain: register encodes 16 * (gain - 100) / gain, gain in 1/100 units */
#define VB1940_GAIN_FACTOR		100
#define VB1940_GAIN_MIN			100
#define VB1940_GAIN_MAX			400
#define VB1940_GAIN_DEFAULT		VB1940_GAIN_MIN
#define VB1940_GAIN_STEP		1

/* Exposure in microseconds; converted to row units (×1000 / 8992 ns per row) */
#define VB1940_COARSE_TIME_SCALE	1000
#define VB1940_COARSE_TIME_PER_ROW_NS	8992
#define VB1940_EXPOSURE_MIN_US		40
#define VB1940_EXPOSURE_MAX_US		30000
#define VB1940_EXPOSURE_DEFAULT_US	30000
#define VB1940_EXPOSURE_STEP_US		1

/* Mode timing */
#define VB1940_NATIVE_WIDTH		2560
#define VB1940_NATIVE_HEIGHT		1984
#define VB1940_FPS_DEFAULT		30

/*
 * MIPI link parameters. The MAX96712 deserializer is configured for
 * 2.3 Gbps/lane in the reference s56 setup; the value below is the
 * symbol clock half-rate exposed to userspace.
 */
#define VB1940_LINK_FREQ_HZ		1150000000ULL	/* 2300 Mbps/lane */
#define VB1940_PIXEL_RATE \
	div_u64(VB1940_LINK_FREQ_HZ * 2ULL * 4ULL, 10ULL)


struct vb1940_mode {
	u32 width;
	u32 height;
	u32 fps;
	u32 code;
	const struct vb1940_reg *reg_list;
};


static const struct vb1940_mode vb1940_modes[] = {
	{
		.width    = VB1940_NATIVE_WIDTH,
		.height   = VB1940_NATIVE_HEIGHT,
		.fps      = VB1940_FPS_DEFAULT,
		.code     = MEDIA_BUS_FMT_SRGGB10_1X10,
		.reg_list = vb1940_2560x1984_30fps_regs,
	},
};

static const s64 vb1940_link_freq_menu[] = {
	VB1940_LINK_FREQ_HZ,
};

struct vb1940 {
	struct i2c_client	*client;
	struct regmap		*regmap;
	struct v4l2_subdev	sd;
	struct media_pad	pad;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl	*pixel_rate;
	struct v4l2_ctrl	*link_freq;
	struct v4l2_ctrl	*gain;
	struct v4l2_ctrl	*exposure;

	struct mutex		lock;	/* serialises s_stream / set_ctrl */
	const struct vb1940_mode *cur_mode;
	bool			streaming;
};

static inline struct vb1940 *to_vb1940(struct v4l2_subdev *sd)
{
	return container_of(sd, struct vb1940, sd);
}

/* ---------------------------------------------------------------- I/O */

/*
 * Apply a sensor register table using cci_multi_reg_write(). The source
 * format embeds DELAY and END pseudo-entries, so the table is walked once
 * to translate plain writes into a contiguous cci_reg_sequence buffer and
 * flushed at every DELAY / END boundary.
 */
static int vb1940_write_table(struct vb1940 *priv,
			      const struct vb1940_reg *table)
{
	struct cci_reg_sequence *seq;
	const struct vb1940_reg *r;
	size_t count = 0;
	unsigned int n = 0;
	int ret = 0;

	for (r = table; r->addr != VB1940_REG_END; r++)
		if (r->addr != VB1940_REG_DELAY)
			count++;

	if (!count)
		return 0;

	seq = kmalloc_array(count, sizeof(*seq), GFP_KERNEL);
	if (!seq)
		return -ENOMEM;

	for (r = table; ; r++) {
		if (r->addr == VB1940_REG_DELAY || r->addr == VB1940_REG_END) {
			if (n) {
				ret = cci_multi_reg_write(priv->regmap, seq,
							  n, NULL);
				if (ret)
					goto out;
				n = 0;
			}
			if (r->addr == VB1940_REG_END)
				break;
			msleep(r->val);
			continue;
		}
		seq[n].reg = CCI_REG8(r->addr);
		seq[n].val = r->val;
		n++;
	}

out:
	kfree(seq);
	return ret;
}

/* ----------------------------------------------------------- Controls */

static int vb1940_set_gain(struct vb1940 *priv, s32 gain)
{
	u8 reg;
	int ret = 0;

	gain = clamp_t(s32, gain, VB1940_GAIN_MIN, VB1940_GAIN_MAX);
	reg = (u8)(16 * (gain - VB1940_GAIN_FACTOR) / gain) & 0x0f;

	cci_write(priv->regmap, CCI_REG8(VB1940_REG_ANALOG_GAIN), reg, &ret);
	return ret;
}

static int vb1940_set_exposure(struct vb1940 *priv, s32 exposure_us)
{
	u32 shs;
	int ret = 0;

	exposure_us = clamp_t(s32, exposure_us,
			      VB1940_EXPOSURE_MIN_US,
			      VB1940_EXPOSURE_MAX_US);

	shs = ((u32)exposure_us * VB1940_COARSE_TIME_SCALE) /
	      VB1940_COARSE_TIME_PER_ROW_NS;

	cci_write(priv->regmap, CCI_REG16(VB1940_REG_EXPOSURE_LOW), shs, &ret);
	return ret;
}

static int vb1940_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct vb1940 *priv =
		container_of(ctrl->handler, struct vb1940, ctrls);
	int ret;

	if (!pm_runtime_get_if_in_use(&priv->client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = vb1940_set_gain(priv, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = vb1940_set_exposure(priv, ctrl->val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&priv->client->dev);
	return ret;
}

static const struct v4l2_ctrl_ops vb1940_ctrl_ops = {
	.s_ctrl = vb1940_s_ctrl,
};

static int vb1940_init_controls(struct vb1940 *priv)
{
	struct v4l2_ctrl_handler *h = &priv->ctrls;
	int ret;

	ret = v4l2_ctrl_handler_init(h, 4);
	if (ret)
		return ret;

	priv->pixel_rate = v4l2_ctrl_new_std(h, &vb1940_ctrl_ops,
					     V4L2_CID_PIXEL_RATE,
					     VB1940_PIXEL_RATE,
					     VB1940_PIXEL_RATE, 1,
					     VB1940_PIXEL_RATE);
	if (priv->pixel_rate)
		priv->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	priv->link_freq = v4l2_ctrl_new_int_menu(h, &vb1940_ctrl_ops,
						 V4L2_CID_LINK_FREQ,
						 ARRAY_SIZE(vb1940_link_freq_menu) - 1,
						 0, vb1940_link_freq_menu);
	if (priv->link_freq)
		priv->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	priv->gain = v4l2_ctrl_new_std(h, &vb1940_ctrl_ops,
				       V4L2_CID_ANALOGUE_GAIN,
				       VB1940_GAIN_MIN, VB1940_GAIN_MAX,
				       VB1940_GAIN_STEP, VB1940_GAIN_DEFAULT);

	priv->exposure = v4l2_ctrl_new_std(h, &vb1940_ctrl_ops,
					   V4L2_CID_EXPOSURE,
					   VB1940_EXPOSURE_MIN_US,
					   VB1940_EXPOSURE_MAX_US,
					   VB1940_EXPOSURE_STEP_US,
					   VB1940_EXPOSURE_DEFAULT_US);

	if (h->error) {
		ret = h->error;
		v4l2_ctrl_handler_free(h);
		return ret;
	}

	priv->sd.ctrl_handler = h;
	return 0;
}

/* --------------------------------------------------------- Subdev ops */

static int vb1940_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct vb1940 *priv = to_vb1940(sd);
	struct i2c_client *client = priv->client;
	int ret = 0;

	mutex_lock(&priv->lock);
	if (priv->streaming == !!enable)
		goto out;

	if (enable) {
		ret = pm_runtime_resume_and_get(&client->dev);
		if (ret < 0)
			goto out;

		ret = vb1940_write_table(priv, priv->cur_mode->reg_list);
		if (ret)
			goto err_rpm;

		ret = __v4l2_ctrl_handler_setup(&priv->ctrls);
		if (ret)
			goto err_rpm;

		ret = vb1940_write_table(priv, vb1940_stream_start);
		if (ret)
			goto err_rpm;
	} else {
		vb1940_write_table(priv, vb1940_stream_stop);
		pm_runtime_mark_last_busy(&client->dev);
		pm_runtime_put_autosuspend(&client->dev);
	}

	priv->streaming = !!enable;
	mutex_unlock(&priv->lock);
	return 0;

err_rpm:
	pm_runtime_put(&client->dev);
out:
	mutex_unlock(&priv->lock);
	return ret;
}

static void vb1940_fill_fmt(const struct vb1940_mode *mode,
			    struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = mode->code;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int vb1940_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *fmt;

	fmt = v4l2_subdev_state_get_format(state, 0);
	vb1940_fill_fmt(&vb1940_modes[0], fmt);
	return 0;
}

static int vb1940_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;
	code->code = vb1940_modes[0].code;
	return 0;
}

static int vb1940_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(vb1940_modes))
		return -EINVAL;
	if (fse->code != vb1940_modes[fse->index].code)
		return -EINVAL;

	fse->min_width  = fse->max_width  = vb1940_modes[fse->index].width;
	fse->min_height = fse->max_height = vb1940_modes[fse->index].height;
	return 0;
}

static int vb1940_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct vb1940 *priv = to_vb1940(sd);
	const struct vb1940_mode *mode = &vb1940_modes[0];
	struct v4l2_mbus_framefmt *mf;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(vb1940_modes); i++) {
		if (vb1940_modes[i].width == fmt->format.width &&
		    vb1940_modes[i].height == fmt->format.height) {
			mode = &vb1940_modes[i];
			break;
		}
	}

	vb1940_fill_fmt(mode, &fmt->format);
	mf = v4l2_subdev_state_get_format(state, 0);
	*mf = fmt->format;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		priv->cur_mode = mode;

	return 0;
}

static const struct v4l2_subdev_video_ops vb1940_video_ops = {
	.s_stream = vb1940_set_stream,
};

static const struct v4l2_subdev_pad_ops vb1940_pad_ops = {
	.enum_mbus_code   = vb1940_enum_mbus_code,
	.enum_frame_size  = vb1940_enum_frame_size,
	.get_fmt          = v4l2_subdev_get_fmt,
	.set_fmt          = vb1940_set_fmt,
};

static const struct v4l2_subdev_ops vb1940_subdev_ops = {
	.video = &vb1940_video_ops,
	.pad   = &vb1940_pad_ops,
};

static const struct v4l2_subdev_internal_ops vb1940_internal_ops = {
	.init_state = vb1940_init_state,
};

/* ---------------------------------------------------- Probe / remove */

static int vb1940_detect(struct vb1940 *priv)
{
	u64 v;
	int ret = 0;

	/*
	 * VB1940 does not expose a documented chip-id register in the
	 * reference sequence; perform a benign read from the stream-stop
	 * register to verify the sensor is responsive on I2C.
	 */
	cci_read(priv->regmap, CCI_REG8(VB1940_REG_STREAM_STOP), &v, &ret);
	if (ret)
		dev_warn(&priv->client->dev,
			 "I2C probe read returned %d\n", ret);
	return ret;
}

static int vb1940_probe(struct i2c_client *client)
{
	struct vb1940 *priv;
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client = client;
	priv->cur_mode = &vb1940_modes[0];
	mutex_init(&priv->lock);

	priv->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(priv->regmap)) {
		ret = PTR_ERR(priv->regmap);
		dev_err(&client->dev, "failed to init CCI regmap: %d\n", ret);
		goto err_mutex;
	}

	v4l2_i2c_subdev_init(&priv->sd, client, &vb1940_subdev_ops);
	priv->sd.internal_ops = &vb1940_internal_ops;
	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	priv->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	priv->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&priv->sd.entity, 1, &priv->pad);
	if (ret)
		goto err_mutex;

	ret = vb1940_init_controls(priv);
	if (ret)
		goto err_entity;

	ret = vb1940_detect(priv);
	if (ret)
		goto err_ctrls;

	priv->sd.state_lock = priv->ctrls.lock;
	ret = v4l2_subdev_init_finalize(&priv->sd);
	if (ret)
		goto err_ctrls;

	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_set_autosuspend_delay(&client->dev, 1000);
	pm_runtime_use_autosuspend(&client->dev);
	pm_runtime_idle(&client->dev);

	ret = v4l2_async_register_subdev_sensor(&priv->sd);
	if (ret)
		goto err_pm;

	dev_info(&client->dev, "VB1940 sensor probed @ 0x%02x\n", client->addr);
	return 0;

err_pm:
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
	v4l2_subdev_cleanup(&priv->sd);
err_ctrls:
	v4l2_ctrl_handler_free(&priv->ctrls);
err_entity:
	media_entity_cleanup(&priv->sd.entity);
err_mutex:
	mutex_destroy(&priv->lock);
	return ret;
}

static void vb1940_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct vb1940 *priv = to_vb1940(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&priv->ctrls);

	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	mutex_destroy(&priv->lock);
}

static const struct i2c_device_id vb1940_id[] = {
	{ "vb1940" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, vb1940_id);

static const struct of_device_id vb1940_of_match[] = {
	{ .compatible = "st,vb1940" },
	{ }
};
MODULE_DEVICE_TABLE(of, vb1940_of_match);

static const struct acpi_device_id vb1940_acpi_match[] = {
	{ "INTC1940" },
	{ }
};
MODULE_DEVICE_TABLE(acpi, vb1940_acpi_match);

static struct i2c_driver vb1940_i2c_driver = {
	.driver = {
		.name             = VB1940_DRV_NAME,
		.of_match_table   = vb1940_of_match,
		.acpi_match_table = vb1940_acpi_match,
	},
	.probe    = vb1940_probe,
	.remove   = vb1940_remove,
	.id_table = vb1940_id,
};
module_i2c_driver(vb1940_i2c_driver);

MODULE_AUTHOR("Generated standalone driver based on s56_ctrl.c");
MODULE_DESCRIPTION("STMicroelectronics VB1940 CMOS image sensor driver");
MODULE_LICENSE("GPL");
