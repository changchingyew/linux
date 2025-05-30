
// SPDX-License-Identifier: GPL-2.0
/*
 * isx031.c - Intel(R) RealSense(TM) ISX031 camera driver
 *
 * Copyright (c) 2017-2023, INTEL CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/unaligned.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/media.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/videodev2.h>
#include <linux/version.h>
#define CONFIG_VIDEO_INTEL_IPU6 1
#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-fwnode.h>

#include "isx031.h"
#ifdef CONFIG_VIDEO_INTEL_IPU6
#define GMSL_CSI_DT_YUV422_8 0x1E
#define V4L2_CID_IPU_BASE	(V4L2_CID_USER_BASE + 0x1080)

#define V4L2_CID_IPU_STORE_CSI2_HEADER	(V4L2_CID_IPU_BASE + 2)
#define V4L2_CID_IPU_ISYS_COMPRESSION	(V4L2_CID_IPU_BASE + 3)

#define V4L2_CID_IPU_QUERY_SUB_STREAM	(V4L2_CID_IPU_BASE + 4)
#define V4L2_CID_IPU_SET_SUB_STREAM	(V4L2_CID_IPU_BASE + 5)

#define V4L2_CID_IPU_ENUMERATE_LINK	(V4L2_CID_IPU_BASE + 6)

#define VIDIOC_IPU_GET_DRIVER_VERSION \
	_IOWR('v', BASE_VIDIOC_PRIVATE + 3, uint32_t)
#endif

#ifdef CONFIG_VIDEO_ISX031_SERDES
#include <media/ipu-acpi-pdata.h>
#include <media/max9295.h>
#include <media/max9296.h>
#endif

#define AGGREGATED_SUFFIX_OFFSET 6 // Is this necessary with only 1 pad?

#define MAX_YUV_EXP			10000
#define DEF_YUV_EXP			1660

enum isx031_mux_pad {
	ISX031_MUX_PAD_EXTERNAL,
	ISX031_MUX_PAD_YUV,
	ISX031_MUX_PAD_COUNT,
};

#define ISX031_N_CONTROLS			8

#define CSI2_MAX_VIRTUAL_CHANNELS	4

#ifdef CONFIG_VIDEO_INTEL_IPU6
#define ISX031_LINK_FREQ_1250MHZ		1250000000ULL
#define ISX031_LINK_FREQ_1125MHZ		1125000000ULL
#define ISX031_LINK_FREQ_1000MHZ		1000000000ULL
#define ISX031_LINK_FREQ_900MHZ		900000000ULL
#define ISX031_LINK_FREQ_840MHZ		840000000ULL
#define ISX031_LINK_FREQ_750MHZ		750000000ULL
#define ISX031_LINK_FREQ_720MHZ		720000000ULL
#define ISX031_LINK_FREQ_600MHZ		600000000ULL
#define ISX031_LINK_FREQ_576MHZ		576000000ULL
#define ISX031_LINK_FREQ_480MHZ		480000000ULL
#define ISX031_LINK_FREQ_450MHZ		450000000ULL
#define ISX031_LINK_FREQ_360MHZ		360000000ULL
#define ISX031_LINK_FREQ_300MHZ		300000000ULL
#define ISX031_LINK_FREQ_288MHZ		288000000ULL
#define ISX031_LINK_FREQ_240MHZ		240000000ULL
#define ISX031_LINK_FREQ_225MHZ		22500000ULL
#endif

#ifdef CONFIG_VIDEO_INTEL_IPU6
static const s64 link_freq_menu_items[] = {
	ISX031_LINK_FREQ_1250MHZ,
	ISX031_LINK_FREQ_1125MHZ,
	ISX031_LINK_FREQ_1000MHZ,
	ISX031_LINK_FREQ_900MHZ,
	ISX031_LINK_FREQ_840MHZ,
	ISX031_LINK_FREQ_750MHZ,
	ISX031_LINK_FREQ_720MHZ,
	ISX031_LINK_FREQ_600MHZ,
	ISX031_LINK_FREQ_576MHZ,
	ISX031_LINK_FREQ_480MHZ,
	ISX031_LINK_FREQ_450MHZ,
	ISX031_LINK_FREQ_360MHZ,
	ISX031_LINK_FREQ_300MHZ,
	ISX031_LINK_FREQ_288MHZ,
	ISX031_LINK_FREQ_240MHZ,
	ISX031_LINK_FREQ_225MHZ,
};
#endif

/*************************/

struct isx031_ctrls {
	struct v4l2_ctrl_handler handler;
	struct v4l2_ctrl_handler handler_yuv;
	struct {
		struct v4l2_ctrl *auto_exp;
		struct v4l2_ctrl *exposure;
		/* in ISX031 manual gain only works with manual exposure */
		struct v4l2_ctrl *gain;
		struct v4l2_ctrl *link_freq;
		struct v4l2_ctrl *query_sub_stream;
		struct v4l2_ctrl *set_sub_stream;
	};
};

struct isx031_resolution {
	u16 width;
	u16 height;
	u8 n_framerates;
	const u16 *framerates;
};

struct isx031_format {
	unsigned int n_resolutions;
	const struct isx031_resolution *resolutions;
	u32 mbus_code;
	u8 data_type;
};

struct isx031_sensor {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;
	u16 mux_pad;
	struct {
		const struct isx031_format *format;
		const struct isx031_resolution *resolution;
		u16 framerate;
	} config;
	bool streaming;
	/*struct isx031_vchan *vchan;*/
	const struct isx031_format *formats;
	unsigned int n_formats;
	int pipe_id;
};

struct isx031_mux_subdev {
	struct v4l2_subdev subdev;
};

struct isx031_variant {
	const struct isx031_format *formats;
	unsigned int n_formats;
};

enum {
	ISX031_ISX031U,
};

#ifdef CONFIG_VIDEO_INTEL_IPU6
#define NR_OF_ISX031_PADS 7
#define NR_OF_ISX031_STREAMS 4
struct v4l2_mbus_framefmt isx031_ffmts[NR_OF_ISX031_PADS];
#endif

#ifdef CONFIG_VIDEO_ISX031_SERDES
struct serdes_state {
	bool isolated;
	int bus_nr;
	int addr;
};
#endif

struct isx031_hwcfg {
	unsigned long link_freq_bitmap;
};

struct isx031 {
	struct { struct isx031_sensor sensor; } yuv;
	struct {
		struct isx031_mux_subdev sd;
		struct media_pad pads[ISX031_MUX_PAD_COUNT];
		struct isx031_sensor *last_set;
	} mux;
	struct isx031_ctrls ctrls;
	bool power;
	struct i2c_client *client;
	struct isx031_hwcfg *hwcfg;
	/*struct isx031_vchan virtual_channels[CSI2_MAX_VIRTUAL_CHANNELS];*/
	/* All below pointers are used for writing, cannot be const */
	struct mutex lock;
	struct regulator *vcc;
	const struct isx031_variant *variant;
	int is_yuv;
	int aggregated;
	u16 fw_version;
	u16 fw_build;
#ifdef CONFIG_VIDEO_ISX031_SERDES
	struct gmsl_link_ctx g_ctx;
	struct device *ser_dev;
	struct device *dser_dev;
	struct i2c_client *i2c_mux_client;
	struct i2c_client *ser_i2c;
	struct i2c_client *dser_i2c;
	struct serdes_state dser_st;
#endif
#ifdef CONFIG_VIDEO_INTEL_IPU6
#define NR_OF_CSI2_BE_SOC_STREAMS	16
#define NR_OF_ISX031_SUB_STREAMS	1 /*d+d.md,c+c.md,ir,imu*/
	int pad_to_vc[ISX031_MUX_PAD_COUNT];
	int pad_to_substream[NR_OF_CSI2_BE_SOC_STREAMS];
#endif
};

struct isx031_counters {
	unsigned int n_res;
	unsigned int n_fmt;
	unsigned int n_ctrl;
};

#ifdef CONFIG_VIDEO_INTEL_IPU6
static s64 isx031_query_sub_stream[NR_OF_CSI2_BE_SOC_STREAMS];
static u8 isx031_set_sub_stream[NR_OF_CSI2_BE_SOC_STREAMS];
static void set_sub_stream_fmt(int index, u32 code)
{
	isx031_query_sub_stream[index] &= 0xFFFFFFFFFFFF0000;
	isx031_query_sub_stream[index] |= code;
}

static void set_sub_stream_h(int index, u32 height)
{
	s64 val = height;

	val &= 0xFFFF;
	isx031_query_sub_stream[index] &= 0xFFFFFFFF0000FFFF;
	isx031_query_sub_stream[index] |= val << 16;
}

static void set_sub_stream_w(int index, u32 width)
{
	s64 val = width;

	val &= 0xFFFF;
	isx031_query_sub_stream[index] &= 0xFFFF0000FFFFFFFF;
	isx031_query_sub_stream[index] |= val << 32;
}

static void set_sub_stream_dt(int index, u32 dt)
{
	s64 val = dt;

	val &= 0xFF;
	isx031_query_sub_stream[index] &= 0xFF00FFFFFFFFFFFF;
	isx031_query_sub_stream[index] |= val << 48;
}

static void set_sub_stream_vc_id(int index, u32 vc_id)
{
	s64 val = vc_id;

	val &= 0xFF;
	isx031_query_sub_stream[index] &= 0x00FFFFFFFFFFFFFF;
	isx031_query_sub_stream[index] |= val << 56;
}

static int get_sub_stream_vc_id(int index)
{
	s64 val = 0;

	val = isx031_query_sub_stream[index] >> 56;
	val &= 0xFF;
	return (int)val;
}
#endif // CONFIG_VIDEO_INTEL_IPU6

/* Pad ops */

static const u16 isx031_default_framerate = 30;

static const u16 isx031_framerate_60 = 60;

static const u16 isx031_framerate_to_60[] = {30, 60};

static const struct isx031_resolution isx031_onsemi_yuv_sizes[] = {
	{
		.width = 1920,
		.height = 1536,
		.framerates = isx031_framerate_to_60,
		.n_framerates = ARRAY_SIZE(isx031_framerate_to_60),
	},
};

static const struct isx031_format isx031_onsemi_yuv_format = {
	.data_type = GMSL_CSI_DT_YUV422_8,	/* UYVY */
	.mbus_code = MEDIA_BUS_FMT_UYVY8_1X16,
	.n_resolutions = ARRAY_SIZE(isx031_onsemi_yuv_sizes),
	.resolutions = isx031_onsemi_yuv_sizes,
};
#define ISX031_ONSEMI_YUV_N_FORMATS 1

static const struct isx031_variant isx031_variants[] = {
	[ISX031_ISX031U] = {
		.formats = &isx031_onsemi_yuv_format,
		.n_formats = ISX031_ONSEMI_YUV_N_FORMATS,
	},
};

static const struct v4l2_mbus_framefmt isx031_mbus_framefmt_template = {
	.width = 0,
	.height = 0,
	.code = MEDIA_BUS_FMT_FIXED,
	.field = V4L2_FIELD_NONE,
	.colorspace = V4L2_COLORSPACE_DEFAULT,
	.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT,
	.quantization = V4L2_QUANTIZATION_DEFAULT,
	.xfer_func = V4L2_XFER_FUNC_DEFAULT,
};

#include "isx031_sensor.c"

/* Get readable sensor name */
static const char *isx031_get_sensor_name(struct isx031 *isx031)
{
	if (isx031->is_yuv)
		return "YUV";
	return "unknown";
}

static void isx031_set_state_last_set(struct isx031 *isx031)
{
	 dev_dbg(&isx031->client->dev, "%s(): %s\n",
		__func__, isx031_get_sensor_name(isx031));

	if (isx031->is_yuv)
		isx031->mux.last_set = &isx031->yuv.sensor;
	else
		isx031->mux.last_set = NULL;
}

/* This is needed for .get_fmt()
 * and if streaming is started without .set_fmt()
 */
static void isx031_sensor_format_init(struct isx031_sensor *sensor)
{
	const struct isx031_format *fmt;
	struct v4l2_mbus_framefmt *ffmt;
	unsigned int i;

	if (sensor->config.format)
		return;

	dev_dbg(sensor->sd.dev, "%s(): on pad %u\n", __func__, sensor->mux_pad);

	ffmt = &sensor->format;
	*ffmt = isx031_mbus_framefmt_template;
	/* Use the first format */
	fmt = sensor->formats;
	ffmt->code = fmt->mbus_code;
	/* and the first resolution */
	ffmt->width = fmt->resolutions->width;
	ffmt->height = fmt->resolutions->height;

	sensor->config.format = fmt;
	sensor->config.resolution = fmt->resolutions;
	/* Set default framerate to 30, or to 1st one if not supported */
	for (i = 0; i < fmt->resolutions->n_framerates; i++) {
		if (fmt->resolutions->framerates[i] == isx031_framerate_60 /* fps */) {
			sensor->config.framerate = isx031_framerate_60;
			return;
		}
	}
	sensor->config.framerate = fmt->resolutions->framerates[0];
}

/* No locking needed for enumeration methods */
static int isx031_sensor_enum_mbus_code(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 10)
				     struct v4l2_subdev_pad_config *cfg,
#else
				     struct v4l2_subdev_state *v4l2_state,
#endif
				     struct v4l2_subdev_mbus_code_enum *mce)
{
	struct isx031_sensor *sensor = container_of(sd, struct isx031_sensor, sd);

	dev_dbg(sensor->sd.dev, "%s(): sensor %s pad: %d index: %d\n",
		__func__, sensor->sd.name, mce->pad, mce->index);
	if (mce->pad)
		return -EINVAL;

	if (mce->index >= sensor->n_formats)
		return -EINVAL;

	mce->code = sensor->formats[mce->index].mbus_code;

	return 0;
}

static int isx031_sensor_enum_frame_size(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 10)
				     struct v4l2_subdev_pad_config *cfg,
#else
				     struct v4l2_subdev_state *v4l2_state,
#endif
				      struct v4l2_subdev_frame_size_enum *fse)
{
	struct isx031_sensor *sensor = container_of(sd, struct isx031_sensor, sd);
	struct isx031 *isx031 = v4l2_get_subdevdata(sd);
	const struct isx031_format *fmt;
	unsigned int i;

	dev_dbg(sensor->sd.dev, "%s(): sensor %s is %s\n",
		__func__, sensor->sd.name, isx031_get_sensor_name(isx031));

	for (i = 0, fmt = sensor->formats; i < sensor->n_formats; i++, fmt++)
		if (fse->code == fmt->mbus_code)
			break;

	if (i == sensor->n_formats)
		return -EINVAL;

	if (fse->index >= fmt->n_resolutions)
		return -EINVAL;

	fse->min_width = fse->max_width = fmt->resolutions[fse->index].width;
	fse->min_height = fse->max_height = fmt->resolutions[fse->index].height;

	return 0;
}

static int isx031_sensor_enum_frame_interval(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 10)
				     struct v4l2_subdev_pad_config *cfg,
#else
				     struct v4l2_subdev_state *v4l2_state,
#endif
					  struct v4l2_subdev_frame_interval_enum *fie)
{
	struct isx031_sensor *sensor = container_of(sd, struct isx031_sensor, sd);
	const struct isx031_format *fmt;
	const struct isx031_resolution *res;
	unsigned int i;

	for (i = 0, fmt = sensor->formats; i < sensor->n_formats; i++, fmt++)
		if (fie->code == fmt->mbus_code)
			break;

	if (i == sensor->n_formats)
		return -EINVAL;

	for (i = 0, res = fmt->resolutions; i < fmt->n_resolutions; i++, res++)
		if (res->width == fie->width && res->height == fie->height)
			break;

	if (i == fmt->n_resolutions)
		return -EINVAL;

	if (fie->index >= res->n_framerates)
		return -EINVAL;

	fie->interval.numerator = 1;
	fie->interval.denominator = res->framerates[fie->index];

	return 0;
}

static int isx031_sensor_get_fmt(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 10)
				     struct v4l2_subdev_pad_config *cfg,
#else
				     struct v4l2_subdev_state *v4l2_state,
#endif
			      struct v4l2_subdev_format *fmt)
{
	struct isx031_sensor *sensor = container_of(sd, struct isx031_sensor, sd);
	struct isx031 *isx031 = v4l2_get_subdevdata(sd);

	if (fmt->pad)
		return -EINVAL;

	mutex_lock(&isx031->lock);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 10)
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(6, 8, 0)
		fmt->format = *v4l2_subdev_get_try_format(sd, v4l2_state, fmt->pad);
#else
		fmt->format = *v4l2_subdev_state_get_format(v4l2_state, fmt->pad);
#endif
	else
		fmt->format = sensor->format;

	mutex_unlock(&isx031->lock);

	dev_dbg(sd->dev, "%s(): pad %x, code %x, res %ux%u\n",
			__func__, fmt->pad, fmt->format.code,
			fmt->format.width, fmt->format.height);

	return 0;
}

/* Called with lock held */
static const struct isx031_format *isx031_sensor_find_format(
		struct isx031_sensor *sensor,
		struct v4l2_mbus_framefmt *ffmt,
		const struct isx031_resolution **best)
{
	const struct isx031_resolution *res;
	const struct isx031_format *fmt;
	unsigned long best_delta = ~0;
	unsigned int i;

	for (i = 0, fmt = sensor->formats; i < sensor->n_formats; i++, fmt++) {
		if (fmt->mbus_code == ffmt->code)
			break;
	}
	dev_dbg(sensor->sd.dev, "%s(): mbus_code = %x, code = %x \n",
		__func__, fmt->mbus_code, ffmt->code);

	if (i == sensor->n_formats) {
		/* Not found, use default */
		dev_dbg(sensor->sd.dev, "%s:%d Not found, use default\n",
			__func__, __LINE__);
		fmt = sensor->formats;
	}
	for (i = 0, res = fmt->resolutions; i < fmt->n_resolutions; i++, res++) {
		unsigned long delta = abs(ffmt->width * ffmt->height -
				res->width * res->height);
		if (delta < best_delta) {
			best_delta = delta;
			*best = res;
		}
	}

	ffmt->code = fmt->mbus_code;
	ffmt->width = (*best)->width;
	ffmt->height = (*best)->height;

	ffmt->field = V4L2_FIELD_NONE;
	ffmt->colorspace = V4L2_COLORSPACE_REC709;
	ffmt->ycbcr_enc = V4L2_YCBCR_ENC_709;

	return fmt;
}

#define MIPI_CSI2_TYPE_NULL	0x10
#define MIPI_CSI2_TYPE_BLANKING		0x11
#define MIPI_CSI2_TYPE_EMBEDDED8	0x12
#define MIPI_CSI2_TYPE_YUV422_8		0x1e
#define MIPI_CSI2_TYPE_YUV422_10	0x1f
#define MIPI_CSI2_TYPE_RGB565	0x22
#define MIPI_CSI2_TYPE_RGB888	0x24
#define MIPI_CSI2_TYPE_RAW6	0x28
#define MIPI_CSI2_TYPE_RAW7	0x29
#define MIPI_CSI2_TYPE_RAW8	0x2a
#define MIPI_CSI2_TYPE_RAW10	0x2b
#define MIPI_CSI2_TYPE_RAW12	0x2c
#define MIPI_CSI2_TYPE_RAW14	0x2d
/* 1-8 */
#define MIPI_CSI2_TYPE_USER_DEF(i)	(0x30 + (i) - 1)
#ifdef CONFIG_VIDEO_INTEL_IPU6
static unsigned int mbus_code_to_mipi(u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_RGB565_1X16:
		return MIPI_CSI2_TYPE_RGB565;
	case MEDIA_BUS_FMT_RGB888_1X24:
		return MIPI_CSI2_TYPE_RGB888;
	case MEDIA_BUS_FMT_YUYV10_1X20:
		return MIPI_CSI2_TYPE_YUV422_10;
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_YUYV8_1X16:
	case MEDIA_BUS_FMT_VYUY8_1X16:
		return MIPI_CSI2_TYPE_YUV422_8;
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		return MIPI_CSI2_TYPE_RAW12;
	case MEDIA_BUS_FMT_Y10_1X10:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		return MIPI_CSI2_TYPE_RAW10;
	case MEDIA_BUS_FMT_Y8_1X8:
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
		return MIPI_CSI2_TYPE_RAW8;
	case MEDIA_BUS_FMT_SBGGR10_DPCM8_1X8:
	case MEDIA_BUS_FMT_SGBRG10_DPCM8_1X8:
	case MEDIA_BUS_FMT_SGRBG10_DPCM8_1X8:
	case MEDIA_BUS_FMT_SRGGB10_DPCM8_1X8:
		return MIPI_CSI2_TYPE_USER_DEF(1);
	default:
		WARN_ON(1);
		return -EINVAL;
	}
}
#endif

#ifdef CONFIG_VIDEO_INTEL_IPU6
static int isx031_s_state_pad(struct isx031 *isx031, int pad)
{
	int ret = 0;

	dev_dbg(&isx031->client->dev, "%s(): set isx031 for pad: %d\n", __func__, pad);

	switch (pad) {
	case ISX031_MUX_PAD_YUV:
		isx031->is_yuv = 1;
		break;
	default:
		dev_warn(&isx031->client->dev, "%s(): unknown pad: %d\n", __func__, pad);
		ret = -EINVAL;
		break;
	}
	isx031_set_state_last_set(isx031);
	return ret;
}

static int isx031_s_state(struct isx031 *isx031, int vc)
{
	int ret = 0;
	int i = 0;
	int pad = 0;
	for (i = 0; i < ARRAY_SIZE(isx031->pad_to_vc); i++) {
		if (isx031->pad_to_vc[i] == vc) {
			pad = i;
			break;
		}
	}

	dev_info(&isx031->client->dev, "%s(): set isx031 for vc: %d on pad: %d\n", __func__, vc, pad);

	ret = isx031_s_state_pad(isx031, pad);
	return ret;
}

#endif

static int __isx031_sensor_set_fmt(struct isx031 *isx031, struct isx031_sensor *sensor,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 10)
				     struct v4l2_subdev_pad_config *cfg,
#else
				     struct v4l2_subdev_state *v4l2_state,
#endif
				struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *mf;
#ifdef CONFIG_VIDEO_INTEL_IPU6
	int substream = -1;
#endif

	dev_dbg(sensor->sd.dev, "%s(): isx031 %p, "
		"sensor %p, fmt %p, fmt->format %p\n",
		__func__, isx031, sensor, fmt,  &fmt->format);

	mf = &fmt->format;

	if (fmt->pad)
		return -EINVAL;

	mutex_lock(&isx031->lock);

	sensor->config.format = isx031_sensor_find_format(sensor, mf,
						&sensor->config.resolution);

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 10)
	if (cfg && fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		*v4l2_subdev_get_try_format(&sensor->sd, cfg, fmt->pad) = *mf;
#elif LINUX_VERSION_CODE < KERNEL_VERSION(6, 8, 0)
	if (v4l2_state && fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		*v4l2_subdev_get_try_format(&sensor->sd, v4l2_state, fmt->pad) = *mf;
#else
	if (v4l2_state && fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		*v4l2_subdev_state_get_format(v4l2_state, fmt->pad) = *mf;
#endif

	else
// FIXME: use this format in .s_stream()
		sensor->format = *mf;

	isx031->mux.last_set = sensor;

	mutex_unlock(&isx031->lock);
#ifdef CONFIG_VIDEO_INTEL_IPU6
	substream = isx031->pad_to_substream[sensor->mux_pad];

	if (substream != -1) {
		set_sub_stream_fmt(substream, mf->code);
		set_sub_stream_h(substream, mf->height);
		set_sub_stream_w(substream, mf->width);
		set_sub_stream_dt(substream, mbus_code_to_mipi(mf->code));
	}

	dev_dbg(sensor->sd.dev, "%s(): fmt->pad: %d, sensor->mux_pad: %d, code: 0x%x, %ux%u substream:%d\n", __func__,
		fmt->pad, sensor->mux_pad, fmt->format.code,
		fmt->format.width, fmt->format.height, substream);
#endif
	return 0;
}

static int isx031_sensor_set_fmt(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 10)
				     struct v4l2_subdev_pad_config *cfg,
#else
				     struct v4l2_subdev_state *v4l2_state,
#endif
			      struct v4l2_subdev_format *fmt)
{
	struct isx031_sensor *sensor = container_of(sd, struct isx031_sensor, sd);
	struct isx031 *isx031 = v4l2_get_subdevdata(sd);
#ifdef CONFIG_VIDEO_INTEL_IPU6
	/* set isx031 by vc */
	isx031_s_state_pad(isx031, sensor->mux_pad);
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 10)
	return __isx031_sensor_set_fmt(isx031, sensor, cfg, fmt);
#else
	return __isx031_sensor_set_fmt(isx031, sensor, v4l2_state, fmt);
#endif
}

#ifdef CONFIG_VIDEO_ISX031_SERDES
static int isx031_setup_pipeline(struct isx031 *isx031, u8 data_type1, u8 data_type2,
			      int pipe_id, u32 vc_id)
{
	struct device *dev = &isx031->client->dev;
	int ret = 0;


	dev_dbg(dev,
			 "set pipe %d, data_type1: 0x%x, "
			 "data_type2: 0x%x, vc_id: %u\n",
			 pipe_id, data_type1, data_type2, vc_id);
	ret |= max9295_set_pipe(isx031->ser_dev, pipe_id, data_type1, data_type2, vc_id);
	ret |= max9296_set_pipe(isx031->dser_dev, pipe_id, data_type1, data_type2, vc_id);
	if (ret)
		dev_warn(dev,
			 "failed to set pipe %d, data_type1: 0x%x, "
			 "data_type2: 0x%x, vc_id: %u\n",
			 pipe_id, data_type1, data_type2, vc_id);

	return ret;
}
#endif

static int isx031_configure(struct isx031 *isx031)
{
	struct isx031_sensor *sensor;
	u16 vc_id;
#ifdef CONFIG_VIDEO_ISX031_SERDES
	u16 data_type1, data_type2;
#endif
	int ret;

	if (isx031->is_yuv) {
		sensor = &isx031->yuv.sensor;
		vc_id = 1;
	} else {
		return -EINVAL;
	}

#ifdef CONFIG_VIDEO_ISX031_SERDES
	data_type1 = sensor->config.format->data_type;
	data_type2 = 0; // GMSL_CSI_DT_EMBED;

	vc_id = isx031->g_ctx.dst_vc;

	ret = isx031_setup_pipeline(isx031, data_type1, data_type2, sensor->pipe_id,
				 vc_id);
	if (ret < 0)
		return ret;

	max9296_reset_oneshot(isx031->dser_dev);
#endif

	ret = isx031_configure_mode(isx031);
	if (ret)
		return ret;

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
/* pad ops */
static int isx031_sensor_g_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_state *sd_state,
		struct v4l2_subdev_frame_interval *fi)
#else
/* Video ops */
static int isx031_sensor_g_frame_interval(struct v4l2_subdev *sd,
		struct v4l2_subdev_frame_interval *fi)
#endif
{
	struct isx031_sensor *sensor = container_of(sd, struct isx031_sensor, sd);

	if (NULL == sd || NULL == fi)
		return -EINVAL;

	fi->interval.numerator = 1;
	fi->interval.denominator = sensor->config.framerate;

	dev_dbg(sd->dev, "%s(): %s %u\n", __func__, sd->name,
			fi->interval.denominator);

	return 0;
}
static u16 __isx031_probe_framerate(const struct isx031_resolution *res, u16 target);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
/* pad ops */
static int isx031_sensor_s_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_state *sd_state,
		struct v4l2_subdev_frame_interval *fi)
#else
/* Video ops */
static int isx031_sensor_s_frame_interval(struct v4l2_subdev *sd,
		struct v4l2_subdev_frame_interval *fi)
#endif
{
	struct isx031_sensor *sensor = container_of(sd, struct isx031_sensor, sd);
	u16 framerate = 1;

	if (NULL == sd || NULL == fi || fi->interval.numerator == 0)
		return -EINVAL;

	framerate = fi->interval.denominator / fi->interval.numerator;
	framerate = __isx031_probe_framerate(sensor->config.resolution, framerate);
	sensor->config.framerate = framerate;
	fi->interval.numerator = 1;
	fi->interval.denominator = framerate;

	dev_dbg(sd->dev, "%s(): %s %u\n", __func__, sd->name, framerate);

	return 0;
}

static int isx031_sensor_s_stream(struct v4l2_subdev *sd, int on)
{
	struct isx031_sensor *sensor = container_of(sd, struct isx031_sensor, sd);

	dev_dbg(sensor->sd.dev, "%s(): sensor: name=%s isx031=%d\n",
		__func__, sensor->sd.name, on);

	sensor->streaming = on;

	return 0;
}

static const struct v4l2_subdev_video_ops isx031_sensor_video_ops = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 8, 0)
	.g_frame_interval	= isx031_sensor_g_frame_interval,
	.s_frame_interval	= isx031_sensor_s_frame_interval,
#endif
	.s_stream		= isx031_sensor_s_stream,
};

static const struct v4l2_subdev_pad_ops isx031_yuv_pad_ops = {
	.enum_mbus_code		= isx031_sensor_enum_mbus_code,
	.enum_frame_size	= isx031_sensor_enum_frame_size,
	.enum_frame_interval	= isx031_sensor_enum_frame_interval,
	.get_fmt		= isx031_sensor_get_fmt,
	.set_fmt		= isx031_sensor_set_fmt,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
	.get_frame_interval	= isx031_sensor_g_frame_interval,
	.set_frame_interval	= isx031_sensor_s_frame_interval,
#endif
};

static const struct v4l2_subdev_ops isx031_yuv_subdev_ops = {
	.pad = &isx031_yuv_pad_ops,
	.video = &isx031_sensor_video_ops,
};

static int isx031_mux_s_stream(struct v4l2_subdev *sd, int on);

static int isx031_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct isx031 *isx031 = container_of(ctrl->handler, struct isx031,
					 ctrls.handler);
	struct v4l2_subdev *sd = &isx031->mux.sd.subdev;
	struct isx031_sensor *sensor = (struct isx031_sensor *)ctrl->priv;
	int ret = -EINVAL;

	if (sensor) {
		switch (sensor->mux_pad) {
		case ISX031_MUX_PAD_YUV:
		default:
			isx031 = container_of(ctrl->handler, struct isx031, ctrls.handler_yuv);
			isx031->is_yuv = 1;
		break;
		}
	}

	v4l2_dbg(3, 1, sd, "ctrl: %s, value: %d\n", ctrl->name, ctrl->val);
	dev_dbg(&isx031->client->dev, "%s(): %s - ctrl: %s, value: %d\n",
		__func__, isx031_get_sensor_name(isx031), ctrl->name, ctrl->val);

	mutex_lock(&isx031->lock);

	switch (ctrl->id) {
#ifdef CONFIG_VIDEO_INTEL_IPU6
	case V4L2_CID_IPU_SET_SUB_STREAM:
	{
		u32 val = (*ctrl->p_new.p_s64 & 0xFFFF);
		u16 on = val & 0x00FF;
		u16 vc_id = (val >> 8) & 0x00FF;
		int substream = -1;
		if (vc_id < ISX031_MUX_PAD_COUNT)
			ret = isx031_s_state(isx031, vc_id);
		substream = isx031->pad_to_substream[isx031->mux.last_set->mux_pad];
		dev_info(&isx031->client->dev, "V4L2_CID_IPU_SET_SUB_STREAM %x vc_id:%d, substream:%d, on:%d\n", val, vc_id, substream, on);
		if (on == 0xff)
			break;
		if (vc_id > NR_OF_ISX031_STREAMS - 1)
			dev_err(&isx031->client->dev, "invalid vc %d\n", vc_id);
		else
			isx031_set_sub_stream[substream] = on;
		ret = 0;
#ifdef CONFIG_VIDEO_ISX031_SERDES
		ret = isx031_mux_s_stream(sd, on);
#endif
	}
		break;
	case V4L2_CID_LINK_FREQ: {
		if ( !sensor && ctrl->p_new.p_u8)
		{
		  /* MTL and RPL/ADL IPU6 CSI-DPHY do NOT share
		   *  the same default link_freq.
		   * V4L2_CID_LINK_FREQ ISX031 mux must be R/W for udev ot set DPHY platform specific link_freq
		   * via systemd-udevd rules.
		   */
		  if (*ctrl->p_new.p_u8 <= (ARRAY_SIZE(link_freq_menu_items) - 1)) {
			struct v4l2_ctrl *link_freq = isx031->ctrls.link_freq;
			dev_info(&isx031->client->dev,
				"user-modified %s index val=%d to user-val=%d",
				 ctrl->name,
				 (unsigned int) link_freq->val,
				 (unsigned int) *ctrl->p_new.p_u8);
			link_freq->val = (s32) *ctrl->p_new.p_u8;
			ret = 0;
		  }
		}

	}
	  break;
#endif
	}

	mutex_unlock(&isx031->lock);

	return ret;
}

static int isx031_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct isx031 *isx031 = container_of(ctrl->handler, struct isx031,
			ctrls.handler);
	u32 data;
	int ret = 0;
	struct isx031_sensor *sensor = (struct isx031_sensor *)ctrl->priv;
	u16 reg;

	if (sensor) {
		switch (sensor->mux_pad) {
		case ISX031_MUX_PAD_YUV:
		default:
			isx031 = container_of(ctrl->handler, struct isx031, ctrls.handler_yuv);
			isx031->is_yuv = 1;
		break;
		}
	}

	dev_dbg(&isx031->client->dev, "%s(): %s - ctrl: %s \n",
		__func__, isx031_get_sensor_name(isx031), ctrl->name);

	switch (ctrl->id) {

#ifdef CONFIG_VIDEO_INTEL_IPU6
	case V4L2_CID_IPU_QUERY_SUB_STREAM: {
		if (sensor) {
			int substream = isx031->pad_to_substream[sensor->mux_pad];
			int vc_id = get_sub_stream_vc_id(substream);

			dev_dbg(sensor->sd.dev,
				"%s(): V4L2_CID_IPU_QUERY_SUB_STREAM sensor->mux_pad:%d"
				", vc:[%d] %d\n",
				__func__, sensor->mux_pad, vc_id, substream);
			*ctrl->p_new.p_s32 = substream;
			isx031->mux.last_set = sensor;
		} else {
				/* we are in ISX031 MUX case */
				*ctrl->p_new.p_s32 = -1;
		}

	}
		break;
#endif
	}
	return ret;
}

static const struct v4l2_ctrl_ops isx031_ctrl_ops = {
	.s_ctrl	= isx031_s_ctrl,
	.g_volatile_ctrl = isx031_g_volatile_ctrl,
};

#ifdef CONFIG_VIDEO_INTEL_IPU6
static const struct v4l2_ctrl_config isx031_controls_link_freq = {
	.ops = &isx031_ctrl_ops,
	.id = V4L2_CID_LINK_FREQ,
	.name = "V4L2_CID_LINK_FREQ",
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.max = ARRAY_SIZE(link_freq_menu_items) - 1,
	.min =  0,
	.step  = 0,
	.def = 12,    // default ISX031_LINK_FREQ_300MHZ
	.qmenu_int = link_freq_menu_items,
};

static struct v4l2_ctrl_config isx031_controls_q_sub_stream = {
	.ops = &isx031_ctrl_ops,
	.id = V4L2_CID_IPU_QUERY_SUB_STREAM,
	.name = "query virtual channel",
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.max = NR_OF_ISX031_SUB_STREAMS - 1,
	.min = 0,
	.def = 0,
	.menu_skip_mask = 0,
	.qmenu_int = isx031_query_sub_stream,
};

static const struct v4l2_ctrl_config isx031_controls_s_sub_stream = {
	.ops = &isx031_ctrl_ops,
	.id = V4L2_CID_IPU_SET_SUB_STREAM,
	.name = "set virtual channel",
	.type = V4L2_CTRL_TYPE_INTEGER64,
	.max = 0xFFFF,
	.min = 0,
	.def = 0,
	.step = 1,
};
#endif

static int isx031_mux_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct isx031 *state = v4l2_get_subdevdata(sd);

	dev_dbg(sd->dev, "%s(): %s (%p)\n", __func__, sd->name, fh);

	return 0;
};

static int isx031_mux_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct isx031 *state = v4l2_get_subdevdata(sd);

	dev_dbg(sd->dev, "%s(): %s (%p)\n", __func__, sd->name, fh);
	return 0;
};

static const struct v4l2_subdev_internal_ops isx031_sensor_internal_ops = {
	.open = isx031_mux_open,
	.close = isx031_mux_close,
};

#ifdef CONFIG_VIDEO_INTEL_IPU6
static short sensor_vc[NR_OF_ISX031_STREAMS * 2] = {0,1,2,3,2,3,0,1};
module_param_array(sensor_vc, ushort, NULL, 0444);
MODULE_PARM_DESC(sensor_vc, "VC set for sensors\n"
		"\t\tsensor_vc=0,1,2,3,2,3,0,1");

//#define PLATFORM_AXIOMTEK 1
#ifdef PLATFORM_AXIOMTEK
static short serdes_bus[4] = {5, 5, 5, 5};
#else
static short serdes_bus[4] = {2, 2, 4, 4};
#endif
module_param_array(serdes_bus, ushort, NULL, 0444);
MODULE_PARM_DESC(serdes_bus, "max9295/6 deserializer i2c bus\n"
		"\t\tserdes_bus=2,2,4,4");

// Deserializer addresses can be 0x40 0x48 0x4a
#ifdef PLATFORM_AXIOMTEK
static unsigned short des_addr[4] = {0x48, 0x4a, 0x68, 0x6c};
#else
static unsigned short des_addr[4] = {0x48, 0x4a, 0x48, 0x4a};
#endif
module_param_array(des_addr, ushort, NULL, 0444);
MODULE_PARM_DESC(des_addr, "max9296 deserializer i2c address\n"
		"\t\tdes_addr=0x48,0x4a,0x48,0x4a");

static int isx031_i2c_addr_setting(struct i2c_client *c, struct isx031 *state)
{
	int i = 0;
	int c_addr_save = c->addr;
	int c_bus = c->adapter->nr;
	for (i = 0; i < 4; i++) {
		if (c_bus == serdes_bus[i]) {
			c->addr = des_addr[i];
			dev_info(&c->dev, "Set max9296@%d-0x%x Link reset\n",
					c_bus, c->addr);
			// Hold link in reset while setting things up
			//max9296_reset_link(&isx031->dser_i2c->dev);
		}
	}
	// restore original slave address
	c->addr = c_addr_save;

	return 0;
}
#endif

#ifdef CONFIG_VIDEO_ISX031_SERDES

/*
 * FIXME
 * temporary solution before changing GMSL data structure or merging all 4 D457
 * sensors into one i2c device. Only first sensor node per max9295 sets up the
 * link.
 *
 * max 24 number from this link:
 * https://docs.nvidia.com/jetson/archives/r35.1/DeveloperGuide/text/
 * SD/CameraDevelopment/JetsonVirtualChannelWithGmslCameraFramework.html
 * #jetson-agx-xavier-series
 */
#define MAX_DEV_NUM 24
static struct isx031 *serdes_inited[MAX_DEV_NUM];
# ifndef CONFIG_OF

static int isx031_serdes_board_setup(struct isx031 *isx031)
{
	struct device *dev = &isx031->client->dev;
	struct serdes_platform_data *pdata = dev->platform_data;
	struct i2c_adapter *adapter = isx031->client->adapter;
	bool dser_new = false;
	int bus = adapter->nr;
	int err = 0;
	int i;

	// TCA9546
	isx031->i2c_mux_client = i2c_new_dummy_device(adapter, 0x70);
	if (IS_ERR_OR_NULL(isx031->i2c_mux_client)) {
		err = PTR_ERR(isx031->i2c_mux_client);
		dev_err(dev, "Failed to create client for i2c mux: %d", err);
		return err;
	}
	// Enable bus 2
	err = i2c_smbus_write_byte(isx031->i2c_mux_client, (1<<1));
	if (err) {
		dev_warn(dev, "Failed to set i2c mux: %d", err);
		i2c_unregister_device(isx031->i2c_mux_client);
		isx031->i2c_mux_client = NULL;
	}
	char suffix = pdata->suffix;
	static struct max9295_pdata max9295_pdata = {
		.is_prim_ser = 1, // todo: configurable
		.def_addr = 0x40, // todo: configurable
		.d4xx_hacks = 0,
	};
	static struct max9296_pdata max9296_pdata = {
		.max_src = 2,
		.csi_mode = GMSL_CSI_4X2_MODE,
		.d4xx_hacks = 0,
	};
	static struct i2c_board_info i2c_info_des = {
		I2C_BOARD_INFO("max9296", 0x48),
		.platform_data = &max9296_pdata,
	};
	static struct i2c_board_info i2c_info_ser = {
		I2C_BOARD_INFO("max9295", 0x42),
		.platform_data = &max9295_pdata,
	};

	i2c_info_ser.addr = pdata->subdev_info[0].ser_alias; //0x42, 0x44, 0x62, 0x64
	isx031->ser_i2c = i2c_new_client_device(adapter, &i2c_info_ser);

	i2c_info_des.addr = pdata->subdev_info[0].board_info.addr; //0x48, 0x4a, 0x68, 0x6a

	isx031->dser_st.bus_nr = bus;
	isx031->dser_st.addr = i2c_info_des.addr;

	/* look for already registered max9296, use same context if found */
	for (i = 0; i < MAX_DEV_NUM; i++) {
		if (serdes_inited[i]) {
			if ( serdes_inited[i]->dser_st.isolated
				    && bus == serdes_inited[i]->dser_st.bus_nr
				    && serdes_inited[i]->dser_st.addr == i2c_info_des.addr ) {

				dev_info(dev, "Isolate unresponsive sensor/serializer AGGREGATED on MAX9296 device 0x%x\n",
					 i2c_info_des.addr);
				isx031->aggregated = 1;
				break;
			} else if ( serdes_inited[i]->dser_i2c
				    && bus == serdes_inited[i]->dser_i2c->adapter->nr
				    && serdes_inited[i]->dser_i2c->addr == i2c_info_des.addr) {
				dev_info(dev, "MAX9296 AGGREGATION found device on 0x%x\n", i2c_info_des.addr);
				isx031->dser_i2c = serdes_inited[i]->dser_i2c;
				isx031->aggregated = 1;
				break;
			}
		}
	}
	if (isx031->aggregated)
		suffix += AGGREGATED_SUFFIX_OFFSET;
	dev_info(dev, "Init SerDes %c on %d@0x%x<->%d@0x%x\n",
		suffix,
		bus, pdata->subdev_info[0].board_info.addr, //48
		bus, pdata->subdev_info[0].ser_alias); //42

	if (!isx031->dser_i2c) {
		isx031->dser_i2c = i2c_new_client_device(adapter, &i2c_info_des);
		dser_new = true;
	}

	err = 0;
	if (IS_ERR_OR_NULL(isx031->ser_i2c)) {
		dev_err(dev, "missing serializer client (%d)\n", PTR_ERR(isx031->ser_i2c));
		err = -EPROBE_DEFER;
		goto error;
	}
	if (isx031->ser_i2c->dev.driver == NULL) {
		err = -EPROBE_DEFER;
		dev_err(dev, "missing serializer driver\n");
		goto error;
	}
	if (IS_ERR_OR_NULL(isx031->dser_i2c)) {
		dev_err(dev, "missing deserializer client (%d)\n", PTR_ERR(isx031->dser_i2c));
		err = -EPROBE_DEFER;
		goto error;
	}
	if (isx031->dser_i2c->dev.driver == NULL) {
		err = -EPROBE_DEFER;
		dev_err(dev, "missing deserializer driver\n");
		goto error;
	}

	isx031->g_ctx.sdev_reg = isx031->client->addr;
	isx031->g_ctx.sdev_def = ISX031_I2C_ADDRESS;// def-addr TODO: configurable
	// Address reassignment for isx031-a 0x10->0x12
	dev_info(dev, "Address reassignment for %s-%c 0x%x->0x%x\n",
		pdata->subdev_info[0].board_info.type, suffix,
		isx031->g_ctx.sdev_def, isx031->g_ctx.sdev_reg);
	//0x42, 0x44, 0x62, 0x64
	isx031->g_ctx.ser_reg = pdata->subdev_info[0].ser_alias;
	dev_info(dev,  "serializer: i2c-%d@0x%x\n",
		isx031->ser_i2c->adapter->nr, isx031->g_ctx.ser_reg);

	if (err < 0) {
		dev_err(dev, "serializer reg not found\n");
		goto error;
	}

	isx031->ser_dev = &isx031->ser_i2c->dev;

	dev_info(dev,  "deserializer: i2c-%d@0x%x\n",
		isx031->dser_i2c->adapter->nr, isx031->dser_i2c->addr);

	isx031->dser_dev = &isx031->dser_i2c->dev;

	/* populate g_ctx from pdata */
	isx031->g_ctx.dst_csi_port = GMSL_CSI_PORT_A;
	isx031->g_ctx.src_csi_port = GMSL_CSI_PORT_B;
	isx031->g_ctx.csi_mode = GMSL_CSI_1X4_MODE;
	if (isx031->aggregated) { // aggregation
		dev_info(dev,  "configure GMSL port B\n");
		isx031->g_ctx.serdes_csi_link = GMSL_SERDES_CSI_LINK_B;
	} else {
		dev_info(dev,  "configure GMSL port A\n");
		isx031->g_ctx.serdes_csi_link = GMSL_SERDES_CSI_LINK_A;
	}
	isx031->g_ctx.st_vc = 0;
	isx031->g_ctx.dst_vc = 0;

	isx031->g_ctx.num_csi_lanes = 2;
	isx031->g_ctx.s_dev = dev;

	for (i = 0; i < MAX_DEV_NUM; i++) {
		if (!serdes_inited[i]) {
			serdes_inited[i] = isx031;
			return 0;
		} else if (serdes_inited[i]->ser_dev == isx031->ser_dev) {
			return -ENOTSUPP;
		}
	}
	err = -EINVAL;
	dev_err(dev, "cannot handle more than %d ISX031 cameras\n", MAX_DEV_NUM);

error:
	return err;
}

# endif // CONFIG_OF
static DEFINE_MUTEX(serdes_lock__);

static int isx031_serdes_link_setup(struct isx031 *isx031)
{
	int err = 0;
	int des_err = 0;
	int ser_err = 0;
	struct device *dev;

	if (!isx031 || !isx031->ser_dev || !isx031->dser_dev || !isx031->client)
		return -EINVAL;

	dev = &isx031->client->dev;

	mutex_lock(&serdes_lock__);

	max9296_power_off(isx031->dser_dev);
	/* For now no separate power on required for serializer device */
	max9296_power_on(isx031->dser_dev);

	dev_dbg(dev, "Setup SERDES addressing and control pipeline\n");
	/* setup serdes addressing and control pipeline */
	err = max9296_setup_link(isx031->dser_dev, &isx031->client->dev);
	if (err) {
		dev_err(dev, "gmsl deserializer link config failed\n");
		goto error;
	}
	msleep(100);

	ser_err = max9295_setup_control(isx031->ser_dev);
	/* proceed even if ser setup failed, to setup deser correctly */
	/* if ser setup failed, graceful deser setup fallback */
	if (ser_err) {
		dev_warn(dev, "gmsl serializer invalid source\n");
		err= -ENOTSUPP;
	}

	des_err = max9296_setup_control(isx031->dser_dev, &isx031->client->dev);
	if (des_err) {
		dev_err(dev, "gmsl deserializer setup failed\n");
		/* overwrite err only if just deser setup has failed */
		err = ( err == -ENOTSUPP) ?  err : des_err;
	}

error:
	mutex_unlock(&serdes_lock__);
	return err;
}


static int isx031_serdes_setup(struct isx031 *isx031)
{
	int ret = 0;
	struct i2c_client *c = isx031->client;
#ifdef CONFIG_VIDEO_INTEL_IPU6
	int i = 0, c_bus = -1;
	int c_bus_new = c->adapter->nr;

	for (i = 0; i < MAX_DEV_NUM; i++) {
#ifdef CONFIG_VIDEO_ISX031_SERDES
		if (serdes_inited[i] && serdes_inited[i]->dser_st.isolated) {
			c_bus = serdes_inited[i]->dser_st.bus_nr;
			if (c_bus == c->adapter->nr) {
				dev_info(&c->dev, "Already configured Isolated camera for bus %d\n", c_bus);
				c_bus_new = -1;
				break;
			}
		} else if (serdes_inited[i] && serdes_inited[i]->dser_i2c) {
#else
		if (serdes_inited[i] && serdes_inited[i]->dser_i2c) {
#endif
			c_bus = serdes_inited[i]->dser_i2c->adapter->nr;
			if (c_bus == c->adapter->nr) {
				dev_info(&c->dev, "Already configured multiple camera for bus %d\n", c_bus);
				c_bus_new = -1;
				break;
			}
		} else {
			break;
		}
	}

	if (c_bus_new >= 0) {
		dev_info(&c->dev, "Apply multiple camera i2c addr setting for bus %d\n", c_bus_new);
		ret = isx031_i2c_addr_setting(c, isx031);
		if (ret) {
			dev_err(&c->dev, "failed apply i2c addr setting\n");
			return ret;
		}
	}
#endif
	ret = isx031_serdes_board_setup(isx031);
	if (ret) {
		if (ret == -ENOTSUPP)
			return 0;
		dev_err(&c->dev, "board setup failed\n");
		return ret;
	}

	/* Pair sensor to serializer dev */
	ret = max9295_sdev_pair(isx031->ser_dev, &isx031->g_ctx);
	if (ret) {
		dev_err(&c->dev, "gmsl ser pairing failed\n");
		return ret;
	}

	/* Register sensor to deserializer dev */
	ret = max9296_sdev_register(isx031->dser_dev, &isx031->g_ctx);
	if (ret) {
		dev_err(&c->dev, "gmsl deserializer register failed\n");
		return ret;
	}

	ret = isx031_serdes_link_setup(isx031);
	if (ret) {
		if (ret == -ENOTSUPP) {
			dev_warn(&c->dev, "gmsl serdes setup gracefully fallback\n");
			if (c_bus_new >= 0 )
				dev_info(&c->dev, "Unresponding serializer on Newly initialized bus %d\n",
					 c_bus_new);
			else
				dev_info(&c->dev, "Unresponding serializer on Already initialized bus %d\n",
					 isx031->dser_i2c->adapter->nr);
		} else
			dev_err(&c->dev, "%s gmsl serdes setup failed\n", __func__);
		return ret;
	}

	ret = max9295_init_settings(isx031->ser_dev);
	if (ret) {
		dev_warn(&c->dev, "%s, failed to init max9295 settings\n",
			__func__);
		return ret;
	}

	ret = max9296_init_settings(isx031->dser_dev);
	if (ret) {
		dev_warn(&c->dev, "%s, failed to init max9296 settings\n",
			__func__);
		return ret;
	}

	return ret;
}
#endif // CONFIG_VIDEO_ISX031_SERDES

enum state_sid {
	YUV_SID,
	MUX_SID = -1
};

static int isx031_ctrl_init(struct isx031 *isx031, int sid)
{
	const struct v4l2_ctrl_ops *ops = &isx031_ctrl_ops;
	struct isx031_ctrls *ctrls = &isx031->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;
	struct v4l2_subdev *sd = &isx031->mux.sd.subdev;
	int ret = -1;
	struct isx031_sensor *sensor = NULL;

	switch (sid) {
	case YUV_SID:
		hdl = &ctrls->handler_yuv;
		sensor = &isx031->yuv.sensor;
		break;
	default:
		/* control for MUX */
		hdl = &ctrls->handler;
		sensor = NULL;
		break;
	}

	dev_dbg(NULL, "%s():%d sid: %d\n", __func__, __LINE__, sid);
	ret = v4l2_ctrl_handler_init(hdl, ISX031_N_CONTROLS);
	if (ret < 0) {
		v4l2_err(sd, "cannot init ctrl handler (%d)\n", ret);
		return ret;
	}

#ifdef CONFIG_VIDEO_INTEL_IPU6
	ctrls->link_freq = v4l2_ctrl_new_custom(hdl, &isx031_controls_link_freq, sensor);
	/* MTL and RPL/ADL IPU6 CSI-DPHY do NOT share
	 *  the same default link_freq.
	 * V4L2_CID_LINK_FREQ ISX031 mux must be R/W for udev to set DPHY platform specific link_freq
	 * via systemd-udevd rules.
	*/
	if (sensor && ctrls->link_freq )
		ctrls->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	if (isx031->aggregated) {
		isx031_controls_q_sub_stream.def = NR_OF_ISX031_SUB_STREAMS;
		isx031_controls_q_sub_stream.min = NR_OF_ISX031_SUB_STREAMS;
		isx031_controls_q_sub_stream.max = NR_OF_ISX031_SUB_STREAMS * 2 - 1;
	} else {
		isx031_controls_q_sub_stream.def = 0;
		isx031_controls_q_sub_stream.min = 0;
		isx031_controls_q_sub_stream.max = NR_OF_ISX031_SUB_STREAMS - 1;
	}
	ctrls->query_sub_stream = v4l2_ctrl_new_custom(hdl, &isx031_controls_q_sub_stream, sensor);

	if (ctrls->query_sub_stream)
		ctrls->query_sub_stream->flags |=
		V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;

	ctrls->set_sub_stream = v4l2_ctrl_new_custom(hdl, &isx031_controls_s_sub_stream, sensor);
#endif
	if (hdl->error) {
		v4l2_err(sd, "error creating controls (%d)\n", hdl->error);
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	switch (sid) {
	case YUV_SID:
		isx031->yuv.sensor.sd.ctrl_handler = hdl;
		dev_info(isx031->yuv.sensor.sd.dev,
			"%s():%d set ctrl_handler pad:%d, %p, %s\n",
			 __func__, __LINE__,
			 isx031->yuv.sensor.mux_pad,
			 isx031->yuv.sensor.sd.ctrl_handler,
			 isx031->yuv.sensor.sd.name);
		break;
	default:
		isx031->mux.sd.subdev.ctrl_handler = hdl;
		dev_info(isx031->mux.sd.subdev.dev,
			"%s():%d set ctrl_handler for MUX: %p, %s, substream id %lld \n",
			 __func__, __LINE__,
			 isx031->mux.sd.subdev.ctrl_handler,
			 isx031->mux.sd.subdev.name,
			 isx031_controls_q_sub_stream.def);
		break;
	}

	return 0;
}

static int isx031_sensor_init(struct i2c_client *c, struct isx031 *isx031,
		struct isx031_sensor *sensor, const struct v4l2_subdev_ops *ops,
		const char *name)
{
	struct v4l2_subdev *sd = &sensor->sd;
	struct media_entity *entity = &sensor->sd.entity;
	struct media_pad *pad = &sensor->pad;
	dev_t *dev_num = &isx031->client->dev.devt;
#ifdef CONFIG_VIDEO_ISX031_SERDES
	struct serdes_platform_data *dpdata = c->dev.platform_data;
	char suffix = dpdata->suffix;
#endif
	v4l2_i2c_subdev_init(sd, c, ops);
	// Set owner to NULL so we can unload the driver module
	sd->owner = NULL;
	sd->internal_ops = &isx031_sensor_internal_ops;
	sd->grp_id = *dev_num;
	v4l2_set_subdevdata(sd, isx031);
#ifdef CONFIG_VIDEO_ISX031_SERDES
	/*
	 * TODO: suffix for 2 sensor connected to 1 Deser
	 */
	if (isx031->aggregated & 1)
		suffix += AGGREGATED_SUFFIX_OFFSET;
	snprintf(sd->name, sizeof(sd->name), "ISX031 %s %c", name, suffix);
#else
	snprintf(sd->name, sizeof(sd->name), "ISX031 %s %d-%04x",
		 name, i2c_adapter_id(c->adapter), c->addr);
#endif

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	pad->flags = MEDIA_PAD_FL_SOURCE;
	entity->obj_type = MEDIA_ENTITY_TYPE_V4L2_SUBDEV;
	entity->function = MEDIA_ENT_F_CAM_SENSOR;

	dev_info(&c->dev,
		 "%s():%d init media_entity %s, type:%x, func:%x\n",
		 __func__, __LINE__,
		 sd->name,
		 entity->obj_type,
		 entity->function);

	return media_entity_pads_init(entity, 1, pad);
}

static int isx031_sensor_register(struct isx031 *isx031, struct isx031_sensor *sensor)
{
	struct v4l2_subdev *sd = &sensor->sd;
	struct media_entity *entity = &sensor->sd.entity;
	int ret = -1;

	// FIXME: is async needed?
	ret = v4l2_device_register_subdev(isx031->mux.sd.subdev.v4l2_dev, sd);
	if (ret < 0) {
		dev_err(sd->dev, "%s(): %d: %d\n", __func__, __LINE__, ret);
		return ret;
	}

	ret = media_create_pad_link(entity, 0,
			&isx031->mux.sd.subdev.entity, sensor->mux_pad,
			MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
	if (ret < 0) {
		dev_err(sd->dev, "%s(): %d: %d\n", __func__, __LINE__, ret);
		goto e_sd;
	}

	dev_dbg(sd->dev, "%s(): 0 -> %d\n", __func__, sensor->mux_pad);

	return 0;

e_sd:
	v4l2_device_unregister_subdev(sd);

	return ret;
}

static void isx031_sensor_remove(struct isx031_sensor *sensor)
{
	media_entity_cleanup(&sensor->sd.entity);
	v4l2_device_unregister_subdev(&sensor->sd);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
	if (sensor->sd.internal_ops)
	  sensor->sd.internal_ops = NULL;
#endif
}

static int isx031_yuv_init(struct i2c_client *c, struct isx031 *isx031)
{
	int ret;

	isx031->yuv.sensor.mux_pad = ISX031_MUX_PAD_YUV;
	ret = isx031_sensor_init(c, isx031, &isx031->yuv.sensor,
		       &isx031_yuv_subdev_ops, "yuv");
	if (ret < 0)
		return ret;

#ifdef CONFIG_VIDEO_ISX031_SERDES
	ret = max9295_set_mfp(isx031->ser_dev, 0, 0);// Set XCLR
	if (ret < 0)
		return ret;
	msleep(10);
	ret = max9295_set_mfp(isx031->ser_dev, 0, 1);// Release XCLR
	if (ret < 0)
		return ret;
	msleep(100);
#endif

	ret = isx031_sensor_boot(isx031);
	if (ret < 0)
		return ret;

	return ret;
}

/* No locking needed */
static int isx031_mux_enum_mbus_code(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 10)
				     struct v4l2_subdev_pad_config *cfg,
#else
				     struct v4l2_subdev_state *v4l2_state,
#endif
				  struct v4l2_subdev_mbus_code_enum *mce)
{
	struct isx031 *isx031 = container_of(sd, struct isx031, mux.sd.subdev);
	struct v4l2_subdev_mbus_code_enum tmp = *mce;
	struct v4l2_subdev *remote_sd;
	int ret = -1;

	dev_dbg(&isx031->client->dev, "%s(): %s \n", __func__, sd->name);
	switch (mce->pad) {
	case ISX031_MUX_PAD_YUV:
		remote_sd = &isx031->yuv.sensor.sd;
		break;
	case ISX031_MUX_PAD_EXTERNAL:
		remote_sd = &isx031->yuv.sensor.sd;
		break;
	default:
		return -EINVAL;
	}

	tmp.pad = 0;
	if (isx031->is_yuv)
		remote_sd = &isx031->yuv.sensor.sd;
	/* Locks internally */
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 10)
	ret = isx031_sensor_enum_mbus_code(remote_sd, cfg, &tmp);
#else
	ret = isx031_sensor_enum_mbus_code(remote_sd, v4l2_state, &tmp);
#endif
	if (!ret)
		mce->code = tmp.code;

	return ret;
}
static int isx031_state_to_pad(struct isx031 *isx031) {
	int pad = -1;
	if (isx031->is_yuv)
		pad = ISX031_MUX_PAD_YUV;
	return pad;
}

/* No locking needed */
static int isx031_mux_enum_frame_size(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 10)
				     struct v4l2_subdev_pad_config *cfg,
#else
				     struct v4l2_subdev_state *v4l2_state,
#endif
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct isx031 *isx031 = container_of(sd, struct isx031, mux.sd.subdev);
	struct v4l2_subdev_frame_size_enum tmp = *fse;
	struct v4l2_subdev *remote_sd;
	u32 pad = fse->pad;
	int ret = -1;

	tmp.pad = 0;
	pad = isx031_state_to_pad(isx031);

	switch (pad) {
	case ISX031_MUX_PAD_YUV:
		remote_sd = &isx031->yuv.sensor.sd;
		break;
	case ISX031_MUX_PAD_EXTERNAL:
		remote_sd = &isx031->yuv.sensor.sd;
		break;
	default:
		return -EINVAL;
	}

	/* Locks internally */
	ret = isx031_sensor_enum_frame_size(remote_sd, NULL, &tmp);
	if (!ret) {
		*fse = tmp;
		fse->pad = pad;
	}

	return ret;
}

/* No locking needed */
static int isx031_mux_enum_frame_interval(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 10)
				     struct v4l2_subdev_pad_config *cfg,
#else
				     struct v4l2_subdev_state *v4l2_state,
#endif
				     struct v4l2_subdev_frame_interval_enum *fie)
{
	struct isx031 *isx031 = container_of(sd, struct isx031, mux.sd.subdev);
	struct device *dev = &isx031->client->dev;
	struct v4l2_subdev_frame_interval_enum tmp = *fie;
	struct v4l2_subdev *remote_sd;
	u32 pad = fie->pad;
	int ret = -1;

	dev_dbg(dev, "%s(): pad %d code %x width %d height %d\n",
			__func__, fie->pad, fie->code, fie->width, fie->height);

	pad = isx031_state_to_pad(isx031);

	switch (pad) {
	case ISX031_MUX_PAD_YUV:
		remote_sd = &isx031->yuv.sensor.sd;
		break;
	case ISX031_MUX_PAD_EXTERNAL:
		remote_sd = &isx031->yuv.sensor.sd;
		break;
	default:
		return -EINVAL;
	}

	/* Locks internally */
	tmp.pad = 0;
	ret = isx031_sensor_enum_frame_interval(remote_sd, NULL, &tmp);
	if (!ret) {
		*fie = tmp;
		fie->pad = pad;
	}

	return ret;
}

/* No locking needed */
static int isx031_mux_set_fmt(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 10)
		struct v4l2_subdev_pad_config *cfg,
#else
		struct v4l2_subdev_state *v4l2_state,
#endif
		struct v4l2_subdev_format *fmt)
{
	struct isx031 *isx031 = container_of(sd, struct isx031, mux.sd.subdev);
	struct v4l2_mbus_framefmt *ffmt;
	struct isx031_sensor *sensor = isx031->mux.last_set;
	u32 pad = sensor->mux_pad;
	int ret = 0;
#ifdef CONFIG_VIDEO_INTEL_IPU6
	int substream = -1;
#endif

	dev_dbg(sd->dev, "%s:%d: fmt->pad:%d, sensor->mux_pad: %d, \
		 for sensor: %s\n",
		__func__, __LINE__,
		fmt->pad, pad,
		sensor->sd.name);

	isx031_s_state_pad(isx031, pad);
	sensor = isx031->mux.last_set;
	switch (pad) {
	case ISX031_MUX_PAD_YUV:
		ffmt = &sensor->format;
		break;
	case ISX031_MUX_PAD_EXTERNAL:
		ffmt = &isx031_ffmts[pad];
	default:
		return -EINVAL;
	}

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		ffmt->width = fmt->format.width;
		ffmt->height = fmt->format.height;
		ffmt->code = fmt->format.code;
	}
	fmt->format = *ffmt;

#ifdef CONFIG_VIDEO_INTEL_IPU6
	substream = isx031->pad_to_substream[pad];

	if (substream != -1) {
		set_sub_stream_fmt(substream, ffmt->code);
		set_sub_stream_h(substream, ffmt->height);
		set_sub_stream_w(substream, ffmt->width);
		set_sub_stream_dt(substream, mbus_code_to_mipi(ffmt->code));
	}

	dev_dbg(sd->dev, "%s(): fmt->pad:%d, sensor->mux_pad: %d, \
		code: 0x%x: %ux%u substream:%d for sensor: %s\n",
		__func__,
		fmt->pad, pad, fmt->format.code,
		fmt->format.width, fmt->format.height,
		substream, sensor->sd.name);
#endif

	return ret;
}

/* No locking needed */
static int isx031_mux_get_fmt(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 10)
				     struct v4l2_subdev_pad_config *cfg,
#else
				     struct v4l2_subdev_state *v4l2_state,
#endif
			   struct v4l2_subdev_format *fmt)
{
	struct isx031 *isx031 = container_of(sd, struct isx031, mux.sd.subdev);
	u32 pad = fmt->pad;
	int ret = 0;
	struct isx031_sensor *sensor = isx031->mux.last_set;

	BUG_ON(sensor == NULL);

#ifdef CONFIG_VIDEO_INTEL_IPU6
	pad = sensor->mux_pad; 
	if (pad != ISX031_MUX_PAD_EXTERNAL)
		isx031_s_state_pad(isx031, pad);
#else
	pad = isx031_state_to_pad(isx031);
#endif
	sensor = isx031->mux.last_set;

	dev_dbg(sd->dev, "%s(): %u %s %p\n", __func__, pad, isx031_get_sensor_name(isx031), isx031->mux.last_set);


	switch (pad) {
	case ISX031_MUX_PAD_YUV:
		fmt->format = sensor->format;
		break;
	case ISX031_MUX_PAD_EXTERNAL:
		fmt->format = isx031_ffmts[pad];
	default:
		return -EINVAL;
	}

	dev_dbg(sd->dev, "%s(): fmt->pad:%d, sensor->mux_pad:%u size:%d-%d, code:0x%x field:%d, color:%d\n",
		__func__, fmt->pad, pad,
		fmt->format.width, fmt->format.height, fmt->format.code,
		fmt->format.field, fmt->format.colorspace);
	return ret;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
/* pad ops */
static int isx031_mux_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *sd_state,
				    struct v4l2_subdev_frame_interval *fi)
#else
/* Video ops */
static int isx031_mux_g_frame_interval(struct v4l2_subdev *sd,
		struct v4l2_subdev_frame_interval *fi)
#endif
{
	struct isx031 *isx031 = container_of(sd, struct isx031, mux.sd.subdev);
	struct isx031_sensor *sensor = NULL;

	if (NULL == sd || NULL == fi)
		return -EINVAL;

	sensor = isx031->mux.last_set;

	fi->interval.numerator = 1;
	fi->interval.denominator = sensor->config.framerate;

	dev_dbg(sd->dev, "%s(): %s %u\n", __func__, sd->name,
			fi->interval.denominator);

	return 0;
}

static u16 __isx031_probe_framerate(const struct isx031_resolution *res, u16 target)
{
	int i;
	u16 framerate;

	for (i = 0; i < res->n_framerates; i++) {
		framerate = res->framerates[i];
		if (target <= framerate)
			return framerate;
	}

	return res->framerates[res->n_framerates - 1];
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
/* pad ops */
static int isx031_mux_s_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *sd_state,
				    struct v4l2_subdev_frame_interval *fi)
#else
/* Video ops */
static int isx031_mux_s_frame_interval(struct v4l2_subdev *sd,
		struct v4l2_subdev_frame_interval *fi)
#endif
{
	struct isx031 *isx031 = container_of(sd, struct isx031, mux.sd.subdev);
	struct isx031_sensor *sensor = NULL;
	u16 framerate = 1;

	if (NULL == sd || NULL == fi || fi->interval.numerator == 0)
		return -EINVAL;

	sensor = isx031->mux.last_set;

	framerate = fi->interval.denominator / fi->interval.numerator;
	framerate = __isx031_probe_framerate(sensor->config.resolution, framerate);
	sensor->config.framerate = framerate;
	fi->interval.numerator = 1;
	fi->interval.denominator = framerate;

	dev_dbg(sd->dev, "%s(): %s %u\n", __func__, sd->name, framerate);

	return 0;
}

#ifdef CONFIG_VIDEO_INTEL_IPU6
static int isx031_state_to_vc(struct isx031 *isx031) {
	int pad = 0;
	if (isx031->is_yuv) {
		pad = ISX031_MUX_PAD_YUV;
	}

	return isx031->pad_to_vc[pad];
}
#endif

static int isx031_mux_s_stream(struct v4l2_subdev *sd, int on)
{
	struct isx031 *isx031 = container_of(sd, struct isx031, mux.sd.subdev);
	u16 streaming;
	int ret = 0;
	unsigned int i = 0;
	int restore_val = 0;
	u16 vc_id;
	struct isx031_sensor *sensor = isx031->mux.last_set;

dev_dbg(&isx031->client->dev, "isx031_mux_s_stream for stream sensor->streaming=%d\n", sensor->streaming);

	// spare duplicate calls
	if (sensor->streaming == on)
		return 0;

#ifdef CONFIG_VIDEO_INTEL_IPU6
	vc_id = isx031_state_to_vc(isx031);
# ifdef CONFIG_VIDEO_ISX031_SERDES
	// set manually, need to configure vc in pdata
	isx031->g_ctx.dst_vc = vc_id;
# endif
#endif
	dev_dbg(&isx031->client->dev, "s_stream for stream %s, vc:%d, SENSOR=%s on = %d\n",
			sensor->sd.name, vc_id, isx031_get_sensor_name(isx031), on);

	restore_val = sensor->streaming;
	sensor->streaming = on;

	if (on) {
#ifdef CONFIG_VIDEO_ISX031_SERDES
		sensor->pipe_id =
			max9296_get_available_pipe_id(isx031->dser_dev,
					(int)isx031->g_ctx.dst_vc);
		if (sensor->pipe_id < 0) {
			dev_err(&isx031->client->dev,
				"No free pipe in max9296\n");
			ret = -(ENOSR);
			goto restore_s_state;
		}

		ret = max9295_setup_streaming(isx031->ser_dev);
		ret |= max9296_setup_streaming(isx031->dser_dev, &isx031->client->dev);
		ret |= max9296_start_streaming(isx031->dser_dev, &isx031->client->dev);
		if (ret)
			dev_err(&isx031->client->dev, "Failure setting up/starting ser/des");
#endif

		ret = isx031_configure(isx031);
		if (ret)
			goto restore_s_state;

		ret = isx031_start_streaming(isx031);
		if (ret < 0)
			goto restore_s_state;

	} else { // off

		isx031_stop_streaming(isx031);

#ifdef CONFIG_VIDEO_ISX031_SERDES
		ret |= max9296_stop_streaming(isx031->dser_dev, &isx031->client->dev);
		if (ret)
			dev_err(&isx031->client->dev, "Failure stopping des");

# ifdef CONFIG_VIDEO_INTEL_IPU6
		// reset for IPU6
		streaming = 0;
		for (i = 0; i < ARRAY_SIZE(isx031_set_sub_stream); i++) {
			if (isx031_set_sub_stream[i]) {
				streaming = 1;
				break;
			}
		}
		if (!streaming) {
			dev_warn(&isx031->client->dev, "max9296_reset_oneshot\n");
				max9296_reset_oneshot(isx031->dser_dev);
		}
# endif
		if (max9296_release_pipe(isx031->dser_dev, sensor->pipe_id) < 0)
			dev_warn(&isx031->client->dev, "release pipe failed\n");
		sensor->pipe_id = -1;
#endif // CONFIG_VIDEO_ISX031_SERDES
	}

	return ret;

restore_s_state:
#ifdef CONFIG_VIDEO_ISX031_SERDES
	if (on && sensor->pipe_id >= 0) {
		if (max9296_release_pipe(isx031->dser_dev, sensor->pipe_id) < 0)
			dev_warn(&isx031->client->dev, "release pipe failed\n");
		sensor->pipe_id = -1;
	}
#endif

	sensor->streaming = restore_val;

	return ret;
}

static int isx031_mux_get_frame_desc(struct v4l2_subdev *sd,
	unsigned int pad, struct v4l2_mbus_frame_desc *desc)
{
	unsigned int i;

	desc->num_entries = V4L2_FRAME_DESC_ENTRY_MAX;

	for (i = 0; i < desc->num_entries; i++) {
		desc->entry[i].flags = 0;
		desc->entry[i].pixelcode = MEDIA_BUS_FMT_FIXED;
		desc->entry[i].length = 0;
	}
	return 0;
}

static const struct v4l2_subdev_pad_ops isx031_mux_pad_ops = {
	.enum_mbus_code		= isx031_mux_enum_mbus_code,
	.enum_frame_size	= isx031_mux_enum_frame_size,
	.enum_frame_interval	= isx031_mux_enum_frame_interval,
	.get_fmt		= isx031_mux_get_fmt,
	.set_fmt		= isx031_mux_set_fmt,
	.get_frame_desc		= isx031_mux_get_frame_desc,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
	.get_frame_interval	= isx031_mux_g_frame_interval,
	.set_frame_interval	= isx031_mux_s_frame_interval,
#endif
};

static const struct v4l2_subdev_core_ops isx031_mux_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
};

static const struct v4l2_subdev_video_ops isx031_mux_video_ops = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 8, 0)
	.g_frame_interval	= isx031_mux_g_frame_interval,
	.s_frame_interval	= isx031_mux_s_frame_interval,
#endif
	.s_stream		= isx031_mux_s_stream,
};

static const struct v4l2_subdev_ops isx031_mux_subdev_ops = {
	.core = &isx031_mux_core_ops,
	.pad = &isx031_mux_pad_ops,
	.video = &isx031_mux_video_ops,
};

static int isx031_mux_registered(struct v4l2_subdev *sd)
{
	struct isx031 *isx031 = v4l2_get_subdevdata(sd);
	int ret;

	ret = isx031_sensor_register(isx031, &isx031->yuv.sensor);
	if (ret < 0)
		goto e;

	dev_dbg(sd->dev, "%s(): %d: registered v4l2_subdev \n",
		__func__, __LINE__);
	return 0;

e:
	return ret;
}

static void isx031_mux_unregistered(struct v4l2_subdev *sd)
{
	dev_dbg(sd->dev, "%s(): %d: unregister v4l2_subdev \n",
		__func__, __LINE__);
	struct isx031 *isx031 = v4l2_get_subdevdata(sd);
	isx031_sensor_remove(&isx031->yuv.sensor);
}

static const struct v4l2_subdev_internal_ops isx031_mux_internal_ops = {
	.registered = isx031_mux_registered,
	.unregistered = isx031_mux_unregistered,
};

static int isx031_mux_register(struct i2c_client *c, struct isx031 *isx031)
{
	return v4l2_async_register_subdev(&isx031->mux.sd.subdev);
}

static int isx031_mux_init(struct i2c_client *c, struct isx031 *isx031)
{
	struct v4l2_subdev *sd = &isx031->mux.sd.subdev;
	struct media_entity *entity = &isx031->mux.sd.subdev.entity;
	struct media_pad *pads = isx031->mux.pads, *pad;
	unsigned int i;
	int ret;
#ifdef CONFIG_VIDEO_ISX031_SERDES
	struct serdes_platform_data *dpdata = c->dev.platform_data;
	char suffix = dpdata->suffix;
#endif
	v4l2_i2c_subdev_init(sd, c, &isx031_mux_subdev_ops);
	// See tegracam_v4l2.c tegracam_v4l2subdev_register()
	// Set owner to NULL so we can unload the driver module
	sd->owner = NULL;
	sd->internal_ops = &isx031_mux_internal_ops;
	v4l2_set_subdevdata(sd, isx031);
#ifndef CONFIG_VIDEO_ISX031_SERDES
	snprintf(sd->name, sizeof(sd->name), "ISX031 mux %d-%04x",
		 i2c_adapter_id(c->adapter), c->addr);
#else
	if (isx031->aggregated)
		suffix += AGGREGATED_SUFFIX_OFFSET;
	snprintf(sd->name, sizeof(sd->name), "ISX031 mux %c", suffix);
#endif
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	entity->obj_type = MEDIA_ENTITY_TYPE_V4L2_SUBDEV;
	entity->function = MEDIA_ENT_F_CAM_SENSOR;

	dev_info(&c->dev,
		 "%s():%d init media_entity %s, type:%x, func:%x\n",
		 __func__, __LINE__,
		 sd->name,
		 entity->obj_type,
		 entity->function);

	pads[0].flags = MEDIA_PAD_FL_SOURCE;
	for (i = 1, pad = pads + 1; i < ARRAY_SIZE(isx031->mux.pads); i++, pad++)
		pad->flags = MEDIA_PAD_FL_SINK;

	ret = media_entity_pads_init(entity, ARRAY_SIZE(isx031->mux.pads), pads);
	if (ret < 0)
		return ret;

	ret = isx031_ctrl_init(isx031, MUX_SID);
	if (ret < 0)
		goto e_entity;

	/*set for yuv*/
	ret = isx031_ctrl_init(isx031, YUV_SID);
	if (ret < 0)
		goto e_entity;

	isx031_set_state_last_set(isx031);

	return 0;

e_entity:
	media_entity_cleanup(entity);

	return ret;
}

static int isx031_fixed_configuration(struct i2c_client *client, struct isx031 *isx031)
{
	struct isx031_sensor *sensor;

	sensor = &isx031->yuv.sensor;
	sensor->formats = &isx031_onsemi_yuv_format;
	sensor->n_formats = ISX031_ONSEMI_YUV_N_FORMATS;
	sensor->mux_pad = ISX031_MUX_PAD_YUV;

	return 0;
}

static int isx031_parse_cam(struct i2c_client *client, struct isx031 *isx031)
{
	int ret;

	ret = isx031_fixed_configuration(client, isx031);
	if (ret < 0)
		return ret;

	isx031_sensor_format_init(&isx031->yuv.sensor);

	return 0;
}

static void isx031_mux_remove(struct isx031 *isx031)
{
	v4l2_async_unregister_subdev(&isx031->mux.sd.subdev);
	v4l2_ctrl_handler_free(isx031->mux.sd.subdev.ctrl_handler);
	media_entity_cleanup(&isx031->mux.sd.subdev.entity);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
	v4l2_device_unregister_subdev(&isx031->mux.sd.subdev);
	if (isx031->mux.sd.subdev.internal_ops)
	  isx031->mux.sd.subdev.internal_ops = NULL;
#endif
}

static struct isx031_hwcfg *isx031_get_hwcfg(struct isx031 *isx031, struct device *dev)
{
	struct isx031_hwcfg *cfg;
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	int ret;

	endpoint =
		fwnode_graph_get_endpoint_by_id(dev_fwnode(dev), 0, 0,
						FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found");
		return -EPROBE_DEFER;
	}

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &bus_cfg);
	if (ret) {
		dev_err(dev, "parsing endpoint node failed");
		goto out_err;
	}

	cfg = devm_kzalloc(dev, sizeof(*cfg), GFP_KERNEL);
	if (!cfg)
		goto out_err;

	/* Check the number of MIPI CSI2 data lanes */
	if (bus_cfg.bus.mipi_csi2.num_data_lanes != 2 ) {
		dev_err(dev, "only 2 data lanes are currently supported");
		goto out_err;
	}

	// ret = v4l2_link_freq_to_bitmap(dev, bus_cfg.link_frequencies,
	// 			       bus_cfg.nr_of_link_frequencies,
	// 			       link_freq_menu_items,
	// 			       ARRAY_SIZE(link_freq_menu_items),
	// 			       &isx031->link_freq_bitmap);
	// if (ret)
	// 	goto out_err;

	v4l2_fwnode_endpoint_free(&bus_cfg);
	fwnode_handle_put(endpoint);
	return cfg;

out_err:
	v4l2_fwnode_endpoint_free(&bus_cfg);
	fwnode_handle_put(endpoint);
	return NULL;
}

static int isx031_v4l_init(struct i2c_client *c, struct isx031 *isx031)
{
	int ret;

	ret = isx031_parse_cam(c, isx031);
	if (ret < 0)
		return ret;

	ret = isx031_yuv_init(c, isx031);
	if (ret < 0)
		return ret;

	ret = isx031_mux_init(c, isx031);
	if (ret < 0)
		goto e_yuv;

	ret = isx031_mux_register(c, isx031);
	if (ret < 0)
		goto e_mux;

	return 0;
e_mux:
	isx031_mux_remove(isx031);
e_yuv:
	media_entity_cleanup(&isx031->yuv.sensor.sd.entity);
	return ret;
}

#ifdef CONFIG_VIDEO_INTEL_IPU6
static void isx031_substream_init(struct isx031 *isx031)
{
	int i;
	isx031->pad_to_vc[ISX031_MUX_PAD_EXTERNAL] = -1;
	if (!isx031->aggregated) {
		isx031->pad_to_vc[ISX031_MUX_PAD_YUV]   = sensor_vc[0];
	} else {
		isx031->pad_to_vc[ISX031_MUX_PAD_YUV]   = sensor_vc[4];
	}

	for (i = 0; i < ARRAY_SIZE(isx031->pad_to_substream); i++)
		isx031->pad_to_substream[i] = -1;
	/* match for IPU6 CSI2 BE SOC video capture pads */
	if (!isx031->aggregated) {
		isx031->pad_to_substream[ISX031_MUX_PAD_YUV]   = 0;
	}
	else {
		isx031->pad_to_substream[ISX031_MUX_PAD_YUV]   = 3;
	}
	dev_info(&isx031->client->dev, "%s() IPU6 CSI2 BE SOC video capture init : \n", __func__);
	for (i = 0; i < ARRAY_SIZE(isx031->pad_to_substream); i++)
	  if (isx031->pad_to_substream[i] >= 0)
	    dev_info(&isx031->client->dev, "pad[%d]->substream=%d\n",
			 i, isx031->pad_to_substream[i]);
	/*YUV*/
	set_sub_stream_fmt  (isx031->pad_to_substream[ISX031_MUX_PAD_YUV], MEDIA_BUS_FMT_UYVY8_1X16);
	set_sub_stream_h    (isx031->pad_to_substream[ISX031_MUX_PAD_YUV], 1920);
	set_sub_stream_w    (isx031->pad_to_substream[ISX031_MUX_PAD_YUV], 1536);
	set_sub_stream_dt   (isx031->pad_to_substream[ISX031_MUX_PAD_YUV], mbus_code_to_mipi(MEDIA_BUS_FMT_UYVY8_1X16));
	set_sub_stream_vc_id(isx031->pad_to_substream[ISX031_MUX_PAD_YUV], isx031->pad_to_vc[ISX031_MUX_PAD_YUV]);
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
static int isx031_probe(struct i2c_client *c, const struct i2c_device_id *id)
#else
static int isx031_probe(struct i2c_client *c)
#endif
{
	struct isx031 *isx031;
	int ret = 0;

	dev_info(&c->dev, "ISX031 probe\n");

	isx031 = devm_kzalloc(&c->dev, sizeof(*isx031), GFP_KERNEL);
	if (!isx031)
		return -ENOMEM;

	mutex_init(&isx031->lock);

	isx031->client = c;
	isx031->is_yuv = 1;
#ifdef CONFIG_VIDEO_ISX031_SERDES
	isx031->dser_st.isolated = false;
#endif
	dev_warn(&c->dev, "Probing driver for ISX031 GMSL\n");
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	isx031->variant = isx031_variants + id->driver_data;
#else
	isx031->variant = isx031_variants;
#endif

	isx031->hwcfg = isx031_get_hwcfg(isx031, &c->dev);
	if (!isx031->hwcfg) {
		ret = -ENODEV;
		goto e_probe;
	}

#ifdef CONFIG_OF
	isx031->vcc = devm_regulator_get(&c->dev, "vcc");
	if (IS_ERR(isx031->vcc)) {
		ret = PTR_ERR(isx031->vcc);
		dev_err(&c->dev, "failed %d to get vcc regulator\n", ret);
		return ret;
	}

	if (isx031->vcc) {
		ret = regulator_enable(isx031->vcc);
		if (ret < 0) {
			dev_err(&c->dev, "failed %d to enable the vcc regulator\n", ret);
			return ret;
		}
	}
#endif

#ifdef CONFIG_VIDEO_ISX031_SERDES
	ret = isx031_serdes_setup(isx031);
	if (ret < 0) {
		if (ret == -ENOTSUPP)
			dev_warn(&c->dev, "max9295 communication failed : %d\n", ret);
		goto e_regulator;
	}
#endif

	ret = isx031_v4l_init(c, isx031);
	if (ret < 0)
		goto e_regulator;

#ifdef CONFIG_VIDEO_INTEL_IPU6
	isx031_substream_init(isx031);
#endif
	return 0;

e_regulator:
	if (isx031->vcc)
		regulator_disable(isx031->vcc);
#ifdef CONFIG_VIDEO_ISX031_SERDES
	int i;
	int c_bus = c->adapter->nr;
	bool graceful_fallback = false;
	for (i = 0; i < MAX_DEV_NUM; i++) {
		if (serdes_inited[i]
		    && serdes_inited[i] != isx031
		    && isx031->dser_i2c
		    && c_bus == serdes_inited[i]->dser_st.bus_nr
		    && isx031->dser_i2c->addr == serdes_inited[i]->dser_st.addr
		    && serdes_inited[i]->dser_st.isolated) {

			dev_info(&c->dev, "Cleanup unresponsive sensor/serializer Isolated on bus %d\n",
				 c_bus);
			graceful_fallback = true;
		}
	}

	if (!isx031->g_ctx.serdev_found)
		dev_warn(&c->dev, "graceful fallback due to unresponsive max9295, isolated SerDes %s single-link\n",
			 isx031->g_ctx.serdes_csi_link == GMSL_SERDES_CSI_LINK_A ? "GMSL A": "GMSL B");
	else
		dev_warn(&c->dev, "graceful fallback due to unresponsive d4xx, isolated SerDes %s single-link\n",
			 isx031->g_ctx.serdes_csi_link == GMSL_SERDES_CSI_LINK_A ? "GMSL A": "GMSL B");

	mutex_lock(&serdes_lock__);
	if (isx031->ser_i2c) {
		dev_info(&c->dev, "remove unresponding serializer i2c device 0x%x\n",
			 isx031->ser_i2c->addr);
		i2c_unregister_device(isx031->ser_i2c);
	}
	if (isx031->dser_i2c && !isx031->aggregated) {
		dev_info(&c->dev, "remove  unresponding %s single-link deserializer i2c device 0x%x\n",
			isx031->g_ctx.serdes_csi_link == GMSL_SERDES_CSI_LINK_A ? "GMSL A": "GMSL B",
			isx031->dser_i2c->addr);
		i2c_unregister_device(isx031->dser_i2c);
		isx031->dser_st.isolated = true;
	} else if (isx031->dser_i2c && graceful_fallback) {
		dev_info(&c->dev, "remove  unresponding %s single-link deserializer i2c device 0x%x\n",
			isx031->g_ctx.serdes_csi_link == GMSL_SERDES_CSI_LINK_A ? "GMSL A": "GMSL B",
			isx031->dser_i2c->addr);
		i2c_unregister_device(isx031->dser_i2c);
	}
	mutex_unlock(&serdes_lock__);
#endif
e_probe:
	return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
static int isx031_remove(struct i2c_client *c)
#else
static void isx031_remove(struct i2c_client *c)
#endif
{
	struct isx031 *isx031 = container_of(i2c_get_clientdata(c), struct isx031, mux.sd.subdev);

#ifdef CONFIG_VIDEO_ISX031_SERDES
	int i, ret;
	int c_bus = c->adapter->nr;
	bool graceful_fallback = false;
	for (i = 0; i < MAX_DEV_NUM; i++) {
		if (serdes_inited[i] && isx031->dser_i2c
		    && c_bus == serdes_inited[i]->dser_st.bus_nr
		    && isx031->dser_i2c->addr == serdes_inited[i]->dser_st.addr
		    && serdes_inited[i]->dser_st.isolated) {

			dev_info(&c->dev, "Cleanup unresponsive sensor/serializer Isolated on bus %d\n",
				 c_bus);
			graceful_fallback = true;
		}
		if (serdes_inited[i] && serdes_inited[i] == isx031) {
			serdes_inited[i] = NULL;
			mutex_lock(&serdes_lock__);

			ret = max9295_reset_control(isx031->ser_dev);
			if (ret)
				dev_warn(&c->dev,
				  "failed in 9295 reset control\n");
			if (isx031->dser_i2c) {
				dev_info(&c->dev, "ignore 9296 reset control, already remove for bus %d\n", c_bus);
			} else {
				dev_info(&c->dev, "trigger 9296 reset control on bus %d\n", c_bus);
				ret = max9296_reset_control(isx031->dser_dev,
							    isx031->g_ctx.s_dev);
				if (ret)
				  dev_warn(&c->dev,
					   "failed in 9296 reset control\n");
			}
			ret = max9295_sdev_unpair(isx031->ser_dev,
				isx031->g_ctx.s_dev);
			if (ret)
				dev_warn(&c->dev, "failed to unpair sdev\n");

			if (isx031->dser_i2c) {
				dev_info(&c->dev, "ignore 9296 unregister sdev, already remove for bus %d\n", c_bus);
			} else {
				dev_info(&c->dev, "unregister 9296 sdev on bus %d\n", c_bus);
				ret = max9296_sdev_unregister(isx031->dser_dev,
							      isx031->g_ctx.s_dev);
				if (ret)
				  dev_warn(&c->dev,
					   "failed to sdev unregister sdev\n");

				max9296_power_off(isx031->dser_dev);
			}
			mutex_unlock(&serdes_lock__);
			break;
		}
	}
	if (isx031->ser_i2c && !isx031->dser_st.isolated) {
		dev_info(&c->dev, "remove unresponding serializer i2c device 0x%x\n",
			isx031->ser_i2c->addr);
		i2c_unregister_device(isx031->ser_i2c);
	}
	if (isx031->dser_i2c && !isx031->aggregated && !isx031->dser_st.isolated) {
		i2c_unregister_device(isx031->dser_i2c);
		dev_info(&c->dev, "remove  unresponding %s single-link deserializer i2c device 0x%x\n",
			isx031->g_ctx.serdes_csi_link == GMSL_SERDES_CSI_LINK_A ? "GMSL A": "GMSL B",
			isx031->dser_i2c->addr);
	} else if (isx031->dser_i2c && graceful_fallback) {
		dev_info(&c->dev, "remove  unresponding %s single-link deserializer i2c device 0x%x\n",
			isx031->g_ctx.serdes_csi_link == GMSL_SERDES_CSI_LINK_A ? "GMSL A": "GMSL B",
			isx031->dser_i2c->addr);
		i2c_unregister_device(isx031->dser_i2c);
	}
#endif
	isx031->is_yuv = 1;
	dev_info(&c->dev, "ISX031 remove %s\n",
			isx031_get_sensor_name(isx031));
	if (isx031->vcc)
		regulator_disable(isx031->vcc);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
	isx031_mux_unregistered(&isx031->mux.sd.subdev);
#endif
	isx031_mux_remove(isx031);

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	return 0;
#endif
}

static const struct acpi_device_id isx031_acpi_ids[] = {
	{ "INTC1031" },
	{}
};
MODULE_DEVICE_TABLE(acpi, isx031_acpi_ids);

static const struct i2c_device_id isx031_id[] = {
	{ ISX031_NAME, ISX031_ISX031U },
	{ },
};
MODULE_DEVICE_TABLE(i2c, isx031_id);

static const struct of_device_id isx031_of_match[] = {
	{ .compatible = "d3,isx031", },
	{ },
};
MODULE_DEVICE_TABLE(of, isx031_of_match);

static struct i2c_driver isx031_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = ISX031_NAME,
		.acpi_match_table = ACPI_PTR(isx031_acpi_ids),
		.of_match_table = isx031_of_match,
	},
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0) && LINUX_VERSION_CODE < KERNEL_VERSION(6, 6, 0)
	.probe_new	= isx031_probe,
#else
	.probe		= isx031_probe,
#endif
	.remove		= isx031_remove,
	.id_table	= isx031_id,
};

module_i2c_driver(isx031_i2c_driver);

MODULE_DESCRIPTION("D3 ISX031 (GMSL)");
MODULE_AUTHOR("No one <noeone@nowhere.com");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0.0");
