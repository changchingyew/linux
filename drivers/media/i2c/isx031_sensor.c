#ifndef ISX031_SENSOR_C
#define ISX031_SENSOR_C

#include "isx031_registers.h"

#define ISX031_REG_VALUE_08BIT      1
#define ISX031_REG_VALUE_16BIT      2
#define ISX031_REG_VALUE_32BIT      4



#define isx031_get_client(_isx031)  (isx031->client)

#define TRACE_ISX031(o) do { \
        struct i2c_client *TRACE_client = isx031_get_client(isx031); \
        dev_dbg(&TRACE_client->dev, "TRACE %s:%d", __func__, __LINE__); \
} while(0)


static int isx031_read_reg(struct isx031 *isx031, u16 reg, u16 len, u32 *val);
static int isx031_write_reg(struct isx031 *isx031, u16 reg, u16 len, u32 val);
static int isx031_modify_reg(struct isx031 *isx031, u16 reg, u16 len, u32 mask, u32 val);
static int isx031_wait_for_sensor_state(struct isx031 *isx031, u32 sensor_state);
static int isx031_sensor_boot(struct isx031 *isx031);
static int isx031_configure_mode(struct isx031 *isx031);
static int isx031_start_streaming(struct isx031 *isx031);
static void isx031_stop_streaming(struct isx031 *isx031);

static int isx031_read_reg(struct isx031 *isx031, u16 reg, u16 len, u32 *val)
{
    struct i2c_client *client = isx031_get_client(isx031);
    struct i2c_msg msgs[2];
    u8 addr_buf[2];
    u8 data_buf[4] = {0};
    int ret;

    if (len > 4)
        return -EINVAL;

    put_unaligned_be16(reg, addr_buf);
    msgs[0].addr = client->addr;
    msgs[0].flags = 0;
    msgs[0].len = sizeof(addr_buf);
    msgs[0].buf = addr_buf;
    msgs[1].addr = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = len;
    msgs[1].buf = data_buf;

    ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
    if (ret != ARRAY_SIZE(msgs))
        return -EIO;

    switch (len) {
        case ISX031_REG_VALUE_08BIT:
            *val = data_buf[0];
            break;
        case ISX031_REG_VALUE_16BIT:
            *val = get_unaligned_le16(data_buf);
            break;
        case ISX031_REG_VALUE_32BIT:
            *val = get_unaligned_le32(data_buf);
            break;

        default:
            return -EINVAL;
    }

    return 0;
}

static int isx031_write_reg(struct isx031 *isx031, u16 reg, u16 len, u32 val)
{
    struct i2c_client *client = isx031_get_client(isx031);
    u8 buf[6];

    if (len > 4)
        return -EINVAL;

    put_unaligned_be16(reg, buf);
    switch (len) {
        case ISX031_REG_VALUE_08BIT:
            buf[2] = (val & 0xFF);
            break;
        case ISX031_REG_VALUE_16BIT:
            put_unaligned_le16(val, &buf[2]);
            break;
        case ISX031_REG_VALUE_32BIT:
            put_unaligned_le32(val, &buf[2]);
            break;

        default:
            return -EINVAL;
    }

    if (i2c_master_send(client, buf, len + 2) != len + 2)
        return -EIO;

    return 0;
}

static int isx031_modify_reg(struct isx031 *isx031, u16 reg, u16 len, u32 mask, u32 val)
{
    u32 temp;
    int ret;

    ret = isx031_read_reg(isx031, reg, len, &temp);
    if (ret)
        return ret;

    temp &= ~(mask);
    temp |= val;

    return isx031_write_reg(isx031, reg, len, temp);
}

static int isx031_wait_for_sensor_state(struct isx031 *isx031, u32 sensor_state)
{
    struct i2c_client *client = isx031_get_client(isx031);
    int ret;
    u32 val;
    unsigned long timeout;

    TRACE_ISX031(isx031);

    timeout = jiffies + msecs_to_jiffies(200);
    do {
        ret = isx031_read_reg(isx031,ISX031_REG_DEVSTS, ISX031_REG_VALUE_08BIT, &val);
        if (ret) {
            dev_warn(&client->dev, "Failed to read DEVSTS: %d", ret);
        }
        if (val == sensor_state) {
            dev_dbg(&client->dev, "Enter state %d", val);
            return 0;
        }
        usleep_range(400, 400);
    } while (time_is_after_jiffies(timeout));

    return -ETIMEDOUT;
}

static int isx031_sensor_boot(struct isx031 *isx031)
{
    struct i2c_client *client = isx031_get_client(isx031);
    int ret;
    u32 major_version, minor_version, val;

    TRACE_ISX031(isx031);

    ret = isx031_write_reg(isx031, ISX031_REG_REMAP_MODE,
            ISX031_REG_VALUE_08BIT,
            ISX031_REMAP_MODE_STARTUP);
    if (ret) {
        dev_err(&client->dev, "%d: failed to init: %d", __LINE__, ret);
        return ret;
    }

    ret = isx031_write_reg(isx031, ISX031_REG_IR_DR_2_FEBD_EN, ISX031_REG_VALUE_08BIT, 0); // Disable FEBD
    if (ret != 0) {
        dev_err(&client->dev, "Write to ISX031_REG_IR_DR_2_FEBD_EN failed: %d", ret);
        return ret;
    }

    ret = isx031_write_reg(isx031, ISX031_REG_IR_DR_2_REBD_EN, ISX031_REG_VALUE_08BIT, 0); // Disable REBD
    if (ret != 0) {
        dev_err(&client->dev, "Write to ISX031_REG_IR_DR_2_REBD_EN failed: %d", ret);
        return ret;
    }

    ret = isx031_read_reg(isx031, ISX031_REG_PARAM_MAJOR_VER, ISX031_REG_VALUE_08BIT, &major_version);
    if (ret != 0) {
        dev_err(&client->dev, "Failed to read MAJOR_VER: %d", ret);
        return ret;
    }
    ret = isx031_read_reg(isx031, ISX031_REG_PARAM_MINOR_VER, ISX031_REG_VALUE_08BIT, &minor_version);
    if (ret != 0) {
        dev_err(&client->dev, "Failed to read MINOR_VER: %d", ret);
        return ret;
    }

    dev_info(&client->dev, "Got version: %d.%d", major_version, minor_version);

    ret = isx031_read_reg(isx031, ISX031_REG_DEVSTS, ISX031_REG_VALUE_08BIT, &val);
    if (ret) {
        dev_err(&client->dev, "Failed to read DEVSTS: %d", ret);
        return ret;
    }
    dev_dbg(&client->dev, "Boot state: %d", val);

    if (val != ISX031_SENSOR_STATE_POWER_ON && val != ISX031_SENSOR_STATE_START_UP)
        isx031_stop_streaming(isx031);

    return 0;
}

static int isx031_configure_mode(struct isx031 *isx031)
{
    struct i2c_client *client = isx031_get_client(isx031);
    struct isx031_sensor *sensor = isx031->mux.last_set;
    int ret;
    u32 mode;

    if (!sensor) {
        dev_err(&client->dev, "set_substream() or set_fmt() not called prior to configure mode!\n");
        return -EINVAL;
    }

    TRACE_ISX031(isx031);

    mode = ISX031_MODE_1920x1536_30_4LANE;

    dev_dbg(&client->dev, "configure mode %d", mode);
    ret = isx031_modify_reg(isx031, ISX031_REG_MODE_SEL, ISX031_REG_VALUE_16BIT,
        ISX031_REG_MODE_SEL_MASK << ISX031_REG_MODE_SEL_POS,
        mode << ISX031_REG_MODE_SEL_POS);
    if (ret) {
        dev_err(&client->dev, "failed to set mode: %d", ret);
        return ret;
    }

    return 0;
}

static int isx031_start_streaming(struct isx031 *isx031)
{
    struct i2c_client *client = isx031_get_client(isx031);
    int ret;

    TRACE_ISX031(isx031);

    ret = isx031_configure_mode(isx031);
    if (ret) {
        return ret;
    }

    ret = isx031_write_reg(isx031, ISX031_REG_MODE_SET_F_LOCK,
        ISX031_REG_VALUE_08BIT, ISX031_MODE_SET_F_APPLICATION_LOCK);
    if (ret) {
        dev_err(&client->dev, "failed to set f-lock");
        return ret;
    }

    ret = isx031_modify_reg(isx031, ISX031_REG_MODE_SEL, ISX031_REG_VALUE_16BIT,
        ISX031_REG_MODE_SET_F_MASK << ISX031_REG_MODE_SET_F_POS,
        ISX031_MODE_SET_F_TRANSITION_TO_STREAMING << ISX031_REG_MODE_SET_F_POS);
    if (ret) {
        dev_err(&client->dev, "failed to start streaming");
        return ret;
    }

    ret = isx031_wait_for_sensor_state(isx031, ISX031_SENSOR_STATE_STREAMING);
    if (ret) {
        dev_err(&client->dev, "failed while waiting for streaming state");
        return ret;
    }

    return 0;
}

static void isx031_stop_streaming(struct isx031 *isx031)
{
    struct i2c_client *client = isx031_get_client(isx031);
    int ret;

    TRACE_ISX031(isx031);

    ret = isx031_write_reg(isx031, ISX031_REG_MODE_SET_F_LOCK,
        ISX031_REG_VALUE_08BIT, ISX031_MODE_SET_F_APPLICATION_LOCK);
    if (ret) {
        dev_err(&client->dev, "failed to set f-lock");
        return;
    }

    ret = isx031_modify_reg(isx031, ISX031_REG_MODE_SEL, ISX031_REG_VALUE_16BIT,
        ISX031_REG_MODE_SET_F_MASK << ISX031_REG_MODE_SET_F_POS,
        ISX031_MODE_SET_F_TRANSITION_TO_STARTUP << ISX031_REG_MODE_SET_F_POS);
    if (ret) {
        dev_err(&client->dev, "failed to stop streaming");
        return;
    }

    ret = isx031_wait_for_sensor_state(isx031, ISX031_SENSOR_STATE_START_UP);
    if (ret) {
        dev_err(&client->dev, "failed while waiting for start up state");
        return;
    }
}

#endif