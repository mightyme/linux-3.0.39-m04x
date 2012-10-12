/* es305B voice processor driver
 *
 * Copyright (C)
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/firmware.h>
#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <linux/kernel.h>
#include <linux/serial_core.h>
#include <linux/es305b_soc.h>
#include <linux/es305b_param.h>

#undef LOG_TAG
#define LOG_TAG "AUD_ES305B"

#define AUD_ERR(fmt, args...) \
	printk(KERN_ERR" [AUD_ES305B] %s: " fmt, __func__ , ## args)
#define AUD_INFO(fmt, args...) \
	printk(KERN_INFO" [AUD_ES305B] %s: " fmt, __func__ , ## args)

#undef AUDIO_DEBUG
#ifdef AUDIO_DEBUG
#define AUD_DBG(fmt, args...) \
	printk(KERN_DEBUG" [AUD_ES305B] %s: " fmt, __func__ , ## args)
#else
#define AUD_DBG(fmt, args...)
#endif

#define _CMD_FIFO_USED_

#define ES305B_24M_SOC_FW "es305b_24m_soc_fw.bin"

static int es305b_i2c_read(struct i2c_client *client,
				  int bytes, void *dest);
static int es305b_i2c_write(struct i2c_client *client,
				int bytes, const void *src);
static int es305b_execute_cmdmsg(unsigned int msg);
static void es305b_soc_firmware_handler(const struct firmware *fw,void *context);
static int es305b_wakeup(void);

struct es305b_soc *es305b = NULL;

/* support at most 1024 set of commands */
#define PARAM_MAX		sizeof(char) * 6 * 1024


struct vp_ctxt {
	unsigned char *data;
	unsigned int img_size;
};

struct vp_ctxt the_vp;

static int es305b_i2c_read(struct i2c_client *client,
				  int bytes, void *dest)
{
	struct i2c_msg xfer;
	int ret;

	xfer.addr = client->addr;
	xfer.flags = I2C_M_RD;
	xfer.len = bytes;
	xfer.buf = (char *)dest;

	ret = i2c_transfer(client->adapter, &xfer, 1);
	if (ret < 0)
		goto err_read;
	if (ret != 1) {
		ret = -EIO;
		goto err_read;
	}

#ifdef AUDIO_DEBUG
	{
			int i = 0;
			for (i = 1; i < bytes; i++) {
				AUD_DBG("rx[%d] = %2x\n", i, *((char *)dest + i - 1));
				if (!(i % 4))
					AUD_DBG("\n");
			}
	}
#endif

	return 0;

err_read:
	AUD_ERR("i2c read error\n");
	return ret;
}

static int es305b_i2c_write(struct i2c_client *client, int bytes, const void *src)
{
	struct i2c_msg xfer;
	int ret;

	xfer.addr = client->addr;
	xfer.flags = 0;
	xfer.len = bytes;
	xfer.buf = (char *)src;

#ifdef AUDIO_DEBUG

	{
			int i = 0;
			for (i = 1; i < bytes; i++) {
				AUD_DBG("rx[%d] = %2x\n", i, *((char *)src + i - 1));
				if (!(i % 4))
					AUD_DBG("\n");
			}
	}
#endif

	ret = i2c_transfer(client->adapter, &xfer, 1);
	if (ret < 0)
		goto err_write;
	if (ret != 1) {
		ret = -EIO;
		goto err_write;
	}

	return 0;

err_write:
	return ret;
}

static void es305b_cold_reset(void)
{
	int err = 0;
	const struct firmware *fw;
	const char *fw_name;

	AUD_DBG("es305b_software_reset");
	fw_name = ES305B_24M_SOC_FW;

	err = request_firmware(&fw, fw_name,  es305b->dev);
	if (err) {
		AUD_ERR("Failed to load firmware %s\n", fw_name);
		return;
	}

	es305b_soc_firmware_handler(fw, (void *)es305b);

	AUD_INFO("load firmware %s\n", fw_name);
}

static int es305b_software_reset(unsigned int reset_cmd)
{
	int ret = 0;

	AUD_DBG("es305b_software_reset");
	AUD_DBG("%08x\n", reset_cmd);

	if (ES305B_RST != es305b->status) {
		unsigned long cmd = be32_to_cpu(reset_cmd);
		ret = es305b_i2c_write(es305b->client, sizeof(cmd), &cmd);
		if (!ret) {
			es305b->status = ES305B_RST;
			msleep(30);
		}
	}

	return ret;
}

static void es305b_hardware_reset(void)
{
	AUD_DBG("es305b_hardware_reset");

	/* Reset es305b chip */
	gpio_set_value(es305b->gpio_es305b_reset, 0);
	mdelay(1);
	/* Take out of reset */
	gpio_set_value(es305b->gpio_es305b_reset, 1);

	msleep(50);
}

static int es305b_sleep(void)
{
	int64_t t1, t2;
	int ret = 0;

	AUD_DBG("es305b_sleep");

	if (es305b->status ==  ES305B_SLEEP) {
		AUD_INFO("already in sleep mode");
		return 0;
	}

	t1 = ktime_to_ms(ktime_get());

	/* Put ES305B into sleep mode */
	ret = es305b_execute_cmdmsg(A200_msg_SetPowerState_Sleep);
	if (ret < 0) {
		AUD_ERR("suspend error\n");
		// return ret;
	}

	es305b->status = ES305B_SLEEP;

	msleep(20);
	/* Disable ES305B clock */
	// gpio_set_value(es305b->gpio_es305_clk, 0);
	// poweroff es305b
	// pr_device_power_off();

	t2 = ktime_to_ms(ktime_get()) - t1;

	AUD_DBG("poweroff es305b takes %lldms --\n", t2);

	return ret;
}

static int es305b_wakeup(void)
{
	int ret = 0;
	int retry = 3;

	AUD_DBG("es305b_wakeup");

	if (ES305B_NORMAL !=  es305b->status) {
		/* Enable es305b clock */
		//gpio_set_value(es305b->gpio_es305b_clk, 1);
		mdelay(1);

		gpio_set_value(es305b->gpio_es305b_wake, 0);
		msleep(120);
		do {
			ret = es305b_execute_cmdmsg((A200_msg_Sync << 16) | A200_msg_Sync_Polling);
			AUD_DBG("es305b sync ");
		} while ((ret < 0) && --retry);
		gpio_set_value(es305b->gpio_es305b_wake, 1);

		if (ret < 0) {
			AUD_ERR("failed (%d)\n", ret);
			goto wakeup_sync_err;
		}

		es305b->status = ES305B_NORMAL;
	}

wakeup_sync_err:
	return ret;
}

static int es305b_send_cmd(unsigned int cmd)
{
	int ret;
	unsigned int msg;

	if (ES305B_NORMAL == es305b->status) {
		msg = be32_to_cpu(cmd);

		ret = es305b_i2c_write(es305b->client, 4, &msg);
		if (ret < 0) {
			AUD_ERR("es305b_i2c_write (0x%.8X) error, ret = %d\n", cmd, ret);
			return -1;
		} else
			AUD_ERR("es305b_i2c_write (0x%.8X) OK\n", cmd);
	} else {
		AUD_INFO("es305b is in %d mode\n", es305b->status);
		return -1;
	}

	return 0;
}

static int es305b_get_cmd(void)
{
	int ret = -1;
	unsigned int msg;

	if (ES305B_NORMAL == es305b->status) {
		ret = es305b_i2c_read(es305b->client, 4, &msg);
		if (ret < 0) {
			AUD_ERR("es305b_i2c_read error, ret = %d\n", ret);
			return -EINVAL;
		} else {
			AUD_INFO("es305b_i2c_read (0x%.8x) OK\n", be32_to_cpu(msg));
			ret = be32_to_cpu(msg);
		}
	} else {
		AUD_INFO("es305b is in %d mode\n", es305b->status);
		ret = -EPERM;
	}

	return ret;
}

static int es305b_soc_config(enum ES305B_MODE mode)
{
	unsigned int sw_reset = 0;
	const u8 *es305b_param;
	u8 ack_buf[256] ={0,};
	int ret = 0;
	int size;

	AUD_INFO("ES305B: set es305b into %s mode\n", (mode == ES305B_SUSPEND ? "Suspend"
		: mode == ES305B_INCALL_CT_NB ? "Incall Receiver narrow band"
		: mode == ES305B_INCALL_WHS_NB ? "Incall Headset narrow band"
		: mode == ES305B_INCALL_DV_NB ? "Incall Speaker narrow band"
		: mode == ES305B_INCALL_BT ? "Incall Bluetooth"
		: mode == ES305B_VOIP_CT_NB ? "Voip Receiver narrow band"
		: mode == ES305B_VOIP_WHS_NB ? "Voip Headset narrow band"
		: mode == ES305B_VOIP_DV_NB ? "Voip Speaker narrow band"
		: mode == ES305B_VOIP_BT ? "Voip Bluetooth"
		: mode == ES305B_BT_RING ? "BT Ringtone" : "Unknown"));

	switch (mode) {
	case ES305B_SUSPEND:
		return es305b_sleep();
		break;

	case ES305B_INCALL_CT_NB:
		es305b_param = incall_ct_buf;
		size = sizeof(incall_ct_buf);
		break;
	case ES305B_INCALL_DV_NB:
		es305b_param = incall_dv_buf;
		size = sizeof(incall_dv_buf);
		break;
	case ES305B_INCALL_WHS_NB:
		es305b_param = incall_whs_buf;
		size = sizeof(incall_whs_buf);
		break;
	case ES305B_INCALL_BT:
		if(es305b->nr_bt) {
			es305b_param = incall_bt_buf;
			size = sizeof(incall_bt_buf);
		} else {
			es305b_param = incall_bt_vpoff_buf;
			size = sizeof(incall_bt_vpoff_buf);
		}
		AUD_INFO("set nr for bt %s \n", es305b->nr_bt ? "On" : "off");
		break;

	case ES305B_VOIP_CT_NB:
		es305b_param = voip_ct_buf;
		size = sizeof(voip_ct_buf);
		break;
	case ES305B_VOIP_WHS_NB:
		es305b_param = voip_whs_buf;
		size = sizeof(voip_whs_buf);
		break;
	case ES305B_VOIP_DV_NB:
		es305b_param = voip_dv_buf;
		size = sizeof(voip_dv_buf);
		break;
	case ES305B_VOIP_BT:
		if(es305b->nr_bt) {
			es305b_param = voip_bt_buf;
			size = sizeof(voip_bt_buf);
		} else {
			es305b_param = voip_bt_vpoff_buf;
			size = sizeof(voip_bt_vpoff_buf);
		}
		AUD_INFO("set nr for bt %s \n", es305b->nr_bt ? "On" : "off");
		break;

	case ES305B_BT_RING:
		es305b_param = bt_ring_buf;
		size = sizeof(bt_ring_buf);
		break;

	default:
		es305b_param = NULL;
		size = 0;
		es305b->mode = ES305B_INVALID;
		AUD_ERR("mode %d invalid\n", mode);
		break;
	}

	if (es305b->mode != ES305B_INVALID) {
#ifdef _CMD_FIFO_USED_
		int i = 0;

		while(i < size) {
			ret = es305b_i2c_write(es305b->client, (size-i) < ES305B_CMD_FIFO_DEPTH ? (size - i) : ES305B_CMD_FIFO_DEPTH, es305b_param + i);
			if (ret < 0) {
				AUD_ERR("ES305B CMD block write error!\n");
				goto err_config;
			}

			msleep(20);
			i += ES305B_CMD_FIFO_DEPTH;
		}
#else
		memset(ack_buf, 0, sizeof(ack_buf));
		ret = es305b_i2c_write(es305b->client, size, es305b_param);
		if (ret) {
			AUD_ERR("failed to es305b_i2c_write at %d line\n", __LINE__);
			goto err_config;
		}

		if (es305b->mode != ES305B_SUSPEND) {
			int i;
			ret = es305b_i2c_read(es305b->client, size, ack_buf);
			if (ret) {
				AUD_ERR("failed to es305b_i2c_read at %d line\n", __LINE__);
				goto err_config;
			}
			for (i = 0; i < size; i += 4) {
				if (ES305B_NORMAL_ACK != ack_buf[i]) {
					AUD_ERR("failed to check ack\n");
					goto err_config;
				}
			}
		}
#endif
	}
	return 0;

err_config:
	sw_reset = ((A200_msg_Reset << 16) | RESET_IMMEDIATE);
	es305b_software_reset(sw_reset);
	return ret;
}

int es305b_setmode(int mode)
{
	int ret = 0;
	int retry = 5;

	AUD_DBG("ES305B: set mode %d\n", mode);

	if (mode == ES305B_LASTMODE) {
		AUD_INFO("ES305B: set to the last setting mode.\n");
		if (ES305B_NORMAL == es305b->status)
			return ret;
		mode = es305b->mode;
	}

	mutex_lock(&es305b->es305b_mutex);
	ret = es305b_wakeup();
	if(ret < 0) {
		AUD_ERR("failed to es305b_wakeup, to cold reset on the device.\n");
		es305b_cold_reset();
		ret = es305b_wakeup();
		if (ret) {
			AUD_ERR("failed to es305b_wakeup at %d line\n", __LINE__);
			goto err_mode;
		}
	}

	do {
		ret = es305b_soc_config(mode);
		if (!ret)
			break;
	} while(--retry > 0);

	if (ret < 0)
		AUD_ERR("failed to set mode to %d \n", mode);
	else
		es305b->mode = mode;

err_mode:
	mutex_unlock(&es305b->es305b_mutex);
	return ret;
}

int es305b_execute_cmdmsg(unsigned int msg)
{
	int ret = 0;
	int retries, pass = 0;
	unsigned char msgbuf[4];
	unsigned char chkbuf[4];
	unsigned int sw_reset = 0;

	sw_reset = ((A200_msg_Reset << 16) | RESET_IMMEDIATE);

	msgbuf[0] = (msg >> 24) & 0xFF;
	msgbuf[1] = (msg >> 16) & 0xFF;
	msgbuf[2] = (msg >> 8) & 0xFF;
	msgbuf[3] = msg & 0xFF;

	retries = 3;//POLLING_RETRY_CNT;
	while (retries--) {
		ret = 0;

		mdelay(POLLING_TIMEOUT); /* use polling */
		ret = es305b_i2c_write(es305b->client, 4, msgbuf);
		if (ret < 0) {
			AUD_ERR("error %d\n", ret);
			es305b_software_reset(sw_reset);
			return ret;
		}
		AUD_DBG("es305b_execute_cmdmsg %8x", msg);

		/* We don't need to get Ack after sending out a suspend command */
		if (msg == A200_msg_SetPowerState_Sleep) {
			return ret;
		}

#if 0
		memset(chkbuf, 0xaa, sizeof(chkbuf));
		ret = es305b_i2c_read(es305b->client, 4, chkbuf);
		if (ret < 0) {
			AUD_ERR("ack-read error %d (%d retries)\n", ret, retries);
			continue;
		}

#ifdef AUDIO_DEBUG
		AUD_DBG("msgbuf[0] = %x\n", chkbuf[0]);
		AUD_DBG("msgbuf[1] = %x\n", chkbuf[1]);
		AUD_DBG("msgbuf[2] = %x\n", chkbuf[2]);
		AUD_DBG("msgbuf[3] = %x\n", chkbuf[3]);
#endif

		if (msgbuf[0] == chkbuf[0]  && msgbuf[1] == chkbuf[1] && msgbuf[2] == chkbuf[2] &&msgbuf[3] == chkbuf[3]) {
			pass = 1;
			AUD_DBG("Execute_cmd OK, %08x", msg);
			break;
		} else if (msgbuf[2] == 0xff && msgbuf[3] == 0xff) {
			AUD_ERR("illegal cmd %08x, %x, %x, %x, %x\n", msg, chkbuf[0], chkbuf[1], chkbuf[2], chkbuf[3] );
			ret = -EINVAL;
			continue;
		} else if ( msgbuf[2] == 0x00 && msgbuf[3] == 0x00 ) {
			AUD_DBG("not ready (%d retries), %x, %x, %x, %x\n", \
				retries, chkbuf[0], chkbuf[1], chkbuf[2], chkbuf[3]);
			ret = -EBUSY;
			continue;
		} else {
			AUD_DBG("cmd/ack mismatch: (%d retries left), %x, %x, %x, %x\n", \
				retries, chkbuf[0], chkbuf[1], chkbuf[2], chkbuf[3]);
			ret = -EBUSY;
			continue;
		}
#endif
	}
#if 0
	if (!pass) {
		AUD_ERR("failed execute cmd %08x (%d)\n", msg, ret);
		es305b_software_reset(sw_reset);
	}
#endif
	return ret;
}


static int es305b_suspend(struct i2c_client *client, pm_message_t mesg)
{
	AUD_DBG();

	/* vp suspend function will be dominated by in-call mode, not system state */
	/* es305b_sleep(); */

	return 0;
}

static int es305b_resume(struct i2c_client *client)
{
	AUD_DBG();

	/* vp resume function will be dominated by in-call mode, not system state */
	/* es305b_wakeup(es305b); */

	return 0;
}


static ssize_t es305b_show_property(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf);
static ssize_t es305b_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count);

#define ES305B_ATTR(_name)\
{\
    .attr = { .name = #_name, .mode = S_IRUGO | S_IWUGO},\
    .show = es305b_show_property,\
    .store = es305b_store,\
}
static struct device_attribute es305b_attrs[] = {
    ES305B_ATTR(mode),
    ES305B_ATTR(status),
    ES305B_ATTR(cmd),
    ES305B_ATTR(nr_bt),
    ES305B_ATTR(reset),
    ES305B_ATTR(version),
};

enum {
	ES305B_NR_MODE,
	ES305B_NR_STATUS,
	ES305B_NR_CMD,
	ES305B_NR_BT,
	ES305B_NR_RESET,
	ES305B_FWR_VER,
};

static ssize_t es305b_show_property(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf)
{
	int i = 0;
	ptrdiff_t off;
	struct es305b_soc *es305b = (struct es305b_soc*)dev_get_drvdata(dev);

	if(!es305b) {
		AUD_ERR("failed!!!\n");
		return -ENODEV;
	}

	off = attr - es305b_attrs;

	switch (off) {
	case ES305B_NR_MODE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", es305b->mode);
		break;

	case ES305B_NR_STATUS:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", es305b->status);
		break;

	case ES305B_NR_CMD:
		i += scnprintf(buf + i, PAGE_SIZE - i, "0x%.8X\n", es305b_get_cmd());
		break;

	case ES305B_NR_BT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", es305b->nr_bt);
		break;

	case ES305B_NR_RESET:
		i += scnprintf(buf + i, PAGE_SIZE - i, "\n");
		break;

	default:
		i += scnprintf(buf + i, PAGE_SIZE - i, "Error\n");
		break;
	}
	return i;
}

static ssize_t es305b_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	unsigned int reg,value;
	int ret = 0;
	ptrdiff_t off;
	unsigned int sw_reset = 0;
	struct es305b_soc *es305b = (struct es305b_soc*)dev_get_drvdata(dev);

	if(!es305b) {
		AUD_ERR("failed!!!\n");
		return -ENODEV;
	}

	off = attr - es305b_attrs;

	switch (off) {
	case ES305B_NR_MODE:
		if (sscanf(buf, "%d\n", &value) == 1)
			es305b_setmode(value);
		ret = count;
		break;

	case ES305B_NR_STATUS:
		if (sscanf(buf, "%d\n", &value) == 1) {
			switch(value) {
				case ES305B_NORMAL:
					es305b_wakeup();
					break;

				case ES305B_SLEEP:
					es305b_sleep();
					break;

				default:
					AUD_ERR("ES305B: wrong status %d!!\n", value);
					break;
			}
		}
		ret = count;
		break;

	case ES305B_NR_CMD:
		if (sscanf(buf, "%x %x", &reg, &value) == 2) {
			es305b_send_cmd(reg);
			es305b_send_cmd(value);
		} else if (sscanf(buf, "%x\n", &value) == 1)
			es305b_send_cmd(value);
		else
			AUD_ERR("failed > 2!!!\n");
		ret = count;
		break;

	case ES305B_NR_BT:
		if (sscanf(buf, "%x\n", &value) == 1) {
			es305b->nr_bt = !!value;
			if(es305b->mode != ES305B_INCALL_BT)
				es305b_setmode(ES305B_INCALL_BT);
			dev_info(es305b->dev, "update nr for bt %s \n", es305b->nr_bt ? "On" : "off");
		}
		ret = count;
		break;

	case ES305B_NR_RESET:
		if (sscanf(buf, "%x\n", &value) == 1) {
			switch(value) {
				case ES305B_SOFTWARE_RST:
					sw_reset = ((A200_msg_Reset << 16) | RESET_IMMEDIATE);
					es305b_software_reset(sw_reset);
					break;

				case ES305B_COLD_RST:
					es305b_cold_reset();
					break;

				case ES305B_HARDWARE_RST:
					es305b_hardware_reset();
					break;
			}
			dev_info(es305b->dev, " %s reset\n", (value == ES305B_SOFTWARE_RST) ? "soft" : \
					(value == ES305B_COLD_RST) ? "cold":"hardware");
		}
		ret = count;
		break;

	case ES305B_FWR_VER:
		ret = count;
		break;

	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int es305b_create_attrs(struct device * dev)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(es305b_attrs); i++) {
		ret = device_create_file(dev, &es305b_attrs[i]);
		if (ret)
			goto es305b_attrs_failed;
	}
	goto succeed;

es305b_attrs_failed:
	AUD_INFO("failed!!!\n");
	while (i--)
		device_remove_file(dev, &es305b_attrs[i]);

succeed:
	return ret;
}

static void es305b_destroy_atts(struct device * dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(es305b_attrs); i++)
		device_remove_file(dev, &es305b_attrs[i]);
}

static void es305b_soc_firmware_handler(const struct firmware *fw, void *context)
{
	struct es305b_soc *es305b = context;
	short cmds;
	int ret;

	es305b_wakeup();

	es305b_hardware_reset();

	cmds = be16_to_cpu(ES305B_MSG_BOOT);
	ret = es305b_i2c_write(es305b->client, sizeof(cmds), &cmds);
	if (ret) {
		AUD_ERR("failed to es305b_i2c_write\n");
		goto error_fw;
	}

	mdelay(1);
	ret = es305b_i2c_read(es305b->client, sizeof(cmds), &cmds);
	if (ret) {
		AUD_ERR("failed to es305b_i2c_read\n");
		goto error_fw;
	}
	if (ES305B_MSG_BOOT_ACK != cmds) {
		AUD_ERR("not a boot-mode ack, cmds = 0x%x\n", cmds);
		goto error_fw;
	}

#ifdef _CMD_FIFO_USED_
{
	cmds = RETRY_CNT;
	while (cmds--) {
		int remaining;
		const u8 *index;

		/* Download firmware to device*/
		remaining = fw->size / ES305B_CMD_FIFO_DEPTH;
		index = fw->data;

		AUD_INFO("ES305B: starting to load firmware ...\n");
		for (; remaining > 0; remaining--, index += ES305B_CMD_FIFO_DEPTH) {
			ret = es305b_i2c_write(es305b->client, ES305B_CMD_FIFO_DEPTH, index);
			if (ret < 0)
				break;
		}

		if (ret >= 0 && fw->size % ES305B_CMD_FIFO_DEPTH)
			ret = es305b_i2c_write(es305b->client, fw->size % ES305B_CMD_FIFO_DEPTH, index);

		if (ret == 0) {
			msleep(120); /* Delay time before issue a Sync Cmd */
			ret = es305b_execute_cmdmsg((A200_msg_Sync << 16) | A200_msg_Sync_Polling);
			if (ret < 0) {
				AUD_ERR("failed to es305b_i2c_write\n");
				continue;
			}

			break;
		}

		AUD_ERR("firmware load error %d (%d retries left)\n", ret, cmds);
	}

	if (ret < 0)
		AUD_ERR("failed to load firmware \n");
	else
		AUD_INFO("firmware load successfully \n");

}
#else
	ret = es305b_i2c_write(es305b->client, fw->size, fw->data);
	if (ret) {
		AUD_ERR("failed to es305b_i2c_write at %d line\n", __LINE__);
		goto error_fw;
	}
#endif
	/* waiting es305b initialize done */
	msleep(120);

	ret = es305b_setmode(ES305B_SUSPEND);
	if (!ret)

error_fw:
	release_firmware(fw);
}

static int __devinit es305b_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct es305b_platform_data *pdata;
	const char *fw_name;
	int ret;

	AUD_INFO();

	pdata = client->dev.platform_data;
	if (!pdata) {
		AUD_ERR("platform data is NULL\n");
		return -ENOMEM;
	}

	es305b = kzalloc(sizeof(*es305b), GFP_KERNEL);
	if (!es305b) {
		AUD_ERR("es305b is NULL\n");
		return -ENOMEM;
	}

	es305b->dev = &client->dev;
	es305b->gpio_es305b_wake = pdata->gpio_es305b_wake;
	es305b->gpio_es305b_reset = pdata->gpio_es305b_reset;
	es305b->client = client;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		AUD_ERR("i2c check functionality error\n");
		ret = -ENODEV;
		goto err_free_gpio_all;
	}

	ret = gpio_request_one(es305b->gpio_es305b_reset, GPIOF_OUT_INIT_HIGH, "es305b_rst");
	if (ret) {
		AUD_ERR("***** %s FAILED :%d *****\n", "AUD_ES305_RESET", gpio_get_value(pdata->gpio_es305b_reset));
		goto err_exit;
	} else
		AUD_INFO("***** %s OK :%d *****\n", "AUD_ES305_RESET", gpio_get_value(pdata->gpio_es305b_reset));

	ret = gpio_request_one(es305b->gpio_es305b_wake, GPIOF_OUT_INIT_HIGH, "es305b_wake");
	if (ret) {
		AUD_ERR("***** %s FAILED :%d *****\n", "AUD_ES305_WAKEUP", gpio_get_value(pdata->gpio_es305b_wake));
		goto err_free_gpio_reset;
	} else
		AUD_INFO("***** %s OK :%d *****\n", "AUD_ES305_WAKEUP", gpio_get_value(pdata->gpio_es305b_wake));

	// power on, regulator enabled
	// clk GPIO on

	mutex_init(&es305b->es305b_mutex);
	i2c_set_clientdata(client, es305b);

	fw_name = ES305B_24M_SOC_FW;
	ret = request_firmware_nowait(THIS_MODULE,
	        FW_ACTION_HOTPLUG,
	        fw_name,
	        &client->dev,
	        GFP_KERNEL | __GFP_ZERO,
	        es305b,
	        es305b_soc_firmware_handler);

	es305b_create_attrs(es305b->dev);

	AUD_INFO("ES305B: load %s\n", fw_name);

	return 0;

err_free_gpio_all:
	gpio_free(pdata->gpio_es305b_reset);

err_free_gpio_reset:
	gpio_free(pdata->gpio_es305b_reset);

err_exit:
	return ret;
}


static int es305b_i2c_remove(struct i2c_client *client)
{
	struct es305b_soc *es305b = i2c_get_clientdata(client);

	AUD_DBG();

	mutex_destroy(&es305b->es305b_mutex);
	es305b_destroy_atts(es305b->dev);
	i2c_set_clientdata(client, NULL);
	gpio_free(es305b->gpio_es305b_reset);
	gpio_free(es305b->gpio_es305b_wake);
	kfree(es305b);

	return 0;
}

/* i2c codec control layer */
static const struct i2c_device_id es305b_i2c_id[] = {
       {ES305B_I2C_NAME, 0},
       { }
};
MODULE_DEVICE_TABLE(i2c, es305b_i2c_id);

static struct i2c_driver es305b_i2c_driver = {
	.driver = {
		.name = ES305B_I2C_NAME,
		.owner = THIS_MODULE,
	},

	.probe = es305b_i2c_probe,
	.remove = es305b_i2c_remove,
	.suspend = es305b_suspend,
	.resume = es305b_resume,
	.id_table = es305b_i2c_id,
};

static int __init es305b_init(void)
{
	int ret;
	AUD_DBG();

	ret = i2c_add_driver(&es305b_i2c_driver);

	return ret;
}
module_init(es305b_init);

static void __exit es305b_deinit(void)
{
	AUD_DBG();

	i2c_del_driver(&es305b_i2c_driver);
}
module_exit(es305b_deinit);

MODULE_DESCRIPTION("Audience es305b driver");
MODULE_AUTHOR(" Corp.");
MODULE_LICENSE("GPL");
