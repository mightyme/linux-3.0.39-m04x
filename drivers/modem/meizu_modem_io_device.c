/* /drivers/modem/meizu_ipc_io_device.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2010 Samsung Electronics.
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

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/if_arp.h>
#include <linux/ip.h>
#include <linux/if_ether.h>
#include <linux/etherdevice.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>

#include <linux/platform_data/modem.h>
#include "modem_prj.h"
#include "modem_utils.h"

static DEFINE_MUTEX(modem_tty_lock);

static int atdebugfunc(struct io_device *iod, const char* buf, int len)
{
	int atdebuglen = 0;

	if (iod->atdebug) {
		char *atdebugbuf;
		
		atdebuglen = iod->atdebug > len ? len : iod->atdebug;
		atdebugbuf = format_hex_string(buf, atdebuglen);
		pr_info("%pF, iod id:%d, data:\n%s\n", __builtin_return_address(0),
				iod->id, atdebugbuf);
	}
		
	return atdebuglen;
}

static ssize_t show_atdebug(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct io_device *iod = dev_get_drvdata(dev);
	char *p = buf;
	int atdebug;

	atdebug = iod->atdebug;
	p += sprintf(buf, "iod id:%d, atdebug:%d\n", iod->id, atdebug);

	return p - buf;
}

static ssize_t store_atdebug(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long atdebug;
	struct io_device *iod = dev_get_drvdata(dev);

	ret = strict_strtoul(buf, 10, &atdebug);
	if (ret)
		return count;

	iod->atdebug = atdebug;

	return count;
}

static struct device_attribute attr_atdebug =
	__ATTR(atdebug, S_IRUGO | S_IWUSR, show_atdebug, store_atdebug);

static void modem_tty_notify(struct io_device *iod)
{
	unsigned char *buf;
	int count;
	struct sk_buff *skb = NULL;
	struct tty_struct *tty = iod->tty;
	
	if (!tty)
		return;
	
	skb = skb_dequeue(&iod->sk_rx_q);

	while(skb) {
		if (test_bit(TTY_THROTTLED, &tty->flags))
			break;
		wake_lock_timeout(&iod->wakelock, HZ*0.5);

		count = tty_prepare_flip_string(tty, &buf, skb->len);

		if (count == 0) {
			mif_err("%s:tty buffer avail size=%d\n",
							__func__, count);
			break;
		}

		if (skb->len > count) {
			mif_err("skb->len %d > count %d\n", skb->len, count);
			memcpy(buf, skb->data, count);
			if (iod->atdebug)
				iod->atdebugfunc(iod, skb->data, count);
			skb_pull(skb, count);
			if (skb->len) {
				mif_debug("queue-head, skb->len = %d\n",
						skb->len);
				skb_queue_head(&iod->sk_rx_q, skb);
			}
		} else {
			memcpy(buf, skb->data, count);
			if (iod->atdebug)
				iod->atdebugfunc(iod, skb->data, count);
			dev_kfree_skb_any(skb);
		}

		tty_flip_buffer_push(tty);
		
		skb = skb_dequeue(&iod->sk_rx_q);
	}
}

/* called from link device when a packet arrives for this io device */
static int io_dev_recv_data_from_link_dev(struct io_device *iod,
		struct link_device *ld, const char *data, unsigned int len)
{
	struct sk_buff *skb;

	skb = rx_alloc_skb(len, GFP_ATOMIC, iod, ld);
	if (!skb) {
		mif_err("fail alloc skb (%d)\n", __LINE__);
		return -ENOMEM;
	}

	memcpy(skb_put(skb, len), data, len);
	skb_queue_tail(&iod->sk_rx_q, skb);

	modem_tty_notify(iod);

	wake_up(&iod->wq);

	return len;
}

static int vnet_open(struct net_device *ndev)
{
	struct vnet *vnet = netdev_priv(ndev);
	netif_start_queue(ndev);
	atomic_inc(&vnet->iod->opened);
	return 0;
}

static int vnet_stop(struct net_device *ndev)
{
	struct vnet *vnet = netdev_priv(ndev);
	atomic_dec(&vnet->iod->opened);
	netif_stop_queue(ndev);
	return 0;
}

static int vnet_xmit(struct sk_buff *skb, struct net_device *ndev)
{
#if 0
	int ret = 0;
	int headroom = 0;
	int tailroom = 0;
	struct sk_buff *skb_new = NULL;
	struct vnet *vnet = netdev_priv(ndev);
	struct io_device *iod = vnet->iod;
	struct link_device *ld = get_current_link(iod);
	struct raw_hdr hd;

	/* When use `handover' with Network Bridge,
	 * user -> TCP/IP(kernel) -> bridge device -> TCP/IP(kernel) -> this.
	 *
	 * We remove the one ethernet header of skb before using skb->len,
	 * because the skb has two ethernet headers.
	 */
	if (iod->use_handover) {
		if (iod->id >= PSD_DATA_CHID_BEGIN &&
			iod->id <= PSD_DATA_CHID_END)
			skb_pull(skb, sizeof(struct ethhdr));
	}

	hd.len = skb->len + sizeof(hd);
	hd.control = 0;
	hd.channel = iod->id & 0x1F;

	headroom = sizeof(hd) + sizeof(hdlc_start);
	tailroom = sizeof(hdlc_end);
	if (ld->aligned)
		tailroom += MAX_LINK_PADDING_SIZE;
	if (skb_headroom(skb) < headroom || skb_tailroom(skb) < tailroom) {
		skb_new = skb_copy_expand(skb, headroom, tailroom, GFP_ATOMIC);
		/* skb_copy_expand success or not, free old skb from caller */
		dev_kfree_skb_any(skb);
		if (!skb_new)
			return -ENOMEM;
	} else
		skb_new = skb;

	memcpy(skb_push(skb_new, sizeof(hd)), &hd, sizeof(hd));
	memcpy(skb_push(skb_new, sizeof(hdlc_start)), hdlc_start,
				sizeof(hdlc_start));
	memcpy(skb_put(skb_new, sizeof(hdlc_end)), hdlc_end, sizeof(hdlc_end));
	skb_put(skb_new, calc_padding_size(iod, ld,  skb_new->len));

	skbpriv(skb_new)->iod = iod;
	skbpriv(skb_new)->ld = ld;

	ret = ld->send(ld, iod, skb_new);
	if (ret < 0) {
		netif_stop_queue(ndev);
		dev_kfree_skb_any(skb_new);
		return NETDEV_TX_BUSY;
	}

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += skb->len;
#endif	
	return NETDEV_TX_OK;
}

static struct net_device_ops vnet_ops = {
	.ndo_open = vnet_open,
	.ndo_stop = vnet_stop,
	.ndo_start_xmit = vnet_xmit,
};

static void vnet_setup(struct net_device *ndev)
{
	ndev->netdev_ops = &vnet_ops;
	ndev->type = ARPHRD_PPP;
	ndev->flags = IFF_POINTOPOINT | IFF_NOARP | IFF_MULTICAST;
	ndev->addr_len = 0;
	ndev->hard_header_len = 0;
	ndev->tx_queue_len = 1000;
	ndev->mtu = ETH_DATA_LEN;
	ndev->watchdog_timeo = 5 * HZ;
}

static void vnet_setup_ether(struct net_device *ndev)
{
	ndev->netdev_ops = &vnet_ops;
	ndev->type = ARPHRD_ETHER;
	ndev->flags = IFF_POINTOPOINT | IFF_NOARP | IFF_MULTICAST | IFF_SLAVE;
	ndev->addr_len = ETH_ALEN;
	random_ether_addr(ndev->dev_addr);
	ndev->hard_header_len = 0;
	ndev->tx_queue_len = 1000;
	ndev->mtu = ETH_DATA_LEN;
	ndev->watchdog_timeo = 5 * HZ;
}

static int modem_tty_open(struct tty_struct *tty, struct file *f)
{
	struct modem_ctl *modemctl = tty->driver->driver_state;
	struct mif_common *commons;
	struct link_device *ld;
	struct io_device *iod;
	int ret = 0;
	
	iod = get_iod_with_channel(&modemctl->commons, tty->index);
	commons = &iod->mc->commons;

	tty->driver_data = iod;
	iod->tty = tty;
	
	mutex_lock(&modem_tty_lock);

	if (atomic_inc_and_test(&iod->opened))
		tty->low_latency = 1;

	mutex_unlock(&modem_tty_lock);

	list_for_each_entry(ld, &commons->link_dev_list, list) {
		if (IS_CONNECTED(iod, ld) && ld->init_comm) {
			ret = ld->init_comm(ld, iod);
			if (ret < 0) {
				mif_err("%s: init_comm error: %d\n",
						ld->name, ret);
				return ret;
			}
		}
	}
	
	return ret;
}

static void modem_tty_close(struct tty_struct *tty, struct file *f)
{
	struct io_device *iod = tty->driver_data;
	struct mif_common *commons = &iod->mc->commons;
	struct link_device *ld;

	mutex_lock(&modem_tty_lock);
	atomic_dec(&iod->opened);
	if (atomic_read(&iod->opened) == 0) {
		tty->driver_data = NULL;
		mif_debug("iod = %s\n", iod->name);
		skb_queue_purge(&iod->sk_rx_q);
		list_for_each_entry(ld, &commons->link_dev_list, list) {
			if (IS_CONNECTED(iod, ld) && ld->terminate_comm)
				ld->terminate_comm(ld, iod);
		}
	}
	mutex_unlock(&modem_tty_lock);
}

static int modem_tty_write_room(struct tty_struct *tty)
{
	int ret = 16 * 1024;

	return ret;
}

static int modem_tty_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
	struct io_device *iod = tty->driver_data;
	struct link_device *ld = get_current_link(iod);
	struct sk_buff *skb;
	int err;
	size_t tx_size;

	skb = alloc_skb(count, GFP_KERNEL);
	if (!skb) {
		mif_err("fail alloc skb (%d)\n", __LINE__);
		return -ENOMEM;
	}
	wake_lock_timeout(&iod->wakelock, HZ*0.5);

	memcpy(skb_put(skb, count), buf, count);
	
	if (iod->atdebug) {
		char *atdebugbuf;
		int atdebuglen = iod->atdebug > count ? count : iod->atdebug;

		atdebugbuf = format_hex_string(skb->data, atdebuglen);
		pr_info("%s, iod id:%d, data:\n%s\n", __func__,
				iod->id, atdebugbuf);
	}

	tx_size = skb->len;

	skbpriv(skb)->iod = iod;
	skbpriv(skb)->ld = ld;

	err = ld->send(ld, iod, skb);
	if (err < 0) {
		dev_kfree_skb_any(skb);
		return err;
	}
	if (err != tx_size)
		mif_debug("WARNNING: wrong tx size: %s, format=%d "
			"count=%d, tx_size=%d, return_size=%d",
			iod->name, iod->format, count, tx_size, err);
	count = err;

	return count;
}

static void modem_tty_throttle(struct tty_struct *tty)
{
	struct io_device *iod = tty->driver_data;
		
	skb_queue_purge(&iod->sk_rx_q);
}

static struct tty_operations tty_ops = {
	.open       = modem_tty_open,
	.close      = modem_tty_close,
	.write      = modem_tty_write,
	.write_room = modem_tty_write_room,
	.throttle   = modem_tty_throttle,
};

int modem_tty_driver_init(struct modem_ctl *modemctl)
{
	struct tty_driver *tty_driver;
	int ret;

	tty_driver = alloc_tty_driver(4);
	if (tty_driver == 0) {
		pr_err("%s alloc_tty_driver -ENOMEM!!\n", __func__);
		return -ENOMEM;
	}

	tty_driver->owner = THIS_MODULE;
	tty_driver->driver_name = "tty_driver";
	tty_driver->name = "ttyACM";
	tty_driver->major = 0;
	tty_driver->minor_start = 0;
	tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	tty_driver->subtype = SERIAL_TYPE_NORMAL;
	tty_driver->init_termios = tty_std_termios;
	tty_driver->init_termios.c_iflag = 0;
	tty_driver->init_termios.c_oflag = 0;
	tty_driver->init_termios.c_cflag = B4000000 | CS8 | CREAD;
	tty_driver->init_termios.c_lflag = 0;
	tty_driver->flags = TTY_DRIVER_RESET_TERMIOS |
		TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	
	tty_set_operations(tty_driver, &tty_ops);

	ret = tty_register_driver(tty_driver);
	if (ret) {
		pr_err("%s error!!\n", __func__);
		return ret;
	}

	tty_driver->driver_state = modemctl;
	modemctl->tty_driver = tty_driver;
	
	return 0;
}

int meizu_ipc_init_io_device(struct io_device *iod)
{
	int ret = 0;
	struct vnet *vnet;

	/* Get data from link device */
	iod->recv = io_dev_recv_data_from_link_dev;

	/* Register misc or net device */
	switch (iod->io_typ) {
	case IODEV_TTY:
		init_waitqueue_head(&iod->wq);
		skb_queue_head_init(&iod->sk_rx_q);
		/*INIT_DELAYED_WORK(&iod->rx_work, rx_iodev_work);*/

		iod->atdebugfunc = atdebugfunc;
		iod->atdebug = 0;
		iod->ttydev = tty_register_device
			(iod->mc->tty_driver, iod->id, NULL);
		
		dev_set_drvdata(iod->ttydev, iod);
		ret = device_create_file(iod->ttydev, &attr_atdebug);
		if (ret)
			mif_err("failed to create sysfs file : %s\n",
					iod->name);
		break;

	case IODEV_NET:
		skb_queue_head_init(&iod->sk_rx_q);
		if (iod->use_handover)
			iod->ndev = alloc_netdev(0, iod->name,
						vnet_setup_ether);
		else
			iod->ndev = alloc_netdev(0, iod->name, vnet_setup);

		if (!iod->ndev) {
			mif_err("failed to alloc netdev\n");
			return -ENOMEM;
		}

		ret = register_netdev(iod->ndev);
		if (ret)
			free_netdev(iod->ndev);

		mif_debug("(iod:0x%p)\n", iod);
		vnet = netdev_priv(iod->ndev);
		mif_debug("(vnet:0x%p)\n", vnet);
		vnet->iod = iod;

		break;

	default:
		mif_err("wrong io_type : %d\n", iod->io_typ);
		return -EINVAL;
	}

	mif_debug("%s(%d) : init_io_device() done : %d\n",
				iod->name, iod->io_typ, ret);
	return ret;
}

