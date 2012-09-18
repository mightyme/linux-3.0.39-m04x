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

int atdebugchannel = 0;
int atdebuglen = 0;

static int __init atdebugchannel_setup(char *args)
{
	int error;
	unsigned long val;

	error = strict_strtoul(args, 16, &val);
	if (!error)
		atdebugchannel = val;

	return error;
}
__setup("atdebugchannel=", atdebugchannel_setup);

static int __init atdebuglen_setup(char *args)
{
	int error;
	unsigned long val;

	error = strict_strtoul(args, 10, &val);
	if (!error)
		atdebuglen = val;

	return error;
}
__setup("atdebuglen=", atdebuglen_setup);

static int atdebugfunc(struct io_device *iod, const char* buf, int len)
{
	int atdebuglen = 0;

	if (iod->atdebug) {
		char *atdebugbuf;

		atdebuglen = iod->atdebug > len ? len : iod->atdebug;
		atdebugbuf = format_hex_string(buf, atdebuglen);
		pr_info("\n%pF, iod id:%d, data len:%d, data:\n%s\n",
			__builtin_return_address(0), iod->id, len, atdebugbuf);
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

static int vnet_is_opened(struct net_device *ndev)
{
	struct vnet *vnet = netdev_priv(ndev);

	return (atomic_read(&vnet->iod->opened));
}

static int get_ip_packet_sz(struct io_device *iod)
{
	struct net_device *ndev = iod->ndev;
	struct vnet *vnet = netdev_priv(ndev);
	struct sk_buff *tmp_skb = NULL;
	struct sk_buff *skb = NULL;
	struct sk_buff *skb2 = NULL;
	struct iphdr rx_ip_hdr;
	int pkt_sz;

retry:
	skb = skb_dequeue(&iod->rx_q);
	if (!skb) {
		return 0;
	}
	if (iod->atdebug)
		iod->atdebugfunc(iod, skb->data, skb->len);
	if (skb->len >= sizeof(struct iphdr)) {
		memcpy(&rx_ip_hdr, skb->data, sizeof(struct iphdr));
		skb_queue_head(&iod->rx_q, skb);
	} else {
		/*combind two skb to one, and continue*/
		skb2 = skb_dequeue(&iod->rx_q);
		if (skb2 == NULL) {
			skb_queue_head(&iod->rx_q, skb);
			return -EINVAL;
		} else {
			tmp_skb = dev_alloc_skb(skb->len + skb2->len);
			if (tmp_skb) {
				memcpy(skb_put(tmp_skb, skb->len),
							skb->data,skb->len);
				memcpy(skb_put(tmp_skb, skb2->len),
							skb->data, skb2->len);
				dev_kfree_skb_any(skb);
				dev_kfree_skb_any(skb2);
				skb_queue_head(&iod->rx_q, tmp_skb);
				goto retry;
			} else {
				skb_queue_head(&iod->rx_q, skb2);
				skb_queue_head(&iod->rx_q, skb);
				return -EINVAL;
			}
		}
	}

	if(rx_ip_hdr.ihl != 5 && rx_ip_hdr.version != 4) {
		pr_err("%s no IP packet!\n", __func__);
		pkt_sz = -EINVAL;
		return pkt_sz;
	}
	pkt_sz = ntohs(rx_ip_hdr.tot_len);
	switch(rx_ip_hdr.protocol) {
	case IPPROTO_IP:
		pr_debug("IP dummy packet", pkt_sz);
		break;
	case IPPROTO_ICMP:
		pr_debug("IP ICMP packet", pkt_sz);
		break;
	case IPPROTO_IGMP:
		pr_debug("IP IGMP packet", pkt_sz);
		break;
	case IPPROTO_IPIP:
		pr_debug("IP tunnel packet", pkt_sz);
		break;
	case IPPROTO_TCP:
		pr_debug("IP TCP packet", pkt_sz);
		break;
	case IPPROTO_UDP:
		pr_debug("IP UDP packet", pkt_sz);
		break;
	default:
		break;
	}

	return pkt_sz;
}

static void hsic_tty_data_handler(struct io_device *iod)
{
	struct tty_struct *tty = iod->tty;
	struct sk_buff *skb = NULL;
	unsigned char *buf;
	int count;

	if (!tty)
		return;
	skb = skb_dequeue(&iod->rx_q);
	while(skb) {
		if (test_bit(TTY_THROTTLED, &tty->flags))
			break;
		wake_lock_timeout(&iod->wakelock, HZ*0.5);
		count = tty_prepare_flip_string(tty, &buf, skb->len);
		if (count <= 0) {
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
				skb_queue_head(&iod->rx_q, skb);
			}
		} else {
			memcpy(buf, skb->data, count);
			if (iod->atdebug)
				iod->atdebugfunc(iod, skb->data, count);
			dev_kfree_skb_any(skb);
		}
		tty_flip_buffer_push(tty);

		skb = skb_dequeue(&iod->rx_q);
	}
}

static void hsic_net_data_handler(struct io_device *iod)
{
	struct net_device *ndev = iod->ndev;
	struct vnet *vnet = netdev_priv(ndev);
	struct sk_buff *net_skb = NULL;
	struct sk_buff *skb = NULL;
	struct sk_buff *skb2 = NULL;
	unsigned char *buf;
	int count;

	if (!vnet_is_opened(ndev)) {
		skb = skb_dequeue(&iod->rx_q);
		if (skb) {
			if (iod->atdebug)
				iod->atdebugfunc(iod, skb->data, skb->len);
			dev_kfree_skb_any(skb);
		}
		return;
	}

	if (vnet->pkt_sz <= 0)
		vnet->pkt_sz = get_ip_packet_sz(iod);

	skb = skb_dequeue(&iod->rx_q);
	while(skb) {
		wake_lock_timeout(&iod->wakelock, HZ * 0.5);
		if (vnet->pkt_sz <= 0) {
			dev_kfree_skb_any(skb);
			break;
		}
		if (vnet->skb == NULL) {
			net_skb = dev_alloc_skb(vnet->pkt_sz);
			if (net_skb == NULL) {
				pr_err("%s allocate skb err!\n", __func__);
				dev_kfree_skb_any(skb);
				break;
			} else {
				net_skb->dev = ndev;
				net_skb->protocol = htons(ETH_P_IP);
				vnet->skb = net_skb;
			}
		} else
			net_skb = vnet->skb;


		if (vnet->pkt_sz >= skb->len) {
			buf = skb_put(net_skb, skb->len);
			memcpy(buf, skb->data, skb->len);
			vnet->pkt_sz -= skb->len;
			dev_kfree_skb_any(skb);
		} else {
			buf = skb_put(net_skb, vnet->pkt_sz);
			memcpy(net_skb->data, skb->data, vnet->pkt_sz);
			skb_pull(skb, vnet->pkt_sz);
			skb_queue_head(&iod->rx_q, skb);
		}

		if (vnet->pkt_sz == 0) {
			vnet->stats.rx_packets++;
			vnet->stats.rx_bytes += net_skb->len;
			skb_reset_mac_header(net_skb);
			netif_rx(net_skb);
			vnet->skb = NULL;
			pr_debug("%s rx size=%d", __func__, net_skb->len);
			if (iod->atdebug)
				iod->atdebugfunc(iod, net_skb->data,
								net_skb->len);
			vnet->pkt_sz = get_ip_packet_sz(iod);
			if (vnet->pkt_sz <= 0)
				break;
		}

		skb = skb_dequeue(&iod->rx_q);
	}
out:
	return;
}

/* called from link device when a packet arrives for this io device */
static int recv_data_handler(struct io_device *iod,
		struct link_device *ld, const char *data, unsigned int len)
{
	struct sk_buff *skb;
	int ret = len;

	skb = rx_alloc_skb(len, GFP_ATOMIC, iod, ld);
	if (!skb) {
		mif_err("fail alloc skb (%d)\n", __LINE__);
		return -ENOMEM;
	}

	switch (iod->io_typ) {
	case IODEV_TTY:
		memcpy(skb_put(skb, len), data, len);
		skb_queue_tail(&iod->rx_q, skb);
		hsic_tty_data_handler(iod);
		break;
	case IODEV_NET:
		memcpy(skb_put(skb, len), data, len);
		skb_queue_tail(&iod->rx_q, skb);
		hsic_net_data_handler(iod);
		break;
	default:
		mif_err("wrong io_type : %d\n", iod->io_typ);
		dev_kfree_skb_any(skb);
		ret = -EINVAL;
		break;
	}

	return ret;
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
	struct vnet *vnet = netdev_priv(ndev);
	struct io_device *iod = vnet->iod;
	struct link_device *ld = get_current_link(iod);
	struct sk_buff *tmp_skb = NULL;
	int ret = 0;

	if (iod->atdebug)
		iod->atdebugfunc(iod, skb->data, skb->len);
	wake_lock_timeout(&iod->wakelock, HZ * 0.5);
	tmp_skb = skb_clone(skb, GFP_ATOMIC);
	if (!tmp_skb) {
		mif_err("fail alloc tmp_skb (%d)\n", __LINE__);
		return -ENOMEM;
	}
	memcpy(tmp_skb->data, skb->data, skb->len);
	skbpriv(tmp_skb)->iod = iod;
	skbpriv(tmp_skb)->ld = ld;
	ret = ld->send(ld, iod, tmp_skb);
	if (ret < 0) {
		dev_kfree_skb_any(tmp_skb);
		ret = NETDEV_TX_BUSY;
	} else {
		if(skb->protocol == htons(ETH_P_IP)) {
			ndev->stats.tx_packets++;
			ndev->stats.tx_bytes += skb->len;
		}
	}
	dev_kfree_skb_any(skb);
	ret = NETDEV_TX_OK;

	return ret;
}

static struct net_device_stats *vnet_get_stats(struct net_device *dev)
{
	struct vnet *vnet = netdev_priv(dev);

	return &vnet->stats;
}

static struct net_device_ops vnet_ops = {
	.ndo_open       = vnet_open,
	.ndo_stop       = vnet_stop,
	.ndo_get_stats  = vnet_get_stats,
	.ndo_start_xmit = vnet_xmit,
};

static void vnet_setup(struct net_device *ndev)
{
	ndev->mtu             = ETH_DATA_LEN;
	ndev->type            = ARPHRD_NONE;
	ndev->flags           = IFF_POINTOPOINT | IFF_NOARP;
	ndev->features        = 0;
	ndev->addr_len        = 0;
	ndev->destructor      = free_netdev;
	ndev->netdev_ops      = &vnet_ops;
	ndev->tx_queue_len    = 1000;
	ndev->watchdog_timeo  = 20 * HZ;
	ndev->hard_header_len = 0;
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
		skb_queue_purge(&iod->rx_q);
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

static int
modem_tty_write(struct tty_struct *tty, const unsigned char *buf, int count)
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
	wake_lock_timeout(&iod->wakelock, HZ * 0.5);

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

	skb_queue_purge(&iod->rx_q);
}

static struct tty_operations tty_ops = {
	.open       = modem_tty_open,
	.close      = modem_tty_close,
	.write      = modem_tty_write,
	.throttle   = modem_tty_throttle,
	.write_room = modem_tty_write_room,
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
	tty_driver->name         = "ttyACM";
	tty_driver->type         = TTY_DRIVER_TYPE_SERIAL;
	tty_driver->major        = 0;
	tty_driver->owner        = THIS_MODULE;
	tty_driver->subtype      = SERIAL_TYPE_NORMAL;
	tty_driver->minor_start  = 0;
	tty_driver->driver_name  = "tty_driver";
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

	iod->recv = recv_data_handler;
	switch (iod->io_typ) {
	case IODEV_TTY:
		skb_queue_head_init(&iod->rx_q);
		/*INIT_DELAYED_WORK(&iod->rx_work, rx_iodev_work);*/
		iod->atdebugfunc = atdebugfunc;
		if (atdebugchannel & (0x1 << iod->id))
			if (atdebuglen)
				iod->atdebug = atdebuglen;
			else
				iod->atdebug = 255;
		else
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
		skb_queue_head_init(&iod->rx_q);
		iod->atdebugfunc = atdebugfunc;
		if (atdebugchannel & (0x1 << iod->id))
			if (atdebuglen)
				iod->atdebug = atdebuglen;
			else
				iod->atdebug = 255;
		else
			iod->atdebug = 0;
		iod->ndev = alloc_netdev(sizeof(struct vnet), iod->name,
			vnet_setup);
		if (!iod->ndev) {
			mif_err("failed to alloc netdev\n");
			return -ENOMEM;
		}
		ret = register_netdev(iod->ndev);
		if (ret)
			free_netdev(iod->ndev);

		dev_set_drvdata(&iod->ndev->dev, iod);
		ret = device_create_file(&iod->ndev->dev, &attr_atdebug);
		if (ret)
			mif_err("failed to create sysfs file : %s\n",
					iod->name);
		vnet = netdev_priv(iod->ndev);
		mif_debug("(vnet:0x%p)\n", vnet);
		vnet->pkt_sz = 0;
		vnet->iod = iod;
		vnet->skb = NULL;
		break;
	default:
		mif_err("wrong io_type : %d\n", iod->io_typ);
		return -EINVAL;
	}

	mif_debug("%s(%d) : init_io_device() done : %d\n",
				iod->name, iod->io_typ, ret);
	return ret;
}
