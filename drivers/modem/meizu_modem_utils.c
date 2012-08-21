/*
 * Copyright (C) 2011 Samsung Electronics.
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

#include <linux/ctype.h>
#include <linux/netdevice.h>
#include <linux/platform_data/modem.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <net/ip.h>

#include "modem_prj.h"
#include "modem_utils.h"

char* format_hex_string(const unsigned char *buf, int count) 
{
	/*define max count of chars to be print*/
#define	MAXCHARS 1024
	/* CHARS_PER_LINE */
#define CPL 16
	const static char hexchar_table[] = {'0', '1', '2', '3', '4', '5', '6',
				'7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
	/*a char=2hex+a space+a printable char+(MAXCHARS+CPL-1)/CPL '\n'+'\0'*/
	static char line[4 * MAXCHARS + (MAXCHARS + CPL - 1)/CPL + 1];
	int actcount = (count < MAXCHARS) ? count : MAXCHARS;
	int index = 0;
	int i, r;

	r = actcount % CPL;
	for (i = 0; i < actcount; i++) {
		index = i/CPL*CPL*4 + i/CPL + i%CPL*3;
		line[index + 0] = hexchar_table[buf[i] >> 4]; 
		line[index + 1] = hexchar_table[buf[i] & 0x0f]; 
		line[index + 2] = ' ';

		if (r && (i >= actcount-r))
			index = i/CPL*CPL*4 + i/CPL + r*3 + i%CPL;
		else
			index = i/CPL*CPL*4 + i/CPL + CPL*3 + i%CPL;

		line[index] = isprint(buf[i]) ?  buf[i]: '.' ;
		
		if (i % CPL == CPL - 1) 
			line[++index] = '\n';
	}
	
	line[++index] = 0;

	return line;
}

struct io_device *get_iod_with_channel(struct mif_common *commons,
					unsigned channel)
{
	struct rb_node *n = commons->iodevs_tree_chan.rb_node;
	struct io_device *iodev;
	while (n) {
		iodev = rb_entry(n, struct io_device, node_chan);
		if (channel < iodev->id)
			n = n->rb_left;
		else if (channel > iodev->id)
			n = n->rb_right;
		else
			return iodev;
	}
	return NULL;
}

struct io_device *insert_iod_with_channel(struct mif_common *commons,
		unsigned channel, struct io_device *iod)
{
	struct rb_node **p = &commons->iodevs_tree_chan.rb_node;
	struct rb_node *parent = NULL;
	struct io_device *iodev;
	while (*p) {
		parent = *p;
		iodev = rb_entry(parent, struct io_device, node_chan);
		if (channel < iodev->id)
			p = &(*p)->rb_left;
		else if (channel > iodev->id)
			p = &(*p)->rb_right;
		else
			return iodev;
	}
	rb_link_node(&iod->node_chan, parent, p);
	rb_insert_color(&iod->node_chan, &commons->iodevs_tree_chan);
	return NULL;
}

void iodevs_for_each(struct mif_common *commons, action_fn action, void *args)
{
	struct io_device *iod;
	struct rb_node *node = rb_first(&commons->iodevs_tree_chan);
	for (; node; node = rb_next(node)) {
		iod = rb_entry(node, struct io_device, node_chan);
		action(iod, args);
	}
}

void iodev_netif_wake(struct io_device *iod, void *args)
{
	if (iod->io_typ == IODEV_NET && iod->ndev) {
		netif_wake_queue(iod->ndev);
		mif_info("%s\n", iod->name);
	}
}

void iodev_netif_stop(struct io_device *iod, void *args)
{
	if (iod->io_typ == IODEV_NET && iod->ndev) {
		netif_stop_queue(iod->ndev);
		mif_info("%s\n", iod->name);
	}
}

static void iodev_set_tx_link(struct io_device *iod, void *args)
{
	struct link_device *ld = (struct link_device *)args;
	if (iod->io_typ == IODEV_NET && IS_CONNECTED(iod, ld)) {
		set_current_link(iod, ld);
		mif_err("%s -> %s\n", iod->name, ld->name);
	}
}

void rawdevs_set_tx_link(struct mif_common *commons, enum modem_link link_type)
{
	struct link_device *ld = find_linkdev(commons, link_type);
	if (ld)
		iodevs_for_each(commons, iodev_set_tx_link, ld);
}

void mif_add_timer(struct timer_list *timer, unsigned long expire,
		void (*function)(unsigned long), unsigned long data)
{
	init_timer(timer);
	timer->expires = get_jiffies_64() + expire;
	timer->function = function;
	timer->data = data;
	add_timer(timer);
}

void mif_print_data(char *buf, int len)
{
	int words = len >> 4;
	int residue = len - (words << 4);
	int i;
	char *b;
	char last[80];
	char tb[8];

	/* Make the last line, if ((len % 16) > 0) */
	if (residue > 0) {
		memset(last, 0, sizeof(last));
		memset(tb, 0, sizeof(tb));
		b = buf + (words << 4);

		sprintf(last, "%04X: ", (words << 4));
		for (i = 0; i < residue; i++) {
			sprintf(tb, "%02x ", b[i]);
			strcat(last, tb);
			if ((i & 0x3) == 0x3) {
				sprintf(tb, " ");
				strcat(last, tb);
			}
		}
	}

	for (i = 0; i < words; i++) {
		b = buf + (i << 4);
		mif_err("%04X: "
			"%02x %02x %02x %02x  %02x %02x %02x %02x  "
			"%02x %02x %02x %02x  %02x %02x %02x %02x\n",
			(i << 4),
			b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7],
			b[8], b[9], b[10], b[11], b[12], b[13], b[14], b[15]);
	}

	/* Print the last line */
	if (residue > 0)
		mif_err("%s\n", last);
}

