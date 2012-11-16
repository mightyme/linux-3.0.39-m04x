/*
 * Copyright (C) 2012 Meizu Technology Co.Ltd, Zhuhai, China
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
 * Modified for Crespo on August, 2010 By Samsung Electronics Co.
 * This is modified operate according to each status.
 *
 */

#include <linux/module.h>
#include <linux/random.h>
#include <linux/rsa.h>
#include <linux/rsa_pubkey.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

extern int deal_private_block(int write, unsigned offset , long len, void *buffer);

// 2M MISC AND 15M MODEM PARTITION IS PRIVATE AREA
// MISC PARTITION USE FOR UBOOT BOARD INFOMATION
// FIRST 1M MODEM DO NOT USE FOR PRIVATE ENTRY
#define BASE_PRIVATE_ENTRY_OFFSET ( 2 * 1024 * 1024 + 1024 * 1024)
#define MAX_PRIVATE_ENTRY (1024)

#define PRIVATE_ENTRY_BLOCK_SIZE (1024)
#define PRIVATE_ENTRY_SIG_SIZE (256)
#define PRIVATE_ENTRY_RANDOM_SIZE (20)

static char private_entry_buf[PRIVATE_ENTRY_BLOCK_SIZE];
static char private_entry_random[PRIVATE_ENTRY_RANDOM_SIZE];

static inline int slot_to_offset(int slot)
{
	if(slot >= MAX_PRIVATE_ENTRY || slot < 0) {
		pr_info("out of private entry range\n");
		return -1;
	}
	return BASE_PRIVATE_ENTRY_OFFSET + slot * PRIVATE_ENTRY_BLOCK_SIZE;
}

int private_entry_read(int slot, __user char *out_buf)
{
	int offset = slot_to_offset(slot);
	int err = -EINVAL;

	if(offset < 0)
		goto out;

	err = deal_private_block(0, offset, PRIVATE_ENTRY_BLOCK_SIZE, private_entry_buf);
	if (err)
		goto out;

	err = copy_to_user(out_buf, private_entry_buf, PRIVATE_ENTRY_BLOCK_SIZE);
	if (err)
		goto out;

out:
	pr_info("%s rtn code %d\n", __func__, err);
	return err;
}

int private_entry_write_prepare(int slot, __user char *random_buf)
{
	int err = 0;

	get_random_bytes(private_entry_random, PRIVATE_ENTRY_RANDOM_SIZE);

	err = copy_to_user(random_buf, private_entry_random, PRIVATE_ENTRY_RANDOM_SIZE);

	return err;
}

int private_entry_write(int slot, __user char *in_buf)
{
	int offset = slot_to_offset(slot);
	const int data_len = PRIVATE_ENTRY_BLOCK_SIZE - PRIVATE_ENTRY_SIG_SIZE;
	struct RSAPublicKey *pk;
	int err = -EINVAL;

	if(offset < 0)
		goto out;

	if(slot < 16)
		pk = &factory_rsa_pk;
	else
		pk = &remote_rsa_pk;

	err = copy_from_user(private_entry_buf, in_buf, PRIVATE_ENTRY_BLOCK_SIZE);
	if (err)
		goto out;

	err = rsa_with_sha1_verify(private_entry_random, sizeof(private_entry_random),
			pk, private_entry_buf);
	if (err) {
		pr_info("RSA verify failed\n");
		goto out;
	}

	memmove(private_entry_buf, private_entry_buf + PRIVATE_ENTRY_SIG_SIZE, data_len);

	memset(private_entry_buf + data_len,  0, PRIVATE_ENTRY_SIG_SIZE);

	err = deal_private_block(1, offset, data_len, private_entry_buf);
	if (err)
		goto out;

out:
	pr_info("%s rtn code %d\n", __func__, err);
	return err;
}

int system_data_func(int cmd , __user char *buf, int size)
{
	int offset = BASE_PRIVATE_ENTRY_OFFSET + MAX_PRIVATE_ENTRY * PRIVATE_ENTRY_BLOCK_SIZE;
	int err = -EINVAL;
	char *tmp_buf = NULL;

	if(size > 4096)
		goto out;
	if(!buf)
		goto out;

	tmp_buf = kzalloc(size, GFP_KERNEL);
	if(!tmp_buf) {
		err = -ENOMEM;
		goto out;
	}

	err = copy_from_user(tmp_buf, buf, size);
	if (err)
		goto out;

	err = deal_private_block(cmd, offset, size, tmp_buf);
	if (err)
		goto out;

	err = copy_to_user(buf, tmp_buf, size);
	if (err)
		goto out;

out:
	if(tmp_buf)
		kfree(tmp_buf);
	pr_info("%s rtn code %d\n", __func__, err);
	return err;
}

static char device_sn[256];
static char device_mac[6];
static char err_mac[6] = {0x0,0x1,0x2,0x3,0x4,0x5};
int system_part_protect = 1;

extern int meizu_set_sn(char *sn, int size);

static void init_device_sn(void)
{
	int offset = slot_to_offset(0);//sn slot 0
	uint16_t len;
	char* data = private_entry_buf + PRIVATE_ENTRY_SIG_SIZE + sizeof(len);

	deal_private_block(0, offset, PRIVATE_ENTRY_BLOCK_SIZE, private_entry_buf);
	len = *(uint16_t *)(private_entry_buf + PRIVATE_ENTRY_SIG_SIZE);

	if(!rsa_with_sha1_verify(data, len, &factory_rsa_pk, private_entry_buf)) {

		memcpy(device_sn, data, len);

		meizu_set_sn(device_sn, sizeof(device_sn));
		pr_info("@@@@ SN %s\n", device_sn);
	} else {
		pr_info("@@@@ SN rsa with sha1 verify failed!!!!\n");
	}
}

static void init_device_mac(void)
{
	int offset = slot_to_offset(1);//mac slot 1
	uint16_t len;
	char* data = private_entry_buf + PRIVATE_ENTRY_SIG_SIZE + sizeof(len);

	memcpy(device_mac, err_mac, sizeof(device_mac));

	deal_private_block(0, offset, PRIVATE_ENTRY_BLOCK_SIZE, private_entry_buf);
	len = 6;

	if(!rsa_with_sha1_verify(data, len, &factory_rsa_pk, private_entry_buf)) {

		memcpy(device_mac, data, len);

		pr_info("@@@@ MAC %02X:%02X:%02X:%02X:%02X:%02X\n",
				device_mac[0], device_mac[1], device_mac[2],
				device_mac[3], device_mac[4], device_mac[5]);
	} else {
		pr_info("@@@@ MAC rsa with sha1 verify failed!!!!\n");
	}
}

/*
 * enable define：
 * 0： None
 * 1：Root/Lock
 * 2：Unroot/Unlock
*/
#define MAGIC_NUMBER_LENGTH 16
struct system_rtx {
	unsigned int magic[MAGIC_NUMBER_LENGTH];
	unsigned int product;
	unsigned int enable;
};

unsigned int magic[MAGIC_NUMBER_LENGTH] = {	0x5A5A5A5A, 0x5A5A5A5A, 0x5A5A5A5A, 0x5A5A5A5A, 
											0x5A5A5A5A, 0x5A5A5A5A, 0x5A5A5A5A, 0x5A5A5A5A, 
											0x5A5A5A5A, 0x5A5A5A5A, 0x5A5A5A5A, 0x5A5A5A5A, 
											0x5A5A5A5A, 0x5A5A5A5A, 0x5A5A5A5A, 0x5A5A5A5A };

static void init_system_protect(void)
{
	int offset = slot_to_offset(16);//system_protect slot 16
	struct system_rtx rtx;

	deal_private_block(0, offset, PRIVATE_ENTRY_BLOCK_SIZE, private_entry_buf);
	memcpy(&rtx, private_entry_buf, sizeof(rtx));

	system_part_protect = 1;
	if(!memcmp(rtx.magic, magic, sizeof(magic))) {
		if(rtx.enable == 1)
			system_part_protect = 0;
		pr_info("SYSTEM PROTECT STATE %d\n", system_part_protect);
	} else {
		pr_info("NOT LEGAL SYSTEM PROTECT SLOT\n");
	}
}

int get_mac_form_device(unsigned char *buf)
{

	if(memcmp(device_mac, err_mac, sizeof(err_mac))) {
		memcpy(buf, device_mac, sizeof(device_mac));
		buf[0] = 0x38;
		buf[1] = 0xBC;
		buf[2] = 0x1A;
		return 0;
	}

	pr_info("%s failed\n", __func__);
	return -EINVAL;
}

int meizu_device_info_init(void)
{
	init_device_sn();
	init_device_mac();
	init_system_protect();
	return 0;
}
