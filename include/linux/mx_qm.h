/*
 * Touch key & led driver for meizu m040
 *
 * Copyright (C) 2012 Meizu Technology Co.Ltd, Zhuhai, China
 * Author:		
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#ifndef	__MX_QMATRIX__
#define	__MX_QMATRIX__


#define FW_VERSION     0x06
#define MX_QM_DEVICE_ID     'M'

enum mx_qm_reg {
        LED_REG0_SELECT0                	= 0x00,
        LED_REG1_SELECT1                	= 0x01,
        LED_REG2_SELECT2                	= 0x02,
        LED_REG3_FADEONTIME                = 0x03,
        LED_REG4_FULLYONTIME                = 0x04,
        LED_REG5_FADEOFFTIME                = 0x05,
        LED_REG6_FIRSTFULLYOFFTIME               		= 0x06,
        LED_REG7_SECONDFULLYOFFTIME                	= 0x07,
        LED_REG8_MAXINTENSITY                			= 0x08,
        LED_REG9_ONESHOT_MASTERINTENSITY		= 0x09,
        LED_REG10_INITIALIZATION					= 0x0A,
        LED_REG15               = 0x0F,
        
        LED_REG0_R                = 0x10,
        LED_REG1_R                = 0x11,
        LED_REG2_R                = 0x12,
        LED_REG3_R                = 0x13,
        LED_REG4_R                = 0x14,
        LED_REG5_R                = 0x15,
        LED_REG6_R                = 0x16,
        LED_REG7_R                = 0x17,
        LED_REG8_R                = 0x18,
        LED_REG9_R                = 0x19,
        LED_REG10_R               = 0x1A,
        LED_REG15_R               = 0x1F,
        
        LED_REG_LEDM0               = 0x20,
        LED_REG_LEDM1               = 0x21,
        LED_REG_LEDM2               = 0x22,
        LED_REG_LEDM3               = 0x23,
        LED_REG_LEDM4               = 0x24,
        LED_REG_LEDM5               = 0x25,
        LED_REG_LEDM6               = 0x26,
        LED_REG_LEDMAUTO            = 0x2F,
        
	QM_REG_DEVICE_ID	= 0xA0,
	QM_REG_VERSION		= 0xA1,
	QM_REG_CONTROL    	= 0xA2,
	QM_REG_STATUS    	= 0xA3,
	QM_REG_STATE    	= 0xA4,
	QM_REG_POSITION		= 0xA5,
	QM_REG_REF		= 0xA6,
	QM_REG_SIG		= 0xA7,
	QM_REG_DELTA		= 0xA8,
	QM_REG_KEY		= 0xA9,
        
	QM_REG_MAX,
};

#define LED_REG_MAX   LED_REG15
#define MXQM_REG_INVALID	(0xff)

enum mx_qm_state {
        QM_STATE_SLEEP = 0,
        QM_STATE_NORMAL = 1,
        
	QM_STATE_MAX,
};

enum mx_qm_key {
        QM_KEY_NONE = 0,
        QM_KEY_1 = 1,
        QM_KEY_2 = 2,
        QM_KEY_3 = 3,
        QM_KEY_4 = 4,
        QM_KEY_L2R = 5,
        QM_KEY_R2L = 6,
        
	QM_KEY_MAX,
};

enum mx_qm_cmd {
	QM_RESET_SENSOR = 0,
	QM_CALIBRATE = 1,

	QM_CMD_MAX,
};


/* Calibrate */
#define MX_QM_CAL_TIME    200

/* Reset */
#define MX_QM_RESET_TIME  255

struct mx_qm_platform_data {
	unsigned int gpio_wake;
	unsigned int gpio_reset;
	unsigned int gpio_irq;
};

struct mx_qm_data {
	struct device *dev;
	struct i2c_client *client;
	unsigned int irq;
	struct mutex iolock;
	unsigned int AVer;
	unsigned int BVer;
	unsigned int gpio_wake;
	unsigned int gpio_reset;
	unsigned int gpio_irq;	
	int (*i2c_readbyte)(struct i2c_client *, u8);
	int (*i2c_writebyte)(struct i2c_client *, u8, u8);
	void(*reset)(struct mx_qm_data *, int);
	void(*wakeup)(struct mx_qm_data *,int);
	int(*update)(struct mx_qm_data *);
};

#endif
