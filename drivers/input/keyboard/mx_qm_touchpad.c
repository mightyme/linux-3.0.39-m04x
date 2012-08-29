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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <plat/gpio-cfg.h>
#include <mach/gpio-m040.h>
#include <linux/firmware.h>
#include	<linux/mx_qm.h>

//#define	__POLL_SENSOR__DATA__
//#define	__TEST_TOCHPAP_DELTA__

#define RESET_COLD	1
#define	RESET_SOFT	0

#define	DETECT_POS_INTERVAL		(10)

#define	MAX_REC_POS_SIZE		(64)
#define	MAX_REC_POS_USED		(16)
#define	MAX_REC_POS_SILIDER	(64)


#define	KEY_INDEX_BACK	0
#define	KEY_INDEX_HOME	1
#define	KEY_INDEX_MENU	2
#define	KEY_INDEX_3	3
#define	KEY_INDEX_4	4
#define	KEY_INDEX_POWER	5


/* MX_QM support up to pos 0 -255 */
static const unsigned short mx_qm_keycode[] = {
	KEY_BACK, KEY_HOME, KEY_MENU,KEY_F23,KEY_F24,KEY_POWER
};

struct mx_qm_touch {
	 struct mx_qm_data *data;
	struct input_dev *input;
	int irq;			/* irq issued by device		*/
	unsigned short keycodes[ARRAY_SIZE(mx_qm_keycode)];
	u8 last_key;
	u8 keys_press;
	u8 pos;
	struct delayed_work pos_work;
	unsigned char  precpos;
	unsigned char rec_pos[MAX_REC_POS_SIZE];
};


void qm_touch_report_pos(struct input_dev *dev, int pos,int press)
{
	pr_debug("%s:Pos = %d  \n",__func__,pos);
	input_report_abs(dev, ABS_X, pos);
	if( press )
		input_sync(dev);
}

void qm_touch_report_key(struct input_dev *dev, unsigned int code, int value)
{
	pr_debug("%s:KeyCode = %d  S = %d  \n",__func__,code,value);
	input_report_key(dev, code, value);
	input_sync(dev);
}


//#define	KEY_DIV		(256/4)
//#define	KEY_DIV		(255/3)
#define	KEY_DIV		(256/ARRY_SIZE(mx_qm_keycode))
unsigned short qm_touch_cal_key(struct input_dev *dev, unsigned char pos)
{
	unsigned short key;
	unsigned short * keycodes;
	unsigned short postion;

#if 0
	postion = pos*(dev->keycodemax-1) /256;
	if(postion >= (dev->keycodemax-1))
		postion = dev->keycodemax - 2;	
#else
	postion = pos*3 /256;
	if(postion >= 3)
		postion = 2;	
#endif

	keycodes = (unsigned short *)dev->keycode;
	key = *(keycodes + postion);
	
	pr_debug("%s:KeyCode = %d  Pos = %d (%d) \n",__func__,key,postion,pos);

	return key;
}


#define ABS(x)		((x) < 0 ? (-x) : (x))
unsigned short qm_touch_cal_key2(struct mx_qm_touch	*touch)
{
	//struct mx_qm_data * mx = touch->data;
	//struct i2c_client *client = mx->client;
	struct input_dev *input = touch->input;
	
	int cal_buf[MAX_REC_POS_SIZE];
	int i;
	int j;
	int res;
	int dir0,dir1;
	unsigned char keypos;
	
	pr_debug("%s:ppos = %d\n",__func__, touch->precpos);
	for(i= 0;i < sizeof(touch->rec_pos) ;i ++)
	{
		pr_debug("%.3d  ", touch->rec_pos[i] );			
	}
	pr_debug( "\n");

	j = touch->precpos;
	memset(cal_buf,0,sizeof(cal_buf));
	for(i= 0;i < MAX_REC_POS_USED  ;i ++)
	{	
		if(j == 0)		
			j = MAX_REC_POS_SIZE-1;
		else
			j --;
		cal_buf[i] = touch->rec_pos[j];
		pr_debug("%.2d  ", cal_buf[i] );		
	}

	keypos = cal_buf[0];

	pr_debug( "\n");
	dir0 = 0;
	dir1 = 0;
	for(i= 0;i < MAX_REC_POS_USED-1  ;i ++)
	{	
		if( cal_buf[i] > cal_buf[i+1] )
			dir1 ++;
		else if( cal_buf[i] < cal_buf[i+1] )
			dir0 --;
		
		cal_buf[i] = cal_buf[i] - cal_buf[i+1];
		pr_debug("%.2d  ", cal_buf[i] );		
	}
	pr_debug( "\ndir0=%d dir1=%d \n",dir0,dir1);
	j = 0;
	for(i= 0;i < MAX_REC_POS_USED  ;i ++)
	{	
		if( cal_buf[i] != 0)
			cal_buf[j++] = cal_buf[i];	
	}
	
	if( j )	 j--;
	for(i= j;i < MAX_REC_POS_USED  ;i ++)
	{	
		cal_buf[i] = 0;				
	}
	pr_debug( "\n");
	res = 0;
	for(i= 0;i < MAX_REC_POS_USED ;i ++)
	{
		res += cal_buf[i];
		pr_debug("%d  ", cal_buf[i] );			
	}
	pr_debug( "\n");

	pr_debug("%s:res = %d\n",__func__, res);

	if( ABS( res ) < MAX_REC_POS_SILIDER )
	{
		touch->last_key = qm_touch_cal_key(input,keypos);
	}
	else
	{
		if(res < 0)
		{
			if(dir1 > 3)
				touch->last_key = qm_touch_cal_key(input,keypos);
			else
				touch->last_key = KEY_POWER;
		}
		else
		{
			if(dir0 < 3)
				touch->last_key = qm_touch_cal_key(input,keypos);
			else
				touch->last_key = KEY_HOME;
		}
	}			
	
	qm_touch_report_key(input, touch->last_key, 1);
	qm_touch_report_key(input, touch->last_key, 0);
			
	return 0;
}


unsigned short qm_touch_get_key(struct mx_qm_touch	*touch)
{
	struct mx_qm_data * mx = touch->data;
	struct i2c_client *client = mx->client;
	struct input_dev *input = touch->input;
	unsigned short * keycodes;
	int key;
	int ret = 0;

	keycodes = (unsigned short *)input->keycode;

	key = mx->i2c_readbyte(client, QM_REG_KEY);

	pr_debug("%s:%.2d  ", __func__,key );		

	switch( key )
	{
		case QM_KEY_1:
		case QM_KEY_M2L:
			key = keycodes[KEY_INDEX_BACK];
			break;
			
		case QM_KEY_2:
		case QM_KEY_L2M:
			key = keycodes[KEY_INDEX_HOME];
			break;
			
		case QM_KEY_3:
		case QM_KEY_R2M:
			key = keycodes[KEY_INDEX_HOME];
			break;
			
		case QM_KEY_4:
		case QM_KEY_M2R:
			key = keycodes[KEY_INDEX_MENU];
			break;
			
		case QM_KEY_R2L:
			key = keycodes[KEY_INDEX_3];
			break;
			
		case QM_KEY_L2R:
			key = keycodes[KEY_INDEX_4];
			break;
			
		case QM_KEY_NONE:
		default:
			ret = -1;
			break;
	}


	touch->last_key =  key;

	if(ret == 0)
	{
		qm_touch_report_key(input, touch->last_key, 1);
		qm_touch_report_key(input, touch->last_key, 0);
		pr_info("%s:%.2d  ", __func__,key );	
	}		
	

	return key;
}

static void get_slider_position_func(struct work_struct *work)
{
	struct mx_qm_touch	*touch =
		container_of(work, struct mx_qm_touch,pos_work.work);
	struct mx_qm_data * mx = touch->data;
	struct i2c_client *client = mx->client;
	struct input_dev *input = touch->input;
	
	u8 new_key,pos;
	u8 state;
		
	/* Read the detected status register, thus clearing interrupt */
	state = mx->i2c_readbyte(client, QM_REG_STATE);
		
	if( mx->poll )
	{
		if(state == 0)
		{
			if(touch->keys_press == 1 )
			{
				//qm_touch_report_key(input, touch->last_key, 1);
				//qm_touch_report_key(input, touch->last_key, 0);

				//qm_touch_cal_key2(touch);
				qm_touch_get_key(touch);
			}

			schedule_delayed_work(&touch->pos_work, msecs_to_jiffies(DETECT_POS_INTERVAL));
			touch->keys_press = 0;
			return;
		}		
		if( touch->keys_press !=  state)
		{
			touch->precpos = 0;
			memset(touch->rec_pos,0,sizeof(touch->rec_pos));
		}
		touch->keys_press = state;
	}

	/* Read which key changed */
	pos = mx->i2c_readbyte(client, QM_REG_POSITION);
	touch->rec_pos[touch->precpos++] = pos;
	if( touch->precpos >= sizeof(touch->rec_pos) )
		touch->precpos = 0;

	dev_dbg(&client->dev, "pos = %d\n", pos);

	if( touch->pos != pos )
		qm_touch_report_pos(touch->input,pos,1);
	touch->pos = pos;	
	
	//new_key = qm_touch_cal_key(input,pos);
	
	//if (data->last_key != new_key)
	//	qm_touch_report_key(input, data->last_key, data->keys_press);

	//touch->last_key = new_key;
	
#ifdef __TEST_TOCHPAP_DELTA__
	{
		u16 ref[4],sig[4],delta[4];	
		mx->i2c_readbuf(client, QM_REG_REF,8,ref);
		dev_info(&client->dev, "ref = 0x%.4X  0x%.4X  0x%.4X  0x%.4X  \n", ref[0],ref[1],ref[2],ref[3]);
		mx->i2c_readbuf(client, QM_REG_SIG,8,sig);
		dev_info(&client->dev, "sig = 0x%.4X  0x%.4X  0x%.4X  0x%.4X  \n", sig[0],sig[1],sig[2],sig[3]);
		mx->i2c_readbuf(client, QM_REG_DELTA,8,delta);
		dev_info(&client->dev, "delta = 0x%.4X  0x%.4X  0x%.4X  0x%.4X  \n", delta[0],delta[1],delta[2],delta[3]);
		dev_info(&client->dev, "pos = %d\n", pos);
	}			
#endif	

	schedule_delayed_work(&touch->pos_work, msecs_to_jiffies(DETECT_POS_INTERVAL));
}

static irqreturn_t mx_qm_irq_handler(int irq, void *dev_id)
{
	struct mx_qm_touch *touch = dev_id;	
	struct mx_qm_data * mx = touch->data;
	struct i2c_client *client = mx->client;
	struct input_dev *input = touch->input;
	
	u8 new_key,pos;
	u8 state;		
		
	/* Read the detected status register, thus clearing interrupt */
	state = mx->i2c_readbyte(client, QM_REG_STATE);

	if( state )		
	{
		/* Read which key changed */
		pos = mx->i2c_readbyte(client, QM_REG_POSITION);
		touch->rec_pos[touch->precpos++] = pos;
		if( touch->precpos >= sizeof(touch->rec_pos) )
			touch->precpos = 0;

		dev_dbg(&client->dev, "pos = %d\n", pos);

		if( touch->pos != pos )
			qm_touch_report_pos(touch->input,pos,0);
		touch->pos = pos;	
		
#ifdef __TEST_TOCHPAP_DELTA__
		{
			u16 ref[4],sig[4],delta[4];	
			mx->i2c_readbuf(client, QM_REG_REF,8,ref);
			dev_info(&client->dev, "ref = 0x%.4X  0x%.4X  0x%.4X  0x%.4X  \n", ref[0],ref[1],ref[2],ref[3]);
			mx->i2c_readbuf(client, QM_REG_SIG,8,sig);
			dev_info(&client->dev, "sig = 0x%.4X  0x%.4X  0x%.4X  0x%.4X  \n", sig[0],sig[1],sig[2],sig[3]);
			mx->i2c_readbuf(client, QM_REG_DELTA,8,delta);
			dev_info(&client->dev, "delta = 0x%.4X  0x%.4X  0x%.4X  0x%.4X  \n", delta[0],delta[1],delta[2],delta[3]);
			dev_info(&client->dev, "pos = %d\n", pos);
		}			
#endif	
		qm_touch_get_key(touch);
	}
	else
	{		
		pos = mx->i2c_readbyte(client, QM_REG_POSITION);
		qm_touch_report_pos(touch->input,pos,1);
		qm_touch_get_key(touch);
	}
	
	return IRQ_HANDLED;
}

static int __devinit mx_qm_touch_probe(struct platform_device *pdev)
{
	struct mx_qm_data *data = dev_get_drvdata(pdev->dev.parent);
	struct mx_qm_platform_data *pdata = dev_get_platdata(data->dev);
	struct i2c_client *client;
	struct mx_qm_touch*touch;
	struct input_dev *input;

	int i;
	int err;
	pr_debug("%s:++\n",__func__);

	 client = data->client;
	 if (!client->irq) {
		 dev_err(&client->dev, "please assign the irq to this device\n");
		 return -EINVAL;
	 }
 
	 touch = kzalloc(sizeof(struct mx_qm_touch), GFP_KERNEL);
	 input = input_allocate_device();
	 if (!touch || !input) {
		 dev_err(&client->dev, "insufficient memory\n");
		 err = -ENOMEM;
		 goto err_free_mem;
	 }
	 
	platform_set_drvdata(pdev, touch);

	 touch->data = data;
	 touch->input = input;
	 touch->irq = data->irq;//client->irq;
	 input->name = "mx-touch-keypad";
	 input->dev.parent = &client->dev;
	 input->id.bustype = BUS_I2C;
 
	 /* Add the keycode */
	 input->keycode = touch->keycodes;
	 input->keycodesize = sizeof(touch->keycodes[0]);
	 input->keycodemax = ARRAY_SIZE(mx_qm_keycode);
 
	 __set_bit(EV_KEY, input->evbit);
 
	 for (i = 0; i < ARRAY_SIZE(mx_qm_keycode); i++) {
		 touch->keycodes[i] = mx_qm_keycode[i];
		 __set_bit(mx_qm_keycode[i], input->keybit);
	 }
	 
	__set_bit(EV_ABS, input->evbit);

	/* For single touch */
	input_set_abs_params(input, ABS_X,0, 255, 0, 0);
 	 
	 INIT_DELAYED_WORK(&touch->pos_work, get_slider_position_func);
 
#ifdef	__POLL_SENSOR__DATA__	 
	 data->poll  = 1;
#endif

	if( data->poll )
	{
		schedule_delayed_work(&touch->pos_work, msecs_to_jiffies(5000));
	}
	else
	{
		 err = request_threaded_irq(touch->irq, NULL, mx_qm_irq_handler,
			 IRQF_TRIGGER_LOW | IRQF_ONESHOT, input->name, touch);
		 if (err) {
			 dev_err(&client->dev, "fail to request irq\n");
			 goto err_free_mem;
		 }
		enable_irq_wake(touch->irq);
	}
 
	 /* Register the input device */
	 err = input_register_device(touch->input);
	 if (err) {
		 dev_err(&client->dev, "Failed to register input device\n");
		 goto err_free_irq;
	 }
 	  
	 pr_debug("%s:--\n",__func__);
	 return 0;
 
 err_free_irq:
	if(data->poll  == 0)
		free_irq(client->irq, data);
 err_free_mem:
	 input_free_device(input);
	 kfree(data);
	 return err;

}

static int __devexit mx_qm_touch_remove(struct platform_device *pdev)
{
	struct mx_qm_touch * touch = platform_get_drvdata(pdev);
	struct mx_qm_data * mx = touch->data;
	
	if(mx->poll  == 0)
	{
		/* Release IRQ */
		free_irq(touch->data->client->irq, touch);
	}
		
	cancel_delayed_work_sync(&touch->pos_work);

	input_unregister_device(touch->input);
	kfree(touch);

	return 0;
}

const struct platform_device_id mx_qm_touch_id[] = {
	{ "mx-qm-touch", 0 },
	{ },
};

static struct platform_driver mx_qm_touch_driver = {
	.driver = {
		.name  = "mx-qm-touch",
		.owner = THIS_MODULE,
	},
	.probe = mx_qm_touch_probe,
	.remove = __devexit_p(mx_qm_touch_remove),
	.id_table = mx_qm_touch_id,
};

static int __init mx_qm_touch_init(void)
{
	return platform_driver_register(&mx_qm_touch_driver);
}
module_init(mx_qm_touch_init);

static void __exit mx_qm_touch_exit(void)
{
	platform_driver_unregister(&mx_qm_touch_driver);
}
module_exit(mx_qm_touch_exit); 

MODULE_AUTHOR("Chwei <Chwei@meizu.com>");
MODULE_DESCRIPTION("MX QMatrix Sensor Touch Pad");
MODULE_LICENSE("GPL");
