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
#include <linux/earlysuspend.h>
#include <plat/gpio-cfg.h>
#include <mach/gpio-m040.h>
#include <linux/firmware.h>
#include	<linux/mx_qm.h>

//#define	__TEST_TOCHPAP_DELTA__

#define RESET_COLD	1
#define	RESET_SOFT	0

#define	DETECT_POS_INTERVAL		(10)

#define	MAX_REC_POS_SIZE		(0x20)
#define	MAX_REC_POS_SIZE_MASK		(0x1F)
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
	struct input_dev *input_key;
	struct input_dev *input_pad;
	int irq;			/* irq issued by device		*/
	unsigned short keycodes[ARRAY_SIZE(mx_qm_keycode)];
	u8 last_key;
	u8 keys_press;
	u8 pos;
	struct work_struct detect_work;
	unsigned char  precpos;
	unsigned char rec_pos[MAX_REC_POS_SIZE];
#ifdef CONFIG_HAS_EARLYSUSPEND
	 struct early_suspend early_suspend;
	 int early_suspend_flag;
#endif
};


void qm_touch_report_pos(struct input_dev *dev, int pos,int bPress)
{
	pr_debug("%s:Pos = %d  \n",__func__,pos);
//	input_report_key(dev, BTN_TOUCH, bPress);
	input_report_abs(dev, ABS_X, pos);
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

#define POS_KEY_THR       32
#define POS_KEYS_INTERVAL_THR       80
unsigned short qm_touch_cal_key2(struct mx_qm_touch	*touch)
{
	//struct mx_qm_data * mx = touch->data;
	//struct i2c_client *client = mx->client;
	//struct input_dev *input = touch->input_key;

	signed short i,j;
	unsigned char len=0,dir0,dir1,mx_qt_key;
	unsigned char posBuf[16];

	// Get the valid data
	j = touch->precpos;  
	for(i=0;i< 16;i++) 
	{
		j--;
		++len;
		posBuf[i] = touch->rec_pos[j&0x1F];  
		if(j == 0)
			break;    
	}
	pr_info("\n\n\n%s:j = %.2d  i = %d len = %d", __func__,j,i,len );	

	mx_qt_key = (posBuf[0] >> 6) + QM_KEY_1;
	//mx_qt_key = (posBuf[0] / 86 ) + QM_KEY_1;

	if( touch->precpos > 0x1F )
		goto end90;

#if 1//#ifndef  __LED_TCA6507__  
	// clear the remain buffer
	for(i = len;i< 16;i++) 
	posBuf[i] = 0;  
#endif  

	// Get the sum of steps
	len--;
	j = 0;
	dir0 = 0; 
	dir1 = 0;
	for(i = 0;i< len ;i++) 
	{
		if( posBuf[i] > posBuf[i+1])
			dir1 ++;
		else if( posBuf[i] < posBuf[i+1])
			dir0 ++;      
	}
	j = posBuf[0] - posBuf[i];
	pr_info("%s:THR = %.2d  dir1 = %d ,dir0 = %d posBuf[len] = %d posBuf[0] = %d len = %d", __func__,j,dir1,dir0,posBuf[len] ,posBuf[0],len );	


	//i = posBuf[len] - posBuf[0]; // Check the interval of start and end postion
	//i = abs(i);
	//if( (j > POS_KEY_THR || j < -POS_KEY_THR) && i > POS_KEYS_INTERVAL_THR)
	if(j > POS_KEY_THR || j < -POS_KEY_THR)
	{
		pr_info("%s:key = %.2d  ", __func__,j);	
		if( j > POS_KEY_THR )
		{
			if( dir0 <= 3 )
			{
				if(posBuf[len] < 80 && posBuf[0] > 160)
				{
					mx_qt_key = QM_KEY_L2R;
				}
				else if(posBuf[len] > 192)
					mx_qt_key = QM_KEY_M2R;
				else if(posBuf[0] < 64)
					mx_qt_key = QM_KEY_L2M;
			}
		}
		else
		{
			if( dir1 <= 3 )
			{
				if(posBuf[0] < 80 && posBuf[len] > 160) // ?
				{
					mx_qt_key = QM_KEY_R2L;
				} 
				else if(posBuf[len] > 192)
					mx_qt_key = QM_KEY_R2M;
				else if(posBuf[0] < 64)
					mx_qt_key = QM_KEY_M2L;
				}
			}          
		} 

end90:  
	pr_info("%s:key = %.2d  ", __func__,mx_qt_key);		


	printk("\nposBuf:len = %.d \n",len+1);
	for(i=0;i<16;i++) 
	printk("%.3d ",posBuf[i] );

	printk("\nprecpos = %.d \n",touch->precpos);
	for(i=0;i<MAX_REC_POS_SIZE;i++) 
	printk("%.3d ",touch->rec_pos[i] );
	memset(touch->rec_pos,0x00,MAX_REC_POS_SIZE);
	touch->precpos = 0;  

	return mx_qt_key;
}


unsigned short qm_touch_get_key(struct mx_qm_touch	*touch)
{
	struct mx_qm_data * mx = touch->data;
	struct i2c_client *client = mx->client;
	struct input_dev *input = touch->input_key;
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
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	if( ( touch->early_suspend_flag ) )
	{
		if(key == KEY_MENU ||key == KEY_BACK )
			return key;
	}
#endif	

	if(ret == 0)
	{
		qm_touch_report_key(input, touch->last_key, 1);
		qm_touch_report_key(input, touch->last_key, 0);
		pr_debug("%s:%.2d  ", __func__,key );	
	}			

	return key;
}

static void get_slider_position_func(struct work_struct *work)
{
	struct mx_qm_touch	*touch =
		container_of(work, struct mx_qm_touch,detect_work);
	
	struct mx_qm_data * mx = touch->data;
	struct i2c_client *client = mx->client;
	struct input_dev *input = touch->input_key;
	
	u8 key,pos;
	u8 state;		
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	if( touch->early_suspend_flag )
	{
		qm_touch_get_key(touch);	
		goto end;
	}
#endif	
		
	/* Read the detected status register, thus clearing interrupt */
	state = mx->i2c_readbyte(client, QM_REG_STATE);
	
	/* Read which key changed */
	pos = mx->i2c_readbyte(client, QM_REG_POSITION);
	touch->rec_pos[touch->precpos++] = pos;
	if( touch->precpos >= sizeof(touch->rec_pos) )
		touch->precpos = 0;
	
	dev_dbg(&client->dev, "pos = %d\n", pos);	

	if( state ) 	
	{	
		if( touch->pos != pos )
			qm_touch_report_pos(touch->input_pad,pos,state);
		touch->pos = pos;		
#ifdef __TEST_TOCHPAP_DELTA__
		{
			u16 info[10]; 
			memset(info,0,sizeof(info));
			mx->i2c_readbuf(client, QM_REG_DBGINFO,sizeof(info),info);
			dev_info(&client->dev, "ref = 0x%.4X  0x%.4X  0x%.4X  0x%.4X  \n", info[0],info[1],info[2],info[3]);
			dev_info(&client->dev, "sig = 0x%.4X  0x%.4X  0x%.4X  0x%.4X  \n", info[4],info[5],info[6],info[7]);
			dev_info(&client->dev, "delta = %.d\n", info[8]);
			dev_info(&client->dev, "pos = %.d\n", info[9] & 0xFF);
			dev_info(&client->dev, "dect = %.d\n", (info[9] >>8)& 0xFF);
		}			
#endif	
		qm_touch_get_key(touch);
	}
	else
	{		
		pos = mx->i2c_readbyte(client, QM_REG_POSITION);
		qm_touch_report_pos(touch->input_pad,pos,0);
		key = qm_touch_get_key(touch);
		//qm_touch_cal_key2(touch);
	}

end:	
	enable_irq(touch->irq);
}

static irqreturn_t mx_qm_irq_handler(int irq, void *dev_id)
{
	struct mx_qm_touch *touch = dev_id;
	
	pr_debug("%s:\n",__func__);
	disable_irq_nosync(touch->irq);
	schedule_work(&touch->detect_work);
	
	return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
 static void mx_qm_touch_early_suspend(struct early_suspend *h)
 {
	 struct mx_qm_touch *touch =
			 container_of(h, struct mx_qm_touch, early_suspend);
	struct mx_qm_data * mx = touch->data;

	touch->early_suspend_flag = true;
	mx->i2c_writebyte(mx->client, QM_REG_STATUS,QM_STATE_SLEEP);

}
 
 static void mx_qm_touch_late_resume(struct early_suspend *h)
 {
	 struct mx_qm_touch *touch =
			 container_of(h, struct mx_qm_touch, early_suspend);
	struct mx_qm_data * mx = touch->data;

	mx->i2c_writebyte(mx->client, QM_REG_STATUS,QM_STATE_NORMAL);
	touch->early_suspend_flag = false;
 }
#endif 

static int __devinit mx_qm_touch_probe(struct platform_device *pdev)
{
	struct mx_qm_data *data = dev_get_drvdata(pdev->dev.parent);
	struct mx_qm_platform_data *pdata = dev_get_platdata(data->dev);
	struct i2c_client *client;
	struct mx_qm_touch*touch;
	struct input_dev *input_key,*input_pad;

	int i;
	int err;
	pr_debug("%s:++\n",__func__);

	 client = data->client;
	 if (!client->irq) {
		 dev_err(&client->dev, "please assign the irq to this device\n");
		 return -EINVAL;
	 }
 
	 touch = kzalloc(sizeof(struct mx_qm_touch), GFP_KERNEL);
	 input_key = input_allocate_device();
	 if (!touch || !input_key) {
		 dev_err(&client->dev, "insufficient memory\n");
		 err = -ENOMEM;
		 goto err_free_mem_key;
	 }
	 
	 input_pad = input_allocate_device();
	 if (!touch || !input_pad) {
		 dev_err(&client->dev, "insufficient memory\n");
		 err = -ENOMEM;
		 goto err_free_mem;
	 }

	 touch->data = data;
	 touch->input_key = input_key;
	 touch->input_pad = input_pad;
	 touch->irq = data->irq;//client->irq
	 
	platform_set_drvdata(pdev, touch);;

	 
	 input_key->name = "mx-touch-keypad";
	 input_key->dev.parent = &client->dev;
	 input_key->id.bustype = BUS_I2C;
 
	 /* Add the keycode */
	 input_key->keycode = touch->keycodes;
	 input_key->keycodesize = sizeof(touch->keycodes[0]);
	 input_key->keycodemax = ARRAY_SIZE(mx_qm_keycode);
 
	 __set_bit(EV_KEY, input_key->evbit);
 
	 for (i = 0; i < ARRAY_SIZE(mx_qm_keycode); i++) {
		 touch->keycodes[i] = mx_qm_keycode[i];
//		 __set_bit(mx_qm_keycode[i], input_key->keybit);
	 }
	 __set_bit(KEY_HOME, input_key->keybit);
	input_set_drvdata(input_key, data);

	input_pad->name = "mx-touch-pad";
	input_pad->id.bustype = BUS_I2C;
	input_pad->dev.parent = &client->dev;

	__set_bit(EV_ABS, input_pad->evbit);
	__set_bit(EV_KEY, input_pad->evbit);
	__set_bit(BTN_TOUCH, input_pad->keybit);

	/* For single touch */
	input_set_abs_params(input_pad, ABS_X,0, 255, 0, 0);
	input_set_abs_params(input_pad, ABS_Y,0, 255, 0, 0);

	input_set_drvdata(input_pad, data);
		
	/* Register the input_pad device */
	err = input_register_device(touch->input_pad);
	if (err) {
		dev_err(&client->dev, "Failed to register input pad device\n");
		goto err_free_mem;
	}
	
	/* Register the input_key device */
	err = input_register_device(touch->input_key);
	if (err) {
		dev_err(&client->dev, "Failed to register input key device\n");
		goto err_un_input_pad;
	}	 

	INIT_WORK(&touch->detect_work, get_slider_position_func);

	s3c_gpio_setpull(data->gpio_irq, S3C_GPIO_PULL_UP);
	s3c_gpio_cfgpin(data->gpio_irq, S3C_GPIO_INPUT);

	 err = request_threaded_irq(touch->irq, NULL, mx_qm_irq_handler,
		// IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING | IRQF_ONESHOT, input_key->name, touch);
		 IRQF_TRIGGER_LOW | IRQF_ONESHOT, input_key->name, touch);
	 if (err) {
		 dev_err(&client->dev, "fail to request irq\n");
		 goto err_un_input_key;
	 }
	enable_irq_wake(touch->irq);
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	 touch->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	 touch->early_suspend.suspend = mx_qm_touch_early_suspend;
	 touch->early_suspend.resume = mx_qm_touch_late_resume;
	 register_early_suspend(&touch->early_suspend);
#endif

	 pr_debug("%s:--\n",__func__);
	 return 0;
 
 err_free_irq:
	free_irq(client->irq, data);	
 err_un_input_key:
	input_unregister_device(touch->input_key);
err_un_input_pad:
	input_unregister_device(touch->input_pad);	
 err_free_mem:
	 input_free_device(input_pad);
 err_free_mem_key:
	 input_free_device(input_key);
	 kfree(data);
	 return err;

}

static int __devexit mx_qm_touch_remove(struct platform_device *pdev)
{
	struct mx_qm_touch * touch = platform_get_drvdata(pdev);
	struct mx_qm_data * mx = touch->data;
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	 unregister_early_suspend(&touch->early_suspend);
#endif

	/* Release IRQ */
	free_irq(touch->data->client->irq, touch);

	input_unregister_device(touch->input_key);
	input_unregister_device(touch->input_pad);

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
