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

//#define	__POLL_SENSOR__DATA__
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
	struct input_dev *input;
	int irq;			/* irq issued by device		*/
	unsigned short keycodes[ARRAY_SIZE(mx_qm_keycode)];
	u8 last_key;
	u8 keys_press;
	u8 pos;
	struct work_struct detect_work;
	struct delayed_work pos_work;
	unsigned char  precpos;
	unsigned char rec_pos[MAX_REC_POS_SIZE];
#ifdef CONFIG_HAS_EARLYSUSPEND
	 struct early_suspend early_suspend;
	 int early_suspend_flag;
#endif
};


void qm_touch_report_pos(struct input_dev *dev, int pos,int bsync)
{
	pr_debug("%s:Pos = %d  \n",__func__,pos);
	input_report_abs(dev, ABS_X, pos);
	if( bsync )
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
	struct input_dev *input = touch->input;
	
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
  pr_info("%s:j = %.2d  i = %d len = %d", __func__,j,i,len );	
  
  mx_qt_key = (posBuf[0] >> 6) + QM_KEY_1;
  //mx_qt_key = (posBuf[0] / 86 ) + QM_KEY_1;

  if( touch->precpos > 0x1F )
    goto end90;
  
#if 0//#ifndef  __LED_TCA6507__  
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
  touch->precpos = 0;  
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
		pr_debug("%s:%.2d  ", __func__,key );	
	}			

	return key;
}

static void get_slider_position_poll_func(struct work_struct *work)
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
	touch->rec_pos[touch->precpos & 0x1F] = pos;
	touch->precpos++;

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


static void get_slider_position_func(struct work_struct *work)
{
	struct mx_qm_touch	*touch =
		container_of(work, struct mx_qm_touch,detect_work);
	
	struct mx_qm_data * mx = touch->data;
	struct i2c_client *client = mx->client;
	struct input_dev *input = touch->input;
	
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
	
	if( touch->pos != pos )
		qm_touch_report_pos(touch->input,pos,1);
	touch->pos = pos;	

	if( state ) 	
	{		
#ifdef __TEST_TOCHPAP_DELTA__
		{
			u16 ref[4],sig[4],delta[4]; 
			mx->i2c_readbuf(client, QM_REG_REF,8,ref);
			dev_info(&client->dev, "ref = 0x%.4X  0x%.4X  0x%.4X  0x%.4X  \n", ref[0],ref[1],ref[2],ref[3]);
			mx->i2c_readbuf(client, QM_REG_SIG,8,sig);
			dev_info(&client->dev, "sig = 0x%.4X  0x%.4X  0x%.4X  0x%.4X  \n", sig[0],sig[1],sig[2],sig[3]);
			mx->i2c_readbuf(client, QM_REG_DELTA,8,delta);
			dev_info(&client->dev, "delta = 0x%.4X	0x%.4X	0x%.4X	0x%.4X	\n", delta[0],delta[1],delta[2],delta[3]);
			dev_info(&client->dev, "pos = %d\n", pos);
		}			
#endif	
		qm_touch_get_key(touch);
	}
	else
	{		
		pos = mx->i2c_readbyte(client, QM_REG_POSITION);
		qm_touch_report_pos(touch->input,pos,1);
		key = qm_touch_get_key(touch);
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

	INIT_WORK(&touch->detect_work, get_slider_position_func);
	INIT_DELAYED_WORK(&touch->pos_work, get_slider_position_poll_func);
 
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
			// IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING | IRQF_ONESHOT, input->name, touch);
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
 	  
#ifdef CONFIG_HAS_EARLYSUSPEND
	 touch->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	 touch->early_suspend.suspend = mx_qm_touch_early_suspend;
	 touch->early_suspend.resume = mx_qm_touch_late_resume;
	 register_early_suspend(&touch->early_suspend);
#endif

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
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	 unregister_early_suspend(&touch->early_suspend);
#endif

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
