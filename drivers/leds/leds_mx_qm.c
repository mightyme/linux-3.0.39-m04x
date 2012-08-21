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
	  
#include <linux/module.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include	<linux/mx_qm.h>

#define	MODE_CURRENT			0x0
#define	MODE_PWM				0x1
#define	MODE_SLOPE			0x02
#define	MODE_DEVICE_OFFSET	0x03

#define DEVICE_MODE_OFF          0x00
#define DEVICE_MODE_OFF_1        0x01
#define DEVICE_MODE_ON_PWM0      0x02
#define DEVICE_MODE_ON_PWM1      0x03
#define DEVICE_MODE_ON_FULL      0x04
#define DEVICE_MODE_ON_BRHT       0x05
#define DEVICE_MODE_ON_BANK0      0x06
#define DEVICE_MODE_ON_BANK1      0x07
#define DEVICE_MODE_MAX      0x08

#define	GET_MODE(x)	((x&0x0F00)>>8)
////////////////////////////////////////////////////////////////// 
 
 /*default we set the slope cycle to 3 second, 
 if slope is disable, this setting doesn't work*/
#define LED_REG_VAL(x) (x | SLOPE_CYCLE_3)
 /*whether slope is enable or not, pwm is enabled all the time*/
#define SLOPE_ENABLE_VAL (SLOPE_EN | SLOPE_QUARTER | PWN_EN)
#define SLOPE_DISABLE_VAL (SLOPE_NONE | PWN_EN)

 /* Slave addr = 0x74: device registers */
 enum bu26507_led_reg {
	 REG_SOFTWARE_RESET = 0x00,
	 REG_OSC_CONTROL = 0x01,
	 REG_LED_ENABLE = 0x11,
	 REG_LED_PWM = 0x20,
	 REG_SYNC_CONTROL = 0x21,
	 REG_SETTING = 0x2d,
	 REG_SCROLL_SETTING = 0x2f,
	 REG_MATRIX_CONTROL = 0x30,
	 REG_REGISTER_MAP = 0x7f,
 };
 
 
 enum bu26507_reg_map {
	 RMCG_CONTROL = 0x00,
	 RMCG_AB,
 };
 
 /* pattern register data */
 enum bu26507_reg_data {
	 REG_LED1 = 0x01,	/*DA00*/
	 REG_LED2 = 0x07,	/*DA11*/
	 REG_LED3 = 0x0D,	/*DA22*/
	 REG_LED4 = 0x13,	/*DA33*/
	 REG_LED5 = 0x19,	/*DA44*/
	 REG_LED6 = 0x1E,	/*DA54*/
 };
 
 /*REG_SETTING setting values*/
 enum bu26507_setting_type {
	 SCROLL_EN = 1 << 0,
	 SLOPE_EN = 1 << 1,
	 PWN_EN = 1 << 2,
	 SLOPE_QUARTER = 0 << 3,	 /*1/4 slope*/
	 SLOPE_NONE = 1 << 3,	 /*none slope*/
	 SLOPE_HALF = 2 << 3,	 /*1/2 slope*/
 };
 
 /*led matrix control, start or stop*/
 enum bu26507_matrix_control {
	 MATRIX_LED_STOP = 0x00,
	 MATRIX_LED_START,
 };
 
 /*slope cycle time values, 0~3 second*/
 enum bu26507_slope_cycle_time {
	 SLOPE_CYCLE_0 = 0 << 6,
	 SLOPE_CYCLE_1 = 1 << 6,
	 SLOPE_CYCLE_2 = 2 << 6,
	 SLOPE_CYCLE_3 = 3 << 6,
 };


 /*led private data*/
struct mx_qm_led {
	 struct mx_qm_data *data;
	 struct led_classdev led_cdev;
	 struct mutex mutex;
	 int id;
#ifdef CONFIG_HAS_EARLYSUSPEND
	 struct early_suspend early_suspend;
#endif
};

 #ifdef CONFIG_FW_MXQM_DEV
 /*all led register address*/
 static unsigned char led_addr_array[] = {
	 REG_LED1,
	 REG_LED2,
	 REG_LED3,
	 REG_LED4,
	 REG_LED5,
 };

 #else
 /*all led register address*/
 static unsigned char led_addr_array[] = {
	 LED_REG_LEDM3,
	 LED_REG_LEDM1,
	 LED_REG_LEDM2,
	 LED_REG_LEDM4,
	 LED_REG_LEDM5,
 };
 #endif
 
 static int bu26507_set_led_current(struct led_classdev *led_cdev, int cur)
 {
	 struct mx_qm_led *led =
			 container_of(led_cdev, struct mx_qm_led, led_cdev);
	 struct mx_qm_data *mx = led->data;
	 int ret = 0;
	 
	 /*led current level, from 0~15*/
	cur = (cur >> 4) & 0xF;
	 
	pr_debug("%s: id = %d (0x%.2X)  cur  = 0x%.2X\n", __func__,led->id,led_addr_array[led->id],cur); 

	/* change to led registers map */
	ret = mx->i2c_writebyte(mx->client, REG_REGISTER_MAP, RMCG_AB);
	if (ret < 0) {
		pr_err("%s: change to led registers map fail!\n", __func__);
		return ret;
	}
	usleep_range(50, 60);

	ret = mx->i2c_writebyte(mx->client, led_addr_array[led->id], LED_REG_VAL(cur));
	if (ret < 0)   /* don't return but try to recover the register map instead */
		pr_err("%s: change led setting fail!\n", __func__); 
	else
		usleep_range(50, 60);
	
	/* recovery back to control registers map and light on the leds */
	ret = mx->i2c_writebyte(mx->client, REG_REGISTER_MAP, RMCG_CONTROL);
	if (ret < 0)
		pr_err("%s: recovery registers map fail!\n", __func__);
	else 
		usleep_range(50, 60);

	return ret;
}
 
static int bu26507_set_led_pwm(struct led_classdev *led_cdev, int pwm)
{
	 struct mx_qm_led *led =
			 container_of(led_cdev, struct mx_qm_led, led_cdev);
	 struct mx_qm_data *mx = led->data;
	 int ret = 0;
	 
	 /*led pwm level, from 0~63*/
	 pwm = (pwm >> 2) & 0x3F;

	pr_debug("%s: pwm  = 0x%X\n", __func__,pwm); 

	ret = mx->i2c_writebyte(mx->client, REG_LED_PWM, (u8)pwm);

	return ret;
}
 
 static int bu26507_set_led_slope(struct led_classdev *led_cdev, int enable)
 {
	 struct mx_qm_led *led =
			 container_of(led_cdev, struct mx_qm_led, led_cdev);
	 struct mx_qm_data *mx = led->data;
	 int ret = 0;
	 
	 pr_debug("%s: enable  = 0x%X\n", __func__,enable); 

	/*stop all leds first*/
	ret = mx->i2c_writebyte(mx->client, REG_MATRIX_CONTROL, MATRIX_LED_STOP);
	if (ret < 0) {
		pr_err("%s: stop all leds fail!\n", __func__);
		return ret;
	}

	if (enable)
		ret = mx->i2c_writebyte(mx->client, REG_SETTING, SLOPE_ENABLE_VAL);
	else 
		ret = mx->i2c_writebyte(mx->client, REG_SETTING, SLOPE_DISABLE_VAL);

	if (ret < 0)
		pr_err("%s: setting REG_SETTING fail!\n", __func__);

	/*start all leds again*/
	ret = mx->i2c_writebyte(mx->client, REG_MATRIX_CONTROL, MATRIX_LED_START);
	if (ret < 0)
		pr_err("%s: start all leds fail!\n", __func__);
	
	return ret;
}
 
 static int bu26507_set_led_mode(struct led_classdev *led_cdev, int mode)
 {
	 int ret = 0;
	 struct mx_qm_led *led =
			 container_of(led_cdev, struct mx_qm_led, led_cdev);
	 struct mx_qm_data *mx = led->data;
	 pr_debug("%s:\n",__func__);

	 if( mode > DEVICE_MODE_MAX )
	 	return -EINVAL;

 	//ret = mx->i2c_writebyte(mx->client,led_addr_array[led->id],mode);
 
	 return ret;
 }

 
  static int tca6507_set_led_current(struct led_classdev *led_cdev, int value)
  {
	  struct mx_qm_led *led =
			  container_of(led_cdev, struct mx_qm_led, led_cdev);
	  struct mx_qm_data *mx = led->data;
	  int ret = 0;
	  unsigned char data;
	  pr_debug("%s:current = %d\n",__func__,value);
	 
	  data = ((value>>4) & 0x0F) |(value & 0xF0);	  
	  //ret = mx->i2c_writebyte(mx->client,LED_REG8_MAXINTENSITY,data);	 

	  if( data == 0 )
	  	ret = mx->i2c_writebyte(mx->client,led_addr_array[led->id],DEVICE_MODE_OFF);
	  else
	  	ret = mx->i2c_writebyte(mx->client,led_addr_array[led->id],DEVICE_MODE_ON_PWM0);
	  
	  return ret;
  }
  
  static int tca6507_set_led_pwm(struct led_classdev *led_cdev, int value)
  {
	  struct mx_qm_led *led =
			  container_of(led_cdev, struct mx_qm_led, led_cdev);
	  struct mx_qm_data *mx = led->data;
	  int ret = 0;
	  unsigned char data;
	  pr_info("%s:value = %d\n",__func__,value);
 
	  data = (value & 0xF0) |((value >> 4) & 0xF);	  
	  
	  ret = mx->i2c_writebyte(mx->client,LED_REG8_MAXINTENSITY,data);		  
	  
	  //ret = mx->i2c_writebyte(mx->client,led_addr_array[0],DEVICE_MODE_ON_PWM0);
	  //ret = mx->i2c_writebyte(mx->client,led_addr_array[1],DEVICE_MODE_ON_PWM0);
	  //ret = mx->i2c_writebyte(mx->client,led_addr_array[2],DEVICE_MODE_ON_PWM0);
	  //ret = mx->i2c_writebyte(mx->client,led_addr_array[3],DEVICE_MODE_ON_PWM0);
  
	  return ret;
  }
  
  static int tca6507_set_led_slope(struct led_classdev *led_cdev, int enable)
  {
	  struct mx_qm_led *led =
			  container_of(led_cdev, struct mx_qm_led, led_cdev);
	  struct mx_qm_data *mx = led->data;
	  int ret = 0;
	  
	  pr_debug("%s:enable = %d\n",__func__,enable);
#if 0
	  int i;
	  for(i = 0; i < sizeof(led_addr_array) ;i++)
	 {
		 if( enable )
			 ret = mx->i2c_writebyte(mx->client,led_addr_array[i],DEVICE_MODE_ON_BANK0);
		 else
			 ret = mx->i2c_writebyte(mx->client,led_addr_array[i],DEVICE_MODE_OFF);
	 }	  
#else
	 if( enable )
	 {
		 ret = mx->i2c_writebyte(mx->client,led_addr_array[2],DEVICE_MODE_ON_BANK1);
	 }
	 else
	 {
		 ret = mx->i2c_writebyte(mx->client,led_addr_array[2],DEVICE_MODE_OFF);
	 }
#endif
  
	  return ret;
  }
  
  static int tca6507_set_led_mode(struct led_classdev *led_cdev, int mode)
  {
	  int ret = 0;
	  struct mx_qm_led *led =
			  container_of(led_cdev, struct mx_qm_led, led_cdev);
	  struct mx_qm_data *mx = led->data;
	  pr_debug("%s:\n",__func__);
 
	  if( mode > DEVICE_MODE_MAX )
		 return -EINVAL;
 
	 ret = mx->i2c_writebyte(mx->client,led_addr_array[led->id],mode);
  
	  return ret;
  }

 
static int mx_qm_set_led_current(struct led_classdev *led_cdev, int value)
{
#ifdef	CONFIG_FW_MXQM_DEV
	return bu26507_set_led_current(led_cdev,value);
#else
	return tca6507_set_led_current(led_cdev,value);
#endif
}

  
static int mx_qm_set_led_pwm(struct led_classdev *led_cdev, int value)
{
#ifdef	CONFIG_FW_MXQM_DEV
	return bu26507_set_led_pwm(led_cdev,value);
#else
	return tca6507_set_led_pwm(led_cdev,value);
#endif
}
  
static int mx_qm_set_led_slope(struct led_classdev *led_cdev, int enable)
{
#ifdef	CONFIG_FW_MXQM_DEV
	return bu26507_set_led_slope(led_cdev,enable);
#else
	return tca6507_set_led_slope(led_cdev,enable);
#endif
}

  
static int mx_qm_set_led_mode(struct led_classdev *led_cdev, int mode)
{
#ifdef	CONFIG_FW_MXQM_DEV
	return bu26507_set_led_mode(led_cdev,mode);
#else
	return tca6507_set_led_mode(led_cdev,mode);
#endif
}

 static void mx_qm_led_brightness_set(struct led_classdev *led_cdev,
				 enum led_brightness value)
 {
	 //struct mx_qm_led *led =container_of(led_cdev, struct mx_qm_led, led_cdev);
	 //struct mx_qm_data *mx = led->data;
	 int ret = 0;
	 int mode;
	 int data;

	 dev_info(led_cdev->dev, "value = 0x%.4X \n",value);

	 mode = GET_MODE(value);
	 data = value & 0xFF; 

	 switch( mode )
 	{
		case MODE_CURRENT:
			ret = mx_qm_set_led_current(led_cdev,data);
			break;

		case MODE_PWM:
			ret = mx_qm_set_led_pwm(led_cdev,data);
			break;

		case MODE_SLOPE:
			ret = mx_qm_set_led_slope(led_cdev,data);
			break;

		default:
			mode = mode -MODE_DEVICE_OFFSET;
			
			 if(mode < DEVICE_MODE_MAX)
			{
				ret = mx_qm_set_led_mode(led_cdev,mode);	
				if(ret < 0)
					dev_err(led_cdev->dev, "set %d mode failed %d \n",mode,ret);			
			}
			 else
			{
				dev_err(led_cdev->dev, "mode  %d is valite \n",mode);
				ret = -EINVAL;
			}
			
			break;
 	}

	if(ret < 0)
		dev_err(led_cdev->dev, "brightness set failed ret = %d \n",ret);
		
 }
 
 static int mx_qm_led_blink_set(struct led_classdev *led_cdev,
				     unsigned long *delay_on,
				     unsigned long *delay_off)
 {
	 int ret = 0;
	 struct mx_qm_led *led =container_of(led_cdev, struct mx_qm_led, led_cdev);
	 struct mx_qm_data *mx = led->data;
	 
 	return ret;
 }
 
#ifdef CONFIG_HAS_EARLYSUSPEND
 static void mx_qm_early_suspend(struct early_suspend *h)
 {
	 struct mx_qm_led *led =
			 container_of(h, struct mx_qm_led, early_suspend);
 
 }
 
 static void mx_qm_late_resume(struct early_suspend *h)
 {
	 struct mx_qm_led *led =
			 container_of(h, struct mx_qm_led, early_suspend);
 
 }
#endif
 
 static int __devinit mx_qm_led_probe(struct platform_device *pdev)
 {
	 struct mx_qm_data *data = dev_get_drvdata(pdev->dev.parent);
	 struct mx_qm_platform_data *pdata = dev_get_platdata(data->dev);
	 struct mx_qm_led *led;
	 char name[20];
	 int ret;
 
	 if (!pdata) {
		 dev_err(&pdev->dev, "no platform data\n");
		 ret = -ENODEV;
	 }
 
	 led = kzalloc(sizeof(*led), GFP_KERNEL);
	 if (led == NULL) {
		 ret = -ENOMEM;
		 goto err_mem;
	 }
 
	 led->id = pdev->id; 
	 led->data = data;
 
	 snprintf(name, sizeof(name), "%s%d", pdev->name, pdev->id);
	 led->led_cdev.name = name;
	 led->led_cdev.brightness = 0;
	 led->led_cdev.max_brightness= 0xFFF;
	 
	 led->led_cdev.brightness_set = mx_qm_led_brightness_set;
	 led->led_cdev.blink_set= mx_qm_led_blink_set;
 
	 mutex_init(&led->mutex);
	 platform_set_drvdata(pdev, led);
 
	 ret = led_classdev_register(&pdev->dev, &led->led_cdev);
	 if (ret < 0)
		 goto err_register_led;
 
#ifdef CONFIG_HAS_EARLYSUSPEND
	 led->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	 led->early_suspend.suspend = mx_qm_early_suspend;
	 led->early_suspend.resume = mx_qm_late_resume;
	 register_early_suspend(&led->early_suspend);
#endif
 
	 return 0;
 
 err_register_led:
	 kfree(led);
 err_mem:
	 return ret;
 }
 
 static int __devexit mx_qm_led_remove(struct platform_device *pdev)
 {
	 struct mx_qm_led *led = platform_get_drvdata(pdev);
 
#ifdef CONFIG_HAS_EARLYSUSPEND
	 unregister_early_suspend(&led->early_suspend);
#endif
	 led_classdev_unregister(&led->led_cdev);
	 kfree(led);
 
	 return 0;
 }
 
 const struct platform_device_id mx_qm_id[] = {
	 { "mx-qm-led",0 },
	 { },
 };
 
 static struct platform_driver mx_qm_led_driver = {
	 .driver = {
		 .name	= "mx-qm-led",
		 .owner = THIS_MODULE,
	 },
	 .probe  = mx_qm_led_probe,
	 .remove = __devexit_p(mx_qm_led_remove),
	 .id_table = mx_qm_id,
 };
 
 static int __init mx_qm_led_init(void)
 {
	 return platform_driver_register(&mx_qm_led_driver);
 }
 module_init(mx_qm_led_init);
 
 static void __exit mx_qm_led_exit(void)
 {
	 platform_driver_unregister(&mx_qm_led_driver);
 }
 module_exit(mx_qm_led_exit); 


MODULE_AUTHOR("Chwei <Chwei@meizu.com>");
MODULE_DESCRIPTION("MX QMatrix Sensor Leds");
MODULE_LICENSE("GPL");
