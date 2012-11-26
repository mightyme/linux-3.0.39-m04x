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

struct mx_qm_touch {
	 struct mx_qm_data *data;
	struct input_dev *input_key;
	int irq;			/* irq issued by device		*/
	u8 keys_press;
#ifdef CONFIG_HAS_EARLYSUSPEND
	 struct early_suspend early_suspend;
	 int early_suspend_flag;
#endif
};
void qm_touch_report_key(struct input_dev *dev, unsigned int code, int value)
{
	pr_debug("%s:KeyCode = %d  S = %d  \n",__func__,code,value);
	input_report_key(dev, code, value);
	input_sync(dev);
}

static irqreturn_t mx_qm_irq_handler(int irq, void *dev_id)
{
	struct mx_qm_touch *touch = dev_id;
	struct mx_qm_data *qm = touch->data;
	struct input_dev *input = touch->input_key;
	u8 bpress;

	bpress = !(gpio_get_value(qm->gpio_irq));
	if( touch->early_suspend_flag && bpress)
	{
		qm_touch_report_key( input,KEY_HOME,1 );	
		qm_touch_report_key( input,KEY_HOME,0 );	
		touch->keys_press = 0;
	}
	else
	{
		if( touch->keys_press != bpress)
		{
			qm_touch_report_key( input,KEY_HOME,bpress );	
			touch->keys_press = bpress;
		}
	}
	
	pr_debug("%s:Key is %s.\n",__func__,touch->keys_press?"Pressed":"Released");		
	
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
	//struct mx_qm_platform_data *pdata = dev_get_platdata(data->dev);
	struct i2c_client *client;
	struct mx_qm_touch*touch;
	struct input_dev *input_key;
	//struct input_dev *input_pad;

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
	 
	 touch->data = data;
	 touch->input_key = input_key;
	 touch->irq = data->irq;//client->irq
	 
	platform_set_drvdata(pdev, touch);;
	 
	 input_key->name = "mx-touch-keypad";
	 input_key->dev.parent = &client->dev;
	 input_key->id.bustype = BUS_I2C;
	 input_key->id.vendor = 0x1111;
	
	 __set_bit(EV_KEY, input_key->evbit);
	 __set_bit(KEY_HOME, input_key->keybit);
	input_set_drvdata(input_key, data);
	
	/* Register the input_key device */
	err = input_register_device(touch->input_key);
	if (err) {
		dev_err(&client->dev, "Failed to register input key device\n");
		goto err_free_mem_key;
	}	 

	s3c_gpio_setpull(data->gpio_irq, S3C_GPIO_PULL_UP);
	s3c_gpio_cfgpin(data->gpio_irq, S3C_GPIO_INPUT);

	 err = request_threaded_irq(touch->irq, NULL, mx_qm_irq_handler,
		 IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING | IRQF_ONESHOT, input_key->name, touch);
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
 
// err_free_irq:
	free_irq(client->irq, data);	
 err_un_input_key:
	input_unregister_device(touch->input_key);
 err_free_mem_key:
	 input_free_device(input_key);
	 kfree(data);
	 return err;

}

static int __devexit mx_qm_touch_remove(struct platform_device *pdev)
{
	struct mx_qm_touch * touch = platform_get_drvdata(pdev);
//	struct mx_qm_data * mx = touch->data;
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	 unregister_early_suspend(&touch->early_suspend);
#endif

	/* Release IRQ */
	free_irq(touch->data->client->irq, touch);

	input_unregister_device(touch->input_key);

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
