/*

 * Battery driver for Maxim MAX77665
 *
 * Copyright (C) 2012 Meizu Technology Co.Ltd, Zhuhai, China
 * Author:  Lvcha qiu <lvcha@meizu.com> Chwei <Chwei@meizu.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/mfd/max77665.h>
#include <linux/mfd/max77665-private.h>
#include <linux/regulator/machine.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/bootmode.h>
#include <linux/android_alarm.h>
#include <mach/usb-detect.h>
#include <linux/workqueue.h>
#include <plat/adc.h>

#define MAX_AC_CURRENT         1000 
#define CHGIN_USB_CURRENT      450
#define TEMP_CHECK_DELAY       (120 * HZ) 
#define WAKE_ALARM_INT         (120) 
#define CURRENT_INCREMENT_STEP 100    /*mA*/
#define COMPLETE_TIMEOUT       200   /*ms*/ 
#define MA_TO_UA               1000  

#define MAX77665_CHGIN_DTLS       0x60 
#define MAX77665_CHGIN_DTLS_SHIFT 5    
#define MAX77665_CHG_DTLS         0x0F 
#define MAX77665_CHG_DTLS_SHIFT   0    
#define MAX77665_BAT_DTLS         0x70 
#define MAX77665_BAT_DTLS_SHIFT   4    
#define MAX77665_BYP_DTLS         0x0F 
#define MAX77665_BYP_DTLS_SHIFT   0    
#define MAX77665_BYP_OK           0x01 
#define MAX77665_BYP_OK_SHIFT     0    
#define MAX77665_CHGIN_OK         0x40 
#define MAX77665_CHGIN_OK_SHIFT   6    

struct max77665_charger
{
	struct device *dev;
	struct power_supply psy_usb;
	struct power_supply psy_ac;
	struct power_supply psy_charger;
	struct max77665_dev *iodev;
	struct wake_lock wake_lock;
#if defined(CONFIG_MX_RECOVERY_KERNEL)
	struct wake_lock charge_wake_lock;
#endif
	struct delayed_work dwork;
	struct delayed_work poll_dwork;
	struct workqueue_struct *ad_curr;
	struct delayed_work ad_work;
	struct work_struct chgin_work;
	struct regulator *ps;
	struct regulator *reverse;
	struct mutex mutex_t;
	struct completion byp_complete;	

	enum cable_status_t {
		CABLE_TYPE_NONE = 0,
		CABLE_TYPE_USB,
		CABLE_TYPE_AC,
		CABLE_TYPE_UNKNOW,
	} cable_status;

	enum chg_status_t {
		CHG_STATUS_FAST,
		CHG_STATUS_DONE,
		CHG_STATUS_RECHG,
	} chg_status;

	bool chgin;
	int chgin_irq;
	int bypass_irq;
	int chr_pin;
	int chgin_ilim_usb;	/* 60mA ~ 500mA */
	int chgin_ilim_ac;	/* 60mA ~ 2.58A */
	int (*usb_attach) (bool);
	int irq_reg;

	struct alarm alarm;
	bool done;
	bool adc_flag;
	struct s3c_adc_client *adc;
};
static BLOCKING_NOTIFIER_HEAD(max77665_charger_chain_head);

int register_max77665_charger_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&max77665_charger_chain_head, nb);
}
EXPORT_SYMBOL_GPL(register_max77665_charger_notifier);

int unregister_max77665_charger_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&max77665_charger_chain_head, nb);
}
EXPORT_SYMBOL_GPL(unregister_max77665_charger_notifier);

static int max77665_charger_notifier_call_chain(unsigned long val)
{
	return (blocking_notifier_call_chain(&max77665_charger_chain_head, val, NULL)
			== NOTIFY_BAD) ? -EINVAL : 0;
}

static char *supply_list[] = {
	"battery",
};

static enum power_supply_property max77665_power_props[] =
{
	POWER_SUPPLY_PROP_ONLINE,
};

static int max77665_usb_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct max77665_charger *charger =
		container_of(psy, struct max77665_charger, psy_usb);

	if (psp != POWER_SUPPLY_PROP_ONLINE)
		return -EINVAL;

	/* Set enable=1 only if the AC charger is connected */
	val->intval = (charger->cable_status == CABLE_TYPE_USB);
	
	return 0;
}

static int max77665_ac_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct max77665_charger *charger =
		container_of(psy, struct max77665_charger, psy_ac);
	
	if (psp != POWER_SUPPLY_PROP_ONLINE)
		return -EINVAL;

	/* Set enable=1 only if the AC charger is connected */
	val->intval = (charger->cable_status == CABLE_TYPE_AC);
	
	return 0;
}

static enum power_supply_property max77665_charger_props[] = {
	POWER_SUPPLY_PROP_STATUS,
};

static int max77665_charger_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct max77665_charger *charger =
		container_of(psy, struct max77665_charger, psy_charger);
	
	if (psp != POWER_SUPPLY_PROP_STATUS)
		return -EINVAL;

	if (charger->cable_status == CABLE_TYPE_USB || 
		charger->cable_status == CABLE_TYPE_AC) {
		if (charger->chg_status == CHG_STATUS_FAST)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_FULL;
	} else {
		val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
	}
	
	return 0;
}

static void set_alarm(struct max77665_charger *chg, int seconds)
{
	ktime_t interval = ktime_set(seconds, 0);
	ktime_t now = alarm_get_elapsed_realtime();
	ktime_t next = ktime_add(now, interval);

	pr_info("set alarm after %d seconds\n", seconds);
	alarm_start_range(&chg->alarm, next, next);
}

static void charger_bat_alarm(struct alarm *alarm)
{
	struct max77665_charger *chg = container_of(alarm, struct max77665_charger, alarm);

	wake_lock_timeout(&chg->wake_lock, 3 * HZ);
	set_alarm(chg, WAKE_ALARM_INT);
}

#define CHG_ADC_CHANNEL 2
static int adc_is_available(struct max77665_charger *charger)
{
	int adc_value = 0;

	msleep(100);
	adc_value = s3c_adc_read(charger->adc, CHG_ADC_CHANNEL);
	if (adc_value > 1024) {
		return 1;
	} else {
		return 0;
	}
}

static int max77665_adjust_current(struct max77665_charger *charger,
		int chgin_ilim)
{
	int ret = 0, ad_current;
	unsigned long expire;
	int MAX_INPUT_CURRENT = charger->chgin_ilim_ac; 

	expire = msecs_to_jiffies(COMPLETE_TIMEOUT);
	
	for (ad_current = chgin_ilim; ad_current <= MAX_INPUT_CURRENT;
			ad_current += CURRENT_INCREMENT_STEP) {
		
		ret = regulator_set_current_limit(charger->ps,
				ad_current*MA_TO_UA,
				(ad_current*MA_TO_UA+CURRENT_INCREMENT_STEP*MA_TO_UA));
		if (ret) {
			pr_err("failed to set current limit\n");
			return ret;
		}
		pr_info("%d,waiting..............\n", ad_current);
		
		init_completion(&charger->byp_complete);
		ret = wait_for_completion_timeout(&charger->byp_complete, expire);
		if (ret > 0) {
			pr_info("by pass not OK!!!\n");
			ad_current -= CURRENT_INCREMENT_STEP;
			ret = regulator_set_current_limit(charger->ps, 
					ad_current * MA_TO_UA,
					(ad_current * MA_TO_UA + 
					CURRENT_INCREMENT_STEP * MA_TO_UA));
			if (ret) {
				pr_err("adjust current to %d failed\n", ad_current);
			}
			break;
		}
		if (charger->adc_flag == true)
			break;
		if (!adc_is_available(charger))
			break;
	}
	return ret;
}

static int max77665_charger_types(struct max77665_charger *charger)
{
	enum cable_status_t cable_status = charger->cable_status;
	int chgin_ilim = 0;
	
	if (!regulator_is_enabled(charger->ps))
		regulator_enable(charger->ps);
	switch (cable_status) {
	case CABLE_TYPE_USB:
	case CABLE_TYPE_AC:	
	chgin_ilim = charger->chgin_ilim_usb;
		if (false == charger-> done) {
			pr_info("we want to adjust the input current now\n");
			max77665_adjust_current(charger, chgin_ilim);
			charger->done = true;
		} else {
			pr_info("we have adjusted already\n");
		}
		break;
	default:
		chgin_ilim = 0;
		break;
	}
	return 0;
}

static void max77665_work_func(struct work_struct *work)
{
	struct max77665_charger *charger =
		container_of(work, struct max77665_charger, dwork.work);
	enum cable_status_t cable_status = CABLE_TYPE_NONE;

	mutex_lock(&charger->mutex_t);
	
	if (charger->chgin) {
#ifndef CONFIG_MACH_M040
		if (mx_is_usb_dock_insert()) {
			pr_info("found dock inserted, treat it as AC\n");
			cable_status = CABLE_TYPE_AC;
		} else {
			if (charger->usb_attach && !charger->usb_attach(true)) {
				msleep(2000);

				if (gpio_get_value(charger->chr_pin)) {
					cable_status = CABLE_TYPE_AC;
				} else {
					cable_status = CABLE_TYPE_USB;
				}
			}
		}
#else
		do {
			u8 reg_data;
			max77665_read_reg(charger->iodev->muic, MAX77665_MUIC_REG_CDETCTRL1, &reg_data);
			max77665_write_reg(charger->iodev->muic, MAX77665_MUIC_REG_CDETCTRL1, reg_data | 0x02);
			pr_info("#############usb start detect\n");
			msleep(500);

			max77665_read_reg(charger->iodev->muic, MAX77665_MUIC_REG_STATUS2, &reg_data);
			pr_info("#############usb end detect (0x%02x)\n", reg_data);
			if (reg_data == 0x00) { 
				cable_status = CABLE_TYPE_NONE;
			} else {
				if ((reg_data & 0x07) == 0x01) {
					cable_status = CABLE_TYPE_USB;
				} else 
					cable_status = CABLE_TYPE_AC;
			}
		} while(0);
#endif
	} else {
		charger->done = false;
		charger->adc_flag = false;
		regulator_set_current_limit(charger->ps, 
				charger->chgin_ilim_usb * MA_TO_UA, 
				MAX_AC_CURRENT * MA_TO_UA);
		cable_status = CABLE_TYPE_NONE;
	}

	charger->cable_status = cable_status;
	
	power_supply_changed(&charger->psy_ac);
	power_supply_changed(&charger->psy_usb);

	max77665_charger_types(charger);
	
	if (delayed_work_pending(&charger->poll_dwork))
		cancel_delayed_work(&charger->poll_dwork);
	schedule_delayed_work_on(0, &charger->poll_dwork, 0);

	if (cable_status == CABLE_TYPE_USB) {
		charger->usb_attach(true);	
		max77665_charger_notifier_call_chain(1);
	} else {
		msleep(500);
		if (charger->usb_attach)
			charger->usb_attach(false);
		max77665_charger_notifier_call_chain(0);
	}
	wake_unlock(&charger->wake_lock);

#if defined(CONFIG_MX_RECOVERY_KERNEL)
	if (charger->chgin) {
		wake_lock(&charger->charge_wake_lock);
		if (is_charging_mode()) {
			if (charger->usb_attach)
				charger->usb_attach(false);
		}
	} else {
		wake_unlock(&charger->charge_wake_lock);
	}
#endif
	mutex_unlock(&charger->mutex_t);
}

static void max77665_poll_work_func(struct work_struct *work)
{
	struct max77665_charger *charger =
		container_of(work, struct max77665_charger, poll_dwork.work);
	struct power_supply *fuelgauge_ps
		= power_supply_get_by_name("fuelgauge");
	union power_supply_propval val;
	int battery_health = POWER_SUPPLY_HEALTH_GOOD;

	mutex_lock(&charger->mutex_t);
	if (fuelgauge_ps)
		if(fuelgauge_ps->get_property(fuelgauge_ps, POWER_SUPPLY_PROP_HEALTH, &val) == 0)
			battery_health = val.intval;

	if (charger->chg_status == CHG_STATUS_FAST ||
			charger->chg_status == CHG_STATUS_RECHG) {
		struct i2c_client *i2c = charger->iodev->i2c;
		u8 reg_data;

		if(max77665_read_reg(i2c, MAX77665_CHG_REG_CHG_DETAILS_01, &reg_data) >= 0) {
			if((reg_data & 0x0F) == 0x04) {
				charger->chg_status = CHG_STATUS_DONE;
			}
		}
	}

	if ( charger->cable_status == CABLE_TYPE_USB ||
			charger->cable_status == CABLE_TYPE_AC) {

		if(battery_health != POWER_SUPPLY_HEALTH_GOOD) {
			if (regulator_is_enabled(charger->ps)) {
				pr_info("----------battery unhealthy, disable charging\n");
				regulator_disable(charger->ps);
			}
		} else {
			if (regulator_is_enabled(charger->ps)) {
				if (charger->chg_status == CHG_STATUS_DONE && fuelgauge_ps) {
					int soc = 100;
					if(fuelgauge_ps->get_property(fuelgauge_ps, POWER_SUPPLY_PROP_CAPACITY, &val) == 0)
						soc = val.intval;
					if(soc <= 98) {
						regulator_enable(charger->ps);
						charger->chg_status = CHG_STATUS_RECHG;
					}
				}
			} else {
				pr_info("----------battery healthy good, enable charging\n");
				regulator_enable(charger->ps);
			}
		}
		schedule_delayed_work_on(0, &charger->poll_dwork, TEMP_CHECK_DELAY);
	} else {
		if (regulator_is_enabled(charger->ps)) {
			pr_info("--------------charger remove, disable charging\n");
		}
	}
	mutex_unlock(&charger->mutex_t);
}

static int max77665_reg_dump(struct max77665_charger *charger)
{
	struct i2c_client *i2c = charger->iodev->i2c;
	u8 int_ok, dtls_00, dtls_01, dtls_02;
	u8 chgin_dtls, chg_dtls, bat_dtls, byp_dtls;
	int ret;
	ret = max77665_read_reg(i2c, MAX77665_CHG_REG_CHG_INT_OK,
			&int_ok);
	/* charger */
	ret = max77665_read_reg(i2c, MAX77665_CHG_REG_CHG_DETAILS_00,
			&dtls_00);
	ret = max77665_read_reg(i2c, MAX77665_CHG_REG_CHG_DETAILS_01,
			&dtls_01);
	ret = max77665_read_reg(i2c, MAX77665_CHG_REG_CHG_DETAILS_02,
			&dtls_02);
	chgin_dtls = ((dtls_00 & MAX77665_CHGIN_DTLS) >>
			MAX77665_CHGIN_DTLS_SHIFT);
	chg_dtls = ((dtls_01 & MAX77665_CHG_DTLS) >>
			MAX77665_CHG_DTLS_SHIFT);
	bat_dtls = ((dtls_01 & MAX77665_BAT_DTLS) >>
			MAX77665_BAT_DTLS_SHIFT);
	byp_dtls = ((dtls_02 & MAX77665_BYP_DTLS) >>
			MAX77665_BYP_DTLS_SHIFT);
	pr_info("INT_OK(0x%x), CHGIN(0x%x), CHG(0x%x), BAT(0x%x),BYP_DTLS(0x%02x)\n",
			int_ok, chgin_dtls,\
			chg_dtls, bat_dtls, byp_dtls);\
		return ret;
}

static void max77665_second_adjust_current(struct work_struct *work)
{
	struct max77665_charger *charger = container_of((struct delayed_work*)work,
			struct max77665_charger, ad_work);
	struct i2c_client *i2c = charger->iodev->i2c;
	int now_cur, ret;
	u8 int_ok = 0;

	do{
		now_cur = regulator_get_current_limit(charger->ps);
		
		pr_info("%s:now_cur = %d\n",__func__, now_cur);
		regulator_set_current_limit(charger->ps, 
				now_cur - CURRENT_INCREMENT_STEP*MA_TO_UA,
				now_cur);
		
		ret = max77665_read_reg(i2c, MAX77665_CHG_REG_CHG_INT_OK, 
				&int_ok);
		if (now_cur < CHGIN_USB_CURRENT * MA_TO_UA)
			break;
	} while (int_ok != 0x5d);
	if (now_cur < CHGIN_USB_CURRENT * MA_TO_UA) {
		msleep(100);
		ret = max77665_read_reg(i2c, MAX77665_CHG_REG_CHG_INT_OK, 
				&int_ok);
		if (int_ok != 0x5d) {
			charger->done = false;
			charger->adc_flag = false;
			charger->cable_status = CABLE_TYPE_NONE;
			power_supply_changed(&charger->psy_usb);
			power_supply_changed(&charger->psy_ac);
			return;
		}
	}
}

static void max77665_chgin_irq_handler(struct work_struct *work)
{
	struct max77665_charger *charger = container_of(work,
			struct max77665_charger, chgin_work);
	struct i2c_client *i2c = charger->iodev->i2c;
	u8 int_ok, prev_int_ok;
	int ret;
	int chgin = 0, now_current = 0;
	int adc_value = 0;

	pr_info("ENTER %s##################\n", __func__);
	if (regulator_is_enabled(charger->reverse)) {
		pr_info("usb host insert, dismiss this isr\n");
		return;
	}
	
	wake_lock(&charger->wake_lock);

	ret =  max77665_read_reg(i2c, MAX77665_CHG_REG_CHG_INT_OK,
			&prev_int_ok);
	if (unlikely(ret < 0)) {
		pr_err("Failed to read MAX77665_CHG_REG_CHG_INT: %d\n", ret);
		wake_unlock(&charger->wake_lock);
	}
	msleep(20);
	ret =  max77665_read_reg(i2c, MAX77665_CHG_REG_CHG_INT_OK,
			&int_ok);
	if (unlikely(ret < 0)) {
		pr_err("Failed to read MAX77665_CHG_REG_CHG_INT: %d\n", ret);
		wake_unlock(&charger->wake_lock);
	}
	if ((charger->irq_reg == int_ok) && (prev_int_ok != int_ok)) {
		pr_info("%s:irq debounced(0x%x, 0x%x, 0x%x),return\n",
				__func__, charger->irq_reg,
				prev_int_ok, int_ok);
		wake_unlock(&charger->wake_lock);
	} else {
		chgin = !!(int_ok & 0x40);
	}
	
	max77665_reg_dump(charger);
	charger->irq_reg = int_ok;
	
	pr_info("-----%s %s\n", __func__, chgin ? "insert" : "remove");
	
	if ((!chgin) && (charger->cable_status == CABLE_TYPE_USB) 
			&& (adc_is_available(charger))) {
		charger->chgin = chgin;		
		charger->adc_flag = true; 
		
		now_current = regulator_get_current_limit(charger->ps);
		do {
			now_current -= CURRENT_INCREMENT_STEP*MA_TO_UA;
		
			pr_info("adc_is_available, now_current= %d \n", now_current);
			regulator_set_current_limit(charger->ps, 
					now_current,
					now_current + CURRENT_INCREMENT_STEP*MA_TO_UA);
			max77665_read_reg(i2c, MAX77665_CHG_REG_CHG_INT_OK,
					&int_ok);
			msleep(50);
			adc_value = s3c_adc_read(charger->adc, CHG_ADC_CHANNEL);
		
			if ((int_ok == 0X5d) && (adc_value >= 2048))
				break;
		} while (now_current > CHGIN_USB_CURRENT * MA_TO_UA);
		
		if (now_current < CHGIN_USB_CURRENT * MA_TO_UA) {
			msleep(100);
			max77665_read_reg(i2c, MAX77665_CHG_REG_CHG_INT_OK,
							&int_ok);
			if (int_ok!= 0x5d) {
				charger->done = false;
				charger->adc_flag = false;
				charger->cable_status = CABLE_TYPE_NONE;
				power_supply_changed(&charger->psy_usb);
				power_supply_changed(&charger->psy_ac);
			}
		}
		return;
	}
	if ((!chgin) && (charger->cable_status == CABLE_TYPE_AC)
			&& (adc_is_available(charger))) {
		charger->adc_flag = true;
		msleep(50);
		if (charger->done == true) {
			pr_info("adjust current done\n");
			now_current = regulator_get_current_limit(charger->ps);
			do{
				now_current -= CURRENT_INCREMENT_STEP * MA_TO_UA;
				regulator_set_current_limit(charger->ps,
						now_current,
						now_current + CURRENT_INCREMENT_STEP*MA_TO_UA);
				msleep(20);
				max77665_read_reg(i2c, MAX77665_CHG_REG_CHG_INT_OK,
						&int_ok);
				pr_info("########now_current = %d\n", now_current);
				if (int_ok == 0X5d)
					break;
			} while (now_current > CHGIN_USB_CURRENT * MA_TO_UA);
		}			
	}
	
	if (charger->chgin != chgin) {
		alarm_cancel(&charger->alarm);
		if(chgin)
			set_alarm(charger, WAKE_ALARM_INT);
		charger->chgin = chgin;		
		charger->chg_status = CHG_STATUS_FAST;
	
		if (delayed_work_pending(&charger->dwork))
			cancel_delayed_work(&charger->dwork);
			
		schedule_delayed_work(&charger->dwork, HZ/4);
	} else {
		if ((charger->cable_status == CABLE_TYPE_USB)
				&& (charger->adc_flag == true)) {
			charger->done = false;
			charger->adc_flag = false;
			if (!charger->chgin)
				charger->cable_status = CABLE_TYPE_NONE;
			power_supply_changed(&charger->psy_ac);
			power_supply_changed(&charger->psy_usb);
		}
		pr_info("unstable charger isr, dismiss this isr\n");
		msleep(2000);
		wake_unlock(&charger->wake_lock);
	}
}

static irqreturn_t max77665_charger_isr(int irq, void *dev_id)
{
	struct max77665_charger *charger = dev_id;
	
	schedule_work_on(0, &charger->chgin_work);
	return IRQ_HANDLED;
}

#define MAX77665_BYP_DTLS3 0x08
static irqreturn_t max77665_bypass_irq(int irq, void *dev_id)
{
	struct max77665_charger *charger = dev_id;
	struct i2c_client *i2c = charger->iodev->i2c;
	u8 reg_data = 0, bypass_flag, int_ok;
	int ret;

	wake_lock(&charger->wake_lock);

	pr_info("###################ENTER %s\n", __func__);
	max77665_reg_dump(charger);

	ret = max77665_read_reg(i2c, MAX77665_CHG_REG_CHG_DETAILS_02,
			&reg_data);
	if (unlikely(ret < 0)) {
		pr_err("Failed to read MAX77665_CHG_REG_CHG_DETAILS_02: %d\n", ret);
		wake_unlock(&charger->wake_lock);
		return IRQ_HANDLED;
	} else { 
		bypass_flag = reg_data & 0x0f;
	}
	/*cancel the chgin interrupt*/
	cancel_delayed_work(&charger->dwork);

	switch(bypass_flag){
	case MAX77665_BYP_DTLS3: /*VCHGINLIM*/
		pr_info("################## MAX77665_BYP_DTLS3:\n");
		complete(&charger->byp_complete);
		break;
	case 0:
		pr_info("######################BYPASS OK!\n");
		ret = max77665_read_reg(i2c,
				MAX77665_CHG_REG_CHG_INT_OK,
				&int_ok);
		if (int_ok != 0x5d) 
			queue_delayed_work(charger->ad_curr, &charger->ad_work, HZ/50);
		break;
	default:
		break;
	}
	wake_unlock(&charger->wake_lock);
	return IRQ_HANDLED; 
}

static __devinit int max77665_init(struct max77665_charger *charger)
{
	struct max77665_dev *iodev = dev_get_drvdata(charger->dev->parent);
	struct max77665_platform_data *pdata = dev_get_platdata(iodev->dev);	
	struct i2c_client *i2c = iodev->i2c;
	int ret = EINVAL;
	u8 reg_data = 0;

	/* Unlock protected registers */
	ret = max77665_write_reg(i2c, MAX77665_CHG_REG_CHG_CNFG_06, 0x0C);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX8957_REG_CHG_CNFG_06: %d\n", ret);
		goto error;
	}

	reg_data = max((u8)MAX77665_FCHGTIME_4H, (u8)pdata->fast_charge_timer);
	ret = max77665_update_reg(i2c, MAX77665_CHG_REG_CHG_CNFG_01, reg_data, 0x7<<0);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX77665_CHG_REG_CHG_CNFG_01: %d\n", ret);
		goto error;
	}

	reg_data = max((u8)MAX77665_CHG_RSTRT_100MV, (u8)pdata->charging_restart_thresold);
	reg_data <<= 4;
	ret = max77665_update_reg(i2c, MAX77665_CHG_REG_CHG_CNFG_01, reg_data, 0x3<<4);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX77665_CHG_REG_CHG_CNFG_01: %d\n", ret);
		goto error;
	}

	reg_data = 1<<7;
	ret = max77665_update_reg(i2c, MAX77665_CHG_REG_CHG_CNFG_01, reg_data, 0x1<<7);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX77665_CHG_REG_CHG_CNFG_01: %d\n", ret);
		goto error;
	}

	reg_data = (min(MAX_AC_CURRENT, pdata->fast_charge_current) /CHG_CC_STEP);
	ret = max77665_update_reg(i2c, MAX77665_CHG_REG_CHG_CNFG_02, reg_data, 0x3f<<0);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX77665_CHG_REG_CHG_CNFG_02: %d\n", ret);
		goto error;
	}

	reg_data = 1<<7;
	ret = max77665_update_reg(i2c, MAX77665_CHG_REG_CHG_CNFG_02, reg_data, 0x1<<7);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX77665_CHG_REG_CHG_CNFG_02: %d\n", ret);
		goto error;
	}

	reg_data = max((u8)MAX77665_CHG_TO_ITH_100MA, (u8)pdata->top_off_current_thresold);
	ret = max77665_update_reg(i2c, MAX77665_CHG_REG_CHG_CNFG_03, reg_data, 0x7<<0);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX77665_CHG_REG_CHG_CNFG_03: %d\n", ret);
		goto error;
	}

	reg_data = max((u8)MAX77665_CHG_TO_TIME_10MIN, (u8)pdata->top_off_current_thresold);
	reg_data <<= 3;
	ret = max77665_update_reg(i2c, MAX77665_CHG_REG_CHG_CNFG_03, reg_data, 0x7<<3);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX77665_CHG_REG_CHG_CNFG_03: %d\n", ret);
		goto error;
	}

	reg_data = max((u8)MAX77665_CHG_CV_PRM_4200MV, (u8)pdata->charger_termination_voltage);
	ret = max77665_update_reg(i2c, MAX77665_CHG_REG_CHG_CNFG_04, reg_data, 0x1f<<0);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX77665_CHG_REG_CHG_CNFG_04: %d\n", ret);
		goto error;
	}

	/* Lock protected registers */
	ret = max77665_write_reg(i2c, MAX77665_CHG_REG_CHG_CNFG_06, 0x00);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX8957_REG_CHG_CNFG_06: %d\n", ret);
		goto error;
	}

	/* disable muic ctrl */
	reg_data = 1<<5;
	ret = max77665_update_reg(i2c, MAX77665_CHG_REG_CHG_CNFG_00, reg_data, 0x1<<5);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX77665_CHG_REG_CHG_CNFG_00: %d\n", ret);
		goto error;
	}
	return 0;

error:
	return ret;
}

static __devinit int max77665_charger_probe(struct platform_device *pdev)
{
	struct max77665_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct max77665_platform_data *pdata = dev_get_platdata(iodev->dev);	
	struct max77665_charger *charger;
	u8 reg_data;
	int ret = EINVAL;

	charger = kzalloc(sizeof(struct max77665_charger), GFP_KERNEL);
	if (unlikely(!charger))
		return -ENOMEM;

	platform_set_drvdata(pdev, charger);

	charger->iodev = iodev;
	charger->dev = &pdev->dev;
	charger->usb_attach = pdata->usb_attach;
	charger->chgin_ilim_usb = pdata->chgin_ilim_usb;
	charger->chgin_ilim_ac = pdata->chgin_ilim_ac;
	charger->chr_pin = pdata->charger_pin;
	charger->done = false;
	charger->adc_flag = false;

	init_completion(&charger->byp_complete);
	mutex_init(&charger->mutex_t);
	wake_lock_init(&charger->wake_lock, WAKE_LOCK_SUSPEND, pdata->name);
#if defined(CONFIG_MX_RECOVERY_KERNEL)
	wake_lock_init(&charger->charge_wake_lock, WAKE_LOCK_SUSPEND, "charge_wake_lock");
#endif
	INIT_DELAYED_WORK(&charger->dwork, max77665_work_func);
	INIT_DELAYED_WORK(&charger->poll_dwork, max77665_poll_work_func);
	charger->ad_curr = create_singlethread_workqueue("max77665_sed_ad_curr");
	INIT_DELAYED_WORK(&charger->ad_work,max77665_second_adjust_current);
	INIT_WORK(&charger->chgin_work, max77665_chgin_irq_handler);

	charger->ps = regulator_get(charger->dev, pdata->supply);
	if (IS_ERR(charger->ps)) {
		dev_err(&pdev->dev, "Failed to regulator_get ps: %ld\n", 
				PTR_ERR(charger->ps));
		goto err_free;
	}
	
	charger->reverse = regulator_get(NULL, "reverse");
	if (IS_ERR(charger->reverse)) {
		dev_err(&pdev->dev, "Failed to regulator_get reverse: %ld\n",
				PTR_ERR(charger->reverse));
		goto err_put0;
	}

	charger->adc = s3c_adc_register(pdev, NULL, NULL, 0);
	if (IS_ERR(charger->adc)) {
		dev_err(&pdev->dev, "cannot register adc\n");
		ret = PTR_ERR(charger->adc);
		goto err_put;
	}

	charger->psy_charger.name = "charger";
	charger->psy_charger.type = POWER_SUPPLY_TYPE_BATTERY;
	charger->psy_charger.properties = max77665_charger_props,
	charger->psy_charger.num_properties = ARRAY_SIZE(max77665_charger_props),
	charger->psy_charger.get_property = max77665_charger_get_property,
	ret = power_supply_register(&pdev->dev, &charger->psy_charger);
	if (unlikely(ret != 0)) {
		dev_err(&pdev->dev, "Failed to power_supply_register psy_charger: %d\n", ret);
		goto err_adc_unregister;
	}

	charger->psy_usb.name = "usb";
	charger->psy_usb.type = POWER_SUPPLY_TYPE_USB;
	charger->psy_usb.supplied_to = supply_list,
	charger->psy_usb.num_supplicants = ARRAY_SIZE(supply_list),
	charger->psy_usb.properties = max77665_power_props,
	charger->psy_usb.num_properties = ARRAY_SIZE(max77665_power_props),
	charger->psy_usb.get_property = max77665_usb_get_property,
	ret = power_supply_register(&pdev->dev, &charger->psy_usb);
	if (unlikely(ret != 0)) {
		dev_err(&pdev->dev, "Failed to power_supply_register psy_usb: %d\n", ret);
		goto err_unregister0;
	}

	charger->psy_ac.name = "ac";
	charger->psy_ac.type = POWER_SUPPLY_TYPE_MAINS;
	charger->psy_ac.supplied_to = supply_list,
	charger->psy_ac.num_supplicants = ARRAY_SIZE(supply_list),
	charger->psy_ac.properties = max77665_power_props,
	charger->psy_ac.num_properties = ARRAY_SIZE(max77665_power_props),
	charger->psy_ac.get_property = max77665_ac_get_property,
	ret = power_supply_register(&pdev->dev, &charger->psy_ac);
	if (unlikely(ret != 0)) {
		dev_err(&pdev->dev, "Failed to power_supply_register psy_ac: %d\n", ret);
		goto err_unregister1;
	}
	
	ret = max77665_init(charger);

	alarm_init(&charger->alarm, ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
				charger_bat_alarm);

	ret = max77665_read_reg(iodev->i2c, MAX77665_CHG_REG_CHG_INT_OK, &reg_data);
	if (unlikely(ret < 0))
		pr_err("Failed to read MAX77665_CHG_REG_CHG_INT: %d\n", ret);
	else {
		if (!(regulator_is_enabled(charger->reverse)) 
				&& (reg_data & 0x40)) {	// CHGIN
			charger->chgin = true;
			if (is_charging_mode()) {
				charger->cable_status = CABLE_TYPE_USB;
				schedule_delayed_work_on(0, &charger->dwork, msecs_to_jiffies(5 * 1000));
			} else {
				schedule_delayed_work_on(0, &charger->dwork, 0);
			}
			set_alarm(charger, WAKE_ALARM_INT);
		}
	}

	charger->chgin_irq = pdata->irq_base + MAX77665_CHG_IRQ_CHGIN_I;
	ret = request_threaded_irq(charger->chgin_irq, 0, max77665_charger_isr,
			0, pdev->name, charger);
	if (unlikely(ret < 0)) {
		dev_err(&pdev->dev, "max77665: failed to request CHGIN IRQ %d\n", charger->chgin_irq);
		goto err_unregister2;
	}

	charger->bypass_irq = pdata->irq_base + MAX77665_CHG_IRQ_BYP_I;
	ret = request_threaded_irq(charger->bypass_irq, NULL,
			max77665_bypass_irq, 0, "bypass-irq", charger);
	if (unlikely(ret < 0)) {
		dev_err(&pdev->dev, "max77665: failed to request BYPASS IRQ %d\n", charger->chgin_irq);
		goto err_unregister2;
	}
	
	return 0;

err_unregister2:
	alarm_cancel(&charger->alarm);
	power_supply_unregister(&charger->psy_ac);
err_unregister1:	
	power_supply_unregister(&charger->psy_usb);
err_unregister0:	
	power_supply_unregister(&charger->psy_charger);
err_adc_unregister:
	s3c_adc_release(charger->adc);
err_put:
	regulator_put(charger->reverse);
err_put0:
	regulator_put(charger->ps);
err_free:
#if defined(CONFIG_MX_RECOVERY_KERNEL)
	wake_lock_destroy(&charger->charge_wake_lock);
#endif
	wake_lock_destroy(&charger->wake_lock);
	platform_set_drvdata(pdev, NULL);
	kfree(charger);
	return ret;
}

static __devexit int max77665_charger_remove(struct platform_device *pdev)
{
	struct max77665_charger *charger = platform_get_drvdata(pdev);

	alarm_cancel(&charger->alarm);
	cancel_delayed_work_sync(&charger->poll_dwork);

	free_irq(charger->chgin_irq, charger);
	free_irq(charger->bypass_irq, charger);
	regulator_put(charger->reverse);
	regulator_put(charger->ps);
	power_supply_unregister(&charger->psy_usb);
	power_supply_unregister(&charger->psy_ac);
	platform_set_drvdata(pdev, NULL);
	kfree(charger);
	return 0;
}

static void max77665_shutdown(struct platform_device *pdev)
{
	struct max77665_charger *charger = platform_get_drvdata(pdev);
	if(regulator_is_enabled(charger->reverse))
		regulator_disable(charger->reverse);
}

#ifdef CONFIG_PM
static int max77665_suspend(struct device *dev)
{
	struct max77665_charger *charger = dev_get_drvdata(dev);

	dev_dbg(charger->dev, "%s\n", __func__);
	cancel_delayed_work_sync(&charger->poll_dwork);

	return 0;
}

static int max77665_resume(struct device *dev)
{
	struct max77665_charger *charger = dev_get_drvdata(dev);

	dev_dbg(charger->dev, "%s\n", __func__);
	schedule_delayed_work_on(0, &charger->poll_dwork, HZ);

	return 0;
}

static const struct dev_pm_ops max77665_pm_ops = {
	.suspend		= max77665_suspend,
	.resume		= max77665_resume,
};
#else
#define max77665_pm_ops NULL
#endif

static struct platform_driver max77665_charger_driver =
{
	.driver = {
		.name  = "max77665-charger", 
		.owner = THIS_MODULE,        
		.pm    = &max77665_pm_ops,   
	},
		.probe    = max77665_charger_probe,               
		.remove   = __devexit_p(max77665_charger_remove), 
		.shutdown = max77665_shutdown,                    
};

static int __init max77665_charger_init(void)
{
	return platform_driver_register(&max77665_charger_driver);
}
late_initcall(max77665_charger_init);

static void __exit max77665_charger_exit(void)
{
	platform_driver_unregister(&max77665_charger_driver);
}
module_exit(max77665_charger_exit);

MODULE_DESCRIPTION("Charger driver for MAX77665");
MODULE_AUTHOR("Lvcha qiu <lvcha@meizu.com>;Chwei <Chwei@meizu.com>");
MODULE_LICENSE("GPLV2");
