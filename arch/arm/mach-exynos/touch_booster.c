#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/cpufreq.h>
#include <linux/workqueue.h>
#include <mach/dev.h>
#include <asm/mach-types.h>
#include <plat/cpu.h>
#ifdef CONFIG_FB_DYNAMIC_FREQ
#include <linux/fb.h>
#endif
#include <mach/touch_booster.h>

#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/err.h>

#define DEFUALT_BOOST_CPUFREQ		600000//khz
#define DEFUALT_BOOST_FREETIME	1000//ms
#define DEFUALT_BOOST_4210_BUSFREQ		267000
#define DEFUALT_BOOST_4412_BUSFREQ		267160

static struct tb_private_data *tb_data=NULL;

static void start_vsync_boost(struct tb_private_data *data)
{
	int ret = 0;
	ktime_t delta_total, rettime_total;
	long long delta_us = 0;

	rettime_total = ktime_get();
#ifdef CONFIG_FB_DYNAMIC_FREQ
	do {
		struct fb_event event;
		event.data = data;
		ret = fb_notifier_call_chain(FB_EVENT_MODE_PAN, &event);
	} while(0);
#endif
	delta_total= ktime_sub(ktime_get(),rettime_total);
	delta_us = ktime_to_us(delta_total);
	if (delta_us > 1000) {
		pr_debug("start_vsync_pulse time = %Lu uS, ret = %d\n", delta_us, ret);
	}
}
static ssize_t set_vsync_pulse(struct sysdev_class *class,
				 struct sysdev_class_attribute *attr,
				 const char *buf,
				 size_t count)
{
	struct tb_private_data *data = list_entry(class, struct tb_private_data, tb_class);
	unsigned int pulse = 0;

	sscanf(buf, "%u", &pulse);
	if(pulse)
		start_vsync_boost(data);
	return count; 
}
static SYSDEV_CLASS_ATTR(vsync_pulse, 0666, NULL, set_vsync_pulse);

static void start_touch_boost(struct tb_private_data *data)
{
	int ret = 0;
	struct cpufreq_policy *policy = cpufreq_cpu_get(0);
	ktime_t delta_total, rettime_total;
	long long delta_us = 0;

	rettime_total = ktime_get();
	if (policy) {
		if (policy->cur < data->boost_cpufreq) {
			ret = cpufreq_driver_target(policy, data->boost_cpufreq, CPUFREQ_RELATION_L);
			if (!WARN(ret, "ret = %d\n", ret)) {
				pr_debug("dev_lock\n");
				dev_lock_timeout(data->bus_dev, &data->dev, data->lock_busfreq, data->down_time);
			}
		}
		cpufreq_cpu_put(policy);
		start_vsync_boost(data);
	}
	delta_total= ktime_sub(ktime_get(),rettime_total);
	delta_us = ktime_to_us(delta_total);
	if (delta_us > 1000) {
		pr_debug("start_touch_boost time = %Lu uS, ret = %d\n", delta_us, ret);
	}
}
static ssize_t set_touch_pulse(struct sysdev_class *class,
				 struct sysdev_class_attribute *attr,
				 const char *buf,
				 size_t count)
{
	struct tb_private_data *data = list_entry(class, struct tb_private_data, tb_class);
	unsigned int pulse = 0;

	sscanf(buf, "%u", &pulse);
	if(pulse)
		start_touch_boost(data);
	return count; 
}
static SYSDEV_CLASS_ATTR(touch_pulse, 0666, NULL, set_touch_pulse);

static ssize_t get_boost_cpufreq(struct sysdev_class *class,
				struct sysdev_class_attribute *attr, char *buf)
{
	struct tb_private_data *data = list_entry(class, struct tb_private_data, tb_class);
	return sprintf(buf,"%d\n", data->boost_cpufreq);
}

static ssize_t set_boost_cpufreq(struct sysdev_class *class,
				 struct sysdev_class_attribute *attr,
				 const char *buf,
				 size_t count)
{
	struct tb_private_data *data = list_entry(class, struct tb_private_data, tb_class);
	unsigned int cpurate = 0;

	sscanf(buf, "%u", &cpurate);
	data->boost_cpufreq = cpurate;

	return count; 
}

static SYSDEV_CLASS_ATTR(boost_cpufreq, 0666, get_boost_cpufreq, set_boost_cpufreq);
static ssize_t get_lock_busfreq(struct sysdev_class *class,
				struct sysdev_class_attribute *attr, char *buf)
{
	struct tb_private_data *data = list_entry(class, struct tb_private_data, tb_class);
	return sprintf(buf,"%d\n", data->lock_busfreq);
}

static ssize_t set_lock_busfreq(struct sysdev_class *class,
				 struct sysdev_class_attribute *attr,
				 const char *buf,
				 size_t count)
{
	struct tb_private_data *data = list_entry(class, struct tb_private_data, tb_class);
	unsigned int busrate = 0;

	sscanf(buf, "%u", &busrate);
	data->lock_busfreq = busrate;

	return count; 
}
static SYSDEV_CLASS_ATTR(lock_busfreq, 0666, get_lock_busfreq, set_lock_busfreq);
static ssize_t get_lock_time(struct sysdev_class *class,
				struct sysdev_class_attribute *attr, char *buf)
{
	struct tb_private_data *data = list_entry(class, struct tb_private_data, tb_class);
	return sprintf(buf,"%d ms\n", data->down_time);
}

static ssize_t set_lock_time(struct sysdev_class *class,
				 struct sysdev_class_attribute *attr,
				 const char *buf,
				 size_t count)
{
	struct tb_private_data *data = list_entry(class, struct tb_private_data, tb_class);
	unsigned int time = 0;

	sscanf(buf, "%u", &time);

	if(time<100)
		time=100;
	
	data->down_time = time;

	return count; 
}

static SYSDEV_CLASS_ATTR(lock_time, 0666, get_lock_time, set_lock_time);

static struct sysdev_class_attribute *tb_sysdev_class_attrs[] = {
	&attr_vsync_pulse,
	&attr_touch_pulse,
	&attr_boost_cpufreq,
	&attr_lock_busfreq,
	&attr_lock_time,
	NULL
};
static int __init tb_init(void)
{
	int err;
	struct tb_private_data *data;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (IS_ERR_OR_NULL(data))
		return PTR_ERR(data);

	data->bus_dev = dev_get("exynos-busfreq");

	device_initialize(&data->dev);
	dev_set_name(&data->dev, "touchbooster");
	err = device_add(&data->dev);
	if (err < 0) {
		printk(KERN_ERR "can't %s %s, status %d\n",
				"add", dev_name(&data->dev), err);
		kfree(data);
		return err;
	}

	data->tb_class.name = "touchbooster";
	data->tb_class.attrs = tb_sysdev_class_attrs;
	err = sysdev_class_register(&data->tb_class);
	if (err) {
		device_del(&data->dev);
		kfree(data);
		pr_err("%s: register sysdev performance erro!\n", __func__);
		return err;
	}
	data->down_time = DEFUALT_BOOST_FREETIME;//ms
	data->boost_cpufreq = DEFUALT_BOOST_CPUFREQ;
	if (soc_is_exynos4210())
		data->lock_busfreq = DEFUALT_BOOST_4210_BUSFREQ;
	else
		data->lock_busfreq = DEFUALT_BOOST_4412_BUSFREQ;
	tb_data = data;
	return 0;
}

static void __exit tb_exit(void)
{
	device_del(&tb_data->dev);
	kfree(tb_data);
	tb_data=NULL;
}

subsys_initcall(tb_init);
module_exit(tb_exit);

/* Module information */
MODULE_AUTHOR("Lvcha qiu <lvcha@meizu.com>");
MODULE_AUTHOR("Wenbin Wu <wenbinwu@meizu.com>");
MODULE_DESCRIPTION("Touch Boost driver");
MODULE_LICENSE("GPL");

