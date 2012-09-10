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

static struct tb_private_data *tb_data=NULL;

static inline int tb_input_boost(struct tb_private_data *data)
{
	struct cpufreq_policy *policy = cpufreq_cpu_get(0);
	if (policy) {
		if (policy->cur < data->lock_cpufreq) {
			int ret;
			ret = cpufreq_driver_target(policy, data->lock_cpufreq, CPUFREQ_RELATION_L);
			if (!WARN(ret, "ret = %d\n", ret)) {
				pr_debug("dev_lock\n");
				dev_lock_timeout(data->bus_dev, &data->dev, data->lock_busfreq, data->down_time);
			}
		}
		cpufreq_cpu_put(policy);
#ifdef CONFIG_FB_DYNAMIC_FREQ
		do {
			struct fb_event event;
			event.data = data;
			fb_notifier_call_chain(FB_EVENT_MODE_PAN, &event);
		} while(0);
#endif
	}
}

void start_touch_boost(void)
{
	ktime_t delta_total, rettime_total;
	long long delta_us = 0;
	int ret;
	rettime_total = ktime_get();
	if(tb_data)
		ret= tb_input_boost(tb_data);
	delta_total= ktime_sub(ktime_get(),rettime_total);
	delta_us = ktime_to_us(delta_total);
	if (delta_us > 1000) {
		pr_debug("tb_input_boost time = %Lu uS, ret = %d\n", delta_us, ret);
	}
}
EXPORT_SYMBOL(start_touch_boost);

static ssize_t get_lock_cpufreq(struct sysdev_class *class,
				struct sysdev_class_attribute *attr, char *buf)
{
	struct tb_private_data *data = list_entry(class, struct tb_private_data, tb_class);
	return sprintf(buf,"%d\n", data->lock_cpufreq);
}

static ssize_t set_lock_cpufreq(struct sysdev_class *class,
				 struct sysdev_class_attribute *attr,
				 const char *buf,
				 size_t count)
{
	struct tb_private_data *data = list_entry(class, struct tb_private_data, tb_class);
	unsigned int cpurate = 0;

	sscanf(buf, "%lu", &cpurate);
	data->lock_cpufreq = cpurate;
out:
	return count; 
}

static SYSDEV_CLASS_ATTR(lock_cpufreq, 0666, get_lock_cpufreq, set_lock_cpufreq);
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

	sscanf(buf, "%lu", &busrate);
	data->lock_busfreq = busrate;

out:
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
	int err = -EINVAL;

	sscanf(buf, "%lu", &time);

	if(time<100)
		time=100;
	
	data->down_time = time;
out:
	return count; 
}

static SYSDEV_CLASS_ATTR(lock_time, 0666, get_lock_time, set_lock_time);

static struct sysdev_class_attribute *tb_sysdev_class_attrs[] = {
	&attr_lock_cpufreq,
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

#ifdef CONFIG_BUSFREQ_OPP
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
#endif
	data->tb_class.name = "touchbooster";
	data->tb_class.attrs = tb_sysdev_class_attrs;
	err = sysdev_class_register(&data->tb_class);
	if (err) {
#ifdef CONFIG_BUSFREQ_OPP
		device_del(&data->dev);
#endif
		kfree(data);
		pr_err("%s: register sysdev performance erro!\n", __func__);
		return err;
	}
	data->down_time = 1000;//ms
	data->lock_cpufreq = 600000;
	if (soc_is_exynos4210())
		data->lock_busfreq = 267000;
	else
		data->lock_busfreq = 267160;
	tb_data = data;
	return 0;
}

static void __exit tb_exit(void)
{
#ifdef CONFIG_BUSFREQ_OPP
	device_del(&tb_data->dev);
#endif
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

