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
#include <linux/pm_qos.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/err.h>

static int boost_time_multi[TABLE_SIZE] = {2, 1, 1};
static int boost_freq_table[TABLE_SIZE] = {DEFUALT_LAUNCH_BOOST_CPUFREQ, DEFUALT_FIRST_BOOST_CPUFREQ, DEFUALT_SECOND_BOOST_CPUFREQ};
static struct pm_qos_request boost_cpu_qos;
static int boost_device_count = 0;
unsigned int app_into = 0;
static struct pm_qos_request booster_cpu_qos;

static void start_vsync_boost(struct tb_private_info *info)
{
	int ret = 0;

#ifdef CONFIG_FB_DYNAMIC_FREQ
	do {
		struct fb_event event;
		event.data = info;
		ret = fb_notifier_call_chain(FB_EVENT_MODE_PAN, &event);
	} while(0);
#endif
}
static ssize_t set_vsync_pulse(struct class *class,
				 struct class_attribute *attr,
				 const char *buf,
				 size_t count)
{
	struct tb_private_info *info = list_entry(class, struct tb_private_info, tb_class);
	unsigned int pulse = 0;

	sscanf(buf, "%u", &pulse);
	if(pulse)
		start_vsync_boost(info);
	return count;
}

static void start_touch_boost(struct tb_private_info *info)
{

	unsigned int cur, target, target_level;
	unsigned int boost_time;

	struct cpufreq_policy *policy = cpufreq_cpu_get(0);
	if (policy) {
		cur = policy->cur;
		cpufreq_cpu_put(policy);

		if (info->boost_debug)
			pr_info("%s ... current freq = %d\n", __func__, policy->cur);

		target_level = info->boost_cpufreq_level;
		app_into = info->boost_cpufreq_level;
		target = boost_freq_table[target_level];

		if (cur < target) {
			boost_time = info->down_time * boost_time_multi[target_level];
			pm_qos_update_request_timeout(&booster_cpu_qos, target, boost_time);
			cpufreq_driver_target(policy, target, CPUFREQ_RELATION_L);
#ifdef CONFIG_BUSFREQ_OPP
			dev_lock_timeout(info->bus_dev, &info->dev, info->lock_busfreq, boost_time / 1000);
#endif
			if (info->boost_debug)
				pr_info("%s: request %d cpu freq for %d msecs\n", __func__, target, boost_time);
		}
	}
}
static ssize_t set_touch_pulse(struct class *class,
				 struct class_attribute *attr,
				 const char *buf,
				 size_t count)
{
	struct tb_private_info *info = list_entry(class, struct tb_private_info, tb_class);
	unsigned int pulse = 0;

	sscanf(buf, "%u", &pulse);
	if(pulse)
		start_touch_boost(info);
	return count;
}

static ssize_t get_boost_level(struct class *class,
				struct class_attribute *attr, char *buf)
{
	struct tb_private_info *data = list_entry(class, struct tb_private_info, tb_class);
	return sprintf(buf, "%d\n", data->boost_cpufreq_level);
}

static ssize_t set_boost_level(struct class *class,
				 struct class_attribute *attr,
				 const char *buf,
				 size_t count)
{
	struct tb_private_info *info = list_entry(class, struct tb_private_info, tb_class);
	unsigned int cpurate = 0;

	sscanf(buf, "%u", &cpurate);
	if (cpurate < TABLE_SIZE) {
		info->boost_cpufreq_level = cpurate;
		app_into = info->boost_cpufreq_level;
	}
	else
		pr_info("level not in range (0~2)\n");

	return count;
}

static ssize_t get_boost_debug(struct class *class,
				struct class_attribute *attr, char *buf)
{
	struct tb_private_info *info = list_entry(class, struct tb_private_info, tb_class);
	return sprintf(buf,"%d\n", info->boost_debug);
}

static ssize_t set_boost_debug(struct class *class,
				 struct class_attribute *attr,
				 const char *buf,
				 size_t count)
{
	struct tb_private_info *info = list_entry(class, struct tb_private_info, tb_class);
	unsigned int boost_debug = 0;

	sscanf(buf, "%u", &boost_debug);
	info->boost_debug = !!boost_debug;

	return count;
}

static ssize_t get_lock_busfreq(struct class *class,
				struct class_attribute *attr, char *buf)
{
	struct tb_private_info *info = list_entry(class, struct tb_private_info, tb_class);
	return sprintf(buf,"%d\n", info->lock_busfreq);
}

static ssize_t set_lock_busfreq(struct class *class,
				 struct class_attribute *attr,
				 const char *buf,
				 size_t count)
{
	struct tb_private_info *info = list_entry(class, struct tb_private_info, tb_class);
	unsigned int busrate = 0;

	sscanf(buf, "%u", &busrate);
	info->lock_busfreq = busrate;

	return count;
}

static ssize_t get_lock_time(struct class *class,
				struct class_attribute *attr, char *buf)
{
	struct tb_private_info *info = list_entry(class, struct tb_private_info, tb_class);
	return sprintf(buf,"%d us\n", info->down_time);
}

static ssize_t set_lock_time(struct class *class,
				 struct class_attribute *attr,
				 const char *buf,
				 size_t count)
{
	struct tb_private_info *info = list_entry(class, struct tb_private_info, tb_class);
	unsigned int time = 0;

	sscanf(buf, "%u", &time);

	if(time<100)
		time=100;

	info->down_time = time;

	return count;
}

static struct class_attribute tb_class_attrs[] = {
	__ATTR(boost_debug, 0660, get_boost_debug, set_boost_debug),
	__ATTR(vsync_pulse, 0660, NULL, set_vsync_pulse),
	__ATTR(touch_pulse, 0660, NULL, set_touch_pulse),
	__ATTR(boost_level, 0660, get_boost_level, set_boost_level),
	__ATTR(lock_busfreq, 0660, get_lock_busfreq, set_lock_busfreq),
	__ATTR(lock_time, 0660, get_lock_time, set_lock_time),
	__ATTR_NULL
};

static void tb_boost_fn(struct work_struct *work)
{
	struct tb_private_info *info=
		list_entry(work, struct tb_private_info, boost_work);
	start_touch_boost(info);
}

static void tb_event(struct input_handle *handle,
	unsigned int type, unsigned int code, int value)
{
	struct tb_private_info *info = (struct tb_private_info*)handle->private;

	/* To queue work when touch device and press only */
	if (code == BTN_TOUCH && value == 1)
		queue_work(info->wq, &info->boost_work);
	if (code == KEY_POWER && value == 1)
		queue_work(info->wq, &info->boost_work);
	if (code == KEY_HOME && value == 1)
		queue_work(info->wq, &info->boost_work);
	if (code == KEY_AGAIN && value == 1)
		queue_work(info->wq, &info->boost_work);
	if (code == ABS_DISTANCE && value == 2)
		queue_work(info->wq, &info->boost_work);
}
static bool  tb_match(struct input_handler *handler, struct input_dev *dev)
{
	/* Avoid touchpads and touchscreens */
	if (test_bit(EV_ABS, dev->evbit) && test_bit(BTN_TOUCH, dev->keybit))
		return true;
	/* Avoid gpio keyboard */
	if (test_bit(EV_KEY, dev->evbit) && test_bit(KEY_POWER, dev->keybit) && dev->id.vendor == 0x0001)
		return true;
	/* Avoid touchpad */
	if (test_bit(EV_KEY, dev->evbit) && test_bit(KEY_HOME, dev->keybit) && dev->id.vendor ==0x1111)
		return true;
	/* Avoid ir input */
	if (test_bit(EV_ABS, dev->evbit) && test_bit(ABS_DISTANCE, dev->absbit))
		return true;
	return false;
}
static int tb_connect(struct input_handler *handler,
				    struct input_dev *dev,
				    const struct input_device_id *id)
{
	int ret;
	int num;
	struct tb_private_info *info = handler->private;

	if(boost_device_count >= MAX_BOOST_DEVICE)
		return -ENODEV;
	num = boost_device_count;
	info->handle[num].dev = dev;
	info->handle[num].handler = handler;
	info->handle[num].open = 0;
	info->handle[num].name = "touch_booster";
	info->handle[num].private = info;
	ret = input_register_handle(&info->handle[num]);
	if (ret) {
		pr_err("%s: register input handler error!\n", __func__);
		goto err_reg;
	}
	ret = input_open_device(&info->handle[num]);
	if (ret) {
		pr_err("%s: Failed to open input device, error %d\n",
			__func__, ret);
		goto err_open;
	}

	if (boost_device_count == 0) {
		pm_qos_add_request(&boost_cpu_qos, PM_QOS_CPUFREQ_MIN, 0);
	}

	boost_device_count++;
	dev_info(&info->dev, "Register input handler for %s\n", dev_name(&dev->dev));
	return 0;

err_open:
	input_unregister_handle(&info->handle[num]);
err_reg:
	return ret;


}

static void tb_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	boost_device_count--;
}

static const struct input_device_id tb_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_KEYBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.keybit = {[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
	},{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_KEYBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = {[BIT_WORD(ABS_DISTANCE)] = BIT_MASK(ABS_DISTANCE) },
	},{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT  | INPUT_DEVICE_ID_MATCH_KEYBIT,
		.evbit = { BIT_MASK(EV_KEY) },
		.keybit = {[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER) },
	},{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT  | INPUT_DEVICE_ID_MATCH_KEYBIT,
		.evbit = { BIT_MASK(EV_KEY) },
		.keybit = {[BIT_WORD(KEY_HOME)] = BIT_MASK(KEY_HOME) },
	},
	{ }	/* Terminating entry */
};

static struct input_handler tb_handler = {
	.event		= tb_event,
	.match		= tb_match,
	.connect		=  tb_connect,
	.disconnect	= tb_disconnect,
	.name		= "touch_booster",
	.id_table		= tb_ids,
};

static int __init tb_init(void)
{
	int ret ;
	struct tb_private_info *info;

	info = kzalloc(sizeof(struct tb_private_info), GFP_KERNEL);
	if (IS_ERR_OR_NULL(info))
		return PTR_ERR(info);

	info->bus_dev = dev_get("exynos-busfreq");
	info->wq = create_singlethread_workqueue("touch_booster");
	if (!info->wq) {
		pr_err("%s: Failed to create_singlethread_workqueue\n", __func__);
		ret = -ENOMEM;
		goto  err_create;
	}

	INIT_WORK(&info->boost_work, tb_boost_fn);
	info->key_time= ktime_get();
	info->down_time = DEFUALT_BOOST_FREETIME;//ms
	info->boost_cpufreq_level = 0;
	app_into = 0;
#ifdef CONFIG_BUSFREQ_OPP
	if (soc_is_exynos4210())
		info->lock_busfreq = DEFUALT_BOOST_4210_BUSFREQ;
	else
		info->lock_busfreq = DEFUALT_BOOST_4412_BUSFREQ;

	device_initialize(&info->dev);
	dev_set_name(&info->dev, "touch_booster");
	ret = device_add(&info->dev);
	if (ret < 0) {
		printk(KERN_ERR "can't %s %s, status %d\n",
				"add", dev_name(&info->dev), ret);
		goto err_device;
	}
#endif

	info->tb_class.name = "touch_booster";
	info->tb_class.class_attrs = tb_class_attrs;

	ret = class_register(&info->tb_class);
	if (ret) {
		pr_err("%s: register sysdev performance error!\n", __func__);
		goto err_sysdev;
	}

	tb_handler.private = info;
	ret = input_register_handler(&tb_handler);
	if (ret) {
		pr_err("Unable to register input handler, error: %d\n",
			ret);
		goto err_handler;
	}
	pm_qos_add_request(&booster_cpu_qos, PM_QOS_CPUFREQ_MIN, 0);
	return 0;
err_handler:
	input_unregister_handler(&tb_handler);
#ifdef CONFIG_BUSFREQ_OPP
err_sysdev:
	device_del(&info->dev);
#endif
err_device:
	destroy_workqueue(info->wq);
err_create:
	kfree(info);
	return ret;
}

static void __exit tb_exit(void)
{
	struct tb_private_info *info = tb_handler.private;

	pm_qos_remove_request(&booster_cpu_qos);
	input_unregister_handler(&tb_handler);
#ifdef CONFIG_BUSFREQ_OPP
	device_del(&info->dev);
#endif
	destroy_workqueue(info->wq);
	kfree(info);
}

subsys_initcall(tb_init);
module_exit(tb_exit);

/* Module information */
MODULE_AUTHOR("Lvcha qiu <lvcha@meizu.com>");
MODULE_AUTHOR("Wenbin Wu <wenbinwu@meizu.com>");
MODULE_DESCRIPTION("Touch Boost driver");
MODULE_LICENSE("GPL");

