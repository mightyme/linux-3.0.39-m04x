/* drivers/base/performance.c
 *
 * Copyright (C) 2012 ZhuHai MEIZU Technology Co., Ltd.
 *	  http://www.meizu.com
 *	Li Tao <litao@meizu.com>
 *
 * Performance info sysfs interface
 *
 * cpufreq_pfm_id: get and set cpufreq performance level(0~2).
 * 0--->High, 1--->med, 2--->low
 */

#include <linux/slab.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/topology.h>
#include <linux/device.h>
#include <linux/node.h>
#include <linux/gfp.h>
#include <linux/cpufreq.h>
#include <linux/performance.h>
#include <linux/pm_qos.h>
#include <mach/dev.h>

#include "base.h"

static struct exynos_power_info *cur_power_mode;

struct exynos_power_info exynos_power_mode[POWER_MODE_END] = {
/* power_mode, cpu_lock, index */
	{ "low", 800000, 2,},
	{ "high", 1600000, 0,},
};

static BLOCKING_NOTIFIER_HEAD(pfm_notifier_list);

int register_pfm_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&pfm_notifier_list, nb);
}
EXPORT_SYMBOL(register_pfm_notifier);

int unregister_pfm_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&pfm_notifier_list, nb);
}
EXPORT_SYMBOL(unregister_pfm_notifier);

static ssize_t get_cpufreq_pfm_level(struct sysdev_class *class,
				struct sysdev_class_attribute *attr, char *buf)
{
	struct pfm_info *info = list_entry(class, struct pfm_info, pfm_class);
	return sprintf(buf,"%d\n", info->pfm_id);
}

static ssize_t set_cpufreq_pfm_level(struct sysdev_class *class,
				 struct sysdev_class_attribute *attr,
				 const char *buf,
				 size_t count)
{
	struct pfm_info *info = list_entry(class, struct pfm_info, pfm_class);
	unsigned int pfm_level = 0;
	int err = -EINVAL;

	sscanf(buf, "%u", &pfm_level);

	if (pfm_level > MAX_PFM_ID) {
		pr_err("pfm_level should be less than 2\n");
		goto out;
	}

	if (info->pfm_id != pfm_level) {
		err = blocking_notifier_call_chain(&pfm_notifier_list, pfm_level, info);
		if (NOTIFY_BAD == err) {
			pr_err("set_cpufreq_profile fail!\n");
			goto out;
		}
		info->pfm_id = pfm_level;
#ifdef CONFIG_SCHED_MC
#endif
	}

out:
	return count;
}

static SYSDEV_CLASS_ATTR(cpufreq_pfm_level, 0644, get_cpufreq_pfm_level,
			set_cpufreq_pfm_level);

static struct sysdev_class_attribute *pfm_sysdev_class_attrs[] = {
	&attr_cpufreq_pfm_level,
	NULL
};

static ssize_t show_power_mode(struct kobject *kobj, struct attribute *attr, char *buf)
{
	int ret;

	ret = snprintf(buf, POWER_MODE_LEN, "%s\n", cur_power_mode?cur_power_mode->mode_name:"unkown");
	return ret;
}

static unsigned int exynos_get_power_mode(char *target_mode, unsigned int *cpu_lock,
				unsigned int *id)
{
	unsigned int i;

	for (i = 0; i < POWER_MODE_END; i++) {
		if (!strnicmp(exynos_power_mode[i].mode_name, target_mode, POWER_MODE_LEN)) {
			goto set_power_mode_info;
		}
	}

	return -EINVAL;

set_power_mode_info:
	*cpu_lock = exynos_power_mode[i].cpu_freq_lock;
	*id = exynos_power_mode[i].index;
	cur_power_mode = &exynos_power_mode[i];

	return 0;
}

static ssize_t __ref store_power_mode(struct kobject *kobj, struct attribute *attr,
			      const char *buf, size_t count)
{
	char str_power_mode[POWER_MODE_LEN];
	int ret;
	unsigned int cpu_lock, number;

	ret = sscanf(buf, "%15s", str_power_mode);
	if (ret != 1)
		return -EINVAL;

	if (exynos_get_power_mode(str_power_mode, &cpu_lock, &number)) {
		pr_err("%s Can not get power mode\n", __func__);
		return -EINVAL;
	}

	pr_info("store_power_mode: %s\t%d\t%d\n", str_power_mode, cpu_lock, number);
	/* notify the client */
	ret = blocking_notifier_call_chain(&pfm_notifier_list, number, str_power_mode);
	if (NOTIFY_BAD == ret) {
		pr_err("set_cpufreq_profile fail!\n");
		return -EINVAL;
	}

	return count;
}

define_one_global_rw(power_mode);

static int __init performance_dev_init(void)
{
	int err;
	struct pfm_info *info;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (IS_ERR_OR_NULL(info))
		return PTR_ERR(info);

	info->pfm_id = -1;
#ifdef CONFIG_BUSFREQ_OPP
	info->bus_dev = dev_get("exynos-busfreq");

	device_initialize(&info->dev);
	dev_set_name(&info->dev, "performance");
	err = device_add(&info->dev);
	if (err < 0) {
		printk(KERN_ERR "can't %s %s, status %d\n",
				"add", dev_name(&info->dev), err);
		return err;
	}
#endif
	info->pfm_class.name = "performance";
	info->pfm_class.attrs = pfm_sysdev_class_attrs;
	err = sysdev_class_register(&info->pfm_class);
	if (err) {
#ifdef CONFIG_BUSFREQ_OPP
		device_del(&info->dev);
#endif
		pr_err("%s: register sysdev performance erro!\n", __func__);
	}

	err = sysfs_create_file(power_kobj, &power_mode.attr);
	if (err) {
		pr_err("%s: failed to create /sys/power/power_mode sysfs interface\n", __func__);
	}

	return err;
}
late_initcall(performance_dev_init);

MODULE_LICENSE("GPLV2");
MODULE_AUTHOR("litao <litao@meizu.com>");
MODULE_DESCRIPTION("System performance sysfs interface");
