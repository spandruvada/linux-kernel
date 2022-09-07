// SPDX-License-Identifier: GPL-2.0-only
/*
 * intel_powerclamp.c - package c-state idle injection
 *
 * Copyright (c) 2022, Intel Corporation.
 *
 * Authors:
 *     Arjan van de Ven <arjan@linux.intel.com>
 *     Jacob Pan <jacob.jun.pan@linux.intel.com>
 *
 *	TODO:
 *           1. better handle wakeup from external interrupts, currently a fixed
 *              compensation is added to clamping duration when excessive amount
 *              of wakeups are observed during idle time. the reason is that in
 *              case of external interrupts without need for ack, clamping down
 *              cpu in non-irq context does not reduce irq. for majority of the
 *              cases, clamping down cpu does help reduce irq as well, we should
 *              be able to differentiate the two cases and give a quantitative
 *              solution for the irqs that we can control. perhaps based on
 *              get_cpu_iowait_time_us()
 *
 *	     2. synchronization with other hw blocks
 */

#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/cpu.h>
#include <linux/thermal.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/idle_inject.h>

#include <asm/msr.h>
#include <asm/mwait.h>
#include <asm/cpu_device_id.h>

#define MAX_TARGET_RATIO (50U)
/* For each undisturbed clamping period (no extra wake ups during idle time),
 * we increment the confidence counter for the given target ratio.
 * CONFIDENCE_OK defines the level where runtime calibration results are
 * valid.
 */
#define CONFIDENCE_OK (3)
/* Default idle injection duration, driver adjust sleep time to meet target
 * idle ratio. Similar to frequency modulation.
 */
#define DEFAULT_DURATION_JIFFIES (6)

static unsigned int target_mwait;
static struct dentry *debug_dir;

/* user selected target */
static unsigned int set_target_ratio;
static unsigned int current_ratio;
static bool should_skip;

static unsigned int control_cpu; /* The cpu assigned to collect stat and update
				  * control parameters. default to BSP but BSP
				  * can be offlined.
				  */
struct powerclamp_data {
	unsigned int cpu;
	unsigned int count;
	unsigned int guard;
	unsigned int window_size_now;
	unsigned int target_ratio;
	bool clamping;
};

static struct powerclamp_data powerclamp_data;

static struct thermal_cooling_device *cooling_dev;

static DEFINE_MUTEX(powerclamp_lock);

static unsigned int def_idle_duration;
static unsigned int pkg_cstate_ratio_cur;
static unsigned int window_size;

static int duration_set(const char *arg, const struct kernel_param *kp)
{
	int ret = 0;
	unsigned long new_duration;

	ret = kstrtoul(arg, 10, &new_duration);
	if (ret)
		goto exit;
	if (new_duration > 25 || new_duration < 6) {
		pr_err("Out of recommended range %lu, between 6-25ms\n",
			new_duration);
		ret = -EINVAL;
	}

	def_idle_duration = clamp(new_duration, 6ul, 25ul);
	smp_mb();

exit:

	return ret;
}

static const struct kernel_param_ops duration_ops = {
	.set = duration_set,
	.get = param_get_int,
};


module_param_cb(duration, &duration_ops, &def_idle_duration, 0644);
MODULE_PARM_DESC(duration, "forced idle time for each attempt in msec.");

struct powerclamp_calibration_data {
	unsigned long confidence;  /* used for calibration, basically a counter
				    * gets incremented each time a clamping
				    * period is completed without extra wakeups
				    * once that counter is reached given level,
				    * compensation is deemed usable.
				    */
	unsigned long steady_comp; /* steady state compensation used when
				    * no extra wakeups occurred.
				    */
	unsigned long dynamic_comp; /* compensate excessive wakeup from idle
				     * mostly from external interrupts.
				     */
};

static struct powerclamp_calibration_data cal_data[MAX_TARGET_RATIO];

static int window_size_set(const char *arg, const struct kernel_param *kp)
{
	int ret = 0;
	unsigned long new_window_size;

	ret = kstrtoul(arg, 10, &new_window_size);
	if (ret)
		goto exit_win;
	if (new_window_size > 10 || new_window_size < 2) {
		pr_err("Out of recommended window size %lu, between 2-10\n",
			new_window_size);
		ret = -EINVAL;
	}

	window_size = clamp(new_window_size, 2ul, 10ul);
	smp_mb();

exit_win:

	return ret;
}

static const struct kernel_param_ops window_size_ops = {
	.set = window_size_set,
	.get = param_get_int,
};

module_param_cb(window_size, &window_size_ops, &window_size, 0644);
MODULE_PARM_DESC(window_size, "sliding window in number of clamping cycles\n"
	"\tpowerclamp controls idle ratio within this window. larger\n"
	"\twindow size results in slower response time but more smooth\n"
	"\tclamping results. default to 2.");

static void find_target_mwait(void)
{
	unsigned int eax, ebx, ecx, edx;
	unsigned int highest_cstate = 0;
	unsigned int highest_subcstate = 0;
	int i;

	if (boot_cpu_data.cpuid_level < CPUID_MWAIT_LEAF)
		return;

	cpuid(CPUID_MWAIT_LEAF, &eax, &ebx, &ecx, &edx);

	if (!(ecx & CPUID5_ECX_EXTENSIONS_SUPPORTED) ||
	    !(ecx & CPUID5_ECX_INTERRUPT_BREAK))
		return;

	edx >>= MWAIT_SUBSTATE_SIZE;
	for (i = 0; i < 7 && edx; i++, edx >>= MWAIT_SUBSTATE_SIZE) {
		if (edx & MWAIT_SUBSTATE_MASK) {
			highest_cstate = i;
			highest_subcstate = edx & MWAIT_SUBSTATE_MASK;
		}
	}
	target_mwait = (highest_cstate << MWAIT_SUBSTATE_SIZE) |
		(highest_subcstate - 1);

}

struct pkg_cstate_info {
	bool skip;
	int msr_index;
	int cstate_id;
};

#define PKG_CSTATE_INIT(id) {				\
		.msr_index = MSR_PKG_C##id##_RESIDENCY, \
		.cstate_id = id				\
			}

static struct pkg_cstate_info pkg_cstates[] = {
	PKG_CSTATE_INIT(2),
	PKG_CSTATE_INIT(3),
	PKG_CSTATE_INIT(6),
	PKG_CSTATE_INIT(7),
	PKG_CSTATE_INIT(8),
	PKG_CSTATE_INIT(9),
	PKG_CSTATE_INIT(10),
	{NULL},
};

static bool has_pkg_state_counter(void)
{
	u64 val;
	struct pkg_cstate_info *info = pkg_cstates;

	/* check if any one of the counter msrs exists */
	while (info->msr_index) {
		if (!rdmsrl_safe(info->msr_index, &val))
			return true;
		info++;
	}

	return false;
}

static u64 pkg_state_counter(void)
{
	u64 val;
	u64 count = 0;
	struct pkg_cstate_info *info = pkg_cstates;

	while (info->msr_index) {
		if (!info->skip) {
			if (!rdmsrl_safe(info->msr_index, &val))
				count += val;
			else
				info->skip = true;
		}
		info++;
	}

	return count;
}

static unsigned int get_compensation(int ratio)
{
	unsigned int comp = 0;

	/* we only use compensation if all adjacent ones are good */
	if (ratio == 1 &&
		cal_data[ratio].confidence >= CONFIDENCE_OK &&
		cal_data[ratio + 1].confidence >= CONFIDENCE_OK &&
		cal_data[ratio + 2].confidence >= CONFIDENCE_OK) {
		comp = (cal_data[ratio].steady_comp +
			cal_data[ratio + 1].steady_comp +
			cal_data[ratio + 2].steady_comp) / 3;
	} else if (ratio == MAX_TARGET_RATIO - 1 &&
		cal_data[ratio].confidence >= CONFIDENCE_OK &&
		cal_data[ratio - 1].confidence >= CONFIDENCE_OK &&
		cal_data[ratio - 2].confidence >= CONFIDENCE_OK) {
		comp = (cal_data[ratio].steady_comp +
			cal_data[ratio - 1].steady_comp +
			cal_data[ratio - 2].steady_comp) / 3;
	} else if (cal_data[ratio].confidence >= CONFIDENCE_OK &&
		cal_data[ratio - 1].confidence >= CONFIDENCE_OK &&
		cal_data[ratio + 1].confidence >= CONFIDENCE_OK) {
		comp = (cal_data[ratio].steady_comp +
			cal_data[ratio - 1].steady_comp +
			cal_data[ratio + 1].steady_comp) / 3;
	}

	/* do not exceed limit */
	if (comp + ratio >= MAX_TARGET_RATIO)
		comp = MAX_TARGET_RATIO - ratio - 1;

	return comp;
}

static void adjust_compensation(int target_ratio, unsigned int win)
{
	int delta;
	struct powerclamp_calibration_data *d = &cal_data[target_ratio];

	/*
	 * adjust compensations if confidence level has not been reached or
	 * there are too many wakeups during the last idle injection period, we
	 * cannot trust the data for compensation.
	 */
	if (d->confidence >= CONFIDENCE_OK)
		return;

	delta = READ_ONCE(set_target_ratio) - current_ratio;

	/* filter out bad data */
	if (delta >= 0 && delta <= (1 + target_ratio / 10)) {
		if (d->steady_comp)
			d->steady_comp =
				roundup(delta+d->steady_comp, 2) / 2;
		else
			d->steady_comp = delta;
		d->confidence++;
	}
}


static bool powerclamp_adjust_controls(unsigned int target_ratio,
				       unsigned int guard, unsigned int win)
{
	static u64 msr_last, tsc_last;
	u64 msr_now, tsc_now;
	u64 val64;

	/* check result for the last window */
	msr_now = pkg_state_counter();
	tsc_now = rdtsc();

	/* calculate pkg cstate vs tsc ratio */
	if (!msr_last || !tsc_last)
		current_ratio = 1;
	else if (tsc_now-tsc_last) {
		val64 = 100*(msr_now-msr_last);
		do_div(val64, (tsc_now-tsc_last));
		current_ratio = val64;
	}

	/* update record */
	msr_last = msr_now;
	tsc_last = tsc_now;

	adjust_compensation(target_ratio, win);
	/* if we are above target+guard, skip */
	return READ_ONCE(set_target_ratio) + guard <= current_ratio;
}

static unsigned int get_run_time(void)
{
	unsigned int compensated_ratio;
	unsigned int runtime;

	/*
	 * make sure user selected ratio does not take effect until
	 * the next round. adjust target_ratio if user has changed
	 * target such that we can converge quickly.
	 */
	powerclamp_data.target_ratio = READ_ONCE(set_target_ratio);
	powerclamp_data.guard = 1 + powerclamp_data.target_ratio / 20;
	powerclamp_data.window_size_now = window_size;
	powerclamp_data.count++;

	/*
	 * systems may have different ability to enter package level
	 * c-states, thus we need to compensate the injected idle ratio
	 * to achieve the actual target reported by the HW.
	 */
	compensated_ratio = powerclamp_data.target_ratio +
		get_compensation(powerclamp_data.target_ratio);
	if (compensated_ratio <= 0)
		compensated_ratio = 1;

	runtime = def_idle_duration * 100 / compensated_ratio - def_idle_duration;

	return runtime;
}

/*
 * 1 HZ polling while clamping is active, useful for userspace
 * to monitor actual idle ratio.
 */
static void poll_pkg_cstate(struct work_struct *dummy);
static DECLARE_DELAYED_WORK(poll_pkg_cstate_work, poll_pkg_cstate);
static void poll_pkg_cstate(struct work_struct *dummy)
{
	static u64 msr_last;
	static u64 tsc_last;

	u64 msr_now;
	u64 tsc_now;
	u64 val64;

	msr_now = pkg_state_counter();
	tsc_now = rdtsc();

	/* calculate pkg cstate vs tsc ratio */
	if (!msr_last || !tsc_last)
		pkg_cstate_ratio_cur = 1;
	else {
		if (tsc_now - tsc_last) {
			val64 = 100 * (msr_now - msr_last);
			do_div(val64, (tsc_now - tsc_last));
			pkg_cstate_ratio_cur = val64;
		}
	}

	/* update record */
	msr_last = msr_now;
	tsc_last = tsc_now;

	if (powerclamp_data.clamping)
		schedule_delayed_work(&poll_pkg_cstate_work, HZ);
}

static struct idle_inject_device *ii_dev;

static int idle_inject_begin(unsigned int cpu, unsigned int idle_duration, unsigned int run_duration)
{
	/*
	 * only elected controlling cpu can collect stats and update
	 * control parameters.
	 */
	if (cpu == control_cpu &&
	    !(powerclamp_data.count % powerclamp_data.window_size_now)) {
		bool skip = powerclamp_adjust_controls(powerclamp_data.target_ratio,
						       powerclamp_data.guard,
						       powerclamp_data.window_size_now);
		WRITE_ONCE(should_skip, skip);

	}

	if (READ_ONCE(should_skip))
		return -EAGAIN;

	idle_inject_set_duration(ii_dev, run_duration, idle_duration);

	return 0;
}

void idle_inject_end(unsigned int cpu, unsigned int idle_duration, unsigned int run_duration)
{
	unsigned int runtime;

	runtime = get_run_time();
	idle_inject_set_duration(ii_dev, runtime, idle_duration);
}

static void trigger_idle_injection(void)
{
	unsigned int runtime = get_run_time();

	idle_inject_set_duration(ii_dev, runtime, def_idle_duration);
	idle_inject_start(ii_dev);
	powerclamp_data.clamping = true;
}

static int powerclamp_idle_injection_register(void)
{
	static cpumask_t idle_injection_cpu_mask;
	unsigned long cpu;

	/*
	 * The idle inject core will only inject for online CPUs,
	 * So we can register for all present CPUs. In this way
	 * if some CPU goes online/offline while idle inject
	 * is registered, nothing additional calls are required.
	 * The same runtime and idle time is applicable for
	 * newly onlined CPUs if any.
	 */
	for_each_present_cpu(cpu) {
		cpumask_set_cpu(cpu, &idle_injection_cpu_mask);
	}

	ii_dev = idle_inject_register(&idle_injection_cpu_mask,
				      idle_inject_begin,
				      idle_inject_end);
	if (!ii_dev) {
		pr_err("powerclamp: idle_inject_register failed \n");
		return -EAGAIN;
	}

	idle_inject_set_duration(ii_dev, TICK_USEC, def_idle_duration);
	idle_inject_set_latency(ii_dev, UINT_MAX);

	return 0;
}

static void remove_idle_injection(void)
{
	if (!powerclamp_data.clamping)
		return;

	powerclamp_data.clamping = false;
	idle_inject_stop(ii_dev);
}

static int start_power_clamp(void)
{
	int ret;

	/* prevent cpu hotplug */
	cpus_read_lock();

	/* prefer BSP */
	control_cpu = 0;
	if (!cpu_online(control_cpu)) {
		control_cpu = get_cpu();
		put_cpu();
	}

	cpus_read_unlock();

	ret = powerclamp_idle_injection_register();
	if (!ret) {
		trigger_idle_injection();
		schedule_delayed_work(&poll_pkg_cstate_work, 0);
	}

	return ret;
}

static void end_power_clamp(void)
{
	if (powerclamp_data.clamping) {
		remove_idle_injection();
		idle_inject_unregister(ii_dev);
	}
}

static int powerclamp_cpu_online(unsigned int cpu)
{
	if (!powerclamp_data.clamping)
		return 0;

	/* prefer BSP as controlling CPU */
	if (cpu == 0) {
		control_cpu = 0;
		smp_mb();
	}

	return 0;
}

static int powerclamp_cpu_predown(unsigned int cpu)
{
	if (cpu != control_cpu)
		return 0;

	control_cpu = cpumask_first(cpu_online_mask);
	if (control_cpu == cpu)
		control_cpu = cpumask_next(cpu, cpu_online_mask);
	smp_mb();

	return 0;
}

static int powerclamp_get_max_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	*state = MAX_TARGET_RATIO;

	return 0;
}

static int powerclamp_get_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	if (powerclamp_data.clamping)
		*state = pkg_cstate_ratio_cur;
	else
		/* to save power, do not poll idle ratio while not clamping */
		*state = -1; /* indicates invalid state */

	return 0;
}

static int powerclamp_set_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long new_target_ratio)
{
	int ret = 0;

	mutex_lock(&powerclamp_lock);

	new_target_ratio = clamp(new_target_ratio, 0UL,
				(unsigned long) (MAX_TARGET_RATIO -1 ));
	if (READ_ONCE(set_target_ratio) == 0 && new_target_ratio > 0) {
		pr_info("Start idle injection to reduce power\n");
		WRITE_ONCE(set_target_ratio, new_target_ratio);
		ret = start_power_clamp();
		goto exit_set;
	} else	if (READ_ONCE(set_target_ratio) > 0 && new_target_ratio == 0) {
		pr_info("Stop forced idle injection\n");
		end_power_clamp();
		WRITE_ONCE(set_target_ratio, 0);
	} else	/* adjust currently running */ {
		WRITE_ONCE(set_target_ratio, new_target_ratio);
	}

exit_set:
	mutex_unlock(&powerclamp_lock);

	return ret;
}

/* bind to generic thermal layer as cooling device*/
static const struct thermal_cooling_device_ops powerclamp_cooling_ops = {
	.get_max_state = powerclamp_get_max_state,
	.get_cur_state = powerclamp_get_cur_state,
	.set_cur_state = powerclamp_set_cur_state,
};

static const struct x86_cpu_id __initconst intel_powerclamp_ids[] = {
	X86_MATCH_VENDOR_FEATURE(INTEL, X86_FEATURE_MWAIT, NULL),
	{}
};
MODULE_DEVICE_TABLE(x86cpu, intel_powerclamp_ids);

static int __init powerclamp_probe(void)
{

	if (!x86_match_cpu(intel_powerclamp_ids)) {
		pr_err("CPU does not support MWAIT\n");
		return -ENODEV;
	}

	/* The goal for idle time alignment is to achieve package cstate. */
	if (!has_pkg_state_counter()) {
		pr_info("No package C-state available\n");
		return -ENODEV;
	}

	/* find the deepest mwait value */
	find_target_mwait();

	return 0;
}

static int powerclamp_debug_show(struct seq_file *m, void *unused)
{
	int i = 0;

	seq_printf(m, "controlling cpu: %d\n", control_cpu);
	seq_printf(m, "pct confidence steady dynamic (compensation)\n");
	for (i = 0; i < MAX_TARGET_RATIO; i++) {
		seq_printf(m, "%d\t%lu\t%lu\t%lu\n",
			i,
			cal_data[i].confidence,
			cal_data[i].steady_comp,
			cal_data[i].dynamic_comp);
	}

	return 0;
}

DEFINE_SHOW_ATTRIBUTE(powerclamp_debug);

static inline void powerclamp_create_debug_files(void)
{
	debug_dir = debugfs_create_dir("intel_powerclamp", NULL);

	debugfs_create_file("powerclamp_calib", S_IRUGO, debug_dir, cal_data,
			    &powerclamp_debug_fops);
}

static enum cpuhp_state hp_state;

static int __init powerclamp_init(void)
{
	int retval;

	/* probe cpu features and ids here */
	retval = powerclamp_probe();
	if (retval)
		return retval;

	/* set default limit, maybe adjusted during runtime based on feedback */
	window_size = 2;
	retval = cpuhp_setup_state_nocalls(CPUHP_AP_ONLINE_DYN,
					   "thermal/intel_powerclamp:online",
					   powerclamp_cpu_online,
					   powerclamp_cpu_predown);
	if (retval < 0)
		return retval;

	hp_state = retval;

	cooling_dev = thermal_cooling_device_register("intel_powerclamp", NULL,
						      &powerclamp_cooling_ops);
	if (IS_ERR(cooling_dev)) {
		retval = -ENODEV;
		goto exit_unregister;
	}

	if (!def_idle_duration)
		def_idle_duration = jiffies_to_usecs(DEFAULT_DURATION_JIFFIES);

	powerclamp_create_debug_files();

	return 0;

exit_unregister:
	cpuhp_remove_state_nocalls(hp_state);
	return retval;
}
module_init(powerclamp_init);

static void __exit powerclamp_exit(void)
{
	mutex_lock(&powerclamp_lock);
	end_power_clamp();
	mutex_unlock(&powerclamp_lock);
	cpuhp_remove_state_nocalls(hp_state);
	thermal_cooling_device_unregister(cooling_dev);

	cancel_delayed_work_sync(&poll_pkg_cstate_work);
	debugfs_remove_recursive(debug_dir);
}
module_exit(powerclamp_exit);

MODULE_IMPORT_NS(IDLE_INJECT);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Arjan van de Ven <arjan@linux.intel.com>");
MODULE_AUTHOR("Jacob Pan <jacob.jun.pan@linux.intel.com>");
MODULE_DESCRIPTION("Package Level C-state Idle Injection for Intel CPUs");
