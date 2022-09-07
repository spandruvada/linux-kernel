// SPDX-License-Identifier: GPL-2.0
/*
 * Per CPU Idle injection cooling device implementation
 *
 * Copyright (c) 2022, Intel Corporation.
 * All rights reserved.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cpufeature.h>
#include <linux/cpuhotplug.h>
#include <linux/idle_inject.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <linux/topology.h>

#include <asm/cpu_device_id.h>

/* Duration match with intel_powerclamp driver */
#define IDLE_DURATION		24000
#define IDLE_LATENCY		UINT_MAX

#define IDLE_ERROR_HYSTERISIS	5

static int idle_duration_us = IDLE_DURATION;
static int idle_latency_us = IDLE_LATENCY;

module_param(idle_duration_us, int, 0644);
MODULE_PARM_DESC(idle_duration_us,
		 "Idle duration in us.");

module_param(idle_latency_us, int, 0644);
MODULE_PARM_DESC(idle_latency_us,
		 "Idle latency in us.");

/**
 * struct cpuidle_cooling - Per instance data for cooling device
 * @cpu: CPU number for this cooling device
 * @ii_dev: Idle inject core instance pointer
 * @cdev: Thermal core cooling device instance
 * @state:  Current cooling device state
 * @run_duration: Current calcuated run duration based on state
 * @last_mperf: last MPERF counter
 * @last_tsc: Last TSC counter
 * @comp_active: compensation is active
 *
 * Stores per instance cooling device state.
 */
struct cpuidle_cooling {
	int cpu;
	struct idle_inject_device *ii_dev;
	struct thermal_cooling_device *cdev;
	unsigned long state;
	unsigned int run_duration;
	u64 last_mperf;
	u64 last_tsc;
	bool comp_active;
};

static DEFINE_PER_CPU(struct cpuidle_cooling, cooling_devs);
static cpumask_t cpuidle_cpu_mask;

/* Used for module unload protection with idle injection operations */
static DEFINE_MUTEX(idle_cooling_lock);

static unsigned int cpuidle_cooling_runtime(unsigned int idle_duration_us,
					    unsigned long state)
{
	if (!state)
		return 0;

	return ((idle_duration_us * 100) / state) - idle_duration_us;
}

static void idle_inject_end(unsigned int cpu, unsigned int idle_duration,
			    unsigned int run_duration)
{
	struct cpuidle_cooling *cooling_dev = &per_cpu(cooling_devs, cpu);
	unsigned long current_state = READ_ONCE(cooling_dev->state);
	int busy, eff_state;
	u64 tsc, mperf;

	tsc= rdtsc();
	rdmsrl(MSR_IA32_MPERF, mperf);

	if (!cooling_dev->last_mperf)
		goto ignore_sample;

	busy = (mperf - cooling_dev->last_mperf) * 100 / (tsc - cooling_dev->last_tsc);
	eff_state = 100 - busy;

        if ((current_state - IDLE_ERROR_HYSTERISIS) > eff_state) {
		run_duration = cpuidle_cooling_runtime(idle_duration, eff_state);
		idle_inject_set_duration(cooling_dev->ii_dev, run_duration, idle_duration);
		cooling_dev->comp_active = true;
        } else if (cooling_dev->comp_active) {
		idle_inject_set_duration(cooling_dev->ii_dev,
					 READ_ONCE(cooling_dev->run_duration),
					 idle_duration);
		cooling_dev->comp_active = false;
	}

ignore_sample:
        cooling_dev->last_mperf = mperf;
        cooling_dev->last_tsc = tsc;
}

static int cpuidle_idle_injection_register(struct cpuidle_cooling *cooling_dev)
{
	struct idle_inject_device *ii_dev;

	ii_dev = idle_inject_register((struct cpumask *)cpumask_of(cooling_dev->cpu),
				      NULL, idle_inject_end);
	if (!ii_dev) {
		/*
		 * It is busy as some other device claimed idle injection for this CPU
		 * Also it is possible that memory allocation failure.
		 */
		pr_err("idle_inject_register failed for cpu:%d\n", cooling_dev->cpu);
		return -EAGAIN;
	}

	idle_inject_set_duration(ii_dev, TICK_USEC, idle_duration_us);
	idle_inject_set_latency(ii_dev, idle_latency_us);

	cooling_dev->ii_dev = ii_dev;

	return 0;
}

static void cpuidle_idle_injection_unregister(struct cpuidle_cooling *cooling_dev)
{
	idle_inject_unregister(cooling_dev->ii_dev);
}

static int cpuidle_cooling_get_max_state(struct thermal_cooling_device *cdev,
					 unsigned long *state)
{
	*state = 100;

	return 0;
}

static int cpuidle_cooling_get_cur_state(struct thermal_cooling_device *cdev,
					 unsigned long *state)
{
	struct cpuidle_cooling *cooling_dev = cdev->devdata;

	*state = READ_ONCE(cooling_dev->state);

	return 0;
}

static int cpuidle_cooling_set_cur_state(struct thermal_cooling_device *cdev,
					 unsigned long state)
{
	struct cpuidle_cooling *cooling_dev = cdev->devdata;
	unsigned int _runtime_us;
	unsigned long curr_state;
	int ret = 0;

	mutex_lock(&idle_cooling_lock);

	curr_state = READ_ONCE(cooling_dev->state);

	if (!curr_state && state > 0) {
		/*
		 * This is the first time to start cooling, so register with
		 * idle injection framework.
		 */
		if (!cooling_dev->ii_dev) {
			ret = cpuidle_idle_injection_register(cooling_dev);
			if (ret)
				goto unlock_set_state;

			cooling_dev->last_mperf = 0;
			cooling_dev->last_tsc = 0;
		}

		_runtime_us = cpuidle_cooling_runtime(idle_duration_us, state);

		WRITE_ONCE(cooling_dev->run_duration, _runtime_us);

		idle_inject_set_duration(cooling_dev->ii_dev, _runtime_us, idle_duration_us);
		idle_inject_start(cooling_dev->ii_dev);
	} else if (curr_state > 0 && state) {
		/* Simply update runtime */
		_runtime_us = cpuidle_cooling_runtime(idle_duration_us, state);
		cooling_dev->run_duration = _runtime_us;
		idle_inject_set_duration(cooling_dev->ii_dev, _runtime_us, idle_duration_us);
	} else if (curr_state > 0 && !state) {
		idle_inject_stop(cooling_dev->ii_dev);
		cpuidle_idle_injection_unregister(cooling_dev);
		cooling_dev->ii_dev = NULL;
	}

	WRITE_ONCE(cooling_dev->state, state);

unlock_set_state:
	mutex_unlock(&idle_cooling_lock);

	return ret;
}

/**
 * cpuidle_cooling_ops - thermal cooling device ops
 */
static struct thermal_cooling_device_ops cpuidle_cooling_ops = {
	.get_max_state = cpuidle_cooling_get_max_state,
	.get_cur_state = cpuidle_cooling_get_cur_state,
	.set_cur_state = cpuidle_cooling_set_cur_state,
};

static int cpuidle_cooling_register(int cpu)
{
	struct cpuidle_cooling *cooling_dev = &per_cpu(cooling_devs, cpu);
	struct thermal_cooling_device *cdev;
	char name[10]; /* storage for idle-XXXX */
	int ret = 0;

	mutex_lock(&idle_cooling_lock);

	snprintf(name, sizeof(name), "idle-%d", cpu);
	cdev = thermal_cooling_device_register(name, cooling_dev, &cpuidle_cooling_ops);
	if (IS_ERR(cdev)) {
		ret = PTR_ERR(cdev);
		goto unlock_register;
	}

	cooling_dev->cdev = cdev;
	cpumask_set_cpu(cpu, &cpuidle_cpu_mask);
	cooling_dev->cpu = cpu;

unlock_register:
	mutex_unlock(&idle_cooling_lock);

	return ret;
}

static void cpuidle_cooling_unregister(int cpu)
{
	struct cpuidle_cooling *cooling_dev = &per_cpu(cooling_devs, cpu);

	mutex_lock(&idle_cooling_lock);

	if (cooling_dev->state) {
		idle_inject_stop(cooling_dev->ii_dev);
		cpuidle_idle_injection_unregister(cooling_dev);
	}

	thermal_cooling_device_unregister(cooling_dev->cdev);
	cooling_dev->state = 0;

	mutex_unlock(&idle_cooling_lock);
}

static int cpuidle_cooling_cpu_online(unsigned int cpu)
{
	cpuidle_cooling_register(cpu);

	return 0;
}

static int cpuidle_cooling_cpu_offline(unsigned int cpu)
{
	cpuidle_cooling_unregister(cpu);

	return 0;
}

static enum cpuhp_state cpuidle_cooling_hp_state __read_mostly;

static const struct x86_cpu_id __initconst intel_cpuidle_cooling_ids[] = {
        X86_MATCH_VENDOR_FEATURE(INTEL, X86_FEATURE_MWAIT, NULL),
        {}
};
MODULE_DEVICE_TABLE(x86cpu, intel_cpuidle_cooling_ids);

static int __init cpuidle_cooling_init(void)
{
	int ret;

       if (!x86_match_cpu(intel_cpuidle_cooling_ids)) {
                pr_err("CPU does not support MWAIT\n");
                return -ENODEV;
        }

	ret = cpuhp_setup_state(CPUHP_AP_ONLINE_DYN,
				"thermal/cpuidle_cooling:online",
				cpuidle_cooling_cpu_online,
				cpuidle_cooling_cpu_offline);
	if (ret < 0)
		return ret;

	cpuidle_cooling_hp_state = ret;

	return 0;
}
module_init(cpuidle_cooling_init)

static void __exit cpuidle_cooling_exit(void)
{
	cpuhp_remove_state(cpuidle_cooling_hp_state);
}
module_exit(cpuidle_cooling_exit)

MODULE_IMPORT_NS(IDLE_INJECT);

MODULE_LICENSE("GPL v2");
