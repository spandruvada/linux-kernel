/*
 * Scheduler code and data structures related to cpufreq.
 *
 * Copyright (C) 2016, Intel Corporation
 * Author: Rafael J. Wysocki <rafael.j.wysocki@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "sched.h"

DEFINE_PER_CPU(struct update_util_data *, cpufreq_update_util_data);

/**
 * cpufreq_add_update_util_hook - Populate the CPU's update_util_data pointer.
 * @cpu: The CPU to set the pointer for.
 * @data: New pointer value.
 * @func: Callback function to set for the CPU.
 *
 * Set and publish the update_util_data pointer for the given CPU.
 *
 * The update_util_data pointer of @cpu is set to @data and the callback
 * function pointer in the target struct update_util_data is set to @func.
 * That function will be called by cpufreq_update_util() from RCU-sched
 * read-side critical sections, so it must not sleep.  @data will always be
 * passed to it as the first argument which allows the function to get to the
 * target update_util_data structure and its container.
 *
 * The update_util_data pointer of @cpu must be NULL when this function is
 * called or it will WARN() and return with no effect.
 */
void cpufreq_add_update_util_hook(int cpu, struct update_util_data *data,
			void (*func)(struct update_util_data *data, u64 time,
				     unsigned int flags))
{
	if (WARN_ON(!data || !func))
		return;

	if (WARN_ON(per_cpu(cpufreq_update_util_data, cpu)))
		return;

	data->func = func;
	rcu_assign_pointer(per_cpu(cpufreq_update_util_data, cpu), data);
}
EXPORT_SYMBOL_GPL(cpufreq_add_update_util_hook);

/**
 * cpufreq_remove_update_util_hook - Clear the CPU's update_util_data pointer.
 * @cpu: The CPU to clear the pointer for.
 *
 * Clear the update_util_data pointer for the given CPU.
 *
 * Callers must use RCU-sched callbacks to free any memory that might be
 * accessed via the old update_util_data pointer or invoke synchronize_sched()
 * right after this function to avoid use-after-free.
 */
void cpufreq_remove_update_util_hook(int cpu)
{
	rcu_assign_pointer(per_cpu(cpufreq_update_util_data, cpu), NULL);
}
EXPORT_SYMBOL_GPL(cpufreq_remove_update_util_hook);


/**
 * cpufreq_get_sched_util - Get utilization values.
 * @cpu: The targetted CPU.
 *
 * Get the CFS, DL and max utilization.
 * This function allows cpufreq driver outside the kernel/sched to access
 * utilization value for a CPUs run queue.
 */
void cpufreq_get_sched_util(int cpu, unsigned long *util_cfs,
			    unsigned long *util_dl, unsigned long *max)
{
	struct rq *rq = cpu_rq(cpu);

	*max = arch_scale_cpu_capacity(NULL, cpu);
	*util_cfs = cpu_util_cfs(rq);
	*util_dl  = cpu_util_dl(rq);
}
EXPORT_SYMBOL_GPL(cpufreq_get_sched_util);
