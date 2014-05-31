/*
 *  drivers/cpufreq/cpufreq_eprjdemand.c
 *
 *  Copyright (C)  2012 EternityProject Developers
 *    Angelo G. Del Regno <kholk11@gmail.com>
 *    Joel Low <joel@joelsplace.sg>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/ktime.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/reboot.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <mach/eternityproject.h>

#if !defined(CONFIG_EPRJ_SYSFS_TOOLS) && defined(USE_OWN_HOTPLUGGING)

/* Runqueue average */
#define RQ_AVG_TIMER_RATE	10

struct runqueue_data {
	unsigned int nr_run_avg;
	unsigned int update_rate;
	int64_t last_time;
	int64_t total_time;
	struct delayed_work work;
	struct workqueue_struct *nr_run_wq;
	spinlock_t lock;
};

static struct runqueue_data *rq_data;
static void rq_work_fn(struct work_struct *work);

static void start_rq_work(void)
{
	rq_data->nr_run_avg = 0;
	rq_data->last_time = 0;
	rq_data->total_time = 0;
	if (rq_data->nr_run_wq == NULL)
		rq_data->nr_run_wq =
			create_singlethread_workqueue("nr_run_avg");

	queue_delayed_work(rq_data->nr_run_wq, &rq_data->work,
			   msecs_to_jiffies(rq_data->update_rate));
	return;
}

static void stop_rq_work(void)
{
	if (rq_data->nr_run_wq)
		cancel_delayed_work(&rq_data->work);
	return;
}

static int __init init_rq_avg(void)
{
	rq_data = kzalloc(sizeof(struct runqueue_data), GFP_KERNEL);
	if (rq_data == NULL) {
		pr_err("%s cannot allocate memory\n", __func__);
		return -ENOMEM;
	}
	spin_lock_init(&rq_data->lock);
	rq_data->update_rate = RQ_AVG_TIMER_RATE;
	INIT_DELAYED_WORK_DEFERRABLE(&rq_data->work, rq_work_fn);

	return 0;
}

static void rq_work_fn(struct work_struct *work)
{
	int64_t time_diff = 0;
	int64_t nr_run = 0;
	unsigned long flags = 0;
	int64_t cur_time = ktime_to_ns(ktime_get());

	spin_lock_irqsave(&rq_data->lock, flags);

	if (rq_data->last_time == 0)
		rq_data->last_time = cur_time;
	if (rq_data->nr_run_avg == 0)
		rq_data->total_time = 0;

	nr_run = nr_running() * 100;
	time_diff = cur_time - rq_data->last_time;
	do_div(time_diff, 1000 * 1000);

	if (time_diff != 0 && rq_data->total_time != 0) {
		nr_run = (nr_run * time_diff) +
			(rq_data->nr_run_avg * rq_data->total_time);
		do_div(nr_run, rq_data->total_time + time_diff);
	}
	rq_data->nr_run_avg = nr_run;
	rq_data->total_time += time_diff;
	rq_data->last_time = cur_time;

	if (rq_data->update_rate != 0)
		queue_delayed_work(rq_data->nr_run_wq, &rq_data->work,
				   msecs_to_jiffies(rq_data->update_rate));

	spin_unlock_irqrestore(&rq_data->lock, flags);
}

static unsigned int get_nr_run_avg(void)
{
	unsigned int nr_run_avg;
	unsigned long flags = 0;

	spin_lock_irqsave(&rq_data->lock, flags);
	nr_run_avg = rq_data->nr_run_avg;
	rq_data->nr_run_avg = 0;
	spin_unlock_irqrestore(&rq_data->lock, flags);

	return nr_run_avg;
}
#endif

/*
 * dbs is used in this file as a shortform for demandbased switching
 * It helps to keep variable names smaller, simpler
 */

#define DEF_SAMPLING_DOWN_FACTOR		(2)
#define MAX_SAMPLING_DOWN_FACTOR		(100000)
#define DEF_FREQUENCY_DOWN_DIFFERENTIAL		(3)
/* The frequency to increase by, when load is above up threshold. This
   is a percentage of the present frequency.

   This constant works differently from the DEF_FREQ_STEP constant.
   This is the maximum percentage of the current frequency we can
   increase by, and this value is used ONLY WHEN the CPU load is at
   100%. At anything less, this will be multiplied by the fraction
   of the present load above the target up threshold.

   Thus: If our target load is 70%, and we are at 85% load, we will
   increment by 50% (since 100% is 30% above 70%, and 85% is 15%
   above 70%) of the up ratio. */
#define DEF_FREQUENCY_UP_RATIO		(30)
#define MIN_FREQUENCY_UP_THRESHOLD             (11)
#define MAX_FREQUENCY_UP_THRESHOLD             (100U)
/* The up thresholds, at the bottom of the range, and at the top of
   the range. This is used only when the device is fully awake.

   We maintain 2 pairs of values: the lower frequency bound and its
   up threshold; as well as the upper frequency bound and its up
   threshold.

   All frequencies below the lower frequency bound will use the lower
   bound threshold and all frequencies above the upper frequency bound
   will use the upper bound threshold. All frequencies in between will
   use a linear extrapolation of the values. */
#define DEF_FREQUENCY_LOWER_BOUND_FREQ		(204000) /* kHz */
#define DEF_FREQUENCY_LOWER_BOUND_THRESHOLD	(50)
#define DEF_FREQUENCY_UPPER_BOUND_FREQ		(1000000) /* kHz */
#define DEF_FREQUENCY_UPPER_BOUND_THRESHOLD	(97)

/* When the device is in early suspend, we will use a constant threshold. */
#define DEF_UP_THRESHOLD_AT_IDLE		(80)
/* The frequency skew changes the behaviour of the governor in SMP
   configurations: whether to favour the minimum required CPU
   frequency among all online CPUs, the maximum, or the average of
   them.

   The default of 50 will cause the governor to use the average load
   among all online CPUs when calculating the frequency to use. Setting
   this to 100 will cause the governor to use the maximum frequency
   required among all CPUs to maintain the given load target. Setting
   this to 0 will cause the governor to use the minimum frequency among
   all CPUs online. Intermediate values divide the average to the maximum
   or minimum, and adds/subtracts from the average, respectively. */
#define DEF_FREQUENCY_SKEW			(60)
#define MAX_FREQUENCY_SKEW			(100U)
#define DEF_SAMPLING_RATE			(48000)
#define MIN_SAMPLING_RATE			(20000U)
#define SLEEP_SAMPLING_RATE_FACTOR		(4)
#if defined(USE_OWN_HOTPLUGGING)
#define MAX_HOTPLUG_RATE			(40u)

#define DEF_MAX_CPU_LOCK			(0)
#define DEF_UP_NR_CPUS				(1)
#define DEF_CPU_UP_RATE				(10)
#define DEF_CPU_DOWN_RATE			(20)
#endif
#define DEF_START_DELAY				(0)

/* Assume the transition latency is <=10mS */
#define MAX_LATENCY				(10 * 1000 * 1000)

#if defined(USE_OWN_HOTPLUGGING)
#define HOTPLUG_DOWN_INDEX			(0)
#define HOTPLUG_UP_INDEX			(1)

/*
 * ADDON_CORES: If your CPU has got more than four cores,
 *              then raise this value.
 */
#define ADDON_CORES				(2)
#define EPRJ_NCORES				(2 + ADDON_CORES)

/*
 * EPRJ_RQ: Number of RQ references to use. Actually, we
 *          are using two references, as DOWNRQ and UPRQ.
 * Note:    Changing this value means you have to change
 *          the code that is using hotplug_rq.
 */
#define EPRJ_RQ					(2)

#if defined(CONFIG_MACH_MIDAS)
static int hotplug_rq[EPRJ_NCORES][EPRJ_RQ] = {
	{0, 200}, {200, 300}, {300, 400}, {400, 0}
};

static int hotplug_freq[EPRJ_NCORES][EPRJ_RQ] = {
	{0, 500000},
	{400000, 500000},
	{400000, 800000},
	{600000, 0}
};
#elif defined(CONFIG_MACH_ENDEAVORU)
static int hotplug_rq[EPRJ_NCORES][EPRJ_RQ] = {
//	{0, 250}, {200, 350}, {340, 410}, {480, 0}
	{0, 215}, {215, 310}, {340, 400}, {450, 0}
};

static int hotplug_freq[EPRJ_NCORES][EPRJ_RQ] = {
	{0, 640000},
	{475000, 640000},
	{475000, 860000},
	{760000, 0}
};
#else
#warning EternityProject: Using a generic hotplug table.
static int hotplug_rq[EPRJ_NCORES][EPRJ_RQ] = {
	{0, 100}, {100, 200}, {200, 300}, {300, 0}
};

static int hotplug_freq[EPRJ_NCORES][EPRJ_RQ] = {
	{0, 500000},
	{200000, 500000},
	{200000, 500000},
	{200000, 0}
};
#endif
#endif

#if defined(USE_OWN_HOTPLUGGING)
static void do_dbs_timer(struct work_struct *work);
#endif
static int cpufreq_governor_dbs(struct cpufreq_policy *policy,
				unsigned int event);

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_EPRJDEMAND
static
#endif
struct cpufreq_governor cpufreq_gov_eprjdemand = {
	.name                   = "eprjdemand",
	.governor               = cpufreq_governor_dbs,
	.max_transition_latency = MAX_LATENCY,
	.owner                  = THIS_MODULE,
};

struct cpu_dbs_info_s {
	cputime64_t prev_cpu_idle;
	cputime64_t prev_cpu_iowait;
	cputime64_t prev_cpu_wall;
	cputime64_t prev_cpu_nice;
	struct cpufreq_policy *cur_policy;
	int cpu;
};
static DEFINE_PER_CPU(struct cpu_dbs_info_s, od_cpu_dbs_info);

/**
 * Stores the state associated with the governor.
 */
static struct dbs_global_info {
	/**
	 * Stores the work object for doing sampling.
	 */
	struct delayed_work work;
#if defined(USE_OWN_HOTPLUGGING)
	struct work_struct up_work;
	struct work_struct down_work;
#endif

#if CONFIG_HAS_EARLYSUSPEND
	/**
	 * Stores if we are in early suspend. This will enable
	 * certain code paths that would increase delay between
	 * samplings, use the sleep thresholds etc.
	 */
	int early_suspend;

#if defined(USE_OWN_HOTPLUGGING)
	int early_suspend_hotplug_lock;
#endif
#endif
	unsigned int rate_mult;

	/**
	 * Mutex that serializes governor limit changes with do_dbs_timer.
	 * We do not want do_dbs_timer to run when user is changing the
	 * governor or limits.
	 */
	struct mutex timer_mutex;

	/**
	 * The workqueue that all our workers will be running on.
	 */
	struct workqueue_struct *workqueue;

	/**
	 * dbs_mutex protects dbs_enable in governor start/stop.
	 */
	struct mutex dbs_mutex;

	/**
	 * number of CPUs using this governor
	 */
	unsigned int dbs_enable;

	/**
	 * The CPU Mask of CPUs using this governor.
	 */
	cpumask_var_t dbs_enable_cpumask;
} dbs_info;

static struct dbs_tuners {
	unsigned int sampling_rate;
	unsigned int lower_bound_freq;
	unsigned int lower_bound_up_threshold;
	unsigned int upper_bound_freq;
	unsigned int upper_bound_up_threshold;
	unsigned int up_threshold_at_idle;
	unsigned int up_ratio;
	unsigned int frequency_skew;
	unsigned int down_differential;
	unsigned int ignore_nice;
	unsigned int sampling_down_factor;
	unsigned int io_is_busy;
	/* eprjdemand tuners */
#if defined(USE_OWN_HOTPLUGGING)
	unsigned int cpu_up_rate;
	unsigned int cpu_down_rate;
	unsigned int up_nr_cpus;
	unsigned int max_cpu_lock;
	atomic_t hotplug_lock;
	unsigned int dvfs_debug;
#endif
} dbs_tuners_ins = {
#if defined(USE_OWN_HOTPLUGGING)
	.cpu_up_rate = DEF_CPU_UP_RATE,
	.cpu_down_rate = DEF_CPU_DOWN_RATE,
	.up_nr_cpus = DEF_UP_NR_CPUS,
	.max_cpu_lock = DEF_MAX_CPU_LOCK,
	.hotplug_lock = ATOMIC_INIT(0),
	.dvfs_debug = 0,
#endif
};


#if defined(USE_OWN_HOTPLUGGING)
/*
 * CPU hotplug lock interface
 */

static atomic_t g_hotplug_count = ATOMIC_INIT(0);
static atomic_t g_hotplug_lock = ATOMIC_INIT(0);

static void apply_hotplug_lock(void)
{
	int online, possible, lock, flag;
	struct work_struct *work;
	struct cpu_dbs_info_s *dbs_info;

	/* do turn_on/off cpus */
	dbs_info = &per_cpu(od_cpu_dbs_info, 0); /* from CPU0 */
	online = num_online_cpus();
	possible = num_possible_cpus();
	lock = atomic_read(&g_hotplug_lock);
	flag = lock - online;

	if (flag == 0)
		return;

	work = flag > 0 ? &dbs_info->up_work : &dbs_info->down_work;

	pr_debug("%s online %d possible %d lock %d flag %d %d\n",
		 __func__, online, possible, lock, flag, (int)abs(flag));

	queue_work_on(dbs_info->cpu, dbs_info.workqueue, work);
}

int cpufreq_eprjdemand_cpu_lock(int num_core)
{
	int prev_lock;

	if (num_core < 1 || num_core > num_possible_cpus())
		return -EINVAL;

	prev_lock = atomic_read(&g_hotplug_lock);

	if (prev_lock != 0 && prev_lock < num_core)
		return -EINVAL;
	else if (prev_lock == num_core)
		atomic_inc(&g_hotplug_count);

	atomic_set(&g_hotplug_lock, num_core);
	atomic_set(&g_hotplug_count, 1);
	apply_hotplug_lock();
	return 0;
}

int cpufreq_eprjdemand_cpu_unlock(int num_core)
{
	int prev_lock = atomic_read(&g_hotplug_lock);

	if (prev_lock < num_core)
		return 0;
	else if (prev_lock == num_core)
		atomic_dec(&g_hotplug_count);

	if (atomic_read(&g_hotplug_count) == 0)
		atomic_set(&g_hotplug_lock, 0);

	return 0;
}

/*
 * History of CPU usage
 */
struct cpu_usage {
	unsigned int freq;
	unsigned int load[NR_CPUS];
	unsigned int rq_avg;
};

struct cpu_usage_history {
	struct cpu_usage usage[MAX_HOTPLUG_RATE];
	unsigned int num_hist;
};

struct cpu_usage_history *hotplug_history;
#endif

static inline cputime64_t get_cpu_idle_time_jiffy(unsigned int cpu,
						  cputime64_t *wall)
{
	cputime64_t idle_time;
	cputime64_t cur_wall_time;
	cputime64_t busy_time;

	cur_wall_time = jiffies64_to_cputime64(get_jiffies_64());
	busy_time = kcpustat_cpu(cpu).cpustat[CPUTIME_USER];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_SYSTEM];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_STEAL];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_NICE];

	idle_time = cur_wall_time - busy_time;
	if (wall)
		*wall = (cputime64_t)jiffies_to_usecs(cur_wall_time);

	return (cputime64_t)jiffies_to_usecs(idle_time);
}

static inline cputime64_t get_cpu_idle_time(unsigned int cpu, cputime64_t *wall)
{
	u64 idle_time = get_cpu_idle_time_us(cpu, wall);

	if (idle_time == -1ULL)
		return get_cpu_idle_time_jiffy(cpu, wall);

	return idle_time;
}

static inline cputime64_t get_cpu_iowait_time(unsigned int cpu,
					      cputime64_t *wall)
{
	u64 iowait_time = get_cpu_iowait_time_us(cpu, wall);

	if (iowait_time == -1ULL)
		return 0;

	return iowait_time;
}

/************************** sysfs interface ************************/

static ssize_t show_sampling_rate_min(struct kobject *kobj,
				      struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", MIN_SAMPLING_RATE);
}

define_one_global_ro(sampling_rate_min);

/* cpufreq_eprjdemand Governor Tunables */
#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)		\
{									\
	return sprintf(buf, "%u\n", dbs_tuners_ins.object);		\
}
show_one(sampling_rate, sampling_rate);
show_one(io_is_busy, io_is_busy);
show_one(up_threshold_at_idle, up_threshold_at_idle);
show_one(sampling_down_factor, sampling_down_factor);
show_one(ignore_nice_load, ignore_nice);
show_one(up_ratio, up_ratio);
show_one(frequency_skew, frequency_skew);
show_one(down_differential, down_differential);
#if defined(USE_OWN_HOTPLUGGING)
show_one(cpu_up_rate, cpu_up_rate);
show_one(cpu_down_rate, cpu_down_rate);
show_one(up_nr_cpus, up_nr_cpus);
show_one(max_cpu_lock, max_cpu_lock);
show_one(dvfs_debug, dvfs_debug);
#endif

#if defined(USE_OWN_HOTPLUGGING)
static ssize_t show_hotplug_lock(struct kobject *kobj,
				struct attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&g_hotplug_lock));
}

#define show_hotplug_param(file_name, num_core, up_down)		\
static ssize_t show_##file_name##_##num_core##_##up_down		\
(struct kobject *kobj, struct attribute *attr, char *buf)		\
{									\
	return sprintf(buf, "%u\n", file_name[num_core - 1][up_down]);	\
}

#define store_hotplug_param(file_name, num_core, up_down)		\
static ssize_t store_##file_name##_##num_core##_##up_down		\
(struct kobject *kobj, struct attribute *attr,				\
	const char *buf, size_t count)					\
{									\
	unsigned int input;						\
	int ret;							\
	ret = sscanf(buf, "%u", &input);				\
	if (ret != 1)							\
		return -EINVAL;						\
	file_name[num_core - 1][up_down] = input;			\
	return count;							\
}

show_hotplug_param(hotplug_freq, 1, 1);
show_hotplug_param(hotplug_freq, 2, 0);
show_hotplug_param(hotplug_freq, 2, 1);
show_hotplug_param(hotplug_freq, 3, 0);
show_hotplug_param(hotplug_freq, 3, 1);
show_hotplug_param(hotplug_freq, 4, 0);

show_hotplug_param(hotplug_rq, 1, 1);
show_hotplug_param(hotplug_rq, 2, 0);
show_hotplug_param(hotplug_rq, 2, 1);
show_hotplug_param(hotplug_rq, 3, 0);
show_hotplug_param(hotplug_rq, 3, 1);
show_hotplug_param(hotplug_rq, 4, 0);

store_hotplug_param(hotplug_freq, 1, 1);
store_hotplug_param(hotplug_freq, 2, 0);
store_hotplug_param(hotplug_freq, 2, 1);
store_hotplug_param(hotplug_freq, 3, 0);
store_hotplug_param(hotplug_freq, 3, 1);
store_hotplug_param(hotplug_freq, 4, 0);

store_hotplug_param(hotplug_rq, 1, 1);
store_hotplug_param(hotplug_rq, 2, 0);
store_hotplug_param(hotplug_rq, 2, 1);
store_hotplug_param(hotplug_rq, 3, 0);
store_hotplug_param(hotplug_rq, 3, 1);
store_hotplug_param(hotplug_rq, 4, 0);

define_one_global_rw(hotplug_freq_1_1);
define_one_global_rw(hotplug_freq_2_0);
define_one_global_rw(hotplug_freq_2_1);
define_one_global_rw(hotplug_freq_3_0);
define_one_global_rw(hotplug_freq_3_1);
define_one_global_rw(hotplug_freq_4_0);

define_one_global_rw(hotplug_rq_1_1);
define_one_global_rw(hotplug_rq_2_0);
define_one_global_rw(hotplug_rq_2_1);
define_one_global_rw(hotplug_rq_3_0);
define_one_global_rw(hotplug_rq_3_1);
define_one_global_rw(hotplug_rq_4_0);
#endif

static ssize_t show_up_thresholds(struct kobject *kobj,
                                  struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u %u\n%u %u\n",
		dbs_tuners_ins.upper_bound_freq,
		dbs_tuners_ins.upper_bound_up_threshold,
		dbs_tuners_ins.lower_bound_freq,
		dbs_tuners_ins.lower_bound_up_threshold
	);
}

static ssize_t store_sampling_rate(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.sampling_rate = max(input, MIN_SAMPLING_RATE);
	return count;
}

static ssize_t store_io_is_busy(struct kobject *a, struct attribute *b,
				const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	dbs_tuners_ins.io_is_busy = !!input;
	return count;
}

static ssize_t store_up_thresholds(struct kobject *a, struct attribute *b,
                                   const char *buf, size_t count)
{
	unsigned int freq[2];
	unsigned int threshold[2];
	int ret = 2;
	int i = 0;
	for (i = 0; i < sizeof(freq) / sizeof(freq[0]) && ret == 2; ++i)
		ret = sscanf(buf, "%u %u", &freq[i], &threshold[i]);

	if (ret != 2 ||
		threshold[0] > MAX_FREQUENCY_UP_THRESHOLD ||
		threshold[0] < MIN_FREQUENCY_UP_THRESHOLD ||
		threshold[1] > MAX_FREQUENCY_UP_THRESHOLD ||
		threshold[1] < MIN_FREQUENCY_UP_THRESHOLD) {
		return -EINVAL;
	}
	dbs_tuners_ins.upper_bound_freq = freq[0];
	dbs_tuners_ins.lower_bound_freq = freq[1];
	dbs_tuners_ins.upper_bound_up_threshold = threshold[0];
	dbs_tuners_ins.lower_bound_up_threshold = threshold[1];
	return count;
}

static ssize_t store_up_threshold_at_idle(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_FREQUENCY_UP_THRESHOLD ||
	    input < MIN_FREQUENCY_UP_THRESHOLD) {
		return -EINVAL;
	}
	dbs_tuners_ins.up_threshold_at_idle = input;
	return count;
}

static ssize_t store_sampling_down_factor(struct kobject *a,
					  struct attribute *b,
					  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_SAMPLING_DOWN_FACTOR || input < 1)
		return -EINVAL;
	dbs_tuners_ins.sampling_down_factor = input;

	/* Reset down sampling multiplier in case it was active */
	dbs_info.rate_mult = 1;
	return count;
}

static ssize_t store_ignore_nice_load(struct kobject *a, struct attribute *b,
				      const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == dbs_tuners_ins.ignore_nice) { /* nothing to do */
		return count;
	}
	dbs_tuners_ins.ignore_nice = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->prev_cpu_idle =
			get_cpu_idle_time(j, &dbs_info->prev_cpu_wall);
		if (dbs_tuners_ins.ignore_nice)
			dbs_info->prev_cpu_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE];
	}
	return count;
}

static ssize_t store_up_ratio(struct kobject *a, struct attribute *b,
			      const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.up_ratio = min(input, MAX_FREQUENCY_UP_THRESHOLD);
	return count;
}

static ssize_t store_frequency_skew(struct kobject *a, struct attribute *b,
			           const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.frequency_skew = min(input, MAX_FREQUENCY_SKEW);
	return count;
}

static ssize_t store_down_differential(struct kobject *a, struct attribute *b,
				       const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.down_differential = min(input, 100u);
	return count;
}

#if defined(USE_OWN_HOTPLUGGING)
static ssize_t store_cpu_up_rate(struct kobject *a, struct attribute *b,
				 const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.cpu_up_rate = min(input, MAX_HOTPLUG_RATE);
	return count;
}

static ssize_t store_cpu_down_rate(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.cpu_down_rate = min(input, MAX_HOTPLUG_RATE);
	return count;
}


static ssize_t store_up_nr_cpus(struct kobject *a, struct attribute *b,
				const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.up_nr_cpus = min(input, num_possible_cpus());
	return count;
}

static ssize_t store_max_cpu_lock(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.max_cpu_lock = min(input, num_possible_cpus());
	return count;
}

static ssize_t store_hotplug_lock(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	int prev_lock;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	input = min(input, num_possible_cpus());
	prev_lock = atomic_read(&dbs_tuners_ins.hotplug_lock);

	if (prev_lock)
		cpufreq_eprjdemand_cpu_unlock(prev_lock);

	if (input == 0) {
		atomic_set(&dbs_tuners_ins.hotplug_lock, 0);
		return count;
	}

	ret = cpufreq_eprjdemand_cpu_lock(input);
	if (ret) {
		pr_info("[EPRJDEMAND] [HOTPLUG] already locked with smaller value %d < %d\n",
			atomic_read(&g_hotplug_lock), input);
		return ret;
	}

	atomic_set(&dbs_tuners_ins.hotplug_lock, input);

	return count;
}

static ssize_t store_dvfs_debug(struct kobject *a, struct attribute *b,
				const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.dvfs_debug = input > 0;
	return count;
}
#endif

define_one_global_rw(sampling_rate);
define_one_global_rw(io_is_busy);
define_one_global_rw(up_thresholds);
define_one_global_rw(up_threshold_at_idle);
define_one_global_rw(sampling_down_factor);
define_one_global_rw(ignore_nice_load);
define_one_global_rw(up_ratio);
define_one_global_rw(frequency_skew);
define_one_global_rw(down_differential);
#if defined(USE_OWN_HOTPLUGGING)
define_one_global_rw(cpu_up_rate);
define_one_global_rw(cpu_down_rate);
define_one_global_rw(up_nr_cpus);
define_one_global_rw(max_cpu_lock);
define_one_global_rw(hotplug_lock);
define_one_global_rw(dvfs_debug);
#endif

static struct attribute *dbs_attributes[] = {
	&sampling_rate_min.attr,
	&sampling_rate.attr,
	&up_thresholds.attr,
	&up_threshold_at_idle.attr,
	&sampling_down_factor.attr,
	&ignore_nice_load.attr,
	&io_is_busy.attr,
	&up_ratio.attr,
	&frequency_skew.attr,
	&down_differential.attr,
#if defined(USE_OWN_HOTPLUGGING)
	&cpu_up_rate.attr,
	&cpu_down_rate.attr,
	&up_nr_cpus.attr,
	/* priority: hotplug_lock > max_cpu_lock */
	&max_cpu_lock.attr,
	&hotplug_lock.attr,
	&dvfs_debug.attr,
	&hotplug_freq_1_1.attr,
	&hotplug_freq_2_0.attr,
	&hotplug_freq_2_1.attr,
	&hotplug_freq_3_0.attr,
	&hotplug_freq_3_1.attr,
	&hotplug_freq_4_0.attr,
	&hotplug_rq_1_1.attr,
	&hotplug_rq_2_0.attr,
	&hotplug_rq_2_1.attr,
	&hotplug_rq_3_0.attr,
	&hotplug_rq_3_1.attr,
	&hotplug_rq_4_0.attr,
#endif
	NULL
};

static struct attribute_group dbs_attr_group = {
	.attrs = dbs_attributes,
	.name = "eprjdemand",
};

/************************** sysfs end ************************/

#if defined(USE_OWN_HOTPLUGGING)
static void __cpuinit cpu_up_work(struct work_struct *work)
{
	int cpu;
	int online = num_online_cpus();
	int nr_up = dbs_tuners_ins.up_nr_cpus;
	int hotplug_lock = atomic_read(&g_hotplug_lock);
	if (hotplug_lock)
		nr_up = hotplug_lock - online;

	if (online == 1) {
		pr_info("[EPRJDEMAND] CPU_UP 3\n");
		cpu_up(num_possible_cpus() - 1);
		nr_up -= 1;
	}

	for_each_cpu_not(cpu, cpu_online_mask) {
		if (nr_up-- == 0)
			break;
		if (cpu == 0)
			continue;
		pr_info("[EPRJDEMAND] CPU_UP %d\n", cpu);
		cpu_up(cpu);
	}
}

static void cpu_down_work(struct work_struct *work)
{
	int cpu;
	int online = num_online_cpus();
	int nr_down = 1;
	int hotplug_lock = atomic_read(&g_hotplug_lock);

	if (hotplug_lock)
		nr_down = online - hotplug_lock;

	while (--nr_down >= 0) {
		/* Find the CPU with the lowest frequency */
		unsigned int min_freq = UINT_MAX;
		int min_load_cpu = -1;
		printk("Offline attempt: freqs");
		for_each_online_cpu(cpu) {
			unsigned int freq;

			/* Do not offline CPU 0 */
			if (cpu == 0)
				continue;

			freq = cpufreq_quick_get(cpu);
			if (!freq) {
				continue;
			}
			printk(" %d=%d", cpu, freq);
			
			if (freq < min_freq) {
				min_freq = freq;
				min_load_cpu = cpu;
			}
		}
		printk("\nDecided: Offline CPU %d at freq %u\n", min_load_cpu, min_freq);
		if (min_load_cpu == 0) {
			pr_err("Attempting to offline CPU 0, ignoring...");
			break;
		}
		if (min_load_cpu == -1) {
			pr_warn("CPUs all with maximum load, offlining last CPU");
			for_each_online_cpu(cpu) {
				if (cpu == 0)
					continue;
				min_load_cpu = cpu;
			}

			if (min_load_cpu == -1) {
				pr_err("No CPUs online!");
				return;
			}
		}
		printk(KERN_ERR "CPU_DOWN %d\n", min_load_cpu);
		cpu_down(min_load_cpu);
	}
}
#endif

/*
 * SMP-aware frequency setting. This will set the same frequency across
 * all CPUs.
 */
static void dbs_freq_target(unsigned int freq)
{
	unsigned int j;

	for_each_cpu(j, dbs_info.dbs_enable_cpumask) {
		struct cpu_dbs_info_s *j_dbs_info;

		j_dbs_info = &per_cpu(od_cpu_dbs_info, j);
		__cpufreq_driver_target(j_dbs_info->cur_policy, freq, CPUFREQ_RELATION_L);
	}
}

#if defined(USE_OWN_HOTPLUGGING)
/*
 * print hotplug debugging info.
 * which 1 : UP, 0 : DOWN
 */
static void debug_hotplug_check(int which, int rq_avg, int freq,
			 struct cpu_usage *usage)
{
	int cpu;
	pr_info("CHECK %s rq %d.%02d freq %d [", which ? "up" : "down",
	       rq_avg / 100, rq_avg % 100, freq);
	for_each_online_cpu(cpu) {
		pr_info("(%d, %d), ", cpu, usage->load[cpu]);
	}
	pr_info("--- END ---\n");
}

static int check_up(void)
{
	int num_hist = hotplug_history->num_hist;
	struct cpu_usage *usage;
	int freq, rq_avg;
	int i;
	int up_rate = dbs_tuners_ins.cpu_up_rate;
	int up_freq, up_rq;
	int min_freq = INT_MAX;
	int min_rq_avg = INT_MAX;
	int online;
	int hotplug_lock = atomic_read(&g_hotplug_lock);

	if (hotplug_lock > 0)
		return 0;

	online = num_online_cpus();
	up_freq = hotplug_freq[online - 1][HOTPLUG_UP_INDEX];
	up_rq = hotplug_rq[online - 1][HOTPLUG_UP_INDEX];

	if (online == num_possible_cpus())
		return 0;
	if (dbs_tuners_ins.max_cpu_lock != 0
		&& online >= dbs_tuners_ins.max_cpu_lock)
		return 0;

	if (num_hist == 0 || num_hist % up_rate)
		return 0;

	for (i = num_hist - 1; i >= num_hist - up_rate; --i) {
		usage = &hotplug_history->usage[i];

		freq = usage->freq;
		rq_avg =  usage->rq_avg;

		min_freq = min(min_freq, freq);
		min_rq_avg = min(min_rq_avg, rq_avg);

		if (dbs_tuners_ins.dvfs_debug)
			debug_hotplug_check(1, rq_avg, freq, usage);
	}

	if (min_freq >= up_freq && min_rq_avg > up_rq) {
		pr_debug("[EPRJDEMAND] [HOTPLUG IN] %s %d>=%d && %d>%d\n",
			__func__, min_freq, up_freq, min_rq_avg, up_rq);
		hotplug_history->num_hist = 0;
		return 1;
	}
	return 0;
}

static int check_down(void)
{
	int num_hist = hotplug_history->num_hist;
	struct cpu_usage *usage;
	int freq, rq_avg;
	int i;
	int down_rate = dbs_tuners_ins.cpu_down_rate;
	int down_freq, down_rq;
	int min_freq = INT_MAX;
	int max_rq_avg = 0;
	int online;
	int hotplug_lock = atomic_read(&g_hotplug_lock);

	if (hotplug_lock > 0)
		return 0;

	online = num_online_cpus();
	down_freq = hotplug_freq[online - 1][HOTPLUG_DOWN_INDEX];
	down_rq = hotplug_rq[online - 1][HOTPLUG_DOWN_INDEX];

	if (online == 1)
		return 0;

	if (dbs_tuners_ins.max_cpu_lock != 0
		&& online > dbs_tuners_ins.max_cpu_lock)
		return 1;

	if (num_hist == 0 || num_hist % down_rate)
		return 0;

	for (i = num_hist - 1; i >= num_hist - down_rate; --i) {
		usage = &hotplug_history->usage[i];

		freq = usage->freq;
		rq_avg =  usage->rq_avg;

		min_freq = min(min_freq, freq);
		max_rq_avg = max(max_rq_avg, rq_avg);

		if (dbs_tuners_ins.dvfs_debug)
			debug_hotplug_check(0, rq_avg, freq, usage);
	}

	if (min_freq <= down_freq || max_rq_avg <= down_rq) {
		pr_debug("[EPRJDEMAND] [HOTPLUG OUT] %s %d<=%d || %d<%d\n",
			__func__, min_freq, down_freq, max_rq_avg, down_rq);
		hotplug_history->num_hist = 0;
		return 1;
	}

	return 0;
}
#endif

/**
 * Helper function to extrapolate the lower and upper bounds of the frequency
 * ladder.
 *
 * \param freq The current frequency of the CPUs.
 */
static unsigned dbs_get_up_threshold(unsigned freq)
{
	if (freq <= dbs_tuners_ins.lower_bound_freq) {
		return dbs_tuners_ins.lower_bound_up_threshold;
	} else if (freq >= dbs_tuners_ins.upper_bound_freq) {
		return dbs_tuners_ins.upper_bound_up_threshold;
	} else {
		/* Find the mille of the current frequency wrt the entire
		   ladder. (per mille = 0.1%) */
		unsigned ladder_position =
			(freq - dbs_tuners_ins.lower_bound_freq) * 1000 /
				(dbs_tuners_ins.upper_bound_freq -
					dbs_tuners_ins.lower_bound_freq);

		unsigned threshold = dbs_tuners_ins.lower_bound_up_threshold +
			ladder_position *
				(dbs_tuners_ins.upper_bound_up_threshold -
					dbs_tuners_ins.lower_bound_up_threshold) /
				1000;

		pr_debug("Freq: %u, ladder: %u, threshold: %u\n",
			 freq, ladder_position, threshold);
		return threshold;
	}
}

/**
 * This is the main code in eprjdemand.
 *
 * This will examine the loads of each CPU enabled with this governor, and
 * compute the frequency to be shared among all CPUs.
 *
 * \remarks The CPU policy which is used is ALWAYS the lowest-numbered CPU
 *          registered with eprjdemand. This may be a problem if the lowest-
 *          numbered CPU changes between invocations (unlikely, but possible)
 * \todo Make this function completely CPU agnostic (so we can run on any of
 *       the CPUs registered with us). Also, dispense with the need of having
 *       a policy object as well as the "this_dbs_info" variable.
 */
static void dbs_check_cpu(void)
{
	unsigned int requested_load_freq;

	struct cpu_dbs_info_s *this_dbs_info;
	struct cpufreq_policy *policy;
	unsigned int j;
#if defined(USE_OWN_HOTPLUGGING)
	int num_hist = hotplug_history->num_hist;
	int max_hotplug_rate = max(dbs_tuners_ins.cpu_up_rate,
				   dbs_tuners_ins.cpu_down_rate);
#endif
	unsigned int up_threshold;

	this_dbs_info = &per_cpu(od_cpu_dbs_info,
		cpumask_first(dbs_info.dbs_enable_cpumask));
	policy = this_dbs_info->cur_policy;
	up_threshold = dbs_info.early_suspend ?
		dbs_tuners_ins.up_threshold_at_idle :
		dbs_get_up_threshold(policy->cur);

#if defined(USE_OWN_HOTPLUGGING)
	hotplug_history->usage[num_hist].freq = policy->cur;
#ifndef CONFIG_EPRJ_SYSFS_TOOLS
	hotplug_history->usage[num_hist].rq_avg = get_nr_run_avg();
#else
	/* Avoid bad workqueue duplication */
	hotplug_history->usage[num_hist].rq_avg = eprj_get_nr_run_avg();
#endif
	++hotplug_history->num_hist;
#endif

	/* Get Absolute Load - in terms of freq */
	{
		unsigned int min_load_freq = UINT_MAX,
			max_load_freq = 0,
			total_load_freq = 0,
			average_load_freq = 0;

		for_each_cpu(j, dbs_info.dbs_enable_cpumask) {
			struct cpu_dbs_info_s *j_dbs_info;
			cputime64_t cur_wall_time, cur_idle_time, cur_iowait_time;
			unsigned int idle_time, wall_time, iowait_time;
			unsigned int load;
			int freq_avg;

			j_dbs_info = &per_cpu(od_cpu_dbs_info, j);

			cur_idle_time = get_cpu_idle_time(j, &cur_wall_time);
			cur_iowait_time = get_cpu_iowait_time(j, &cur_wall_time);

			wall_time = (unsigned int)
				(cur_wall_time - j_dbs_info->prev_cpu_wall);
			j_dbs_info->prev_cpu_wall = cur_wall_time;

			idle_time = (unsigned int)
				(cur_idle_time - j_dbs_info->prev_cpu_idle);
			j_dbs_info->prev_cpu_idle = cur_idle_time;

			iowait_time = (unsigned int)
				(cur_iowait_time - j_dbs_info->prev_cpu_iowait);
			j_dbs_info->prev_cpu_iowait = cur_iowait_time;

			if (dbs_tuners_ins.ignore_nice) {
				cputime64_t cur_nice;
				unsigned long cur_nice_jiffies;

				cur_nice = (kcpustat_cpu(j).cpustat[CPUTIME_NICE] -
							j_dbs_info->prev_cpu_nice);
				/*
				* Assumption: nice time between sampling periods will
				* be less than 2^32 jiffies for 32 bit sys
				*/
				cur_nice_jiffies = (unsigned long)
					cputime64_to_jiffies64(cur_nice);

				j_dbs_info->prev_cpu_nice =
					kcpustat_cpu(j).cpustat[CPUTIME_NICE];
				idle_time += jiffies_to_usecs(cur_nice_jiffies);
			}

			if (!dbs_tuners_ins.io_is_busy)
				idle_time += iowait_time;

			if (unlikely(!wall_time || wall_time < idle_time))
				continue;

			load = 100 * (wall_time - idle_time) / wall_time;
#if defined(USE_OWN_HOTPLUGGING)
			hotplug_history->usage[num_hist].load[j] = load;
#endif

			freq_avg = __cpufreq_driver_getavg(j_dbs_info->cur_policy, j);
			if (freq_avg <= 0)
				freq_avg = j_dbs_info->cur_policy->cur;

			freq_avg *= load;
			min_load_freq = min(min_load_freq, (unsigned int)freq_avg);
			max_load_freq = max(max_load_freq, (unsigned int)freq_avg);
			total_load_freq += freq_avg;
		}

		/* Compute the load frequency which should drive our frequency decision.
		   This is with 0 at minimum frequency, 100 at maximum frequency, and 50
		   being the average. Intermediate values are a scale between the average
		   and the minimum/maximum. */
		average_load_freq = total_load_freq / max(1U, min(
			cpumask_weight(dbs_info.dbs_enable_cpumask),
			eprj_get_nr_run_avg() / 100));

		if (dbs_tuners_ins.frequency_skew == 0) {
			requested_load_freq = min_load_freq;
		} else if (dbs_tuners_ins.frequency_skew == MAX_FREQUENCY_SKEW) {
			requested_load_freq = max_load_freq;
		} else if (dbs_tuners_ins.frequency_skew == 50) {
			requested_load_freq = average_load_freq;

		/* we are somewhere between 50 and min/max. */
		} else if (dbs_tuners_ins.frequency_skew >= 51) {
			unsigned percent_of_max = 2 *
				(dbs_tuners_ins.frequency_skew - 50);
			unsigned skew = (max_load_freq - average_load_freq) *
				percent_of_max / 100;
			requested_load_freq = average_load_freq + skew;
		} else {
			unsigned percent_of_max = 2 * dbs_tuners_ins.frequency_skew;
			unsigned skew = (average_load_freq - min_load_freq) *
				percent_of_max / 100;
			requested_load_freq = min_load_freq + skew;
		}
	}

#if defined(USE_OWN_HOTPLUGGING)
	/* Check for CPU hotplug */
	if (check_up()) {
		queue_work_on(this_dbs_info->cpu, dbs_info.workqueue,
			      &this_dbs_info->up_work);
	} else if (check_down()) {
		queue_work_on(this_dbs_info->cpu, dbs_info.workqueue,
			      &this_dbs_info->down_work);
	}
	if (hotplug_history->num_hist  == max_hotplug_rate)
		hotplug_history->num_hist = 0;
#endif

	if (requested_load_freq > up_threshold * policy->cur) {
		int inc;
		int target;

		if (up_threshold == 100) {
			/* Can't divide by zero, so just go up by the maximum up
			   ratio. */
			inc = (policy->max * dbs_tuners_ins.up_ratio) / 100;
		} else {
			/* Try to predict the reduction in load after we scale to
			  the new frequency. */
			int load_above_target = requested_load_freq / policy->cur -
				up_threshold;
			int up_threshold_steps = 100 - up_threshold;

			/* Increment (load above target)% wrt to the current
			   frequency:

			   inc = policy->cur * load_above_target / up_threshold_steps *
			       dbs_tuners_ins.up_ratio / 100;

			   Rearrange to reduce rounding error. */
			inc = policy->cur * load_above_target * dbs_tuners_ins.up_ratio /
				(up_threshold_steps * 100);
			pr_debug("[eprjdemand] load %u, freq %u, curr_freq %u, "
				"load_above_target %d, increment %d",
				requested_load_freq / policy->cur,
				requested_load_freq / 100, policy->cur,
				load_above_target, inc
			);
		}

		target = min(policy->max, policy->cur + inc);

		/* If switching to max speed, apply sampling_down_factor */
		if (policy->cur < policy->max && target == policy->max)
			dbs_info.rate_mult =
				dbs_tuners_ins.sampling_down_factor;
		dbs_freq_target(target);
		return;
	}

	/* Check for frequency decrease */
#ifndef CONFIG_ARCH_EXYNOS4
	/* if we cannot reduce the frequency anymore, break out early */

	if (policy->cur == policy->min)
		return;
#endif

	/*
	 * The optimal frequency is the frequency that is the lowest that
	 * can support the current CPU usage without triggering the up
	 * policy. To be safe, we focus DOWN_DIFFERENTIAL points under
	 * the threshold.
	 */
	if (requested_load_freq <
	    (up_threshold - dbs_tuners_ins.down_differential) *
	    policy->cur) {
		unsigned int freq_next;

		freq_next = requested_load_freq /
			(up_threshold -
			 dbs_tuners_ins.down_differential);

		/* No longer fully busy, reset rate_mult */
		dbs_info.rate_mult = 1;

		if (freq_next < policy->min)
			freq_next = policy->min;

		if (policy->cur == freq_next)
			return;

		dbs_freq_target(freq_next);

#if defined(CONFIG_ARCH_TEGRA_3x_SOC) && defined(USE_OWN_HOTPLUGGING)
		/*
		 * If we want performance, we will power up the G cluster.
		 * Otherwise, save power and switch to LP.
		 *
		 * **WARNING** We can't wakeup other cores while in LP!
		 */
		if ((freq_next > FREQ_LPTOG) && !isactive_gcluster())
			eprj_cluster_want_performance(1);
		else if ((freq_next <= FREQ_LPTOG) && isactive_gcluster())
			eprj_cluster_want_performance(0);
#endif
	}
}

static void do_dbs_timer(struct work_struct *work)
{
	int delay;

	mutex_lock(&dbs_info.timer_mutex);

	dbs_check_cpu();
	delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);
	delay *= dbs_info.rate_mult +
		(dbs_info.early_suspend ? SLEEP_SAMPLING_RATE_FACTOR : 1);

	queue_delayed_work(dbs_info.workqueue, &dbs_info.work, delay);
	mutex_unlock(&dbs_info.timer_mutex);
}

static inline void dbs_timer_init(void)
{
	int delay = usecs_to_jiffies(DEF_START_DELAY * 1000 * 1000
				     + dbs_tuners_ins.sampling_rate);

	INIT_DELAYED_WORK_DEFERRABLE(&dbs_info.work, do_dbs_timer);
#if defined(USE_OWN_HOTPLUGGING)
	INIT_WORK(&dbs_info.up_work, cpu_up_work);
	INIT_WORK(&dbs_info.down_work, cpu_down_work);
#endif

	queue_delayed_work(dbs_info.workqueue, &dbs_info.work,
		delay + 2 * HZ);
}

static inline void dbs_timer_exit(void)
{
	cancel_delayed_work_sync(&dbs_info.work);
#if defined(USE_OWN_HOTPLUGGING)
	cancel_work_sync(&dbs_info.up_work);
	cancel_work_sync(&dbs_info.down_work);
#endif
}

#if !defined(CONFIG_ARCH_TEGRA_3x_SOC) && defined(USE_OWN_HOTPLUGGING)
static int pm_notifier_call(struct notifier_block *this,
			    unsigned long event, void *ptr)
{
	static unsigned int prev_hotplug_lock;
	switch (event) {
	case PM_SUSPEND_PREPARE:
		prev_hotplug_lock = atomic_read(&g_hotplug_lock);
		atomic_set(&g_hotplug_lock, 1);
		apply_hotplug_lock();
		pr_debug("%s enter suspend\n", __func__);
		return NOTIFY_OK;
	case PM_POST_RESTORE:
	case PM_POST_SUSPEND:
		atomic_set(&g_hotplug_lock, prev_hotplug_lock);
		if (prev_hotplug_lock)
			apply_hotplug_lock();
		prev_hotplug_lock = 0;
		pr_debug("%s exit suspend\n", __func__);
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static struct notifier_block pm_notifier = {
	.notifier_call = pm_notifier_call,
};
#endif

#if defined(USE_OWN_HOTPLUGGING)
static int reboot_notifier_call(struct notifier_block *this,
				unsigned long code, void *_cmd)
{
	atomic_set(&g_hotplug_lock, 1);
	return NOTIFY_DONE;
}

static struct notifier_block reboot_notifier = {
	.notifier_call = reboot_notifier_call,
};
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend early_suspend;
static void cpufreq_eprjdemand_early_suspend(struct early_suspend *h)
{
#if defined(USE_OWN_HOTPLUGGING)
	dbs_tuners_ins.early_suspend_hotplug_lock =
		atomic_read(&g_hotplug_lock);
	atomic_set(&g_hotplug_lock, 1);
	apply_hotplug_lock();
#ifndef CONFIG_EPRJ_SYSFS_TOOLS
	stop_rq_work();
#endif
#endif
	dbs_info.early_suspend = true;
}

static void cpufreq_eprjdemand_late_resume(struct early_suspend *h)
{
	dbs_info.early_suspend = false;
#if defined(USE_OWN_HOTPLUGGING)
	atomic_set(&g_hotplug_lock, dbs_tuners_ins.early_suspend_hotplug_lock);
	dbs_tuners_ins.early_suspend_hotplug_lock = -1;
	apply_hotplug_lock();
#ifndef CONFIG_EPRJ_SYSFS_TOOLS
	start_rq_work();
#endif
#endif
}
#endif

static int cpufreq_governor_dbs(struct cpufreq_policy *policy,
				unsigned int event)
{
	unsigned int cpu = policy->cpu;

	struct cpu_dbs_info_s *this_dbs_info;
	unsigned int j;
	int rc;

	this_dbs_info = &per_cpu(od_cpu_dbs_info, cpu);
	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!policy->cur))
			return -EINVAL;

#if defined(USE_OWN_HOTPLUGGING)
		hotplug_history->num_hist = 0;
#ifndef CONFIG_EPRJ_SYSFS_TOOLS
		start_rq_work();
#endif
#endif
		mutex_lock(&dbs_info.dbs_mutex);

		dbs_info.dbs_enable++;
		cpumask_set_cpu(cpu, dbs_info.dbs_enable_cpumask);
		for_each_cpu(j, policy->cpus) {
			struct cpu_dbs_info_s *j_dbs_info;
			j_dbs_info = &per_cpu(od_cpu_dbs_info, j);
			j_dbs_info->cur_policy = policy;

			j_dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
				&j_dbs_info->prev_cpu_wall);
			if (dbs_tuners_ins.ignore_nice) {
				j_dbs_info->prev_cpu_nice =
					kcpustat_cpu(j).cpustat[CPUTIME_NICE];
			}
		}
		this_dbs_info->cpu = cpu;
		/*
		 * Start the timerschedule work, when this governor
		 * is used for first time
		 */
		if (dbs_info.dbs_enable == 1) {
			rc = sysfs_create_group(cpufreq_global_kobject,
						&dbs_attr_group);
			if (rc) {
				mutex_unlock(&dbs_info.dbs_mutex);
				return rc;
			}

			dbs_tuners_ins.sampling_rate = DEF_SAMPLING_RATE;
			dbs_tuners_ins.upper_bound_freq = DEF_FREQUENCY_UPPER_BOUND_FREQ,
			dbs_tuners_ins.upper_bound_up_threshold = DEF_FREQUENCY_UPPER_BOUND_THRESHOLD;
			dbs_tuners_ins.lower_bound_freq = DEF_FREQUENCY_LOWER_BOUND_FREQ,
			dbs_tuners_ins.lower_bound_up_threshold = DEF_FREQUENCY_LOWER_BOUND_THRESHOLD;
			dbs_tuners_ins.up_threshold_at_idle = DEF_UP_THRESHOLD_AT_IDLE,
			dbs_tuners_ins.up_ratio = DEF_FREQUENCY_UP_RATIO,
			dbs_tuners_ins.frequency_skew = DEF_FREQUENCY_SKEW,
			dbs_tuners_ins.down_differential = DEF_FREQUENCY_DOWN_DIFFERENTIAL,
			dbs_tuners_ins.ignore_nice = 0,
			dbs_tuners_ins.sampling_down_factor = DEF_SAMPLING_DOWN_FACTOR,
			dbs_tuners_ins.io_is_busy = 0;

#if defined(USE_OWN_HOTPLUGGING)
#ifndef CONFIG_ARCH_TEGRA_3x_SOC
			register_pm_notifier(&pm_notifier);
#endif
			register_reboot_notifier(&reboot_notifier);
#endif

			dbs_info.rate_mult = 1;
			dbs_timer_init();

#ifdef CONFIG_HAS_EARLYSUSPEND
			register_early_suspend(&early_suspend);
#endif
#if defined(CONFIG_ARCH_TEGRA_3x_SOC) && defined(USE_OWN_HOTPLUGGING)
			manage_auto_hotplug(0);
#endif
		}
		mutex_unlock(&dbs_info.dbs_mutex);
		break;

	case CPUFREQ_GOV_STOP:
		mutex_lock(&dbs_info.dbs_mutex);
		cpumask_clear_cpu(cpu, dbs_info.dbs_enable_cpumask);
		dbs_info.dbs_enable--;

		if (dbs_info.dbs_enable == 0) {
#if defined(CONFIG_ARCH_TEGRA_3x_SOC) && defined(USE_OWN_HOTPLUGGING)
			manage_auto_hotplug(1);
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
			unregister_early_suspend(&early_suspend);
#endif

			dbs_timer_exit();

#if defined(USE_OWN_HOTPLUGGING)
			unregister_reboot_notifier(&reboot_notifier);
#ifndef CONFIG_ARCH_TEGRA_3x_SOC
			unregister_pm_notifier(&pm_notifier);
#endif
#endif

			sysfs_remove_group(cpufreq_global_kobject,
					   &dbs_attr_group);
		}

		mutex_unlock(&dbs_info.dbs_mutex);

#if defined(CONFIG_EPRJ_SYSFS_TOOLS) && defined(USE_OWN_HOTPLUGGING)
		stop_rq_work();
#endif
		break;

	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&dbs_info.timer_mutex);

		if (policy->max < this_dbs_info->cur_policy->cur)
			__cpufreq_driver_target(this_dbs_info->cur_policy,
						policy->max,
						CPUFREQ_RELATION_H);
		else if (policy->min > this_dbs_info->cur_policy->cur)
			__cpufreq_driver_target(this_dbs_info->cur_policy,
						policy->min,
						CPUFREQ_RELATION_L);

		mutex_unlock(&dbs_info.timer_mutex);
		break;
	}
	return 0;
}

static int __init cpufreq_gov_dbs_init(void)
{
	int ret;

#ifdef USE_OWN_HOTPLUGGING
#if !defined(CONFIG_EPRJ_SYSFS_TOOLS)
	ret = init_rq_avg();
	if (ret)
		return ret;
#endif

	hotplug_history = kzalloc(sizeof(struct cpu_usage_history), GFP_KERNEL);
	if (!hotplug_history) {
		pr_err("%s cannot create hotplug history array\n", __func__);
		ret = -ENOMEM;
		goto err_hist;
	}
#endif

	mutex_init(&dbs_info.dbs_mutex);
	mutex_init(&dbs_info.timer_mutex);

	dbs_info.workqueue = create_workqueue("keprjdemand");
	if (!dbs_info.workqueue) {
		pr_err("%s cannot create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_queue;
	}

	if (!alloc_cpumask_var(&dbs_info.dbs_enable_cpumask, GFP_KERNEL))
		goto err_cpumask;

	ret = cpufreq_register_governor(&cpufreq_gov_eprjdemand);
	if (ret)
		goto err_reg;

#ifdef CONFIG_HAS_EARLYSUSPEND
	early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	early_suspend.suspend = cpufreq_eprjdemand_early_suspend;
	early_suspend.resume = cpufreq_eprjdemand_late_resume;

	dbs_info.early_suspend = false;
#if defined(USE_OWN_HOTPLUGGING)
	dbs_info.early_suspend_hotplug_lock = -1;
#endif
#endif
	return ret;

err_reg:
	free_cpumask_var(dbs_info.dbs_enable_cpumask);
err_cpumask:
	destroy_workqueue(dbs_info.workqueue);
err_queue:
#if defined(USE_OWN_HOTPLUGGING)
	kfree(hotplug_history);
err_hist:
#ifndef CONFIG_EPRJ_SYSFS_TOOLS
	kfree(rq_data);
#endif
#endif
	return ret;
}

static void __exit cpufreq_gov_dbs_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_eprjdemand);
	free_cpumask_var(dbs_info.dbs_enable_cpumask);
	destroy_workqueue(dbs_info.workqueue);

	mutex_destroy(&dbs_info.timer_mutex);
	mutex_destroy(&dbs_info.dbs_mutex);

#if defined(USE_OWN_HOTPLUGGING)
	kfree(hotplug_history);
#ifndef CONFIG_EPRJ_SYSFS_TOOLS
	kfree(rq_data);
#endif
#endif
}

MODULE_AUTHOR("Angelo G. Del Regno <kholk11@gmail.com>");
MODULE_DESCRIPTION("'cpufreq_eprjdemand' - A dynamic cpufreq/cpuhotplug governor");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_EPRJDEMAND
fs_initcall(cpufreq_gov_dbs_init);
#else
module_init(cpufreq_gov_dbs_init);
#endif
module_exit(cpufreq_gov_dbs_exit);
