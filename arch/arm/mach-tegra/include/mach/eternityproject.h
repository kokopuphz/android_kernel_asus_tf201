/*
 * arch/arm/mach-tegra/include/mach/eternityproject.h
 *
 * Copyright (C) 2012, EternityProject Development
 *
 * Author:
 * 	Angelo G. Del Regno <kholk11@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __MACH_TEGRA_ETERNITYPROJECT_H
#define __MACH_TEGRA_ETERNITYPROJECT_H
#endif

#include "../../board.h"	/* Thanks for the bad hacks, nVidia! */

#define EPRJ_DEBUGGING	1

/*
 * Tegra 3 Clocks Management
 *
 * TODO: Make everything better and use the code in there
 *	 to simplify Tegra clocks management.
 */
#define BASEFREQ	51
#define eprjc(c)	(BASEFREQ * c)		/* For cpu-tegra's CPU_DVFS */
#define eprjf(c)	(BASEFREQ * c * 1000)	/* For cpufreq tables */

/*
 * EDP Management
 */
#define FAC		100000

#ifdef CONFIG_ETERNITYPROJECT_CPUFMAN
#define EEDP_MAX 	1700000			/* Highest EDP Clock */
#define EPRJEDP1	(EEDP_MAX - FAC)
#else
#define EEDP_MAX	1500000
#define EPRJEDP1	EEDP_MAX
#endif

#define EPRJEDP2	(EPRJEDP1 - FAC)
#define EPRJEDP3	(EPRJEDP2 - FAC)

#define COOLDWN0	1000000			/* CPU is HOT! */
#define COOLDWN1	(COOLDWN0 + FAC)
#define COOLDWN2	(COOLDWN1 + FAC)
#define COOLDWN3	(COOLDWN2 + FAC)

#define CRITICAL	204000			/* CPU is MELTING! */

/*
 * TODO: GPU Clocks Management
 */


/*
 * TODO: EternityProject CPUFREQ Governor (eprjdemand)
 *	 -> Tegra highly specific things has to be there
 *	    to mantain the cpufreq governor code clean
 *	    and GENERIC!
 */
#if 0 /* Silence warnings until the code will be used */
#define EPRJDEMAND_GOVERNOR	"eprjdemand"
static void eternityproject_governor_enable(void)
{
#if EPRJ_DEBUGGING
	printk("EternityProject: Setting eprjdemand governor...\n");
#endif
	cpufreq_set_governor(EPRJDEMAND_GOVERNOR);
}
#endif

/*
 * EternityProject sysfs Tools
 */

/* Android API Levels */
#define ANDROID_API_ICS		15
#define ANDROID_API_JB		16

/* Debug */
#define EPRJ_CHATWITHME
#if defined(EPRJ_CHATWITHME)
#define EPRJ_PRINT(c) \
		pr_info(c)
#else
#define EPRJ_PRINT(c)
#endif

void eprj_hsmgr_35mm_os(unsigned short int); 	/* Headset Compatibility 	*/
void eprj_extreme_powersave(bool);		/* Powersave - LP Cluster Lock	*/
uint8_t __eprjsearchlock(char *lock);

/* Wakelock Blacklist Management */
#define MAXSTRINGS	20
static inline uint8_t is_blacklisted(const char *name)
{
	char *nname = (char*) name;
	return __eprjsearchlock(nname);
}

struct eprj_sysfs {
	struct attribute attr;
	int value;
	char *strings[MAXSTRINGS];
};

extern bool eprj_chargeboost;
extern bool eprj_fryme;

/* Initial implementation of userspace voltage control */
#define MAX_ALLOWED_VOLT 1350

/* The number of cpufreq frequency entries in tegra3_clocks.c allocated to LP
 * and not user-modifiable. */
#define CPU_LP_FREQ_ENTRIES 3

/* EternityProject Tegra 3 CPU Manager */
#define EPRJ_TEGRA3_G_CLUSTER 1
#define EPRJ_TEGRA3_LP_CLUSTER 2

void eprj_tegra_auto_hotplug_reinit(void);
void eprj_tegra_auto_hotplug_set_enabled_clusters(unsigned char clusters);

/*
 * EternityProject Scheduling Helper
 */
unsigned int eprj_get_nr_run_avg(void);		/* Number of average running threads */

/*
 * EternityProject LiveColor
 */
#ifdef CONFIG_ETERNITYPROJECT_LIVECOLOR
extern void eprj_livecolor_apply(bool);
#else
static inline void eprj_livecolor_apply(bool enb) { };
#endif

/*
 * EternityProject Tegra CPU Cache Management API
 */
void clean_and_invalidate_caches(unsigned long vstart,
	unsigned long length, unsigned long pstart);
void clean_caches(unsigned long vstart,
	unsigned long length, unsigned long pstart);
void invalidate_caches(unsigned long vstart,
	unsigned long length, unsigned long pstart);
