/*
 *  (C) 2004-2009  Dominik Brodowski <linux@dominikbrodowski.de>
 *
 *  Licensed under the terms of the GNU GPL License version 2.
 */


#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

#include <getopt.h>

#include "cpufreq.h"
#include "helpers/sysfs.h"
#include "helpers/helpers.h"
#include "helpers/bitmask.h"

#define LINE_LEN 10

static unsigned int count_cpus(void)
{
	FILE *fp;
	char value[LINE_LEN];
	unsigned int ret = 0;
	unsigned int cpunr = 0;

	fp = fopen("/proc/stat", "r");
	if (!fp) {
		printf(_("Couldn't count the number of CPUs (%s: %s), assuming 1\n"), "/proc/stat", strerror(errno));
		return 1;
	}

	while (!feof(fp)) {
		if (!fgets(value, LINE_LEN, fp))
			continue;
		value[LINE_LEN - 1] = '\0';
		if (strlen(value) < (LINE_LEN - 2))
			continue;
		if (strstr(value, "cpu "))
			continue;
		if (sscanf(value, "cpu%d ", &cpunr) != 1)
			continue;
		if (cpunr > ret)
			ret = cpunr;
	}
	fclose(fp);

	/* cpu count starts from 0, on error return 1 (UP) */
	return ret + 1;
}


static void proc_cpufreq_output(void)
{
	unsigned int cpu, nr_cpus;
	struct cpufreq_policy *policy;
	unsigned int min_pctg = 0;
	unsigned int max_pctg = 0;
	unsigned long min, max;

	printf(_("          minimum CPU frequency  -  maximum CPU frequency  -  governor\n"));

	nr_cpus = count_cpus();
	for (cpu = 0; cpu < nr_cpus; cpu++) {
		policy = cpufreq_get_policy(cpu);
		if (!policy)
			continue;

		if (cpufreq_get_hardware_limits(cpu, &min, &max)) {
			max = 0;
		} else {
			min_pctg = (policy->min * 100) / max;
			max_pctg = (policy->max * 100) / max;
		}
		printf("CPU%3d    %9lu kHz (%3d %%)  -  %9lu kHz (%3d %%)  -  %s\n",
			cpu , policy->min, max ? min_pctg : 0, policy->max,
			max ? max_pctg : 0, policy->governor);

		cpufreq_put_policy(policy);
	}
}

static int no_rounding;
static void print_speed(unsigned long speed)
{
	unsigned long tmp;

	if (no_rounding) {
		if (speed > 1000000)
			printf("%u.%06u GHz", ((unsigned int) speed/1000000),
				((unsigned int) speed%1000000));
		else if (speed > 100000)
			printf("%u MHz", (unsigned int) speed);
		else if (speed > 1000)
			printf("%u.%03u MHz", ((unsigned int) speed/1000),
				(unsigned int) (speed%1000));
		else
			printf("%lu kHz", speed);
	} else {
		if (speed > 1000000) {
			tmp = speed%10000;
			if (tmp >= 5000)
				speed += 10000;
			printf("%u.%02u GHz", ((unsigned int) speed/1000000),
				((unsigned int) (speed%1000000)/10000));
		} else if (speed > 100000) {
			tmp = speed%1000;
			if (tmp >= 500)
				speed += 1000;
			printf("%u MHz", ((unsigned int) speed/1000));
		} else if (speed > 1000) {
			tmp = speed%100;
			if (tmp >= 50)
				speed += 100;
			printf("%u.%01u MHz", ((unsigned int) speed/1000),
				((unsigned int) (speed%1000)/100));
		}
	}

	return;
}

static void print_duration(unsigned long duration)
{
	unsigned long tmp;

	if (no_rounding) {
		if (duration > 1000000)
			printf("%u.%06u ms", ((unsigned int) duration/1000000),
				((unsigned int) duration%1000000));
		else if (duration > 100000)
			printf("%u us", ((unsigned int) duration/1000));
		else if (duration > 1000)
			printf("%u.%03u us", ((unsigned int) duration/1000),
				((unsigned int) duration%1000));
		else
			printf("%lu ns", duration);
	} else {
		if (duration > 1000000) {
			tmp = duration%10000;
			if (tmp >= 5000)
				duration += 10000;
			printf("%u.%02u ms", ((unsigned int) duration/1000000),
				((unsigned int) (duration%1000000)/10000));
		} else if (duration > 100000) {
			tmp = duration%1000;
			if (tmp >= 500)
				duration += 1000;
			printf("%u us", ((unsigned int) duration / 1000));
		} else if (duration > 1000) {
			tmp = duration%100;
			if (tmp >= 50)
				duration += 100;
			printf("%u.%01u us", ((unsigned int) duration/1000),
				((unsigned int) (duration%1000)/100));
		} else
			printf("%lu ns", duration);
	}
	return;
}

/* --boost / -b */

static int get_boost_mode(unsigned int cpu)
{
	int support, active, b_states = 0, ret, pstate_no, i;
	/* ToDo: Make this more global */
	unsigned long pstates[MAX_HW_PSTATES] = {0,};

	if (cpupower_cpu_info.vendor != X86_VENDOR_AMD &&
	    cpupower_cpu_info.vendor != X86_VENDOR_INTEL)
		return 0;

	ret = cpufreq_has_boost_support(cpu, &support, &active, &b_states);
	if (ret) {
		printf(_("Error while evaluating Boost Capabilities"
				" on CPU %d -- are you root?\n"), cpu);
		return ret;
	}
	/* P state changes via MSR are identified via cpuid 80000007
	   on Intel and AMD, but we assume boost capable machines can do that
	   if (cpuid_eax(0x80000000) >= 0x80000007
	   && (cpuid_edx(0x80000007) & (1 << 7)))
	*/

	printf(_("  boost state support:\n"));

	printf(_("    Supported: %s\n"), support ? _("yes") : _("no"));
	printf(_("    Active: %s\n"), active ? _("yes") : _("no"));

	if (cpupower_cpu_info.vendor == X86_VENDOR_AMD &&
	    cpupower_cpu_info.family >= 0x10) {
		ret = decode_pstates(cpu, cpupower_cpu_info.family, b_states,
				     pstates, &pstate_no);
		if (ret)
			return ret;

		printf(_("    Boost States: %d\n"), b_states);
		printf(_("    Total 