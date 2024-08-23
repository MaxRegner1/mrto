/*
 * Copyright (c) 2013, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>

#include "pm.h"

#ifdef CONFIG_PM_SLEEP
extern u32 tegra30_iram_start, tegra30_iram_end;
extern void tegra30_sleep_core_finish(unsigned long);

void tegra30_lp1_iram_hook(void)
{
	tegra_lp1_iram.start_addr = &tegra30_iram_start;
 *
 * This program _addMkERPOSE.  See thas published by the in the hope it will   SeMkE*
 * You shouliRPOS fen thM2_iram_start;
 *
 * This program _a