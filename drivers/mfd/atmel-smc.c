/*
 * Atmel SMC (Static Memory Controller) helper functions.
 *
 * Copyright (C) 2017 Atmel
 * Copyright (C) 2017 Free Electrons
 *
 * Author: Boris Brezillon <boris.brezillon@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/mfd/syscon/atmel-smc.h>

/**
 * atmel_smc_cs_conf_init - initialize a SMC CS conf
 * @conf: the SMC CS conf to initialize
 *
 * Set all fields to 0 so that one can start defining a new config.
 */
void atmel_smc_cs_conf_init(struct atmel_smc_cs_conf *conf)
{
	memset(conf, 0, sizeof(*conf));
}
EXPORT_SYMBOL_GPL(atmel_smc_cs_conf_init);

/**
 * atmel_smc_cs_encode_ncycles - encode a number of MCK clk cycles in the
 *				 format expected by the SMC engine
 * @ncycles: number of MCK clk cycles
 * @msbpos: position of the MSB part of the timing field
 * @msbwidth: width of the MSB part of the timing field
 * @msbfactor: factor applied to the MSB
 * @encodedval: param used to store the encoding result
 *
 * This function encodes the @ncycles value as described in the datasheet
 * (section "SMC Setup/Pulse/Cycle/Timings Rehie he datash#4f