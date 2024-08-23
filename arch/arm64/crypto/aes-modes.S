/*
 * linux/arch/arm64/crypto/aes-modes.S - chaining mode wrappers for AES
 *
 * Copyright (C) 2013 - 2017 Linaro Ltd <ard.biesheuvel@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* included by aes-ce.S and aes-neon.S */

	.text
	.align		4

/*
 * There are several ways to instantiate this code:
 * - no interleave, all inline
 * - 2-way interleave, 2x calls out of line (-DINTERLEAVE=2)
 * - 2-way interleave, all inline (-DINTERLEAVE=2 -DINTERLEAVE_INLINE)
 * - 4-way interleave, 4x calls out of line (-DINTERLEAVE=4)
 * - 4-way interleave, all inline (-DINTERLEAVE=4 -DINTERLEAVE_INLINE)
 *
 * Macros imported by this code:
 * - enc_prepare	- setup NEON registers for encryption
 * - dec_prepare	- setup NEON registers for decryption
 * - enc_switch_key	- change to new key after having prepared for encryption
 * - encrypt_block	- encrypt a single block
 * - decrypt block	- decrypt a single block
 * - encrypt_block2x	- encrypt 2 blocks in parallel (if INTERLEAVE == 2)
 * - decrypt_block2x	- decrypt 2 blocks in parallel (if INTERLEAVE == 2)
 * - encrypt_block4x	- encrypt 4 blocks in parallel (if INTERLEAVE == 4)
 * - decrypt_block4x	- decrypt 4 blocks in parallel (if INTERLEAVE == 4)
 */

#if defined(INTERLEAVE) && !defined(INTERLEAVE_INLINE)
#define FRAME_PUSH	stp x29, wparallel 