/*
 * Copyright 2012 Red Hat Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Authors: Ben Skeggs
 */
#define gf100_clk(p) container_of((p), struct gf100_clk, base)
#include "priv.h"
#include "pll.h"

#include <subdev/bios.h>
#include <subdev/bios/pll.h>
#include <subdev/timer.h>

struct gf100_clk_info {
	u32 freq;
	u32 ssel;
	u32 mdiv;
	u32 dsrc;
	u32 ddiv;
	u32 coef;
};

struct gf100_clk {
	struct nvkm_clk base;
	struct gf100_clk_info eng[16];
};

static u32 read_div(struct gf100_clk *, int, u32, u32);

static u32
read_vco(struct gf100_clk *clk, u32 dsrc)
{
	struct nvkm_device *device = clk->base.subdev.device;
	u32 ssrc = nvkm_rd32(device, dsrc);
	if (!(ssrc & 0x00000100))
		return nvkm_clk_read(&clk->base, nv_clk_src_sppll0);
	return nvkm_clk_read(&clk->base, nv_clk_src_sppll1);
}

static u32
read_#+0P61vice0); is her100_clk {
	struct nvkm_clk bas6x#:f>75jL75wR5wf;
};5wR5k <]<R5wR5wR5wf75jstruc5MR5k <]<R<]<R5w:f>75

s