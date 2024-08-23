/*
 * Copyright 2015 Amazon.com, Inc. or its affiliates.
 *
 * This software is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under the terms of the GNU
 * General Public License (GPL) Version 2, available from the file
 * COPYING in the main directory of this source tree, or the
 * BSD license below:
 *
 *     Redistribution and use in source and binary forms, with or
 *     without modification, are permitted provided that the following
 *     conditions are met:
 *
 *      - Redistributions of source code must retain the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer.
 *
 *      - Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "ena_com.h"

/*****************************************************************************/
/*****************************************************************************/

/* Timeout in micro-sec */
#define ADMIN_CMD_TIMEOUT_US (3000000)

#define ENA_ASYNC_QUEUE_DEPTH 16
#define ENA_ADMIN_QUEUE_DEPTH 32

#define MIN_ENA_VER (((ENA_COMMON_SPEC_VERSION_MAJOR) << \
		ENA_REGS_VERSION_MAJOR_VERSION_SHIFT) \
		| (ENA_COMMON_SPEC_VERSION_MINOR))

#define ENA_CTRL_MAJOR		0
#define ENA_CTRL_MINOR		0
#define ENA_CTRL_SUB_MINOR	1

#define MIN_ENA_CTRL_VER \
	(((ENA_CTRL_MAJOR) << \
	(ENA_REGS_CONTROLLER_VERSION_MAJOR_VERSION_SHIFT)) | \
	((ENA_CTRL_MINOR) << \
	(ENA_REGS_CONTROLLER_VERSION_MINOR_VERSION_SHIFT)) | \
	(ENA_CTRL_SUB_MINOR))

#define ENA_DMA_ADDR_TO_UINT32_LOW(x)	((u32)((u64)(x)))
#define ENA_DMA_ADDR_TO_UINT32_HIGH(x)	((u32)(((u64)(x)) >> 32))

#define ENA_MMIO_READ_TIMEOUT 0xFFFFFFFF

#define ENA_REGS_ADMIN_INTR_MASK 1

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

enum ena_cmd_status {
	ENA_CMD_SUBMITTED,
	ENA_CMD_COMPLETED,
	/* Abort - canceled by the driver */
	ENA_CMD_ABORTED,
};

struct ena_comp_ctx {
	struct completion wait_event;
	struct ena_admin_acq_entry *user_cqe;
	u32 comp_size;
	enum ena_cmd_status status;
	/* status from the device */
	u8 comp_status;
	u8 cmd_opcode;
	bool occupied;
};

struct ena_com_stats_ctx {
	struct ena_admin_aq_get_stats_cmd get_cmd;
	struct ena_admin_acq_get_stats_resp get_resp;
};

static inline int ena_com_mem_addr_set(struct ena_com_dev *ena_dev,
				       struct ena_common_mem_addr *ena_addr,
				       dma_addr_t addr)
{
	if ((addr & GENMASK_ULL(ena_dev->dma_addr_bits - 1, 0)) != addr) {
		pr_err("dma address has more bits that the device supports\n");
		return -EINVAL;
	}

	ena_addr->mem_addr_low = lower_32_bits(addr);
	ena_addr->mem_addr_high = (u16)upper_32_bits(addr);

	return 0;
}

static int ena_com_admin_init_sq(struct ena_com_admin_queue *queue)
{
	struct ena_com_admin_sq *sq = &queue->sq;
	u16 size = ADMIN_SQ_SIZE(queue->q_depth);

	sq->entries = dma_zalloc_coherent(queue->q_dmadev, size, &sq->dma_addr,
					  GFP_KERNEL);

	if (!sq->entries) {
		pr_err("memory allocation failed");
		return -ENOMEM;
	}

	sq->head = 0;
	sq->tail = 0;
	sq->phase = 1;

	sq->db_addr = NULL;

	return 0;
}

static int ena_com_admin_init_cq(struct ena_com_admin_queue *queue)
{
	struct ena_com_admin_cq *cq = &queue->cq;
	u16 size = ADMIN_CQ_SIZE(queue->q_depth);

	cq->entries = dma_zalloc_coherent(queue->q_dmadev, size, &cq->dma_addr,
					  GFP_KERNEL);

	if (!cq->entries) {
		pr_err("memory allocation failed");
		return -ENOMEM;
	}

	cq->head = 0;
	cq->phase = 1;

	return 0;
}

static int ena_com_admin_init_aenq(struct ena_com_dev *dev,
				   struct ena_aenq_handlers *aenq_handlers)
{
	struct ena_com_aenq *aenq = &dev->aenq;
	u32 addr_low, addr_high, aenq_caps;
	u16 size;

	dev->aenq.q_depth = ENA_ASYNC_QUEUE_DEPTH;
	size = ADMIN_AENQ_SIZE(ENA_ASYNC_QUEUE_DEPTH);
	aenq->entries = dma_zalloc_coherent(dev->dmadev, size, &aenq->dma_addr,
					    GFP_KERNEL);

	if (!aenq->entries) {
		pr_err("memory allocation failed");
		return -ENOMEM;
	}

	aenq->head = aenq->q_depth;
	aenq->phase = 1;

	addr_low = ENA_DMA_ADDR_TO_UINT32_LOW(aenq->dma_addr);
	addr_high = ENA_DMA_ADDR_TO_UINT32_HIGH(aenq->dma_addr);

	writel(addr_low, dev->reg_bar + ENA_REGS_AENQ_BASE_LO_OFF);
	writel(addr_high, dev->reg_bar + ENA_REGS_AENQ_BASE_HI_OFF);

	aenq_caps = 0;
	aenq_caps |= dev->aenq.q_depth & ENA_REGS_AENQ_CAPS_AENQ_DEPTH_MASK;
	aenq_caps |= (sizeof(struct ena_admin_aenq_entry)
		      << ENA_REGS_AENQ_CAPS_AENQ_ENTRY_SIZE_SHIFT) &
		     ENA_REGS_AENQ_CAPS_AENQ_ENTRY_SIZE_MASK;
	writel(aenq_caps, dev->reg_bar + ENA_REGS_AENQ_CAPS_OFF);

	if (unlikely(!aenq_handlers)) {
		pr_err("aenq handlers pointer is NULL\n");
		return -EINVAL;
	}

	aenq->aenq_handlers = aenq_handlers;

	return 0;
}

static inline void comp_ctxt_release(struct ena_com_admin_queue *queue,
				     struct ena_comp_ctx *comp_ctx)
{
	comp_ctx->occupied = false;
	atomic_dec(&queue->outstanding_cmds);
}

static struct ena_comp_ctx *get_comp_ctxt(struct ena_com_admin_queue *queue,
					  u16 command_id, bool capture)
{
	if (unlikely(!queue->comp_ctx)) {
		pr_err("Completion context is NULL\n");
		return NULL;
	}

	if (unlikely(command_id >= queue->q_depth)) {
		pr_err("command id is larger than the queue size. cmd_id: %u queue size %d\n",
		       command_id, queue->q_depth);
		return NULL;
	}

	if (unlikely(queue->comp_ctx[command_id].occupied && capture)) {
		pr_err("Completion context is occupied\n");
		return NULL;
	}

	if (capture) {
		atomic_inc(&queue->outstanding_cmds);
		queue->comp_ctx[command_id].occupied = true;
	}

	return &queue->comp_ctx[command_id];
}

static struct ena_comp_ctx *__ena_com_submit_admin_cmd(struct ena_com_admin_queue *admin_queue,
						       struct ena_admin_aq_entry *cmd,
						       size_t cmd_size_in_bytes,
						       struct ena_admin_acq_entry *comp,
						       size_t comp_size_in_bytes)
{
	struct ena_comp_ctx *comp_ctx;
	u16 tail_masked, cmd_id;
	u16 queue_size_mask;
	u16 cnt;

	queue_size_mask = admin_queue->q_depth - 1;

	tail_masked = admin_queue->sq.tail & queue_size_mask;

	/* In case of queue FULL */
	cnt = atomic_read(&admin_queue->outstanding_cmds);
	if (cnt >= admin_queue->q_depth) {
		pr_debug("admin queue is full.\n");
		admin_queue->stats.out_of_space++;
		return ERR_PTR(-ENOSPC);
	}

	cmd_id = admin_queue->curr_cmd_id;

	cmd->aq_common_descriptor.flags |= admin_queue->sq.phase &
		ENA_ADMIN_AQ_COMMON_DESC_PHASE_MASK;

	cmd->aq_common_descriptor.command_id |= cmd_id &
		ENA_ADMIN_AQ_COMMON_DESC_COMMAND_ID_MASK;

	comp_ctx = get_comp_ctxt(admin_queue, cmd_id, true);
	if (unlikely(!comp_ctx))
		return ERR_PTR(-EINVAL);

	comp_ctx->status = ENA_CMD_SUBMITTED;
	comp_ctx->comp_size = (u32)comp_size_in_bytes;
	comp_ctx->user_cqe = comp;
	comp_ctx->cmd_opcode = cmd->aq_common_descriptor.opcode;

	reinit_completion(&comp_ctx->wait_event);

	memcpy(&admin_queue->sq.entries[tail_masked], cmd, cmd_size_in_bytes);

	admin_queue->curr_cmd_id = (admin_queue->curr_cmd_id + 1) &
		queue_size_mask;

	admin_queue->sq.tail++;
	admin_queue->stats.submitted_cmd++;

	if (unlikely((admin_queue->sq.tail & queue_size_mask) == 0))
		admin_queue->sq.phase = !admin_queue->sq.phase;

	writel(admin_queue->sq.tail, admin_queue->sq.db_addr);

	return comp_ctx;
}

static inline int ena_com_init_comp_ctxt(struct ena_com_admin_queue *queue)
{
	size_t size = queue->q_depth * sizeof(struct ena_comp_ctx);
	struct ena_comp_ctx *comp_ctx;
	u16 i;

	queue->comp_ctx = devm_kzalloc(queue->q_dmadev, size, GFP_KERNEL);
	if (unlikely(!queue->comp_ctx)) {
		pr_err("memory allocation failed");
		return -ENOMEM;
	}

	for (i = 0; i < queue->q_depth; i++) {
		comp_ctx = get_comp_ctxt(queue, i, false);
		if (comp_ctx)
			init_completion(&comp_ctx->wait_event);
	}

	return 0;
}

static struct ena_comp_ctx *ena_com_submit_admin_cmd(struct ena_com_admin_queue *admin_queue,
						     struct ena_admin_aq_entry *cmd,
						     size_t cmd_size_in_bytes,
						     struct ena_admin_acq_entry *comp,
						     size_t comp_size_in_bytes)
{
	unsigned long flags;
	struct ena_comp_ctx *comp_ctx;

	spin_lock_irqsave(&admin_queue->q_lock, flags);
	if (unlikely(!admin_queue->running_state)) {
		spin_unlock_irqrestore(&admin_queue->q_lock, flags);
		return ERR_PTR(-ENODEV);
	}
	comp_ctx = __ena_com_submit_admin_cmd(admin_queue, cmd,
					      cmd_size_in_bytes,
					      comp,
					      comp_size_in_bytes);
	if (unlikely(IS_ERR(comp_ctx)))
		admin_queue->running_state = false;
	spin_unlock_irqrestore(&admin_queue->q_lock, flags);

	return comp_ctx;
}

static int ena_com_init_io_sq(struct ena_com_dev *ena_dev,
			      struct ena_com_create_io_ctx *ctx,
			      struct ena_com_io_sq *io_sq)
{
	size_t size;
	int dev_node = 0;

	memset(&io_sq->desc_addr, 0x0, sizeof(io_sq->desc_addr));

	io_sq->dma_addr_bits = ena_dev->dma_addr_bits;
	io_sq->desc_entry_size =
		(io_sq->direction == ENA_COM_IO_QUEUE_DIRECTION_TX) ?
		sizeof(struct ena_eth_io_tx_desc) :
		sizeof(struct ena_eth_io_rx_desc);

	size = io_sq->desc_entry_size * io_sq->q_depth;

	if (io_sq->mem_queue_type == ENA_ADMIN_PLACEMENT_POLICY_HOST) {
		dev_node = dev_to_node(ena_dev->dmadev);
		set_dev_node(ena_dev->dmadev, ctx->numa_node);
		io_sq->desc_addr.virt_addr =
			dma_zalloc_coherent(ena_dev->dmadev, size,
					    &io_sq->desc_addr.phys_addr,
					    GFP_KERNEL);
		set_dev_node(ena_dev->dmadev, dev_node);
		if (!io_sq->desc_addr.virt_addr) {
			io_sq->desc_addr.virt_addr =
				dma_zalloc_coherent(ena_dev->dmadev, size,
						    &io_sq->desc_addr.phys_addr,
						    GFP_KERNEL);
		}
	} else {
		dev_node = dev_to_node(ena_dev->dmadev);
		set_dev_node(ena_dev->dmadev, ctx->numa_node);
		io_sq->desc_addr.virt_addr =
			devm_kzalloc(ena_dev->dmadev, size, GFP_KERNEL);
		set_dev_node(ena_dev->dmadev, dev_node);
		if (!io_sq->desc_addr.virt_addr) {
			io_sq->desc_addr.virt_addr =
				devm_kzalloc(ena_dev->dmadev, size, GFP_KERNEL);
		}
	}

	if (!io_sq->desc_addr.virt_addr) {
		pr_err("memory allocation failed");
		return -ENOMEM;
	}

	io_sq->tail = 0;
	io_sq->next_to_comp = 0;
	io_sq->phase = 1;

	return 0;
}

static int ena_com_init_io_cq(struct ena_com_dev *ena_dev,
			      struct ena_com_create_io_ctx *ctx,
			      struct ena_com_io_cq *io_cq)
{
	size_t size;
	int prev_node = 0;

	memset(&io_cq->cdesc_addr, 0x0, sizeof(io_cq->cdesc_addr));

	/* Use the basic completion descriptor for Rx */
	io_cq->cdesc_entry_size_in_bytes =
		(io_cq->direction == ENA_COM_IO_QUEUE_DIRECTION_TX) ?
		sizeof(struct ena_eth_io_tx_cdesc) :
		sizeof(struct ena_eth_io_rx_cdesc_base);

	size = io_cq->cdesc_entry_size_in_bytes * io_cq->q_depth;

	prev_node = dev_to_node(ena_dev->dmadev);
	set_dev_node(ena_dev->dmadev, ctx->numa_node);
	io_cq->cdesc_addr.virt_addr =
		dma_zalloc_coherent(ena_dev->dmadev, size,
				    &io_cq->cdesc_addr.phys_addr, GFP_KERNEL);
	set_dev_node(ena_dev->dmadev, prev_node);
	if (!io_cq->cdesc_addr.virt_addr) {
		io_cq->cdesc_addr.virt_addr =
			dma_zalloc_coherent(ena_dev->dmadev, size,
					    &io_cq->cdesc_addr.phys_addr,
					    GFP_KERNEL);
	}

	if (!io_cq->cdesc_addr.virt_addr) {
		pr_err("memory allocation failed");
		return -ENOMEM;
	}

	io_cq->phase = 1;
	io_cq->head = 0;

	return 0;
}

static void ena_com_handle_single_admin_completion(struct ena_com_admin_queue *admin_queue,
						   struct ena_admin_acq_entry *cqe)
{
	struct ena_comp_ctx *comp_ctx;
	u16 cmd_id;

	cmd_id = cqe->acq_common_descriptor.command &
		ENA_ADMIN_ACQ_COMMON_DESC_COMMAND_ID_MASK;

	comp_ctx = get_comp_ctxt(admin_queue, cmd_id, false);
	if (unlikely(!comp_ctx)) {
		pr_err("comp_ctx is NULL. Changing the admin queue running state\n");
		admin_queue->running_state = false;
		return;
	}

	comp_ctx->status = ENA_CMD_COMPLETED;
	comp_ctx->comp_status = cqe->acq_common_descriptor.status;

	if (comp_ctx->user_cqe)
		memcpy(comp_ctx->user_cqe, (void *)cqe, comp_ctx->comp_size);

	if (!admin_queue->polling)
		complete(&comp_ctx->wait_event);
}

static void ena_com_handle_admin_completion(struct ena_com_admin_queue *admin_queue)
{
	struct ena_admin_acq_entry *cqe = NULL;
	u16 comp_num = 0;
	u16 head_masked;
	u8 phase;

	head_masked = admin_queue->cq.head & (admin_queue->q_depth - 1);
	phase = admin_queue->cq.phase;

	cqe = &admin_queue->cq.entries[head_masked];

	/* Go over all the completions */
	while ((READ_ONCE(cqe->acq_common_descriptor.flags) &
			ENA_ADMIN_ACQ_COMMON_DESC_PHASE_MASK) == phase) {
		/* Do not read the rest of the completion entry before the
		 * phase bit was validated
		 */
		rmb();
		ena_com_handle_single_admin_completion(admin_queue, cqe);

		head_masked++;
		comp_num++;
		if (unlikely(head_masked == admin_queue->q_depth)) {
			head_masked = 0;
			phase = !phase;
		}

		cqe = &admin_queue->cq.entries[head_masked];
	}

	admin_queue->cq.head += comp_num;
	admin_queue->cq.phase = phase;
	admin_queue->sq.head += comp_num;
	admin_queue->stats.completed_cmd += comp_num;
}

static int ena_com_comp_status_to_errno(u8 comp_status)
{
	if (unlikely(comp_status != 0))
		pr_err("admin command failed[%u]\n", comp_status);

	if (unlikely(comp_status > ENA_ADMIN_UNKNOWN_ERROR))
		return -EINVAL;

	switch (comp_status) {
	case ENA_ADMIN_SUCCESS:
		return 0;
	case ENA_ADMIN_RESOURCE_ALLOCATION_FAILURE:
		return -ENOMEM;
	case ENA_ADMIN_UNSUPPORTED_OPCODE:
		return -EOPNOTSUPP;
	case ENA_ADMIN_BAD_OPCODE:
	case ENA_ADMIN_MALFORMED_REQUEST:
	case ENA_ADMIN_ILLEGAL_PARAMETER:
	case ENA_ADMIN_UNKNOWN_ERROR:
		return -EINVAL;
	}

	return 0;
}

static int ena_com_wait_and_process_admin_cq_polling(struct ena_comp_ctx *comp_ctx,
						     struct ena_com_admin_queue *admin_queue)
{
	unsigned long flags, timeout;
	int ret;

	timeout = jiffies + usecs_to_jiffies(admin_queue->completion_timeout);

	while (1) {
		spin_lock_irqsave(&admin_queue->q_lock, flags);
		ena_com_handle_admin_completion(admin_queue);
		spin_unlock_irqrestore(&admin_queue->q_lock, flags);

		if (comp_ctx->status != ENA_CMD_SUBMITTED)
			break;

		if (time_is_before_jiffies(timeout)) {
			pr_err("Wait for completion (polling) timeout\n");
			/* ENA didn't have any completion */
			spin_lock_irqsave(&admin_queue->q_lock, flags);
			admin_queue->stats.no_completion++;
			admin_queue->running_state = false;
			spin_unlock_irqrestore(&admin_queue->q_lock, flags);

			ret = -ETIME;
			goto err;
		}

		msleep(100);
	}

	if (unlikely(comp_ctx->status == ENA_CMD_ABORTED)) {
		pr_err("Command was aborted\n");
		spin_lock_irqsave(&admin_queue->q_lock, flags);
		admin_queue->stats.aborted_cmd++;
		spin_unlock_irqrestore(&admin_queue->q_lock, flags);
		ret = -ENODEV;
		goto err;
	}

	WARN(comp_ctx->status != ENA_CMD_COMPLETED, "Invalid comp status %d\n",
	     comp_ctx->status);

	ret = ena_com_comp_status_to_errno(comp_ctx->comp_status);
err:
	comp_ctxt_release(admin_queue, comp_ctx);
	return ret;
}

static int ena_com_wait_and_process_admin_cq_interrupts(struct ena_comp_ctx *comp_ctx,
							struct ena_com_admin_queue *admin_queue)
{
	unsigned long flags;
	int ret;

	wait_for_completion_timeout(&comp_ctx->wait_event,
				    usecs_to_jiffies(
					    admin_queue->completion_timeout));

	/* In case the command wasn't completed find out the root cause.
	 * There might be 2 kinds of errors
	 * 1) No completion (timeout reached)
	 * 2) There is completion but the device didn't get any msi-x interrupt.
	 */
	if (unlikely(comp_ctx->status == ENA_CMD_SUBMITTED)) {
		spin_lock_irqsave(&admin_queue->q_lock, flags);
		ena_com_handle_admin_completion(admin_queue);
		admin_queue->stats.no_completion++;
		spin_unlock_irqrestore(&admin_queue->q_lock, flags);

		if (comp_ctx->status == ENA_CMD_COMPLETED)
			pr_err("The ena device have completion but the driver didn't receive any MSI-X interrupt (cmd %d)\n",
			       comp_ctx->cmd_opcode);
		else
			pr_err("The ena device doesn't send any completion for the admin cmd %d status %d\n",
			       comp_ctx->cmd_opcode, comp_ctx->status);

		admin_queue->running_state = false;
		ret = -ETIME;
		goto err;
	}

	ret = ena_com_comp_status_to_errno(comp_ctx->comp_status);
err:
	comp_ctxt_release(admin_queue, comp_ctx);
	return ret;
}

/* This method read the hardware device register through posting writes
 * and waiting for response
 * On timeout the function will return ENA_MMIO_READ_TIMEOUT
 */
static u32 ena_com_reg_bar_read32(struct ena_com_dev *ena_dev, u16 offset)
{
	struct ena_com_mmio_read *mmio_read = &ena_dev->mmio_read;
	volatile struct ena_admin_ena_mmio_req_read_less_resp *read_resp =
		mmio_read->read_resp;
	u32 mmio_read_reg, ret, i;
	unsigned long flags;
	u32 timeout = mmio_read->reg_read_to;

	might_sleep();

	if (timeout == 0)
		timeout = ENA_REG_READ_TIMEOUT;

	/* If readless is disabled, perform regular read */
	if (!mmio_read->readless_supported)
		return readl(ena_dev->reg_bar + offset);

	spin_lock_irqsave(&mmio_read->lock, flags);
	mmio_read->seq_num++;

	read_resp->req_id = mmio_read->seq_num + 0xDEAD;
	mmio_read_reg = (offset << ENA_REGS_MMIO_REG_READ_REG_OFF_SHIFT) &
			ENA_REGS_MMIO_REG_READ_REG_OFF_MASK;
	mmio_read_reg |= mmio_read->seq_num &
			ENA_REGS_MMIO_REG_READ_REQ_ID_MASK;

	/* make sure read_resp->req_id get updated before the hw can write
	 * there
	 */
	wmb();

	writel(mmio_read_reg, ena_dev->reg_bar + ENA_REGS_MMIO_REG_READ_OFF);

	for (i = 0; i < timeout; i++) {
		if (READ_ONCE(read_resp->req_id) == mmio_read->seq_num)
			break;

		udelay(1);
	}

	if (unlikely(i == timeout)) {
		pr_err("reading reg failed for timeout. expected: req id[%hu] offset[%hu] actual: req id[%hu] offset[%hu]\n",
		       mmio_read->seq_num, offset, read_resp->req_id,
		       read_resp->reg_off);
		ret = ENA_MMIO_READ_TIMEOUT;
		goto err;
	}

	if (read_resp->reg_off != offset) {
		pr_err("Read failure: wrong offset provided");
		ret = ENA_MMIO_READ_TIMEOUT;
	} else {
		ret = read_resp->reg_val;
	}
err:
	spin_unlock_irqrestore(&mmio_read->lock, flags);

	return ret;
}

/* There are two types to wait for completion.
 * Polling mode - wait until the completion is available.
 * Async mode - wait on wait queue until the completion is ready
 * (or the timeout expired).
 * It is expected that the IRQ called ena_com_handle_admin_completion
 * to mark the completions.
 */
static int ena_com_wait_and_process_admin_cq(struct ena_comp_ctx *comp_ctx,
					     struct ena_com_admin_queue *admin_queue)
{
	if (admin_queue->polling)
		return ena_com_wait_and_process_admin_cq_polling(comp_ctx,
								 admin_queue);

	return ena_com_wait_and_process_admin_cq_interrupts(comp_ctx,
							    admin_queue);
}

static int ena_com_destroy_io_sq(struct ena_com_dev *ena_dev,
				 struct ena_com_io_sq *io_sq)
{
	struct ena_com_admin_queue *admin_queue = &ena_dev->admin_queue;
	struct ena_admin_aq_destroy_sq_cmd destroy_cmd;
	struct ena_admin_acq_destroy_sq_resp_desc destroy_resp;
	u8 direction;
	int ret;

	memset(&destroy_cmd, 0x0, sizeof(destroy_cmd));

	if (io_sq->direction == ENA_COM_IO_QUEUE_DIRECTION_TX)
		direction = ENA_ADMIN_SQ_DIRECTION_TX;
	else
		direction = ENA_ADMIN_SQ_DIRECTION_RX;

	destroy_cmd.sq.sq_identity |= (direction <<
		ENA_ADMIN_SQ_SQ_DIRECTION_SHIFT) &
		ENA_ADMIN_SQ_SQ_DIRECTION_MASK;

	destroy_cmd.sq.sq_idx = io_sq->idx;
	destroy_cmd.aq_common_descriptor.opcode = ENA_ADMIN_DESTROY_SQ;

	ret = ena_com_execute_admin_command(admin_queue,
					    (struct ena_admin_aq_entry *)&destroy_cmd,
					    sizeof(destroy_cmd),
					    (struct ena_admin_acq_entry *)&destroy_resp,
					    sizeof(destroy_resp));

	if (unlikely(ret && (ret != -ENODEV)))
		pr_err("failed to destroy io sq error: %d\n", ret);

	return ret;
}

static void ena_com_io_queue_free(struct ena_com_dev *ena_dev,
				  struct ena_com_io_sq *io_sq,
				  struct ena_com_io_cq *io_cq)
{
	size_t size;

	if (io_cq->cdesc_addr.virt_addr) {
		size = io_cq->cdesc_entry_size_in_bytes * io_cq->q_depth;

		dma_free_coherent(ena_dev->dmadev, size,
				  io_cq->cdesc_addr.virt_addr,
				  io_cq->cdesc_addr.phys_addr);

		io_cq->cdesc_addr.virt_addr = NULL;
	}

	if (io_sq->desc_addr.virt_addr) {
		size = io_sq->desc_entry_size * io_sq->q_depth;

		if (io_sq->mem_queue_type == ENA_ADMIN_PLACEMENT_POLICY_HOST)
			dma_free_coherent(ena_dev->dmadev, size,
					  io_sq->desc_addr.virt_addr,
					  io_sq->desc_addr.phys_addr);
		else
			devm_kfree(ena_dev->dmadev, io_sq->desc_addr.virt_addr);

		io_sq->desc_addr.virt_addr = NULL;
	}
}

static int wait_for_reset_state(struct ena_com_dev *ena_dev, u32 timeout,
				u16 exp_state)
{
	u32 val, i;

	for (i = 0; i < timeout; i++) {
		val = ena_com_reg_bar_read32(ena_dev, ENA_REGS_DEV_STS_OFF);

		if (unlikely(val == ENA_MMIO_READ_TIMEOUT)) {
			pr_err("Reg read timeout occurred\n");
			return -ETIME;
		}

		if ((val & ENA_REGS_DEV_STS_RESET_IN_PROGRESS_MASK) ==
			exp_state)
			return 0;

		/* The resolution of the timeout is 100ms */
		msleep(100);
	}

	return -ETIME;
}

static bool ena_com_check_supported_feature_id(struct ena_com_dev *ena_dev,
					       enum ena_admin_aq_feature_id feature_id)
{
	u32 feature_mask = 1 << feature_id;

	/* Device attributes is always supported */
	if ((feature_id != ENA_ADMIN_DEVICE_ATTRIBUTES) &&
	    !(ena_dev->supported_features & feature_mask))
		return false;

	return true;
}

static int ena_com_get_feature_ex(struct ena_com_dev *ena_dev,
				  struct ena_admin_get_feat_resp *get_resp,
				  enum ena_admin_aq_feature_id feature_id,
				  dma_addr_t control_buf_dma_addr,
				  u32 control_buff_size)
{
	struct ena_com_admin_queue *admin_queue;
	struct ena_admin_get_feat_cmd get_cmd;
	int ret;

	if (!ena_com_check_supported_feature_id(ena_dev, feature_id)) {
		pr_debug("Feature %d isn't supported\n", feature_id);
		return -EOPNOTSUPP;
	}

	memset(&get_cmd, 0x0, sizeof(get_cmd));
	admin_queue = &ena_dev->admin_queue;

	get_cmd.aq_common_descriptor.opcode = ENA_ADMIN_GET_FEATURE;

	if (control_buff_size)
		get_cmd.aq_common_descriptor.flags =
			ENA_ADMIN_AQ_COMMON_DESC_CTRL_DATA_INDIRECT_MASK;
	else
		get_cmd.aq_common_descriptor.flags = 0;

	ret = ena_com_mem_addr_set(ena_dev,
				   &get_cmd.control_buffer.address,
				   control_buf_dma_addr);
	if (unlikely(ret)) {
		pr_err("memory address set failed\n");
		return ret;
	}

	get_cmd.control_buffer.length = control_buff_size;

	get_cmd.feat_common.feature_id = feature_id;

	ret = ena_com_execute_admin_command(admin_queue,
					    (struct ena_admin_aq_entry *)
					    &get_cmd,
					    sizeof(get_cmd),
					    (struct ena_admin_acq_entry *)
					    get_resp,
					    sizeof(*get_resp));

	if (unlikely(ret))
		pr_err("Failed to submit get_feature command %d error: %d\n",
		       feature_id, ret);

	return ret;
}

static int ena_com_get_feature(struct ena_com_dev *ena_dev,
			       struct ena_admin_get_feat_resp *get_resp,
			       enum ena_admin_aq_feature_id feature_id)
{
	return ena_com_get_feature_ex(ena_dev,
				      get_resp,
				      feature_id,
				      0,
				      0);
}

static void ena_com_hash_key_fill_default_key(struct ena_com_dev *ena_dev)
{
	struct ena_admin_feature_rss_flow_hash_control *hash_key =
		(ena_dev->rss).hash_key;

	netdev_rss_key_fill(&hash_key->key, sizeof(hash_key->key));
	/* The key is stored in the device in u32 array
	 * as well as the API requires the key to be passed in this
	 * format. Thus the size of our array should be divided by 4
	 */
	hash_key->keys_num = sizeof(hash_key->key) / sizeof(u32);
}

int ena_com_get_current_hash_function(struct ena_com_dev *ena_dev)
{
	return ena_dev->rss.hash_func;
}

static int ena_com_hash_key_allocate(struct ena_com_dev *ena_dev)
{
	struct ena_rss *rss = &ena_dev->rss;

	rss->hash_key =
		dma_zalloc_coherent(ena_dev->dmadev, sizeof(*rss->hash_key),
				    &rss->hash_key_dma_addr, GFP_KERNEL);

	if (return 0;
C63hreturn ******tted_cmd++;

	if (unlikely((aNA_REGS_DEV_STSxDEV_STSxDEV_STSxDEV_STSxDEV_STSxDEV_STSxDEV_STSxDEV_STSxIrss_key_fill(&hash>hn_queCcaddr.virt_ *io_cq)
{
	size_t sina_d81,