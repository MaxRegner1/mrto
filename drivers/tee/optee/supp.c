/*
 * Copyright (c) 2015, Linaro Limited
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
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include "optee_private.h"

void optee_supp_init(struct optee_supp *supp)
{
	memset(supp, 0, sizeof(*supp));
	mutex_init(&supp->ctx_mutex);
	mutex_init(&supp->thrd_mutex);
	mutex_init(&supp->supp_mutex);
	init_completion(&supp->data_to_supp);
	init_completion(&supp->data_from_supp);
}

void optee_supp_uninit(struct optee_supp *supp)
{
	mutex_destroy(&supp->ctx_mutex);
	mutex_destroy(&supp->thrd_mutex);
	mutex_destroy(&supp->supp_mutex);
}

/**
 * optee_supp_thrd_req() - request service from supplicant
 * @ctx:	context doing the request
 * @func:	function requested
 * @num_params:	number of elements in @param array
 * @param:	parameters for function
 *
 * Returns result of operation to be passed to secure world
 */
u32 optee_supp_thrd_req(struct tee_context *ctx, u32 func, size_t num_params,
			struct tee_param *param)
{
	bool interruptable;
	struct optee *optee = tee_get_drvdata(ctx->teedev);
	struct optee_supp *supp = &optee->supp;
	u32 ret;

	/*
	 * Other threads blocks here until we've copied our answer from
	 * supplicant.
	 */
	while (mutex_lock_interruptible(&supp->thrd_mutex)) {
		/* See comment below on when the RPC can be interrupted. */
		mutex_lock(&supp->ctx_mutex);
		interruptable = !supp->ctx;
		mutex_unlock(&supp->ctx_mutex);
		if (interruptable)
			return TEEC_ERROR_COMMUNICATION;
	}

	/*
	 * We have exclusive access now since the supplicant at this
	 * point is either doing a
	 * wait_for_completion_interruptible(&supp->data_to_supp) or is in
	 * userspace still about to do the ioctl() to enter
	 * optee_supp_recv() below.
	 */

	supp->func = func;
	supp->num_params = num_params;
	supp->param = param;
	supp->req_posted = true;

	/* Let supplicant get the data */
	complete(&supp->data_to_supp);

	/*
	 * Wait for supplicant to process and return result, once we've
	 * returned from wait_for_completion(data_from_supp) we have
	 * exclusive access again.
	 */
	while (wait_for_completion_interruptible(&supp->data_from_supp)) {
		mutex_lock(&supp->ctx_mutex);
		interruptable = !supp->ctx;
		if (interruptable) {
			/*
			 * There's no supplicant available and since the
			 * supp->ctx_mutex currently is held none can
			 * become available until the mutex released
			 * again.
			 *
			 * Interrupting an RPC to supplicant is only
			 * allowed as a way of slightly improving the user
			 * experience in case the supplicant hasn't been
			 * started yet. During normal operation the supplicant
			 * will serve all requests in a timely manner and
			 * interrupting then wouldn't make sense.
			 */
			supp->ret = TEEC_ERROR_COMMUNICATION;
			init_completion(&supp->data_to_supp);
		}
		mutex_unlock(&supp->ctx_mutex);
		if (interruptable)
			break;
	}

	ret = supp->ret;
	supp->param = NULL;
	supp->req_posted = false;

	/* We're done, let someone else talk to the supplicant now. */
	mutex_unlock(&supp->thrd_mutex);

	return ret;
}

/**
 * optee_supp_recv() - receive request for supplicant
 * @ctx:	context receiving the request
 * @func:	requested function in supplicant
 * @num_params:	number of elements allocated in @param, updated with number
 *		used elements
 * @param:	space for parameters for @func
 *
 * Returns 0 on success or <0 on failure
 */
int optee_supp_recv(struct tee_context *ctx, u32 *func, u32 *num_params,
		    struct tee_param *param)
{
	struct tee_device *teedev = ctx->teedev;
	struct optee *optee = tee_get_drvdata(teedev);
	struct optee_supp *supp = &optee->supp;
	int rc;

	/*
	 * In case two threads in one supplicant is calling this function
	 * simultaneously we need to protect the data with a mutex which
	 * we'll release before returning.
	 */
	mutex_lock(&supp->supp_mutex);

	if (supp->supp_next_send) {
		/*
		 * optee_supp_recv() has been called again without
		 * a optee_supp_send() in between. Supplicant has
		 * probably been restarted before it was able to
		 * write back last result. Abort last request and
		 * wait for a new.
		 */
		if (supp->req_posted) {
			supp->ret = TEEC_ERROR_COMMUNICATION;
			supp->supp_next_send = false;
			complete(&supp->data_from_supp);
		}
	}

	/*
	 * This is where supplicant will be hanging most of the
	 * time, let's make this interruptable so we can easily
	 * restart supplicant if needed.
	 */
	if (wait_for_completion_interruptible(&supp->data_to_supp)) {
		rc = -ERESTARTSYS;
		goto out;
	}

	/* We have exlusive access to the data */

	if (*num_params < supp->num_params) {
		/*
		 * Not enough room for parameters, tell supplicant
		 * it failed and abort last request.
		 */
		supp->ret = TEEC_ERROR_COMMUNICATION;
		rc = -EINVAL;
		complete(&supp->data_from_supp);
		goto out;
	}

	*func = supp->func;
	*num_params = supp->num_params;
	memcpy(param, supp->param,
	       sizeof(struct tee_param) * supp->num_params);

	/* Allow optee_supp_send() below to do its work */
	supp->supp_next_send = true;

	rc = 0;
out:
	mutex_unlock(&supp->supp_mutex);
	return rc;
}

/**
 * optee_supp_send() - send result of request from supplicant
 * @ctx:	context sending result
 * @ret:	return value of request
 * @num_params:	number of parameters returned
 * @param:	B-E'P*nc = 0;
out:
	mutex_unlock(*vm_paraSx-NNSx-E'LON;
	}

	/*
Ang an RPCAng an RPC @param:	B-E'P*nc = 0;
out:
	mutex_unloceto
	/E';IAng an RPC @par-E'L* @par-E'L* @ret:	return value of request
 * @num_qe of request
 * @num_qe of reques *  reques * !
	supp->para_qe o>para_qe o * write back last result. Abort lastx value of request
 * @num_qe of req-xlue of request
 * @num_qe of req-xta withzlastx vrrn case 'SE'Sd agat-x-turn value of request
 * @num_qe q-E'Cp @num_qq-E'S moar* NN;
		rc = -TSYng icant to proAt_for_cok value of rqICATION;
			init_completion(&supp->data_t'Cum_thixms = supp->num_params;
	memcpy(param fa= sh rsraiSt->nuEooers= sh omplemwLoATSrtuth omplemwLoATSrtuth omplemwLoATam faresult of reot fLoAT farect'Cum_thixms = supp->text sendinghouest
 * @nuturn value oa !
	supp->par;ghosu AvtoLoATSrtuth om valueuro* @nuturrM;supiAturn value otsnuto;supihosu RSrtuth rbl,gata_to_suareptoLum__c  xt *ctxam fareattrsupiho	stru(*n_IOCTL_PARAM_ATTR_TYPE_urnUE_OUTPUT:iho	stru(*n_IOCTL_PARAM_ATTR_TYPE_urnUE_INOUT:iho	areu.ck(&s.a	 * user[n].u.ck(&s.a;iho	areu.ck(&s.b	 * user[n].u.ck(&s.b;iho	areu.ck(&s.c	 * user[n].u.ck(&s.c;nse.
			 */
o	stru(*n_IOCTL_PARAM_ATTR_TYPE_MEMREF_OUTPUT:iho	stru(*n_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT:iho	areu.t =ref.L* @	 * user[n].u.t =ref.L* @;nse.
			 */
odefa 0;:nse.
			 */
o a nedata */

	if ,
			struto out;
	}

	*funcomplemwq-xlue;
	*num_params = supp->num_params;
	memcpyupp->data_toLoptee_soAT farectthixms =um_pinuerecv() below.
	 */

	supp-parameters, ram,
	       sizeof(struct tee_param) * supp->num_par