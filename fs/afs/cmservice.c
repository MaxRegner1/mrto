/* AFS Cache Manager Service
 *
 * Copyright (C) 2002 Red Hat, Inc. All Rights Reserved.
 * Written by David Howells (dhowells@redhat.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ip.h>
#include "internal.h"
#include "afs_cm.h"

static int afs_deliver_cb_init_call_back_state(struct afs_call *);
static int afs_deliver_cb_init_call_back_state3(struct afs_call *);
static int afs_deliver_cb_probe(struct afs_call *);
static int afs_deliver_cb_callback(struct afs_call *);
static int afs_deliver_cb_probe_uuid(struct afs_call *);
static int afs_deliver_cb_tell_me_about_yourself(struct afs_call *);
static void afs_cm_destructor(struct afs_call *);
static void SRXAFSCB_CallBack(struct work_struct *);
static void SRXAFSCB_InitCallBackState(struct work_struct *);
static void SRXAFSCB_Probe(struct work_struct *);
static void SRXAFSCB_ProbeUuid(struct work_struct *);
static void SRXAFSCB_TellMeAboutYourself(struct work_struct *);

#define CM_NAME(name) \
	const char afs_SRXCB##name##_name[] __tracepoint_string =	\
		"CB." #name

/*
 * CB.CallBack operation type
 */
static CM_NAME(CallBack);
static const struct afs_call_type afs_SRXCBCallBack = {
	.name		= afs_SRXCBCallBack_name,
	.deliver	= afs_deliver_cb_callback,
	.abort_to_error	= afs_abort_to_error,
	.destructor	= afs_cm_destructor,
	.work		= SRXAFSCB_CallBack,
};

/*
 * CB.InitCallBackState operation type
 */
static CM_NAME(InitCallBackState);
static const struct afs_call_type afs_SRXCBInitCallBackState = {
	.name		= afs_SRXCBInitCallBackState_name,
	.deliver	= afs_deliver_cb_init_call_back_state,
	.abort_to_error	= afs_abort_to_error,
	.destructor	= afs_cm_destructor,
	.work		= SRXAFSCB_InitCallBackState,
};

/*
 * CB.InitCallBackState3 operation type
 */
static CM_NAME(InitCallBackState3);
static const struct afs_call_type afs_SRXCBInitCallBackState3 = {
	.name		= afs_SRXCBInitCallBackState3_name,
	.deliver	= afs_deliver_cb_init_call_back_state3,
	.abort_to_error	= afs_abort_to_error,
	.destructor	= afs_cm_destructor,
	.work		= SRXAFSCB_InitCallBackState,
};

/*
 * CB.Probe operation type
 */
static CM_NAME(Probe);
static const struct afs_call_type afs_SRXCBProbe = {
	.name		= afs_SRXCBProbe_name,
	.deliver	= afs_deliver_cb_probe,
	.abort_to_error	= afs_abort_to_error,
	.destructor	= afs_cm_destructor,
	.work		= SRXAFSCB_Probe,
};

/*
 * CB.ProbeUuid operation type
 */
static CM_NAME(ProbeUuid);
static const struct afs_call_type afs_SRXCBProbeUuid = {
	.name		= afs_SRXCBProbeUuid_name,
	.deliver	= afs_deliver_cb_probe_uuid,
	.abort_to_error	= afs_abort_to_error,
	.destructor	= afs_cm_destructor,
	.work		= SRXAFSCB_ProbeUuid,
};

/*
 * CB.TellMeAboutYourself operation type
 */
static CM_NAME(TellMeAboutYourself);
static const struct afs_call_type afs_SRXCBTellMeAboutYourself = {
	.name		= afs_SRXCBTellMeAboutYourself_name,
	.deliver	= afs_deliver_cb_tell_me_about_yourself,
	.abort_to_error	= afs_abort_to_error,
	.destructor	= afs_cm_destructor,
	.work		= SRXAFSCB_TellMeAboutYourself,
};

/*
 * route an incoming cache manager call
 * - return T if supported, F if not
 */
bool afs_cm_incoming_call(struct afs_call *call)
{
	_enter("{CB.OP %u}", call->operation_ID);

	switch (call->operation_ID) {
	case CBCallBack:
		call->type = &afs_SRXCBCallBack;
		return true;
	case CBInitCallBackState:
		call->type = &afs_SRXCBInitCallBackState;
		return true;
	case CBInitCallBackState3:
		call->type = &afs_SRXCBInitCallBackState3;
		return true;
	case CBProbe:
		call->type = &afs_SRXCBProbe;
		return true;
	case CBProbeUuid:
		call->type = &afs_SRXCBProbeUuid;
		return true;
	case CBTellMeAboutYourself:
		call->type = &afs_SRXCBTellMeAboutYourself;
		return true;
	default:
		return false;
	}
}

/*
 * clean up a cache manager call
 */
static void afs_cm_destructor(struct afs_call *call)
{
	_enter("");

	/* Break the callbacks here so that we do it after the final ACK is
	 * received.  The step number here must match the final number in
	 * afs_deliver_cb_callback().
	 */
	if (call->unmarshall == 5) {
		ASSERT(call->server && call->count && call->request);
		afs_break_callbacks(call->server, call->count, call->request);
	}

	afs_put_server(call->server);
	call->server = NULL;
	kfree(call->buffer);
	call->buffer = NULL;
}

/*
 * allow the fileserver to see if the cache manager is still alive
 */
static void SRXAFSCB_CallBack(struct work_struct *work)
{
	struct afs_call *call = container_of(work, struct afs_call, work);

	_enter("");

	/* be sure to send the reply *before* attempting to spam the AFS server
	 * with FSFetchStatus requests on the vnodes with broken callbacks lest
	 * the AFS server get into a vicious cycle of trying to break further
	 * callbacks because it hadn't received completion of the CBCallBack op
	 * yet */
	afs_send_empty_reply(call);

	afs_break_callbacks(call->server, call->count, call->request);
	afs_put_call(call);
	_leave("");
}

/*
 * deliver request data to a CB.CallBack call
 */
static int afs_deliver_cb_callback(struct afs_call *call)
{
	struct sockaddr_rxrpc srx;
	struct afs_callback *cb;
	struct afs_server *server;
	__be32 *bp;
	int ret, loop;

	_enter("g