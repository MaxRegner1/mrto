/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _PARISC_ERRNO_H
#define _PARISC_ERRNO_H

#include <asm-generic/errno-base.h>

#define	ENOMSG		35	/* No message of desired type */
#define	EIDRM		36	/* Identifier removed */
#define	ECHRNG		37	/* Channel number out of range */
#define	EL2NSYNC	38	/* Level 2 not synchronized */
#define	EL3HLT		39	/* Level 3 halted */
#define	EL3RST		40	/* Level 3 reset */
#define	ELNRNG		41	/* Link number out of range */
#define	EUNATCH		42	/* Protocol driver not attached */
#define	ENOCSI		43	/* No CSI structure available */
#define	EL2HLT		44	/* Level 2 halted */
#define	EDEADLK		45	/* Resource deadlock would occur */
#define	EDEADLOCK	EDEADLK
#define	ENOLCK		46	/* No record locks available */
#define	EILSEQ		47	/* Illegal byte sequence */

#define	ENONET		50	/* Machine is not on the network */
#define	ENODATA		51	/* No data available */
#define	ETIME		52	/* Timer expired */
#define	ENOSR		53	/* Out of streams resources */
#define	ENOSTR		54	/* Device not a stream */
#define	ENOPKG		55	/* Package not installed */

#define	ENOLINK		57	/* Link has been severed */
#define	EADV		58	/* Advertise error */
#define	ESRMNT		59	/* Srmount error */
#define	ECOMM		60	/* Communication error on send */
#define	EPROTO		61	/* Protocol error */

#define	EMULTIHOP	64	/* Multihop attempted */

#define	EDOTDOT		66	/* RFS specific error */
#define	EBADMSG		67	/* Not a data message */
#define	EUSERS		68	/* Too many users */
#define	EDQUOT		69	/* Quota exceeded */
#define	ESTALE		70	/* Stale file handle */
#define	EREMOTE		71	/* Object is remote */
#define	EOVERFLOW	72	/* Value too large for defined data type */

/* these errnos are defined by Linux but not HPUX. */

#define	EBADE		160	/* Invalid exchange */
#define	EBADR		161	/* Invalid request descriptor */
#define	EXFULL		162	/* Exchange full */
#define	ENOANO		163	/* No anode */
#define	EBADRQC		164	/* Invalid request code */
#define	EBADSLT		165	/* Invalid slot */
#define	EBFONT		166	/* Bad font file format */
#define	ENOTUNIQ	167	/* Name not unique on network */
#define	EBADFD		168	/* File descriptor in bad state */
#define	EREMCHG		169	/* Remote address changed */
#define	ELIBACC		170	/* Can not access a needed shared library */
#define	ELIBBAD		171	/* Accessing a corrupted shared library */
#define	ELIBSCN		172	/* .lib section in a.out corrupted */
#define	ELIBMAX		173	/* Attempting to link in too many shared libraries */
#define	ELIBEXEC	174	/* Cannot exec Inv