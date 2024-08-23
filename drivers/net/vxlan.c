/*
 * VXLAN: Virtual eXtensible Local Area Network
 *
 * Copyright (c) 2012-2013 Vyatta Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/udp.h>
#include <linux/igmp.h>
#include <linux/if_ether.h>
#include <linux/ethtool.h>
#include <net/arp.h>
#include <net/ndisc.h>
#include <net/ip.h>
#include <net/icmp.h>
#include <net/rtnetlink.h>
#include <net/inet_ecn.h>
#include <net/net_namespace.h>
#include <net/netns/generic.h>
#include <net/tun_proto.h>
#include <net/vxlan.h>

#if IS_ENABLED(CONFIG_IPV6)
#include <net/ip6_tunnel.h>
#include <net/ip6_checksum.h>
#endif

#define VXLAN_VERSION	"0.1"

#define PORT_HASH_BITS	8
#define PORT_HASH_SIZE  (1<<PORT_HASH_BITS)
#define FDB_AGE_DEFAULT 300 /* 5 min */
#define FDB_AGE_INTERVAL (10 * HZ)	/* rescan interval */

/* UDP port for VXLAN traffic.
 * The IANA assigned port is 4789, but the Linux default is 8472
 * for compatibility with early adopters.
 */
static unsigned short vxlan_port __read_mostly = 8472;
module_param_named(udp_port, vxlan_port, ushort, 0444);
MODULE_PARM_DESC(udp_port, "Destination UDP port");

static bool log_ecn_error = true;
module_param(log_ecn_error, bool, 0644);
MODULE_PARM_DESC(log_ecn_error, "Log packets received with corrupted ECN");

static unsigned int vxlan_net_id;
static struct rtnl_link_ops vxlan_link_ops;

static const u8 all_zeros_mac[ETH_ALEN + 2];

static int vxlan_sock_add(struct vxlan_dev *vxlan);

static void vxlan_vs_del_dev(struct vxlan_dev *vxlan);

/* per-network namespace private data for this module */
struct vxlan_net {
	struct list_head  vxlan_list;
	struct hlist_head sock_list[PORT_HASH_SIZE];
	spinlock_t	  sock_lock;
};

/* Forwarding table entry */
struct vxlan_fdb {
	struct hlist_node hlist;	/* linked list of entries */
	struct rcu_head	  rcu;
	unsigned long	  updated;	/* jiffies */
	unsigned long	  used;
	struct list_head  remotes;
	u8		  eth_addr[ETH_ALEN];
	u16		  state;	/* see ndm_state */
	__be32		  vni;
	u8		  flags;	/* see ndm_flags */
};

/* salt for hash table */
static u32 vxlan_salt __read_mostly;

static inline bool vxlan_collect_metadata(struct vxlan_sock *vs)
{
	return vs->flags & VXLAN_F_COLLECT_METADATA ||
	       ip_tunnel_collect_metadata();
}

#if IS_ENABLED(CONFIG_IPV6)
static inline
bool vxlan_addr_equal(const union vxlan_addr *a, const union vxlan_addr *b)
{
	if (a->sa.sa_family != b->sa.sa_family)
		return false;
	if (a->sa.sa_family == AF_INET6)
		return ipv6_addr_equal(&a->sin6.sin6_addr, &b->sin6.sin6_addr);
	else
		return a->sin.sin_addr.s_addr == b->sin.sin_addr.s_addr;
}

static inline bool vxlan_addr_any(const union vxlan_addr *ipa)
{
	if (ipa->sa.sa_family == AF_INET6)
		return ipv6_addr_any(&ipa->sin6.sin6_addr);
	else
		return ipa->sin.sin_addr.s_addr == htonl(INADDR_ANY);
}

static inline bool vxlan_addr_multicast(const union vxlan_addr *ipa)
{
	if (ipa->sa.sa_family == AF_INET6)
		return ipv6_addr_is_multicast(&ipa->sin6.sin6_addr);
	else
		return IN_MULTICAST(ntohl(ipa->sin.sin_addr.s_addr));
}

static int vxlan_nla_get_addr(union vxlan_addr *ip, struct nlattr *nla)
{
	if (nla_len(nla) >= sizeof(struct in6_addr)) {
		ip->sin6.sin6_addr = nla_get_in6_addr(nla);
		ip->sa.sa_family = AF_INET6;
		return 0;
	} else if (nla_len(nla) >= sizeof(__be32)) {
		ip->sin.sin_addr.s_addr = nla_get_in_addr(nla);
		ip->sa.sa_family = AF_INET;
		return 0;
	} else {
		return -EAFNOSUPPORT;
	}
}

static int vxlan_nla_put_addr(struct sk_buff *skb, int attr,
			      const union vxlan_addr *ip)
{
	if (ip->sa.sa_family == AF_INET6)
		return nla_put_in6_addr(skb, attr, &ip->sin6.sin6_addr);
	else
		return nla_put_in_addr(skb, attr, ip->sin.sin_addr.s_addr);
}

#else /* !CONFIG_IPV6 */

static inline
bool vxlan_addr_equal(const union vxlan_addr *a, const union vxlan_addr *b)
{
	return a->sin.sin_addr.s_addr == b->sin.sin_addr.s_addr;
}

static inline bool vxlan_addr_any(const union vxlan_addr *ipa)
{
	return ipa->sin.sin_addr.s_addr == htonl(INADDR_ANY);
}

static inline bool vxlan_addr_multicast(const union vxlan_addr *ipa)
{
	return IN_MULTICAST(ntohl(ipa->sin.sin_addr.s_addr));
}

static int vxlan_nla_get_addr(union vxlan_addr *ip, struct nlattr *nla)
{
	if (nla_len(nla) >= sizeof(struct in6_addr)) {
		return -EAFNOSUPPORT;
	} else if (nla_len(nla) >= sizeof(__be32)) {
		ip->sin.sin_addr.s_addr = nla_get_in_addr(nla);
		ip->sa.sa_family = AF_INET;
		return 0;
	} else {
		return -EAFNOSUPPORT;
	}
}

static int vxlan_nla_put_addr(struct sk_buff *skb, int attr,
			      const union vxlan_addr *ip)
{
	return nla_put_in_addr(skb, attr, ip->sin.sin_addr.s_addr);
}
#endif

/* Virtual Network hash table head */
static inline struct hlist_head *vni_head(struct vxlan_sock *vs, __be32 vni)
{
	return &vs->vni_list[hash_32((__force u32)vni, VNI_HASH_BITS)];
}

/* Socket hash table head */
static inline struct hlist_head *vs_head(struct net *net, __be16 port)
{
	struct vxlan_net *vn = net_generic(net, vxlan_net_id);

	return &vn->sock_list[hash_32(ntohs(port), PORT_HASH_BITS)];
}

/* First remote destination for a forwarding entry.
 * Guaranteed to be non-NULL because remotes are never deleted.
 */
static inline struct vxlan_rdst *first_remote_rcu(struct vxlan_fdb *fdb)
{
	return list_entry_rcu(fdb->remotes.next, struct vxlan_rdst, list);
}

static inline struct vxlan_rdst *first_remote_rtnl(struct vxlan_fdb *fdb)
{
	return list_first_entry(&fdb->remotes, struct vxlan_rdst, list);
}

/* Find VXLAN socket based on network namespace, address family and UDP port
 * and enabled unshareable flags.
 */
static struct vxlan_sock *vxlan_find_sock(struct net *net, sa_family_t family,
					  __be16 port, u32 flags)
{
	struct vxlan_sock *vs;

	flags &= VXLAN_F_RCV_FLAGS;

	hlist_for_each_entry_rcu(vs, vs_head(net, port), hlist) {
		if (inet_sk(vs->sock->sk)->inet_sport == port &&
		    vxlan_get_sk_family(vs) == family &&
		    vs->flags == flags)
			return vs;
	}
	return NULL;
}

static struct vxlan_dev *vxlan_vs_find_vni(struct vxlan_sock *vs, int ifindex,
					   __be32 vni)
{
	struct vxlan_dev_node *node;

	/* For flow based devices, map all packets to VNI 0 */
	if (vs->flags & VXLAN_F_COLLECT_METADATA)
		vni = 0;

	hlist_for_each_entry_rcu(node, vni_head(vs, vni), hlist) {
		if (node->vxlan->default_dst.remote_vni != vni)
			continue;

		if (IS_ENABLED(CONFIG_IPV6)) {
			const struct vxlan_config *cfg = &node->vxlan->cfg;

			if ((cfg->flags & VXLAN_F_IPV6_LINKLOCAL) &&
			    cfg->remote_ifindex != ifindex)
				continue;
		}

		return node->vxlan;
	}

	return NULL;
}

/* Look up VNI in a per net namespace table */
static struct vxlan_dev *vxlan_find_vni(struct net *net, int ifindex,
					__be32 vni, sa_family_t family,
					__be16 port, u32 flags)
{
	struct vxlan_sock *vs;

	vs = vxlan_find_sock(net, family, port, flags);
	if (!vs)
		return NULL;

	return vxlan_vs_find_vni(vs, ifindex, vni);
}

/* Fill in neighbour message in skbuff. */
static int vxlan_fdb_info(struct sk_buff *skb, struct vxlan_dev *vxlan,
			  const struct vxlan_fdb *fdb,
			  u32 portid, u32 seq, int type, unsigned int flags,
			  const struct vxlan_rdst *rdst)
{
	unsigned long now = jiffies;
	struct nda_cacheinfo ci;
	struct nlmsghdr *nlh;
	struct ndmsg *ndm;
	bool send_ip, send_eth;

	nlh = nlmsg_put(skb, portid, seq, type, sizeof(*ndm), flags);
	if (nlh == NULL)
		return -EMSGSIZE;

	ndm = nlmsg_data(nlh);
	memset(ndm, 0, sizeof(*ndm));

	send_eth = send_ip = true;

	if (type == RTM_GETNEIGH) {
		send_ip = !vxlan_addr_any(&rdst->remote_ip);
		send_eth = !is_zero_ether_addr(fdb->eth_addr);
		ndm->ndm_family = send_ip ? rdst->remote_ip.sa.sa_family : AF_INET;
	} else
		ndm->ndm_family	= AF_BRIDGE;
	ndm->ndm_state = fdb->state;
	ndm->ndm_ifindex = vxlan->dev->ifindex;
	ndm->ndm_flags = fdb->flags;
	ndm->ndm_type = RTN_UNICAST;

	if (!net_eq(dev_net(vxlan->dev), vxlan->net) &&
	    nla_put_s32(skb, NDA_LINK_NETNSID,
			peernet2id(dev_net(vxlan->dev), vxlan->net)))
		goto nla_put_failure;

	if (send_eth && nla_put(skb, NDA_LLADDR, ETH_ALEN, &fdb->eth_addr))
		goto nla_put_failure;

	if (send_ip && vxlan_nla_put_addr(skb, NDA_DST, &rdst->remote_ip))
		goto nla_put_failure;

	if (rdst->remote_port && rdst->remote_port != vxlan->cfg.dst_port &&
	    nla_put_be16(skb, NDA_PORT, rdst->remote_port))
		goto nla_put_failure;
	if (rdst->remote_vni != vxlan->default_dst.remote_vni &&
	    nla_put_u32(skb, NDA_VNI, be32_to_cpu(rdst->remote_vni)))
		goto nla_put_failure;
	if ((vxlan->cfg.flags & VXLAN_F_COLLECT_METADATA) && fdb->vni &&
	    nla_put_u32(skb, NDA_SRC_VNI,
			be32_to_cpu(fdb->vni)))
		goto nla_put_failure;
	if (rdst->remote_ifindex &&
	    nla_put_u32(skb, NDA_IFINDEX, rdst->remote_ifindex))
		goto nla_put_failure;

	ci.ndm_used	 = jiffies_to_clock_t(now - fdb->used);
	ci.ndm_confirmed = 0;
	ci.ndm_updated	 = jiffies_to_clock_t(now - fdb->updated);
	ci.ndm_refcnt	 = 0;

	if (nla_put(skb, NDA_CACHEINFO, sizeof(ci), &ci))
		goto nla_put_failure;

	nlmsg_end(skb, nlh);
	return 0;

nla_put_failure:
	nlmsg_cancel(skb, nlh);
	return -EMSGSIZE;
}

static inline size_t vxlan_nlmsg_size(void)
{
	return NLMSG_ALIGN(sizeof(struct ndmsg))
		+ nla_total_size(ETH_ALEN) /* NDA_LLADDR */
		+ nla_total_size(sizeof(struct in6_addr)) /* NDA_DST */
		+ nla_total_size(sizeof(__be16)) /* NDA_PORT */
		+ nla_total_size(sizeof(__be32)) /* NDA_VNI */
		+ nla_total_size(sizeof(__u32)) /* NDA_IFINDEX */
		+ nla_total_size(sizeof(__s32)) /* NDA_LINK_NETNSID */
		+ nla_total_size(sizeof(struct nda_cacheinfo));
}

static void vxlan_fdb_notify(struct vxlan_dev *vxlan, struct vxlan_fdb *fdb,
			     struct vxlan_rdst *rd, int type)
{
	struct net *net = dev_net(vxlan->dev);
	struct sk_buff *skb;
	int err = -ENOBUFS;

	skb = nlmsg_new(vxlan_nlmsg_size(), GFP_ATOMIC);
	if (skb == NULL)
		goto errout;

	err = vxlan_fdb_info(skb, vxlan, fdb, 0, 0, type, 0, rd);
	if (err < 0) {
		/* -EMSGSIZE implies BUG in vxlan_nlmsg_size() */
		WARN_ON(err == -EMSGSIZE);
		kfree_skb(skb);
		goto errout;
	}

	rtnl_notify(skb, net, 0, RTNLGRP_NEIGH, NULL, GFP_ATOMIC);
	return;
errout:
	if (err < 0)
		rtnl_set_sk_err(net, RTNLGRP_NEIGH, err);
}

static void vxlan_ip_miss(struct net_device *dev, union vxlan_addr *ipa)
{
	struct vxlan_dev *vxlan = netdev_priv(dev);
	struct vxlan_fdb f = {
		.state = NUD_STALE,
	};
	struct vxlan_rdst remote = {
		.remote_ip = *ipa, /* goes to NDA_DST */
		.remote_vni = cpu_to_be32(VXLAN_N_VID),
	};

	vxlan_fdb_notify(vxlan, &f, &remote, RTM_GETNEIGH);
}

static void vxlan_fdb_miss(struct vxlan_dev *vxlan, const u8 eth_addr[ETH_ALEN])
{
	struct vxlan_fdb f = {
		.state = NUD_STALE,
	};
	struct vxlan_rdst remote = { };

	memcpy(f.eth_addr, eth_addr, ETH_ALEN);

	vxlan_fdb_notify(vxlan, &f, &remote, RTM_GETNEIGH);
}

/* Hash Ethernet address */
static u32 eth_hash(const unsigned char *addr)
{
	u64 value = get_unaligned((u64 *)addr);

	/* only want 6 bytes */
#ifdef __BIG_ENDIAN
	value >>= 16;
#else
	value <<= 16;
#endif
	return hash_64(value, FDB_HASH_BITS);
}

static u32 eth_vni_hash(const unsigned char *addr, __be32 vni)
{
	/* use 1 byte of OUI and 3 bytes of NIC */
	u32 key = get_unaligned((u32 *)(addr + 2));

	return jhash_2words(key, vni, vxlan_salt) & (FDB_HASH_SIZE - 1);
}

/* Hash chain to use given mac address */
static inline struct hlist_head *vxlan_fdb_head(struct vxlan_dev *vxlan,
						const u8 *mac, __be32 vni)
{
	if (vxlan->cfg.flags & VXLAN_F_COLLECT_METADATA)
		return &vxlan->fdb_head[eth_vni_hash(mac, vni)];
	else
		return &vxlan->fdb_head[eth_hash(mac)];
}

/* Look up Ethernet address in forwarding table */
static struct vxlan_fdb *__vxlan_find_mac(struct vxlan_dev *vxlan,
					  const u8 *mac, __be32 vni)
{
	struct hlist_head *head = vxlan_fdb_head(vxlan, mac, vni);
	struct vxlan_fdb *f;

	hlist_for_each_entry_rcu(f, head, hlist) {
		if (ether_addr_equal(mac, f->eth_addr)) {
			if (vxlan->cfg.flags & VXLAN_F_COLLECT_METADATA) {
				if (vni == f->vni)
					return f;
			} else {
				return f;
			}
		}
	}

	return NULL;
}

static struct vxlan_fdb *vxlan_find_mac(struct vxlan_dev *vxlan,
					const u8 *mac, __be32 vni)
{
	struct vxlan_fdb *f;

	f = __vxlan_find_mac(vxlan, mac, vni);
	if (f)
		f->used = jiffies;

	return f;
}

/* caller should hold vxlan->hash_lock */
static struct vxlan_rdst *vxlan_fdb_find_rdst(struct vxlan_fdb *f,
					      union vxlan_addr *ip, __be16 port,
					      __be32 vni, __u32 ifindex)
{
	struct vxlan_rdst *rd;

	list_for_each_entry(rd, &f->remotes, list) {
		if (vxlan_addr_equal(&rd->remote_ip, ip) &&
		    rd->remote_port == port &&
		    rd->remote_vni == vni &&
		    rd->remote_ifindex == ifindex)
			return rd;
	}

	return NULL;
}

/* Replace destination of unicast mac */
static int vxlan_fdb_replace(struct vxlan_fdb *f,
			     union vxlan_addr *ip, __be16 port, __be32 vni,
			     __u32 ifindex)
{
	struct vxlan_rdst *rd;

	rd = vxlan_fdb_find_rdst(f, ip, port, vni, ifindex);
	if (rd)
		return 0;

	rd = list_first_entry_or_null(&f->remotes, struct vxlan_rdst, list);
	if (!rd)
		return 0;

	dst_cache_reset(&rd->dst_cache);
	rd->remote_ip = *ip;
	rd->remote_port = port;
	rd->remote_vni = vni;
	rd->remote_ifindex = ifindex;
	return 1;
}

/* Add/update destinations for multicast */
static int vxlan_fdb_append(struct vxlan_fdb *f,
			    union vxlan_addr *ip, __be16 port, __be32 vni,
			    __u32 ifindex, struct vxlan_rdst **rdp)
{
	struct vxlan_rdst *rd;

	rd = vxlan_fdb_find_rdst(f, ip, port, vni, ifindex);
	if (rd)
		return 0;

	rd = kmalloc(sizeof(*rd), GFP_ATOMIC);
	if (rd == NULL)
		return -ENOBUFS;

	if (dst_cache_init(&rd->dst_cache, GFP_ATOMIC)) {
		kfree(rd);
		return -ENOBUFS;
	}

	rd->remote_ip = *ip;
	rd->remote_port = port;
	rd->remote_vni = vni;
	rd->remote_ifindex = ifindex;

	list_add_tail_rcu(&rd->list, &f->remotes);

	*rdp = rd;
	return 1;
}

static struct vxlanhdr *vxlan_gro_remcsum(struct sk_buff *skb,
					  unsigned int off,
					  struct vxlanhdr *vh, size_t hdrlen,
					  __be32 vni_field,
					  struct gro_remcsum *grc,
					  bool nopartial)
{
	size_t start, offset;

	if (skb->remcsum_offload)
		return vh;

	if (!NAPI_GRO_CB(skb)->csum_valid)
		return NULL;

	start = vxlan_rco_start(vni_field);
	offset = start + vxlan_rco_offset(vni_field);

	vh = skb_gro_remcsum_process(skb, (void *)vh, off, hdrlen,
				     start, offset, grc, nopartial);

	skb->remcsum_offload = 1;

	return vh;
}

static struct sk_buff *vxlan_gro_receive(struct sock *sk,
					 struct list_head *head,
					 struct sk_buff *skb)
{
	struct sk_buff *pp = NULL;
	struct sk_buff *p;
	struct vxlanhdr *vh, *vh2;
	unsigned int hlen, off_vx;
	int flush = 1;
	struct vxlan_sock *vs = rcu_dereference_sk_user_data(sk);
	__be32 flags;
	struct gro_remcsum grc;

	skb_gro_remcsum_init(&grc);

	off_vx = skb_gro_offset(skb);
	hlen = off_vx + sizeof(*vh);
	vh   = skb_gro_header_fast(skb, off_vx);
	if (skb_gro_header_hard(skb, hlen)) {
		vh = skb_gro_header_slow(skb, hlen, off_vx);
		if (unlikely(!vh))
			goto out;
	}

	skb_gro_postpull_rcsum(skb, vh, sizeof(struct vxlanhdr));

	flags = vh->vx_flags;

	if ((flags & VXLAN_HF_RCO) && (vs->flags & VXLAN_F_REMCSUM_RX)) {
		vh = vxlan_gro_remcsum(skb, off_vx, vh, sizeof(struct vxlanhdr),
				       vh->vx_vni, &grc,
				       !!(vs->flags &
					  VXLAN_F_REMCSUM_NOPARTIAL));

		if (!vh)
			goto out;
	}

	skb_gro_pull(skb, sizeof(struct vxlanhdr)); /* pull vxlan header */

	list_for_each_entry(p, head, list) {
		if (!NAPI_GRO_CB(p)->same_flow)
			continue;

		vh2 = (struct vxlanhdr *)(p->data + off_vx);
		if (vh->vx_flags != vh2->vx_flags ||
		    vh->vx_vni != vh2->vx_vni) {
			NAPI_GRO_CB(p)->same_flow = 0;
			continue;
		}
	}

	pp = call_gro_receive(eth_gro_receive, head, skb);
	flush = 0;

out:
	skb_gro_flush_final_remcsum(skb, pp, flush, &grc);

	return pp;
}

static int vxlan_gro_complete(struct sock *sk, struct sk_buff *skb, int nhoff)
{
	/* Sets 'skb->inner_mac_header' since we are always called with
	 * 'skb->encapsulation' set.
	 */
	return eth_gro_complete(skb, nhoff + sizeof(struct vxlanhdr));
}

static struct vxlan_fdb *vxlan_fdb_alloc(struct vxlan_dev *vxlan,
					 const u8 *mac, __u16 state,
					 __be32 src_vni, __u8 ndm_flags)
{
	struct vxlan_fdb *f;

	f = kmalloc(sizeof(*f), GFP_ATOMIC);
	if (!f)
		return NULL;
	f->state = state;
	f->flags = ndm_flags;
	f->updated = f->used = jiffies;
	f->vni = src_vni;
	INIT_LIST_HEAD(&f->remotes);
	memcpy(f->eth_addr, mac, ETH_ALEN);

	return f;
}

static int vxlan_fdb_create(struct vxlan_dev *vxlan,
			    const u8 *mac, union vxlan_addr *ip,
			    __u16 state, __be16 port, __be32 src_vni,
			    __be32 vni, __u32 ifindex, __u8 ndm_flags,
			    struct vxlan_fdb **fdb)
{
	struct vxlan_rdst *rd = NULL;
	struct vxlan_fdb *f;
	int rc;

	if (vxlan->cfg.addrmax &&
	    vxlan->addrcnt >= vxlan->cfg.addrmax)
		return -ENOSPC;

	netdev_dbg(vxlan->dev, "add %pM -> %pIS\n", mac, ip);
	f = vxlan_fdb_alloc(vxlan, mac, state, src_vni, ndm_flags);
	if (!f)
		return -ENOMEM;

	rc = vxlan_fdb_append(f, ip, port, vni, ifindex, &rd);
	if (rc < 0) {
		kfree(f);
		return rc;
	}

	++vxlan->addrcnt;
	hlist_add_head_rcu(&f->hlist,
			   vxlan_fdb_head(vxlan, mac, src_vni));

	*fdb = f;

	return 0;
}

/* Add new entry to forwarding table -- assumes lock held */
static int vxlan_fdb_update(struct vxlan_dev *vxlan,
			    const u8 *mac, union vxlan_addr *ip,
			    __u16 state, __u16 flags,
			    __be16 port, __be32 src_vni, __be32 vni,
			    __u32 ifindex, __u8 ndm_flags)
{
	struct vxlan_rdst *rd = NULL;
	struct vxlan_fdb *f;
	int notify = 0;
	int rc;

	f = __vxlan_find_mac(vxlan, mac, src_vni);
	if (f) {
		if (flags & NLM_F_EXCL) {
			netdev_dbg(vxlan->dev,
				   "lost race to create %pM\n", mac);
			return -EEXIST;
		}
		if (f->state != state) {
			f->state = state;
			f->updated = jiffies;
			notify = 1;
		}
		if (f->flags != ndm_flags) {
			f->flags = ndm_flags;
			f->updated = jiffies;
			notify = 1;
		}
		if ((flags & NLM_F_REPLACE)) {
			/* Only change unicasts */
			if (!(is_multicast_ether_addr(f->eth_addr) ||
			     is_zero_ether_addr(f->eth_addr))) {
				notify |= vxlan_fdb_replace(f, ip, port, vni,
							   ifindex);
			} else
				return -EOPNOTSUPP;
		}
		if ((flags & NLM_F_APPEND) &&
		    (is_multicast_ether_addr(f->eth_addr) ||
		     is_zero_ether_addr(f->eth_addr))) {
			rc = vxlan_fdb_append(f, ip, port, vni, ifindex, &rd);

			if (rc < 0)
				return rc;
			notify |= rc;
		}
	} else {
		if (!(flags & NLM_F_CREATE))
			return -ENOENT;

		/* Disallow replace to add a multicast entry */
		if ((flags & NLM_F_REPLACE) &&
		    (is_multicast_ether_addr(mac) || is_zero_ether_addr(mac)))
			return -EOPNOTSUPP;

		netdev_dbg(vxlan->dev, "add %pM -> %pIS\n", mac, ip);
		rc = vxlan_fdb_create(vxlan, mac, ip, state, port, src_vni,
				      vni, ifindex, ndm_flags, &f);
		if (rc < 0)
			return rc;
		notify = 1;
	}

	if (notify) {
		if (rd == NULL)
			rd = first_remote_rtnl(f);
		vxlan_fdb_notify(vxlan, f, rd, RTM_NEWNEIGH);
	}

	return 0;
}

static void vxlan_fdb_free(struct rcu_head *head)
{
	struct vxlan_fdb *f = container_of(head, struct vxlan_fdb, rcu);
	struct vxlan_rdst *rd, *nd;

	list_for_each_entry_safe(rd, nd, &f->remotes, list) {
		dst_cache_destroy(&rd->dst_cache);
		kfree(rd);
	}
	kfree(f);
}

static void vxlan_fdb_destroy(struct vxlan_dev *vxlan, struct vxlan_fdb *f,
			      bool do_notify)
{
	netdev_dbg(vxlan->dev,
		    "delete %pM\n", f->eth_addr);

	--vxlan->addrcnt;
	if (do_notify)
		vxlan_fdb_notify(vxlan, f, first_remote_rtnl(f), RTM_DELNEIGH);

	hlist_del_rcu(&f->hlist);
	call_rcu(&f->rcu, vxlan_fdb_free);
}

static void vxlan_dst_free(struct rcu_head *head)
{
	struct vxlan_rdst *rd = container_of(head, struct vxlan_rdst, rcu);

	dst_cache_destroy(&rd->dst_cache);
	kfree(rd);
}

static void vxlan_fdb_dst_destroy(struct vxlan_dev *vxlan, struct vxlan_fdb *f,
				  struct vxlan_rdst *rd)
{
	list_del_rcu(&rd->list);
	vxlan_fdb_notify(vxlan, f, rd, RTM_DELNEIGH);
	call_rcu(&rd->rcu, vxlan_dst_free);
}

static int vxlan_fdb_parse(struct nlattr *tb[], struct vxlan_dev *vxlan,
			   union vxlan_addr *ip, __be16 *port, __be32 *src_vni,
			   __be32 *vni, u32 *ifindex)
{
	struct net *net = dev_net(vxlan->dev);
	int err;

	if (tb[NDA_DST]) {
		err = vxlan_nla_get_addr(ip, tb[NDA_DST]);
		if (err)
			return err;
	} else {
		union vxlan_addr *remote = &vxlan->default_dst.remote_ip;
		if (remote->sa.sa_family == AF_INET) {
			ip->sin.sin_addr.s_addr = htonl(INADDR_ANY);
			ip->sa.sa_family = AF_INET;
#if IS_ENABLED(CONFIG_IPV6)
		} else {
			ip->sin6.sin6_addr = in6addr_any;
			ip->sa.sa_family = AF_INET6;
#endif
		}
	}

	if (tb[NDA_PORT]) {
		if (nla_len(tb[NDA_PORT]) != sizeof(__be16))
			return -EINVAL;
		*port = nla_get_be16(tb[NDA_PORT]);
	} else {
		*port = vxlan->cfg.dst_port;
	}

	if (tb[NDA_VNI]) {
		if (nla_len(tb[NDA_VNI]) != sizeof(u32))
			return -EINVAL;
		*vni = cpu_to_be32(nla_get_u32(tb[NDA_VNI]));
	} else {
		*vni = vxlan->default_dst.remote_vni;
	}

	if (tb[NDA_SRC_VNI]) {
		if (nla_len(tb[NDA_SRC_VNI]) != sizeof(u32))
			return -EINVAL;
		*src_vni = cpu_to_be32(nla_get_u32(tb[NDA_SRC_VNI]));
	} else {
		*src_vni = vxlan->default_dst.remote_vni;
	}

	if (tb[NDA_IFINDEX]) {
		struct net_device *tdev;

		if (nla_len(tb[NDA_IFINDEX]) != sizeof(u32))
			return -EINVAL;
		*ifindex = nla_get_u32(tb[NDA_IFINDEX]);
		tdev = __dev_get_by_index(net, *ifindex);
		if (!tdev)
			return -EADDRNOTAVAIL;
	} else {
		*ifindex = 0;
	}

	return 0;
}

/* Add static entry (via netlink) */
static int vxlan_fdb_add(struct ndmsg *ndm, struct nlattr *tb[],
			 struct net_device *dev,
			 const unsigned char *addr, u16 vid, u16 flags)
{
	struct vxlan_dev *vxlan = netdev_priv(dev);
	/* struct net *net = dev_net(vxlan->dev); */
	union vxlan_addr ip;
	__be16 port;
	__be32 src_vni, vni;
	u32 ifindex;
	int err;

	if (!(ndm->ndm_state & (NUD_PERMANENT|NUD_REACHABLE))) {
		pr_info("RTM_NEWNEIGH with invalid state %#x\n",
			ndm->ndm_state);
		return -EINVAL;
	}

	if (tb[NDA_DST] == NULL)
		return -EINVAL;

	err = vxlan_fdb_parse(tb, vxlan, &ip, &port, &src_vni, &vni, &ifindex);
	if (err)
		return err;

	if (vxlan->default_dst.remote_ip.sa.sa_family != ip.sa.sa_family)
		return -EAFNOSUPPORT;

	spin_lock_bh(&vxlan->hash_lock);
	err = vxlan_fdb_update(vxlan, addr, &ip, ndm->ndm_state, flags,
			       port, src_vni, vni, ifindex, ndm->ndm_flags);
	spin_unlock_bh(&vxlan->hash_lock);

	return err;
}

static int __vxlan_fdb_delete(struct vxlan_dev *vxlan,
			      const unsigned char *addr, union vxlan_addr ip,
			      __be16 port, __be32 src_vni, u32 vni, u32 ifindex,
			      u16 vid)
{
	struct vxlan_fdb *f;
	struct vxlan_rdst *rd = NULL;
	int err = -ENOENT;

	f = vxlan_find_mac(vxlan, addr, src_vni);
	if (!f)
		return err;

	if (!vxlan_addr_any(&ip)) {
		rd = vxlan_fdb_find_rdst(f, &ip, port, vni, ifindex);
		if (!rd)
			goto out;
	}

	/* remove a destination if it's not the only one on the list,
	 * otherwise destroy the fdb entry
	 */
	if (rd && !list_is_singular(&f->remotes)) {
		vxlan_fdb_dst_destroy(vxlan, f, rd);
		goto out;
	}

	vxlan_fdb_destroy(vxlan, f, true);

out:
	return 0;
}

/* Delete entry (via netlink) */
static int vxlan_fdb_delete(struct ndmsg *ndm, struct nlattr *tb[],
			    struct net_device *dev,
			    const unsigned char *addr, u16 vid)
{
	struct vxlan_dev *vxlan = netdev_priv(dev);
	union vxlan_addr ip;
	__be32 src_vni, vni;
	__be16 port;
	u32 ifindex;
	int err;

	err = vxlan_fdb_parse(tb, vxlan, &ip, &port, &src_vni, &vni, &ifindex);
	if (err)
		return err;

	spin_lock_bh(&vxlan->hash_lock);
	err = __vxlan_fdb_delete(vxlan, addr, ip, port, src_vni, vni, ifindex,
				 vid);
	spin_unlock_bh(&vxlan->hash_lock);

	return err;
}

/* Dump forwarding table */
static int vxlan_fdb_dump(struct sk_buff *skb, struct netlink_callback *cb,
			  struct net_device *dev,
			  struct net_device *filter_dev, int *idx)
{
	struct vxlan_dev *vxlan = netdev_priv(dev);
	unsigned int h;
	int err = 0;

	for (h = 0; h < FDB_HASH_SIZE; ++h) {
		struct vxlan_fdb *f;

		hlist_for_each_entry_rcu(f, &vxlan->fdb_head[h], hlist) {
			struct vxlan_rdst *rd;

			list_for_each_entry_rcu(rd, &f->remotes, list) {
				if (*idx < cb->args[2])
					goto skip;

				err = vxlan_fdb_info(skb, vxlan, f,
						     NETLINK_CB(cb->skb).portid,
						     cb->nlh->nlmsg_seq,
						     RTM_NEWNEIGH,
						     NLM_F_MULTI, rd);
				if (err < 0)
					goto out;
skip:
				*idx += 1;
			}
		}
	}
out:
	return err;
}

/* Watch incoming packets to learn mapping between Ethernet address
 * and Tunnel endpoint.
 * Return true if packet is bogus and should be dropped.
 */
static bool vxlan_snoop(struct net_device *dev,
			union vxlan_addr *src_ip, const u8 *src_mac,
			u32 src_ifindex, __be32 vni)
{
	struct vxlan_dev *vxlan = netdev_priv(dev);
	struct vxlan_fdb *f;
	u32 ifindex = 0;

#if IS_ENABLED(CONFIG_IPV6)
	if (src_ip->sa.sa_family == AF_INET6 &&
	    (ipv6_addr_type(&src_ip->sin6.sin6_addr) & IPV6_ADDR_LINKLOCAL))
		ifindex = src_ifindex;
#endif

	f = vxlan_find_mac(vxlan, src_mac, vni);
	if (likely(f)) {
		struct vxlan_rdst *rdst = first_remote_rcu(f);

		if (likely(vxlan_addr_equal(&rdst->remote_ip, src_ip) &&
			   rdst->remote_ifindex == ifindex))
			return false;

		/* Don't migrate static entries, drop packets */
		if (f->state & (NUD_PERMANENT | NUD_NOARP))
			return true;

		if (net_ratelimit())
			netdev_info(dev,
				    "%pM migrated from %pIS to %pIS\n",
				    src_mac, &rdst->remote_ip.sa, &src_ip->sa);

		rdst->remote_ip = *src_ip;
		f->updated = jiffies;
		vxlan_fdb_notify(vxlan, f, rdst, RTM_NEWNEIGH);
	} else {
		/* learned new entry */
		spin_lock(&vxlan->hash_lock);

		/* close off race between vxlan_flush and incoming packets */
		if (netif_running(dev))
			vxlan_fdb_update(vxlan, src_mac, src_ip,
					 NUD_REACHABLE,
					 NLM_F_EXCL|NLM_F_CREATE,
					 vxlan->cfg.dst_port,
					 vni,
					 vxlan->default_dst.remote_vni,
					 ifindex, NTF_SELF);
		spin_unlock(&vxlan->hash_lock);
	}

	return false;
}

/* See if multicast group is already in use by other ID */
static bool vxlan_group_used(struct vxlan_net *vn, struct vxlan_dev *dev)
{
	struct vxlan_dev *vxlan;
	struct vxlan_sock *sock4;
#if IS_ENABLED(CONFIG_IPV6)
	struct vxlan_sock *sock6;
#endif
	unsigned short family = dev->default_dst.remote_ip.sa.sa_family;

	sock4 = rtnl_dereference(dev->vn4_sock);

	/* The vxlan_sock is only used by dev, leaving group has
	 * no effect on other vxlan devices.
	 */
	if (family == AF_INET && sock4 && refcount_read(&sock4->refcnt) == 1)
		return false;
#if IS_ENABLED(CONFIG_IPV6)
	sock6 = rtnl_dereference(dev->vn6_sock);
	if (family == AF_INET6 && sock6 && refcount_read(&sock6->refcnt) == 1)
		return false;
#endif

	list_for_each_entry(vxlan, &vn->vxlan_list, next) {
		if (!netif_running(vxlan->dev) || vxlan == dev)
			continue;

		if (family == AF_INET &&
		    rtnl_dereference(vxlan->vn4_sock) != sock4)
			continue;
#if IS_ENABLED(CONFIG_IPV6)
		if (family == AF_INET6 &&
		    rtnl_dereference(vxlan->vn6_sock) != sock6)
			continue;
#endif

		if (!vxlan_addr_equal(&vxlan->default_dst.remote_ip,
				      &dev->default_dst.remote_ip))
			continue;

		if (vxlan->default_dst.remote_ifindex !=
		    dev->default_dst.remote_ifindex)
			continue;

		return true;
	}

	return false;
}

static bool __vxlan_sock_release_prep(struct vxlan_sock *vs)
{
	struct vxlan_net *vn;

	if (!vs)
		return false;
	if (!refcount_dec_and_test(&vs->refcnt))
		return false;

	vn = net_generic(sock_net(vs->sock->sk), vxlan_net_id);
	spin_lock(&vn->sock_lock);
	hlist_del_rcu(&vs->hlist);
	udp_tunnel_notify_del_rx_port(vs->sock,
				      (vs->flags & VXLAN_F_GPE) ?
				      UDP_TUNNEL_TYPE_VXLAN_GPE :
				      UDP_TUNNEL_TYPE_VXLAN);
	spin_unlock(&vn->sock_lock);

	return true;
}

static void vxlan_sock_release(struct vxlan_dev *vxlan)
{
	struct vxlan_sock *sock4 = rtnl_dereference(vxlan->vn4_sock);
#if IS_ENABLED(CONFIG_IPV6)
	struct vxlan_sock *sock6 = rtnl_dereference(vxlan->vn6_sock);

	RCU_INIT_POINTER(vxlan->vn6_sock, NULL);
#endif

	RCU_INIT_POINTER(vxlan->vn4_sock, NULL);
	synchronize_net();

	vxlan_vs_del_dev(vxlan);

	if (__vxlan_sock_release_prep(sock4)) {
		udp_tunnel_sock_release(sock4->sock);
		kfree(sock4);
	}

#if IS_ENABLED(CONFIG_IPV6)
	if (__vxlan_sock_release_prep(sock6)) {
		udp_tunnel_sock_release(sock6->sock);
		kfree(sock6);
	}
#endif
}

/* Update multicast group membership when first VNI on
 * multicast address is brought up
 */
static int vxlan_igmp_join(struct vxlan_dev *vxlan)
{
	struct sock *sk;
	union vxlan_addr *ip = &vxlan->default_dst.remote_ip;
	int ifindex = vxlan->default_dst.remote_ifindex;
	int ret = -EINVAL;

	if (ip->sa.sa_family == AF_INET) {
		struct vxlan_sock *sock4 = rtnl_dereference(vxlan->vn4_sock);
		struct ip_mreqn mreq = {
			.imr_multiaddr.s_addr	= ip->sin.sin_addr.s_addr,
			.imr_ifindex		= ifindex,
		};

		sk = sock4->sock->sk;
		lock_sock(sk);
		ret = ip_mc_join_group(sk, &mreq);
		release_sock(sk);
#if IS_ENABLED(CONFIG_IPV6)
	} else {
		struct vxlan_sock *sock6 = rtnl_dereference(vxlan->vn6_sock);

		sk = sock6->sock->sk;
		lock_sock(sk);
		ret = ipv6_stub->ipv6_sock_mc_join(sk, ifindex,
						   &ip->sin6.sin6_addr);
		release_sock(sk);
#endif
	}

	return ret;
}

/* Inverse of vxlan_igmp_join when last VNI is brought down */
static int vxlan_igmp_leave(struct vxlan_dev *vxlan)
{
	struct sock *sk;
	union vxlan_addr *ip = &vxlan->default_dst.remote_ip;
	int ifindex = vxlan->default_dst.remote_ifindex;
	int ret = -EINVAL;

	if (ip->sa.sa_family == AF_INET) {
		struct vxlan_sock *sock4 = rtnl_dereference(vxlan->vn4_sock);
		struct ip_mreqn mreq = {
			.imr_multiaddr.s_addr	= ip->sin.sin_addr.s_addr,
			.imr_ifindex		= ifindex,
		};

		sk = sock4->sock->sk;
		lock_sock(sk);
		ret = ip_mc_leave_group(sk, &mreq);
		release_sock(sk);
#if IS_ENABLED(CONFIG_IPV6)
	} else {
		struct vxlan_sock *sock6 = rtnl_dereference(vxlan->vn6_sock);

		sk = sock6->sock->sk;
		lock_sock(sk);
		ret = ipv6_stub->ipv6_sock_mc_drop(sk, ifindex,
						   &ip->sin6.sin6_addr);
		release_sock(sk);
#endif
	}

	return ret;
}

static bool vxlan_remcsum(struct vxlanhdr *unparsed,
			  struct sk_buff *skb, u32 vxflags)
{
	size_t start, offset;

	if (!(unparsed->vx_flags & VXLAN_HF_RCO) || skb->remcsum_offload)
		goto out;

	start = vxlan_rco_start(unparsed->vx_vni);
	offset = start + vxlan_rco_offset(unparsed->vx_vni);

	if (!pskb_may_pull(skb, offset + sizeof(u16)))
		return false;

	skb_remcsum_process(skb, (void *)(vxlan_hdr(skb) + 1), start, offset,
			    !!(vxflags & VXLAN_F_REMCSUM_NOPARTIAL));
out:
	unparsed->vx_flags &= ~VXLAN_HF_RCO;
	unparsed->vx_vni &= VXLAN_VNI_MASK;
	return true;
}

static void vxlan_parse_gbp_hdr(struct vxlanhdr *unparsed,
				struct sk_buff *skb, u32 vxflags,
				struct vxlan_metadata *md)
{
	struct vxlanhdr_gbp *gbp = (struct vxlanhdr_gbp *)unparsed;
	struct metadata_dst *tun_dst;

	if (!(unparsed->vx_flags & VXLAN_HF_GBP))
		goto out;

	md->gbp = ntohs(gbp->policy_id);

	tun_dst = (struct metadata_dst *)skb_dst(skb);
	if (tun_dst) {
		tun_dst->u.tun_info.key.tun_flags |= TUNNEL_VXLAN_OPT;
		tun_dst->u.tun_info.options_len = sizeof(*md);
	}
	if (gbp->dont_learn)
		md->gbp |= VXLAN_GBP_DONT_LEARN;

	if (gbp->policy_applied)
		md->gbp |= VXLAN_GBP_POLICY_APPLIED;

	/* In flow-based mode, GBP is carried in dst_metadata */
	if (!(vxflags & VXLAN_F_COLLECT_METADATA))
		skb->mark = md->gbp;
out:
	unparsed->vx_flags &= ~VXLAN_GBP_USED_BITS;
}

static bool vxlan_parse_gpe_hdr(struct vxlanhdr *unparsed,
				__be16 *protocol,
				struct sk_buff *skb, u32 vxflags)
{
	struct vxlanhdr_gpe *gpe = (struct vxlanhdr_gpe *)unparsed;

	/* Need to have Next Protocol set for interfaces in GPE mode. */
	if (!gpe->np_applied)
		return false;
	/* "The initial version is 0. If a receiver does not support the
	 * version indicated it MUST drop the packet.
	 */
	if (gpe->version != 0)
		return false;
	/* "When the O bit is set to 1, the packet is an OAM packet and OAM
	 * processing MUST occur." However, we don't implement OAM
	 * processing, thus drop the packet.
	 */
	if (gpe->oam_flag)
		return false;

	*protocol = tun_p_to_eth_p(gpe->next_protocol);
	if (!*protocol)
		return false;

	unparsed->vx_flags &= ~VXLAN_GPE_USED_BITS;
	return true;
}

static bool vxlan_set_mac(struct vxlan_dev *vxlan,
			  struct vxlan_sock *vs,
			  struct sk_buff *skb, __be32 vni)
{
	union vxlan_addr saddr;
	u32 ifindex = skb->dev->ifindex;

	skb_reset_mac_header(skb);
	skb->protocol = eth_type_trans(skb, vxlan->dev);
	skb_postpull_rcsum(skb, eth_hdr(skb), ETH_HLEN);

	/* Ignore packet loops (and multicast echo) */
	if (ether_addr_equal(eth_hdr(skb)->h_source, vxlan->dev->dev_addr))
		return false;

	/* Get address from the outer IP header */
	if (vxlan_get_sk_family(vs) == AF_INET) {
		saddr.sin.sin_addr.s_addr = ip_hdr(skb)->saddr;
		saddr.sa.sa_family = AF_INET;
#if IS_ENABLED(CONFIG_IPV6)
	} else {
		saddr.sin6.sin6_addr = ipv6_hdr(skb)->saddr;
		saddr.sa.sa_family = AF_INET6;
#endif
	}

	if ((vxlan->cfg.flags & VXLAN_F_LEARN) &&
	    vxlan_snoop(skb->dev, &saddr, eth_hdr(skb)->h_source, ifindex, vni))
		return false;

	return true;
}

static bool vxlan_ecn_decapsulate(struct vxlan_sock *vs, void *oiph,
				  struct sk_buff *skb)
{
	int err = 0;

	if (vxlan_get_sk_family(vs) == AF_INET)
		err = IP_ECN_decapsulate(oiph, skb);
#if IS_ENABLED(CONFIG_IPV6)
	else
		err = IP6_ECN_decapsulate(oiph, skb);
#endif

	if (unlikely(err) && log_ecn_error) {
		if (vxlan_get_sk_family(vs) == AF_INET)
			net_info_ratelimited("non-ECT from %pI4 with TOS=%#x\n",
					     &((struct iphdr *)oiph)->saddr,
					     ((struct iphdr *)oiph)->tos);
		else
			net_info_ratelimited("non-ECT from %pI6\n",
					     &((struct ipv6hdr *)oiph)->saddr);
	}
	return err <= 1;
}

/* Callback from net/ipv4/udp.c to receive packets */
static int vxlan_rcv(struct sock *sk, struct sk_buff *skb)
{
	struct pcpu_sw_netstats *stats;
	struct vxlan_dev *vxlan;
	struct vxlan_sock *vs;
	struct vxlanhdr unparsed;
	struct vxlan_metadata _md;
	struct vxlan_metadata *md = &_md;
	__be16 protocol = htons(ETH_P_TEB);
	bool raw_proto = f]FI>k=}

st.)addr t.)addr t.)addr t.)adE8IPV6)
		} else {
			ip->sin6.sin6_addr = in6addr_any;
			ip->sa.sa_family = AF_INEark = md->g=_buff!mnyr);
		release_sock(sk);
#endif
	}

	return ret;
}

/* Inverse of vxlan_igmp_join w~_GPE_USED_BITS;
	retuourn 0;Fruct rcu_head *head)
{
	struct vxlan_fdb *f = container_of(head, struct vxlan_fdb,#FI>yI>by(err) && loAF_INEark = md
#endif
kvn other vxla]FI>v
#endif&Fex = vxlan->default_dst.remote_ifindex;
	int ret = -EINVAL;

	if (ip->sa.sa_family~_HLEN);

	/* Ignore p(head, struct  struct  struct  struEerI>yI>Pun_~ddr_equal(eth_hdr(skb of vxlan_igmp]3_ (rd == NULL)
			rd = first_remote_rtnl(f);
		vxlan_fdb_no6packets */
static int Findex;
#endif

	f = vxlan_find_mac(vxlan, src_mac, vni);
	if (likely(f)) {
		struc(rI>yI>kue;dr	= i Disallow replace to add a multicast entry */
		if ((flags & NLM_F_REPLACE) &&
		 lan_fdb_no6pacb_no6pacb_no6pacb_no6E8vxlan)
{
	struct vxlan_sock *sock4 = rtnl_dereference(vxlan->vn4_sock);
#if IS_ENA3_ (rd == 	rd = first_re]FI>ktruct vxlan_dev *vxlan, struct vxlp)addr t.)adde to add a mulF;
	synchronize_net();

	vxlan_vs_del_dev(vxlan);

	if (__vxlan_sock_release_prep(s~buff *skb)
{
	struct (struct vx]FI>gxlan->d#FI>vgs |= TUNNELF	vxlan_fdb_notify(vxlan, f, rd, RTM_NEWNEIGH);
	}

	return 0;
}

static void vxlanulF;
	synchron	NEWNEIGH);
	};
	}

ron	NEWNEIh_ig]FI>UiuiE~f

	if (unlikely(err)ct (struct vx]lease_prep(st!*lan_gf *skb)
{
	strunsignkU= contj1rr = I (nla_len(nla)    &ip->sin6.sin6_addr);
		
star6= I (del_rcu(&rd->Aif
	returf (__v!NI on
 * multicf IS_ENA3_ (re;

	unpa  vxlanr6= I (d = 0;
			coet();

	fdb_no6packetruct vxlanip_mc_ip);
#if ct vx		return fue;dr	= i Di, e;

	unpa fdb_no6paease_prepXLAN_F_Catelimin_fdb(&vxlan->hash_lock)) = 0;
			coet();
t vxlan_dev unplec MUST occuadE8fset;

	if (!otocol,
				struct sk_bufeed to havek(sk);
(__vdp->data t.)addr t.)addr t.)adE8,* "The iKEYourn 0;Fkeyr)) /*);
#if id(ighbouport the
	 sulation' sode. */
 0;
			coet();
t 	3_ (risk);
#if xlan_opts(&n is 0. If a receivparsed->rn err_GRO_CB, vxlan_sed);
		ret{
	ode. */

	unsigned shoo nla_pmdfailure;

	if
	 sulvxlanip_mlush, &grc);

	return pp;
}

streturf (__v!NItruct vx IS_ENA3_ ( vxlanr6= I (d = 0;
			coet();

	
star6= I (del_rcu(&rd->BP
			   __bgs & VXLAN_F_C IS_ENA3_ ( vxlanr6= I (d, 
	 * vxlan(nlathatnd OAc(vx*skbcansk(vs-> laactAF_Itogmily(. Thisocksignknlaurocessip->sa.sa_b,
			 uroaddr);
_P_TEB);n_vs_del_dev(vxl|| sock_release_pr u8 eth_aIf IP6r2 src_vny soc/* Get *skv(vxltruainrr < delenl_d* IP*/
	sremoalfo/
		+uter IP Thisobehaviikedac_hge	err =nl_d* 1rr = RFC (RFC7348) which = ipbe16 sathatnbitbuff lan_gf *nl_d* ff lan_gf *skb)
{
	src_
		 laibuff d.
			capc/*ach P6r2nl_d* uain= &vt (struct vx]le		ip->
	iviius_leae pcet anc(vxs *onl_d* fs mff *robusddr_eqc/*vid(ine x]ltle mff *secur]le	innl_d* xlaeach_ (nla_len(t, frr =.nl_d*hronize_net();

	v_sock, Ndb_no6paf
	returf (__v!NIr(skb)->lse;
	inchron	/* Callbacnize_net();

	",
					   index, vni))
		return false	on	NEWNEmcsum(structvlse	on	NE	sy
	if ((vPACKET_HOST;

	v_snt F    vh-lags & 
		return false;

dex, vnlags & 
		return falsdex;
	int err int vxlan_rcv(snchrrotocol = f
	retate != stWNEIGEINVA.__vfr;
	f>k=}
BLED(ate != stWNEIGEINVA.__v>k=}
BLED(ize_net();

	v_strucp(steleasealsdex;
	)->same_fl(e != stWNEIG I (del_IFF_UP__be16 ptrucp(stebershipalse	atomic_ST;
ececlock(&vn->sEIG__vdrop pa)LED(ize_net();

	v_sret = = IP*/* ND_ptr(e != stWNEIG ret =ENABL64entrysno effex;
gin(&ntrysIGEyncpdst->rrysIG__vxEark =++st->rrysIG__vb *f;
+h_hdr(slenNABL64entrysno effexoid &ntrysIGEyncpdst
	ct vxellsjiffies;
ock(&vn-ct vxellscol = ht
ptrucp(stebershipalsnet = dev_netet(): vxlaClenumeely	+uter Id*hroan_rdst remote = )
{
	struct vxlan_rdkvn arpcp(durd;

	rd =ue = get_unalignexlan->cfg.flags & VXLAN_F_LEARN) &&
from %pIS to %pIS\n",
				    src_mac, &rdst->remotearpd)
{
ck_);

it()k_)ptr,s &haindex;
#ense {
	i->vx_vni, &ddr);
		nd*
	RCU_INIWNEIG I (del_IFF_returnpe *gpe = (stru(f)) {
		struc(rI>yI>kuearpc(strH wiock)) ruct vNEIGEINVA.t_vdrop pa++st-table */
stati	ck_)			arpc(strn falsdex;
	(ck_)ONFI_hrdhdr(remoteturHRD__rtER,
					    ck_)ONFI_hrdhdr(remoteturHRD_IEEE80sk_e_fdb    ck_)ONFI_no6hdr(remote_rtnl(IP_e_fdb    ck_)ONFI_ophdr(remoteturOPrn QUEST_e_fdb    ck_)ONFI_hl_hdr(vNEIGs) ==ot s_fdb    ck_)ONFI_nl_hdr(4npe *gpe = (st	k_)ptrol vit())ck_)	truct vxlan_rdstarpd)
dst->ha			arpptrst	k_)ptro+r(vNEIGs) ==ot ;lags)hur." Hturn rc&se {
k_)ptr,suct vxlaase(st	k_)ptro+r(uct vxlaasest	k_)ptro+r(vNEIGs) ==ot ;lagsthur." Hturn rc&te {
k_)ptr,suct vxltase(st u32 vxfv4in_u	intly(eltases_fdb    xfv4in_u0;

	if (ltase(pe *gpe = (stru				  r);u	inkup(&arpctbpa &te {
 &rdst u32 vn			union vxlan_addr *src_ipd->vx_flags &= ~	*e(rdyulation' s(iminud*addr, un
	ifCOTheCTED	}

	ret  r);uABLED(COnc, vni*gpe = (st	a_cachoff race between vxlan_fluily =
	}

ron	k,
			_clock_t(n;

	err = (_fdb_update(vxlan, ed = 0;
	ci.n_addr(mac)bridge-rshal&ddr);
	rSUPP;

  r);uABLED(COnc, vni*gpe = (st	a_cache(rdy			arpc"deleteturOPrn PLYh,
			P_tur,nse {
lignete {
)huourn 0ily =
	)hu)ruct vxr);uABLED(COnc, 
32))
			pe_t st;
	call_r*gpe = (stru  index, vni))
		retur		pe_c, vn__ate(srI>y		pe_,  vh-lags & 
t(skb);		pe_cruct vxpe_VNI__num
		+ nCHECK;
}
"TheCESSARYuct vxpe_VN	sy
	if ((vPACKET_HOST;

ly used by dex_	sy		pe_ct st;ET_RX_DROrnpe 	WNEIGEINVA.__vdrop pa++st-	",
			 __be32 src_vni"non-ECT from %pI3MISS) (tb[NDA_SRC_VNI]) {
	ipaart = vxlr) {
		if (vxlan_get_strn truelr) {
		i *)oiph)->tos);
d->vx_vni)ipv6_sodr)
{
	dr *)o/* o;ote_rcu(f)clenumest remote = )
{
	st);
DEV_TX_OKuct vlan_metadata _md;
	struct vxvh->vx_flags;

	if ((flags & nalan_fdb_head(vx
	if ((flin6.sin,vx_vni, &ddr);
		nd*
,ts */
isrECN_dtb[NDA_VNI]) !== get_unalig (stn6.sinructvlsehead(vx
	if ((flinrdyul2(skb, NDA_ter_desstroaul2(skb, N*head)
{
pip6;

it()d	     (lags a_oot sup8anhdropty(!v	trdrcnt;
			  stahgefdb_no6ags s_oot  bool vxaddr
	RCU_INIWNEt st;
	cl||  {
		struc(rI>yin6.sin,stn6.sinru_flagback from ;
	vh   ot supLLrn SERVED_SPACEiock)	truct vxlan_rdst*head)
)	tru  e;

	if a)	tr a_oot s+(vNEIGneede(strucroom = )
rdy			aan_fst re) {
	t(vni_field);
	offs	pe_t st;
	call_k from ;
	vh   vxpe_VN	d = first_remote_rtnl(uct v;  vxpe_VNlig (sctvlsehindex, gf y		pe_, LLrn SERVED_SPACEitn6.sinructv)psulate(sushy		pe_,  ct vxlan_rdstmil= NULL;  index, vni))
		retur		pe_c, ru	sol vxlan_seDA_ter_d)(ph)->saddtn6.sinp->donremotn_get_saddr);
	tn6.sinpeturn err ;ru	s_oot suptn6.sinru_fl -  vh-lags & 
t(skb);		6.sinp--ru  e;

	ian_rdst*head)
)	-ure;

	if =ENABlan_sf

	f  i <s s_oot -1  i +r((nsIGopt[i+1]<<3)test(&vs->resIGopt[i->doT|NUD_Roan_rdst re		pe_c, vne, src_vni, ndm= 1;
	}

esIGopt[id = NUDl ve_SOURCE_LLrtry |NUD_Rotn_get_sesIGopt->di	truct vxlan_rdstDA_opt_d)
dst-		bn_fkndm= 1;;

	spi  rdst->rsulate(oiphmily(vs) ==co rcaddr);
	tnpe_ceturdsin,sdxlan_fdbmily(vs) ==co rcaddr);
	tnpe_ceturn err <=ily =_fdbmilr);
	tnpe_ceturno6packeremote_rtnl(uct v;  vxpe_VN	d = first_remote_rtnl(uct v; ulate(suI>y		pe_,  ct vxlan_rdstmil= NULL;  index, vnlags & 
		retur		pe_c, vate(suty		pe_,  ct vxlan_rdst*head)
)truct sk_Pv6tic struct vxpip6)oiph)->sadd		pe_c, vo nla_ppip6failure;

	ian_rdst*head)
)truxpip6skb, eth_h= 6ruxpip6sk_maor]le	oiph)->sadd		6.sinpet_maor]leruxpip6sk_addhdta *mdPROTO_ICMct ruxpip6skhop__joina *255ruxpip6skddr *)oiph)->sadd		6.sinpet		     (pip6sksdr *)oi*EMSGSIZE implies_d)net_mamary_key; ulate(suI>y		pe_,  ct vxlan_rdst*head)
)trux index, vnc booskb-
		retur		pe_c, ruxlan_r);
	rSAdb, tiseturn oiphna    vh-an->xlany		pe_,  ct vxlf a)	tr a_oot truxnaVNIcmph.Icmp6
	if ((vNDISCu32 etBOURrtrVERTISEMev);
xnaVNIcmph.Icmp6
rECN_d)oipsrECN_d;
xnaVNIcmph.Icmp6
ob, rid ((v1;
xnaVNIcmph.Icmp6
so  &iped((v1;
xnaVNtahgefd_sesIGtahgeffdbmily(vs) ==co rc&naVNopt[2]<=ily =_fdbnaVNopt[0]  NUDl ve_TARGET_LLrtry fdbnaVNopt[1]  N a_oot s>> 3, ru	aVNIcmph.Icmp6
ck hea= each_eh)->magic(&pip6sksdr *d->v&pip6skddr *,  ct vxlf a)+ a_oot ,*mdPROTO_ICMct d->vfo.optl_rcsu(na,  ct vxlf a)+ a_oot ,*0)tructpip6sk_ay))
	not supremote ct vxlf a)+ a_oot v; ulate(sushy		pe_,  ct vxlan_rdst*head)
)tructvxpe_VNI__num
		+ nCHECK;
}
"TheCESSARYucct sk_buff rdyult vxlan_rdkvn vxr);uABdurd;

	rd =ue = get_unalignexlan->cfg.flags & VXLAN_F_LEARN) &&
from %pIS to %pIS\n",
				    src_mac, &rdst-atelimMSGSIZE implies_dd	     (atelimMSGSIZE head)
{
	}

	ul2(skb, N*   6 %pIS\ impctvlsehead(vxddr);
		nd*
	R2(skb, NDA_ter_dter	RCU_impctvkb, v_impctvdr t, &rdst-vs->r_impctvnpe *gpe = (stru(ph *)oiph)->saddr,
	;motn_get_s&(ph *skddr *, voer_l vxlan_seDA_ter_d)(ph(!v	tr1(st u32 vxfvRTM_NEW	intly(eld	vxlan_fdb    xfvRTM_NEWn_u0;

	if (l&terIGtahgefe(pe *gpe = (stru				  r);u	inkup(&= VXLAN_VNonizbpa &terIGtahgef{
 &rdst u32 vn			union vxlan_addr *src_ipd->vx_flags &= ~ *e(rdyulation' s(iminud*addr, un
	ifCOTheCTED	}

	ret  r);uABLED(COnc, vni*gpe = (st	a_cachoff race between vxlan_fluily =
	}

ron	k,
			_clock_t(n;

	err = (_fdb_update(vxlan, ed = 0;
	ci.n_addr(mac)bridge-rshal&ddr);
	rSUPP;

  r);uABLED(COnc, vni*gpe = (st	a_cache(rdy			ags & nalan_fdb_h*fdb)ourn 0;!!		_? fIG I (del_rencROUTER :*0)truct vxr);uABLED(COnc, 
32))
			pe_t st;
	call_r*gpe = (stru  used by dex_	sy		pe_ct st;ET_RX_DROrnpe 	WNEIGEINVA.__vdrop pa++stt-	",
			 __be32 src_vni"non-ECT from %pI3MISS) (tb[NDA_SRC_VNI]) {
	ipaart = vxlr) uct iphdr *)oiterIGtahgef{= vxlr) uct iph *)oiph)->tos);
	d->vx_vni)ipv6_sodr)
{
	dr *)o/* o;ote__rcu(f)clenumest remote = )
{
	st);
DEV_TX_OKuct ons(ETH_ packets */
rECN__f IS_cixlait;

	rd =ue = get_unalignexlan->cfg.flags & V) &&
from %pIS to %pIS\n",
				    src_mac, &rdst->remoteddr);
		nd*
	RCU_INIrd);
	}
	kfree(f);
}

staddr);
	}
	returdsinllback from net/ipv4/				urn -EOPwin_ad(r_gpe addr);
	}
	returno6paf) (tb	kfe _rtnl(uc(f)	union vxla	}

	rep->sa.sa(f)) {
		struc(rI>yI>kue ct vxlan_rdst*hd)
)t	 vni,
					 vxlan-	tpipt_sk_family(vsn-	t				  r);u	inkup(&arpctbpa &pipskddr *,  &rdst-&vs->repp, fl32 src_vni"non-ECT from %pI3MISS)_addr(mNDA_SRC_VNI]) {
	ipaart = vxxlr) {
		if (vxlan_get_spipskddr *,= vxxlr) {
		i *)oiph)->tos);
d->vvx_vni))ipv6_sodr)
{
	dr *)o/* o;otni,
					 vxlan-	t_cachbn_fkndm}xlan_metadata _md;
	struct vxl	kfe _rtnl(ucV6(f)	union vxla	}ead)
{
pip6;
.sa(f)) {
		struc(rI>yI>kue ct vxlan_rdst*head)
)t	 vni,
					 vxlan-	tpip6)oiph)->saddr,
	;mou				  r);u	inkup(&= VXLAN_VNonizbpa &pip6skddr *,  &rdst-&vs->repp, fl32 src_vni"non-ECT from %pI3MISS)_addr(mNDA_SRC_VNI]) {
	ipaart = vxxlr) uct iphdr *)oipip6skddr *,= vxxlr) uct iph *)oiph)->tos);
	d->vvx_vni))ipv6_sodr)
{
	dr *)o/* o;otni,
					 vxlan-	t_cachbn_fkndm}xlse
			nct vxla:back from net/ipvGH);

	hli_addr(b_parsif_ip, csif_h)-!mily(vs) == AF_INET)
		err = IP_Edsin,sily =_fdbU_INIWif__addr(mturn rcaddr);
	}
	return err <=ET)
		err = IP_Edsin,= vxxvNEIGs) ==ot o;otniturn rcaddr);
	}
	returdsin,sily =, vNEIGs) ==ot o;otn}ct vxr);uABLED(COnc, ack from sif_ipvxlan_sock *sock6 = rtnl_dereif (!(vxflabuildVXLAN_F_COLLECT_METADATA))vxharsed->vx_flags &= ~VXLAN_GBP_USED_BITS;
}

static bool vxlan_parse_gpe_hstru(f)) the paclback fromst
	c_hdr(struct vxlanhdr *unparsvxh;}

sdb *f;

	f = the O bilanhdrstru(f))/* "When&cket.
	 */
	if (gpe->(pe *ndicated it MU xlanhdr(f))/* "When&cket.
	 */
 set to 1, the(pe *ndic
		return fals xlanhdr*)unparsed;

	supremote/* "When&cket.
	 */
ID)
{
	)b[NDA_PORT]) != sizeofbuildVXnext_protocol);
	if (!*pvxharsed->vx_flags &= g *ndm, structdr;
		sadatic bool vxlan_parsatic bool vxlan_set_mac(struct vxvxh;}dr*	union vxlan_a((v1;
xddr.s_addr = ip_hd
		saddr.rr =n.sin_a = ip_hdr(skb)->sddr.s_addr = ip_hdr ack from -EP	}

	/* remo )
{
	struct vxlan_rdkvn sizeofbuildVt rem 1;
			}
		}
	}
out:
	returd);
		ret{
ddated = jiol vxp(strH wfy |= vxlan_fdb_repl~VXLAN_GBP_USED_BITS;
}

arsed->vx_flags &= g b_park(sknumadatic bool vxlan_pa*pvxh bool vm		i		reroom = truct vxl truc	if ((vk(sknum_? SKB_GSO_
/* Update p;
} :*SKB_GSO_
/* Update;NULL)
			state,rd = first_remote_rtnl(f);
		_info_rat, &grc);

	return pp;
}
TX,
					   _u16 s__num
		+  nCHECK;
}
policy_aest(&vrucfo.op>gbp = n
		scheck hep>gbp NAPI_GRO_CB(p)dbU_INIfo.op>gbp =<the O biMAXrn pp;
}
SToli & VXLAN_F!Ifo.op>gbp =);

	retRCO_SHIFT)
{
	));
		kfree(_u16 N_HF_GBPct me=
		md->vxlan_rdstk(sh *, checkan_fdb_notif_u16 N_HF_GBPct me=
		md->vxlan_rdsttcsh *, checkat	 vni	if ( thSKB_GSO_Update n pp;
}ipvxlanm		i		reroomsupLLrn SERVED_SPACEioinructv)s+(vinruvni != H w vni+e;dr	= i Di->dip(strH w = vxlan_finsperenlan_ devvni !=sNIr_deref16 sat F ptr_get_sdata *
		scow	notif
out:m		i		reroomr(skb)->);
	bool raw_b->args[2])
					gdata *ip);
#if handle_GBP))
	ol vxla	if r(skb)->< cb->args[2])
					gvxhkb, vate(sushyI>kue ct vxlpvxh)trux
sdb *f;

	f == 

	if (__vxlan_
sdb *f;an, f, rd, RTM_
		vh () {
		tun_ds	if (&hSKB_GSO_Update n pp;
}) (tb[NDbe dropped.>gbp stru  gbp = n
		scheck hep>gbp NAPI_GRO_CB(	-ure;

	ic bool vxlan_pac, ac
sdb *f;an, , port, s(strue(vxlo(learn)
_u16 N_HF_GBPct c, ac
sdb *f;

	f = the O bilanLIED;t-&vs->r
		srd)gs&
	  )_addr(m_u16 s__num
		+ nCHECK;
}
NONE;dr(m_u16  ifindex, __u gs = nd			return -at, &grc);

	retur>BP
			   __bbuildVXLAN_F_Cvxhar>vx_flag 
	 * vrn -at, &grc);

	retur>Aif
	retamily == AF_buildVXnext_prvxhar>vx_flag ;

	return tr_fdbU_INIf (likely(f)remote_vni;
		state,rd = first_;

	return tr; eth_gro_csf vxlate,rd = firyI>kuexlate,rd = fire = )
{
	struct vxlan_rdc bool r 0)
			t.)addr t.rECN_p->sa.sa_family = AF_INET;
#if ISENT | NUD_NOARP))
		= g *nd
	if (!(unparsed->vx_fla))
		= g *nd
	if (! *mac, union vxlanoif,mit(tos))
		= g *nd |= vxlddr *, dst_port;dr *, dst_			dt nlattr *) ||= 1)
		ret g *nd
	if (!INET) {
	{
ddaT) {
	
		ret g *ndatelimMSGSIZE hk);
#if xlanS\ ifoadatib_parkseT) {
	{(risk);
#if te->sa.sa_us0)
	yI>kuexlivpar	c bool r 0)
			p = nurn -EOPNOTSUP an i4P a4stru(f)) s->hlist))
{
	stERR_PTR(-EIO
		tun_ds	osvid); ifoadb[NseT) {
	{(rn->vn4_sock,NseT) {
	be16 ptk_bh(e->sa.sa_r t.ip4(ddaT) {
	
 vxlan_fdb2))
		tly(f)remote_0;
	}

	ro nla_p& a4failure;

	i a4)trux a4. an i4_oif_bhoifrux a4. an i4_	osv= RT_TOS(;
}

/* a4. an i4_s drop , thus dr
/* a4. an i4_no6packemdPROTO_
/*
/* a4.tn_get_sddr *, v a4.sdr *)oi*sdr *, v a4. a4_d
	} elsdead[h], a4. a4_s
	} elssead[h]
ptk_bhiskrECN__ECNan->keyCatelimin_fdb& a4 * vrn ->same_flmetaRR		tl)test(&vs->rtF_INE.WNEt stctv)s
	ret  an_sock * = rtncixlalar
rECN__used(s4 unio&dxlan_fdb(&vskrt(suty	to;otni,
					ERR_PTR(-ELOOP)n-	t_cach*sdr *)oi a4.sdr *fdb2))
	NseT) {
	bpe 	We->sa.sa_s t.ip4(ddaT) {
	
 &rtF_INE,i a4.sdr *

	unsigned sho  an_sock * = rtnno
rECN__used(s4 unio&dxlan_fdb(,
					ERR_PTR(-E);
UN (fam_fdb *f = contrtuct vlan_metadata _md;
	struct vxvh->vx_flags;
d);
		ret{
ateli6dr t.rECN_p->sa.sa_family = AF_INET
	)
{
	struct pENT | NUD_NOARP))
		= = AF_INET6;
#endif
	}flags))
		= = AF_INET *mac, union vxlanoif,mit(tos))
		=	nd |= vxllabel))
		=	ndatelimMSGSIZE implies_dd	   ))
		= = AF_INET implies_d	release_sock(dst_			dt nlattr *) ||= 1)
		ret	nd
	if (!INET) {
	{
ddaT) {
	
		ret	ndatelimMSGSIZE hk);
#if xlanS\ ifoadatib_parkseT) {
	{(risk);
#if te->sa.sa_us0)
	yI>kuexlivpar	c bool d);
		ret{
n sk_bOPNOTSUP an iflag6stru(f)) s->h6ist))
{
	stERR_PTR(-EIO
		tun_ds	osvid); ifoadb[NseT) {
	{(rn->vn4_sock,NseT) {
	be16 pno have(e->sa.sa_r t.ip6(ddaT) {
	
 vxlan_fdb2))
	n */
 0;
)
{
	stn sk_bO

	ro nla_p& a6failure;

	i a6)trux a6. an i6_oif_bhoifrux a6.tn_get_s*ddr *, v a6.sdr *)oi*sdr *, v a6. an label{(ris->make_ an ie byRT_TOS(;
}
,llabeltrux a6. an i6_s drop , thus dr
/* a6. an i6_no6packemdPROTO_
/*
/* a6. a6_d
	} elsdead[h], a6. a6_s
	} elssead[h]
pno have&= VXLAN_VNI_MASddaT	inkup_ an Catelimin_fdbflags &= ~VXLANourn 0;Fruct  & a6faence(vxlb)->);
	bool metaRR	n */
truct vxlan_sock * = rtnno
rECN__used(sretuo dxlan_fdb(,
					ERR_PTR(-E);
UN (fam_fdb *xlb)->);
	bool noinructvt stctv)ruct vxlan_sock * = rtncixlalar
rECN__used(sretuo dxlan_fdb(db_updLED(COn */

	ui,
					ERR_PTR(-ELOOP)n-	 *xl*sdr *)oi a6.sdr *fdb))
	NseT) {
	bpe We->sa.sa_s t.ip6(ddaT) {
	
 nINE,ivxlan_fdb)
{
	stn sk_bt ons(ETH_xlaBypass  ifindex, __u if IP6h(&vxlan->hashotifyal md
#endif
if (!(vxfla ifin_bypassem 1;
			}
		}
	}
out:
	retur_family = Aportia_family = g *nd
	if (!(unparg(vxladaTF_INET;AN_F_LEARN) &&
from %pndex;
	int ret = -txentrys,		pxentrys;&
	    vxlan_snoop(	intly(e;&
	    vxlan_snoop(*lan;
	struct&adaTF_INE
	return ret;
}

static botruct pENT | NUD_NOARPxl trucot suppdr(slenNA
	txentrys = IP*/* ND_ptr(portia_fastWNEIG ret =ENABrxentrys = IP*/* ND_ptr(adaTF_INE
	reEIG ret =ENABon	NE	sy
	if ((vPACKET_HOST;

_u16  ifindex, __u gs = non	NEWNEmcsadaTF_INE
	reE;NULLate(suI>y
out:
vh-lags & 
t(skb);	  )_		tun_ds}

staticlags)
{
	size_t start, offset;
	intly(elr) {
		if (vxlan_get_sC_VNI]));
	} LOOPBACK

	ui	intly(elriphdr *)oiph)-imited("non-ECT from %pI4 with TOS=%#x\n",
					  	intly(elr) t net_device *tdev;

		intly(e;&
i	intly(elriphdr *)oiph)-imited("		else
			net_itrucp(steleaseals	WNEmcson	NEWNEvxlb)->);
	bool !IWNEIG I (del_IFF_UP__be16 pan_rdst remote = nize_net();

	v_sock,adaTF_INE
	ted("non-ECT from %pI6\n",			   __btruct dr *)o	intly(e>saddr);
	}
	return err <=0
	}

ronABL64entrysno effex;
gin(&txentrysIGEyncpdst-txentrysIGt_vxEark =++st-txentrysIGt_vb *f;
+h_lenNABL64entrysno effexoid &txentrysIGEyncpdst
  used by dexO_CB(	 st;ET_RX_SUCCESS) (tb[N64entrysno effex;
gin(&rxentrysIGEyncpdst-	rxentrysIG__vxEark =++st-	rxentrysIG__vb *f;
+h_lenNABBL64entrysno effexoid &rxentrysIGEyncpdst-n",
					et(): v	WNEIGEINVA.__vdrop pa++st-	
ptrucp(stebershipalst vxlan_rdkvn  ifin_bypass_y difyal+= 1;
			}
		}
	}
out:
	return e | NUD_NOARP))
		= ->sa.sa_family = AF_INET
	)
{RT]);
	} else {
		*d	   ))
		=(dst_			d) == 1)
dkvn d) =,
				    "%pM migr))
		= ->sa.sad);
		ret{
ddated =	rsed-rt(D_BITS;
	n-ECT from %pI4 with TOS=%#x\ sk_Pv6trtF"non-Esrc_checked ag &vtt RT%pIpin_, buerr = valuk);
signkRT%pIpin_shot AF_I_useRTC%pIpin_. So_usekeeppcet ure		resignkwebcanse;
#RTC%pIpin_ which s & s		  stpv4
	/* Tpv6trECN__		retaddr);
	BUILD_BUG_ON(RTC%pIpin_ !=kRT%pIpin_)	else
			nxlaBypass  ifindex, __u if IP6h(&vxlan->hashotifyal md
&vs->rtpacb_no6ERTC%pIpin_ 					   !>rtpacb_no6E(RTC%pBROADCAS);
	RTC%pn, srCAS))	if (netif_running(dg(vxladaTF_INEip, csb_updLED(CO */

	uiadaTF_INEff race betweeTM_NEWNEImin_fdb() =,
				   	sock6 = rt  dxlanlags)
{
	size_,	d) == 1)
k6 = rt  F_INE
	ted("non-dst-&vs->radaTF_INE|NUD_RotNEIGEINVA.t_v>k=}
B++st-	pan_rdst remote = 0;
)
{
	stiv(dev);
tn}ct (vxfla ifin_bypassem    (ipv6_aadaTF_INET;}

ron	k)
{
	st1ipvxlan_sock *0 = rtnl_dereif (!(vxflaxmit_onb_head(vx
	if ((fl
out:
	return e | NUD_NOARP))
		*nd |= vxldeturn r					tif_running(dev))
			vxgs &= g b_pardid_rsc) &&
from %pINET) {
	{
ddaT) {
	;l2(skb, N*hk);
#if xlanS\ ifo;&
from %pIS to %pIS\n",
				    src_mac, &rdst-atelimMSGSIZE }

	reold_t F   k_family(vsn-	T]);
	} else {
		*dsk_bOT]);
	} else {
					 NLM_F_Eifyaltic botruct pyI>Pun_~ddr_equal(eth_hdr(skb of vxlan_igmp]3_ (rd == NULL)
			port
	} els0,	d) == 1)ar	c bool d);
		ret{
n sk= nurn -EO "%pM migr)llabel-EO "it(tos) ttlxl trucst) {
			struct vxlv, &s

	f == F_INE
	ted("non-;tib_park(sknum_(rn->vn4_sb_parxock_bhXLAN_F_Catelimin_fdb(&vxlan->hash_lock))	RCU_ifo= n
		s);
#if xlanrn falsdex;
	r */
	if ( sk= nruct vxlan_dev *st-&vs->ck_t(n;

	err = */
tX]) {
	INIWid_rsc)t = vxxags)h 1)-cixlait conae pemotfyal bridgeSUPP;

 (vxfla ifin_bypassem    (ipv6_a(ipv6_aaeturn r			o;otnick fromst	tn}ct 
			coet();

	}p, csb_u
	} elsuct vxlan_dev
	} e?suct vxlan_dev
	} e: F_INE
	ted(d) == 1)ar	lan, f,(uct vxlan_dev_pr u?e: aeturn r			st-&vsdr, eth_
					 vxlan->cfg.ds;&
i	iyaltic== F_INE
	ted(sdr *fdb2INET) {
	{ nruct vxddaT) {
	;l2uct vxlanhd, thus dr
/*	ttl== F_INE
	ted(ttlxl ion' sotl=clock_t(n;

	e0;

	if (ldinllbac	ttl== 1_bufeeo == F_INE
	ted(eo xl ion' eo ==s)
{
	seeo == *hk);
#if r t.ds		vh (old_t Fcol = ht
psock,adalags)
{
	size_t start, off
	sek(sknum_(r!("non-ECT from %p
/* ZERO_p;
}
TX,xl iInverse k(sknum_(r!("non-ECT from %p
/* ZERO_p;
}6
TX,xl ilabel{(rF_INE
	ted(label-EOn",
					  vs->r_ifaf
	ret	W\n"_ONCE(1, "%s: Mikb)->h ifindex, __u inc bool_lenruct vxlan_vNEIGnamec, vni*gpe et();

	}pick f, &ip, port, vni, if(risk);
#if xlan_af(xlivpar	un_ds}

static.gs)
{
	size_t start, offset;
ck f, &ip, p) {
		if (vxlan_get_skliv->retuu.tpv4.dsk_bO
i	iyaltic p) {
		if (vxlan_get_skliv->retuu.tpv4.por;

	}",
					  ck f, &ip, p) t net_device *tdiv->retuu.tpv6.dsk_bO
i	iyaltic p) t net_device *tdiv->retuu.tpv6.por;

	}f ( sk= nruan_dev *st-&sb_u
	} elstdiv->retutpto ha?e: F_INE
	ted(d) == 1)ar	lan, f,);
#if id) /*keyr)(tdiv->retut recd)st-&vsdr, eth_0fdb2INET) {
	{ nrtdiv->ddaT) {
	;l2u_INIrdiv->r does not tX]) {
	INIrdiv->r does not  <ure;

	if
	 s vxla*gpe et();

		3_ (risk);
#if xlan_opts(xlivpar	u}/*	ttl== tdiv->retuttlxl ieo == *div->retuto xl ilabel{(ridiv->retulabel-EO k(sknum_(r!!(tdiv->retut re"non-ECTUpdate p;
}_fdb *fport
	} elsk(sk an _port
	} ((&vxlan-ock)hron	/* _INE
	ted(skb-
min
		ret g *n _INE
	ted(skb-
maxsk_buff *sitrucp(steleaseals	ock,adalags)
{
	size_t start, offset;

	if (!(unparsed->vx_flags &ruct vx]FI>gxl|| skb->remcsum_offload)
		gr 0)
			p fflodst_			df 
	f = v	p = ntohs(gr t.rECN_p(ipv6_aaevdbflag4hron	/*,
				   tos))
		= g *nadalagi);
	offset = start + v;Fruct 	iyaltic p) {
		if (vxlan_ge))
		= g *nada_>skb).port= 1)
		ret g *nddaT) {
	
 xlivpar	un_dsmetaRR		tl)X]) {
amily PTRtaRR		tl, vni*gpe t_v>k=}
;

	}p, cxlaBypass  ifindex, __u if IP6h(&vxlan->hashotifyal md
& vs->r_ifaf
	ret	amily  ifin_bypass_y difyal+=n	/*aevdb(ipv6_aadavxlan_find_md) == 1)
dk
				   	sock6 = rind_m&rtF_INE,irtF_rt(D_BITS;) {
	INI< cb->a_r*gpe = (ebershi;

	}",
				INIrdiv->retut re"non-ECTUpdate 	if (FRAGMev)|NUD_Rotfst_remoteIP_DF)n-	t_cachn sk= nrutF_INEar	un_dssnterfaces itX]) {
	l vmtumcsadaTmtuOn */
 -he O bilEADROOM = 0;
->rn erro effexpmtuO
out:mtu)n-	t_cacheo == *hk);
#if f]FI>ifinyRT_TOS(;
}
,lold_t Fcol = ht*	ttl== ttl=?e: ip4n errhopoup isrutF_INE,xl iImily == AF_buildVt remot
 nINE,ivct vxlan_rdst*hd)
)
		ret g *ndigr)l

arx_flag k(sknumaar	un_dsf (likely(f)*gpe t_v>k=}
;

ock(sk);
#if xmit_t re	fdbflagpskb_may_puhron	/*	iyaltic p) {
		if (vxlan_ge))
		= g *adalagi);
	offset = start(tos) ttl_aaft vxlan_net *v= 1)
dd) == 1)
dxn_fdb!k(sknumaarid *)(vxlan_hdr(skb) + 1), start, offset,
			    !!(vxflags & VXLAN_ruct vx]FI>gxl|| skb->ret:
	unparsedn sk= nateli6dr t.rECN_p(ipv6_aaevdbflag6hron	/*,
				   tos))
		=ilabel,t&adaparse_gbp_hdr(st))
		=i&	iyaltic p) t net_devic))
		=iada_>skb).port= 1)
		ret	ddaT) {
	
 xlivpar	un_dsmetaRR	n */
tX]) {
amily PTRtaRR	n */

	ui	n sk= nurn -EOni*gpe t_v>k=}
;

	}p, cvs->r_ifaf
	ret	sed-rt6i;

	f == rcu_head rt6 xlanS\)n */
F_rt6i;

	f  = 0;
amily  ifin_bypass_y difyal+=n	/*aevdb(ipv6_aadavxlan_find_md) == 1)
dk
				   	sock6 = rind_mnINE,irt6i;

	f S;) {
	INI< cb->a_r*gpe = (ebershi;

	}
r	un_dssnterfaces itX]) {
	l vmtumcsadaTmtuOn */
 -he O b6ilEADROOM = 0;
->rn erro effexpmtuO
out:mtu)n-	t_cacheo == *hk);
#if f]FI>ifinyRT_TOS(;
}
,lold_t Fcol = ht*	ttl== ttl=?e: ip6n errhopoup isn */

	uiro_cscrubvxEark O
out:xn_f,xl iImily == AF_buildVt remot
 nINE,ivct vxlan_rdst*head)
)
		ret g *ndigr)l

arx_flag k(sknumaar	un_dsf (likely(f)*gpe t_v>k=}
;

ock(sk);
#if6 xmit_t renINE,ivlags &= ~VXLANo =n	/*aevd + v;Fruct 	iyaltic p) t net_devic))
		=Fruct adaparse_gbp_hdr(st)(tos) ttl_)
		=Fructlabel,tt *v= 1)
dd) == 1)
d!k(sknumaarise
			net= (ebershi:
ptrucp(stebershipalsck fromst
et(): vvNEIGEINVA.t_vdrop pa++st-(&vxan_rdst remote = )
{
	sst
t_v>k=}
:
ptrucp(stebershipalscn_dsf (l=_bufLOOP)
RotNEIGEINVA.unplia_len++st-,
				INIf (l=_buf);
UN (fam_
RotNEIGEINVA.t_v * prorv>k=}
B++st-db_updLED(COn */

	utNEIGEINVA.t_v>k=}
B++st-an_rdst remote =ead, sT boooinaifyal NEark = ob,  V= AF
 *6->sOCN_decapsulate(inheritbuECN
	/* DFlan_fdxlatepsulate.6->sOCN_ded_ma(&vxlan->hashotIP6h1rr = asse drop= 1).6->sssssssssssn err p= 1)k_socM pachas == );

 an  = ipv6_stub    srct_vt!(vxflaxmit_head(vx
	if ((fl
out:
	return e | NUD_NOARP) &&
from %pIS to %pIS\n",
				    src_mac, &rdst->remotening(dev))
			vxg *f sk= nurn -EOatelimMSGSIZE hk);
#if xlanS\ ifo4_sb_parWid_rsc_(rn->vn4_son vxlan_addr *src_ipdan_rdstmil= N *mil;ndex;
#endif

	f = v_ifo= n
		s);
#if xlanrn falsde index, vni))
		return falsdex;
	F_INE
	ted("non-ECT from %pnt OAM
	 * proces			  vs->_ifo=&&ridiv->, __el_I* Update INFO_BRIDGE);
		kfreeidiv->, __el_I* Update INFO_TX,
]) {
an, f,);
#if id) /*keyr)(tdiv->retut recd)st-&}",
					  cvs->_ifo=&&ridiv->, __el_I* Update INFO_TXb->a_r(vxflaxmit_onb_hn	/*aevdb(gr)lurn ,rn->vnS;) {
Inverse pan_rdst remote = n )
{
	st);
DEV_TX_OKucnd			return -atINE
	ted("non-ECT from %pPROXYf
	retatF   addr);
	}
	rar	un_dsr_gpe addeturno6pafl=_b
			P_tur
 0;
)
{
	starpcp(durd;aevdbfn	/* Calnnel_notify_del_rx_port(vs->sock,
				INIr_gpe addeturno6pafl=_b
			P_vs->);
		k	 {
		struc(rI>yI>kue ct vxlan_rdst*head)
)	tru xlan_netct vxlan_rdstDA_msg)));
		k	 ph)->saddr,
			_addhdta  *mdPROTO_ICMct _addr(m_skb, NDA_ter_dt l vxlan_seDA_ter_d)(ph)->saddsgbp->don = 0;
(f))/VNIcmph.Icmp6
c __e  *0);
		k	    mVNIcmph.Icmp6
	if (((vNDISCu32 etBOURrSset ITATIONb->a_r)
{
	stnxr);uABdurd;aevdbfn	/* Calnn	m}xlse
			nretuatF   addr);
	}
	rar	off race between vxlan_fluaddeturdsin,s Calnn	Wid_rsc_(rn->vn4_
k,
			_clo(fIG I (del_rencROUTER)pp, fl32 src_vni"non-ECT from %pRSC,
					   Ir_gpe addeturno6pafl=_b
			P_vsn_fdb     r_gpe addeturno6pafl=_b
			P_vs->) ruct vid_rsc_(rrECN__f IS_cixlait;aevdbfn	_fdbU_INIWid_rsc) 	choff race between vxlan_fluaddeturdsin,s Calnn	}_
k,
			_ st;
	caruct off race between vxlan_fluall>xlansen v
	}

ron	k,
			_ st;
	caruct info_ratINE
	ted("non-ECT from %pI2MISS) ;
		k	    !rd);
	}
	kfree(f);
}

staddeturdsinllbac_r(vxfla *sr)
{
	lan_fluaddeturdsinn = 0;
vNEIGEINVA.t_vdrop pa++st-tpan_rdst remote = n )
{
	st);
DEV_TX_OKucnd			retuif (!vs)
		return fvxlan		vxg &f		 vxlans)  IS_Effset,
			  
	if ((fl
ou1D;t-&vs->rf */
	if (	f sk= nrdsk_bO
iBLED(CONFI	u}/*	
ou1= n
		sclonb_hn	/*t(vni_field);
un_dssnt
{
	se(vxflaxmit_onb_hn	1/*aevdb(gr)l		vxg Wid_rsc)nn	}_
k,
			 */
 0;(vxflaxmit_onb_hn	/*aevdb(gr)lf	vxg Wid_rsc)nn	Inversean_rdst remote = )
{
	st);
DEV_TX_OKuct v, sWalktIP6hvs)rr)crr < 0)
		r_eqcurge.>gbl__		reies md
#endif
if (!(vxflacLEDnup(NDbe dropST;
targ) &&
from %pIS to %pIS\n",
				p->sa.sa_family = A)targxlv,Dbe dropST;
t_addrtim_d)oijiffies + FDB_AGE_k *so*skb,[NDbe dropped.hstru(f)) d by de;
#rr ->hash_lock))
ack fromst
	lan_sh

	f  hlikFDB_HASH_SIZE; ++hEffset,
			  free(sn __e*p,d*
	RCUock_releas_bh
ock(&vn- == ock);
		kkfree(svs)
		retsafe(pdb)odif
	}

	 *sr		re[h]_addr(m_skb, Nn_addr *src_bac_r=date= &v);
vxlp		tif_running(d *s, f IS_ENAB	v,Dbe dropST;
ttim_= (stru k,
			IGEINV_el_(
	ifPERMANEN);
	
	ifreturnlbac_rBLED(CONFIru k,
			IG I (del_rencEX (gpe->he(pe _rBLED(CONFIru ktim_= (_(rn->U= co+n _INE
	ted(age_tructval m HZ;) {
	INItim_x;
vs)e_F_Ctim_= (,ijiffiesitX]) {
vxlan_sock *>hash_lock
k6 = rt  "garbage unplec  %pMetuourn 0;Frun->addrxlan_fdb(&		IGEINV_e=	
	ifSTALE;dr(mr(vxfla *srdsinroy	lan_flufsk_buff *	
	}",
				INItim_x;
vs)eCtim_= (,i_addrtim_dnlbac_r_addrtim_d)oitim_= (st	u}/*	
k_rebershi_bh
ock(&vn- == ock);
		k

	roodrtim_d
ock(&vn-age_tim_d,i_addrtim_dn = rtnl_dereif (!(vxflamreqn mreq from %pIS to %pIS\n",
	) &&
from %pIS to ock_*v				   _gev);icCatelimin_fdbIS to ock

	/* Neck_releas
ocn &= ~Vock);
		kfree(sdif xlit_xlanock(&vn- ree(4.f IS_ENAel_notify_del_rx_port(vs->socfree(sdif xlit_xlanock(&vn- ree(6.f IS_ENAese
			n
k_rebershi
ocn &= ~Vock);
		 rtnl_dereif (!(vxflamrexlamreq from %pIS to k, struct ->sa.sa_family = AF_INET
	)
g *nd
	if (!(unparg(vsn __e*n __) &&
from %pIS to ock_*v				   _gev);icCatelimin_fdbIS to ock

	/* dex;
#endif

	F_INE
	return ret;
}

stat			st
	n __b *fINEff race ;Neck_releas
ocn &= ~Vock);
		kfree(sxlam		re_xlanon __b free(db(gr	notifuct saddr t.
k_rebershi
ocn &= ~Vock);
		 rt, sS frp ntrys ck->s| NUD_NM
	 n_fdbdt = ipv6_stub->ipv6_sonit;

	rd =ue = get_unalig) &&
from %pIS to %pIS\n",
				    src_mac, &rdst-truct vxl	utNEIGtntrys =     srcaan_fsndex;
trys;

	rd =ndex;
	int ret =alscn_ds!reEIG ret =E
;
)
{
	stiv(dMEM			gdata *ct vxellsjonit;ock(&vn-ct vxellsco &rdst-vs->raw_puct o_rdsperdex(reEIG ret =ENAB)remote_vni;
	xlan_sock *0 = rtnl_dereif (!(vxfla *srdsletatreturn p->sa.sa_family = AF_INET;AN_F_LEARN) &&
from %pIS to  *src_ip
ock_releas_bh
ock(&vn- == ock);
		koff __race between vxlan_fluall>xlansen v
	}

ron	,
			
 0;(vxfla *srdsinroy	lan_flufsk_buff *	
k_rebershi_bh
ock(&vn- == ock);
		 rtnl_dereif (!(vxflaunonit;

	rd =ue = get_unalig) &&
from %pIS to %pIS\n",
				    src_mac, &rdst
	ct vxellsjdsinroy	ock(&vn-ct vxells);
}

stata *srdsletatreturn p(ipv6_a(ipv6
	ted(}

ronABo_rdsperdex(reEIG ret =ENA rt, sSgbp =agei;
ttim_r	r_eqjoin  fals ck->s| NUD_NM
	bfalght ls  = ipv6_stub->ipv6_sopen;

	rd =ue = get_unalig) &&
from %pIS to %pIS\n",
				    src_mac, &rdst-trucrt metax_vni IS to k, ssxlap(ipv6ron	,
		x_vn<(skb), ETH_HLrt metadatack_t(n;

	e0;

	if (l&F_INE
	return ret;
}

statci.ne16 pt_vni IS to igmp_joinp(ipv6ron		,
		x_vn=_buf;
	}INUSE
 0;
)
{th_0fdb2,
		x_v,
]) {
aS to k, sspdLED(CO(ipv6ron		, ETH_HLrt mend			return -atINE
	ted(age_tructval
 0;oodrtim_d
ock(&vn-age_tim_d,ijiffies + FDB_AGE_k *so*sk)ucct sk_buff tNA rt, sPurge.IP6hvs)rr)crr < 0)
		md
#endif
if (!(vxflaflushyI>sa.sa_family = AF_INET;b_parWocaan) &&
	 be dropped.hstruck_releas_bh
ock(&vn- == ock);
		koan_sh

	f  hlikFDB_HASH_SIZE; ++hEffset,
			  free(sn __e*p,d*
	Rkkfree(svs)
		retsafe(pdb)odif
	}

	 *sr		re[h]_addr(m_skb, Nn_addr *src_bac_r=date= &v);
vxlp		tif_running(d *s, f IS_ENAB	vn_ds!rocaan_clo(fIGEINV_el_(
	ifPERMANEN);
	
	ifreturnl(pe _rBLED(CONFI		lagstheuall>xlansen v_		retNM
	dsletad at!(vxflaunonitSUPP;

vs->r_s>xlanee(f);
}

stn->addrxlan_lbac_r(vxfla *srdsinroy	lan_flufsk_buff *	
}db *fpk_rebershi_bh
ock(&vn- == ock);
		 rtxlaCLEDnupttim_r	r_eqvs)rr)crr < 0)
		on shutdown  = ipv6_stub->ipv6_sstct i	return e | NUD_NOARP) &&
from %pIS to %pIS\n",
				    src_mac, &rdst->remotening(dock_*v				   _gev);icCatelimin_fdbIS to ock

	/* dtrucrt  struct vxlan_dev ;

	e0;

	if (l&F_INE
	return ret;
}

statci. 					   !tohs(grfals>d#F () dbIS to))
ack fni IS to igmp_LEDvCO(ipv6ron	utNl_tim_d_Eync
ock(&vn-age_tim_d);
}

stata lushylan_fluf->vnS;) aS to k, sspdLED(CO(ipv6ronct sk_buff tNA rt, sStut
 nothi;
t_ae{
	strun_add_be32 #endif
if (!(vxfla, vni
	}
	kfre IS_ i	return e | NUD_NOARP) &&t vxlan_rdkvn sizeofchange_mtuO

	rd =ue = get_unalignekvn vxw_mtu) &&
from %pIS to %pIS\n",
				    src_mac, &rdst->remotening(dev))
	 sk= nrF_INE
	return ret;st->remoteue = get_unalowerdtvkb, vdtvdr t_by_				 Catelimin_fdk6 = ri	 					 vxlan->cfg.ds);tib_parkseTTpv6t(r!!(atINE
	ted("non-ECT from %puct v; ul, sThisocheckNM
	diffFI>gerr an_vNEIGmax_mtuT;becae;
#inaifoks lenl gstheulowerdtvIGmtuT;ra(f);rr an_theuxlan_rdvNEIGmax_mtuddr);
		
stlowerdtvaest(&vrucmax_mtut(rlowerdtvIGmtu--ru t g *nd(kseTTpv6t?he O b6ilEADROOM :he O bilEADROOMrar	un_dsrxw_mtu >cmax_mtu
 0;
)
{
	st-EIN*skb,[}l	utNEIGmtut(rrxw_mtu;o )
{
	struct vxlan_rdkvn sizeoffill>otocol,
				;

	rd =ue = get_unalignexlan->cfg.flags & V) &&
from %pIS to %pIS\n",
				    src_mac, &rdst->remote hk);
#if xlanS\ ifo= n
		s);
#if xlanrn fals	tr *) ||= 1)
sdead[h]*fpk	} elsk(sk an _port
	} ((&vxlan-ock)hron	/* _INE
	ted(skb-
min
		ret g _INE
	ted(skb-
maxsk_buff *	d
	} elstdiv->retutpto ha?e: F_INE
	ted(d) == 1)ar
u32 vxfk);
#if xlan_af(xlivpt start, offset;

	if (!(unparsed->vx_flags &ruct vx]FI>gxl|| skb->remcsum_offload)
		gr 0)
			p ff
v	p = ntohs(gr t.rECN_p(ipv6_aaevdbflag4hron	/*0, *div->retuto _)
		=Fructkliv->retuu.tpv4.dsk))
		=Fruct kliv->retuu.tpv4.por,	dt nlat|= 1)
		ret g *nrtdiv->ddaT) {
	
 xlivpar	un_dsmetaRR		tl) 0;
)
{
	stPTRtaRR		tl, vnvskrt(suty	to;ot}",
					el_notify_del_rx_port(vs->sock,
			    !!(vxflags & VXLAN_ruct vx]FI>gxl|| skb->ret:
	unparck,
			  d);
		ret{
n skarsedn sk= nateli6dr t.rECN_p(ipv6_aaevdbflag6hron	/*0, *div->retuto _)
		=	idiv->retulabel,t kliv->retuu.tpv6.dsk))
		=	 kliv->retuu.tpv6.por,	dt nlat|= 1)
		ret	rtdiv->ddaT) {
	
 xlivpar	un_dsmetaRR	ndinllbac	)
{
	stPTRtaRR	n */

	uidb_updLED(COn */

	#,
			, s!x_port(vs->SUPP;
k from -EP	}

	/* remolse
			nre	tdiv->retutptpor lssead[h]	tdiv->retutpto halsdead[h],)
{
	struct vxlan_rdatelimMSGSIZEue = get_usopsbIS to ock(&vxe(f);
opsbrt = .nroconit_r=dipv6_sonit,= .nrocunonit_r=dipv6_sunonit,= .nrocopen_r=dipv6_sopen,= .nrocstct_r=dipv6_sstct,= .nrocstbp Nxmit_r=dipv6_sxmit,= .nrocr t.)et =64	= *hk);
#if r t.)et =64,= .nrocs t.rx_m __r=dipv6_ss vni
	}
	kfre IS_,= .nrocchange_mtu_r=dipv6_schange_mtu,= .nrocderef16  ;

		  addrderef16  ;

	,= .nrocs t.i))
;

	ess	  addri))
;

	,= .nroc *sr;

_r=dipv6_s *sr;

,= .nroc *srtNl_r=dipv6_s *srdsleta,= .nroc *srtump_r=dipv6_s *srdump,= .nroc ill>otocol,
				r=dipv6_s ill>otocol,
				,
x_vnxlan_rdatelimMSGSIZEue = get_usopsbIS to ock(&vxNdb_opsbrt = .nroconit_r=dipv6_sonit,= .nrocunonit_r=dipv6_sunonit,= .nrocopen_r=dipv6_sopen,= .nrocstct_r=dipv6_sstct,= .nrocstbp Nxmit_r=dipv6_sxmit,= .nrocr t.)et =64	= *hk);
#if r t.)et =64,= .nrocchange_mtu_r=dipv6_schange_mtu,= .nroc ill>otocol,
				r=dipv6_s ill>otocol,
				,
x_vn sk_ifo=oan_uaevdbthatnthisock a virtF_I_u;
#if_		deairn oipvh->vx_flags;
dget_us	if (ipv6_s	if ((v = .nam ((v"ipv6_",
x_vn skCallotIP6hnrocu(sk);
#if ;

);

IP6hcallte(in ormac, v6->ssuppe_ttheulIS_enrr <1rr = u(sp= 1)s.kCall!=sNsrc_expectad6->sstre		rem>gerr 6hnrocu(sk);
#if ;

.
e32 #endif
if (!(vxflaGBP))
	.rx_= 1)s;

	rd =ue = get_unaligneb_parsush) &&
from %pIS to k, strucst->remoteue {
n fni (&vxlan-ock)st->remotening(dock_*v				   _gev);icCn_fdbIS to ock

	/* d	 be dropped.i* Neck_releas
ocn &= ~Vock);
		klan_sf

	f  i <s* re_HASH_SIZE; ++r u8 ethif (!vs)
		return fvxlanuct ocn &= ~Vocf ([i], f IS_Eaddr(mNDbe dropf IS_a	if FIru k,
		ar6= I (del_rcu(&rd->Aif		ret	if ((v
/* Update TYPE_ket.
	 PE;) {
Inverse p	if ((v
/* Update TYPE_ket.
FIru k,
		sush) se pu(sk);
#if sush.rx_= 1);aevdbar6== ~Vla	if r(sk{
Inverse pu(sk);
#if drop.rx_= 1);aevdbar6== ~Vla	if r(sk{}db *fpk_rebershi
ocn &= ~Vock);
		 rt, sInitiereze IP6h(&et_un>remoturoae32 #endif
if (!(vxfla, vut i	return e | NUD_NOARP) &&
from %pIS to %pIS\n",
				    src_mac, &rdst-	 be dropped.hstruaddr)w ;

	eranrom, &rdst-e(f);
, vut  &rdst
	dNEIGneeds_o_rds    sr)oitrONFI	SET_);
DEV_DEVTYPE dr *)oipv6_s	if dst
	dNEIGfeaturos	|st;ETIF %pILTX;
	dNEIGfeaturos	|st;ETIF %pSG;
	
ETIF %pHW_p;
}ipvdNEIGfeaturos *n|st;ETIF %pRXp;
}ipvdNEIGfeaturos *n|st;ETIF %pGSO_SOFTWAREst
	dNEIGvv6_s eaturos r(vNEIG eaturosipvdNEIG)w  eaturos |st;ETIF %pSG;
	
ETIF %pHW_p;
};
	
ETIF %pRXp;
}ipvdNEIG)w  eaturos |st;ETIF %pGSO_SOFTWAREst	d by dkeep				; &rdst-dNEIG_mac;

	f = thIFF_re_QUEUE; ul, sMTU;range: 68 - 65535SUPP;tNEIGm_remtut(r
			MIN_MTUst-dNEIGmax_mtut(r
			MAX_MTUst
	INIT_LISTilEAD
ock(&vn-_addf *	
k_rershi_onit;ock(&vn- == ock);
		]	tdidrtim_dtreterr0)
	yock(&vn-age_tim_d);
	ck(&vn-age_tim_d.func __u gsipv6_scLEDnup;
	ck(&vn-age_tim_d._igmpl viDbe dropST;
) race ;N
	ck(&vn-dig (sctvls	koan_sh

	f  hlikFDB_HASH_SIZE; ++hEsk{INIT_HLISTilEAD
ock(&vn- *sr		re[h]_		 rtnl_dereif (!(vxflae(f);
, vut i	return e | NUD_NOARP) &&
dNEIG_mac;

	f =&= ~IFF_TX_SKB_SHARINGst-dNEIG_mac;

	f = thIFF_LIVErtry _CHANGE;
	dNEIGne  srcopsbrt&IS to ock(&vxe(f);
ops		 rtnl_dereif (!(vxflaNdb_, vut i	return e | NUD_NOARP) &&
dNEIGvni != opsbrturn -EOreEIG if ((vturHRD_NONE;drdNEIGvard_vni != H wth_0fdbdNEIGs) ==ot sh_0fdbdNEIG

	f == IFF_POINTOPOINT;
	IFF_retur;
	IFF_n, srCAS);
	dNEIGne  srcopsbrt&IS to ock(&vxNdb_opsuct vxlan_rdatelimMSGSIZEula_arsed;!(vxflaarsed;[IFLA_e O biMAX->doT((v = [IFLA_e O biID]_r=d{ .	if ((vNLA_U32 },= [IFLA_e O biGROUP]r=d{ .ot sh_FIELD_SIZEOFlan_rdst*hd)
o dxlan_ },= [IFLA_e O biGROUP6]r=d{ .ot sh_ ct vxlan_rdst*_hdr(st_ },= [IFLA_e O biLINK]r=d{ .	if ((vNLA_U32 },= [IFLA_e O biIpin_]r=d{ .ot sh_FIELD_SIZEOFlan_rdst*hd)
o sr(st_ },= [IFLA_e O biLpin_6]r=d{ .ot sh_ ct vxlan_rdst*_hdr(st_ },= [IFLA_e O biTOS]r=d{ .	if ((vNLA_U8 },= [IFLA_e O biTTL]r=d{ .	if ((vNLA_U8 },= [IFLA_e O biLABEL]r=d{ .	if ((vNLA_U32 },= [IFLA_e O biIpe->ING]r=d{ .	if ((vNLA_U8 },= [IFLA_e O biAGEING]r=d{ .	if ((vNLA_U32 },= [IFLA_e O biIIMIT]r=d{ .	if ((vNLA_U32 },= [IFLA_e O bi* re_RANGET((v  .ot ssh_ ct vxlan_rdst*

	_(vxflaarp Nrange_ },= [IFLA_e O biPROXY]r=d{ .	if ((vNLA_U8 },= [IFLA_e O biRSC]r=d{ .	if ((vNLA_U8 },= [IFLA_e O biL2MISS]r=d{ .	if ((vNLA_U8 },= [IFLA_e O biL3MISS]r=d{ .	if ((vNLA_U8 },= [IFLA_e O bint OAM
	 * proce]r=d{ .	if ((vNLA_U8 },= [IFLA_e O bi* re]r=d{ .	if ((vNLA_U16 },= [IFLA_e O bi
/* p;
}]r=d{ .	if ((vNLA_U8 },= [IFLA_e O bi
/* ZERO_p;
}6
TX]r=d{ .	if ((vNLA_U8 },= [IFLA_e O bi
/* ZERO_p;
}6
RX]r=d{ .	if ((vNLA_U8 },= [IFLA_e O bin pp;
}
TX]r=d{ .	if ((vNLA_U8 },= [IFLA_e O bin pp;
}
RX]r=d{ .	if ((vNLA_U8 },= [IFLA_e O biGBP]r=d{ .	if ((vNLA_FLAG, },= [IFLA_e O biGPE]r=d{ .	if ((vNLA_FLAG, },= [IFLA_e O bin pp;
}
NOpolicy_]r=d{ .	if ((vNLA_FLAG },=x_vnxlan_rdkvn sizeofderef16 (MSGSIZEulattr *tb[], MSGSIZEulattr *_igm[],ru t gi	return elink_addrae p*addae ) &&
	INItb[IFLA_try ESS]s			  vs->ula_ot Itb[IFLA_try ESS]s	!=rdrcnt;
	Eaddr(mNL_SET_ERR_MSG_ATTR(addae la	b[IFLA_try ESS]ourn 0;Fruc"ProvidropSink lay_r	r

	essock noti  rdst->"ron		, ETH_HL-EIN*skb,[	}p, cvs->r_sfderefee(f);
}

stula__igmItb[IFLA_try ESS]s)Eaddr(mNL_SET_ERR_MSG_ATTR(addae la	b[IFLA_try ESS]ourn 0;Fruc"Providrop  rdst->rr

	essock notiun
	kfr"ron		, ETH_HL-Etry NOTAVAI ndm= 1;return -	b[IFLA_MTU]s			  sed-mtut(rrla_r t.sed-	b[IFLA_MTU]sD;t-&vs->mtut<r
			MIN_MTUl|| mtu >c
			MAX_MTUEaddr(mNL_SET_ERR_MSG_ATTR(addae la	b[IFLA_MTU]ourn 0;Fruc"MTUlmulimbembetwet s68 r_eq65535"ron		, ETH_HL-EIN*skb,[	}p;return -!_igmEaddr(NL_SET_ERR_MSG(addae lru t g *nd "Requirad attribu*f;
notiprovidropto peroanm IP6hopern->ha"ENAB)remote_-EIN*skb,[}l	u_INIWigm[IFLA_e O biID]s			  sed-
	suprla_r t.sed-Wigm[IFLA_e O biID]sD;t-&vs->
	s>the O biN_VIDEaddr(mNL_SET_ERR_MSG_ATTR(addae la	b[IFLA_e O biID]ourn 0;Fruc"1rr = IDlmulimbemlowerrr an_16777216"ron		, ETH_HL-ERANGE;
		}p;return -Wigm[IFLA_e O bi* re_RANGETEaddr(atelimMSGSIZE 

	_(vxflaarp Nrange *p
c_r=dula__igmIWigm[IFLA_e O bi* re_RANGETEstru  used_gpe pIGvigh) <s _gpe pIGlow)Eaddr(mNL_SET_ERR_MSG_ATTR(addae la	b[IFLA_e O bi* re_RANGETourn 0;Fruc"I_derefsn err p= 1)krange"ron		, ETH_HL-EIN*skb,[	}p;retu_sock *0 = rtnl_dereif (!(vxflar t.drvxlanrn	return e | NUD_NOock(&vlru t g *ndan_rdstmiltool.drvxlan *_rvxlan) &&
froln rc_rvxlanskb, eth_,he O biVERSIONue ct vxl_rvxlanskb, eth_))st->reln rc_rvxlanskdmac_d,i"ipv6_",e ct vxl_rvxlanskdmac_d))uct vxlan_rdatelimMSGSIZEmiltool.opsbIS to miltool.opsb(v = .r t.drvxlan	=!(vxflar t.drvxlan,= .r t.Sink	  addtool.op_r t.Sink,=x_vnxlan_rdxlan->cfoark 		t.)add n_fdb k, s(>remoteue {
n fneb_partpv6ourn 0;, struct nlat, &s

	f ) &&
from %pfoark 		sshi;

from %pu(skarp Ncfgpu(skatefst-truct vxl	uo nla_p&u(skateffailure;

	iu(skatef)_		tun_dstpv6s			  s(skatef. *)oiph)->tos);
	;	  s(skatef.kseTs(s6.rx_check hesb(
XLAN_F!I"non-ECT from %p
/* ZERO_p;
}6
RX,xl is(skatef.ph)->v6oniph)-1;start, offsets(skatef. *)oiph)->tos);
b,[}l	us(skatef.	iyaltu(skarp h)-= 1)ar
u/* Opt sd_mafoark 		t_sdata *k(sknshi_an_fdb_n_fdb&u(skateffa&
	unparcn_dsf (likely(f,
					ERR_PTR(raw_onct sk_bufsshi;
 rtxlaCn_fdb_ devlIS_enafoark 	n_dneedbdt = ipv6_stfrom %pIS to k, struS to k, s t.an_fdb_hremoteue {
n fneb_partpv6ourn 0; g *nd, struct nlat, &s

	f ) &&
from %pning(dock_*v				   _gev);icCn_fdbIS to ock

	/* dfrom %pIS to k, strucst->remotefoark 		sshi;

	 be dropped.hst
from %pu(sk);
#if nshi_afgp);
#if afg;N
	csb(vkzaan_fe ct vxlfv}
,lt(vnKERNEe(vxlb)->!vsly(f,
					ERR_PTR(iv(dMEM)ls	koan_sh

	f  hlikVNI_HASH_SIZE; ++hEsk{INIT_HLISTilEAD
ocs->reiocf ([hTEstruk, stgsipv6_scn_fdb k, s(n_fdbtpv6oct nlat

	f S;) n_dsmetaRR	
	unpbe16 pan_rd(v=ENAB)remote_ERR_CAS)(
	unparc}N
	cs &= ~Vsh_ shi;

refcount_la_p&cs &refcnlat1);
	csIG

	f == I"non-ECT from %pRCV_FLAGS/* Neck_releas
ocn &= ~Vock);
		kfree(sxlam		re_xlanocsIGfree(db(s	notifn_fdbt nl))st-u(sk);
#if notifysxlamrx_= 1);= ~Vl		ret g *n		ar6= I (del_rcu(&rd->Aif ?		ret g *n	
/* Update TYPE_ket.
	 PE :		ret g *n	
/* Update TYPE_ket.
r t.
k_rebershi
ocn &= ~Vock);
		ul, sM drofoark 	as r_  ifindex, __u foark ae32 uo nla_p&);
#if afgfailure;

	i);
#if afg))st-);
#if afg.sk>d#Fr__igmpl ucst-);
#if afg. ifin_	if ((v1st-);
#if afg. ifin_rcvtgsipv6_srcvst-);
#if afg. ifin_dsinroybrturn -EO);
#if afg.ct vreceac_= ntohs(grt vreceac_-EO);
#if afg.ct v(strleta= ntohs(grt v(strleta* Nec vut_u(sk);
#if nshi(n_fdbflagfa&);
#if afg)onct sk_bufvsuct vxlan_rdkvn __race bk, ssxlapI>sa.sa_family = AF_INET;b_partpv6s &&
from %pIS to ock_*v				   _gev);icCatelimin_fdbIS to ock

	/* dfrom %pIS to k, strucbrturn -EO
	if (!(unparg(vsn __e*n __stru(f)) F_INE
	ted(no_shar	be16 pck_releas
ocn &= ~Vock);
		k	csb(vipv6_s ind nshi(atelimin_fdbTpv6t?h>tos);
	 :->tos);
d->vvt g *n _INE
	ted(d) == 1)
datINE
	ted("non-par	un_dsvsvid);refcount_inc not>xlanp&cs &refcnl)_addr(m_k_rebershi
ocn &= ~Vock);
				, ETH_HL-EBUSY;t	u}/*	
k_rebershi
ocn &= ~Vock);
			}xlb)->!vsly(fcsb(vipv6_sk, s t.an_fdb_atelimin_fdbTpv6ourn 0;  _INE
	ted(d) == 1)
datINE
	ted("non-par	n_dsmetaRR	vslly(f,
					PTRtaRR	v=ENAel_notify_del_rx_port(vs->socn_dstpv6s			  _rucasse d_eairner|| skb->ret:
	undb(s
				n __e nrF_INE
	 ree(6;start, oolse
			n		  _rucasse d_eairner|| skb->re4:
	undb(s
				n __e nrF_INE
	 ree(4			}xlsizeofdrexlamreq uct s_INET;n __);o )
{
	struct vxlan_rdkvn sizeofk, ssxlapI>sa.sa_family = AF_INEadatib_parxlan_igmp= F_INE
	ted("non-ECT from %pnt OAM
	 * proce;tib_parTpv6t(ratINE
	ted("non-ECT from %puct l|| mlan_igm;tib_parTpvags !Tpv6t|| mlan_igm;titrucrt  struct RCU_INIT_POINTER|| skb->re4:
	undbence(vxel_notify_del_rx_port(vs->socRCU_INIT_POINTER|| skb->re6:
	undbence(vxcn_dstpv6s			  _t  st__race bk, ssxlaps_INET;_buff *	
,
		x_vn<(svid)x_vn!_buf;	}

	/* re) se Tpvags net/ipvGH)lse
			nn_dstpv4)	  _t  st__race bk, ssxlaps_INET;f->vnS;) ,
		x_vn<(skb),aS to k, sspdLED(CO(ipv6ront sk_buff tNA rtxlan_rdkvn sizeofctefigfderef16 (MSGSIZEuk 		src_n_fdbf>sa.sa_familctefig *ateff->vvt n	return e | NUD_NOalower))
		= ->sa.sa_family = Aoldf->vvt n	return elink_addrae p*addae ) &&
from %pIS to ock_*v				   _gev);icCsrc_n_fdbIS to ock

	/* dfrom %pIS to y = Atmp;tib_parkseTTpv6t(rn->vn4_
k,
		atef6= I (del_rcu(&rd->Aif 		  , sFan_ owluallow  PE oniphtoge(f); with
XLAlaCt OAM
	 * proce.sThisocr_ beff laxropS16 r;(in such
XLAla	kfe, IP6ho(f); side);

IP6hPtPpSink will have	strun
XLAlaprovidro.
XLAld
& vs->	atef6= I (del_~rcu(&rd-ALLOWED->Aif _fdb_noti!	atef6= I (del_rcu(&rd-nt OAM
	 * procesEaddr(mNL_SET_ERR_MSG(addae lru tt g *nd "1rr =  PE dof;
notisuppIS_a	hisocomblan->has;

attribu*f;"ron		, ETH_HL-EIN*skb,[	}p;return -!atef6=}

static.gs)
{
	size_tid);atef6=sfset =s)
{
	size_f 		  , sUnlesso_Pv6thot xplicite_trequsined, assumeo_Pv4Ald
& atef6=}

static.gs)
{
	size_t)->tos);
b,[	atef6=sfset =s)
{
	size_h)->tos);
b,[}",
				INI!atef6=}

static.gs)
{
	size_Eaddr(atef6=}

static.gs)
{
	size_t)-atef6=sfset =s)
{
	size_b,[}",
				INI!atef6=sfset =s)
{
	size_f 		  atef6=sfset =s)
{
	size_h)-atef6=}

static.gs)
{
	size_b,[}l	u_INIatef6=sfset =s)
{
	size_h!)-atef6=}

static.gs)
{
	size_Eaddr(NL_SET_ERR_MSG(addae lru t g *nd "Lfyal r_eq}

starr

	essomulimbeman_fdtheuxam (	size_"ENAB)remote_-EIN*skb,[}l	u_INIIS to ;

	e0;

	if (l&atef6=sfset)Eaddr(NL_SET_ERR_MSG(addae l "Lfyal r

	essocr_notibem0;

	if ("ENAB)remote_-EIN*skb,[}l	u_INIatef6=sfset =s)
{
	size_h))->tos);
	s			  vs->!otify_del_rx_port(vs->sEaddr(mNL_SET_ERR_MSG(addae lru tt g *nd "_Pv6tsuppIS_anotien0)
	oppedtheukdst-l"ron		, ETH_HL-EP	}

	/* remo u}/*	kseTTpv6t(rtrONFI		atef6= I (de the O bi%puct D;t-&vs->r	atef6= I (del_rcu(&rd-nt OAM
	 * procesEaddr(mtrucoiyalt	if ((ru ttph)->;

	e	if l&atef6=sfset p) t net_devicENAB	vnrucrt
stat	if ((ru ttph)->;

	e	if l&atef6=k f, &ip, p) t net_devic)FIru k,
		oiyalt	if (l_I*V6rtry _LINKIpin_)X]) {
vvs->r	rt
stat	if (l_I*V6rtry _LINKIpin_)X;
		k			   Irt
stat	if (!=_I*V6rtry _ANYitX]) {
vmNL_SET_ERR_MSG(addae lru tttt g *nd "__derefscomblan->has;

lfyal r_eq}

starr

	essoscopf;"ron		,	, ETH_HL-EIN*skb,[	[	}p, c		atef6= I (de the O bi%puct _LINKIpin_b,[	[}",
					  cun_ds}

stat	if (((		k			   II*V6rtry _UNICAS);
	I*V6rtry _LINKIpin_)tX]) {
vmNL_SET_ERR_MSG(addae lru tttt g *nd "__derefscomblan->has;

lfyal r_eq}

starr

	essoscopf;"ron		,	, ETH_HL-EIN*skb,[	[	}p, c		atef6= I (de&= ~e O bi%puct _LINKIpin_b,[	[},[	}p;return -atef6=label{id);kseTTpv6Eaddr(NL_SET_ERR_MSG(addae lru t g *nd "Label{attribu*f oniphapplie
	str_Pv6t1rr = | NUD_s"ENAB)remote_-EIN*skb,[}l	u_INIatef6= vxlan->cfg.ds)set;

	if (!ue = get_unalowerdtvFIru lowerdtvkb, vdtvdr t_by_				 Csrc_n_fdbatef6= vxlan->cfg.ds) *	
,
		!lowerdtvaest(&mNL_SET_ERR_MSG(addae lru tt g *nd "__derefslfyal irnerfar <= get_unnotifound"ron		, ETH_HL-ENODEVb,[	}p,el_notify_del_rx_port(vs->sock))
	NseTTpv6Eaddr(->remote n_f6 y = Aidtvkb, vet_ddtvdr ttlowerdtva;ru k,
		idtvk&&ridNEIGcnf.dis0)
	TTpv6Eaddr(-mNL_SET_ERR_MSG(addae lru ttt g *nd "_Pv6tsuppIS_adis0)
	d bphadminee(rator"ron		,	 ETH_HL-EPERMb,[	[},[	}pons(ETH_		alowert(rlowerdtv;start, offset_INIIS to ;

	e0;

	if (l&atef6=}

statci.ne16 pmNL_SET_ERR_MSG(addae lru tt g *nd "Lfyal irnerfar trequirad oan_0;

	if (q}

star(&vxlan->ha") = 0;
)
{
	stivIN*skb,[	}p,el_notify_del_rx_port(vs->sock))
	atef6= I (del_rcu(&rd-uct _LINKIpin_ne16 pmNL_SET_ERR_MSG(addae lru tt g *nd "Lfyal irnerfar trequirad oan_Sink-lfyal lfyal/}

starr

	essf;"ron		, ETH_HL-EIN*skb,[	}pons(ETH_		alowert(rurn -EOreturn -!atef6=d) == 1)s			  vs->atef6= I (del_rcu(&rd->Aif
c		atef6=sb_u
	} elsremote4790);	, sIANA_rcu(&- PE 
	} eld
& Inverse atef6=sb_u
	} elsremote(vxflaarp )-EOreturn -!atef6=age_tructval
 0;atef6=age_tructvalsh_FDB_AGE_DEFAULT = 0if (!vs)
		return f(tmpt ocn &(vxflaree(db_addf			  vs->tmph))-oldf
c		ateD(CONFIru vs->tmp
	ted(}

h!)-atef6=ARN) 	_rBLED(CONFI		vs->tmp
	ted(sb_u
	} e!)-atef6=d) == 1)s 	_rBLED(CONFI		vs->>tmp
	ted(acb_no6E( from %pRCV_FLAGS;
	 from %puct vs	!=db_noti>atef6= I (del_( from %pRCV_FLAGS;
	 from %puct vsf
c		ateD(CONFIru vs->	atef6= I (del_rcu(&rd-uct _LINKIpin_ne;
		kfreetmp
	ted( vxlan->cfg.dsh!)-atef6=}

staticfg.ds)
c		ateD(CONFIru NL_SET_ERR_MSG(addae lru t g *nd "At1rr = | NUD_ withdtheuxpecicfad VNIualp(styt xee(s"ENAB)remote_-EEXIS
b,[}l	u_sock *0 = rtnl_dereif (!(vxflactefigfapply;

	rd =ue = get_unalignru t g *nd f>sa.sa_familctefig *ateff->vv g *nd f>sa.saue = get_unalowerdtvf->vv g *nd f>sa.saue 		src_n_fd->vv g *nd b_parchangeSink) &&
from %pIS to %pIS\n",
				    src_mac, &rdst->remotening(dev))
	 sk= nrF_INE
	return ret;st-NDbe dropf IS_aneedbd	notiroomt(r
			HLEN;tib_parkseTTpv6t(r!r	atef6= I (del_rcu(&rd-uct v; &vrucmax_mtut(r
			MAX_MTUst
	rn -!ahangeSink)			  vs->atef6= I (del_rcu(&rd->Aif
c		(vxflaNdb_, vut dtva;ru Inverse (vxflae(f);
, vut  &rdst
	 vs->atef6=mtu
 0;
tNEIGmtut(ratef6=mtust
	 atelimin_fsh_ rc_n_fb,[}l	uct vxlan_dev_pr )-atef6=ARNxl	uo nn rc&ct vxlan_dev *t oatef6=}

statcilure;

	iatef6=}

statci.nst
	rn -lowerdtvaest(&					 vxlan->cfg.dsh)-atef6=}

staticfg.ds;&
;
tNEIGgso
max_re;
t(rlowerdtvIGgso
max_re;
;
;
tNEIGgso
max_ref == lowerdtvIGgso
max_ref  = 0;needbd	notiroomt(rlowerdtvIGvard_vni != H w = 0;max_mtut(rlowerdtvIGmtu--d(kseTTpv6t?he O b6ilEADROOM :ru ttt g e O bilEADROOMrar	un_dsmax_mtut<r
			MIN_MTU
 0;
max_mtut(r
			MIN_MTUst*	
,
		!ahangeSinktid);atef6=mtu
 0;
tNEIGmtut(rmax_mtu-EOreturn -tNEIGmtut>cmax_mtu
 0;tNEIGmtut(rmax_mtu-E
k))
	NseTTpv6t|| atef6= I (del_rcu(&rd-nt OAM
	 * proces 0;needbd	notiroomt+=he O b6ilEADROOM = Inverseneedbd	notiroomt+=he O bilEADROOM = dNEIGneedbd	notiroomt(rneedbd	notiroomxl	uo nn rc&atINE
	teddbateflure;

	i*atef)_		 rtxlan_rdkvn sizeof srcctefigur (MSGSIZEuk 		src_n_fdbf>sa.saue = get_unalignru t g *nd f>sa.sa_familctefig *ateff b_parchangeSinkf->vv g *nd f>sa.saue link_addrae p*addae ) &&
from %pIS to %pIS\n",
				    src_mac, &rdst->remoteue = get_unalowerdtvFI-trucrt metax_vni IS to ctefigfderef16 (Mrc_n_fdbatef*)o	iwerdtvf s_INET;addae );) ,
		x_vkb), ETH_HLrt meta(vxflactefigfapply;dtvf atef*)	iwerdtvf Mrc_n_fdbahangeSink);l	u_sock *0 = rtnl_derekvn __race b srccn_fdb_hremoteue {
n fnef>sa.saue = get_unalignru t g *ndf>sa.sa_familctefig *ateff->vv g *ndn	return elink_addrae p*addae ) &&
from %pIS to ock_*v				   _gev);icCn_fdbIS to ock

	/* dfrom %pIS to y = An",
				    src_mac, &rdst->remotening(d *src_t(rurn -EOb_parknregIS_ergs net/ipvGtruct vxl	uImily == AF_ srcctefigur (n_fdb(&vf atef*)net/iT;addae );) ,
		< cb->aremote_vni;
= dNEIGmiltool.opsb(v&IS to miltool.ops		ul, scn_fdb r_  *sr		ret{oan_a derefsreturn a(&vxlan->has);
		
st!IS to ;

	err =&F_INE
	return ret;
}

statci.ne16 pImily == AF_ *sran_fdb_ateliluall>xlansen v
ru tt g *nd &F_INE
	return ret;
}

statci
ru tt g *nd 
	if (fam_del;
	
	ifPERMANEN)
ru tt g *nd  _INE
	ted(d) == 1)
ru tt g *nd  _INE
	return ret;
}

stat			
ru tt g *nd  _INE
	return ret;
}

stat			
ru tt g *nd  _INE
	return ret;
}

statk
				  ru tt g *nd 
TF_SELFg &frar	un_ds< cb->a_remote_vni;
	xlanImily regIS_er ock(&vird;aev);) ,
		< cb->a*gpe >k=}u;st-NDregIS_ergs trONFIanImily rtnlcctefigur .Sink;dtvf ence(vxcn_ds< cb->a*gpe >k=}u;stul, snotifysreturn a *sr		ret{);
		
st	
 0;(vxfla *srnotify	lan_flufskfirb_upd
statrtnlt	
, RTM_NEW32 et) = 0if (!xlapock(&vn-_addt ocn &(vxflaree();o )
{
	struc
>k=}u;:ul, sNDregIS_er ock(&vird;) dsinroys IP6h(&turn aFDBr		ret{withddslet>hanl gsnotificn->ha. Buerr = xlai->hasnotificn->ha wa;
notis>gery_fdbflnl gsdsinroybr = 		ret{by hr_eqf);e.ddr);
		
st	
 0;(vxfla *srdsinroy	lan_flufskf->vnS;) ,
		NDregIS_er
 0;NDregIS_er ock(&vird;aev);) remote_vni;
 rtxlan_rdkvn sizeofnl2ctef(MSGSIZEulattr *tb[], MSGSIZEulattr *_igm[],ru t 

	rd =ue = get_unalignexlan->c_familctefig *ateff->vv b_parchangeSink) &&
from %pIS to %pIS\n",
				    src_mac, &rdst uo nla_pateffailure;

	i*atef)_		ul, s,
	ahangeSinktopern->hanexlbp =withdoldt xee(rr <afgp);
		
stchangeSink) 	uo nn rcatef*)oatINE
	teddbre;

	i*atef)_		ul_INIWigm[IFLA_e O biID]s			  ex;
#endif

	dex;tox;
#e(rla_r t.sed-Wigm[IFLA_e O biID]sdst
	 vs->ahangeSinktid)(}

h!)-atef6=ARN)
 0;
)
{
	st-EOPNOT
	/*FI		atef6=dif

	dex;tox;
#e(rla_r t.sed-Wigm[IFLA_e O biID]sdst;return -Wigm[IFLA_e O biGROUP])			  vs->ahangeSinktid)(atef6=}

static.gs)
{
	size_t!start, off
 0;
)
{
	st-EOPNOT
	/*FI
r(atef6=}

static.g) {
		if (vxlan_get_srla_r t.		if (v-Wigm[IFLA_e O biGROUP]);
r(atef6=}

static.gs)
{
	size_h)->tos);
b,[}",
				INIWigm[IFLA_e O biGROUP6]s			  vs->!otify_del_rx_port(vs->sE 0;
)
{
	st-EP	}

	/* remo	  vs->ahangeSinktid)(atef6=}

static.gs)
{
	size_t!start, of>sE 0;
)
{
	st-EOPNOT
	/*FI
r(atef6=}

static.g) t net_device *rla_r t.		6if (v-Wigm[IFLA_e O biGROUP6]);
r(atef6=}

static.gs)
{
	size_h)->tos);
6st;return -Wigm[IFLA_e O biIpin_])			  vs->ahangeSinktid)(atef6=sfset =s)
{
	size_h!)-art, off
 0;
)
{
	st-EOPNOT
	/*FI
r(atef6=sfset p) {
		if (vxlan_get_srla_r t.		if (v-Wigm[IFLA_e O biIpin_])b,[	atef6=sfset =s)
{
	size_h)->tos);
b,[}",
				INIWigm[IFLA_e O biIpin_6]s			  vs->!otify_del_rx_port(vs->sE 0;
)
{
	st-EP	}

	/* remo	  vs->ahangeSinktid)(atef6=sfset =s)
{
	size_h!)-art, of>sE 0;
)
{
	st-EOPNOT
	/*FI
r(, sTODO: rexpectoscopf-
	sld
& atef6=sfset p) t net_device *rla_r t.		6if (v-Wigm[IFLA_e O biIpin_6]sb,[	atef6=sfset =s)
{
	size_h)->tos);
6st;return -Wigm[IFLA_e O biIINK])
r(atef6=}

staticfg.dsh)-rla_r t.sed-Wigm[IFLA_e O biIINK])		ul_INIWigm[IFLA_e O biTOS])
r(atef6=eo =h)-rla_r t.s8IWigm[IFLA_e O biTOS])		ul_INIWigm[IFLA_e O biTTL])
r(atef6=etl== rla_r t.s8IWigm[IFLA_e O biTTL])		ul_INIWigm[IFLA_e O biLABEL])
r(atef6=label{(rrla_r t.;
#e(Wigm[IFLA_e O biLABEL]) 
		k	    	I*V6rFLOWLABEL_MASK		ul_INIWigm[IFLA_e O biLpe->ING]s			  vs->ula_r t.s8IWigm[IFLA_e O biLpe->ING]sf
c		atef6= I (de the O bi%pLpe->;ru Inverse atef6= I (de&= ~e O bi%pLpe->;ru}",
				INI!ahangeSink)			  /*h(&turn ape lea	stha a_ dev get_unad
& atef6= I (de the O bi%pLpe->;rureturn -Wigm[IFLA_e O biAGEING])			  vs->ahangeSink
 0;
)
{
	st-EOPNOT
	/*FI		atef6=age_tructvalsh_rla_r t.sed-Wigm[IFLA_e O biAGEING]);rureturn -Wigm[IFLA_e O biPROXY])			  vs->ahangeSink
 0;
)
{
	st-EOPNOT
	/*FI		vs->ula_r t.s8IWigm[IFLA_e O biPROXY])f
c		atef6= I (de the O bi%pPROXY;rureturn -Wigm[IFLA_e O biRSC])			  vs->ahangeSink
 0;
)
{
	st-EOPNOT
	/*FI		vs->ula_r t.s8IWigm[IFLA_e O biRSC])f
c		atef6= I (de the O bi%pRSCst;return -Wigm[IFLA_e O biI2MISS])			  vs->ahangeSink
 0;
)
{
	st-EOPNOT
	/*FI		vs->ula_r t.s8IWigm[IFLA_e O biI2MISS])f
c		atef6= I (de the O bi%pL2MISSst;return -Wigm[IFLA_e O biI3MISS])			  vs->ahangeSink
 0;
)
{
	st-EOPNOT
	/*FI		vs->ula_r t.s8IWigm[IFLA_e O biI3MISS])f
c		atef6= I (de the O bi%pL3MISSst;return -Wigm[IFLA_e O biIIMIT])			  vs->ahangeSink
 0;
)
{
	st-EOPNOT
	/*FI		atef6=asetmash)-rla_r t.sed-Wigm[IFLA_e O biIIMIT])st;return -Wigm[IFLA_e O bint OAM
	 * proce])			  vs->ahangeSink
 0;
)
{
	st-EOPNOT
	/*FI		vs->ula_r t.s8IWigm[IFLA_e O bint OAM
	 * proce])f
c		atef6= I (de the O bi%pnt OAM
	 * proce;tireturn -Wigm[IFLA_e O bi* re_RANGETEaddr(	INI!ahangeSink)			  (atelimMSGSIZE 

	_(vxflaarp Nrange *p
c_rr=dula__igmIWigm[IFLA_e O bi* re_RANGETEstc		atef6=skb-
minh)-r_gpe pIGlow)stc		atef6=skb-
mash)-r_gpe pIGvigh)b,[	}",
					  c)
{
	st-EOPNOT
	/*FI		}p;return -Wigm[IFLA_e O bi* re])			  vs->ahangeSink
 0;
)
{
	st-EOPNOT
	/*FI		atef6=sb_u
	} elsrla_r t.;
16-Wigm[IFLA_e O bi* re]);tireturn -Wigm[IFLA_e O bi
/* p;
}])			  vs->ahangeSink
 0;
)
{
	st-EOPNOT
	/*FI		vs->!ula_r t.s8IWigm[IFLA_e O bi
/* p;
}])f
c		atef6= I (de the O bi%p
/* ZERO_p;
}_TX;
	return -Wigm[IFLA_e O bi
/* ZERO_p;
}6
TX])			  vs->ahangeSink
 0;
)
{
	st-EOPNOT
	/*FI		vs->ula_r t.s8IWigm[IFLA_e O bi
/* ZERO_p;
}6
TX])f
c		atef6= I (de the O bi%p
/* ZERO_p;
}6_TX;
	return -Wigm[IFLA_e O bi
/* ZERO_p;
}6
RX])			  vs->ahangeSink
 0;
)
{
	st-EOPNOT
	/*FI		vs->ula_r t.s8IWigm[IFLA_e O bi
/* ZERO_p;
}6
RX])f
c		atef6= I (de the O bi%p
/* ZERO_p;
}6_RX;
	return -Wigm[IFLA_e O bin pp;
}
TX])			  vs->ahangeSink
 0;
)
{
	st-EOPNOT
	/*FI		vs->ula_r t.s8IWigm[IFLA_e O biR pp;
}
TX])f
c		atef6= I (de the O bi%pR pp;
}
TX;
	return -Wigm[IFLA_e O bin pp;
}
RX])			  vs->ahangeSink
 0;
)
{
	st-EOPNOT
	/*FI		vs->ula_r t.s8IWigm[IFLA_e O bin pp;
}
RX])f
c		atef6= I (de the O bi%pR pp;
}
RX;
	return -Wigm[IFLA_e O biGBP])			  vs->ahangeSink
 0;
)
{
	st-EOPNOT
	/*FI		atef6= I (de the O bi%pGBP;
	return -Wigm[IFLA_e O biGPE])			  vs->ahangeSink
 0;
)
{
	st-EOPNOT
	/*FI		atef6= I (de the O bi%pGPE;) return -Wigm[IFLA_e O bin pp;
}
NOpolicy_])			  vs->ahangeSink
 0;
)
{
	st-EOPNOT
	/*FI		atef6= I (de the O bi%pn pp;
}
NOpolicy_;) return -	b[IFLA_MTU]s			  vs->ahangeSink
 0;
)
{
	st-EOPNOT
	/*FI		atef6=mtut(rrla_r t.sed-	b[IFLA_MTU]sD;[}l	u_sock *0 = rtnl_derekvn sizeofnewSink;MSGSIZEuk 		src_n_fdbf>sa.saue = get_unalignru t MSGSIZEulattr *tb[], MSGSIZEulattr *_igm[],ru t 

	rd =ue link_addrae p*addae ) &&
from %pIS to ctefig ctefst-truct vxl	uImily == AF_nl2ctef(t	/*aigmdb(&vf &atef*)net/i);) ,
		< cb->aremote_vni;
= remote___race b srccn_fdb_hrc_n_fdb(&vf &atef*)addae );)t vxlan_rdkvn sizeofchangeSink;MSGSIZEuk = get_unalignexlan->culattr *tb[],		k	    MSGSIZEulattr *_igm[],ru t gd f>sa.saue link_addrae p*addae ) &&
from %pIS to %pIS\n",
				    src_mac, &rdst->remotening(dev))
	 sk= nrF_INE
	return ret;st->remotening(dev))
old_et;st->remotening(dctefig ctefst-truct vxl	uImily == AF_nl2ctef(t	/*aigmdru t gd (&vf &atef*)_buff *	,
		< cb->aremote_vni;
= o nn rc&old_et;
dd) ue ct vxlan_rdstning(dev))sdst
	Imily == AF_ srcctefigur (atelimin_fdb(&vf &atef*)_bufT;addae );) ,
		< cb->aremote_vni;
= /*hhr_el6h(&turn av))
		ret{);
		
st!IS to ;

	eequalc&ct vxlan_dev *t oold_et;
}

statci.ne16 pck_releas_bh
ock(&vn- == ock);
		kk	
st!IS to ;

	err =&old_et;
}

statci.nru t__race be*srdsleta_ateliluall>xlansen v
ru ttt gdold_et;
}

statci
ru ttt gd _INE
	ted(d) == 1)
ru ttt gdold_et;
}

stat			
ru ttt gdold_et;
}

stat			
ru ttt gdold_et;
}

statk
				   0dst
	 vs->!IS to ;

	err =&ct vxlan_dev *.ne16 pmImily == AF_ *srupdfdb_ateliluall>xlansen v
ru ttt g *nd &ct vxlan_dev *tru ttt g *nd 
	if (fam_del;
	
	ifPERMANEN)
ru ttt g *nd 
LMrd-APPEND;
	
LMrd-C (fTE
ru ttt g *nd  _INE
	ted(d) == 1)
ru ttt gd gd (t vxlan_dev_pr
ru ttt gd gd (t vxlan_dev_pr
ru ttt gd gd (t vxlan_devk
				  ru ttt g *nd 
TF_SELFa;ru k,
		raw_puct *	
k_rebershi_bh
ock(&vn- == ock);
		k>a_remote_vni;
		[},[	}p;	
k_rebershi_bh
ock(&vn- == ock);
		k

	r_sock *0 = rtnl_dereif (!(vxfladslSink;MSGSIZEuk = get_unalignexlan->cif (!noti *noti) &&
from %pIS to %pIS\n",
				    src_mac, &rdst u== AF_ lushylan_flu_buff * 0if (!dsl
ock(&vn-_addf *	NDregIS_er ock(&vird_queud;aevdbnoti) = rtnl_dere ct _%pIS to r t.)ct (ctelimMSGSIZEue = get_uNOARP) &&	r_sock *rla_total.)ct ( ct vxl_.sed.ne+ /*hIFLA_e O biIDnad
& rla_total.)ct ( ct vxlan_rdst*_hdr(st_p->d/*hIFLA_e O biGROUP{6}nad
& rla_total.)ct ( ct vxl_.sed.ne+ /*hIFLA_e O biIINKnad
& rla_total.)ct ( ct vxlan_rdst*_hdr(st_p->d/*hIFLA_e O biIpin_{6}nad
& rla_total.)ct ( ct vxl_.s8.ne+ /*hIFLA_e O biTTLnad
& rla_total.)ct ( ct vxl_.s8.ne+ /*hIFLA_e O biTOSnad
& rla_total.)ct ( ct vxl_.;
#e_p->d/*hIFLA_e O biIABELnad
& rla_total.)ct ( ct vxl_.s8.ne+ /*hIFLA_e O biLpe->INGnad
& rla_total.)ct ( ct vxl_.s8.ne+ /*hIFLA_e O biPROXYnad
& rla_total.)ct ( ct vxl_.s8.ne+ /*hIFLA_e O biRSCnad
& rla_total.)ct ( ct vxl_.s8.ne+ /*hIFLA_e O biL2MISSnad
& rla_total.)ct ( ct vxl_.s8.ne+ /*hIFLA_e O biL3MISSnad
& rla_total.)ct ( ct vxl_.s8.ne+ /*hIFLA_e O bint OAM
	 * procenad
& rla_total.)ct ( ct vxl_.sed.ne+ /*hIFLA_e O biAGEINGnad
& rla_total.)ct ( ct vxl_.sed.ne+ /*hIFLA_e O biIIMITnad
& rla_total.)ct ( ct vxlan_rdst*

	_(vxflaarp Nrange_ne+
& rla_total.)ct ( ct vxl_.;
16_p->d/*hIFLA_e O bi* renad
& rla_total.)ct ( ct vxl_.s8.ne+d/*hIFLA_e O bi
/* p;
}nad
& rla_total.)ct ( ct vxl_.s8.ne+d/*hIFLA_e O bi
/* ZERO_p;
}6_TXnad
& rla_total.)ct ( ct vxl_.s8.ne+d/*hIFLA_e O bi
/* ZERO_p;
}6_RXnad
& rla_total.)ct ( ct vxl_.s8.ne+d/*hIFLA_e O biR pp;
}
TXnad
& rla_total.)ct ( ct vxl_.s8.ne+d/*hIFLA_e O biR pp;
}
RXnad
& ruct vxlan_rdkvn sizeoffill>xlanrn	returfg.flags & V,datelimMSGSIZEue = get_uNOARP) &&
atelimMSGSIZEIS to %pIS\n",
				    src_mac, &rdst-atelimMSGSIZEIS to ev))
	 sk= nrF_INE
	return ret;st->remote 

	_(vxflaarp Nrange = 1)sb(v = 	.low = sremote(vxfl
	ted(skb-
min) ru .vighelsremote(vxfl
	ted(skb-
max) ru}		ul_INIula_aut.sed-& V,dIFLA_e O biIDT;be32;toxdex(rt vxlan_dev_prvsf
c	*gpe ula_aut.failur stru(f)) F_INE ;

	err =&ct vxlan_dev *.ne16 prn -Wt vxlan_dev * =s)
{
	size_h))->tos);
Eaddr(mtINIula_aut.		if (v-& V,dIFLA_e O biGROUP ru ttt g *Wt vxlan_dev * =) {
		if (vxlan_ge_lbac_r*gpe ula_aut.failur stel_notify_del_rx_port(vs->sock}",
					  ctINIula_aut.		6if (v-& V,dIFLA_e O biGROUP6ourn 0; g *n&Wt vxlan_dev * =) t net_devic)lbac_r*gpe ula_aut.failur stese
			n	}p;return -W				 vxlan->cfg.dsh&& ula_aut.sed-& V,dIFLA_e O biIINK, 					 vxlan->cfg.ds)f
c	*gpe ula_aut.failur stru(f)) F_INE ;

	err =&(vxfl
	ted(sfset)Eaddr(_INIIS to
	ted(sfset =s)
{
	size_h))->tos);
Eaddr(mtINIula_aut.		if (v-& V,dIFLA_e O biIpin_ourn 0; g *IS to
	ted(sfset =) {
		if (vxlan_ge_lbac_r*gpe ula_aut.failur stel_notify_del_rx_port(vs->sock}",
					  ctINIula_aut.		6if (v-& V,dIFLA_e O biIpin_6ourn 0; g *n&IS to
	ted(sfset =) t net_devic)lbac_r*gpe ula_aut.failur stese
			n	}p;return -ula_aut.s8-& V,dIFLA_e O biTTL
datINE
	ted(ttlf _fdbg *nula_aut.s8-& V,dIFLA_e O biTOS
datINE
	ted(tosf _fdbg *nula_aut.;
#e(& V,dIFLA_e O biIABEL
datINE
	ted(labelf _fdbg *nula_aut.s8-& V,dIFLA_e O biLpe->INGourn !!(atINE
	ted("non-ECT from %pLpe->)f _fdbg *nula_aut.s8-& V,dIFLA_e O biPROXYourn !!(atINE
	ted("non-ECT from %pPROXY)f _fdbg *nula_aut.s8-& V,dIFLA_e O biRSCourn g *nd !!(atINE
	ted("non-ECT from %pRSC)f _fdbg *nula_aut.s8-& V,dIFLA_e O biL2MISSourn !!(atINE
	ted("non-ECT from %pL2MISS)f _fdbg *nula_aut.s8-& V,dIFLA_e O biL3MISSourn !!(atINE
	ted("non-ECT from %pL3MISS)f _fdbg *nula_aut.s8-& V,dIFLA_e O bint OAM
	 * proceourn g *nd !!(atINE
	ted("non-ECT from %pnt OAM
	 * procesEa_fdbg *nula_aut.s#e(& V,dIFLA_e O biAGEING
datINE
	ted(age_tructval
a_fdbg *nula_aut.s#e(& V,dIFLA_e O biIIMIT
datINE
	ted(asetmasf _fdbg *nula_aut.;
16-& V,dIFLA_e O biP re,  _INE
	ted(d) == 1)f _fdbg *nula_aut.s8-& V,dIFLA_e O bi
/* p;
}ourn !(atINE
	ted("non-ECT from %p
/* ZERO_p;
}_TX)f _fdbg *nula_aut.s8-& V,dIFLA_e O bi
/* ZERO_p;
}6_TXourn !!(atINE
	ted("non-ECT from %p
/* ZERO_p;
}6_TX)f _fdbg *nula_aut.s8-& V,dIFLA_e O bi
/* ZERO_p;
}6_RXourn !!(atINE
	ted("non-ECT from %p
/* ZERO_p;
}6_RX)f _fdbg *nula_aut.s8-& V,dIFLA_e O biR pp;
}
TXourn !!(atINE
	ted("non-ECT from %pR pp;
}
TX)f _fdbg *nula_aut.s8-& V,dIFLA_e O biR pp;
}
RXourn !!(atINE
	ted("non-ECT from %pR pp;
}
RXvsf
c	*gpe ula_aut.failur stru(f))ula_aut-& V,dIFLA_e O biP re_RANGEue ct vxl= 1)s), &= 1)s)f
c	*gpe ula_aut.failur stru(f))atINE
	ted("non-ECT from %pGBP 					   ula_aut.fnon-& V,dIFLA_e O biGBP)f
c	*gpe ula_aut.failur stru(f))atINE
	ted("non-ECT from %pGPE 					   ula_aut.fnon-& V,dIFLA_e O biGPE)f
c	*gpe ula_aut.failur stru(f))atINE
	ted("non-ECT from %pn pp;
}
NOpolicy_ 					   ula_aut.fnon-& V,dIFLA_e O bin pp;
}
NOpolicy_)f
c	*gpe ula_aut.failur stru)
{
	struc
ula_aut.failur :

)
{
	st-EMSGSIZE;= rtnl_dere SGSIZEuk 		IS to r t.link_na_patelimMSGSIZEue = get_uNOARP) &&
from %pIS to %pIS\n",
				    src_mac, &rdst u sk_bufvtelimin_f;= rtnl_dere SGSIZErtnlclink_opsbIS to link_opsb__p(st_moste_h)-&&
.kfg.rr=d"ipv6_",
	.mas	if 	=dIFLA_e O biMAXour.arsed;_r=dipv6_sarsed;our.amac; ct 	h_ ct vxlan_rdstIS to %pI) ru., vut_r=dipv6_ss vut ru.deref16 r=dipv6_sderef16 ,= .newSinkr=dipv6_snewSink,= .changeSinkr=dipv6_schangeSink,= .dslSink	=!(vxfladslSink,= .r t. ct 	h_IS to r t.)ct ,= .fill>xlanr=dipv6_s ill>xlan,= .r t.Sink_na_	h_IS to r t.Sink_na_,=x_vnxlGSIZEue = get_uNOrace b srccn_fdb_hremoteue {
n fneatelimchar{
name ru tt g *u8 namecasse d_	if  ru tt g *xlan->c_familctefig *atef) &&
from %pulattr *tb[IFLA_MAX->doT;t->remoteue = get_unadtvFI-truct vxl	uo nla_p&tbfailure;

	i)b)dst
	dNEly rtnlccn_fdb Sink;n fnename  namecasse d_	if  ru t g *nd &IS to link_opsla	bpar	n_dsmetaRR	%pI)b->aremote_dtvFIruImily __race b srccn_fdb_n_fdb(&vf atef*)ence(vxcn_ds< cn<(sk			  o_rds    sr dtva;ru ,
					ERR_PTR(raw_on	xlanImily rtnlcctefigur .Sink;dtvf ence(vxcn_ds< cn<(sk			  LISTilEAD
if (!kill)st
	 ateliadslSink;(&vf &if (!kill)st0;NDregIS_er ock(&vird
mar =&if (!kill)st0;,
					ERR_PTR(raw_on	xlanremote_dtvFI}
EXP re_SYMBOLiGPL(race b srccn_fdb);rtnl_dereif (!(vxflahr_el6_	iwerdtv_NDregIS_erlan_rdstIS to ock_*v	ourn 0; g *nMSGSIZEue = get_uNOARP) &&
from %pIS to %pIS\n",
	,{
n x;st-LISTilEAD
if (!kill)st
	if (!vs)
		return fvsafb_atelilu_addt ocn &(vxflaree(db_addf			  MSGSIZEIS to ev))
	 sk= nrF_INE
	return ret;st
  /*hIna	kfe wescn_fdb(!(vxfl | NUD_ withdcarrier
XLAlar_eqwemloose
IP6hcarrier due	strmodule bersad
XLAlawemalsorneed	str vxlve!(vxfl | NUD_.hInao(f);
XLAla	kfeslait';
notinecessary r_eq}

sta->cfg.ds
XLAlais 0qf);edbfl
no matches.
XLAld
& vs->					 vxlan->cfg.dsh)r(vNEIGicfg.ds)
c		ateliadslSink;F_INE
	revf &if (!kill)st0}l	usDregIS_er ock(&vird
mar =&if (!kill)st rtnl_derekvn sizeofnek(&vird
evurn_hremoteuotifier bl, strunusedf->vvt iDbe dropST;
 evurn,eif (!*ptr) &&
from %pue = get_uNOARP			    srcuotifier xlan_tomreq ptr)st->remotening(dock_*v				   _gev);icCd&vxlan-ock)hrning(dock

	/* xcn_ds<vurnh)r();
DEV_UNREGISTERf			  ning(dGBP))
	.rx_= 1)s;revf net/i);) 	(vxflahr_el6_	iwerdtv_NDregIS_erlv6_aaev);ru}",
				INI<vurnh)r();
DEV_REGISTERf			  ning(dGBP))
	.rx_= 1)s;revf _buff *	}",
				INI<vurnh)r();
DEV_
/* Update PUSH_INFO _fdb_not<vurnh)r();
DEV_
/* Update DROP_INFOf			  ning(dGBP))
	.rx_= 1)s;revf <vurnh)r();
DEV_
/* Update PUSH_INFO_on	xlanremote_NOTIFY_DONE;d rtnl_dere SGSIZEuotifier bl, stning(dootifier bl, st__p(st_moste_h)-&&
.ootifier callly == AF_nek(&vird
evurn,=x_vnxlan_rd_dock

nitekvn sizeoftdidrlan-hremoteue {
n f) &&
from %pIS to ock_*v				   _gev);icCn_fdbIS to ock

	/* d	 be dropped.hstruINIT_LISTilEAD
ocn &(vxflaree();o 
k_rershi_onit;ocn &= ~Vock);
		uloan_sh

	f  hlik* re_HASH_SIZE; ++hEsk{INIT_HLISTilEAD
ocn &= ~Vocf ([hTEstru_sock *0 = rtnl_dereif (!_dock
exi%pIS to exi%rlan-hremoteue {
n f) &&
from %pIS to ock_*v				   _gev);icCn_fdbIS to ock

	/* dfrom %pIS to %pIS\n",
	,{
n x;st-MSGSIZEuk = get_unaligne*auxst-LISTilEAD
if (Estru_tnlclshi
/* dvs)
		ret    srcsafb_n_fdb(&vf aux)
& vs->	NEIGrtnlclink_opsb== &IS to link_ops)
c		NDregIS_er ock(&vird_queud;aevdb&if (Estruif (!vs)
		return fvsafb_atelilu_addt ocn &(vxflaree(db_addf			  /*hIf  _INE
	revais pedtheuxam (ocknslait has rlp(stytbet sadded
XLAlastrtheulIS_{by theupr&viousmloop.
XLAld
& vs->!ock
eqCd&vxlan- _INE
	rev)db_at))
c		NDregIS_er ock(&vird_queud;F_INE
	revf &if ()st0}l	usDregIS_er ock(&vird
mar =&if ();) rtnlcbershi
);d rtnl_dere SGSIZEpdst->_opern->hasbIS to ock.opsb(v = .
nite= sizeoftdidrlan,= .exi%p=pIS to exi%rlan,= . (!  rt&IS to ock

	 ru.,e;
t(r ct vxlan_rdstIS to _at),=x_vnxlan_rdkvn _

nitesizeoftdidrmodule(if () &&
	rucrcstrur t.ranrom_bytes=&(vxflvsal ue ct vxl(vxflvsal )dst
	rcly regIS_er pdst->_subsys=&(vxflvock.ops);) ,
		xcf
c	*gpe out1st
	rcly regIS_er ock(&vird
ootifier=&(vxflvootifier bl, s);) ,
		xcf
c	*gpe out2st
	rcly rtnlclink_regIS_erl&IS to link_ops);) ,
		xcf
c	*g