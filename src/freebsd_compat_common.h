/******************************************************************************

  Copyright (c) 2001-2017, Intel Corporation
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

   3. Neither the name of the Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

******************************************************************************/
/*$FreeBSD$*/

/**
 * @file freebsd_compat_common.h
 * @brief FreeBSD kernel compatibility macros used by Intel drivers
 *
 * Contains macros and function backports used to enable compatibility with
 * a variety of FreeBSD kernel versions.
 *
 * iflib or legacy driver specific compatibility definitions can be found in
 * the freebsd_compat_iflib.[ch] and freebsd_compat_legacy.[ch] files.
 */
#ifndef _FREEBSD_COMPAT_COMMON_H_
#define _FREEBSD_COMPAT_COMMON_H_

#include <sys/types.h>
#include <sys/param.h>
#include <sys/socket.h>
#include <net/if.h>
#include <net/if_var.h>

#if ((__FreeBSD_version >= 1102505 && __FreeBSD_version < 1200000) || \
     __FreeBSD_version >= 1200062)
/* In r339639, the FreeBSD netmap code modified the ring array in the
 * netmap_adapter to be an array of pointers. This was MFC to FreeBSD
 * 11.2 as r341477
 */
#define HAVE_NETMAP_RINGS_ARRAY_OF_POINTERS
#endif

#ifndef ETHER_IS_BROADCAST
#define	ETHER_IS_BROADCAST(addr) \
	(((addr)[0] & (addr)[1] & (addr)[2] & \
	  (addr)[3] & (addr)[4] & (addr)[5]) == 0xff)
#endif

#ifndef ETHER_IS_ZERO
#define	ETHER_IS_ZERO(addr) \
	(((addr)[0] | (addr)[1] | (addr)[2] | \
	  (addr)[3] | (addr)[4] | (addr)[5]) == 0x00)
#endif

#ifndef USEC_2_TICKS
#define USEC_2_TICKS(u) max(1, (uint32_t)((hz == 1000) ? \
	  ((u) / 1000) : ((uint64_t)(u) * (uint64_t)hz)/(uint64_t)1000000))
#endif
#ifndef MSEC_2_TICKS
#define MSEC_2_TICKS(m) max(1, (uint32_t)((hz == 1000) ? \
	  (m) : ((uint64_t)(m) * (uint64_t)hz)/(uint64_t)1000))
#endif
#ifndef TICKS_2_MSEC
#define TICKS_2_MSEC(t) max(1, (uint32_t)(hz == 1000) ? \
          (t) : (((uint64_t)(t) * (uint64_t)1000)/(uint64_t)hz))
#endif
#ifndef TICKS_2_USEC
#define TICKS_2_USEC(t) max(1, (uint32_t)(hz == 1000) ? \
          ((t) * 1000) : (((uint64_t)(t) * (uint64_t)1000000)/(uint64_t)hz))
#endif

#if __FreeBSD_version <= 1100000
/**
 * uqmax - Find the maximum of two u_quad_t values
 * @a: first value to compare
 * @b: second value to compare
 *
 * Backport of uqmax function to determine the maximum value of a u_quad
 * value. This released originally in FreeBSD 11
 */
static __inline u_quad_t uqmax(u_quad_t a, u_quad_t b)
{
	return (a > b ? a : b);
}
#endif

/* FreeBSD 11.2 fixed the typo in CSUM_COALESED */
#ifndef CSUM_COALESCED
#define CSUM_COALESCED CSUM_COALESED
#endif

/*
 * Old versions of <sys/bitstring.h> lack bit_ffs_area_at and bit_ffc_area_at.
 * Indicate that drivers should use the bitstring_compat.h when building on
 * these older kernels.
 */
#if __FreeBSD_version < 1300061
#define USE_BITSTRING_COMPAT
#endif

/* Older versions of FreeBSD do not have all the if_media type macros, so
 * we'll treat them as IFM_UNKNOWN.
 */
#ifndef IFM_10G_AOC
#define IFM_10G_AOC IFM_UNKNOWN
#endif

#ifndef IFM_25G_LR
#define IFM_25G_LR IFM_UNKNOWN
#endif

#ifndef IFM_25G_AOC
#define IFM_25G_AOC IFM_UNKNOWN
#endif

/*
 * Newer versions of FreeBSD introduced new KPI for accessing lists of
 * multicast addresses. The old KPI is eventually removed in 1300054, so we
 * provide the new implementation for older kernels that lack them.
 */
#if __FreeBSD_version < 1300051
/* Part of the new KPI was MFCed to FreeBSD 12.2 */
#if (__FreeBSD_version < 1202000 || __FreeBSD_version > 1300000 )
struct sockaddr_dl;
typedef u_int iflladdr_cb_t(void *, struct sockaddr_dl *, u_int);
u_int if_foreach_lladdr(if_t ifp, iflladdr_cb_t cb, void *cb_arg);
u_int if_foreach_llmaddr(if_t ifp, iflladdr_cb_t cb, void *cb_arg);
#endif
u_int if_lladdr_count(if_t ifp);
u_int if_llmaddr_count(if_t ifp);
#endif

/*
 * We can check for just IFM_100_SGMII since the following media types
 * were all added at the same time.
 */
#ifndef IFM_100_SGMII
#define	IFM_100_SGMII	IFM_UNKNOWN
#define	IFM_2500_X	IFM_UNKNOWN
#define	IFM_5000_KR	IFM_UNKNOWN
#define	IFM_25G_T	IFM_UNKNOWN
#define	IFM_25G_CR_S	IFM_UNKNOWN
#define	IFM_25G_CR1	IFM_UNKNOWN
#define	IFM_25G_KR_S	IFM_UNKNOWN
#define	IFM_5000_KR_S	IFM_UNKNOWN
#define	IFM_5000_KR1	IFM_UNKNOWN
#define	IFM_25G_AUI	IFM_UNKNOWN
#define	IFM_40G_XLAUI	IFM_UNKNOWN
#define	IFM_40G_XLAUI_AC IFM_UNKNOWN
#define	IFM_50G_SR2	IFM_UNKNOWN
#define	IFM_50G_LR2	IFM_UNKNOWN
#define	IFM_50G_LAUI2_AC IFM_UNKNOWN
#define	IFM_50G_LAUI2	IFM_UNKNOWN
#define	IFM_50G_AUI2_AC	IFM_UNKNOWN
#define	IFM_50G_AUI2	IFM_UNKNOWN
#define	IFM_50G_CP	IFM_UNKNOWN
#define	IFM_50G_SR	IFM_UNKNOWN
#define	IFM_50G_LR	IFM_UNKNOWN
#define	IFM_50G_FR	IFM_UNKNOWN
#define	IFM_50G_KR_PAM4	IFM_UNKNOWN
#define	IFM_25G_KR1	IFM_UNKNOWN
#define	IFM_50G_AUI1_AC	IFM_UNKNOWN
#define	IFM_50G_AUI1	IFM_UNKNOWN
#define	IFM_100G_CAUI4_AC IFM_UNKNOWN
#define	IFM_100G_CAUI4 IFM_UNKNOWN
#define	IFM_100G_AUI4_AC IFM_UNKNOWN
#define	IFM_100G_AUI4 IFM_UNKNOWN
#define	IFM_100G_CR_PAM4 IFM_UNKNOWN
#define	IFM_100G_KR_PAM4 IFM_UNKNOWN
#define	IFM_100G_CP2	IFM_UNKNOWN
#define	IFM_100G_SR2	IFM_UNKNOWN
#define	IFM_100G_DR	IFM_UNKNOWN
#define	IFM_100G_KR2_PAM4 IFM_UNKNOWN
#define	IFM_100G_CAUI2_AC IFM_UNKNOWN
#define	IFM_100G_CAUI2	IFM_UNKNOWN
#define	IFM_100G_AUI2_AC IFM_UNKNOWN
#define	IFM_100G_AUI2	IFM_UNKNOWN
#endif

#endif /* _FREEBSD_COMPAT_COMMON_H_ */
