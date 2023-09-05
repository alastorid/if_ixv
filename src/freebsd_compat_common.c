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
 * @file freebsd_compat_common.c
 * @brief FreeBSD kernel function backports
 *
 * Contains backports for functions declared in freebsd_compat_common.h, and
 * is used to aid in developing out-of-tree network drivers that remain
 * compile-time compatible across a variety of FreeBSD kernel versions.
 */

#include "freebsd_compat_common.h"

#if __FreeBSD_version < 1300051

/* newer versions of FreeBSD use CK_STAILQ to access if_multiaddrs */
#if __FreeBSD_version >= 1200063
#define IF_MADDRS_FOREACH	CK_STAILQ_FOREACH
#else
#define IF_MADDRS_FOREACH	TAILQ_FOREACH
#endif

u_int
if_lladdr_count(if_t ifp)
{
	struct ifaddr *ifa;
	u_int count;

	count = 0;
	if_addr_rlock(ifp);
	IF_MADDRS_FOREACH(ifa, &ifp->if_addrhead, ifa_link)
		if (ifa->ifa_addr->sa_family == AF_LINK)
			count++;
	if_addr_runlock(ifp);

	return (count);
}

u_int
if_llmaddr_count(if_t ifp)
{
	struct ifmultiaddr *ifma;
	int count;

	count = 0;
	if_maddr_rlock(ifp);
	IF_MADDRS_FOREACH(ifma, &ifp->if_multiaddrs, ifma_link)
		if (ifma->ifma_addr->sa_family == AF_LINK)
			count++;
	if_maddr_runlock(ifp);

	return (count);
}

#if (__FreeBSD_version < 1202000 || __FreeBSD_version > 1300000)
u_int
if_foreach_lladdr(if_t ifp, iflladdr_cb_t cb, void *cb_arg)
{
	struct ifaddr *ifa;
	u_int count;

	MPASS(cb);

	count = 0;
	if_addr_rlock(ifp);
	IF_MADDRS_FOREACH(ifa, &ifp->if_addrhead, ifa_link) {
		if (ifa->ifa_addr->sa_family != AF_LINK)
			continue;
		count += (*cb)(cb_arg, (struct sockaddr_dl *)ifa->ifa_addr,
		    count);
	}
	if_addr_runlock(ifp);

	return (count);
}

u_int
if_foreach_llmaddr(if_t ifp, iflladdr_cb_t cb, void *cb_arg)
{
	struct ifmultiaddr *ifma;
	u_int count;

	MPASS(cb);

	count = 0;
	if_maddr_rlock(ifp);
	IF_MADDRS_FOREACH(ifma, &ifp->if_multiaddrs, ifma_link) {
		if (ifma->ifma_addr->sa_family != AF_LINK)
			continue;
		count += (*cb)(cb_arg, (struct sockaddr_dl *)ifma->ifma_addr,
		    count);
	}
	if_maddr_runlock(ifp);

	return (count);
}
#endif /* __FreeBSD_version < 1202000 || __FreeBSD_version > 1300000 */
#endif /* FreeBSD_version < 1300051 */
