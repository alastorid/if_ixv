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


#ifndef IXGBE_STANDALONE_BUILD
#include "opt_inet.h"
#include "opt_inet6.h"
#include "opt_rss.h"
#endif

#include "ixgbe.h"

extern int ix_crcstrip;

/*
 * HW RSC control:
 *  this feature only works with
 *  IPv4, and only on 82599 and later.
 *  Also this will cause IP forwarding to
 *  fail and that can't be controlled by
 *  the stack as LRO can. For all these
 *  reasons I've deemed it best to leave
 *  this off and not bother with a tuneable
 *  interface, this would need to be compiled
 *  to enable.
 */
static bool ixgbe_rsc_enable = FALSE;

/*
 * For Flow Director: this is the
 * number of TX packets we sample
 * for the filter pool, this means
 * every 20th packet will be probed.
 *
 * This feature can be disabled by
 * setting this to 0.
 */
static int atr_sample_rate = 20;

/************************************************************************
 *  Local Function prototypes
 ************************************************************************/
static void          ixgbe_setup_transmit_ring(struct tx_ring *);
static void          ixgbe_free_transmit_buffers(struct tx_ring *);
static int           ixgbe_setup_receive_ring(struct rx_ring *);
static void          ixgbe_free_receive_buffers(struct rx_ring *);
static void          ixgbe_rx_checksum(u32, struct mbuf *, u32);
static void          ixgbe_refresh_mbufs(struct rx_ring *, int);
static int           ixgbe_xmit(struct tx_ring *, struct mbuf **);
static int           ixgbe_tx_ctx_setup(struct tx_ring *,
                                        struct mbuf *, u32 *, u32 *);
static int           ixgbe_tso_setup(struct tx_ring *,
                                     struct mbuf *, u32 *, u32 *);
static __inline void ixgbe_rx_discard(struct rx_ring *, int);
static __inline void ixgbe_rx_input(struct rx_ring *, struct ifnet *,
                                    struct mbuf *, u32);
static int           ixgbe_dma_malloc(struct ixgbe_softc *, bus_size_t,
                                      struct ixgbe_dma_alloc *, int);
static void          ixgbe_dma_free(struct ixgbe_softc *, struct ixgbe_dma_alloc *);

MALLOC_DECLARE(M_IXV);

/************************************************************************
 * ixgbe_legacy_start_locked - Transmit entry point
 *
 *   Called by the stack to initiate a transmit.
 *   The driver will remain in this routine as long as there are
 *   packets to transmit and transmit resources are available.
 *   In case resources are not available, the stack is notified
 *   and the packet is requeued.
 ************************************************************************/
int
ixgbe_legacy_start_locked(struct ifnet *ifp, struct tx_ring *txr)
{
	struct mbuf    *m_head;
	struct ixgbe_softc *sc = txr->sc;

	IXGBE_TX_LOCK_ASSERT(txr);

	if ((ifp->if_drv_flags & IFF_DRV_RUNNING) == 0)
		return (ENETDOWN);
	if (!sc->link_active)
		return (ENETDOWN);

	while (!IFQ_DRV_IS_EMPTY(&ifp->if_snd)) {
		if (txr->tx_avail <= IXGBE_QUEUE_MIN_FREE)
			break;

		IFQ_DRV_DEQUEUE(&ifp->if_snd, m_head);
		if (m_head == NULL)
			break;

		if (ixgbe_xmit(txr, &m_head)) {
			if (m_head != NULL)
				IFQ_DRV_PREPEND(&ifp->if_snd, m_head);
			break;
		}
		/* Send a copy of the frame to the BPF listener */
		ETHER_BPF_MTAP(ifp, m_head);
	}

	return IXGBE_SUCCESS;
} /* ixgbe_legacy_start_locked */

/************************************************************************
 * ixgbe_legacy_start
 *
 *   Called by the stack, this always uses the first tx ring,
 *   and should not be used with multiqueue tx enabled.
 ************************************************************************/
void
ixgbe_legacy_start(struct ifnet *ifp)
{
	struct ixgbe_softc *sc = ifp->if_softc;
	struct tx_ring *txr = sc->tx_rings;

	if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
		IXGBE_TX_LOCK(txr);
		ixgbe_legacy_start_locked(ifp, txr);
		IXGBE_TX_UNLOCK(txr);
	}
} /* ixgbe_legacy_start */

/************************************************************************
 * ixgbe_mq_start - Multiqueue Transmit Entry Point
 *
 *   (if_transmit function)
 ************************************************************************/
int
ixgbe_mq_start(struct ifnet *ifp, struct mbuf *m)
{
	struct ixgbe_softc  *sc = ifp->if_softc;
	struct ix_queue *que;
	struct tx_ring  *txr;
	int             i, err = 0;
	uint32_t        bucket_id;

	/*
	 * When doing RSS, map it to the same outbound queue
	 * as the incoming flow would be mapped to.
	 *
	 * If everything is setup correctly, it should be the
	 * same bucket that the current CPU we're on is.
	 */
	if (M_HASHTYPE_GET(m) != M_HASHTYPE_NONE) {
		if ((sc->feat_en & IXGBE_FEATURE_RSS) &&
		    (rss_hash2bucket(m->m_pkthdr.flowid, M_HASHTYPE_GET(m),
		    &bucket_id) == 0)) {
			i = bucket_id % sc->num_queues;
#ifdef IXGBE_DEBUG
			if (bucket_id > sc->num_queues)
				if_printf(ifp,
				    "bucket_id (%d) > num_queues (%d)\n",
				    bucket_id, sc->num_queues);
#endif
		} else
			i = m->m_pkthdr.flowid % sc->num_queues;
	} else
		i = curcpu % sc->num_queues;

	/* Check for a hung queue and pick alternative */
	if (((1 << i) & sc->active_queues) == 0)
		i = ffsl(sc->active_queues);

	txr = &sc->tx_rings[i];
	que = &sc->queues[i];

	err = drbr_enqueue(ifp, txr->br, m);
	if (err)
		return (err);
	if (IXGBE_TX_TRYLOCK(txr)) {
		ixgbe_mq_start_locked(ifp, txr);
		IXGBE_TX_UNLOCK(txr);
	} else
		taskqueue_enqueue(que->tq, &txr->txq_task);

	return (0);
} /* ixgbe_mq_start */

/************************************************************************
 * ixgbe_mq_start_locked
 ************************************************************************/
int
ixgbe_mq_start_locked(struct ifnet *ifp, struct tx_ring *txr)
{
	struct mbuf    *next;
	int            enqueued = 0, err = 0;

	if ((ifp->if_drv_flags & IFF_DRV_RUNNING) == 0)
		return (ENETDOWN);
	if (!txr->sc->link_active)
		return (ENETDOWN);

	/* Process the queue */
	while ((next = drbr_peek(ifp, txr->br)) != NULL) {
		err = ixgbe_xmit(txr, &next);
		if (err != 0) {
			if (next == NULL)
				drbr_advance(ifp, txr->br);
			else
				drbr_putback(ifp, txr->br, next);
			break;
		}
		drbr_advance(ifp, txr->br);
		enqueued++;
		/*
		 * Since we're looking at the tx ring, we can check
		 * to see if we're a VF by examing our tail register
		 * address.
		 */
		if ((txr->sc->feat_en & IXGBE_FEATURE_VF) &&
		    (next->m_flags & M_MCAST))
			if_inc_counter(ifp, IFCOUNTER_OMCASTS, 1);

		/* Send a copy of the frame to the BPF listener */
		ETHER_BPF_MTAP(ifp, next);
		if ((ifp->if_drv_flags & IFF_DRV_RUNNING) == 0)
			break;
	}

	if (txr->tx_avail < IXGBE_TX_CLEANUP_THRESHOLD(txr->sc))
		ixgbe_txeof(txr);

	return (err);
} /* ixgbe_mq_start_locked */

/************************************************************************
 * ixgbe_deferred_mq_start
 *
 *   Called from a taskqueue to drain queued transmit packets.
 ************************************************************************/
void
ixgbe_deferred_mq_start(void *arg, int pending)
{
	struct tx_ring *txr = arg;
	struct ixgbe_softc *sc = txr->sc;
	struct ifnet   *ifp = sc->ifp;

	IXGBE_TX_LOCK(txr);
	if (!drbr_empty(ifp, txr->br))
		ixgbe_mq_start_locked(ifp, txr);
	IXGBE_TX_UNLOCK(txr);
} /* ixgbe_deferred_mq_start */

/************************************************************************
 * ixgbe_qflush - Flush all ring buffers
 ************************************************************************/
void
ixgbe_qflush(struct ifnet *ifp)
{
	struct ixgbe_softc *sc = ifp->if_softc;
	struct tx_ring *txr = sc->tx_rings;
	struct mbuf    *m;

	for (int i = 0; i < sc->num_queues; i++, txr++) {
		IXGBE_TX_LOCK(txr);
		while ((m = buf_ring_dequeue_sc(txr->br)) != NULL)
			m_freem(m);
		IXGBE_TX_UNLOCK(txr);
	}
	if_qflush(ifp);
} /* ixgbe_qflush */


/************************************************************************
 * ixgbe_xmit
 *
 *   Maps the mbufs to tx descriptors, allowing the
 *   TX engine to transmit the packets.
 *
 *   Return 0 on success, positive on failure
 ************************************************************************/
static int
ixgbe_xmit(struct tx_ring *txr, struct mbuf **m_headp)
{
	struct ixgbe_softc          *sc = txr->sc;
	struct ixgbe_tx_buf     *txbuf;
	union ixgbe_adv_tx_desc *txd = NULL;
	struct mbuf             *m_head;
	int                     i, j, error, nsegs;
	int                     first;
	u32                     olinfo_status = 0, cmd_type_len;
	bool                    remap = TRUE;
	bus_dma_segment_t       segs[sc->num_segs];
	bus_dmamap_t            map;

	m_head = *m_headp;

	/* Basic descriptor defines */
	cmd_type_len = (IXGBE_ADVTXD_DTYP_DATA |
	    IXGBE_ADVTXD_DCMD_IFCS | IXGBE_ADVTXD_DCMD_DEXT);

	if (m_head->m_flags & M_VLANTAG)
		cmd_type_len |= IXGBE_ADVTXD_DCMD_VLE;

	/*
	 * Important to capture the first descriptor
	 * used because it will contain the index of
	 * the one we tell the hardware to report back
	 */
	first = txr->next_avail_desc;
	txbuf = &txr->tx_buffers[first];
	map = txbuf->map;

	/*
	 * Map the packet for DMA.
	 */
retry:
	error = bus_dmamap_load_mbuf_sg(txr->txtag, map, *m_headp, segs,
	    &nsegs, BUS_DMA_NOWAIT);

	if (__predict_false(error)) {
		struct mbuf *m;

		switch (error) {
		case EFBIG:
			/* Try it again? - one try */
			if (remap == TRUE) {
				remap = FALSE;
				/*
				 * XXX: m_defrag will choke on
				 * non-MCLBYTES-sized clusters
				 */
				m = m_defrag(*m_headp, M_NOWAIT);
				if (m == NULL) {
					sc->mbuf_defrag_failed++;
					m_freem(*m_headp);
					*m_headp = NULL;
					return (ENOBUFS);
				}
				*m_headp = m;
				goto retry;
			} else
				return (error);
		case ENOMEM:
			txr->no_tx_dma_setup++;
			return (error);
		default:
			txr->no_tx_dma_setup++;
			m_freem(*m_headp);
			*m_headp = NULL;
			return (error);
		}
	}

	/* Make certain there are enough descriptors */
	if (txr->tx_avail < (nsegs + 2)) {
		txr->no_desc_avail++;
		bus_dmamap_unload(txr->txtag, map);
		return (ENOBUFS);
	}
	m_head = *m_headp;

	/*
	 * Set up the appropriate offload context
	 * this will consume the first descriptor
	 */
	error = ixgbe_tx_ctx_setup(txr, m_head, &cmd_type_len, &olinfo_status);
	if (__predict_false(error)) {
		if (error == ENOBUFS)
			*m_headp = NULL;
		return (error);
	}

	/* Do the flow director magic */
	if ((sc->feat_en & IXGBE_FEATURE_FDIR) &&
	    (txr->atr_sample) && (!sc->fdir_reinit)) {
		++txr->atr_count;
		if (txr->atr_count >= atr_sample_rate) {
			ixgbe_atr(txr, m_head);
			txr->atr_count = 0;
		}
	}

	olinfo_status |= IXGBE_ADVTXD_CC;
	i = txr->next_avail_desc;
	for (j = 0; j < nsegs; j++) {
		bus_size_t seglen;
		bus_addr_t segaddr;

		txbuf = &txr->tx_buffers[i];
		txd = &txr->tx_base[i];
		seglen = segs[j].ds_len;
		segaddr = htole64(segs[j].ds_addr);

		txd->read.buffer_addr = segaddr;
		txd->read.cmd_type_len = htole32(txr->txd_cmd |
		    cmd_type_len | seglen);
		txd->read.olinfo_status = htole32(olinfo_status);

		if (++i == txr->num_desc)
			i = 0;
	}

	txd->read.cmd_type_len |= htole32(IXGBE_TXD_CMD_EOP | IXGBE_TXD_CMD_RS);
	txr->tx_avail -= nsegs;
	txr->next_avail_desc = i;

	txbuf->m_head = m_head;
	/*
	 * Here we swap the map so the last descriptor,
	 * which gets the completion interrupt has the
	 * real map, and the first descriptor gets the
	 * unused map from this descriptor.
	 */
	txr->tx_buffers[first].map = txbuf->map;
	txbuf->map = map;
	bus_dmamap_sync(txr->txtag, map, BUS_DMASYNC_PREWRITE);

	/* Set the EOP descriptor that will be marked done */
	txbuf = &txr->tx_buffers[first];
	txbuf->eop = txd;

	bus_dmamap_sync(txr->txdma.dma_tag, txr->txdma.dma_map,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
	/*
	 * Advance the Transmit Descriptor Tail (Tdt), this tells the
	 * hardware that this frame is available to transmit.
	 */
	++txr->total_packets;
	IXGBE_WRITE_REG(&sc->hw, txr->tail, i);

	/* Mark queue as having work */
	if (txr->busy == 0)
		txr->busy = 1;

	return (0);
} /* ixgbe_xmit */


/************************************************************************
 * ixgbe_allocate_transmit_buffers
 *
 *   Allocate memory for tx_buffer structures. The tx_buffer stores all
 *   the information needed to transmit a packet on the wire. This is
 *   called only once at attach, setup is done every reset.
 ************************************************************************/
static int
ixgbe_allocate_transmit_buffers(struct tx_ring *txr)
{
	struct ixgbe_softc      *sc = txr->sc;
	device_t            dev = sc->dev;
	struct ixgbe_tx_buf *txbuf;
	int                 error, i;

	/*
	 * Setup DMA descriptor areas.
	 */
	error = bus_dma_tag_create(
	         /*      parent */ bus_get_dma_tag(sc->dev),
	         /*   alignment */ 1,
	         /*      bounds */ 0,
	         /*     lowaddr */ BUS_SPACE_MAXADDR,
	         /*    highaddr */ BUS_SPACE_MAXADDR,
	         /*      filter */ NULL,
	         /*   filterarg */ NULL,
	         /*     maxsize */ IXGBE_TSO_SIZE,
	         /*   nsegments */ sc->num_segs,
	         /*  maxsegsize */ PAGE_SIZE,
	         /*       flags */ 0,
	         /*    lockfunc */ NULL,
	         /* lockfuncarg */ NULL,
	                           &txr->txtag);
	if (error) {
		device_printf(dev, "Unable to allocate TX DMA tag\n");
		goto fail;
	}

	txr->tx_buffers =
	    (struct ixgbe_tx_buf *)malloc(sizeof(struct ixgbe_tx_buf) *
	    sc->num_tx_desc, M_IXV, M_NOWAIT | M_ZERO);
	if (!txr->tx_buffers) {
		device_printf(dev, "Unable to allocate tx_buffer memory\n");
		error = ENOMEM;
		goto fail;
	}

	/* Create the descriptor buffer dma maps */
	txbuf = txr->tx_buffers;
	for (i = 0; i < sc->num_tx_desc; i++, txbuf++) {
		error = bus_dmamap_create(txr->txtag, 0, &txbuf->map);
		if (error != 0) {
			device_printf(dev, "Unable to create TX DMA map\n");
			goto fail;
		}
	}

	return 0;
fail:
	/* We free all, it handles case where we are in the middle */
	ixgbe_free_transmit_structures(sc);

	return (error);
} /* ixgbe_allocate_transmit_buffers */

/************************************************************************
 * ixgbe_setup_transmit_ring - Initialize a transmit ring.
 ************************************************************************/
static void
ixgbe_setup_transmit_ring(struct tx_ring *txr)
{
	struct ixgbe_softc        *sc = txr->sc;
	struct ixgbe_tx_buf   *txbuf;
#ifdef DEV_NETMAP
	struct netmap_sc *na = NA(sc->ifp);
	struct netmap_slot    *slot;
#endif /* DEV_NETMAP */

	/* Clear the old ring contents */
	IXGBE_TX_LOCK(txr);

#ifdef DEV_NETMAP
	if (sc->feat_en & IXGBE_FEATURE_NETMAP) {
		/*
		 * (under lock): if in netmap mode, do some consistency
		 * checks and set slot to entry 0 of the netmap ring.
		 */
		slot = netmap_reset(na, NR_TX, txr->me, 0);
	}
#endif /* DEV_NETMAP */

	bzero((void *)txr->tx_base,
	    (sizeof(union ixgbe_adv_tx_desc)) * sc->num_tx_desc);
	/* Reset indices */
	txr->next_avail_desc = 0;
	txr->next_to_clean = 0;

	/* Free any existing tx buffers. */
	txbuf = txr->tx_buffers;
	for (int i = 0; i < txr->num_desc; i++, txbuf++) {
		if (txbuf->m_head != NULL) {
			bus_dmamap_sync(txr->txtag, txbuf->map,
			    BUS_DMASYNC_POSTWRITE);
			bus_dmamap_unload(txr->txtag, txbuf->map);
			m_freem(txbuf->m_head);
			txbuf->m_head = NULL;
		}

#ifdef DEV_NETMAP
		/*
		 * In netmap mode, set the map for the packet buffer.
		 * NOTE: Some drivers (not this one) also need to set
		 * the physical buffer address in the NIC ring.
		 * Slots in the netmap ring (indexed by "si") are
		 * kring->nkr_hwofs positions "ahead" wrt the
		 * corresponding slot in the NIC ring. In some drivers
		 * (not here) nkr_hwofs can be negative. Function
		 * netmap_idx_n2k() handles wraparounds properly.
		 */
		if ((sc->feat_en & IXGBE_FEATURE_NETMAP) && slot) {
#if ((__FreeBSD_version >= 1102505 && __FreeBSD_version < 1200000) || \
    __FreeBSD_version >= 1200062)
			int si = netmap_idx_n2k(na->tx_rings[txr->me], i);
#else
			int si = netmap_idx_n2k(&na->tx_rings[txr->me], i);
#endif
			netmap_load_map(na, txr->txtag,
			    txbuf->map, NMB(na, slot + si));
		}
#endif /* DEV_NETMAP */

		/* Clear the EOP descriptor pointer */
		txbuf->eop = NULL;
	}

	/* Set the rate at which we sample packets */
	if (sc->feat_en & IXGBE_FEATURE_FDIR)
		txr->atr_sample = atr_sample_rate;

	/* Set number of descriptors available */
	txr->tx_avail = sc->num_tx_desc;

	bus_dmamap_sync(txr->txdma.dma_tag, txr->txdma.dma_map,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
	IXGBE_TX_UNLOCK(txr);
} /* ixgbe_setup_transmit_ring */

/************************************************************************
 * ixgbe_setup_transmit_structures - Initialize all transmit rings.
 ************************************************************************/
int
ixgbe_setup_transmit_structures(struct ixgbe_softc *sc)
{
	struct tx_ring *txr = sc->tx_rings;

	for (int i = 0; i < sc->num_queues; i++, txr++)
		ixgbe_setup_transmit_ring(txr);

	return (0);
} /* ixgbe_setup_transmit_structures */

/************************************************************************
 * ixgbe_free_transmit_structures - Free all transmit rings.
 ************************************************************************/
void
ixgbe_free_transmit_structures(struct ixgbe_softc *sc)
{
	struct tx_ring *txr = sc->tx_rings;

	for (int i = 0; i < sc->num_queues; i++, txr++) {
		IXGBE_TX_LOCK(txr);
		ixgbe_free_transmit_buffers(txr);
		ixgbe_dma_free(sc, &txr->txdma);
		IXGBE_TX_UNLOCK(txr);
		IXGBE_TX_LOCK_DESTROY(txr);
	}
	free(sc->tx_rings, M_IXV);
} /* ixgbe_free_transmit_structures */

/************************************************************************
 * ixgbe_free_transmit_buffers
 *
 *   Free transmit ring related data structures.
 ************************************************************************/
static void
ixgbe_free_transmit_buffers(struct tx_ring *txr)
{
	struct ixgbe_softc      *sc = txr->sc;
	struct ixgbe_tx_buf *tx_buffer;
	int                 i;

	INIT_DEBUGOUT("ixgbe_free_transmit_ring: begin");

	if (txr->tx_buffers == NULL)
		return;

	tx_buffer = txr->tx_buffers;
	for (i = 0; i < sc->num_tx_desc; i++, tx_buffer++) {
		if (tx_buffer->m_head != NULL) {
			bus_dmamap_sync(txr->txtag, tx_buffer->map,
			    BUS_DMASYNC_POSTWRITE);
			bus_dmamap_unload(txr->txtag, tx_buffer->map);
			m_freem(tx_buffer->m_head);
			tx_buffer->m_head = NULL;
			if (tx_buffer->map != NULL) {
				bus_dmamap_destroy(txr->txtag, tx_buffer->map);
				tx_buffer->map = NULL;
			}
		} else if (tx_buffer->map != NULL) {
			bus_dmamap_unload(txr->txtag, tx_buffer->map);
			bus_dmamap_destroy(txr->txtag, tx_buffer->map);
			tx_buffer->map = NULL;
		}
	}
	if (txr->br != NULL)
		buf_ring_free(txr->br, M_IXV);
	if (txr->tx_buffers != NULL) {
		free(txr->tx_buffers, M_IXV);
		txr->tx_buffers = NULL;
	}
	if (txr->txtag != NULL) {
		bus_dma_tag_destroy(txr->txtag);
		txr->txtag = NULL;
	}
} /* ixgbe_free_transmit_buffers */

/************************************************************************
 * ixgbe_tx_ctx_setup
 *
 *   Advanced Context Descriptor setup for VLAN, CSUM or TSO
 ************************************************************************/
static int
ixgbe_tx_ctx_setup(struct tx_ring *txr, struct mbuf *mp,
    u32 *cmd_type_len, u32 *olinfo_status)
{
	struct ixgbe_adv_tx_context_desc *TXD;
	struct ether_vlan_header         *eh;
#ifdef INET
	struct ip                        *ip;
#endif
#ifdef INET6
	struct ip6_hdr                   *ip6;
#endif
	int                              ehdrlen, ip_hlen = 0;
	int                              offload = TRUE;
	int                              ctxd = txr->next_avail_desc;
	u32                              vlan_macip_lens = 0;
	u32                              type_tucmd_mlhl = 0;
	u16                              vtag = 0;
	u16                              etype;
	u8                               ipproto = 0;
	caddr_t                          l3d;


	/* First check if TSO is to be used */
	if (mp->m_pkthdr.csum_flags & (CSUM_IP_TSO | CSUM_IP6_TSO))
		return (ixgbe_tso_setup(txr, mp, cmd_type_len, olinfo_status));

	if ((mp->m_pkthdr.csum_flags & CSUM_OFFLOAD) == 0)
		offload = FALSE;

	/* Indicate the whole packet as payload when not doing TSO */
	*olinfo_status |= mp->m_pkthdr.len << IXGBE_ADVTXD_PAYLEN_SHIFT;

	/* Now ready a context descriptor */
	TXD = (struct ixgbe_adv_tx_context_desc *)&txr->tx_base[ctxd];

	/*
	 * In advanced descriptors the vlan tag must
	 * be placed into the context descriptor. Hence
	 * we need to make one even if not doing offloads.
	 */
	if (mp->m_flags & M_VLANTAG) {
		vtag = htole16(mp->m_pkthdr.ether_vtag);
		vlan_macip_lens |= (vtag << IXGBE_ADVTXD_VLAN_SHIFT);
	} else if (!(txr->sc->feat_en & IXGBE_FEATURE_NEEDS_CTXD) &&
	           (offload == FALSE))
		return (0);

	/*
	 * Determine where frame payload starts.
	 * Jump over vlan headers if already present,
	 * helpful for QinQ too.
	 */
	eh = mtod(mp, struct ether_vlan_header *);
	if (eh->evl_encap_proto == htons(ETHERTYPE_VLAN)) {
		etype = ntohs(eh->evl_proto);
		ehdrlen = ETHER_HDR_LEN + ETHER_VLAN_ENCAP_LEN;
	} else {
		etype = ntohs(eh->evl_encap_proto);
		ehdrlen = ETHER_HDR_LEN;
	}

	/* Set the ether header length */
	vlan_macip_lens |= ehdrlen << IXGBE_ADVTXD_MACLEN_SHIFT;

	if (offload == FALSE)
		goto no_offloads;

	/*
	 * If the first mbuf only includes the ethernet header,
	 * jump to the next one
	 * XXX: This assumes the stack splits mbufs containing headers
	 *      on header boundaries
	 * XXX: And assumes the entire IP header is contained in one mbuf
	 */
	if (mp->m_len == ehdrlen && mp->m_next)
		l3d = mtod(mp->m_next, caddr_t);
	else
		l3d = mtod(mp, caddr_t) + ehdrlen;

	switch (etype) {
#ifdef INET
		case ETHERTYPE_IP:
			ip = (struct ip *)(l3d);
			ip_hlen = ip->ip_hl << 2;
			ipproto = ip->ip_p;
			type_tucmd_mlhl |= IXGBE_ADVTXD_TUCMD_IPV4;
			/* Insert IPv4 checksum into data descriptors */
			if (mp->m_pkthdr.csum_flags & CSUM_IP) {
				ip->ip_sum = 0;
				*olinfo_status |= IXGBE_TXD_POPTS_IXSM << 8;
			}
			break;
#endif
#ifdef INET6
		case ETHERTYPE_IPV6:
			ip6 = (struct ip6_hdr *)(l3d);
			ip_hlen = sizeof(struct ip6_hdr);
			ipproto = ip6->ip6_nxt;
			type_tucmd_mlhl |= IXGBE_ADVTXD_TUCMD_IPV6;
			break;
#endif
		default:
			offload = FALSE;
			break;
	}

	vlan_macip_lens |= ip_hlen;

	/* No support for offloads for non-L4 next headers */
	switch (ipproto) {
		case IPPROTO_TCP:
			if (mp->m_pkthdr.csum_flags &
			    (CSUM_IP_TCP | CSUM_IP6_TCP))
				type_tucmd_mlhl |= IXGBE_ADVTXD_TUCMD_L4T_TCP;
			else
				offload = false;
			break;
		case IPPROTO_UDP:
			if (mp->m_pkthdr.csum_flags &
			    (CSUM_IP_UDP | CSUM_IP6_UDP))
				type_tucmd_mlhl |= IXGBE_ADVTXD_TUCMD_L4T_UDP;
			else
				offload = false;
			break;
		case IPPROTO_SCTP:
			if (mp->m_pkthdr.csum_flags &
			    (CSUM_IP_SCTP | CSUM_IP6_SCTP))
				type_tucmd_mlhl |= IXGBE_ADVTXD_TUCMD_L4T_SCTP;
			else
				offload = false;
			break;
		default:
			offload = false;
			break;
	}

	if (offload) /* Insert L4 checksum into data descriptors */
		*olinfo_status |= IXGBE_TXD_POPTS_TXSM << 8;

no_offloads:
	type_tucmd_mlhl |= IXGBE_ADVTXD_DCMD_DEXT | IXGBE_ADVTXD_DTYP_CTXT;

	/* Now copy bits into descriptor */
	TXD->vlan_macip_lens = htole32(vlan_macip_lens);
	TXD->type_tucmd_mlhl = htole32(type_tucmd_mlhl);
	TXD->seqnum_seed = htole32(0);
	TXD->mss_l4len_idx = htole32(0);

	/* We've consumed the first desc, adjust counters */
	if (++ctxd == txr->num_desc)
		ctxd = 0;
	txr->next_avail_desc = ctxd;
	--txr->tx_avail;

	return (0);
} /* ixgbe_tx_ctx_setup */

/************************************************************************
 * ixgbe_tso_setup
 *
 *   Setup work for hardware segmentation offload (TSO) on
 *   scs using advanced tx descriptors
 ************************************************************************/
static int
ixgbe_tso_setup(struct tx_ring *txr, struct mbuf *mp, u32 *cmd_type_len,
    u32 *olinfo_status)
{
	struct ixgbe_adv_tx_context_desc *TXD;
	struct ether_vlan_header         *eh;
#ifdef INET6
	struct ip6_hdr                   *ip6;
#endif
#ifdef INET
	struct ip                        *ip;
#endif
	struct tcphdr                    *th;
	int                              ctxd, ehdrlen, ip_hlen, tcp_hlen;
	u32                              vlan_macip_lens = 0;
	u32                              type_tucmd_mlhl = 0;
	u32                              mss_l4len_idx = 0, paylen;
	u16                              vtag = 0, eh_type;

	/*
	 * Determine where frame payload starts.
	 * Jump over vlan headers if already present
	 */
	eh = mtod(mp, struct ether_vlan_header *);
	if (eh->evl_encap_proto == htons(ETHERTYPE_VLAN)) {
		ehdrlen = ETHER_HDR_LEN + ETHER_VLAN_ENCAP_LEN;
		eh_type = eh->evl_proto;
	} else {
		ehdrlen = ETHER_HDR_LEN;
		eh_type = eh->evl_encap_proto;
	}

	switch (ntohs(eh_type)) {
#ifdef INET
	case ETHERTYPE_IP:
		ip = (struct ip *)(mp->m_data + ehdrlen);
		if (ip->ip_p != IPPROTO_TCP)
			return (ENXIO);
		ip->ip_sum = 0;
		ip_hlen = ip->ip_hl << 2;
		th = (struct tcphdr *)((caddr_t)ip + ip_hlen);
		th->th_sum = in_pseudo(ip->ip_src.s_addr,
		    ip->ip_dst.s_addr, htons(IPPROTO_TCP));
		type_tucmd_mlhl |= IXGBE_ADVTXD_TUCMD_IPV4;
		/* Tell transmit desc to also do IPv4 checksum. */
		*olinfo_status |= IXGBE_TXD_POPTS_IXSM << 8;
		break;
#endif
#ifdef INET6
	case ETHERTYPE_IPV6:
		ip6 = (struct ip6_hdr *)(mp->m_data + ehdrlen);
		/* XXX-BZ For now we do not pretend to support ext. hdrs. */
		if (ip6->ip6_nxt != IPPROTO_TCP)
			return (ENXIO);
		ip_hlen = sizeof(struct ip6_hdr);
		th = (struct tcphdr *)((caddr_t)ip6 + ip_hlen);
		th->th_sum = in6_cksum_pseudo(ip6, 0, IPPROTO_TCP, 0);
		type_tucmd_mlhl |= IXGBE_ADVTXD_TUCMD_IPV6;
		break;
#endif
	default:
		panic("%s: CSUM_TSO but no supported IP version (0x%04x)",
		    __func__, ntohs(eh_type));
		break;
	}

	ctxd = txr->next_avail_desc;
	TXD = (struct ixgbe_adv_tx_context_desc *)&txr->tx_base[ctxd];

	tcp_hlen = th->th_off << 2;

	/* This is used in the transmit desc in encap */
	paylen = mp->m_pkthdr.len - ehdrlen - ip_hlen - tcp_hlen;

	/* VLAN MACLEN IPLEN */
	if (mp->m_flags & M_VLANTAG) {
		vtag = htole16(mp->m_pkthdr.ether_vtag);
		vlan_macip_lens |= (vtag << IXGBE_ADVTXD_VLAN_SHIFT);
	}

	vlan_macip_lens |= ehdrlen << IXGBE_ADVTXD_MACLEN_SHIFT;
	vlan_macip_lens |= ip_hlen;
	TXD->vlan_macip_lens = htole32(vlan_macip_lens);

	/* ADV DTYPE TUCMD */
	type_tucmd_mlhl |= IXGBE_ADVTXD_DCMD_DEXT | IXGBE_ADVTXD_DTYP_CTXT;
	type_tucmd_mlhl |= IXGBE_ADVTXD_TUCMD_L4T_TCP;
	TXD->type_tucmd_mlhl = htole32(type_tucmd_mlhl);

	/* MSS L4LEN IDX */
	mss_l4len_idx |= (mp->m_pkthdr.tso_segsz << IXGBE_ADVTXD_MSS_SHIFT);
	mss_l4len_idx |= (tcp_hlen << IXGBE_ADVTXD_L4LEN_SHIFT);
	TXD->mss_l4len_idx = htole32(mss_l4len_idx);

	TXD->seqnum_seed = htole32(0);

	if (++ctxd == txr->num_desc)
		ctxd = 0;

	txr->tx_avail--;
	txr->next_avail_desc = ctxd;
	*cmd_type_len |= IXGBE_ADVTXD_DCMD_TSE;
	*olinfo_status |= IXGBE_TXD_POPTS_TXSM << 8;
	*olinfo_status |= paylen << IXGBE_ADVTXD_PAYLEN_SHIFT;
	++txr->tso_tx;

	return (0);
} /* ixgbe_tso_setup */


/************************************************************************
 * ixgbe_txeof
 *
 *   Examine each tx_buffer in the used queue. If the hardware is done
 *   processing the packet then free associated resources. The
 *   tx_buffer is put back on the free queue.
 ************************************************************************/
void
ixgbe_txeof(struct tx_ring *txr)
{
	struct ixgbe_softc          *sc = txr->sc;
	struct ixgbe_tx_buf     *buf;
	union ixgbe_adv_tx_desc *txd;
	u32                     work, processed = 0;
	u32                     limit = sc->tx_process_limit;

	mtx_assert(&txr->tx_mtx, MA_OWNED);

#ifdef DEV_NETMAP
	if ((sc->feat_en & IXGBE_FEATURE_NETMAP) &&
	    (sc->ifp->if_capenable & IFCAP_NETMAP)) {
		struct netmap_sc *na = NA(sc->ifp);
#if ((__FreeBSD_version >= 1102505 && __FreeBSD_version < 1200000) || \
    __FreeBSD_version >= 1200062)
		struct netmap_kring *kring = na->tx_rings[txr->me];
#else
		struct netmap_kring *kring = &na->tx_rings[txr->me];
#endif
		txd = txr->tx_base;
		bus_dmamap_sync(txr->txdma.dma_tag, txr->txdma.dma_map,
		    BUS_DMASYNC_POSTREAD);
		/*
		 * In netmap mode, all the work is done in the context
		 * of the client thread. Interrupt handlers only wake up
		 * clients, which may be sleeping on individual rings
		 * or on a global resource for all rings.
		 * To implement tx interrupt mitigation, we wake up the client
		 * thread roughly every half ring, even if the NIC interrupts
		 * more frequently. This is implemented as follows:
		 * - ixgbe_txsync() sets kring->nr_kflags with the index of
		 *   the slot that should wake up the thread (nkr_num_slots
		 *   means the user thread should not be woken up);
		 * - the driver ignores tx interrupts unless netmap_mitigate=0
		 *   or the slot has the DD bit set.
		 */

#if __FreeBSD_version >= 1200062
		if (kring->nr_kflags < kring->nkr_num_slots &&
		     txd[kring->nr_kflags].wb.status & IXGBE_TXD_STAT_DD) {
#else
		if (!netmap_mitigate ||
		    (kring->nr_kflags < kring->nkr_num_slots &&
		     txd[kring->nr_kflags].wb.status & IXGBE_TXD_STAT_DD)) {
#endif
			netmap_tx_irq(sc->ifp, txr->me);
		}
		return;
	}
#endif /* DEV_NETMAP */

	if (txr->tx_avail == txr->num_desc) {
		txr->busy = 0;
		return;
	}

	/* Get work starting point */
	work = txr->next_to_clean;
	buf = &txr->tx_buffers[work];
	txd = &txr->tx_base[work];
	work -= txr->num_desc; /* The distance to ring end */
	bus_dmamap_sync(txr->txdma.dma_tag, txr->txdma.dma_map,
	    BUS_DMASYNC_POSTREAD);

	do {
		union ixgbe_adv_tx_desc *eop = buf->eop;
		if (eop == NULL) /* No work */
			break;

		if ((eop->wb.status & IXGBE_TXD_STAT_DD) == 0)
			break;	/* I/O not complete */

		if (buf->m_head) {
			txr->bytes += buf->m_head->m_pkthdr.len;
			bus_dmamap_sync(txr->txtag, buf->map,
			    BUS_DMASYNC_POSTWRITE);
			bus_dmamap_unload(txr->txtag, buf->map);
			m_freem(buf->m_head);
			buf->m_head = NULL;
		}
		buf->eop = NULL;
		++txr->tx_avail;

		/* We clean the range if multi segment */
		while (txd != eop) {
			++txd;
			++buf;
			++work;
			/* wrap the ring? */
			if (__predict_false(!work)) {
				work -= txr->num_desc;
				buf = txr->tx_buffers;
				txd = txr->tx_base;
			}
			if (buf->m_head) {
				txr->bytes += buf->m_head->m_pkthdr.len;
				bus_dmamap_sync(txr->txtag, buf->map,
				    BUS_DMASYNC_POSTWRITE);
				bus_dmamap_unload(txr->txtag, buf->map);
				m_freem(buf->m_head);
				buf->m_head = NULL;
			}
			++txr->tx_avail;
			buf->eop = NULL;

		}
		++txr->packets;
		++processed;

		/* Try the next packet */
		++txd;
		++buf;
		++work;
		/* reset with a wrap */
		if (__predict_false(!work)) {
			work -= txr->num_desc;
			buf = txr->tx_buffers;
			txd = txr->tx_base;
		}
		prefetch(txd);
	} while (__predict_true(--limit));

	bus_dmamap_sync(txr->txdma.dma_tag, txr->txdma.dma_map,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

	work += txr->num_desc;
	txr->next_to_clean = work;

	/*
	 * Queue Hang detection, we know there's
	 * work outstanding or the first return
	 * would have been taken, so increment busy
	 * if nothing managed to get cleaned, then
	 * in local_timer it will be checked and
	 * marked as HUNG if it exceeds a MAX attempt.
	 */
	if ((processed == 0) && (txr->busy != IXGBE_QUEUE_HUNG))
		++txr->busy;
	/*
	 * If anything gets cleaned we reset state to 1,
	 * note this will turn off HUNG if its set.
	 */
	if (processed)
		txr->busy = 1;

	if (txr->tx_avail == txr->num_desc)
		txr->busy = 0;

	return;
} /* ixgbe_txeof */

/************************************************************************
 * ixgbe_rsc_count
 *
 *   Used to detect a descriptor that has been merged by Hardware RSC.
 ************************************************************************/
static inline u32
ixgbe_rsc_count(union ixgbe_adv_rx_desc *rx)
{
	return (le32toh(rx->wb.lower.lo_dword.data) &
	    IXGBE_RXDADV_RSCCNT_MASK) >> IXGBE_RXDADV_RSCCNT_SHIFT;
} /* ixgbe_rsc_count */

/************************************************************************
 * ixgbe_setup_hw_rsc
 *
 *   Initialize Hardware RSC (LRO) feature on 82599
 *   for an RX ring, this is toggled by the LRO capability
 *   even though it is transparent to the stack.
 *
 *   NOTE: Since this HW feature only works with IPv4 and
 *         testing has shown soft LRO to be as effective,
 *         this feature will be disabled by default.
 ************************************************************************/
static void
ixgbe_setup_hw_rsc(struct rx_ring *rxr)
{
	struct ixgbe_softc  *sc = rxr->sc;
	struct ixgbe_hw *hw = &sc->hw;
	u32             rscctrl, rdrxctl;

	/* If turning LRO/RSC off we need to disable it */
	if ((sc->ifp->if_capenable & IFCAP_LRO) == 0) {
		rscctrl = IXGBE_READ_REG(hw, IXGBE_RSCCTL(rxr->me));
		rscctrl &= ~IXGBE_RSCCTL_RSCEN;
		return;
	}

	rdrxctl = IXGBE_READ_REG(hw, IXGBE_RDRXCTL);
	rdrxctl &= ~IXGBE_RDRXCTL_RSCFRSTSIZE;
#ifdef DEV_NETMAP
	/* Always strip CRC unless Netmap disabled it */
	if (!(sc->feat_en & IXGBE_FEATURE_NETMAP) ||
	    !(sc->ifp->if_capenable & IFCAP_NETMAP) ||
	    ix_crcstrip)
#endif /* DEV_NETMAP */
		rdrxctl |= IXGBE_RDRXCTL_CRCSTRIP;
	rdrxctl |= IXGBE_RDRXCTL_RSCACKC;
	IXGBE_WRITE_REG(hw, IXGBE_RDRXCTL, rdrxctl);

	rscctrl = IXGBE_READ_REG(hw, IXGBE_RSCCTL(rxr->me));
	rscctrl |= IXGBE_RSCCTL_RSCEN;
	/*
	 * Limit the total number of descriptors that
	 * can be combined, so it does not exceed 64K
	 */
	if (rxr->mbuf_sz == MCLBYTES)
		rscctrl |= IXGBE_RSCCTL_MAXDESC_16;
	else if (rxr->mbuf_sz == MJUMPAGESIZE)
		rscctrl |= IXGBE_RSCCTL_MAXDESC_8;
	else if (rxr->mbuf_sz == MJUM9BYTES)
		rscctrl |= IXGBE_RSCCTL_MAXDESC_4;
	else  /* Using 16K cluster */
		rscctrl |= IXGBE_RSCCTL_MAXDESC_1;

	IXGBE_WRITE_REG(hw, IXGBE_RSCCTL(rxr->me), rscctrl);

	/* Enable TCP header recognition */
	IXGBE_WRITE_REG(hw, IXGBE_PSRTYPE(0),
	    (IXGBE_READ_REG(hw, IXGBE_PSRTYPE(0)) | IXGBE_PSRTYPE_TCPHDR));

	/* Disable RSC for ACK packets */
	IXGBE_WRITE_REG(hw, IXGBE_RSCDBU,
	    (IXGBE_RSCDBU_RSCACKDIS | IXGBE_READ_REG(hw, IXGBE_RSCDBU)));

	rxr->hw_rsc = TRUE;
} /* ixgbe_setup_hw_rsc */

/************************************************************************
 * ixgbe_refresh_mbufs
 *
 *   Refresh mbuf buffers for RX descriptor rings
 *    - now keeps its own state so discards due to resource
 *      exhaustion are unnecessary, if an mbuf cannot be obtained
 *      it just returns, keeping its placeholder, thus it can simply
 *      be recalled to try again.
 ************************************************************************/
static void
ixgbe_refresh_mbufs(struct rx_ring *rxr, int limit)
{
	struct ixgbe_softc      *sc = rxr->sc;
	struct ixgbe_rx_buf *rxbuf;
	struct mbuf         *mp;
	bus_dma_segment_t   seg[1];
	int                 i, j, nsegs, error;
	bool                refreshed = FALSE;

	i = j = rxr->next_to_refresh;
	/* Control the loop with one beyond */
	if (++j == rxr->num_desc)
		j = 0;

	while (j != limit) {
		rxbuf = &rxr->rx_buffers[i];
		if (rxbuf->buf == NULL) {
			mp = m_getjcl(M_NOWAIT, MT_DATA, M_PKTHDR,
			    rxr->mbuf_sz);
			if (mp == NULL)
				goto update;
			if (sc->max_frame_size <= (MCLBYTES - ETHER_ALIGN))
				m_adj(mp, ETHER_ALIGN);
		} else
			mp = rxbuf->buf;

		mp->m_pkthdr.len = mp->m_len = rxr->mbuf_sz;

		/* If we're dealing with an mbuf that was copied rather
		 * than replaced, there's no need to go through busdma.
		 */
		if ((rxbuf->flags & IXGBE_RX_COPY) == 0) {
			/* Get the memory mapping */
			bus_dmamap_unload(rxr->ptag, rxbuf->pmap);
			error = bus_dmamap_load_mbuf_sg(rxr->ptag, rxbuf->pmap,
			    mp, seg, &nsegs, BUS_DMA_NOWAIT);
			if (error != 0) {
				printf("Refresh mbufs: payload dmamap load failure - %d\n", error);
				m_free(mp);
				rxbuf->buf = NULL;
				goto update;
			}
			rxbuf->buf = mp;
			bus_dmamap_sync(rxr->ptag, rxbuf->pmap,
			    BUS_DMASYNC_PREREAD);
			rxbuf->addr = rxr->rx_base[i].read.pkt_addr =
			    htole64(seg[0].ds_addr);
		} else {
			rxr->rx_base[i].read.pkt_addr = rxbuf->addr;
			rxbuf->flags &= ~IXGBE_RX_COPY;
		}

		refreshed = TRUE;
		/* Next is precalculated */
		i = j;
		rxr->next_to_refresh = i;
		if (++j == rxr->num_desc)
			j = 0;
	}

update:
	if (refreshed) /* Update hardware tail index */
		IXGBE_WRITE_REG(&sc->hw, rxr->tail, rxr->next_to_refresh);

	return;
} /* ixgbe_refresh_mbufs */

/************************************************************************
 * ixgbe_allocate_receive_buffers
 *
 *   Allocate memory for rx_buffer structures. Since we use one
 *   rx_buffer per received packet, the maximum number of rx_buffer's
 *   that we'll need is equal to the number of receive descriptors
 *   that we've allocated.
 ************************************************************************/
static int
ixgbe_allocate_receive_buffers(struct rx_ring *rxr)
{
	struct ixgbe_softc      *sc = rxr->sc;
	device_t            dev = sc->dev;
	struct ixgbe_rx_buf *rxbuf;
	int                 bsize, error;

	bsize = sizeof(struct ixgbe_rx_buf) * rxr->num_desc;
	rxr->rx_buffers = (struct ixgbe_rx_buf *)malloc(bsize, M_IXV,
	    M_NOWAIT | M_ZERO);
	if (!rxr->rx_buffers) {
		device_printf(dev, "Unable to allocate rx_buffer memory\n");
		error = ENOMEM;
		goto fail;
	}

	error = bus_dma_tag_create(
	         /*      parent */ bus_get_dma_tag(dev),
	         /*   alignment */ 1,
	         /*      bounds */ 0,
	         /*     lowaddr */ BUS_SPACE_MAXADDR,
	         /*    highaddr */ BUS_SPACE_MAXADDR,
	         /*      filter */ NULL,
	         /*   filterarg */ NULL,
	         /*     maxsize */ MJUM16BYTES,
	         /*   nsegments */ 1,
	         /*  maxsegsize */ MJUM16BYTES,
	         /*       flags */ 0,
	         /*    lockfunc */ NULL,
	         /* lockfuncarg */ NULL,
	                           &rxr->ptag);
	if (error) {
		device_printf(dev, "Unable to create RX DMA tag\n");
		goto fail;
	}

	for (int i = 0; i < rxr->num_desc; i++, rxbuf++) {
		rxbuf = &rxr->rx_buffers[i];
		error = bus_dmamap_create(rxr->ptag, 0, &rxbuf->pmap);
		if (error) {
			device_printf(dev, "Unable to create RX dma map\n");
			goto fail;
		}
	}

	return (0);

fail:
	/* Frees all, but can handle partial completion */
	ixgbe_free_receive_structures(sc);

	return (error);
} /* ixgbe_allocate_receive_buffers */

/************************************************************************
 * ixgbe_free_receive_ring
 ************************************************************************/
static void
ixgbe_free_receive_ring(struct rx_ring *rxr)
{
	struct ixgbe_rx_buf *rxbuf;

	for (int i = 0; i < rxr->num_desc; i++) {
		rxbuf = &rxr->rx_buffers[i];
		if (rxbuf->buf != NULL) {
			bus_dmamap_sync(rxr->ptag, rxbuf->pmap,
			    BUS_DMASYNC_POSTREAD);
			bus_dmamap_unload(rxr->ptag, rxbuf->pmap);
			rxbuf->buf->m_flags |= M_PKTHDR;
			m_freem(rxbuf->buf);
			rxbuf->buf = NULL;
			rxbuf->flags = 0;
		}
	}
} /* ixgbe_free_receive_ring */

/************************************************************************
 * ixgbe_setup_receive_ring
 *
 *   Initialize a receive ring and its buffers.
 ************************************************************************/
static int
ixgbe_setup_receive_ring(struct rx_ring *rxr)
{
	struct ixgbe_softc        *sc;
	struct ifnet          *ifp;
	device_t              dev;
	struct ixgbe_rx_buf   *rxbuf;
	struct lro_ctrl       *lro = &rxr->lro;
#ifdef DEV_NETMAP
	struct netmap_sc *na = NA(rxr->sc->ifp);
	struct netmap_slot    *slot;
#endif /* DEV_NETMAP */
	bus_dma_segment_t     seg[1];
	int                   rsize, nsegs, error = 0;

	sc = rxr->sc;
	ifp = sc->ifp;
	dev = sc->dev;

	/* Clear the ring contents */
	IXGBE_RX_LOCK(rxr);

#ifdef DEV_NETMAP
	if (sc->feat_en & IXGBE_FEATURE_NETMAP)
		slot = netmap_reset(na, NR_RX, rxr->me, 0);
#endif /* DEV_NETMAP */

	rsize = roundup2(sc->num_rx_desc *
	    sizeof(union ixgbe_adv_rx_desc), DBA_ALIGN);
	bzero((void *)rxr->rx_base, rsize);
	/* Cache the size */
	rxr->mbuf_sz = sc->rx_mbuf_sz;

	/* Free current RX buffer structs and their mbufs */
	ixgbe_free_receive_ring(rxr);

	/* Now replenish the mbufs */
	for (int j = 0; j != rxr->num_desc; ++j) {
		struct mbuf *mp;

		rxbuf = &rxr->rx_buffers[j];

#ifdef DEV_NETMAP
		/*
		 * In netmap mode, fill the map and set the buffer
		 * address in the NIC ring, considering the offset
		 * between the netmap and NIC rings (see comment in
		 * ixgbe_setup_transmit_ring() ). No need to allocate
		 * an mbuf, so end the block with a continue;
		 */
		if ((sc->feat_en & IXGBE_FEATURE_NETMAP) && slot) {
#if ((__FreeBSD_version >= 1102505 && __FreeBSD_version < 1200000) || \
    __FreeBSD_version >= 1200062)
			int sj = netmap_idx_n2k(na->rx_rings[rxr->me], j);
#else
			int sj = netmap_idx_n2k(&na->rx_rings[rxr->me], j);
#endif
			uint64_t paddr;
			void *addr;

			addr = PNMB(na, slot + sj, &paddr);
			netmap_load_map(na, rxr->ptag, rxbuf->pmap, addr);
			/* Update descriptor and the cached value */
			rxr->rx_base[j].read.pkt_addr = htole64(paddr);
			rxbuf->addr = htole64(paddr);
			continue;
		}
#endif /* DEV_NETMAP */

		rxbuf->flags = 0;
		rxbuf->buf = m_getjcl(M_NOWAIT, MT_DATA, M_PKTHDR,
		    sc->rx_mbuf_sz);
		if (rxbuf->buf == NULL) {
			error = ENOBUFS;
			goto fail;
		}
		mp = rxbuf->buf;
		mp->m_pkthdr.len = mp->m_len = rxr->mbuf_sz;
		/* Get the memory mapping */
		error = bus_dmamap_load_mbuf_sg(rxr->ptag, rxbuf->pmap, mp, seg,
		    &nsegs, BUS_DMA_NOWAIT);
		if (error != 0)
			goto fail;
		bus_dmamap_sync(rxr->ptag, rxbuf->pmap, BUS_DMASYNC_PREREAD);
		/* Update the descriptor and the cached value */
		rxr->rx_base[j].read.pkt_addr = htole64(seg[0].ds_addr);
		rxbuf->addr = htole64(seg[0].ds_addr);
	}


	/* Setup our descriptor indices */
	rxr->next_to_check = 0;
	rxr->next_to_refresh = 0;
	rxr->lro_enabled = FALSE;
	rxr->rx_copies = 0;
	rxr->rx_bytes = 0;
	rxr->vtag_strip = FALSE;

	bus_dmamap_sync(rxr->rxdma.dma_tag, rxr->rxdma.dma_map,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

	/*
	 * Now set up the LRO interface
	 */
	if (ixgbe_rsc_enable)
		ixgbe_setup_hw_rsc(rxr);
	else if (ifp->if_capenable & IFCAP_LRO) {
		int err = tcp_lro_init(lro);
		if (err) {
			device_printf(dev, "LRO Initialization failed!\n");
			goto fail;
		}
		INIT_DEBUGOUT("RX Soft LRO Initialized\n");
		rxr->lro_enabled = TRUE;
		lro->ifp = sc->ifp;
	}

	IXGBE_RX_UNLOCK(rxr);

	return (0);

fail:
	ixgbe_free_receive_ring(rxr);
	IXGBE_RX_UNLOCK(rxr);

	return (error);
} /* ixgbe_setup_receive_ring */

/************************************************************************
 * ixgbe_setup_receive_structures - Initialize all receive rings.
 ************************************************************************/
int
ixgbe_setup_receive_structures(struct ixgbe_softc *sc)
{
	struct rx_ring *rxr = sc->rx_rings;
	int            j;

	for (j = 0; j < sc->num_queues; j++, rxr++)
		if (ixgbe_setup_receive_ring(rxr))
			goto fail;

	return (0);
fail:
	/*
	 * Free RX buffers allocated so far, we will only handle
	 * the rings that completed, the failing case will have
	 * cleaned up for itself. 'j' failed, so its the terminus.
	 */
	for (int i = 0; i < j; ++i) {
		rxr = &sc->rx_rings[i];
		ixgbe_free_receive_ring(rxr);
	}

	return (ENOBUFS);
} /* ixgbe_setup_receive_structures */


/************************************************************************
 * ixgbe_free_receive_structures - Free all receive rings.
 ************************************************************************/
void
ixgbe_free_receive_structures(struct ixgbe_softc *sc)
{
	struct rx_ring *rxr = sc->rx_rings;
	struct lro_ctrl *lro;

	INIT_DEBUGOUT("ixgbe_free_receive_structures: begin");

	for (int i = 0; i < sc->num_queues; i++, rxr++) {
		lro = &rxr->lro;
		ixgbe_free_receive_buffers(rxr);
		/* Free LRO memory */
		tcp_lro_free(lro);
		/* Free the ring memory as well */
		ixgbe_dma_free(sc, &rxr->rxdma);
	}

	free(sc->rx_rings, M_IXV);
} /* ixgbe_free_receive_structures */


/************************************************************************
 * ixgbe_free_receive_buffers - Free receive ring data structures
 ************************************************************************/
static void
ixgbe_free_receive_buffers(struct rx_ring *rxr)
{
	struct ixgbe_softc      *sc = rxr->sc;
	struct ixgbe_rx_buf *rxbuf;

	INIT_DEBUGOUT("ixgbe_free_receive_buffers: begin");

	/* Cleanup any existing buffers */
	if (rxr->rx_buffers != NULL) {
		for (int i = 0; i < sc->num_rx_desc; i++) {
			rxbuf = &rxr->rx_buffers[i];
			if (rxbuf->buf != NULL) {
				bus_dmamap_sync(rxr->ptag, rxbuf->pmap,
				    BUS_DMASYNC_POSTREAD);
				bus_dmamap_unload(rxr->ptag, rxbuf->pmap);
				rxbuf->buf->m_flags |= M_PKTHDR;
				m_freem(rxbuf->buf);
			}
			rxbuf->buf = NULL;
			if (rxbuf->pmap != NULL) {
				bus_dmamap_destroy(rxr->ptag, rxbuf->pmap);
				rxbuf->pmap = NULL;
			}
		}
		if (rxr->rx_buffers != NULL) {
			free(rxr->rx_buffers, M_IXV);
			rxr->rx_buffers = NULL;
		}
	}

	if (rxr->ptag != NULL) {
		bus_dma_tag_destroy(rxr->ptag);
		rxr->ptag = NULL;
	}

	return;
} /* ixgbe_free_receive_buffers */

/************************************************************************
 * ixgbe_rx_input
 ************************************************************************/
static __inline void
ixgbe_rx_input(struct rx_ring *rxr, struct ifnet *ifp, struct mbuf *m,
    u32 ptype)
{
	/*
	 * ATM LRO is only for IP/TCP packets and TCP checksum of the packet
	 * should be computed by hardware. Also it should not have VLAN tag in
	 * ethernet header.  In case of IPv6 we do not yet support ext. hdrs.
	 */
	if (rxr->lro_enabled &&
	    (ifp->if_capenable & IFCAP_VLAN_HWTAGGING) != 0 &&
	    (ptype & IXGBE_RXDADV_PKTTYPE_ETQF) == 0 &&
	    ((ptype & (IXGBE_RXDADV_PKTTYPE_IPV4 | IXGBE_RXDADV_PKTTYPE_TCP)) ==
	     (IXGBE_RXDADV_PKTTYPE_IPV4 | IXGBE_RXDADV_PKTTYPE_TCP) ||
	     (ptype & (IXGBE_RXDADV_PKTTYPE_IPV6 | IXGBE_RXDADV_PKTTYPE_TCP)) ==
	     (IXGBE_RXDADV_PKTTYPE_IPV6 | IXGBE_RXDADV_PKTTYPE_TCP)) &&
	    (m->m_pkthdr.csum_flags & (CSUM_DATA_VALID | CSUM_PSEUDO_HDR)) ==
	    (CSUM_DATA_VALID | CSUM_PSEUDO_HDR)) {
		/*
		 * Send to the stack if:
		 *  - LRO not enabled, or
		 *  - no LRO resources, or
		 *  - lro enqueue fails
		 */
		if (rxr->lro.lro_cnt != 0)
			if (tcp_lro_rx(&rxr->lro, m, 0) == 0)
				return;
	}
	IXGBE_RX_UNLOCK(rxr);
	(*ifp->if_input)(ifp, m);
	IXGBE_RX_LOCK(rxr);
} /* ixgbe_rx_input */

/************************************************************************
 * ixgbe_rx_discard
 ************************************************************************/
static __inline void
ixgbe_rx_discard(struct rx_ring *rxr, int i)
{
	struct ixgbe_rx_buf *rbuf;

	rbuf = &rxr->rx_buffers[i];

	/*
	 * With advanced descriptors the writeback
	 * clobbers the buffer addrs, so its easier
	 * to just free the existing mbufs and take
	 * the normal refresh path to get new buffers
	 * and mapping.
	 */

	if (rbuf->fmp != NULL) {/* Partial chain ? */
		rbuf->fmp->m_flags |= M_PKTHDR;
		m_freem(rbuf->fmp);
		rbuf->fmp = NULL;
		rbuf->buf = NULL; /* rbuf->buf is part of fmp's chain */
	} else if (rbuf->buf) {
		m_free(rbuf->buf);
		rbuf->buf = NULL;
	}
	bus_dmamap_unload(rxr->ptag, rbuf->pmap);

	rbuf->flags = 0;

	return;
} /* ixgbe_rx_discard */


/************************************************************************
 * ixgbe_rxeof
 *
 *   Executes in interrupt context. It replenishes the
 *   mbufs in the descriptor and sends data which has
 *   been dma'ed into host memory to upper layer.
 *
 *   Return TRUE for more work, FALSE for all clean.
 ************************************************************************/
bool
ixgbe_rxeof(struct ix_queue *que)
{
	struct ixgbe_softc          *sc = que->sc;
	struct rx_ring          *rxr = que->rxr;
	struct ifnet            *ifp = sc->ifp;
	struct lro_ctrl         *lro = &rxr->lro;
	union ixgbe_adv_rx_desc *cur;
	struct ixgbe_rx_buf     *rbuf, *nbuf;
	int                     i, nextp, processed = 0;
	u32                     staterr = 0;
	u32                     count = sc->rx_process_limit;
	u16                     pkt_info;

	IXGBE_RX_LOCK(rxr);

#ifdef DEV_NETMAP
	if (sc->feat_en & IXGBE_FEATURE_NETMAP) {
		/* Same as the txeof routine: wakeup clients on intr. */
		if (netmap_rx_irq(ifp, rxr->me, &processed)) {
			IXGBE_RX_UNLOCK(rxr);
			return (FALSE);
		}
	}
#endif /* DEV_NETMAP */

	for (i = rxr->next_to_check; count != 0;) {
		struct mbuf *sendmp, *mp;
		u32         rsc, ptype;
		u16         len;
		u16         vtag = 0;
		bool        eop;

		/* Sync the ring. */
		bus_dmamap_sync(rxr->rxdma.dma_tag, rxr->rxdma.dma_map,
		    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);

		cur = &rxr->rx_base[i];
		staterr = le32toh(cur->wb.upper.status_error);
		pkt_info = le16toh(cur->wb.lower.lo_dword.hs_rss.pkt_info);

		if ((staterr & IXGBE_RXD_STAT_DD) == 0)
			break;
		if ((ifp->if_drv_flags & IFF_DRV_RUNNING) == 0)
			break;

		count--;
		sendmp = NULL;
		nbuf = NULL;
		rsc = 0;
		cur->wb.upper.status_error = 0;
		rbuf = &rxr->rx_buffers[i];
		mp = rbuf->buf;

		len = le16toh(cur->wb.upper.length);
		ptype = le32toh(cur->wb.lower.lo_dword.data) &
		    IXGBE_RXDADV_PKTTYPE_MASK;
		eop = ((staterr & IXGBE_RXD_STAT_EOP) != 0);

		/* Make sure bad packets are discarded */
		if (eop && (staterr & IXGBE_RXDADV_ERR_FRAME_ERR_MASK) != 0) {
			if (sc->feat_en & IXGBE_FEATURE_VF)
				if_inc_counter(ifp, IFCOUNTER_IERRORS, 1);
			rxr->rx_discarded++;
			ixgbe_rx_discard(rxr, i);
			goto next_desc;
		}

		/*
		 * On 82599 which supports a hardware
		 * LRO (called HW RSC), packets need
		 * not be fragmented across sequential
		 * descriptors, rather the next descriptor
		 * is indicated in bits of the descriptor.
		 * This also means that we might proceses
		 * more than one packet at a time, something
		 * that has never been true before, it
		 * required eliminating global chain pointers
		 * in favor of what we are doing here.  -jfv
		 */
		if (!eop) {
			/*
			 * Figure out the next descriptor
			 * of this frame.
			 */
			if (rxr->hw_rsc == TRUE) {
				rsc = ixgbe_rsc_count(cur);
				rxr->rsc_num += (rsc - 1);
			}
			if (rsc) { /* Get hardware index */
				nextp = ((staterr & IXGBE_RXDADV_NEXTP_MASK) >>
				    IXGBE_RXDADV_NEXTP_SHIFT);
			} else { /* Just sequential */
				nextp = i + 1;
				if (nextp == sc->num_rx_desc)
					nextp = 0;
			}
			nbuf = &rxr->rx_buffers[nextp];
			prefetch(nbuf);
		}
		/*
		 * Rather than using the fmp/lmp global pointers
		 * we now keep the head of a packet chain in the
		 * buffer struct and pass this along from one
		 * descriptor to the next, until we get EOP.
		 */
		mp->m_len = len;
		/*
		 * See if there is a stored head
		 * that determines what we are
		 */
		sendmp = rbuf->fmp;
		if (sendmp != NULL) {  /* secondary frag */
			rbuf->buf = rbuf->fmp = NULL;
			mp->m_flags &= ~M_PKTHDR;
			sendmp->m_pkthdr.len += mp->m_len;
		} else {
			/*
			 * Optimize.  This might be a small packet,
			 * maybe just a TCP ACK.  Do a fast copy that
			 * is cache aligned into a new mbuf, and
			 * leave the old mbuf+cluster for re-use.
			 */
			if (eop && len <= IXGBE_RX_COPY_LEN) {
				sendmp = m_gethdr(M_NOWAIT, MT_DATA);
				if (sendmp != NULL) {
					sendmp->m_data += IXGBE_RX_COPY_ALIGN;
					ixgbe_bcopy(mp->m_data, sendmp->m_data,
					    len);
					sendmp->m_len = len;
					rxr->rx_copies++;
					rbuf->flags |= IXGBE_RX_COPY;
				}
			}
			if (sendmp == NULL) {
				rbuf->buf = rbuf->fmp = NULL;
				sendmp = mp;
			}

			/* first desc of a non-ps chain */
			sendmp->m_flags |= M_PKTHDR;
			sendmp->m_pkthdr.len = mp->m_len;
		}
		++processed;

		/* Pass the head pointer on */
		if (eop == 0) {
			nbuf->fmp = sendmp;
			sendmp = NULL;
			mp->m_next = nbuf->buf;
		} else { /* Sending this frame */
			sendmp->m_pkthdr.rcvif = ifp;
			rxr->rx_packets++;
			/* capture data for AIM */
			rxr->bytes += sendmp->m_pkthdr.len;
			rxr->rx_bytes += sendmp->m_pkthdr.len;
			/* Process vlan info */
			if ((rxr->vtag_strip) && (staterr & IXGBE_RXD_STAT_VP))
				vtag = le16toh(cur->wb.upper.vlan);
			if (vtag) {
				sendmp->m_pkthdr.ether_vtag = vtag;
				sendmp->m_flags |= M_VLANTAG;
			}
			if ((ifp->if_capenable & IFCAP_RXCSUM) != 0)
				ixgbe_rx_checksum(staterr, sendmp, ptype);

			/*
			 * In case of multiqueue, we have RXCSUM.PCSD bit set
			 * and never cleared. This means we have RSS hash
			 * available to be used.
			 */
			if (sc->num_queues > 1) {
				sendmp->m_pkthdr.flowid =
				    le32toh(cur->wb.lower.hi_dword.rss);
				switch (pkt_info & IXGBE_RXDADV_RSSTYPE_MASK) {
				case IXGBE_RXDADV_RSSTYPE_IPV4:
					M_HASHTYPE_SET(sendmp,
					    M_HASHTYPE_RSS_IPV4);
					break;
				case IXGBE_RXDADV_RSSTYPE_IPV4_TCP:
					M_HASHTYPE_SET(sendmp,
					    M_HASHTYPE_RSS_TCP_IPV4);
					break;
				case IXGBE_RXDADV_RSSTYPE_IPV6:
					M_HASHTYPE_SET(sendmp,
					    M_HASHTYPE_RSS_IPV6);
					break;
				case IXGBE_RXDADV_RSSTYPE_IPV6_TCP:
					M_HASHTYPE_SET(sendmp,
					    M_HASHTYPE_RSS_TCP_IPV6);
					break;
				case IXGBE_RXDADV_RSSTYPE_IPV6_EX:
					M_HASHTYPE_SET(sendmp,
					    M_HASHTYPE_RSS_IPV6_EX);
					break;
				case IXGBE_RXDADV_RSSTYPE_IPV6_TCP_EX:
					M_HASHTYPE_SET(sendmp,
					    M_HASHTYPE_RSS_TCP_IPV6_EX);
					break;
				case IXGBE_RXDADV_RSSTYPE_IPV4_UDP:
					M_HASHTYPE_SET(sendmp,
					    M_HASHTYPE_RSS_UDP_IPV4);
					break;
				case IXGBE_RXDADV_RSSTYPE_IPV6_UDP:
					M_HASHTYPE_SET(sendmp,
					    M_HASHTYPE_RSS_UDP_IPV6);
					break;
				case IXGBE_RXDADV_RSSTYPE_IPV6_UDP_EX:
					M_HASHTYPE_SET(sendmp,
					    M_HASHTYPE_RSS_UDP_IPV6_EX);
					break;
				default:
					M_HASHTYPE_SET(sendmp,
					    M_HASHTYPE_OPAQUE_HASH);
				}
			} else {
				sendmp->m_pkthdr.flowid = que->msix;
				M_HASHTYPE_SET(sendmp, M_HASHTYPE_OPAQUE);
			}
		}
next_desc:
		bus_dmamap_sync(rxr->rxdma.dma_tag, rxr->rxdma.dma_map,
		    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

		/* Advance our pointers to the next descriptor. */
		if (++i == rxr->num_desc)
			i = 0;

		/* Now send to the stack or do LRO */
		if (sendmp != NULL) {
			rxr->next_to_check = i;
			ixgbe_rx_input(rxr, ifp, sendmp, ptype);
			i = rxr->next_to_check;
		}

		/* Every 8 descriptors we go to refresh mbufs */
		if (processed == 8) {
			ixgbe_refresh_mbufs(rxr, i);
			processed = 0;
		}
	}

	/* Refresh any remaining buf structs */
	if (ixgbe_rx_unrefreshed(rxr))
		ixgbe_refresh_mbufs(rxr, i);

	rxr->next_to_check = i;

	/*
	 * Flush any outstanding LRO work
	 */
	tcp_lro_flush_all(lro);

	IXGBE_RX_UNLOCK(rxr);

	/*
	 * Still have cleaning to do?
	 */
	if ((staterr & IXGBE_RXD_STAT_DD) != 0)
		return (TRUE);

	return (FALSE);
} /* ixgbe_rxeof */


/************************************************************************
 * ixgbe_rx_checksum
 *
 *   Verify that the hardware indicated that the checksum is valid.
 *   Inform the stack about the status of checksum so that stack
 *   doesn't spend time verifying the checksum.
 ************************************************************************/
static void
ixgbe_rx_checksum(u32 staterr, struct mbuf * mp, u32 ptype)
{
	u16  status = (u16)staterr;
	u8   errors = (u8)(staterr >> 24);
	bool sctp = false;

	if ((ptype & IXGBE_RXDADV_PKTTYPE_ETQF) == 0 &&
	    (ptype & IXGBE_RXDADV_PKTTYPE_SCTP) != 0)
		sctp = true;

	/* IPv4 checksum */
	if (status & IXGBE_RXD_STAT_IPCS) {
		if (!(errors & IXGBE_RXD_ERR_IPE)) {
			/* IP Checksum Good */
			mp->m_pkthdr.csum_flags |= (CSUM_L3_CALC | CSUM_L3_VALID);
		} else
			mp->m_pkthdr.csum_flags = 0;

	}
	/* TCP/UDP/SCTP checksum */
	if (status & IXGBE_RXD_STAT_L4CS) {
		mp->m_pkthdr.csum_flags |= CSUM_L4_CALC;
		if (!(errors & IXGBE_RXD_ERR_TCPE)) {
			mp->m_pkthdr.csum_flags |= CSUM_L4_VALID;
			if (!sctp)
				mp->m_pkthdr.csum_data = htons(0xffff);
		}
	}
} /* ixgbe_rx_checksum */

/************************************************************************
 * ixgbe_dmamap_cb - Manage DMA'able memory.
 ************************************************************************/
static void
ixgbe_dmamap_cb(void *arg, bus_dma_segment_t * segs, int nseg, int error)
{
	if (error)
		return;
	*(bus_addr_t *)arg = segs->ds_addr;

	return;
} /* ixgbe_dmamap_cb */

/************************************************************************
 * ixgbe_dma_malloc
 ************************************************************************/
static int
ixgbe_dma_malloc(struct ixgbe_softc *sc, bus_size_t size,
                 struct ixgbe_dma_alloc *dma, int mapflags)
{
	device_t dev = sc->dev;
	int      r;

	r = bus_dma_tag_create(
	     /*      parent */ bus_get_dma_tag(sc->dev),
	     /*   alignment */ DBA_ALIGN,
	     /*      bounds */ 0,
	     /*     lowaddr */ BUS_SPACE_MAXADDR,
	     /*    highaddr */ BUS_SPACE_MAXADDR,
	     /*      filter */ NULL,
	     /*   filterarg */ NULL,
	     /*     maxsize */ size,
	     /*   nsegments */ 1,
	     /*  maxsegsize */ size,
	     /*       flags */ BUS_DMA_ALLOCNOW,
	     /*    lockfunc */ NULL,
	     /* lockfuncarg */ NULL,
	                       &dma->dma_tag);
	if (r != 0) {
		device_printf(dev,
		    "ixgbe_dma_malloc: bus_dma_tag_create failed; error %u\n",
		    r);
		goto fail_0;
	}
	r = bus_dmamem_alloc(dma->dma_tag, (void **)&dma->dma_vaddr,
	    BUS_DMA_NOWAIT, &dma->dma_map);
	if (r != 0) {
		device_printf(dev,
		    "ixgbe_dma_malloc: bus_dmamem_alloc failed; error %u\n", r);
		goto fail_1;
	}
	r = bus_dmamap_load(dma->dma_tag, dma->dma_map, dma->dma_vaddr, size,
	    ixgbe_dmamap_cb, &dma->dma_paddr, mapflags | BUS_DMA_NOWAIT);
	if (r != 0) {
		device_printf(dev,
		    "ixgbe_dma_malloc: bus_dmamap_load failed; error %u\n", r);
		goto fail_2;
	}
	dma->dma_size = size;

	return (0);
fail_2:
	bus_dmamem_free(dma->dma_tag, dma->dma_vaddr, dma->dma_map);
fail_1:
	bus_dma_tag_destroy(dma->dma_tag);
fail_0:
	dma->dma_tag = NULL;

	return (r);
} /* ixgbe_dma_malloc */

/************************************************************************
 * ixgbe_dma_free
 ************************************************************************/
static void
ixgbe_dma_free(struct ixgbe_softc *sc, struct ixgbe_dma_alloc *dma)
{
	bus_dmamap_sync(dma->dma_tag, dma->dma_map,
	    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
	bus_dmamap_unload(dma->dma_tag, dma->dma_map);
	bus_dmamem_free(dma->dma_tag, dma->dma_vaddr, dma->dma_map);
	bus_dma_tag_destroy(dma->dma_tag);
} /* ixgbe_dma_free */


/************************************************************************
 * ixgbe_allocate_queues
 *
 *   Allocate memory for the transmit and receive rings, and then
 *   the descriptors associated with each, called only once at attach.
 ************************************************************************/
int
ixgbe_allocate_queues(struct ixgbe_softc *sc)
{
	device_t        dev = sc->dev;
	struct ix_queue *que;
	struct tx_ring  *txr;
	struct rx_ring  *rxr;
	int             rsize, tsize, error = IXGBE_SUCCESS;
	int             txconf = 0, rxconf = 0;

	/* First, allocate the top level queue structs */
	sc->queues = (struct ix_queue *)malloc(sizeof(struct ix_queue) *
	    sc->num_queues, M_IXV, M_NOWAIT | M_ZERO);
	if (!sc->queues) {
		device_printf(dev, "Unable to allocate queue memory\n");
		error = ENOMEM;
		goto fail;
	}

	/* Second, allocate the TX ring struct memory */
	sc->tx_rings = (struct tx_ring *)malloc(sizeof(struct tx_ring) *
	    sc->num_queues, M_IXV, M_NOWAIT | M_ZERO);
	if (!sc->tx_rings) {
		device_printf(dev, "Unable to allocate TX ring memory\n");
		error = ENOMEM;
		goto tx_fail;
	}

	/* Third, allocate the RX ring */
	sc->rx_rings = (struct rx_ring *)malloc(sizeof(struct rx_ring) *
	    sc->num_queues, M_IXV, M_NOWAIT | M_ZERO);
	if (!sc->rx_rings) {
		device_printf(dev, "Unable to allocate RX ring memory\n");
		error = ENOMEM;
		goto rx_fail;
	}

	/* For the ring itself */
	tsize = roundup2(sc->num_tx_desc * sizeof(union ixgbe_adv_tx_desc),
	    DBA_ALIGN);

	/*
	 * Now set up the TX queues, txconf is needed to handle the
	 * possibility that things fail midcourse and we need to
	 * undo memory gracefully
	 */
	for (int i = 0; i < sc->num_queues; i++, txconf++) {
		/* Set up some basics */
		txr = &sc->tx_rings[i];
		txr->sc = sc;
		txr->br = NULL;
		/* In case SR-IOV is enabled, align the index properly */
		txr->me = ixgbe_vf_que_index(sc->iov_mode, sc->pool,
		    i);
		txr->num_desc = sc->num_tx_desc;

		/* Initialize the TX side lock */
		snprintf(txr->mtx_name, sizeof(txr->mtx_name), "%s:tx(%d)",
		    device_get_nameunit(dev), txr->me);
		mtx_init(&txr->tx_mtx, txr->mtx_name, NULL, MTX_DEF);

		if (ixgbe_dma_malloc(sc, tsize, &txr->txdma,
		    BUS_DMA_NOWAIT)) {
			device_printf(dev,
			    "Unable to allocate TX Descriptor memory\n");
			error = ENOMEM;
			goto err_tx_desc;
		}
		txr->tx_base = (union ixgbe_adv_tx_desc *)txr->txdma.dma_vaddr;
		bzero((void *)txr->tx_base, tsize);

		/* Now allocate transmit buffers for the ring */
		if (ixgbe_allocate_transmit_buffers(txr)) {
			device_printf(dev,
			    "Critical Failure setting up transmit buffers\n");
			error = ENOMEM;
			goto err_tx_desc;
		}
		if (!(sc->feat_en & IXGBE_FEATURE_LEGACY_TX)) {
			/* Allocate a buf ring */
			txr->br = buf_ring_alloc(IXGBE_BR_SIZE, M_IXV,
			    M_WAITOK, &txr->tx_mtx);
			if (txr->br == NULL) {
				device_printf(dev,
				    "Critical Failure setting up buf ring\n");
				error = ENOMEM;
				goto err_tx_desc;
			}
		}
	}

	/*
	 * Next the RX queues...
	 */
	rsize = roundup2(sc->num_rx_desc * sizeof(union ixgbe_adv_rx_desc),
	    DBA_ALIGN);
	for (int i = 0; i < sc->num_queues; i++, rxconf++) {
		rxr = &sc->rx_rings[i];
		/* Set up some basics */
		rxr->sc = sc;
		/* In case SR-IOV is enabled, align the index properly */
		rxr->me = ixgbe_vf_que_index(sc->iov_mode, sc->pool,
		    i);
		rxr->num_desc = sc->num_rx_desc;

		/* Initialize the RX side lock */
		snprintf(rxr->mtx_name, sizeof(rxr->mtx_name), "%s:rx(%d)",
		    device_get_nameunit(dev), rxr->me);
		mtx_init(&rxr->rx_mtx, rxr->mtx_name, NULL, MTX_DEF);

		if (ixgbe_dma_malloc(sc, rsize, &rxr->rxdma,
		    BUS_DMA_NOWAIT)) {
			device_printf(dev,
			    "Unable to allocate RxDescriptor memory\n");
			error = ENOMEM;
			goto err_rx_desc;
		}
		rxr->rx_base = (union ixgbe_adv_rx_desc *)rxr->rxdma.dma_vaddr;
		bzero((void *)rxr->rx_base, rsize);

		/* Allocate receive buffers for the ring */
		if (ixgbe_allocate_receive_buffers(rxr)) {
			device_printf(dev,
			    "Critical Failure setting up receive buffers\n");
			error = ENOMEM;
			goto err_rx_desc;
		}
	}

	/*
	 * Finally set up the queue holding structs
	 */
	for (int i = 0; i < sc->num_queues; i++) {
		que = &sc->queues[i];
		que->sc = sc;
		que->me = i;
		que->txr = &sc->tx_rings[i];
		que->rxr = &sc->rx_rings[i];
	}

	return (0);

err_rx_desc:
	for (rxr = sc->rx_rings; rxconf > 0; rxr++, rxconf--)
		ixgbe_dma_free(sc, &rxr->rxdma);
err_tx_desc:
	for (txr = sc->tx_rings; txconf > 0; txr++, txconf--)
		ixgbe_dma_free(sc, &txr->txdma);
	free(sc->rx_rings, M_IXV);
rx_fail:
	free(sc->tx_rings, M_IXV);
tx_fail:
	free(sc->queues, M_IXV);
fail:
	return (error);
} /* ixgbe_allocate_queues */
