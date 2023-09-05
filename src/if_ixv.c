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
#endif

#include "ixgbe.h"

/************************************************************************
 * Driver version
 ************************************************************************/
char ixv_driver_version[] = "1.5.33";

/************************************************************************
 * PCI Device ID Table
 *
 *   Used by probe to select devices to load on
 *   Last field stores an index into ixv_strings
 *   Last entry must be all 0s
 *
 *   { Vendor ID, Device ID, SubVendor ID, SubDevice ID, String Index }
 ************************************************************************/
static ixgbe_vendor_info_t ixv_vendor_info_array[] =
{
	{IXGBE_INTEL_VENDOR_ID, IXGBE_DEV_ID_82599_VF, 0, 0, 0},
	{IXGBE_INTEL_VENDOR_ID, IXGBE_DEV_ID_X540_VF, 0, 0, 0},
	{IXGBE_INTEL_VENDOR_ID, IXGBE_DEV_ID_X540_VF_HV, 0, 0, 0},
	{IXGBE_INTEL_VENDOR_ID, IXGBE_DEV_ID_X550_VF, 0, 0, 0},
	{IXGBE_INTEL_VENDOR_ID, IXGBE_DEV_ID_X550EM_X_VF, 0, 0, 0},
	{IXGBE_INTEL_VENDOR_ID, IXGBE_DEV_ID_X550EM_A_VF, 0, 0, 0},
	/* required last entry */
	{0, 0, 0, 0, 0}
};

/************************************************************************
 * Table of branding strings
 ************************************************************************/
static char *ixv_strings[] = {
	"Intel(R) PRO/10GbE Virtual Function Network Driver"
};

/************************************************************************
 * Function prototypes
 ************************************************************************/
static int      ixv_probe(device_t);
static int      ixv_attach(device_t);
static int      ixv_detach(device_t);
static int      ixv_shutdown(device_t);
static int      ixv_ioctl(struct ifnet *, u_long, caddr_t);
static void     ixv_init(void *);
static void     ixv_stop(void *);
static uint64_t ixv_get_counter(struct ifnet *, ift_counter);
static void     ixv_init_device_features(struct ixgbe_softc *);
static void     ixv_media_status(struct ifnet *, struct ifmediareq *);
static int      ixv_media_change(struct ifnet *);
static int      ixv_allocate_pci_resources(struct ixgbe_softc *);
static int      ixv_allocate_msix(struct ixgbe_softc *);
static int      ixv_configure_interrupts(struct ixgbe_softc *);
static void	ixv_free_msix_queues(struct ixgbe_softc *);
static void     ixv_free_pci_resources(struct ixgbe_softc *);
static void     ixv_local_timer(void *);
static void     ixv_setup_interface(device_t, struct ixgbe_softc *);
static int      ixv_negotiate_api(struct ixgbe_softc *);

static void     ixv_initialize_transmit_units(struct ixgbe_softc *);
static void     ixv_initialize_receive_units(struct ixgbe_softc *);
static void     ixv_initialize_rss_mapping(struct ixgbe_softc *);
static s32      ixv_check_link(struct ixgbe_softc *);

static void     ixv_enable_intr(struct ixgbe_softc *);
static void     ixv_disable_intr(struct ixgbe_softc *);
static void     ixv_set_multi(struct ixgbe_softc *);
static void     ixv_update_link_status(struct ixgbe_softc *);
#ifdef IXGBE_DEBUG
static int      ixv_sysctl_debug(SYSCTL_HANDLER_ARGS);
#endif
static void     ixv_set_ivar(struct ixgbe_softc *, u8, u8, s8);
static void     ixv_configure_ivars(struct ixgbe_softc *);
static u8       *ixv_mc_array_itr(struct ixgbe_hw *, u8 **, u32 *);

static void     ixv_setup_vlan_support(struct ixgbe_softc *);
static void     ixv_register_vlan(void *, struct ifnet *, u16);
static void     ixv_unregister_vlan(void *, struct ifnet *, u16);

static void     ixv_save_stats(struct ixgbe_softc *);
static void     ixv_init_stats(struct ixgbe_softc *);
static void     ixv_update_stats(struct ixgbe_softc *);
static void     ixv_add_stats_sysctls(struct ixgbe_softc *);
static void     ixv_set_sysctl_value(struct ixgbe_softc *, const char *,
                                     const char *, int *, int);

/* The MSI-X Interrupt handlers */
static void     ixv_msix_que(void *);
static void     ixv_msix_mbx(void *);

/* Deferred interrupt tasklets */
static void     ixv_handle_que(void *, int);
static void     ixv_handle_link(void *, int);

/************************************************************************
 * FreeBSD Device Interface Entry Points
 ************************************************************************/
static device_method_t ixv_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, ixv_probe),
	DEVMETHOD(device_attach, ixv_attach),
	DEVMETHOD(device_detach, ixv_detach),
	DEVMETHOD(device_shutdown, ixv_shutdown),
	DEVMETHOD_END
};

static driver_t ixv_driver = {
	"ixv", ixv_methods, sizeof(struct ixgbe_softc),
};

devclass_t ixv_devclass;
DRIVER_MODULE(ixv, pci, ixv_driver, ixv_devclass, 0, 0);
MODULE_DEPEND(ixv, pci, 1, 1, 1);
MODULE_DEPEND(ixv, ether, 1, 1, 1);
#ifdef DEV_NETMAP
MODULE_DEPEND(ixv, netmap, 1, 1, 1);
#endif

/*
 * TUNEABLE PARAMETERS:
 */

static SYSCTL_NODE(_hw, OID_AUTO, ixv, CTLFLAG_RD, 0, "IXV driver parameters");

/* Number of Queues - do not exceed MSI-X vectors - 1 */
static int ixv_num_queues = 1;
TUNABLE_INT("hw.ixv.num_queues", &ixv_num_queues);
SYSCTL_INT(_hw_ixv, OID_AUTO, num_queues, CTLFLAG_RDTUN, &ixv_num_queues, 0,
    "Number of queues to configure, 0 indicates autoconfigure");

/*
 * AIM: Adaptive Interrupt Moderation
 * which means that the interrupt rate
 * is varied over time based on the
 * traffic for that interrupt vector
 */
static int ixv_enable_aim = FALSE;
TUNABLE_INT("hw.ixv.enable_aim", &ixv_enable_aim);
SYSCTL_INT(_hw_ixv, OID_AUTO, enable_aim, CTLFLAG_RDTUN, &ixv_enable_aim, 0,
    "Adaptive Interrupt Moderation");

/* How many packets rxeof tries to clean at a time */
static int ixv_rx_process_limit = 256;
TUNABLE_INT("hw.ixv.rx_process_limit", &ixv_rx_process_limit);
SYSCTL_INT(_hw_ixv, OID_AUTO, rx_process_limit, CTLFLAG_RDTUN,
    &ixv_rx_process_limit, 0, "Limit to RX packet processing");

/* How many packets txeof tries to clean at a time */
static int ixv_tx_process_limit = 256;
TUNABLE_INT("hw.ixv.tx_process_limit", &ixv_tx_process_limit);
SYSCTL_INT(_hw_ixv, OID_AUTO, tx_process_limit, CTLFLAG_RDTUN,
    &ixv_tx_process_limit, 0, "Limit to TX packet processing");

/* Flow control setting, default to full */
static int ixv_flow_control = ixgbe_fc_full;
TUNABLE_INT("hw.ixv.flow_control", &ixv_flow_control);
SYSCTL_INT(_hw_ixv, OID_AUTO, flow_control, CTLFLAG_RDTUN, &ixv_flow_control, 0,
    "Flow Control");

/*
 * Header split: this causes the hardware to DMA
 * the header into a separate mbuf from the payload,
 * it can be a performance win in some workloads, but
 * in others it actually hurts, its off by default.
 */
static int ixv_header_split = FALSE;
TUNABLE_INT("hw.ixv.hdr_split", &ixv_header_split);
SYSCTL_INT(_hw_ixv, OID_AUTO, header_split, CTLFLAG_RDTUN, &ixv_header_split, 0,
    "Header Split: DMA header into separate mbuf");

/*
 * Number of TX descriptors per ring,
 * setting higher than RX as this seems
 * the better performing choice.
 */
static int ixv_txd = DEFAULT_TXD;
TUNABLE_INT("hw.ixv.txd", &ixv_txd);
SYSCTL_INT(_hw_ixv, OID_AUTO, txd, CTLFLAG_RDTUN, &ixv_txd, 0,
    "Number of Transmit descriptors");

/* Number of RX descriptors per ring */
static int ixv_rxd = DEFAULT_RXD;
TUNABLE_INT("hw.ixv.rxd", &ixv_rxd);
SYSCTL_INT(_hw_ixv, OID_AUTO, rxd, CTLFLAG_RDTUN, &ixv_rxd, 0,
    "Number of Receive descriptors");

/* Legacy Transmit (single queue) */
static int ixv_enable_legacy_tx = 0;
TUNABLE_INT("hw.ixv.enable_legacy_tx", &ixv_enable_legacy_tx);
SYSCTL_INT(_hw_ixv, OID_AUTO, enable_legacy_tx, CTLFLAG_RDTUN,
    &ixv_enable_legacy_tx, 0, "Enable Legacy TX flow");

/*
 * Shadow VFTA table, this is needed because
 * the real filter table gets cleared during
 * a soft reset and we need to repopulate it.
 */
static u32 ixv_shadow_vfta[IXGBE_VFTA_SIZE];

static int (*ixv_start_locked)(struct ifnet *, struct tx_ring *);
static int (*ixv_ring_empty)(struct ifnet *, struct buf_ring *);

MALLOC_DEFINE(M_IXV, "ixv", "ixv driver allocations");

/************************************************************************
 * ixv_probe - Device identification routine
 *
 *   Determines if the driver should be loaded on
 *   adapter based on its PCI vendor/device ID.
 *
 *   return BUS_PROBE_DEFAULT on success, positive on failure
 ************************************************************************/
static int
ixv_probe(device_t dev)
{
	ixgbe_vendor_info_t *ent;
	u16                 pci_vendor_id = 0;
	u16                 pci_device_id = 0;
	u16                 pci_subvendor_id = 0;
	u16                 pci_subdevice_id = 0;
#define ADAPTER_NAME_BUFFER_LENGTH 256
	char                adapter_name[ADAPTER_NAME_BUFFER_LENGTH];


	pci_vendor_id = pci_get_vendor(dev);
	if (pci_vendor_id != IXGBE_INTEL_VENDOR_ID)
		return (ENXIO);

	pci_device_id = pci_get_device(dev);
	pci_subvendor_id = pci_get_subvendor(dev);
	pci_subdevice_id = pci_get_subdevice(dev);

	ent = ixv_vendor_info_array;
	while (ent->vendor_id != 0) {
		if ((pci_vendor_id == ent->vendor_id) &&
		    (pci_device_id == ent->device_id) &&
		    ((pci_subvendor_id == ent->subvendor_id) ||
		     (ent->subvendor_id == 0)) &&
		    ((pci_subdevice_id == ent->subdevice_id) ||
		     (ent->subdevice_id == 0))) {
			snprintf(adapter_name, ADAPTER_NAME_BUFFER_LENGTH,
			    "%s, Version - %s",
			    ixv_strings[ent->index], ixv_driver_version);
			device_set_desc_copy(dev, adapter_name);
			return (BUS_PROBE_DEFAULT);
		}
		ent++;
	}

	return (ENXIO);
} /* ixv_probe */

/************************************************************************
 * ixv_attach - Device initialization routine
 *
 *   Called when the driver is being loaded.
 *   Identifies the type of hardware, allocates all resources
 *   and initializes the hardware.
 *
 *   return 0 on success, positive on failure
 ************************************************************************/
static int
ixv_attach(device_t dev)
{
	struct ixgbe_softc  *sc;
	struct ixgbe_hw *hw;
	int             error = 0;

	INIT_DEBUGOUT("ixv_attach: begin");

	/*
	 * Make sure BUSMASTER is set, on a VM under
	 * KVM it may not be and will break things.
	 */
	pci_enable_busmaster(dev);

	/* Allocate, clear, and link in our adapter structure */
	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->hw.back = sc;
	hw = &sc->hw;

	sc->init_locked = ixv_init_locked;
	sc->stop_locked = ixv_stop;

	/* Core Lock Init*/
	IXGBE_CORE_LOCK_INIT(sc, device_get_nameunit(dev));

	/* Do base PCI setup - map BAR0 */
	if (ixv_allocate_pci_resources(sc)) {
		device_printf(dev, "ixv_allocate_pci_resources() failed!\n");
		error = ENXIO;
		goto err_out;
	}

	/* SYSCTL APIs */
#ifdef IXGBE_DEBUG
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "debug",
	    CTLTYPE_INT | CTLFLAG_RW, sc, 0, ixv_sysctl_debug, "I",
	    "Debug Info");
#endif

	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "enable_aim", CTLFLAG_RW, &ixv_enable_aim, 1,
	    "Interrupt Moderation");

	/* Set up the timer callout */
	callout_init_mtx(&sc->timer, &sc->core_mtx, 0);

	/* Save off the information about this board */
	hw->vendor_id = pci_get_vendor(dev);
	hw->device_id = pci_get_device(dev);
	hw->revision_id = pci_get_revid(dev);
	hw->subsystem_vendor_id = pci_get_subvendor(dev);
	hw->subsystem_device_id = pci_get_subdevice(dev);

	/* A subset of set_mac_type */
	switch (hw->device_id) {
	case IXGBE_DEV_ID_82599_VF:
		hw->mac.type = ixgbe_mac_82599_vf;
		break;
	case IXGBE_DEV_ID_X540_VF:
	case IXGBE_DEV_ID_X540_VF_HV:
		hw->mac.type = ixgbe_mac_X540_vf;
		break;
	case IXGBE_DEV_ID_X550_VF:
		hw->mac.type = ixgbe_mac_X550_vf;
		break;
	case IXGBE_DEV_ID_X550EM_X_VF:
		hw->mac.type = ixgbe_mac_X550EM_x_vf;
		break;
	case IXGBE_DEV_ID_X550EM_A_VF:
		hw->mac.type = ixgbe_mac_X550EM_a_vf;
		break;
	default:
		/* Shouldn't get here since probe succeeded */
		device_printf(dev, "Unknown device ID!\n");
		error = ENXIO;
		goto err_out;
		break;
	}

	ixv_init_device_features(sc);

	/* Initialize the shared code */
	error = ixgbe_init_ops_vf(hw);
	if (error) {
		device_printf(dev, "ixgbe_init_ops_vf() failed!\n");
		error = EIO;
		goto err_out;
	}

	/* Setup the mailbox */
	ixgbe_init_mbx_params_vf(hw);

	//hw->mac.max_tx_queues = 2;
	//hw->mac.max_rx_queues = 2;

	/* Set the right number of segments */
	sc->num_segs = IXGBE_82599_SCATTER;

	error = hw->mac.ops.reset_hw(hw);
	if (error == IXGBE_ERR_RESET_FAILED)
		device_printf(dev, "...reset_hw() failure: Reset Failed!\n");
	else if (error)
		device_printf(dev, "...reset_hw() failed with error %d\n",
		    error);
	if (error) {
		error = EIO;
		goto err_out;
	}

	error = hw->mac.ops.init_hw(hw);
	if (error) {
		device_printf(dev, "...init_hw() failed with error %d\n",
		    error);
		error = EIO;
		goto err_out;
	}

	/* Negotiate mailbox API version */
	error = ixv_negotiate_api(sc);
	if (error) {
		device_printf(dev,
		    "Mailbox API negotiation failed during attach!\n");
		goto err_out;
	}

	/* If no mac address was assigned, make a random one */
	if (!ixv_check_ether_addr(hw->mac.addr)) {
		u8 addr[ETHER_ADDR_LEN];
		arc4rand(&addr, sizeof(addr), 0);
		addr[0] &= 0xFE;
		addr[0] |= 0x02;
		bcopy(addr, hw->mac.addr, sizeof(addr));
		bcopy(addr, hw->mac.perm_addr, sizeof(addr));
	}

	/* Register for VLAN events */
	sc->vlan_attach = EVENTHANDLER_REGISTER(vlan_config,
	    ixv_register_vlan, sc, EVENTHANDLER_PRI_FIRST);
	sc->vlan_detach = EVENTHANDLER_REGISTER(vlan_unconfig,
	    ixv_unregister_vlan, sc, EVENTHANDLER_PRI_FIRST);

	/* Sysctls for limiting the amount of work done in the taskqueues */
	ixv_set_sysctl_value(sc, "rx_processing_limit",
	    "max number of rx packets to process",
	    &sc->rx_process_limit, ixv_rx_process_limit);

	ixv_set_sysctl_value(sc, "tx_processing_limit",
	    "max number of tx packets to process",
	    &sc->tx_process_limit, ixv_tx_process_limit);

	/* Do descriptor calc and sanity checks */
	if (((ixv_txd * sizeof(union ixgbe_adv_tx_desc)) % DBA_ALIGN) != 0 ||
	    ixv_txd < MIN_TXD || ixv_txd > MAX_TXD) {
		device_printf(dev, "TXD config issue, using default!\n");
		sc->num_tx_desc = DEFAULT_TXD;
	} else
		sc->num_tx_desc = ixv_txd;

	if (((ixv_rxd * sizeof(union ixgbe_adv_rx_desc)) % DBA_ALIGN) != 0 ||
	    ixv_rxd < MIN_RXD || ixv_rxd > MAX_RXD) {
		device_printf(dev, "RXD config issue, using default!\n");
		sc->num_rx_desc = DEFAULT_RXD;
	} else
		sc->num_rx_desc = ixv_rxd;

	device_printf(dev, "Using %d RX descriptors and %d TX descriptors\n",
			sc->num_rx_desc, sc->num_tx_desc);

	/* Setup MSI-X */
	error = ixv_configure_interrupts(sc);
	if (error)
		goto err_out;

	/* Allocate our TX/RX Queues */
	if (ixgbe_allocate_queues(sc)) {
		device_printf(dev, "ixgbe_allocate_queues() failed!\n");
		error = ENOMEM;
		goto err_out;
	}

	/* Setup OS specific network interface */
	ixv_setup_interface(dev, sc);

	error = ixv_allocate_msix(sc);
	if (error) {
		device_printf(dev, "ixv_allocate_msix() failed!\n");
		goto err_late;
	}

	/* Allocate multicast array memory */
	sc->mta = malloc(sizeof(*sc->mta) *
	    MAX_NUM_MULTICAST_ADDRESSES, M_IXV, M_NOWAIT);
	if (sc->mta == NULL) {
		device_printf(dev, "Can not allocate multicast setup array\n");
		error = ENOMEM;
		goto err_late;
	}

	/* Check if VF was disabled by PF */
	error = hw->mac.ops.get_link_state(hw, &sc->link_enabled);
	if (error) {		
		/* PF is not capable of controlling VF state. Enable the link. */
		sc->link_enabled = TRUE;
	}

	/* Do the stats setup */
	ixv_save_stats(sc);
	ixv_init_stats(sc);
	ixv_add_stats_sysctls(sc);

	if (sc->feat_en & IXGBE_FEATURE_NETMAP)
		ixgbe_netmap_attach(sc);

	INIT_DEBUGOUT("ixv_attach: end");

	return (0);

err_late:
	ixgbe_free_transmit_structures(sc);
	ixgbe_free_receive_structures(sc);
	ixv_free_msix_queues(sc);
err_out:
	ixv_free_pci_resources(sc);
	IXGBE_CORE_LOCK_DESTROY(sc);

	return (error);
} /* ixv_attach */

/************************************************************************
 * ixv_detach - Device removal routine
 *
 *   Called when the driver is being removed.
 *   Stops the adapter and deallocates all the resources
 *   that were allocated for driver operation.
 *
 *   return 0 on success, positive on failure
 ************************************************************************/
static int
ixv_detach(device_t dev)
{
	struct ixgbe_softc  *sc = device_get_softc(dev);
	struct ix_queue *que = sc->queues;

	INIT_DEBUGOUT("ixv_detach: begin");

	/* Make sure VLANS are not using driver */
	if (sc->ifp->if_vlantrunk != NULL) {
		device_printf(dev, "Vlan in use, detach first\n");
		return (EBUSY);
	}

	ether_ifdetach(sc->ifp);
	IXGBE_CORE_LOCK(sc);
	ixv_stop(sc);
	IXGBE_CORE_UNLOCK(sc);

	free(sc->mta, M_IXV);

	for (int i = 0; i < sc->num_queues; i++, que++) {
		if (que->tq) {
			struct tx_ring  *txr = que->txr;
			taskqueue_drain(que->tq, &txr->txq_task);
			taskqueue_drain(que->tq, &que->que_task);
			taskqueue_free(que->tq);
		}
	}

	/* Drain the Mailbox(link) queue */
	if (sc->tq) {
		taskqueue_drain(sc->tq, &sc->link_task);
		taskqueue_free(sc->tq);
	}

	/* Unregister VLAN events */
	if (sc->vlan_attach != NULL)
		EVENTHANDLER_DEREGISTER(vlan_config, sc->vlan_attach);
	if (sc->vlan_detach != NULL)
		EVENTHANDLER_DEREGISTER(vlan_unconfig, sc->vlan_detach);

	callout_drain(&sc->timer);

	if (sc->feat_en & IXGBE_FEATURE_NETMAP)
		netmap_detach(sc->ifp);

	ixv_free_msix_queues(sc);
	ixv_free_pci_resources(sc);
	bus_generic_detach(dev);
	if_free(sc->ifp);

	ixgbe_free_transmit_structures(sc);
	ixgbe_free_receive_structures(sc);

	IXGBE_CORE_LOCK_DESTROY(sc);

	return (0);
} /* ixv_detach */

/************************************************************************
 * ixv_init_locked - Init entry point
 *
 *   Used in two ways: It is used by the stack as an init entry
 *   point in network interface structure. It is also used
 *   by the driver as a hw/sw initialization routine to get
 *   to a consistent state.
 *
 *   return 0 on success, positive on failure
 ************************************************************************/
void
ixv_init_locked(struct ixgbe_softc *sc)
{
	struct ifnet    *ifp = sc->ifp;
	device_t        dev = sc->dev;
	struct ixgbe_hw *hw = &sc->hw;
	int             error = 0;

	INIT_DEBUGOUT("ixv_init_locked: begin");
	mtx_assert(&sc->core_mtx, MA_OWNED);
	hw->adapter_stopped = FALSE;
	hw->mac.ops.stop_adapter(hw);
	callout_stop(&sc->timer);

	/* reprogram the RAR[0] in case user changed it. */
	hw->mac.ops.set_rar(hw, 0, hw->mac.addr, 0, IXGBE_RAH_AV);

	/* Get the latest mac address, User can use a LAA */
	bcopy(IF_LLADDR(sc->ifp), hw->mac.addr,
	    IXGBE_ETH_LENGTH_OF_ADDRESS);
	hw->mac.ops.set_rar(hw, 0, hw->mac.addr, 0, 1);

	/* Prepare transmit descriptors and buffers */
	if (ixgbe_setup_transmit_structures(sc)) {
		device_printf(dev, "Could not setup transmit structures\n");
		ixv_stop(sc);
		return;
	}

	/* Reset VF and renegotiate mailbox API version */
	hw->mac.ops.reset_hw(hw);
	error = ixv_negotiate_api(sc);
	if (error) {
		device_printf(dev,
		    "Mailbox API negotiation failed in init_locked!\n");
		return;
	}

	ixv_initialize_transmit_units(sc);

	/* Setup Multicast table */
	ixv_set_multi(sc);

	/*
	 * Determine the correct mbuf pool
	 * for doing jumbo/headersplit
	 */
	if (ifp->if_mtu > ETHERMTU)
		sc->rx_mbuf_sz = MJUMPAGESIZE;
	else
		sc->rx_mbuf_sz = MCLBYTES;

	/* Prepare receive descriptors and buffers */
	if (ixgbe_setup_receive_structures(sc)) {
		device_printf(dev, "Could not setup receive structures\n");
		ixv_stop(sc);
		return;
	}

	/* Configure RX settings */
	ixv_initialize_receive_units(sc);

	/* Set the various hardware offload abilities */
	ifp->if_hwassist = 0;
	if (ifp->if_capenable & IFCAP_TSO4)
		ifp->if_hwassist |= CSUM_IP_TSO;
	if (ifp->if_capenable & IFCAP_TSO6)
		ifp->if_hwassist |= CSUM_IP6_TSO;
	if (ifp->if_capenable & IFCAP_TXCSUM) {
		ifp->if_hwassist |= (CSUM_IP | CSUM_IP_UDP | CSUM_IP_TCP);
		ifp->if_hwassist |= CSUM_IP_SCTP;
	}
	if (ifp->if_capenable & IFCAP_TXCSUM_IPV6) {
		ifp->if_hwassist |= (CSUM_IP6_UDP | CSUM_IP6_TCP);
		ifp->if_hwassist |= CSUM_IP6_SCTP;
	}

	/* Set up VLAN offload and filter */
	ixv_setup_vlan_support(sc);

	/* Set up MSI-X routing */
	ixv_configure_ivars(sc);

	/* Set up auto-mask */
	IXGBE_WRITE_REG(hw, IXGBE_VTEIAM, IXGBE_EICS_RTX_QUEUE);

	/* Set moderation on the Link interrupt */
	IXGBE_WRITE_REG(hw, IXGBE_VTEITR(sc->vector), IXGBE_LINK_ITR);

	/* Stats init */
	ixv_init_stats(sc);

	/* Config/Enable Link */
	error = hw->mac.ops.get_link_state(hw, &sc->link_enabled);
	if (error) {
		/* PF is not capable of controlling VF state. Enable the link. */
		sc->link_enabled = TRUE;
	} else if (sc->link_enabled == FALSE)
		device_printf(dev, "VF is disabled by PF\n");

	hw->mac.ops.check_link(hw, &sc->link_speed, &sc->link_up,
	    FALSE);

	/* Start watchdog */
	callout_reset(&sc->timer, hz, ixv_local_timer, sc);

	/* And now turn on interrupts */
	ixv_enable_intr(sc);

	/* Now inform the stack we're ready */
	ifp->if_drv_flags |= IFF_DRV_RUNNING;
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;

	return;
} /* ixv_init_locked */

/*
 * MSI-X Interrupt Handlers and Tasklets
 */

static inline void
ixv_enable_queue(struct ixgbe_softc *sc, u32 vector)
{
	struct ixgbe_hw *hw = &sc->hw;
	u32             queue = 1 << vector;
	u32             mask;

	mask = (IXGBE_EIMS_RTX_QUEUE & queue);
	IXGBE_WRITE_REG(hw, IXGBE_VTEIMS, mask);
} /* ixv_enable_queue */

static inline void
ixv_disable_queue(struct ixgbe_softc *sc, u32 vector)
{
	struct ixgbe_hw *hw = &sc->hw;
	u64             queue = (1ULL << vector);
	u32             mask;

	mask = (IXGBE_EIMS_RTX_QUEUE & queue);
	IXGBE_WRITE_REG(hw, IXGBE_VTEIMC, mask);
} /* ixv_disable_queue */

static inline void
ixv_rearm_queues(struct ixgbe_softc *sc, u64 queues)
{
	u32 mask = (IXGBE_EIMS_RTX_QUEUE & queues);
	IXGBE_WRITE_REG(&sc->hw, IXGBE_VTEICS, mask);
} /* ixv_rearm_queues */


/************************************************************************
 * ixv_msix_que - MSI Queue Interrupt Service routine
 ************************************************************************/
void
ixv_msix_que(void *arg)
{
	struct ix_queue *que = arg;
	struct ixgbe_softc  *sc = que->sc;
	struct ifnet    *ifp = sc->ifp;
	struct tx_ring  *txr = que->txr;
	struct rx_ring  *rxr = que->rxr;
	bool            more;
	u32             newitr = 0;

	ixv_disable_queue(sc, que->msix);
	++que->irqs;

	more = ixgbe_rxeof(que);

	IXGBE_TX_LOCK(txr);
	ixgbe_txeof(txr);
	/*
	 * Make certain that if the stack
	 * has anything queued the task gets
	 * scheduled to handle it.
	 */
	if (!ixv_ring_empty(sc->ifp, txr->br))
		ixv_start_locked(ifp, txr);
	IXGBE_TX_UNLOCK(txr);

	/* Do AIM now? */

	if (ixv_enable_aim == FALSE)
		goto no_calc;
	/*
	 * Do Adaptive Interrupt Moderation:
	 *  - Write out last calculated setting
	 *  - Calculate based on average size over
	 *    the last interval.
	 */
	if (que->eitr_setting)
		IXGBE_WRITE_REG(&sc->hw, IXGBE_VTEITR(que->msix),
		    que->eitr_setting);

	que->eitr_setting = 0;

	/* Idle, do nothing */
	if ((txr->bytes == 0) && (rxr->bytes == 0))
		goto no_calc;

	if ((txr->bytes) && (txr->packets))
		newitr = txr->bytes/txr->packets;
	if ((rxr->bytes) && (rxr->packets))
		newitr = max(newitr, (rxr->bytes / rxr->packets));
	newitr += 24; /* account for hardware frame, crc */

	/* set an upper boundary */
	newitr = min(newitr, 3000);

	/* Be nice to the mid range */
	if ((newitr > 300) && (newitr < 1200))
		newitr = (newitr / 3);
	else
		newitr = (newitr / 2);

	newitr |= newitr << 16;

	/* save for next interrupt */
	que->eitr_setting = newitr;

	/* Reset state */
	txr->bytes = 0;
	txr->packets = 0;
	rxr->bytes = 0;
	rxr->packets = 0;

no_calc:
	if (more)
		taskqueue_enqueue(que->tq, &que->que_task);
	else /* Reenable this interrupt */
		ixv_enable_queue(sc, que->msix);

	return;
} /* ixv_msix_que */

/************************************************************************
 * ixv_msix_mbx
 ************************************************************************/
static void
ixv_msix_mbx(void *arg)
{
	struct ixgbe_softc  *sc = arg;
	struct ixgbe_hw *hw = &sc->hw;
	u32             reg;

	++sc->link_irq;

	/* First get the cause */
	reg = IXGBE_READ_REG(hw, IXGBE_VTEICS);
	/* Clear interrupt with write */
	IXGBE_WRITE_REG(hw, IXGBE_VTEICR, reg);

	/* Link status change */
	if (reg & IXGBE_EICR_LSC)
		taskqueue_enqueue(sc->tq, &sc->link_task);

	IXGBE_WRITE_REG(hw, IXGBE_VTEIMS, IXGBE_EIMS_OTHER);

	return;
} /* ixv_msix_mbx */

/************************************************************************
 * ixv_media_status - Media Ioctl callback
 *
 *   Called whenever the user queries the status of
 *   the interface using ifconfig.
 ************************************************************************/
static void
ixv_media_status(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	struct ixgbe_softc *sc = ifp->if_softc;

	INIT_DEBUGOUT("ixv_media_status: begin");
	IXGBE_CORE_LOCK(sc);
	ixv_update_link_status(sc);

	ifmr->ifm_status = IFM_AVALID;
	ifmr->ifm_active = IFM_ETHER;

	if (!sc->link_active) {
		IXGBE_CORE_UNLOCK(sc);
		return;
	}

	ifmr->ifm_status |= IFM_ACTIVE;

	switch (sc->link_speed) {
		case IXGBE_LINK_SPEED_1GB_FULL:
			ifmr->ifm_active |= IFM_1000_T | IFM_FDX;
			break;
		case IXGBE_LINK_SPEED_10GB_FULL:
			ifmr->ifm_active |= IFM_10G_T | IFM_FDX;
			break;
		case IXGBE_LINK_SPEED_100_FULL:
			ifmr->ifm_active |= IFM_100_TX | IFM_FDX;
			break;
		case IXGBE_LINK_SPEED_10_FULL:
			ifmr->ifm_active |= IFM_10_T | IFM_FDX;
			break;
	}

	IXGBE_CORE_UNLOCK(sc);

	return;
} /* ixv_media_status */

/************************************************************************
 * ixv_media_change - Media Ioctl callback
 *
 *   Called when the user changes speed/duplex using
 *   media/mediopt option with ifconfig.
 ************************************************************************/
static int
ixv_media_change(struct ifnet *ifp)
{
	struct ixgbe_softc *sc = ifp->if_softc;
	struct ifmedia *ifm = &sc->media;

	INIT_DEBUGOUT("ixv_media_change: begin");

	if (IFM_TYPE(ifm->ifm_media) != IFM_ETHER)
		return (EINVAL);

	switch (IFM_SUBTYPE(ifm->ifm_media)) {
	case IFM_AUTO:
		break;
	default:
		device_printf(sc->dev, "Only auto media type\n");
		return (EINVAL);
	}

	return (0);
} /* ixv_media_change */


/************************************************************************
 * ixv_negotiate_api
 *
 *   Negotiate the Mailbox API with the PF;
 *   start with the most featured API first.
 ************************************************************************/
static int
ixv_negotiate_api(struct ixgbe_softc *sc)
{
	struct ixgbe_hw *hw = &sc->hw;
	int             mbx_api[] = { ixgbe_mbox_api_12,
	                              ixgbe_mbox_api_11,
	                              ixgbe_mbox_api_10,
	                              ixgbe_mbox_api_unknown };
	int             i = 0;

	while (mbx_api[i] != ixgbe_mbox_api_unknown) {
		if (ixgbevf_negotiate_api_version(hw, mbx_api[i]) == 0)
			return (0);
		i++;
	}

	return (EINVAL);
} /* ixv_negotiate_api */


/**
 * ixv_mc_filter_apply - Save MAC address in MTA table for this VF
 * @arg: pointer to the device softc
 * @sdl: MAC multicast address
 * @cnt: number of mac addresses added
 *
 * Save a multicast address for addition to the function's Multicast Address
 * Table.  Intended to be used with the map-like function if_foreach_llmaddr().
 *
 * @returns 1 on success, or 0 if no more multicast addresses can be added
 */
static u_int
ixv_mc_filter_apply(void *arg, struct sockaddr_dl *sdl, u_int count)
{
	struct ixgbe_softc *sc = arg;
	struct ixgbe_mc_addr *mta = sc->mta;

	if (count >= MAX_NUM_MULTICAST_ADDRESSES)
		return (0);

	bcopy(LLADDR(sdl), mta[count].addr, IXGBE_ETH_LENGTH_OF_ADDRESS);

	return (1);
} /* ixv_mc_filter_apply */

/************************************************************************
 * ixv_set_multi - Multicast Update
 *
 *   Called whenever multicast address list is updated.
 ************************************************************************/
static void
ixv_set_multi(struct ixgbe_softc *sc)
{
	struct ixgbe_mc_addr *mta;
	u8                 *update_ptr;
	struct ifnet       *ifp = sc->ifp;
	int                mcnt = 0;

	IOCTL_DEBUGOUT("ixv_set_multi: begin");

	mta = sc->mta;
	bzero(mta, sizeof(*mta) * MAX_NUM_MULTICAST_ADDRESSES);

	mcnt = if_foreach_llmaddr(ifp, ixv_mc_filter_apply, sc);

	update_ptr = (u8 *)mta;
	sc->hw.mac.ops.update_mc_addr_list(&sc->hw, update_ptr, mcnt,
	    ixv_mc_array_itr, TRUE);

	return;
} /* ixv_set_multi */

/************************************************************************
 * ixv_mc_array_itr
 *
 *   An iterator function needed by the multicast shared code.
 *   It feeds the shared code routine the addresses in the
 *   array of ixv_set_multi() one by one.
 ************************************************************************/
static u8 *
ixv_mc_array_itr(struct ixgbe_hw *hw, u8 **update_ptr, u32 *vmdq)
{
	struct ixgbe_mc_addr *mta;

	mta = (struct ixgbe_mc_addr *)*update_ptr;

	*vmdq = 0;
	*update_ptr = (u8*)(mta + 1);

	return (mta->addr);
} /* ixv_mc_array_itr */

/************************************************************************
 * ixv_local_timer - Timer routine
 *
 *   Checks for link status, updates statistics,
 *   and runs the watchdog check.
 ************************************************************************/
static void
ixv_local_timer(void *arg)
{
	struct ixgbe_softc  *sc = arg;
	device_t        dev = sc->dev;
	struct ix_queue *que = sc->queues;
	u64             queues = 0;
	int             hung = 0;

	mtx_assert(&sc->core_mtx, MA_OWNED);

	if (ixv_check_link(sc)) {
		ixv_init_locked(sc);
		return;
	}

	/* Stats Update */
	ixv_update_stats(sc);

	/*
	 * Check the TX queues status
	 *      - mark hung queues so we don't schedule on them
	 *      - watchdog only if all queues show hung
	 */
	for (int i = 0; i < sc->num_queues; i++, que++) {
		/* Keep track of queues with work for soft irq */
		if (que->txr->busy)
			queues |= ((u64)1 << que->me);
		/*
		 * Each time txeof runs without cleaning, but there
		 * are uncleaned descriptors it increments busy. If
		 * we get to the MAX we declare it hung.
		 */
		if (que->busy == IXGBE_QUEUE_HUNG) {
			++hung;
			/* Mark the queue as inactive */
			sc->active_queues &= ~((u64)1 << que->me);
			continue;
		} else {
			/* Check if we've come back from hung */
			if ((sc->active_queues & ((u64)1 << que->me)) == 0)
				sc->active_queues |= ((u64)1 << que->me);
		}
		if (que->busy >= IXGBE_MAX_TX_BUSY) {
			device_printf(dev,
			    "Warning queue %d appears to be hung!\n", i);
			que->txr->busy = IXGBE_QUEUE_HUNG;
			++hung;
		}

	}

	/* Only truly watchdog if all queues show hung */
	if (hung == sc->num_queues)
		goto watchdog;
	else if (queues != 0) { /* Force an IRQ on queues with work */
		ixv_rearm_queues(sc, queues);
	}

	callout_reset(&sc->timer, hz, ixv_local_timer, sc);

	return;

watchdog:

	device_printf(sc->dev, "Watchdog timeout -- resetting\n");
	sc->ifp->if_drv_flags &= ~IFF_DRV_RUNNING;
	sc->watchdog_events++;
	ixv_init_locked(sc);
} /* ixv_local_timer */

/************************************************************************
 * ixv_update_link_status - Update OS on link state
 *
 * Note: Only updates the OS on the cached link state.
 *       The real check of the hardware only happens with
 *       a link interrupt.
 ************************************************************************/
static void
ixv_update_link_status(struct ixgbe_softc *sc)
{
	struct ifnet *ifp = sc->ifp;
	device_t     dev = sc->dev;

	if (sc->link_up && sc->link_enabled) {
		if (sc->link_active == FALSE) {
			if (bootverbose)
				device_printf(dev,"Link is up %d Gbps %s \n",
				    ((sc->link_speed == 128) ? 10 : 1),
				    "Full Duplex");
			sc->link_active = TRUE;
			if_link_state_change(ifp, LINK_STATE_UP);
		}
	} else { /* Link down */
		if (sc->link_active == TRUE) {
			if (bootverbose)
				device_printf(dev,"Link is Down\n");
			if_link_state_change(ifp, LINK_STATE_DOWN);
			sc->link_active = FALSE;
		}
	}

	return;
} /* ixv_update_link_status */


/************************************************************************
 * ixv_stop - Stop the hardware
 *
 *   Disables all traffic on the adapter by issuing a
 *   global reset on the MAC and deallocates TX/RX buffers.
 ************************************************************************/
static void
ixv_stop(void *arg)
{
	struct ifnet    *ifp;
	struct ixgbe_softc  *sc = arg;
	struct ixgbe_hw *hw = &sc->hw;

	ifp = sc->ifp;

	mtx_assert(&sc->core_mtx, MA_OWNED);

	INIT_DEBUGOUT("ixv_stop: begin\n");
	ixv_disable_intr(sc);

	/* Tell the stack that the interface is no longer active */
	ifp->if_drv_flags &= ~(IFF_DRV_RUNNING | IFF_DRV_OACTIVE);

	hw->mac.ops.reset_hw(hw);
	sc->hw.adapter_stopped = FALSE;
	hw->mac.ops.stop_adapter(hw);
	callout_stop(&sc->timer);

	/* reprogram the RAR[0] in case user changed it. */
	hw->mac.ops.set_rar(hw, 0, hw->mac.addr, 0, IXGBE_RAH_AV);

	return;
} /* ixv_stop */


/************************************************************************
 * ixv_allocate_pci_resources
 ************************************************************************/
static int
ixv_allocate_pci_resources(struct ixgbe_softc *sc)
{
	device_t dev = sc->dev;
	int      rid;

	rid = PCIR_BAR(0);
	sc->pci_mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);

	if (!(sc->pci_mem)) {
		device_printf(dev, "Unable to allocate bus resource: memory\n");
		return (ENXIO);
	}

	sc->osdep.mem_bus_space_tag = rman_get_bustag(sc->pci_mem);
	sc->osdep.mem_bus_space_handle =
	    rman_get_bushandle(sc->pci_mem);
	sc->hw.hw_addr = (u8 *)&sc->osdep.mem_bus_space_handle;

	/* Pick up the tuneable queues */
	if (ixv_num_queues <= 0) {
		device_printf(dev, "ixv_num_queues (%d) is invalid, using default (1).\n", ixv_num_queues);
		sc->num_queues = 1;
	} else
		sc->num_queues = ixv_num_queues;

	return (0);
} /* ixv_allocate_pci_resources */

/************************************************************************
 * ixv_free_pci_resources
 ************************************************************************/
static void
ixv_free_pci_resources(struct ixgbe_softc *sc)
{
	device_t        dev = sc->dev;
	int             rid, memrid;

	memrid = PCIR_BAR(MSIX_82598_BAR);

	/*
	 * There is a slight possibility of a failure mode
	 * in attach that will result in entering this function
	 * before interrupt resources have been initialized, and
	 * in that case we do not want to execute the loops below
	 * We can detect this reliably by the state of the adapter
	 * res pointer.
	 */
	if (sc->res == NULL)
		goto mem;

	/* Clean the Mailbox interrupt last */
	rid = sc->vector + 1;

	if (sc->tag != NULL) {
		bus_teardown_intr(dev, sc->res, sc->tag);
		sc->tag = NULL;
	}
	if (sc->res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, rid, sc->res);

mem:
	pci_release_msi(dev);

	if (sc->msix_mem != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, memrid,
		    sc->msix_mem);

	if (sc->pci_mem != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, PCIR_BAR(0),
		    sc->pci_mem);

	return;
} /* ixv_free_pci_resources */

/************************************************************************
 * ixv_free_msix_queues
 ************************************************************************/
static void
ixv_free_msix_queues(struct ixgbe_softc *sc)
{
	device_t        dev = sc->dev;
	struct ix_queue *que = sc->queues;
	int 		rid;

	/*
	 *  Release all msix queue resources:
	 */
	for (int i = 0; i < sc->num_queues; i++, que++) {
		rid = que->msix + 1;
		if (que->tag != NULL) {
			bus_teardown_intr(dev, que->res, que->tag);
			que->tag = NULL;
		}
		if (que->res != NULL)
			bus_release_resource(dev, SYS_RES_IRQ, rid, que->res);
	}

	free(sc->queues, M_IXV);
} /* ixv_free_msix_queues */

/************************************************************************
 * ixv_setup_interface
 *
 *   Setup networking device structure and register an interface.
 ************************************************************************/
static void
ixv_setup_interface(device_t dev, struct ixgbe_softc *sc)
{
	struct ifnet *ifp;

	INIT_DEBUGOUT("ixv_setup_interface: begin");

	ifp = sc->ifp = if_alloc(IFT_ETHER);
	if (ifp == NULL)
		panic("%s: can not if_alloc()\n", device_get_nameunit(dev));
	if_initname(ifp, device_get_name(dev), device_get_unit(dev));
	ifp->if_baudrate = IF_Gbps(10);
	ifp->if_init = ixv_init;
	ifp->if_softc = sc;
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_ioctl = ixv_ioctl;
	if_setgetcounterfn(ifp, ixv_get_counter);
	/* TSO parameters */
	ifp->if_hw_tsomax = 65518;
	ifp->if_hw_tsomaxsegcount = IXGBE_82599_SCATTER;
	ifp->if_hw_tsomaxsegsize = 2048;
	if (sc->feat_en & IXGBE_FEATURE_LEGACY_TX) {
		ifp->if_start = ixgbe_legacy_start;
		ixv_start_locked = ixgbe_legacy_start_locked;
		ixv_ring_empty = ixgbe_legacy_ring_empty;
	} else {
		ifp->if_transmit = ixgbe_mq_start;
		ifp->if_qflush = ixgbe_qflush;
		ixv_start_locked = ixgbe_mq_start_locked;
		ixv_ring_empty = drbr_empty;
	}
	IFQ_SET_MAXLEN(&ifp->if_snd, sc->num_tx_desc - 2);

	ether_ifattach(ifp, sc->hw.mac.addr);

	sc->max_frame_size = ifp->if_mtu + IXGBE_MTU_HDR;

	/*
	 * Tell the upper layer(s) we support long frames.
	 */
	ifp->if_hdrlen = sizeof(struct ether_vlan_header);

	/* Set capability flags */
	ifp->if_capabilities |= IFCAP_HWCSUM
	                     |  IFCAP_HWCSUM_IPV6
	                     |  IFCAP_TSO
	                     |  IFCAP_LRO
	                     |  IFCAP_VLAN_HWTAGGING
	                     |  IFCAP_VLAN_HWTSO
	                     |  IFCAP_VLAN_HWCSUM
	                     |  IFCAP_JUMBO_MTU
	                     |  IFCAP_VLAN_MTU;

	/* Frame limit applies to PF devices that must enable
	 * jumbo frames in order to accomodate VLAN headers on
	 * top of MTU payload/header.
	 */
	if (sc->feat_en & IXGBE_FEATURE_FRAME_LIMIT) {
		sc->max_frame_size -= ETHER_VLAN_ENCAP_LEN;
		ifp->if_capabilities &= ~IFCAP_VLAN_MTU;
	}

	/* Enable the above capabilities by default */
	ifp->if_capenable = ifp->if_capabilities;

	/*
	 * Specify the media types supported by this adapter and register
	 * callbacks to update media and link information
	 */
	ifmedia_init(&sc->media, IFM_IMASK, ixv_media_change,
	    ixv_media_status);
	ifmedia_add(&sc->media, IFM_ETHER | IFM_AUTO, 0, NULL);
	ifmedia_set(&sc->media, IFM_ETHER | IFM_AUTO);

	return;
} /* ixv_setup_interface */


/************************************************************************
 * ixv_initialize_transmit_units - Enable transmit unit.
 ************************************************************************/
static void
ixv_initialize_transmit_units(struct ixgbe_softc *sc)
{
	struct tx_ring  *txr = sc->tx_rings;
	struct ixgbe_hw *hw = &sc->hw;


	for (int i = 0; i < sc->num_queues; i++, txr++) {
		u64 tdba = txr->txdma.dma_paddr;
		u32 txctrl, txdctl;

		/* Set WTHRESH to 8, burst writeback */
		txdctl = IXGBE_READ_REG(hw, IXGBE_VFTXDCTL(i));
		txdctl |= (8 << 16);
		IXGBE_WRITE_REG(hw, IXGBE_VFTXDCTL(i), txdctl);

		/* Set the HW Tx Head and Tail indices */
		IXGBE_WRITE_REG(&sc->hw, IXGBE_VFTDH(i), 0);
		IXGBE_WRITE_REG(&sc->hw, IXGBE_VFTDT(i), 0);

		/* Set Tx Tail register */
		txr->tail = IXGBE_VFTDT(i);

		/* Set Ring parameters */
		IXGBE_WRITE_REG(hw, IXGBE_VFTDBAL(i),
		    (tdba & 0x00000000ffffffffULL));
		IXGBE_WRITE_REG(hw, IXGBE_VFTDBAH(i), (tdba >> 32));
		IXGBE_WRITE_REG(hw, IXGBE_VFTDLEN(i),
		    sc->num_tx_desc * sizeof(struct ixgbe_legacy_tx_desc));
		txctrl = IXGBE_READ_REG(hw, IXGBE_VFDCA_TXCTRL(i));
		txctrl &= ~IXGBE_DCA_TXCTRL_DESC_WRO_EN;
		IXGBE_WRITE_REG(hw, IXGBE_VFDCA_TXCTRL(i), txctrl);

		/* Now enable */
		txdctl = IXGBE_READ_REG(hw, IXGBE_VFTXDCTL(i));
		txdctl |= IXGBE_TXDCTL_ENABLE;
		IXGBE_WRITE_REG(hw, IXGBE_VFTXDCTL(i), txdctl);
	}

	return;
} /* ixv_initialize_transmit_units */


/************************************************************************
 * ixv_initialize_rss_mapping
 ************************************************************************/
static void
ixv_initialize_rss_mapping(struct ixgbe_softc *sc)
{
	struct ixgbe_hw *hw = &sc->hw;
	u32             reta = 0, mrqc, rss_key[10];
	int             queue_id;
	int             i, j;
	u32             rss_hash_config;

	if (sc->feat_en & IXGBE_FEATURE_RSS) {
		/* Fetch the configured RSS key */
		rss_getkey((uint8_t *)&rss_key);
	} else {
		/* set up random bits */
		arc4rand(&rss_key, sizeof(rss_key), 0);
	}

	/* Now fill out hash function seeds */
	for (i = 0; i < 10; i++)
		IXGBE_WRITE_REG(hw, IXGBE_VFRSSRK(i), rss_key[i]);

	/* Set up the redirection table */
	for (i = 0, j = 0; i < 64; i++, j++) {
		if (j == sc->num_queues)
			j = 0;

		if (sc->feat_en & IXGBE_FEATURE_RSS) {
			/*
			 * Fetch the RSS bucket id for the given indirection
			 * entry. Cap it at the number of configured buckets
			 * (which is num_queues.)
			 */
			queue_id = rss_get_indirection_to_bucket(i);
			queue_id = queue_id % sc->num_queues;
		} else
			queue_id = j;

		/*
		 * The low 8 bits are for hash value (n+0);
		 * The next 8 bits are for hash value (n+1), etc.
		 */
		reta >>= 8;
		reta |= ((uint32_t)queue_id) << 24;
		if ((i & 3) == 3) {
			IXGBE_WRITE_REG(hw, IXGBE_VFRETA(i >> 2), reta);
			reta = 0;
		}
	}

	/* Perform hash on these packet types */
	if (sc->feat_en & IXGBE_FEATURE_RSS)
		rss_hash_config = rss_gethashconfig();
	else {
		/*
		 * Disable UDP - IP fragments aren't currently being handled
		 * and so we end up with a mix of 2-tuple and 4-tuple
		 * traffic.
		 */
		rss_hash_config = RSS_HASHTYPE_RSS_IPV4
		                | RSS_HASHTYPE_RSS_TCP_IPV4
		                | RSS_HASHTYPE_RSS_IPV6
		                | RSS_HASHTYPE_RSS_TCP_IPV6;
	}

	mrqc = IXGBE_MRQC_RSSEN;
	if (rss_hash_config & RSS_HASHTYPE_RSS_IPV4)
		mrqc |= IXGBE_MRQC_RSS_FIELD_IPV4;
	if (rss_hash_config & RSS_HASHTYPE_RSS_TCP_IPV4)
		mrqc |= IXGBE_MRQC_RSS_FIELD_IPV4_TCP;
	if (rss_hash_config & RSS_HASHTYPE_RSS_IPV6)
		mrqc |= IXGBE_MRQC_RSS_FIELD_IPV6;
	if (rss_hash_config & RSS_HASHTYPE_RSS_TCP_IPV6)
		mrqc |= IXGBE_MRQC_RSS_FIELD_IPV6_TCP;
	if (rss_hash_config & RSS_HASHTYPE_RSS_IPV6_EX)
		device_printf(sc->dev, "%s: RSS_HASHTYPE_RSS_IPV6_EX defined, but not supported\n",
		    __func__);
	if (rss_hash_config & RSS_HASHTYPE_RSS_TCP_IPV6_EX)
		device_printf(sc->dev, "%s: RSS_HASHTYPE_RSS_TCP_IPV6_EX defined, but not supported\n",
		    __func__);
	if (rss_hash_config & RSS_HASHTYPE_RSS_UDP_IPV4)
		mrqc |= IXGBE_MRQC_RSS_FIELD_IPV4_UDP;
	if (rss_hash_config & RSS_HASHTYPE_RSS_UDP_IPV4_EX)
		device_printf(sc->dev, "%s: RSS_HASHTYPE_RSS_UDP_IPV4_EX defined, but not supported\n",
		    __func__);
	if (rss_hash_config & RSS_HASHTYPE_RSS_UDP_IPV6)
		mrqc |= IXGBE_MRQC_RSS_FIELD_IPV6_UDP;
	if (rss_hash_config & RSS_HASHTYPE_RSS_UDP_IPV6_EX)
		device_printf(sc->dev, "%s: RSS_HASHTYPE_RSS_UDP_IPV6_EX defined, but not supported\n",
		    __func__);
	IXGBE_WRITE_REG(hw, IXGBE_VFMRQC, mrqc);
} /* ixv_initialize_rss_mapping */


/************************************************************************
 * ixv_initialize_receive_units - Setup receive registers and features.
 ************************************************************************/
static void
ixv_initialize_receive_units(struct ixgbe_softc *sc)
{
	struct rx_ring  *rxr = sc->rx_rings;
	struct ixgbe_hw *hw = &sc->hw;
	struct ifnet    *ifp = sc->ifp;
	u32             bufsz, psrtype;

	if (ifp->if_mtu > ETHERMTU)
		bufsz = 4096 >> IXGBE_SRRCTL_BSIZEPKT_SHIFT;
	else
		bufsz = 2048 >> IXGBE_SRRCTL_BSIZEPKT_SHIFT;

	psrtype = IXGBE_PSRTYPE_TCPHDR
	        | IXGBE_PSRTYPE_UDPHDR
	        | IXGBE_PSRTYPE_IPV4HDR
	        | IXGBE_PSRTYPE_IPV6HDR
	        | IXGBE_PSRTYPE_L2HDR;

	if (sc->num_queues > 1)
		psrtype |= 1 << 29;

	IXGBE_WRITE_REG(hw, IXGBE_VFPSRTYPE, psrtype);

	/* Tell PF our max_frame size */
	if (ixgbevf_rlpml_set_vf(hw, sc->max_frame_size) != 0) {
		device_printf(sc->dev, "There is a problem with the PF setup.  It is likely the receive unit for this VF will not function correctly.\n");
	}

	for (int i = 0; i < sc->num_queues; i++, rxr++) {
		u64 rdba = rxr->rxdma.dma_paddr;
		u32 reg, rxdctl;

		/* Disable the queue */
		rxdctl = IXGBE_READ_REG(hw, IXGBE_VFRXDCTL(i));
		rxdctl &= ~IXGBE_RXDCTL_ENABLE;
		IXGBE_WRITE_REG(hw, IXGBE_VFRXDCTL(i), rxdctl);
		for (int j = 0; j < 10; j++) {
			if (IXGBE_READ_REG(hw, IXGBE_VFRXDCTL(i)) &
			    IXGBE_RXDCTL_ENABLE)
				msec_delay(1);
			else
				break;
		}
		wmb();
		/* Setup the Base and Length of the Rx Descriptor Ring */
		IXGBE_WRITE_REG(hw, IXGBE_VFRDBAL(i),
		    (rdba & 0x00000000ffffffffULL));
		IXGBE_WRITE_REG(hw, IXGBE_VFRDBAH(i), (rdba >> 32));
		IXGBE_WRITE_REG(hw, IXGBE_VFRDLEN(i),
		    sc->num_rx_desc * sizeof(union ixgbe_adv_rx_desc));

		/* Reset the ring indices */
		IXGBE_WRITE_REG(hw, IXGBE_VFRDH(rxr->me), 0);
		IXGBE_WRITE_REG(hw, IXGBE_VFRDT(rxr->me), 0);

		/* Set up the SRRCTL register */
		reg = IXGBE_READ_REG(hw, IXGBE_VFSRRCTL(i));
		reg &= ~IXGBE_SRRCTL_BSIZEHDR_MASK;
		reg &= ~IXGBE_SRRCTL_BSIZEPKT_MASK;
		reg |= bufsz;
		reg |= IXGBE_SRRCTL_DESCTYPE_ADV_ONEBUF;
		IXGBE_WRITE_REG(hw, IXGBE_VFSRRCTL(i), reg);

		/* Capture Rx Tail index */
		rxr->tail = IXGBE_VFRDT(rxr->me);

		/* Do the queue enabling last */
		rxdctl |= IXGBE_RXDCTL_ENABLE | IXGBE_RXDCTL_VME;
		IXGBE_WRITE_REG(hw, IXGBE_VFRXDCTL(i), rxdctl);
		for (int k = 0; k < 10; k++) {
			if (IXGBE_READ_REG(hw, IXGBE_VFRXDCTL(i)) &
			    IXGBE_RXDCTL_ENABLE)
				break;
			msec_delay(1);
		}
		wmb();

		/* Set the Tail Pointer */
		/*
		 * In netmap mode, we must preserve the buffers made
		 * available to userspace before the if_init()
		 * (this is true by default on the TX side, because
		 * init makes all buffers available to userspace).
		 *
		 * netmap_reset() and the device specific routines
		 * (e.g. ixgbe_setup_receive_rings()) map these
		 * buffers at the end of the NIC ring, so here we
		 * must set the RDT (tail) register to make sure
		 * they are not overwritten.
		 *
		 * In this driver the NIC ring starts at RDH = 0,
		 * RDT points to the last slot available for reception (?),
		 * so RDT = num_rx_desc - 1 means the whole ring is available.
		 */
#ifdef DEV_NETMAP
		if ((sc->feat_en & IXGBE_FEATURE_NETMAP) &&
		    (ifp->if_capenable & IFCAP_NETMAP)) {
			struct netmap_adapter *na = NA(sc->ifp);
#if ((__FreeBSD_version >= 1102505 && __FreeBSD_version < 1200000) || \
    __FreeBSD_version >= 1200062)
			struct netmap_kring *kring = na->rx_rings[i];
#else
			struct netmap_kring *kring = &na->rx_rings[i];
#endif
			int t = na->num_rx_desc - 1 - nm_kr_rxspace(kring);

			IXGBE_WRITE_REG(hw, IXGBE_VFRDT(rxr->me), t);
		} else
#endif /* DEV_NETMAP */
			IXGBE_WRITE_REG(hw, IXGBE_VFRDT(rxr->me),
			    sc->num_rx_desc - 1);
	}

	ixv_initialize_rss_mapping(sc);

	return;
} /* ixv_initialize_receive_units */

/************************************************************************
 * ixv_setup_vlan_support
 ************************************************************************/
static void
ixv_setup_vlan_support(struct ixgbe_softc *sc)
{
	struct ixgbe_hw *hw = &sc->hw;
	u32             ctrl, vid, vfta, retry;

	/*
	 * We get here thru init_locked, meaning
	 * a soft reset, this has already cleared
	 * the VFTA and other state, so if there
	 * have been no vlan's registered do nothing.
	 */
	if (sc->num_vlans == 0)
		return;

	/* Enable the queues */
	for (int i = 0; i < sc->num_queues; i++) {
		ctrl = IXGBE_READ_REG(hw, IXGBE_VFRXDCTL(i));
		ctrl |= IXGBE_RXDCTL_VME;
		IXGBE_WRITE_REG(hw, IXGBE_VFRXDCTL(i), ctrl);
		/*
		 * Let Rx path know that it needs to store VLAN tag
		 * as part of extra mbuf info.
		 */
		sc->rx_rings[i].vtag_strip = TRUE;
	}

	/*
	 * A soft reset zero's out the VFTA, so
	 * we need to repopulate it now.
	 */
	for (int i = 0; i < IXGBE_VFTA_SIZE; i++) {
		if (ixv_shadow_vfta[i] == 0)
			continue;
		vfta = ixv_shadow_vfta[i];
		/*
		 * Reconstruct the vlan id's
		 * based on the bits set in each
		 * of the array ints.
		 */
		for (int j = 0; j < 32; j++) {
			retry = 0;
			if ((vfta & (1 << j)) == 0)
				continue;
			vid = (i * 32) + j;
			/* Call the shared code mailbox routine */
			while (hw->mac.ops.set_vfta(hw, vid, 0, TRUE, FALSE)) {
				if (++retry > 5)
					break;
			}
		}
	}
} /* ixv_setup_vlan_support */

/************************************************************************
 * ixv_register_vlan
 *
 *   Run via a vlan config EVENT, it enables us to use the
 *   HW Filter table since we can get the vlan id. This just
 *   creates the entry in the soft version of the VFTA, init
 *   will repopulate the real table.
 ************************************************************************/
static void
ixv_register_vlan(void *arg, struct ifnet *ifp, u16 vtag)
{
	struct ixgbe_softc *sc = ifp->if_softc;
	u16            index, bit;

	if (ifp->if_softc != arg) /* Not our event */
		return;

	if ((vtag == 0) || (vtag > 4095)) /* Invalid */
		return;

	IXGBE_CORE_LOCK(sc);
	index = (vtag >> 5) & 0x7F;
	bit = vtag & 0x1F;
	ixv_shadow_vfta[index] |= (1 << bit);
	++sc->num_vlans;
	/* Re-init to load the changes */
	ixv_init_locked(sc);
	IXGBE_CORE_UNLOCK(sc);
} /* ixv_register_vlan */

/************************************************************************
 * ixv_unregister_vlan
 *
 *   Run via a vlan unconfig EVENT, remove our entry
 *   in the soft vfta.
 ************************************************************************/
static void
ixv_unregister_vlan(void *arg, struct ifnet *ifp, u16 vtag)
{
	struct ixgbe_softc *sc = ifp->if_softc;
	u16            index, bit;

	if (ifp->if_softc !=  arg)
		return;

	if ((vtag == 0) || (vtag > 4095))  /* Invalid */
		return;

	IXGBE_CORE_LOCK(sc);
	index = (vtag >> 5) & 0x7F;
	bit = vtag & 0x1F;
	ixv_shadow_vfta[index] &= ~(1 << bit);
	--sc->num_vlans;
	/* Re-init to load the changes */
	ixv_init_locked(sc);
	IXGBE_CORE_UNLOCK(sc);
} /* ixv_unregister_vlan */

/************************************************************************
 * ixv_enable_intr
 ************************************************************************/
static void
ixv_enable_intr(struct ixgbe_softc *sc)
{
	struct ixgbe_hw *hw = &sc->hw;
	struct ix_queue *que = sc->queues;
	u32             mask = (IXGBE_EIMS_ENABLE_MASK & ~IXGBE_EIMS_RTX_QUEUE);


	IXGBE_WRITE_REG(hw, IXGBE_VTEIMS, mask);

	mask = IXGBE_EIMS_ENABLE_MASK;
	mask &= ~(IXGBE_EIMS_OTHER | IXGBE_EIMS_LSC);
	IXGBE_WRITE_REG(hw, IXGBE_VTEIAC, mask);

	for (int i = 0; i < sc->num_queues; i++, que++)
		ixv_enable_queue(sc, que->msix);

	IXGBE_WRITE_FLUSH(hw);

	return;
} /* ixv_enable_intr */

/************************************************************************
 * ixv_disable_intr
 ************************************************************************/
static void
ixv_disable_intr(struct ixgbe_softc *sc)
{
	IXGBE_WRITE_REG(&sc->hw, IXGBE_VTEIAC, 0);
	IXGBE_WRITE_REG(&sc->hw, IXGBE_VTEIMC, ~0);
	IXGBE_WRITE_FLUSH(&sc->hw);

	return;
} /* ixv_disable_intr */

/************************************************************************
 * ixv_set_ivar
 *
 *   Setup the correct IVAR register for a particular MSI-X interrupt
 *    - entry is the register array entry
 *    - vector is the MSI-X vector for this queue
 *    - type is RX/TX/MISC
 ************************************************************************/
static void
ixv_set_ivar(struct ixgbe_softc *sc, u8 entry, u8 vector, s8 type)
{
	struct ixgbe_hw *hw = &sc->hw;
	u32             ivar, index;

	vector |= IXGBE_IVAR_ALLOC_VAL;

	if (type == -1) { /* MISC IVAR */
		ivar = IXGBE_READ_REG(hw, IXGBE_VTIVAR_MISC);
		ivar &= ~0xFF;
		ivar |= vector;
		IXGBE_WRITE_REG(hw, IXGBE_VTIVAR_MISC, ivar);
	} else {          /* RX/TX IVARS */
		index = (16 * (entry & 1)) + (8 * type);
		ivar = IXGBE_READ_REG(hw, IXGBE_VTIVAR(entry >> 1));
		ivar &= ~(0xFF << index);
		ivar |= (vector << index);
		IXGBE_WRITE_REG(hw, IXGBE_VTIVAR(entry >> 1), ivar);
	}
} /* ixv_set_ivar */

/************************************************************************
 * ixv_configure_ivars
 ************************************************************************/
static void
ixv_configure_ivars(struct ixgbe_softc *sc)
{
	struct ix_queue *que = sc->queues;

	for (int i = 0; i < sc->num_queues; i++, que++) {
		/* First the RX queue entry */
		ixv_set_ivar(sc, i, que->msix, 0);
		/* ... and the TX */
		ixv_set_ivar(sc, i, que->msix, 1);
		/* Set an initial value in EITR */
		IXGBE_WRITE_REG(&sc->hw, IXGBE_VTEITR(que->msix),
		    IXGBE_EITR_DEFAULT);
	}

	/* For the mailbox interrupt */
	ixv_set_ivar(sc, 1, sc->vector, -1);
} /* ixv_configure_ivars */

/************************************************************************
 * ixv_get_counter
 ************************************************************************/
static uint64_t
ixv_get_counter(struct ifnet *ifp, ift_counter cnt)
{
	struct ixgbe_softc *sc;

	sc = if_getsoftc(ifp);

	switch (cnt) {
	case IFCOUNTER_IPACKETS:
		return (sc->ipackets);
	case IFCOUNTER_OPACKETS:
		return (sc->opackets);
	case IFCOUNTER_IBYTES:
		return (sc->ibytes);
	case IFCOUNTER_OBYTES:
		return (sc->obytes);
	case IFCOUNTER_IMCASTS:
		return (sc->imcasts);
	default:
		return (if_get_counter_default(ifp, cnt));
	}
} /* ixv_get_counter */

/************************************************************************
 * ixv_save_stats
 *
 *   The VF stats registers never have a truly virgin
 *   starting point, so this routine tries to make an
 *   artificial one, marking ground zero on attach as
 *   it were.
 ************************************************************************/
static void
ixv_save_stats(struct ixgbe_softc *sc)
{
	if (sc->stats_vf.vfgprc || sc->stats_vf.vfgptc) {
		sc->stats_vf.saved_reset_vfgprc +=
		    sc->stats_vf.vfgprc - sc->stats_vf.base_vfgprc;
		sc->stats_vf.saved_reset_vfgptc +=
		    sc->stats_vf.vfgptc - sc->stats_vf.base_vfgptc;
		sc->stats_vf.saved_reset_vfgorc +=
		    sc->stats_vf.vfgorc - sc->stats_vf.base_vfgorc;
		sc->stats_vf.saved_reset_vfgotc +=
		    sc->stats_vf.vfgotc - sc->stats_vf.base_vfgotc;
		sc->stats_vf.saved_reset_vfmprc +=
		    sc->stats_vf.vfmprc - sc->stats_vf.base_vfmprc;
	}
} /* ixv_save_stats */

/************************************************************************
 * ixv_init_stats
 ************************************************************************/
static void
ixv_init_stats(struct ixgbe_softc *sc)
{
	struct ixgbe_hw *hw = &sc->hw;

	sc->stats_vf.last_vfgprc = IXGBE_READ_REG(hw, IXGBE_VFGPRC);
	sc->stats_vf.last_vfgorc = IXGBE_READ_REG(hw, IXGBE_VFGORC_LSB);
	sc->stats_vf.last_vfgorc |=
	    (((u64)(IXGBE_READ_REG(hw, IXGBE_VFGORC_MSB))) << 32);

	sc->stats_vf.last_vfgptc = IXGBE_READ_REG(hw, IXGBE_VFGPTC);
	sc->stats_vf.last_vfgotc = IXGBE_READ_REG(hw, IXGBE_VFGOTC_LSB);
	sc->stats_vf.last_vfgotc |=
	    (((u64)(IXGBE_READ_REG(hw, IXGBE_VFGOTC_MSB))) << 32);

	sc->stats_vf.last_vfmprc = IXGBE_READ_REG(hw, IXGBE_VFMPRC);

	sc->stats_vf.base_vfgprc = sc->stats_vf.last_vfgprc;
	sc->stats_vf.base_vfgorc = sc->stats_vf.last_vfgorc;
	sc->stats_vf.base_vfgptc = sc->stats_vf.last_vfgptc;
	sc->stats_vf.base_vfgotc = sc->stats_vf.last_vfgotc;
	sc->stats_vf.base_vfmprc = sc->stats_vf.last_vfmprc;
} /* ixv_init_stats */

#define UPDATE_STAT_32(reg, last, count)                \
{                                                       \
	u32 current = IXGBE_READ_REG(hw, reg);          \
	if (current < last)                             \
		count += 0x100000000LL;                 \
	last = current;                                 \
	count &= 0xFFFFFFFF00000000LL;                  \
	count |= current;                               \
}

#define UPDATE_STAT_36(lsb, msb, last, count)           \
{                                                       \
	u64 cur_lsb = IXGBE_READ_REG(hw, lsb);          \
	u64 cur_msb = IXGBE_READ_REG(hw, msb);          \
	u64 current = ((cur_msb << 32) | cur_lsb);      \
	if (current < last)                             \
		count += 0x1000000000LL;                \
	last = current;                                 \
	count &= 0xFFFFFFF000000000LL;                  \
	count |= current;                               \
}

/************************************************************************
 * ixv_update_stats - Update the board statistics counters.
 ************************************************************************/
void
ixv_update_stats(struct ixgbe_softc *sc)
{
	struct ixgbe_hw *hw = &sc->hw;
	struct ixgbevf_hw_stats *stats = &sc->stats_vf;

	UPDATE_STAT_32(IXGBE_VFGPRC, sc->stats_vf.last_vfgprc,
	    sc->stats_vf.vfgprc);
	UPDATE_STAT_32(IXGBE_VFGPTC, sc->stats_vf.last_vfgptc,
	    sc->stats_vf.vfgptc);
	UPDATE_STAT_36(IXGBE_VFGORC_LSB, IXGBE_VFGORC_MSB,
	    sc->stats_vf.last_vfgorc, sc->stats_vf.vfgorc);
	UPDATE_STAT_36(IXGBE_VFGOTC_LSB, IXGBE_VFGOTC_MSB,
	    sc->stats_vf.last_vfgotc, sc->stats_vf.vfgotc);
	UPDATE_STAT_32(IXGBE_VFMPRC, sc->stats_vf.last_vfmprc,
	    sc->stats_vf.vfmprc);

	/* Fill out the OS statistics structure */
	IXGBE_SET_IPACKETS(sc, stats->vfgprc);
	IXGBE_SET_OPACKETS(sc, stats->vfgptc);
	IXGBE_SET_IBYTES(sc, stats->vfgorc);
	IXGBE_SET_OBYTES(sc, stats->vfgotc);
	IXGBE_SET_IMCASTS(sc, stats->vfmprc);
} /* ixv_update_stats */

/************************************************************************
 * ixv_add_stats_sysctls - Add statistic sysctls for the VF.
 ************************************************************************/
static void
ixv_add_stats_sysctls(struct ixgbe_softc *sc)
{
	device_t                dev = sc->dev;
	struct tx_ring          *txr = sc->tx_rings;
	struct rx_ring          *rxr = sc->rx_rings;
	struct sysctl_ctx_list  *ctx = device_get_sysctl_ctx(dev);
	struct sysctl_oid       *tree = device_get_sysctl_tree(dev);
	struct sysctl_oid_list  *child = SYSCTL_CHILDREN(tree);
	struct ixgbevf_hw_stats *stats = &sc->stats_vf;
	struct sysctl_oid       *stat_node, *queue_node;
	struct sysctl_oid_list  *stat_list, *queue_list;

#define QUEUE_NAME_LEN 32
	char                    namebuf[QUEUE_NAME_LEN];

	/* Driver Statistics */
	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "dropped",
	    CTLFLAG_RD, &sc->dropped_pkts, "Driver dropped packets");
	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "mbuf_defrag_failed",
	    CTLFLAG_RD, &sc->mbuf_defrag_failed, "m_defrag() failed");
	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "watchdog_events",
	    CTLFLAG_RD, &sc->watchdog_events, "Watchdog timeouts");
	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "link_irq",
	    CTLFLAG_RD, &sc->link_irq, "Link MSI-X IRQ Handled");

	for (int i = 0; i < sc->num_queues; i++, txr++) {
		snprintf(namebuf, QUEUE_NAME_LEN, "queue%d", i);
		queue_node = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, namebuf,
		    CTLFLAG_RD, NULL, "Queue Name");
		queue_list = SYSCTL_CHILDREN(queue_node);

		SYSCTL_ADD_UQUAD(ctx, queue_list, OID_AUTO, "irqs",
		    CTLFLAG_RD, &(sc->queues[i].irqs), "IRQs on queue");
		SYSCTL_ADD_UQUAD(ctx, queue_list, OID_AUTO, "no_tx_dma_setup",
		    CTLFLAG_RD, &(txr->no_tx_dma_setup),
		    "Driver Tx DMA failure in Tx");
		SYSCTL_ADD_UQUAD(ctx, queue_list, OID_AUTO, "tx_no_desc",
		    CTLFLAG_RD, &(txr->no_desc_avail),
		    "Not-enough-descriptors count: TX");
		SYSCTL_ADD_UQUAD(ctx, queue_list, OID_AUTO, "tx_packets",
		    CTLFLAG_RD, &(txr->total_packets), "TX Packets");
		if (!(sc->feat_en & IXGBE_FEATURE_LEGACY_TX)) {
			SYSCTL_ADD_UQUAD(ctx, queue_list, OID_AUTO, "br_drops",
			    CTLFLAG_RD, &txr->br->br_drops,
			    "Packets dropped in buf_ring");
		}
	}

	for (int i = 0; i < sc->num_queues; i++, rxr++) {
		snprintf(namebuf, QUEUE_NAME_LEN, "queue%d", i);
		queue_node = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, namebuf,
		    CTLFLAG_RD, NULL, "Queue Name");
		queue_list = SYSCTL_CHILDREN(queue_node);

		SYSCTL_ADD_UQUAD(ctx, queue_list, OID_AUTO, "rx_packets",
		    CTLFLAG_RD, &(rxr->rx_packets), "RX packets");
		SYSCTL_ADD_UQUAD(ctx, queue_list, OID_AUTO, "rx_bytes",
		    CTLFLAG_RD, &(rxr->rx_bytes), "RX bytes");
		SYSCTL_ADD_UQUAD(ctx, queue_list, OID_AUTO, "rx_discarded",
		    CTLFLAG_RD, &(rxr->rx_discarded), "Discarded RX packets");
	}

	stat_node = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, "mac",
	    CTLFLAG_RD, NULL, "VF Statistics (read from HW registers)");
	stat_list = SYSCTL_CHILDREN(stat_node);

	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "good_pkts_rcvd",
	    CTLFLAG_RD, &stats->vfgprc, "Good Packets Received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "good_octets_rcvd",
	    CTLFLAG_RD, &stats->vfgorc, "Good Octets Received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "mcast_pkts_rcvd",
	    CTLFLAG_RD, &stats->vfmprc, "Multicast Packets Received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "good_pkts_txd",
	    CTLFLAG_RD, &stats->vfgptc, "Good Packets Transmitted");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "good_octets_txd",
	    CTLFLAG_RD, &stats->vfgotc, "Good Octets Transmitted");
} /* ixv_add_stats_sysctls */

/************************************************************************
 * ixv_set_sysctl_value
 ************************************************************************/
static void
ixv_set_sysctl_value(struct ixgbe_softc *sc, const char *name,
	const char *description, int *limit, int value)
{
	*limit = value;
	SYSCTL_ADD_INT(device_get_sysctl_ctx(sc->dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev)),
	    OID_AUTO, name, CTLFLAG_RW, limit, value, description);
} /* ixv_set_sysctl_value */

#ifdef IXGBE_DEBUG
/************************************************************************
 * ixv_print_debug_info
 *
 *   Provides a way to take a look at important statistics
 *   maintained by the driver and hardware.
 ************************************************************************/
static void
ixv_print_debug_info(struct ixgbe_softc *sc)
{
	device_t        dev = sc->dev;
	struct ix_queue *que = sc->queues;
	struct rx_ring  *rxr;
	struct tx_ring  *txr;
	struct lro_ctrl *lro;

	for (int i = 0; i < sc->num_queues; i++, que++) {
		txr = que->txr;
		rxr = que->rxr;
		lro = &rxr->lro;
		device_printf(dev, "QUE(%d) IRQs Handled: %lu\n",
		    que->msix, (long)que->irqs);
		device_printf(dev, "RX(%d) Packets Received: %lld\n",
		    rxr->me, (long long)rxr->rx_packets);
		device_printf(dev, "RX(%d) Bytes Received: %lu\n",
		    rxr->me, (long)rxr->rx_bytes);
		device_printf(dev, "RX(%d) LRO Queued= %ju\n",
		    rxr->me, (uintmax_t)lro->lro_queued);
		device_printf(dev, "RX(%d) LRO Flushed= %ju\n",
		    rxr->me, (uintmax_t)lro->lro_flushed);
		device_printf(dev, "TX(%d) Packets Sent: %lu\n",
		    txr->me, (long)txr->total_packets);
		device_printf(dev, "TX(%d) NO Desc Avail: %lu\n",
		    txr->me, (long)txr->no_desc_avail);
	}

	device_printf(dev, "MBX IRQ Handled: %lu\n", (long)sc->link_irq);
} /* ixv_print_debug_info */

/************************************************************************
 * ixv_sysctl_debug
 ************************************************************************/
static int
ixv_sysctl_debug(SYSCTL_HANDLER_ARGS)
{
	struct ixgbe_softc *sc;
	int            error, result;

	result = -1;
	error = sysctl_handle_int(oidp, &result, 0, req);

	if (error || !req->newptr)
		return (error);

	if (result == 1) {
		sc = (struct ixgbe_softc *)arg1;
		ixv_print_debug_info(sc);
	}

	return error;
} /* ixv_sysctl_debug */
#endif

/************************************************************************
 * ixv_init_device_features
 ************************************************************************/
static void
ixv_init_device_features(struct ixgbe_softc *sc)
{
	sc->feat_cap = IXGBE_FEATURE_NETMAP
	                  | IXGBE_FEATURE_VF
	                  | IXGBE_FEATURE_RSS
	                  | IXGBE_FEATURE_LEGACY_TX;

	/* A tad short on feature flags for VFs, atm. */
	switch (sc->hw.mac.type) {
	case ixgbe_mac_82599_vf:
		sc->feat_cap |= IXGBE_FEATURE_FRAME_LIMIT;
		break;
	case ixgbe_mac_X540_vf:
		break;
	case ixgbe_mac_X550_vf:
	case ixgbe_mac_X550EM_x_vf:
	case ixgbe_mac_X550EM_a_vf:
		sc->feat_cap |= IXGBE_FEATURE_NEEDS_CTXD;
	default:
		break;
	}

	/* Enabled by default... */
	/* Is a virtual function (VF) */
	if (sc->feat_cap & IXGBE_FEATURE_VF)
		sc->feat_en |= IXGBE_FEATURE_VF;
	/* Netmap */
	if (sc->feat_cap & IXGBE_FEATURE_NETMAP)
		sc->feat_en |= IXGBE_FEATURE_NETMAP;
	/* Receive-Side Scaling (RSS) */
	if (sc->feat_cap & IXGBE_FEATURE_RSS)
		sc->feat_en |= IXGBE_FEATURE_RSS;
	/* Frame size limitation */
	if (sc->feat_cap & IXGBE_FEATURE_FRAME_LIMIT)
		sc->feat_en |= IXGBE_FEATURE_FRAME_LIMIT;
	/* Needs advanced context descriptor regardless of offloads req'd */
	if (sc->feat_cap & IXGBE_FEATURE_NEEDS_CTXD)
		sc->feat_en |= IXGBE_FEATURE_NEEDS_CTXD;

	/* Enabled via sysctl... */
	/* Legacy (single queue) transmit */
	if ((sc->feat_cap & IXGBE_FEATURE_LEGACY_TX) &&
	    ixv_enable_legacy_tx)
		sc->feat_en |= IXGBE_FEATURE_LEGACY_TX;
} /* ixv_init_device_features */

/************************************************************************
 * ixv_shutdown - Shutdown entry point
 ************************************************************************/
static int
ixv_shutdown(device_t dev)
{
	struct ixgbe_softc *sc = device_get_softc(dev);
	IXGBE_CORE_LOCK(sc);
	ixv_stop(sc);
	IXGBE_CORE_UNLOCK(sc);

	return (0);
} /* ixv_shutdown */


/************************************************************************
 * ixv_ioctl - Ioctl entry point
 *
 *   Called when the user wants to configure the interface.
 *
 *   return 0 on success, positive on failure
 ************************************************************************/
static int
ixv_ioctl(struct ifnet *ifp, u_long command, caddr_t data)
{
	struct ixgbe_softc *sc = ifp->if_softc;
	struct ifreq   *ifr = (struct ifreq *)data;
#if defined(INET) || defined(INET6)
	struct ifaddr  *ifa = (struct ifaddr *)data;
	bool           avoid_reset = FALSE;
#endif
	int            error = 0;

	switch (command) {

	case SIOCSIFADDR:
#ifdef INET
		if (ifa->ifa_addr->sa_family == AF_INET)
			avoid_reset = TRUE;
#endif
#ifdef INET6
		if (ifa->ifa_addr->sa_family == AF_INET6)
			avoid_reset = TRUE;
#endif
#if defined(INET) || defined(INET6)
		/*
		 * Calling init results in link renegotiation,
		 * so we avoid doing it when possible.
		 */
		if (avoid_reset) {
			ifp->if_flags |= IFF_UP;
			if (!(ifp->if_drv_flags & IFF_DRV_RUNNING))
				ixv_init(sc);
			if (!(ifp->if_flags & IFF_NOARP))
				arp_ifinit(ifp, ifa);
		} else
			error = ether_ioctl(ifp, command, data);
		break;
#endif
	case SIOCSIFMTU:
		IOCTL_DEBUGOUT("ioctl: SIOCSIFMTU (Set Interface MTU)");
		if (ifr->ifr_mtu > IXGBE_MAX_MTU) {
			error = EINVAL;
		} else {
			IXGBE_CORE_LOCK(sc);
			ifp->if_mtu = ifr->ifr_mtu;
			sc->max_frame_size =
			    ifp->if_mtu + IXGBE_MTU_HDR;
			if (sc->feat_en & IXGBE_FEATURE_FRAME_LIMIT)
				sc->max_frame_size -= ETHER_VLAN_ENCAP_LEN;
			ixv_init_locked(sc);
			IXGBE_CORE_UNLOCK(sc);
		}
		break;
	case SIOCSIFFLAGS:
		IOCTL_DEBUGOUT("ioctl: SIOCSIFFLAGS (Set Interface Flags)");
		IXGBE_CORE_LOCK(sc);
		if (ifp->if_flags & IFF_UP) {
			if ((ifp->if_drv_flags & IFF_DRV_RUNNING) == 0)
				ixv_init_locked(sc);
		} else
			if (ifp->if_drv_flags & IFF_DRV_RUNNING)
				ixv_stop(sc);
		sc->if_flags = ifp->if_flags;
		IXGBE_CORE_UNLOCK(sc);
		break;
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		IOCTL_DEBUGOUT("ioctl: SIOC(ADD|DEL)MULTI");
		if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
			IXGBE_CORE_LOCK(sc);
			ixv_disable_intr(sc);
			ixv_set_multi(sc);
			ixv_enable_intr(sc);
			IXGBE_CORE_UNLOCK(sc);
		}
		break;
	case SIOCSIFMEDIA:
	case SIOCGIFMEDIA:
		IOCTL_DEBUGOUT("ioctl: SIOCxIFMEDIA (Get/Set Interface Media)");
		error = ifmedia_ioctl(ifp, ifr, &sc->media, command);
		break;
	case SIOCSIFCAP:
	{
		int mask = ifr->ifr_reqcap ^ ifp->if_capenable;
		IOCTL_DEBUGOUT("ioctl: SIOCSIFCAP (Set Capabilities)");
		/* HW cannot turn rxcsum and rxcsum6 on/off separately */
		if (mask & (IFCAP_RXCSUM | IFCAP_RXCSUM_IPV6)) {
			ifp->if_capenable ^= IFCAP_RXCSUM;
			ifp->if_capenable ^= IFCAP_RXCSUM_IPV6;
		}
		if (mask & IFCAP_TXCSUM)
			ifp->if_capenable ^= IFCAP_TXCSUM;
		if (mask & IFCAP_TXCSUM_IPV6)
			ifp->if_capenable ^= IFCAP_TXCSUM_IPV6;
		if (mask & IFCAP_TSO4)
			ifp->if_capenable ^= IFCAP_TSO4;
		if (mask & IFCAP_TSO6)
			ifp->if_capenable ^= IFCAP_TSO6;
		if (mask & IFCAP_LRO)
			ifp->if_capenable ^= IFCAP_LRO;
		if (mask & IFCAP_VLAN_HWTAGGING)
			ifp->if_capenable ^= IFCAP_VLAN_HWTAGGING;
		if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
			IXGBE_CORE_LOCK(sc);
			ixv_init_locked(sc);
			IXGBE_CORE_UNLOCK(sc);
		}
		VLAN_CAPABILITIES(ifp);
		break;
	}

	default:
		IOCTL_DEBUGOUT1("ioctl: UNKNOWN (0x%X)\n", (int)command);
		error = ether_ioctl(ifp, command, data);
		break;
	}

	return (error);
} /* ixv_ioctl */

/************************************************************************
 * ixv_init
 ************************************************************************/
static void
ixv_init(void *arg)
{
	struct ixgbe_softc *sc = arg;

	IXGBE_CORE_LOCK(sc);
	ixv_init_locked(sc);
	IXGBE_CORE_UNLOCK(sc);

	return;
} /* ixv_init */


static void
ixv_handle_que(void *context, int pending)
{
	struct ix_queue *que = context;
	struct ixgbe_softc  *sc = que->sc;
	struct tx_ring  *txr = que->txr;
	struct ifnet    *ifp = sc->ifp;
	bool            more;

	if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
		more = ixgbe_rxeof(que);
		IXGBE_TX_LOCK(txr);
		ixgbe_txeof(txr);
		if (!ixv_ring_empty(ifp, txr->br))
			ixv_start_locked(ifp, txr);
		IXGBE_TX_UNLOCK(txr);
		if (more) {
			taskqueue_enqueue(que->tq, &que->que_task);
			return;
		}
	}

	/* Reenable this interrupt */
	ixv_enable_queue(sc, que->msix);

	return;
} /* ixv_handle_que */

/************************************************************************
 * ixv_allocate_msix - Setup MSI-X Interrupt resources and handlers
 ************************************************************************/
static int
ixv_allocate_msix(struct ixgbe_softc *sc)
{
	device_t        dev = sc->dev;
	struct ix_queue *que = sc->queues;
	struct tx_ring  *txr = sc->tx_rings;
	int             error, msix_ctrl, rid, vector = 0;

	for (int i = 0; i < sc->num_queues; i++, vector++, que++, txr++) {
		rid = vector + 1;
		que->res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
		    RF_SHAREABLE | RF_ACTIVE);
		if (que->res == NULL) {
			device_printf(dev, "Unable to allocate bus resource: que interrupt [%d]\n",
			    vector);
			return (ENXIO);
		}
		/* Set the handler function */
		error = bus_setup_intr(dev, que->res,
		    INTR_TYPE_NET | INTR_MPSAFE, NULL,
		    ixv_msix_que, que, &que->tag);
		if (error) {
			que->res = NULL;
			device_printf(dev, "Failed to register QUE handler");
			return (error);
		}
		bus_describe_intr(dev, que->res, que->tag, "que %d", i);
		que->msix = vector;
		sc->active_queues |= (1ULL << que->msix);
		/*
		 * Bind the MSI-X vector, and thus the
		 * ring to the corresponding CPU.
		 */
		if (sc->num_queues > 1)
			bus_bind_intr(dev, que->res, i);
		TASK_INIT(&txr->txq_task, 0, ixgbe_deferred_mq_start, txr);
		TASK_INIT(&que->que_task, 0, ixv_handle_que, que);
		que->tq = taskqueue_create_fast("ixv_que", M_NOWAIT,
		    taskqueue_thread_enqueue, &que->tq);
		taskqueue_start_threads(&que->tq, 1, PI_NET, "%s que",
		    device_get_nameunit(sc->dev));
	}

	/* and Mailbox */
	rid = vector + 1;
	sc->res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_SHAREABLE | RF_ACTIVE);
	if (!sc->res) {
		device_printf(dev,
		    "Unable to allocate bus resource: MBX interrupt [%d]\n",
		    rid);
		return (ENXIO);
	}
	/* Set the mbx handler function */
	error = bus_setup_intr(dev, sc->res, INTR_TYPE_NET | INTR_MPSAFE,
	    NULL, ixv_msix_mbx, sc, &sc->tag);
	if (error) {
		sc->res = NULL;
		device_printf(dev, "Failed to register LINK handler");
		return (error);
	}
	bus_describe_intr(dev, sc->res, sc->tag, "mbx");
	sc->vector = vector;
	/* Tasklets for Mailbox */
	TASK_INIT(&sc->link_task, 0, ixv_handle_link, sc);
	sc->tq = taskqueue_create_fast("ixv_mbx", M_NOWAIT,
	    taskqueue_thread_enqueue, &sc->tq);
	taskqueue_start_threads(&sc->tq, 1, PI_NET, "%s mbxq",
	    device_get_nameunit(sc->dev));
	/*
	 * Due to a broken design QEMU will fail to properly
	 * enable the guest for MSI-X unless the vectors in
	 * the table are all set up, so we must rewrite the
	 * ENABLE in the MSI-X control register again at this
	 * point to cause it to successfully initialize us.
	 */
	if (sc->hw.mac.type == ixgbe_mac_82599_vf) {
		pci_find_cap(dev, PCIY_MSIX, &rid);
		rid += PCIR_MSIX_CTRL;
		msix_ctrl = pci_read_config(dev, rid, 2);
		msix_ctrl |= PCIM_MSIXCTRL_MSIX_ENABLE;
		pci_write_config(dev, rid, msix_ctrl, 2);
	}

	return (0);
} /* ixv_allocate_msix */

/************************************************************************
 * ixv_configure_interrupts - Setup MSI-X resources
 *
 *   Note: The VF device MUST use MSI-X, there is no fallback.
 ************************************************************************/
static int
ixv_configure_interrupts(struct ixgbe_softc *sc)
{
	device_t dev = sc->dev;
	int      rid, want, msgs;

	rid = PCIR_BAR(3);

	/* Must have at least 2 MSI-X vectors */
	msgs = pci_msix_count(dev);
	if (msgs < 2)
		goto out;
	sc->msix_mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->msix_mem == NULL) {
		device_printf(sc->dev, "Unable to map MSI-X table \n");
		goto out;
	}

	/*
	 * Want vectors for the queues,
	 * plus an additional for mailbox.
	 */
	want = sc->num_queues + 1;
	if (want > msgs) {
		want = msgs;
		sc->num_queues = msgs - 1;
	} else
		msgs = want;
	if ((pci_alloc_msix(dev, &msgs) == 0) && (msgs == want)) {
		device_printf(sc->dev,
		    "Using MSI-X interrupts with %d vectors\n", want);
		/* reflect correct sysctl value */
		ixv_num_queues = sc->num_queues;

		return (0);
	}
	/* Release in case alloc was insufficient */
	pci_release_msi(dev);
out:
	if (sc->msix_mem != NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY, rid,
		    sc->msix_mem);
		sc->msix_mem = NULL;
	}
	device_printf(sc->dev, "MSI-X config error\n");

	return (ENXIO);
} /* ixv_configure_interrupts */


/************************************************************************
 * ixv_handle_link - Tasklet handler for MSI-X MBX interrupts
 *
 *   Done outside of interrupt context since the driver might sleep
 ************************************************************************/
static void
ixv_handle_link(void *context, int pending)
{
	struct ixgbe_softc *sc = context;

	sc->hw.mac.ops.check_link(&sc->hw, &sc->link_speed,
	    &sc->link_up, FALSE);
	ixv_update_link_status(sc);
} /* ixv_handle_link */

/************************************************************************
 * ixv_check_link - Used in the local timer to poll for link changes
 ************************************************************************/
static s32
ixv_check_link(struct ixgbe_softc *sc)
{
	s32 error = 0;

	sc->hw.mac.get_link_status = TRUE;

	error = sc->hw.mac.ops.check_link(&sc->hw, &sc->link_speed,
	    &sc->link_up, FALSE);
	ixv_update_link_status(sc);

	return error;
} /* ixv_check_link */

int
ixgbe_vf_que_index(int iov_mode, int vf_id, int index)
{
	UNREFERENCED_2PARAMETER(iov_mode, vf_id);

	return index;
} /* ixgbe_vf_que_index */
