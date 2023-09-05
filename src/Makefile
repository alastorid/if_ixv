#$FreeBSD$

KMODDIR ?= /boot/kernel

.include <bsd.own.mk>

.PATH:  ${.CURDIR}

KMOD    = if_ixv
SRCS    = device_if.h bus_if.h pci_if.h
SRCS    += if_ixv.c if_fdir.c ix_txrx.c ixgbe_osdep.c ixgbe_netmap.c
SRCS	+= freebsd_compat_common.c
# Shared source
SRCS    += ixgbe_mbx.c ixgbe_vf.c
CFLAGS  += -DSMP

# If building this driver as a standalone module, or using IXGBE_LEGACY_BUILD,
# keep these flags:
CFLAGS  += -DINET -DINET6 -DIXGBE_STANDALONE_BUILD -DIS_VF

# Enable debug sysctl
# CFLAGS  += -DIXGBE_DEBUG

# IXGBE_LEGACY_TX can be defined to use the older stack interface,
# however it will limit all transmission to the 0 queue, and is thus
# not recommended.
# CFLAGS += -DIXGBE_LEGACY_TX

clean:
	rm -f opt_bdg.h device_if.h bus_if.h pci_if.h setdef* *_StripErr
	rm -f *.o *.kld *.ko .depend.*
	rm -f @ export_syms machine x86

.include <bsd.kmod.mk>
