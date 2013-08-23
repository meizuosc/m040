#
# Makefile for the Linux Bluetooth BNEP layer.
#

obj-$(CONFIG_BT_BNEP) += bnep.o

ifeq ($(CONFIG_BT_MGMT),y)
bnep-objs := _mgmt/core.o _mgmt/sock.o _mgmt/netdev.o
else
bnep-objs := core.o sock.o netdev.o
endif
