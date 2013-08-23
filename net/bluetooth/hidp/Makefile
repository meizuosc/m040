#
# Makefile for the Linux Bluetooth HIDP layer
#

obj-$(CONFIG_BT_HIDP) += hidp.o

ifeq ($(CONFIG_BT_MGMT),y)
hidp-objs := _mgmt/core.o _mgmt/sock.o
else
hidp-objs := core.o sock.o
endif
