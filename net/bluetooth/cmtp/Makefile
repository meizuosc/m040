#
# Makefile for the Linux Bluetooth CMTP layer
#

obj-$(CONFIG_BT_CMTP) += cmtp.o

ifeq ($(CONFIG_BT_MGMT),y)
cmtp-objs := _mgmt/core.o _mgmt/sock.o _mgmt/capi.o
else
cmtp-objs := core.o sock.o capi.o
endif
