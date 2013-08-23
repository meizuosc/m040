#
# Makefile for the Linux Bluetooth RFCOMM layer.
#

obj-$(CONFIG_BT_RFCOMM) += rfcomm.o

ifeq ($(CONFIG_BT_MGMT),y)
rfcomm-y			:= _mgmt/core.o _mgmt/sock.o
rfcomm-$(CONFIG_BT_RFCOMM_TTY)	+= _mgmt/tty.o
else
rfcomm-y			:= core.o sock.o
rfcomm-$(CONFIG_BT_RFCOMM_TTY)	+= tty.o
endif
