#
# Makefile for the Linux Bluetooth subsystem.
#

obj-$(CONFIG_BT)	+= bluetooth.o
obj-$(CONFIG_BT_RFCOMM)	+= rfcomm/
obj-$(CONFIG_BT_BNEP)	+= bnep/
obj-$(CONFIG_BT_CMTP)	+= cmtp/
obj-$(CONFIG_BT_HIDP)	+= hidp/

ifeq ($(CONFIG_BT_MGMT),y)
bluetooth-y := _mgmt/af_bluetooth.o _mgmt/hci_core.o _mgmt/hci_conn.o _mgmt/hci_event.o _mgmt/mgmt.o _mgmt/hci_sock.o _mgmt/hci_sysfs.o _mgmt/lib.o
bluetooth-$(CONFIG_BT_L2CAP)	+= _mgmt/l2cap_core.o _mgmt/l2cap_sock.o _mgmt/smp.o
bluetooth-$(CONFIG_BT_SCO)	+= _mgmt/sco.o
else
bluetooth-y := af_bluetooth.o hci_core.o hci_conn.o hci_event.o mgmt.o hci_sock.o hci_sysfs.o lib.o
bluetooth-$(CONFIG_BT_L2CAP)	+= l2cap_core.o l2cap_sock.o smp.o
bluetooth-$(CONFIG_BT_SCO)	+= sco.o
endif
