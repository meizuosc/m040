# Makefile of modem_if

EXTRA_CFLAGS += -Idrivers/modem/xmm6x60

obj-$(CONFIG_MEIZU_MODEM_WD) += meizu_modem.o meizu_modem_io_device.o
obj-$(CONFIG_MEIZU_MODEM_WD) += meizu_modem_utils.o

obj-$(CONFIG_UMTS_MODEM_XMM6260) += meizu_modemctl_xmm6260.o
obj-$(CONFIG_UMTS_MODEM_XMM6262) += meizu_modemctl_xmm6262.o
obj-$(CONFIG_LINK_DEVICE_HSIC) += meizu_modem_hsic.o
