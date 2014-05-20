#
# Makefile for Unisys virtnic
#

obj-$(CONFIG_UNISYS_VIRTNIC)	+= virtnic.o

ccflags-y += -Idrivers/staging/unisys/include
ccflags-y += -Idrivers/staging/unisys/uislib
ccflags-y += -Idrivers/staging/unisys/visorchipset
ccflags-y += -Idrivers/staging/unisys/virtpci
ccflags-y += -Idrivers/staging/unisys/common-spar/include
ccflags-y += -Idrivers/staging/unisys/common-spar/include/channels

ccflags-y += -DCONFIG_SPAR_GUEST -DGUESTDRIVERBUILD -DNOAUTOVERSION

