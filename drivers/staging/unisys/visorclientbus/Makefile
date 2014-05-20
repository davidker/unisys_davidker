#
# Makefile for Unisys visorclientbus
#

obj-$(CONFIG_UNISYS_VISORCLIENTBUS)	+= visorclientbus.o

visorclientbus-y := visorclientbus_main.o

ccflags-y += -Idrivers/staging/unisys/include
ccflags-y += -Idrivers/staging/unisys/uislib
ccflags-y += -Idrivers/staging/unisys/visorchipset
ccflags-y += -Idrivers/staging/unisys/common-spar/include
ccflags-y += -Idrivers/staging/unisys/common-spar/include/channels

ccflags-y += -DCONFIG_SPAR_GUEST -DGUESTDRIVERBUILD -DNOAUTOVERSION

