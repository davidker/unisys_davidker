#
# Makefile for Unisys visorconinclient
#

obj-$(CONFIG_UNISYS_VISORCONINCLIENT)	+= visorconinclient.o

visorconinclient-y := visorconinclient_main.o

ccflags-y += -Idrivers/staging/unisys/include
ccflags-y += -Idrivers/staging/unisys/visorutil
ccflags-y += -Idrivers/staging/unisys/visorbus
ccflags-y += -Idrivers/staging/unisys/visorchannel
ccflags-y += -Idrivers/staging/unisys/common-spar/include
ccflags-y += -Idrivers/staging/unisys/common-spar/include/channels

ccflags-y += -DCONFIG_SPAR_GUEST -DGUESTDRIVERBUILD -DNOAUTOVERSION

