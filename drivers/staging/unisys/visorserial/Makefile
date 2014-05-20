#
# Makefile for Unisys visorserial
#

obj-$(CONFIG_UNISYS_VISORSERIAL)	+= visorserial.o

visorserial-y := visorserial_main.o linuxconsole.o linuxserial.o

ccflags-y += -Idrivers/staging/unisys/include
ccflags-y += -Idrivers/staging/unisys/uislib
ccflags-y += -Idrivers/staging/unisys/visorutil
ccflags-y += -Idrivers/staging/unisys/visorbus
ccflags-y += -Idrivers/staging/unisys/visorchannel
ccflags-y += -Idrivers/staging/unisys/common-spar/include
ccflags-y += -Idrivers/staging/unisys/common-spar/include/channels
ccflags-y += -DCONFIG_SPAR_GUEST -DGUESTDRIVERBUILD -DNOAUTOVERSION
ccflags-y += -DSTANDALONE_CLIENT

