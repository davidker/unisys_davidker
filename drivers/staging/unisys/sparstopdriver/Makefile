#
# Makefile for Unisys channel
#

obj-$(CONFIG_UNISYS_SPARSTOP)	+= sparstop.o

sparstop-y := sparstop_main.o

ccflags-y += -Idrivers/staging/unisys/include
ccflags-y += -Idrivers/staging/unisys/visorutil
ccflags-y += -Idrivers/staging/unisys/common-spar/include
ccflags-y += -Idrivers/staging/unisys/common-spar/include/channels

ccflags-y += -DCONFIG_SPAR_GUEST -DGUESTDRIVERBUILD -DNOAUTOVERSION

