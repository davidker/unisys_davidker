/* visorclientbus_main.c
 *
 * Copyright (C) 2010 - 2013 UNISYS CORPORATION
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, GOOD TITLE or
 * NON INFRINGEMENT.  See the GNU General Public License for more
 * details.
 */

/*
 *  This module processes bus+device messages received for devices which we are
 *  to act as a client for.  Currently the only device for which we can act
 *  as a client is VNIC.
 */

#include "timskmod.h"
#include "visorchipset.h"
#include "uisutils.h"
#include "iochannel.h"
#include "version.h"
#include "guestlinuxdebug.h"
#include <linux/mm.h>

#define CURRENT_FILE_PC VISOR_CLIENT_BUS_PC_visorclientbus_main_c
#define MYDRVNAME "visorclientbus"

static int dump_vhba_bus = -1;

static void chipset_bus_create(ulong busNo);
static void chipset_bus_destroy(ulong busNo);

static void chipset_device_create(ulong busNo, ulong devNo);
static void chipset_device_destroy(ulong busNo, ulong devNo);
static void chipset_device_pause(ulong busNo, ulong devNo);
static void chipset_device_resume(ulong busNo, ulong devNo);

/** These functions are implemented herein, and are called by the chipset
 *  driver to notify us about specific events.
 */
static VISORCHIPSET_BUSDEV_NOTIFIERS chipset_notifiers = {
	.bus_create = chipset_bus_create,
	.bus_destroy = chipset_bus_destroy,
	.device_create = chipset_device_create,
	.device_destroy = chipset_device_destroy,
	.device_pause = chipset_device_pause,
	.device_resume = chipset_device_resume,
	.get_channel_info = NULL,
};

/** These functions are implemented in the chipset driver, and we call them
 *  herein when we want to acknowledge a specific event.
 */
static VISORCHIPSET_BUSDEV_RESPONDERS chipset_responders;

/* filled in with info about parent chipset driver when we register with it */
static struct ultra_vbus_deviceinfo chipset_driver_info;

static void __iomem *
get_virt(u64 phys_addr, u32 bytes, VISORCHIPSET_ADDRESSTYPE addrType)
{
	if (addrType == ADDRTYPE_localTest) {
		if (phys_addr > virt_to_phys(high_memory - 1)) {
			ERRDRV("%s - bad localTest address for channel (0x%-16.16Lx for %lu bytes)",
			       __func__,
			       (unsigned long long)phys_addr, (ulong)bytes);
			return NULL;
		}
		return (void __iomem *)__va(phys_addr);
	}

/*walk through the "iomem_resource" tables, check the requested
channel addresses is in RESERVED or UNDEFINED/AVAILABLE or greater
than HIGH_MEMORY.  If channel addresses is TRUE with the above
mentioned scenario, then use ioremap_cache to get a valid pointer.
otherwise return NULL.
 */
	else if (addrType == ADDRTYPE_localPhysical) {
		struct resource *tmp, **p;
		struct resource *root = NULL;
		void __iomem  *pcpy = NULL;

		root = &iomem_resource;	/*Root node, Global var */
		p = &root->child;
		for (;;) {
			tmp = *p;
			if (!tmp || tmp->start > (phys_addr + bytes - 1)) {
				/* Memory region is undefined */
				break;
			}
			p = &tmp->sibling;
			if (tmp->end < phys_addr) /*start */
				continue;

			if (phys_addr <= virt_to_phys(high_memory - 1)) {
				/*Memory is reserved and within HIGH_MEMORY */
				ERRDRV("%s - localPhysical address overlaps memory our OS is currently using! (0x%-16.16Lx for %lu bytes)",
				       __func__,
				       (unsigned long long)phys_addr,
				       (ulong)bytes);
				return NULL;
			}
			break;	/* greater then HIGH_MEMORY */
		}
		/* come out, if Memory is undefined or greater then
		 * HIGM_MEMORY
		 */
		if (phys_addr > (u64)ULONG_MAX) {
			ERRDRV("%s - localPhysical address is too large to be be mapped (0x%-16.16Lx for %lu bytes)",
			       __func__,
			       (unsigned long long)phys_addr, (ulong)bytes);
			return NULL;
		}
		pcpy = ioremap_cache((ulong)phys_addr, (ulong)bytes);
		if (pcpy == NULL) {
			ERRDRV("%s - ioremap_cache(0x%lx,%lu) failed",
			       __func__, (ulong)phys_addr, (ulong)bytes);
			return NULL;
		}
		return pcpy;
	}
	return NULL;
}

static void __iomem *
chipset_preamble(ulong busNo, ulong devNo, VISORCHIPSET_DEVICE_INFO *devinfo)
{
	if (!visorchipset_get_device_info(busNo, devNo, devinfo)) {
		ERRDRV("%s - visorchipset_get_device_info returned false",
		       __func__);
		return NULL;
	}
	if ((uuid_le_cmp(devinfo->chanInfo.channelTypeGuid,
			 spar_vnic_channel_protocol_uuid) != 0) &&
	    (uuid_le_cmp(devinfo->chanInfo.channelTypeGuid,
			 spar_vhba_channel_protocol_uuid) != 0)) {
		ERRDRV("%s - I only know how to handle VNIC or VHBA client channels",
		       __func__);
		return NULL;
	}
	return get_virt(devinfo->chanInfo.channelAddr,
			devinfo->chanInfo.nChannelBytes,
			devinfo->chanInfo.addrType);
}

static void
chipset_bus_create(ulong busNo)
{
	int rc = 0;
	u64 channeladdr = 0;
	ulong nchannelbytes = 0;
	VISORCHIPSET_BUS_INFO businfo;
	struct controlvm_message msg;

	POSTCODE_LINUX_3(BUS_CREATE_ENTRY_PC, busNo, POSTCODE_SEVERITY_INFO);
	if ((visorchipset_get_bus_info(busNo, &businfo)) &&
	    (businfo.chanInfo.channelAddr > 0) &&
	    (businfo.chanInfo.nChannelBytes > 0)) {
		channeladdr = businfo.chanInfo.channelAddr;
		nchannelbytes = (ulong)businfo.chanInfo.nChannelBytes;
	}
	/* Save off message with IOVM bus info in case of crash */
	if ((uuid_le_cmp(businfo.chanInfo.channelInstGuid,
			 spar_siovm_uuid) == 0)) {
		msg.hdr.id = CONTROLVM_BUS_CREATE;
		msg.hdr.flags.response_expected = 0;
		msg.hdr.flags.server = 0;
		msg.cmd.create_bus.bus_no = busNo;
		msg.cmd.create_bus.dev_count = businfo.devNo;
		msg.cmd.create_bus.channel_addr = channeladdr;
		msg.cmd.create_bus.channel_bytes = nchannelbytes;
		dump_vhba_bus = busNo;
		visorchipset_save_message(&msg, CRASH_bus);
	}

	if (!uislib_client_inject_add_bus(busNo, UltraVbusChannelProtocolGuid,
					  channeladdr, nchannelbytes)) {
		rc = -1;
	}

	if (rc >= 0) {
		INFODRV("%s(%lu) successful", __func__, busNo);
		POSTCODE_LINUX_3(BUS_CREATE_EXIT_PC, busNo,
				 POSTCODE_SEVERITY_INFO);
	} else {
		ERRDRV("%s(%lu) failed", __func__, busNo);
		POSTCODE_LINUX_3(BUS_CREATE_FAILURE_PC, busNo,
				 POSTCODE_SEVERITY_ERR);
	}
	if (chipset_responders.bus_create)
		(*chipset_responders.bus_create) (busNo, rc);
}

static void
chipset_bus_destroy(ulong busNo)
{
	int rc = 0;

	if (!uislib_client_inject_del_bus(busNo))
		rc = -1;

	if (rc >= 0)
		INFODRV("%s(%lu) successful", __func__, busNo);
	else
		ERRDRV("%s(%lu) failed", __func__, busNo);
	if (chipset_responders.bus_destroy)
		(*chipset_responders.bus_destroy) (busNo, rc);
}

static void
chipset_device_create(ulong busNo, ulong devNo)
{
	void __iomem *paddr = NULL;
	int rc = 0;
	VISORCHIPSET_DEVICE_INFO devInfo;
	struct controlvm_message msg;

	paddr = chipset_preamble(busNo, devNo, &devInfo);
	POSTCODE_LINUX_4(DEVICE_CREATE_ENTRY_PC, devNo, busNo,
			 POSTCODE_SEVERITY_INFO);

	if (!paddr) {
		rc = -1;
		goto away;
	}
	if (!uuid_le_cmp(devInfo.chanInfo.channelTypeGuid,
			 spar_vnic_channel_protocol_uuid)) {
		if (!uislib_client_inject_add_vnic
		    (busNo, devNo,
		     devInfo.chanInfo.channelAddr,
		     devInfo.chanInfo.nChannelBytes,
		     devInfo.chanInfo.addrType == ADDRTYPE_localTest,
		     devInfo.devInstGuid, &devInfo.chanInfo.intr)) {
			rc = -2;
			goto away;
		}
		goto away;
	} else if (!uuid_le_cmp(devInfo.chanInfo.channelTypeGuid,
				spar_vhba_channel_protocol_uuid)) {
		/* Save off message with hba info in case of crash */
		if (busNo == dump_vhba_bus) {
			msg.hdr.id = CONTROLVM_DEVICE_CREATE;
			msg.hdr.flags.response_expected = 0;
			msg.hdr.flags.server = 0;
			msg.cmd.create_device.bus_no = busNo;
			msg.cmd.create_device.dev_no = devNo;
			msg.cmd.create_device.dev_inst_uuid =
				devInfo.devInstGuid;
			msg.cmd.create_device.intr = devInfo.chanInfo.intr;
			msg.cmd.create_device.channel_addr =
			    devInfo.chanInfo.channelAddr;
			msg.cmd.create_device.channel_bytes =
			    devInfo.chanInfo.nChannelBytes;
			msg.cmd.create_device.data_type_uuid =
					spar_vhba_channel_protocol_uuid;
			visorchipset_save_message(&msg, CRASH_dev);
		}

		if (!uislib_client_inject_add_vhba
		    (busNo, devNo,
		     devInfo.chanInfo.channelAddr,
		     devInfo.chanInfo.nChannelBytes,
		     devInfo.chanInfo.addrType == ADDRTYPE_localTest,
		     devInfo.devInstGuid, &devInfo.chanInfo.intr)) {
			rc = -3;
			goto away;
		}
		goto away;
	}

	rc = -4;		/* unsupported GUID */
away:
	if (rc >= 0) {
		INFODRV("%s(%lu,%lu) successful", __func__, busNo, devNo);
		POSTCODE_LINUX_4(DEVICE_CREATE_SUCCESS_PC, devNo, busNo,
				 POSTCODE_SEVERITY_INFO);
	} else {
		ERRDRV("%s(%lu,%lu)=%d failed", __func__, busNo, devNo, rc);
		POSTCODE_LINUX_4(DEVICE_CREATE_FAILURE_PC, devNo, busNo,
				 POSTCODE_SEVERITY_ERR);
	}
	if (chipset_responders.device_create)
		(*chipset_responders.device_create) (busNo, devNo, rc);
}

static void
chipset_device_destroy(ulong busNo, ulong devNo)
{
	void __iomem *paddr = NULL;
	int rc = 0;
	VISORCHIPSET_DEVICE_INFO devInfo;

	paddr = chipset_preamble(busNo, devNo, &devInfo);
	if (!paddr) {
		rc = -1;
		goto away;
	}
	if (!uuid_le_cmp(devInfo.chanInfo.channelTypeGuid,
			 spar_vnic_channel_protocol_uuid)) {
		uislib_client_inject_del_vnic(busNo, devNo);
		goto away;
	} else if (!uuid_le_cmp(devInfo.chanInfo.channelTypeGuid,
				spar_vhba_channel_protocol_uuid)) {
		uislib_client_inject_del_vhba(busNo, devNo);
		goto away;
	}
	rc = -1;		/* no match on GUID */
away:
	if (rc >= 0)
		INFODRV("%s(%lu,%lu) successful", __func__, busNo, devNo);
	else
		ERRDRV("%s(%lu,%lu) failed", __func__, busNo, devNo);
	if (chipset_responders.device_destroy)
		(*chipset_responders.device_destroy) (busNo, devNo, rc);
}

static void
chipset_device_pause(ulong busNo, ulong devNo)
{
	void __iomem *paddr = NULL;
	int rc = 0;
	VISORCHIPSET_DEVICE_INFO devInfo;

	paddr = chipset_preamble(busNo, devNo, &devInfo);
	if (!paddr) {
		rc = -1;
		goto away;
	}
	if (!uuid_le_cmp(devInfo.chanInfo.channelTypeGuid,
			 spar_vnic_channel_protocol_uuid)) {
		rc = uislib_client_inject_pause_vnic(busNo, devNo);
		goto away;
	} else if (!uuid_le_cmp(devInfo.chanInfo.channelTypeGuid,
				spar_vhba_channel_protocol_uuid)) {
		rc = uislib_client_inject_pause_vhba(busNo, devNo);
		goto away;
	}
	rc = -1;		/* no match on GUID */
away:
	if (rc == CONTROLVM_RESP_SUCCESS)
		INFODRV("%s(%lu,%lu) successful", __func__, busNo, devNo);
	/* Response sent when the pause is completed */
	else {
		ERRDRV("%s(%lu,%lu) failed", __func__, busNo, devNo);
		if (chipset_responders.device_pause)
			(*chipset_responders.device_pause) (busNo, devNo, rc);
	}
}

static void
chipset_device_resume(ulong busNo, ulong devNo)
{
	void __iomem *paddr = NULL;
	int rc = 0;
	VISORCHIPSET_DEVICE_INFO devInfo;

	paddr = chipset_preamble(busNo, devNo, &devInfo);
	if (!paddr) {
		rc = -1;
		goto away;
	}
	if (!uuid_le_cmp(devInfo.chanInfo.channelTypeGuid,
			 spar_vnic_channel_protocol_uuid)) {
		rc = uislib_client_inject_resume_vnic(busNo, devNo);
		goto away;
	} else if (!uuid_le_cmp(devInfo.chanInfo.channelTypeGuid,
				spar_vhba_channel_protocol_uuid)) {
		rc = uislib_client_inject_resume_vhba(busNo, devNo);
		goto away;
	}
	rc = -1;		/* no match on GUID */
away:
	if (rc == CONTROLVM_RESP_SUCCESS)
		INFODRV("%s(%lu,%lu) successful", __func__, busNo, devNo);
	else
		ERRDRV("%s(%lu,%lu) failed", __func__, busNo, devNo);
	if (chipset_responders.device_resume)
		(*chipset_responders.device_resume) (busNo, devNo, rc);
}

static int __init
visorclientbus_init(void)
{
	INFODRV("client bus driver version %s loaded", VERSION);
	POSTCODE_LINUX_2(CHIPSET_INIT_ENTRY_PC, POSTCODE_SEVERITY_INFO);
	/* This enables us to receive notifications when devices appear for
	 * which this service partition is to be a client for.
	 */
	visorchipset_register_busdev_client(&chipset_notifiers,
					    &chipset_responders,
					    &chipset_driver_info);

	POSTCODE_LINUX_2(CHIPSET_INIT_EXIT_PC, POSTCODE_SEVERITY_INFO);

	return 0;
}

static void
visorclientbus_exit(void)
{
	visorchipset_register_busdev_client(NULL, NULL, NULL);
	INFODRV("client bus driver unloaded");
}

module_init(visorclientbus_init);
module_exit(visorclientbus_exit);

MODULE_AUTHOR("Unisys");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Supervisor client device bus driver for service partition: ver " VERSION);
MODULE_VERSION(VERSION);
