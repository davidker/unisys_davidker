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

static void chipset_bus_create(ulong bus_no);
static void chipset_bus_destroy(ulong bus_no);

static void chipset_device_create(ulong bus_no, ulong dev_no);
static void chipset_device_destroy(ulong bus_no, ulong dev_no);
static void chipset_device_pause(ulong bus_no, ulong dev_no);
static void chipset_device_resume(ulong bus_no, ulong dev_no);

/** These functions are implemented herein, and are called by the chipset
 *  driver to notify us about specific events.
 */
static struct visorchipset_busdev_notifiers chipset_notifiers = {
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
static struct visorchipset_busdev_responders chipset_responders;

/* filled in with info about parent chipset driver when we register with it */
static struct ultra_vbus_deviceinfo chipset_driver_info;

static void __iomem *
get_virt(u64 phys_addr, u32 bytes, enum visorchipset_addresstype addr_type)
{
	if (addr_type == ADDRTYPE_LOCALTEST) {
		if (phys_addr > virt_to_phys(high_memory - 1))
				return NULL;
		return (void __iomem *)__va(phys_addr);
	}

/*walk through the "iomem_resource" tables, check the requested
channel addresses is in RESERVED or UNDEFINED/AVAILABLE or greater
than HIGH_MEMORY.  If channel addresses is TRUE with the above
mentioned scenario, then use ioremap_cache to get a valid pointer.
otherwise return NULL.
 */
	else if (addr_type == ADDRTYPE_LOCALPHYSICAL) {
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
				return NULL;
			}
			break;	/* greater then HIGH_MEMORY */
		}
		/* come out, if Memory is undefined or greater then
		 * HIGM_MEMORY
		 */
		if (phys_addr > (u64)ULONG_MAX) {
			return NULL;
		}
		pcpy = ioremap_cache((ulong)phys_addr, (ulong)bytes);
		if (pcpy == NULL) {
			return NULL;
		}
		return pcpy;
	}
	return NULL;
}

static void __iomem *
chipset_preamble(ulong bus_no, ulong dev_no, struct visorchipset_device_info *devinfo)
{
	if (!visorchipset_get_device_info(bus_no, dev_no, devinfo))
			return NULL;

	if ((uuid_le_cmp(devinfo->chan_info.channel_type_uuid,
			 spar_vnic_channel_protocol_uuid) != 0) &&
	    (uuid_le_cmp(devinfo->chan_info.channel_type_uuid,
			 spar_vhba_channel_protocol_uuid) != 0)) {
		return NULL;
	}
	return get_virt(devinfo->chan_info.channel_addr,
			devinfo->chan_info.n_channel_bytes,
			devinfo->chan_info.addr_type);
}

static void
chipset_bus_create(ulong bus_no)
{
	int rc = 0;
	u64 channeladdr = 0;
	ulong nchannelbytes = 0;
	struct visorchipset_bus_info businfo;
	struct controlvm_message msg;

	POSTCODE_LINUX_3(BUS_CREATE_ENTRY_PC, bus_no, POSTCODE_SEVERITY_INFO);
	if ((visorchipset_get_bus_info(bus_no, &businfo)) &&
	    (businfo.chan_info.channel_addr > 0) &&
	    (businfo.chan_info.n_channel_bytes > 0)) {
		channeladdr = businfo.chan_info.channel_addr;
		nchannelbytes = (ulong)businfo.chan_info.n_channel_bytes;
	}
	/* Save off message with IOVM bus info in case of crash */
	if ((uuid_le_cmp(businfo.chan_info.channel_inst_uuid,
			 spar_siovm_uuid) == 0)) {
		msg.hdr.id = CONTROLVM_BUS_CREATE;
		msg.hdr.flags.response_expected = 0;
		msg.hdr.flags.server = 0;
		msg.cmd.create_bus.bus_no = bus_no;
		msg.cmd.create_bus.dev_count = businfo.dev_no;
		msg.cmd.create_bus.channel_addr = channeladdr;
		msg.cmd.create_bus.channel_bytes = nchannelbytes;
		dump_vhba_bus = bus_no;
		visorchipset_save_message(&msg, CRASH_BUS);
	}

	if (!uislib_client_inject_add_bus(bus_no, 
					  spar_vbus_channel_protocol_uuid,
					  channeladdr, nchannelbytes)) {
		rc = -1;
	}

	if (rc >= 0) {
		POSTCODE_LINUX_3(BUS_CREATE_EXIT_PC, bus_no,
				 POSTCODE_SEVERITY_INFO);
	} else {
		POSTCODE_LINUX_3(BUS_CREATE_FAILURE_PC, bus_no,
				 POSTCODE_SEVERITY_ERR);
	}
	if (chipset_responders.bus_create)
		(*chipset_responders.bus_create) (bus_no, rc);
}

static void
chipset_bus_destroy(ulong bus_no)
{
	int rc = 0;

	if (!uislib_client_inject_del_bus(bus_no))
		rc = -1;

	if (chipset_responders.bus_destroy)
		(*chipset_responders.bus_destroy) (bus_no, rc);
}

static void
chipset_device_create(ulong bus_no, ulong dev_no)
{
	void __iomem *paddr = NULL;
	int rc = 0;
	struct visorchipset_device_info devInfo;
	struct controlvm_message msg;

	paddr = chipset_preamble(bus_no, dev_no, &devInfo);
	POSTCODE_LINUX_4(DEVICE_CREATE_ENTRY_PC, dev_no, bus_no,
			 POSTCODE_SEVERITY_INFO);

	if (!paddr) {
		rc = -1;
		goto cleanup;
	}
	if (!uuid_le_cmp(devInfo.chan_info.channel_type_uuid,
			 spar_vnic_channel_protocol_uuid)) {
		if (!uislib_client_inject_add_vnic
		    (bus_no, dev_no,
		     devInfo.chan_info.channel_addr,
		     devInfo.chan_info.n_channel_bytes,
		     devInfo.chan_info.addr_type == ADDRTYPE_LOCALTEST,
		     devInfo.dev_inst_uuid, &devInfo.chan_info.intr)) {
			rc = -2;
			goto cleanup;
		}
		goto cleanup;
	} else if (!uuid_le_cmp(devInfo.chan_info.channel_type_uuid,
				spar_vhba_channel_protocol_uuid)) {
		/* Save off message with hba info in case of crash */
		if (bus_no == dump_vhba_bus) {
			msg.hdr.id = CONTROLVM_DEVICE_CREATE;
			msg.hdr.flags.response_expected = 0;
			msg.hdr.flags.server = 0;
			msg.cmd.create_device.bus_no = bus_no;
			msg.cmd.create_device.dev_no = dev_no;
			msg.cmd.create_device.dev_inst_uuid =
				devInfo.dev_inst_uuid;
			msg.cmd.create_device.intr = devInfo.chan_info.intr;
			msg.cmd.create_device.channel_addr =
			    devInfo.chan_info.channel_addr;
			msg.cmd.create_device.channel_bytes =
			    devInfo.chan_info.n_channel_bytes;
			msg.cmd.create_device.data_type_uuid =
					spar_vhba_channel_protocol_uuid;
			visorchipset_save_message(&msg, CRASH_DEV);
		}

		if (!uislib_client_inject_add_vhba
		    (bus_no, dev_no,
		     devInfo.chan_info.channel_addr,
		     devInfo.chan_info.n_channel_bytes,
		     devInfo.chan_info.addr_type == ADDRTYPE_LOCALTEST,
		     devInfo.dev_inst_uuid, &devInfo.chan_info.intr)) {
			rc = -3;
			goto cleanup;
		}
		goto cleanup;
	}

	rc = -4;		/* unsupported GUID */
cleanup:
	if (rc >= 0) {
		POSTCODE_LINUX_4(DEVICE_CREATE_SUCCESS_PC, dev_no, bus_no,
				 POSTCODE_SEVERITY_INFO);
	} else {
		POSTCODE_LINUX_4(DEVICE_CREATE_FAILURE_PC, dev_no, bus_no,
				 POSTCODE_SEVERITY_ERR);
	}
	if (chipset_responders.device_create)
		(*chipset_responders.device_create) (bus_no, dev_no, rc);
}

static void
chipset_device_destroy(ulong bus_no, ulong dev_no)
{
	void __iomem *paddr = NULL;
	int rc = 0;
	struct visorchipset_device_info devInfo;

	paddr = chipset_preamble(bus_no, dev_no, &devInfo);
	if (!paddr) {
		rc = -1;
		goto cleanup;
	}
	if (!uuid_le_cmp(devInfo.chan_info.channel_type_uuid,
			 spar_vnic_channel_protocol_uuid)) {
		uislib_client_inject_del_vnic(bus_no, dev_no);
		goto cleanup;
	} else if (!uuid_le_cmp(devInfo.chan_info.channel_type_uuid,
				spar_vhba_channel_protocol_uuid)) {
		uislib_client_inject_del_vhba(bus_no, dev_no);
		goto cleanup;
	}
	rc = -1;		/* no match on GUID */
cleanup:
	if (chipset_responders.device_destroy)
		(*chipset_responders.device_destroy) (bus_no, dev_no, rc);
}

static void
chipset_device_pause(ulong bus_no, ulong dev_no)
{
	void __iomem *paddr = NULL;
	struct visorchipset_device_info devInfo;

	paddr = chipset_preamble(bus_no, dev_no, &devInfo);
	if (!paddr)
			return;

	if (!uuid_le_cmp(devInfo.chan_info.channel_type_uuid,
			 spar_vnic_channel_protocol_uuid)) {
		uislib_client_inject_pause_vnic(bus_no, dev_no);
	} else if (!uuid_le_cmp(devInfo.chan_info.channel_type_uuid,
				spar_vhba_channel_protocol_uuid)) {
		uislib_client_inject_pause_vhba(bus_no, dev_no);
	}
}

static void
chipset_device_resume(ulong bus_no, ulong dev_no)
{
	void __iomem *paddr = NULL;
	struct visorchipset_device_info devInfo;

	paddr = chipset_preamble(bus_no, dev_no, &devInfo);
	if (!paddr)
			goto cleanup;
	if (!uuid_le_cmp(devInfo.chan_info.channel_type_uuid,
			 spar_vnic_channel_protocol_uuid)) {
		uislib_client_inject_resume_vnic(bus_no, dev_no);
		goto cleanup;
	} else if (!uuid_le_cmp(devInfo.chan_info.channel_type_uuid,
				spar_vhba_channel_protocol_uuid)) {
		uislib_client_inject_resume_vhba(bus_no, dev_no);
		goto cleanup;
	}
cleanup:
	if (chipset_responders.device_resume)
		(*chipset_responders.device_resume)(bus_no, dev_no, 0);
}

static int __init
visorclientbus_init(void)
{
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
}

module_init(visorclientbus_init);
module_exit(visorclientbus_exit);

MODULE_AUTHOR("Unisys");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Supervisor client device bus driver for service partition: ver " VERSION);
MODULE_VERSION(VERSION);
