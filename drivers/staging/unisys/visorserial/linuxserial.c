/* linuxserial.c
 *
 * Copyright © 2010 - 2013 UNISYS CORPORATION
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

#include "linuxserial.h"
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/module.h>
#include "visorserial_private.h"
#include "periodic_work.h"

#define SERIAL_STATE state
#define SERIAL_TTY port.tty

#define TTY_MAJOR_DEVNO       240  /* major number of the tty device that
				    * will get created */
#define TTY_MINOR_DEVNO_START   0  /* first tty device minor number */
#define UART_NR                 1  /* max number of devices */
#define DELAY_TIME         (HZ/10) /* check for xmit chars 10 times per sec */

struct LINUXSERIAL_Tag {
	int devno;
	struct uart_port port;
	void (*transmit_char) (void *, u8);
	void *context;
	PERIODIC_WORK *periodic_work;
};

static struct workqueue_struct *Workqueue;
static BOOL Driver_Registered = FALSE;
static int Registered_Ports;

/* Here are some "no-ops" */
static void
lxser_stop_tx(struct uart_port *port)
{
}

static void
lxser_start_tx(struct uart_port *port)
{
}

static void
lxser_set_termios(struct uart_port *port,
		  struct ktermios *new, struct ktermios *old)
{
}

static void
lxser_stop_rx(struct uart_port *port)
{
}

static void
lxser_enable_ms(struct uart_port *port)
{
}

static void
lxser_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

static void
lxser_break_ctl(struct uart_port *port, int break_state)
{
}

static void
lxser_release_port(struct uart_port *port)
{
}

static void
lxser_config_port(struct uart_port *port, int flags)
{
	/* Here's something stupid... you MUST set this non-0, or nothing
	 * will work.  It doesn't matter which type you set, just do it.
	 */
	port->type = PORT_MUX;
}

/*  This function must be called at periodic intervals.
 *  It simply checks the tty's output queue to see if data has been queued
 *  there (it would have been placed there by the kernel, in response to
 *  someone writing to the tty device), and if so, calls transmit_char()
 *  to logically transmit onto the "wire".
 */
static void
lxser_tx_chars(struct uart_port *port, void *context,
	       void (*transmit_char) (void *, u8))
{
	struct circ_buf *xmit = NULL;
	struct tty_struct *tty;
	int count;
	u8 c;

	if (!port)
		return;
	if (!port->SERIAL_STATE)
		return;
	xmit = &port->SERIAL_STATE->xmit;
	tty = port->SERIAL_STATE->SERIAL_TTY;
	if (port->x_char) {
		c = port->x_char;
		(*transmit_char) (context, c);
		port->icount.tx++;
		port->x_char = 0;
		return;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		lxser_stop_tx(port);
		return;
	}
	count = port->fifosize >> 1;
	do {
		c = xmit->buf[xmit->tail];
		(*transmit_char) (context, c);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);
	if (uart_circ_empty(xmit))
		lxser_stop_tx(port);
}

/*  This function is called by the low-level serial code to indicate that a
 *  character has come in on the "wire".
 */
void
linuxserial_rx_char(LINUXSERIAL *ls, u8 c)
{
	struct uart_port *port;
	struct tty_struct *tty;

	port = &ls->port;
	if (!port)
		return;
	if (!port->SERIAL_STATE)
		return;
	tty = port->SERIAL_STATE->SERIAL_TTY;
	if (!tty)
		return;

	/* add one character to the tty port this doesn't actually
	 * push the data through unless tty->low_latency is set
	 */
	tty_insert_flip_char(tty->port, c, TTY_NORMAL);
	tty_schedule_flip(tty->port);
}

static unsigned int
lxser_tx_empty(struct uart_port *port)
{
	/* yeah, yeah, yeah, our "transmitter" is empty... */
	return TIOCSER_TEMT;
}

static unsigned int
lxser_get_mctrl(struct uart_port *port)
{
	/* All "lines" are permanently ready. */
	return TIOCM_CAR | TIOCM_DSR | TIOCM_CTS;
}

static void
lxser_periodic_work(void *xls)
{
	LINUXSERIAL *ls = (LINUXSERIAL *) (xls);

	lxser_tx_chars(&ls->port, ls->context, ls->transmit_char);
	visor_periodic_work_nextperiod(ls->periodic_work);
}

static int
lxser_startup(struct uart_port *port)
{
	/* this is the first time this port is opened do any hardware
	 * initialization needed here
	 */
	LINUXSERIAL *ls = (__force LINUXSERIAL *) (port->membase);

	INFODRV("%s", __func__);
	ls->periodic_work = visor_periodic_work_create(DELAY_TIME,
						       Workqueue,
						       lxser_periodic_work,
						       ls, "visortty");
	if (ls->periodic_work == NULL) {
		ERRDEVX(ls->devno, "failed to create periodic_work");
		return -ENOMEM;
	}
	visor_periodic_work_start(ls->periodic_work);
	INFODRV("port started");
	return 0;
}

static void
lxser_shutdown(struct uart_port *port)
{
	/* The port is being closed by the last user.  Do any hardware
	* specific stuff here */
	LINUXSERIAL *ls = (__force LINUXSERIAL *) (port->membase);

	INFODRV("%s", __func__);
	if (ls->periodic_work != NULL) {
		visor_periodic_work_stop(ls->periodic_work);
		visor_periodic_work_destroy(ls->periodic_work);
		ls->periodic_work = NULL;
	}
}

static const char *
lxser_type(struct uart_port *port)
{
	return "visorserial";
}

static int
lxser_request_port(struct uart_port *port)
{
	return 0;
}

static int
lxser_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	return 0;
}

static struct uart_ops lxser_ops = {
	.tx_empty = lxser_tx_empty,
	.set_mctrl = lxser_set_mctrl,
	.get_mctrl = lxser_get_mctrl,
	.stop_tx = lxser_stop_tx,
	.start_tx = lxser_start_tx,
	.stop_rx = lxser_stop_rx,
	.enable_ms = lxser_enable_ms,
	.break_ctl = lxser_break_ctl,
	.startup = lxser_startup,
	.shutdown = lxser_shutdown,
	.type = lxser_type,
	.release_port = lxser_release_port,
	.request_port = lxser_request_port,
	.config_port = lxser_config_port,
	.verify_port = lxser_verify_port,
	.set_termios = lxser_set_termios,
};

struct uart_driver visorserial_lxser_reg = {
	.owner = THIS_MODULE,
	.driver_name = MYDRVNAME,
	.dev_name = MYDRVNAME,
	.major = TTY_MAJOR_DEVNO,
	.minor = TTY_MINOR_DEVNO_START,
	.nr = UART_NR,
};

LINUXSERIAL *
linuxserial_create(int devno, void *context, void (*transmit_char) (void *, u8))
{
	int result;
	LINUXSERIAL *rc = NULL;
	LINUXSERIAL *ls = NULL;

	INFODRV("%s", __func__);
	if (devno >= UART_NR) {
		ERRDEVX(devno, "tty device NOT created (max tty devices=%d)",
			UART_NR);
		rc = NULL;
		goto Away;
	}
	ls = kmalloc(sizeof(LINUXSERIAL), GFP_KERNEL|__GFP_NORETRY);
	if (ls == NULL) {
		ERRDEVX(devno, "%s allocation failed ", __func__);
		rc = NULL;
		goto Away;
	}
	memset(ls, '\0', sizeof(LINUXSERIAL));
	ls->devno = devno;
	ls->context = context;
	ls->transmit_char = transmit_char;
	ls->port.ops = &lxser_ops;
	ls->port.line = devno;
	ls->port.fifosize = 255;
	ls->port.flags = UPF_BOOT_AUTOCONF;

	/* Strange... we just need to set these non-0 to coerce the serial
	 * core to work right.  Obviously, nobody needs them for anything
	 * useful, since they are all LIES...
	 */
	ls->port.mapbase = devno + 1;
	ls->port.membase = (void __iomem *) ls;	/* use for context */
	ls->port.uartclk = (921600 * 16);
	ls->port.iotype = SERIAL_IO_MEM;

	if (!Driver_Registered) {
		result = uart_register_driver(&visorserial_lxser_reg);
		if (result) {
			ERRDEVX(devno, "uart_register_driver failed");
			rc = NULL;
			goto Away;
		}
		Workqueue = create_singlethread_workqueue("visortty");
		if (Workqueue == NULL) {
			ERRDEVX(devno, "cannot create workqueue");
			uart_unregister_driver(&visorserial_lxser_reg);
			rc = NULL;
			goto Away;
		}
		INFODRV("tty driver registered");
		Driver_Registered = TRUE;
	}

	result = uart_add_one_port(&visorserial_lxser_reg, &ls->port);
	if (result) {
		ERRDEVX(devno, "uart_add_one_port failed");
		uart_unregister_driver(&visorserial_lxser_reg);
		rc = NULL;
		goto Away;
	}
	INFODEVX(devno, "tty port added");
	Registered_Ports++;

	rc = ls;
Away:
	if (rc == NULL) {
		if (ls != NULL) {
			kfree(ls);
			ls = NULL;
		}
	}
	return rc;
}

void
linuxserial_destroy(LINUXSERIAL *ls)
{
	INFODEVX(ls->devno, "%s", __func__);
	if (ls == NULL)
		return;
	uart_remove_one_port(&visorserial_lxser_reg, &ls->port);
	Registered_Ports--;
	if (Registered_Ports <= 0)
		uart_unregister_driver(&visorserial_lxser_reg);
	if (Workqueue != NULL) {
		flush_workqueue(Workqueue);
		destroy_workqueue(Workqueue);
		Workqueue = NULL;
	}
	kfree(ls);
}
