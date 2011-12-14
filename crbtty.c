
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/ioctl.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>

#include "mcedev.h"
#include "mcedev_common.h"


//////////////////
// TTY ring buffer

struct crb_tty_buf {
	char* buffer;
	int len, wp, rp;
};

#define crbtty_buf_full(buf)			((((buf)->wp + 1) % (buf)->len) == (buf)->rp)
#define crbtty_buf_empty(buf)			((buf)->wp == (buf)->rp)

static void crbtty_buf_init(struct crb_tty_buf* buf, char* base, int len)
{
	buf->buffer = base;
	buf->len = len;
	buf->wp = buf->rp = 0;
}

static void crbtty_buf_clear(struct crb_tty_buf* buf)
{
	buf->wp = buf->rp = 0;
}

static int crbtty_buf_write_chunk(struct crb_tty_buf* buf, const char* data, int len, int from_user)
{
	int chunk = (buf->rp <= buf->wp) ? buf->len - buf->wp : buf->rp - 1 - buf->wp;
	if (chunk <= 0) {
		return 0;
	} else if (chunk > len) {
		chunk = len;
	}

	if (from_user) {
		if (copy_from_user(buf->buffer + buf->wp, data, chunk)) {
			return -1;
		}
	} else {
		memcpy(buf->buffer + buf->wp, data, chunk);
	}
	buf->wp = (buf->wp + chunk) % buf->len;

	return chunk;
}

static int crbtty_buf_read_chunk(struct crb_tty_buf* buf, char* data, int len, int to_user)
{
	int chunk = (buf->rp <= buf->wp) ? buf->wp - buf->rp : buf->len - buf->rp;
	if (chunk <= 0) {
		return 0;
	} else if (chunk > len) {
		chunk = len;
	}

	if (to_user) {
		if (copy_to_user(data, buf->buffer + buf->rp, chunk)) {
			return -1;
		}
	} else {
		memcpy(data, buf->buffer + buf->rp, chunk);
	}
	buf->rp = (buf->rp + chunk) % buf->len;

	return chunk;
}

static int crbtty_buf_write(struct crb_tty_buf* buf, const char* data, int len, int from_user)
{
	int chunk;

	if (len <= 0 || crbtty_buf_full(buf)) {
		return 0;
	}

	chunk = crbtty_buf_write_chunk(buf, data, len, from_user);
	if (chunk < 0) {
		return -1;
	}

	data += chunk;
	len -= chunk;

	if (len > 0) {
		int chunk2 = crbtty_buf_write_chunk(buf, data, len, from_user);
		if (chunk2 < 0) {
			return chunk;
		}
		chunk += chunk2;
	}

	return chunk;
}

static int crbtty_buf_read(struct crb_tty_buf* buf, char* data, int len, int to_user)
{
	int chunk;

	if (len <= 0 || crbtty_buf_empty(buf)) {
		return 0;
	}

	chunk = crbtty_buf_read_chunk(buf, data, len, to_user);
	if (chunk < 0) {
		return -1;
	}

	data += chunk;
	len -= chunk;

	if (len > 0) {
		int chunk2 = crbtty_buf_read_chunk(buf, data, len, to_user);
		if (chunk2 < 0) {
			return chunk;
		}
		chunk += chunk2;
	}

	return chunk;
}


/////////////////////////
// TTY driver definitions

struct crb_tty {
	// Wait queues for blocking read and write calls
	wait_queue_head_t readq, writeq;

	// FIFO buffers
	struct crb_tty_buf rbuf, wbuf;

	// Control flags
	int flags;

	// Device information
	dev_t devt;
	struct class* class;
	struct device* device;
	struct cdev cdev;

	// Lock
	struct semaphore sem;

	// Shadow registers
	u_int16_t divisor_latch;
	unsigned char scratch_register;

	// Modem control structure for host
	struct ktermios ktermios;
};

// struct cdev valid
#define CRBTTY_FLG_CDEV_ADDED				0x00000001

// Device has been opened
#define CRBTTY_FLG_OPEN						0x00000002

// Send more than 1 stop bit
#define CRBTTY_FLG_MORE_STOP_BITS			0x00000008

// Access DLAB
#define CRBTTY_FLG_DLAB						0x00000010

// Send break
#define CRBTTY_FLG_SET_BREAK				0x00000020

// Parity
#define CRBTTY_FLG_STICK_PARITY				0x00000040
#define CRBTTY_FLG_EVEN_PARITY				0x0000080
#define CRBTTY_FLG_ENABLE_PARITY			0x00000100

// Word Length Flags
#define CRBTTY_FLG_WORD_LEN_0				0x00000200
#define CRBTTY_FLG_WORD_LEN_1				0x00000400

// Loopback mode
#define CRBTTY_FLG_LOOPBACK					0x00000800

// Status of internal lines.
#define CRBTTY_FLG_OUT2						0x00001000
#define CRBTTY_FLG_OUT1						0x00002000
#define CRBTTY_FLG_DTR						0x00004000
#define CRBTTY_FLG_RTS						0x00008000
#define CRBTTY_FLG_EX_LOOPBACK_ALL				0x0000F000

// Status of external modem lines. Chosen to reflect the flags for internal lines, according to the specification of the loopback mode.
#define CRBTTY_FLG_EX_DCD					0x00010000
#define CRBTTY_FLG_EX_RI					0x00020000
#define CRBTTY_FLG_EX_DSR					0x00040000
#define CRBTTY_FLG_EX_CTS					0x00080000
#define CRBTTY_FLG_EX_REAL_ALL					0x000F0000

// Deltas of external modem lines. Chosen to reflect those flags.
#define CRBTTY_FLG_EX_DDCD					0x00100000
#define CRBTTY_FLG_EX_TERI					0x00200000
#define CRBTTY_FLG_EX_DDSR					0x00400000
#define CRBTTY_FLG_EX_DCTS					0x00800000

#define CRBTTY_FLG_IER_RDA					0x01000000
#define CRBTTY_FLG_IER_THRE					0x02000000
#define CRBTTY_FLG_IER_LSRC					0x04000000
#define CRBTTY_FLG_IER_MSRC					0x08000000

#define CRBTTY_FLG_IRQ_RDA					0x10000000
#define CRBTTY_FLG_IRQ_THRE					0x20000000
#define CRBTTY_FLG_IRQ_LSRC					0x40000000
#define CRBTTY_FLG_IRQ_MSRC					0x80000000
#define CRBTTY_FLG_IRQ_ALL					0xF0000000

// Locking on the TTY structure
#define crbtty_acquire(tty)					down(&(tty)->sem)
#define crbtty_acquire_interruptible(tty)	down_interruptible(&(tty)->sem)
#define crbtty_release(tty)					up(&(tty)->sem)


static void crb_tty_set_interrupt(struct crb_tty* tty, int flag)
/*
	Enable interrupt signal.
*/
{
	tty->flags |= flag;

	// TODO: provide a means to signal the core
}

static void crb_tty_clear_interrupt(struct crb_tty* tty, int flag)
/*
	Reset interrupt signal.
*/
{
	tty->flags &= ~flag;

	// TODO: provide a means to signal the core
}

static void crb_tty_set_internal_line(struct crb_tty* tty, int flag, int value)
/*
	Set new status of an internally-controlled line: RTS, DTR, OUT1, OUT2.
*/
{
	int old_flags = tty->flags;

	if (tty->flags & CRBTTY_FLG_LOOPBACK) {
		// Change flag for internal line and set _CHANGED flags if necessary
		if (value) {
			if (!(tty->flags & flag)) {
				tty->flags |= flag | (flag << 8);
			}
		} else {
			if (tty->flags & flag) {
				tty->flags &= ~flag;
				tty->flags |= (flag << 8);
			}
		}

		// If the MSR value has changed, send an MSRC interrupt
		if (((old_flags ^ tty->flags) & CRBTTY_FLG_EX_LOOPBACK_ALL) != 0) {
			crb_tty_set_interrupt(tty, CRBTTY_FLG_IRQ_MSRC);
		}
	} else {
		// Simply change flag for internal line.
		if (value) {
			tty->flags |= flag;
		} else {
			tty->flags &= ~flag;
		}
	}
}

static void crb_tty_set_external_line(struct crb_tty* tty, int flag, int value)
/*
	Set new status of an externally-controlled line: CTS, DSR, RI, DCD
*/
{
	int old_flags = tty->flags;

	if (tty->flags & CRBTTY_FLG_LOOPBACK) {
		// Simply change flag for external line, but ignore _CHANGED flags
		if (value) {
			tty->flags |= flag;
		} else {
			tty->flags &= ~flag;
		}
	} else {
		// Change flag for external line and set _CHANGED flags if necessary
		if (value) {
			tty->flags = tty->flags | flag    | (((tty->flags & flag) ^ flag) << 4);
		} else {
			tty->flags = (tty->flags & ~flag) | (((tty->flags & flag) ^ flag) << 4);
		}

		// If the MSR value has changed, send an MSRC interrupt
		if (((old_flags ^ tty->flags) & CRBTTY_FLG_EX_REAL_ALL) != 0) {
			crb_tty_set_interrupt(tty, CRBTTY_FLG_IRQ_MSRC);
		}
	}

}


///////////////////////
// TTY target interface

const char* crbtty_get_register_name(int offset, int is_read)
{
	switch (offset) {
	default:
		return "???";
	case 0:
		return is_read ? "RBR/DLL" : "THR/DLL";
	case 1:
		return "IER/DLM";
	case 2:
		return is_read ? "IIR" : "FCR";
	case 3:
		return "LCR";
	case 4:
		return "MCR";
	case 5:
		return is_read ? "LSR" : "???";
	case 6:
		return is_read ? "MSR" : "???";
	case 7:
		return "SCR";
	}
}

unsigned char crbtty_target_read_byte(struct crb_tty* tty, int offset)
/*
	Read byte from register of virtual UART device.
*/
{
	unsigned char value;
	int wake_writer = 0;

	crbtty_acquire(tty);

	switch (offset) {
	case 0:
		if (tty->flags & CRBTTY_FLG_DLAB) {
			// DLL: Divisor Latch Low Byte
			value = (unsigned char)(tty->divisor_latch & 0x00FF);
		} else {
			// RBR: Receive Buffer Register
			
			if (crbtty_buf_read(&tty->wbuf, &value, 1, 0) > 0) {
				wake_writer = 1;
			}

			// Reading the RBR clears the RDA interrupt.
			crb_tty_clear_interrupt(tty, CRBTTY_FLG_IRQ_RDA);
		}
		break;
	case 1:
		if (tty->flags & CRBTTY_FLG_DLAB) {
			// DLH: Divisor Latch High Byte
			value = (unsigned char)((tty->divisor_latch >> 8) & 0xFF);
		} else {
			// IER: Interrupt Enable Register
			value =
			((tty->flags & CRBTTY_FLG_IER_MSRC)		? 0x08 : 0x00) |
				((tty->flags & CRBTTY_FLG_IER_LSRC)		? 0x04 : 0x00) |
				((tty->flags & CRBTTY_FLG_IER_THRE)			? 0x02 : 0x00) |
				((tty->flags & CRBTTY_FLG_IER_RDA)			? 0x01 : 0x00)
			;
		}
		break;
	case 2:
		// IIR: Interrupt Identification Register.
		value = 0xC0 | ((tty->flags & CRBTTY_FLG_IRQ_ALL) ? 0x00 : 0x01);
		if ((tty->flags & CRBTTY_FLG_IRQ_LSRC) && (tty->flags & CRBTTY_FLG_IER_LSRC)) {		// Receiver Error.
			value |= 0x06;
		} else if ((tty->flags & CRBTTY_FLG_IRQ_RDA) && (tty->flags & CRBTTY_FLG_IER_RDA)) {	// Received Data
			value |= 0x04;
		} else if ((tty->flags & CRBTTY_FLG_IRQ_THRE) && (tty->flags & CRBTTY_FLG_IER_THRE)) {	// THR Empty
			value |= 0x02;
		} else if ((tty->flags & CRBTTY_FLG_IRQ_MSRC) && (tty->flags & CRBTTY_FLG_IER_MSRC)) {	// Modem Status Change
			value |= 0x00;
		}

		// Reading the IIR is expected to clear the THRE interrupt,
		// but it seems to stall the transmitter logic in 8250.c.
		// CHECKME: crb_tty_clear_interrupt(tty, CRBTTY_FLG_IRQ_THRE);
		break;
	case 3:
		// LCR: Line Control Register
		value =
			((tty->flags & CRBTTY_FLG_DLAB)					? 0x80 : 0x00) |
			((tty->flags & CRBTTY_FLG_SET_BREAK)			? 0x40 : 0x00) |
			((tty->flags & CRBTTY_FLG_STICK_PARITY)			? 0x20 : 0x00) |
			((tty->flags & CRBTTY_FLG_EVEN_PARITY)			? 0x10 : 0x00) |
			((tty->flags & CRBTTY_FLG_ENABLE_PARITY)		? 0x08 : 0x00) |
			((tty->flags & CRBTTY_FLG_MORE_STOP_BITS)		? 0x04 : 0x00) |
			((tty->flags & CRBTTY_FLG_WORD_LEN_1)			? 0x02 : 0x00) |
			((tty->flags & CRBTTY_FLG_WORD_LEN_0)			? 0x01 : 0x00)
			;
		break;
	case 4:
		// MCR: Modem Control Register
		value =
			((tty->flags & CRBTTY_FLG_LOOPBACK)				? 0x10 : 0x00) |
			((tty->flags & CRBTTY_FLG_OUT2)					? 0x08 : 0x00) |
			((tty->flags & CRBTTY_FLG_OUT1)					? 0x04 : 0x00) |
			((tty->flags & CRBTTY_FLG_RTS)					? 0x02 : 0x00) |
			((tty->flags & CRBTTY_FLG_DTR)					? 0x01 : 0x00);
		break;
	case 5:
		// LSR: Line Status Register
		value = 0x60 | (!crbtty_buf_empty(&tty->wbuf) ? 0x01 : 0x00);  // No error, THR empty, line idle, no break, no FE, no PE, no OE

		// Reading the LSR clears the LSRC interrupt.
		crb_tty_clear_interrupt(tty, CRBTTY_FLG_IRQ_LSRC);
		break;
	case 6:
		// MSR: Modem Status Register
		if (tty->flags & CRBTTY_FLG_LOOPBACK) {
			value =
				((tty->flags & (CRBTTY_FLG_EX_DCD >> 4))	? 0x80 : 0x00) |
				((tty->flags & (CRBTTY_FLG_EX_RI  >> 4))	? 0x40 : 0x00) |
				((tty->flags & (CRBTTY_FLG_EX_DSR >> 4))	? 0x20 : 0x00) |
				((tty->flags & (CRBTTY_FLG_EX_CTS >> 4))	? 0x10 : 0x00);
		} else {
			value =
				((tty->flags & CRBTTY_FLG_EX_DCD)			? 0x80 : 0x00) |
				((tty->flags & CRBTTY_FLG_EX_RI)			? 0x40 : 0x00) |
				((tty->flags & CRBTTY_FLG_EX_DSR)			? 0x20 : 0x00) |
				((tty->flags & CRBTTY_FLG_EX_CTS)			? 0x10 : 0x00);
		}

		// Read and clear delta flags.
		value |=
			((tty->flags & CRBTTY_FLG_EX_DDCD)				? 0x08 : 0x00) |
			((tty->flags & CRBTTY_FLG_EX_TERI)				? 0x04 : 0x00) |
			((tty->flags & CRBTTY_FLG_EX_DDSR)				? 0x02 : 0x00) |
			((tty->flags & CRBTTY_FLG_EX_DCTS)				? 0x01 : 0x00);
		tty->flags &= ~(CRBTTY_FLG_EX_DDCD | CRBTTY_FLG_EX_TERI | CRBTTY_FLG_EX_DDSR | CRBTTY_FLG_EX_DCTS);

		// Reading the MSR clears the MSRC interrupt
		crb_tty_clear_interrupt(tty, CRBTTY_FLG_IRQ_MSRC);
		break;
	case 7:
		// SCR: Scratch Register
		value = tty->scratch_register;
		break;
	default:
		// Invalid register access. Ignored.
		value = 0;
		break;
	}

	crbtty_release(tty);

	if (wake_writer) {
		wake_up_interruptible(&tty->writeq);
	}

	return value;
}

void crbtty_target_write_byte(struct crb_tty* tty, int offset, unsigned char value)
/*
	Write byte to register of virtual UART device.
*/
{
	int wake_reader = 0;

	crbtty_acquire(tty);

	switch (offset) {
	case 0:
		if (tty->flags & CRBTTY_FLG_DLAB) {
			// DLL: Divisor Latch Low Byte
			tty->divisor_latch = (tty->divisor_latch & 0xFF00) | (value & 0xFF);
		} else {
			// THR: Transmit Hold Register

			// If we are in loopback mode, transfer data to input FIFO
			if (tty->flags & CRBTTY_FLG_LOOPBACK) {
				crbtty_buf_write(&tty->wbuf, &value, 1, 0);

				// New data is available, so we send the RDA interrupt
				crb_tty_set_interrupt(tty, CRBTTY_FLG_IRQ_RDA);
			} else {
				// Transfer data directly into host's read buffer
				if (tty->flags & CRBTTY_FLG_OPEN) {
					if (crbtty_buf_write(&tty->rbuf, &value, 1, 0) > 0) {
						wake_reader = 1;
					} else {
						printk("CRBTTY: buffer full.\n");
					}
				} else {
					// Device not open, so this data cannot be received
				}
			}

			// Writing the THR clears the THRE interrupt.
			crb_tty_clear_interrupt(tty, CRBTTY_FLG_IRQ_THRE);

			// ... but as we do not implement the PHY logic, the interrupt is re-asserted immediately :)
			crb_tty_set_interrupt(tty, CRBTTY_FLG_IRQ_THRE);
		}
		break;
	case 1:
		if (tty->flags & CRBTTY_FLG_DLAB) {
			// DLH: Divisor Latch High Byte
			tty->divisor_latch = (tty->divisor_latch & 0x00FF) | ((value & 0xFF) << 8);
		} else {
			// IER: Interrupt Enable Register
			tty->flags = (tty->flags & ~CRBTTY_FLG_IER_MSRC)			| ((value & 0x08) ? CRBTTY_FLG_IER_MSRC				: 0);
			tty->flags = (tty->flags & ~CRBTTY_FLG_IER_LSRC)			| ((value & 0x04) ? CRBTTY_FLG_IER_LSRC				: 0);
			tty->flags = (tty->flags & ~CRBTTY_FLG_IER_THRE)			| ((value & 0x02) ? CRBTTY_FLG_IER_THRE				: 0);
			tty->flags = (tty->flags & ~CRBTTY_FLG_IER_RDA)			| ((value & 0x01) ? CRBTTY_FLG_IER_RDA				: 0);
		}
		break;
	case 2:
		// FCR: FIFO Control Register
		// TODO: Ignored for now.
		break;
	case 3:
		// LCR: Line Control Register
		tty->flags = (tty->flags & ~CRBTTY_FLG_DLAB)			| ((value & 0x80) ? CRBTTY_FLG_DLAB				: 0);
		tty->flags = (tty->flags & ~CRBTTY_FLG_SET_BREAK)		| ((value & 0x40) ? CRBTTY_FLG_SET_BREAK		: 0);
		tty->flags = (tty->flags & ~CRBTTY_FLG_STICK_PARITY)	| ((value & 0x20) ? CRBTTY_FLG_STICK_PARITY		: 0);
		tty->flags = (tty->flags & ~CRBTTY_FLG_EVEN_PARITY)		| ((value & 0x10) ? CRBTTY_FLG_EVEN_PARITY		: 0);
		tty->flags = (tty->flags & ~CRBTTY_FLG_ENABLE_PARITY)	| ((value & 0x08) ? CRBTTY_FLG_ENABLE_PARITY	: 0);
		tty->flags = (tty->flags & ~CRBTTY_FLG_MORE_STOP_BITS)	| ((value & 0x04) ? CRBTTY_FLG_MORE_STOP_BITS	: 0);
		tty->flags = (tty->flags & ~CRBTTY_FLG_WORD_LEN_1)		| ((value & 0x02) ? CRBTTY_FLG_WORD_LEN_1		: 0);
		tty->flags = (tty->flags & ~CRBTTY_FLG_WORD_LEN_0)		| ((value & 0x01) ? CRBTTY_FLG_WORD_LEN_0		: 0);
		break;
	case 4:
		// MCR: Modem Control Register
		tty->flags = (tty->flags & ~CRBTTY_FLG_LOOPBACK)		| ((value & 0x10) ? CRBTTY_FLG_LOOPBACK			: 0);
		crb_tty_set_internal_line(tty, CRBTTY_FLG_OUT2, (value & 0x08));
		crb_tty_set_internal_line(tty, CRBTTY_FLG_OUT1, (value & 0x04));
		crb_tty_set_internal_line(tty, CRBTTY_FLG_RTS,  (value & 0x02));
		crb_tty_set_internal_line(tty, CRBTTY_FLG_DTR,  (value & 0x01));
		break;
	case 5:
		// LSR: Line Status Register
		// Ignored.
		break;
	case 6:
		// MSR: Modem Status Register
		// Ignored.
		break;
	case 7:
		// SCR: Scratch Register
		tty->scratch_register = value;
		break;
	default:
		// Invalid register access. Ignored.
		break;
	}

	crbtty_release(tty);

	if (wake_reader) {
		wake_up_interruptible(&tty->readq);
	}
}


///////////////////////
// TTY host interface

static ssize_t crbtty_f_read(struct file* filp, char __user *buf, size_t count, loff_t* f_pos)
{
	struct crb_tty* tty = filp->private_data;

	if (crbtty_acquire_interruptible(tty)) {
		return -ERESTARTSYS;
	}

	while (crbtty_buf_empty(&tty->rbuf)) {
		crbtty_release(tty);
		if (filp->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		}
		if (wait_event_interruptible(tty->readq, !crbtty_buf_empty(&tty->rbuf))) {
			return -ERESTARTSYS;
		}
		if (crbtty_acquire_interruptible(tty)) {
			return -ERESTARTSYS;
		}
	}

	count = crbtty_buf_read(&tty->rbuf, buf, count, 1);
	if (count < 0) {
		crbtty_release(tty);
		return -EFAULT;
	}

	crbtty_release(tty);
	return count;
}

static ssize_t crbtty_f_write(struct file* filp, const char __user *buf, size_t count, loff_t* f_pos)
{
	struct crb_tty* tty = filp->private_data;
	int was_empty;

	if (crbtty_acquire_interruptible(tty)) {
		return -ERESTARTSYS;
	}

	while (crbtty_buf_full(&tty->wbuf)) {
		crbtty_release(tty);
		if (filp->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		}
		if (wait_event_interruptible(tty->writeq, !crbtty_buf_full(&tty->wbuf))) {
			return -ERESTARTSYS;
		}
		if (crbtty_acquire_interruptible(tty)) {
			return -ERESTARTSYS;
		}
	}

	was_empty = crbtty_buf_empty(&tty->wbuf);
	count = crbtty_buf_write(&tty->wbuf, buf, count, 1);
	if (count < 0) {
		crbtty_release(tty);
		return -EFAULT;
	}

	// New data is available, so we send the RDA interrupt
	crb_tty_set_interrupt(tty, CRBTTY_FLG_IRQ_RDA);

	/* If the FIFO was empty before, the LSR will also change to indicate
	// the new status, so we also need to send an LSRC interrupt
	if (was_empty) {
		crb_tty_set_interrupt(tty, CRBTTY_FLG_IRQ_LSRC);
	}*/

	crbtty_release(tty);
	return count;
}

static unsigned int crbtty_f_poll(struct file* filp, struct poll_table_struct* wait)
{
	struct crb_tty* tty = filp->private_data;
	unsigned int mask = 0;

	poll_wait(filp, &tty->readq, wait);
	poll_wait(filp, &tty->writeq, wait);

	crbtty_acquire(tty);
	if (!crbtty_buf_empty(&tty->rbuf)) {
		mask |= POLLIN | POLLRDNORM;
	}
	if (!crbtty_buf_full(&tty->wbuf)) {
		mask |= POLLOUT | POLLWRNORM;
	}
	crbtty_release(tty);

	return mask;
}

static long crbtty_f_ioctl(struct file* filp, unsigned int cmd, unsigned long arg)
{
	void __user *user_arg = (void __user *)arg;
	struct crb_tty* tty = filp->private_data;
	int err;
	int mflags;

	crbtty_acquire(tty);
	switch (cmd) {
	case TCGETS:
		if (!access_ok(VERIFY_WRITE, user_arg, sizeof(struct termios2))) {
			err = -EFAULT;
			break;
		} else if (kernel_termios_to_user_termios((struct termios2 __user *)user_arg, &tty->ktermios)) {
			err = -EFAULT;
			break;
		}
		err = 0;
		break;

	case TCSETS:
		if (!access_ok(VERIFY_READ, user_arg, sizeof(struct termios2))) {
			err = -EFAULT;
			break;
		} else if (user_termios_to_kernel_termios(&tty->ktermios, (struct termios2 __user *)user_arg)) {
			err = -EFAULT;
			break;
		}
		err = 0;
		break;

	case TCFLSH:
		switch (arg) {
		default:
			err = -EINVAL;
			break;
		case TCIFLUSH:
			crbtty_buf_clear(&tty->rbuf);
			err = 0;
			break;
		case TCIOFLUSH:
			crbtty_buf_clear(&tty->rbuf);
			err = 0;
			break;
		case TCOFLUSH:
			err = 0;
			break;
		}
		break;

	case TIOCMGET:
		if (!access_ok(VERIFY_WRITE, user_arg, sizeof(int))) {
			err = -EFAULT;
			break;
		}

		/* Simulate Null-Modem cabling:
                 DTR -> DCD + DSR
		 RTS -> CTS
		 RI  -> (unconnected)
		 */
		mflags = 0;
		mflags |= (tty->flags & CRBTTY_FLG_DTR) ? (TIOCM_DSR | TIOCM_CAR) : 0;
		mflags |= (tty->flags & CRBTTY_FLG_RTS) ? TIOCM_CTS : 0;
		mflags |= (tty->flags & CRBTTY_FLG_EX_DSR) ? TIOCM_DTR : 0;
		mflags |= (tty->flags & CRBTTY_FLG_EX_CTS) ? TIOCM_RTS : 0;

		err = put_user(mflags, (int __user *)user_arg);
		break;

	case TIOCMSET:
		if (!access_ok(VERIFY_READ, user_arg, sizeof(int))) {
			err = -EFAULT;
			break;
		} else if (get_user(mflags, (int __user *)user_arg)) {
			err = -EFAULT;
			break;
		}
		crb_tty_set_external_line(tty, CRBTTY_FLG_EX_DSR | CRBTTY_FLG_EX_DCD, mflags & TIOCM_DTR);
		crb_tty_set_external_line(tty, CRBTTY_FLG_EX_CTS, mflags & TIOCM_RTS);
		err = 0;
		break;

	default:
		printk("CRBTTY: Unknown command 0x%x\n", cmd);
		err = -EINVAL;
		break;
	}
	crbtty_release(tty);

	if (err < 0) {
		printk("CRBTTY: ioctl(%x) FAILED %d\n", cmd, err);
	}

	return err;
}

static int crbtty_f_open(struct inode* inode, struct file* filp)
{
	struct crb_tty* tty = container_of(inode->i_cdev, struct crb_tty, cdev);
	int ret;

	if (crbtty_acquire_interruptible(tty)) {
		return -ERESTARTSYS;
	}

	if (tty->flags & CRBTTY_FLG_OPEN) {
		ret = -EBUSY;
	} else {
		ret = 0;
		tty->flags |= CRBTTY_FLG_OPEN;
		filp->private_data = tty;
	}

	crbtty_release(tty);

	return (ret == 0) ? nonseekable_open(inode, filp) : ret;
}

static int crbtty_f_release(struct inode* inode, struct file* filp)
{
	struct crb_tty* tty = filp->private_data;

	if (!tty || (container_of(inode->i_cdev, struct crb_tty, cdev) != tty)) {
		return -EINVAL;
	}

	crbtty_acquire(tty);
	tty->flags &= ~CRBTTY_FLG_OPEN;
	filp->private_data = NULL;

	crbtty_buf_clear(&tty->rbuf);

	crbtty_release(tty);

	return 0;
}

struct file_operations crbtty_fops = {
	.owner =		THIS_MODULE,
	.llseek =		no_llseek,
	.read =			crbtty_f_read,
	.write =		crbtty_f_write,
	.poll =			crbtty_f_poll,
	.unlocked_ioctl =	crbtty_f_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl =		crbtty_f_ioctl,
#endif
	.open =			crbtty_f_open,
	.release =		crbtty_f_release,
};


void crbtty_cleanup(struct crb_tty* tty)
/*
	Cleanup and destroy a crb_tty descriptor.
*/
{
	printk(KERN_INFO "CRBTTY: Cleanup %p\n", tty);
	if (tty != NULL) {
		if (tty->device) {
			device_destroy(tty->class, tty->devt);
			tty->device = NULL;
		}
		if (tty->flags & CRBTTY_FLG_CDEV_ADDED) {
			cdev_del(&tty->cdev);
			tty->flags &= ~CRBTTY_FLG_CDEV_ADDED;
		}
	}
	kfree(tty);
}

int crbtty_init(struct crb_tty** tty, dev_t devt, int buf_len, struct class* class, int crbif_id, int fun_id, int core_id, int serial_id)
/*
	Allocate and initialize a crb_tty descriptor.

	devt: device number. Something like MKDEV(major, MKMINOR(dev_found, MAXNUM_RCKCRB_FUNCTIONS + tty_id)) when this
		routine is being invoked from crbif_init.

	buf_len: length of the ring buffers for reading and writing. If <= 0, 1024 is assumed.

	class: struct class* of the new device.

	crbif_id, core_id, serial_id: numbers used to create the sysfs entry for the device: "crbif%drb%dc%dttyS%d".
*/
{
	struct crb_tty* dev;
	int err;

	buf_len = buf_len <= 0 ? 1024 : buf_len;

	*tty = NULL;

	// Allocate crb_tty structure
	dev = (struct crb_tty*)kmalloc(sizeof(struct crb_tty) + 2*buf_len, GFP_KERNEL);
	if (!dev) {
		return -ENOMEM;
	}
	memset(dev, 0, sizeof(struct crb_tty) + 2*buf_len);

	// Initialize
	sema_init(&dev->sem, 1);
	init_waitqueue_head(&dev->readq);
	init_waitqueue_head(&dev->writeq);
	crbtty_buf_init(&dev->rbuf, (char*)(dev + 1), buf_len);
	crbtty_buf_init(&dev->wbuf, (char*)(dev + 1) + buf_len, buf_len);
	dev->class = class;
	dev->devt = devt;

	// Add device to kernel
	err = allocate_device_node(dev->devt, 1, &crbtty_fops, &dev->cdev);
	if (err) {
		crbtty_cleanup(dev);
		return err;
	}

	dev->flags |= CRBTTY_FLG_CDEV_ADDED;

	// Create sysfs entry
	dev->device = device_create(dev->class, NULL, dev->devt, dev, "crbif%drb%dc%dttyS%d", crbif_id, fun_id, core_id, serial_id);
	if (!dev->device) {
		crbtty_cleanup(dev);
		return -ENOMEM;
	}

	printk(KERN_INFO "CRBTTY: Init succeeded: tty=%p, dev=%d, fun=%d, core=%d, port=%d\n", dev, crbif_id, fun_id, core_id, serial_id);
	*tty = dev;
	return 0;
}
