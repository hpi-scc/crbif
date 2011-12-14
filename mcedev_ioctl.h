/********************************************************************************
 *
 *
 *
 *
 * $Date: 2009-06-11 14:51:39 +0200 (Thu, 11 Jun 2009) $
 * $Revision: 15400 $
 * $Author: tlehnig $
 * $HeadURL: https://subversion.jf.intel.com/ctg/mtl/mc/mcemu/software/trunk/linuxdrv-2.6/mcedev/mcedev_ioctl.h $
 * $Id: mcedev_ioctl.h 15400 2009-06-11 12:51:39Z tlehnig $
 *
 ********************************************************************************
 */


#ifndef MCEDEV_IOCTL_H
#define MCEDEV_IOCTL_H

#include <linux/ioctl.h>

#define MCEDEV_IOC_MAGIC        0x86

#define MCEDEV_IOC_DEBUG         _IO(MCEDEV_IOC_MAGIC, 0)

// input: 1: set reset   0: clear reset
#define MCEDEV_IOC_RESET         _IO(MCEDEV_IOC_MAGIC, 1)

// return: masked bit of LBC register
#define MCEDEV_IOC_GET_DONE      _IO(MCEDEV_IOC_MAGIC, 2)
#define MCEDEV_IOC_GET_INIT      _IO(MCEDEV_IOC_MAGIC, 3)
#define MCEDEV_IOC_GET_PROG      _IO(MCEDEV_IOC_MAGIC, 4)

// input: 1: set prog   0: clear prog
#define MCEDEV_IOC_SET_PROG      _IO(MCEDEV_IOC_MAGIC, 5)

// input: WH number to use
// return: TTY line number, -9999 for deactivated
#define MCEDEV_IOC_ENA_WH        _IO(MCEDEV_IOC_MAGIC,6)
#define MCEDEV_IOC_DIS_WH        _IO(MCEDEV_IOC_MAGIC, 7)
#define MCEDEV_IOC_GET_WH        _IO(MCEDEV_IOC_MAGIC, 8)

// input: PID of program to send signal to (only for set), 0 to disable.
// return: PID of program that is registered as signal consumer
#define MCEDEV_IOC_SET_PID       _IO(MCEDEV_IOC_MAGIC, 9)
#define MCEDEV_IOC_GET_PID       _IO(MCEDEV_IOC_MAGIC, 10)
  
// return: board revision number as defined by BUSY_STAT and
// STATUS_STAT bit in LBC register
#define MCEDEV_IOC_GET_REV       _IO(MCEDEV_IOC_MAGIC, 11)

// return: internal state / flags of BB driver for this instance
#define MCEDEV_IOC_GET_STATE     _IO(MCEDEV_IOC_MAGIC, 12)

#define MCEDEV_IOC_JTAG_ENABLE   _IO(MCEDEV_IOC_MAGIC, 13)
#define MCEDEV_IOC_JTAG_DISABLE  _IO(MCEDEV_IOC_MAGIC, 14)
// input: 1: set bit   0: clear bit
#define MCEDEV_IOC_JTAG_TCK      _IO(MCEDEV_IOC_MAGIC, 15)
#define MCEDEV_IOC_JTAG_TMS      _IO(MCEDEV_IOC_MAGIC, 16)
#define MCEDEV_IOC_JTAG_TDI      _IO(MCEDEV_IOC_MAGIC, 18)
// return: masked bit of LBC register
#define MCEDEV_IOC_JTAG_TDO      _IO(MCEDEV_IOC_MAGIC, 17)

// return: I2C adapter number (use /dev/i2c-<n>) or
// -9999 for deactivated
#define MCEDEV_IOC_ENA_I2C       _IO(MCEDEV_IOC_MAGIC, 19)
#define MCEDEV_IOC_DIS_I2C       _IO(MCEDEV_IOC_MAGIC, 20)
#define MCEDEV_IOC_GET_I2C       _IO(MCEDEV_IOC_MAGIC, 21)

// input: number of rd waitstates
#define MCEDEV_IOC_SET_RDWS      _IO(MCEDEV_IOC_MAGIC, 22)

//
#define MCEDEV_IOC_ENA_RDBACK    _IO(MCEDEV_IOC_MAGIC, 23)
#define MCEDEV_IOC_DIS_RDBACK    _IO(MCEDEV_IOC_MAGIC, 24)

// input 1: set sda / scl, 0: clear sda / scl
#define MCEDEV_IOC_SET_SDA       _IO(MCEDEV_IOC_MAGIC, 25)
#define MCEDEV_IOC_SET_SCL       _IO(MCEDEV_IOC_MAGIC, 26)

// output: value of respective line
#define MCEDEV_IOC_GET_SDA       _IO(MCEDEV_IOC_MAGIC, 27)
#define MCEDEV_IOC_GET_SCL       _IO(MCEDEV_IOC_MAGIC, 28)

// for debugging purposes only. This IOCTL sends a signal to the process
// as if an interrupt had occured.
#define MCEDEV_IOC_SENDSIG       _IO(MCEDEV_IOC_MAGIC, 29)

// Reenable CD Plane interrupts after signal handler has handled them.
#define MCEDEV_IOC_ENA_INTR	 _IO(MCEDEV_IOC_MAGIC, 30)
#define MCEDEV_IOC_ENA_CDINTR	 _IO(MCEDEV_IOC_MAGIC, 30)


/*
  #define MCEDEV_IOC_GET/SET_DMA_ADDR
  #define MCEDEV_IOC_START/STOP_DMA
*/

#define MCEDEV_IOC_MAXNR 30

#endif // ifndef MCEDEV_IOCTL_H
