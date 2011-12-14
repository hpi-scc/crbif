/********************************************************************************
 *
 *
 *
 *
 * $Date: 2009-11-19 14:46:05 +0100 (Thu, 19 Nov 2009) $
 * $Revision: 16326 $
 * $Author: tlehnig $
 * $HeadURL: https://subversion.jf.intel.com/ctg/mtl/mc/mcemu/software/trunk/linuxdrv-2.6/mcedev/mcedev.h $
 * $Id: mcedev.h 16326 2009-11-19 13:46:05Z tlehnig $
 *
 ********************************************************************************
 */


#ifndef MCEDEV_H
#define MCEDEV_H

#include <linux/pci.h>
#include <linux/sysfs.h>
#include <linux/device.h>

/*
 * General defines and structures for all specific drivers
 */


// definition of devices supported for easy access

#define MCEDEV_TYPE_UNDEFINED    0
#define MCEDEV_TYPE_BB_CD        1
#define MCEDEV_TYPE_BB_APPL      2
#define MCEDEV_TYPE_DINI_CD      3
#define MCEDEV_TYPE_RCK_CRB      4


// We have a maximum of 6 (0 - 5) supported BARs on our PCI devices
#define MAXNUM_PCI_BAR 6

// Maximum number of MCEMU devices supported in the system
#define MAXNUM_MCE_DEV 10

// Default major number for our driver
#define MCEDEV_MAJOR 1024


// Debug printk filter
#define MCEDBG_GLOBAL     0x0001
#define MCEDBG_COMMON     0x0002
#define MCEDBG_OPEN       0x0004
#define MCEDBG_RELEASE    0x0008
#define MCEDBG_MMAP       0x0010
#define MCEDBG_INIT       0x0020
#define MCEDBG_CLEANUP    0x0040
#define MCEDBG_INTERRUPT  0x0080
#define MCEDBG_IOCTL      0x0100
#define MCEDBG_I2C        0x0200
#define MCEDBG_SERIAL     0x0400
#define MCEDBG_DMA        0x0800
#define MCEDBG_READWRITE  0x1000
#define MCEDBG_NET        0x2000
#define MCEDBG_PIO	  0x4000
// debug low level DMA only
#define MCEDBG_LLDMA	  0x8000


// #define MCEDBG(mask) if (mcedev_debug & mask) 
#define MPRINTK(mask, format, args...) if (mcedev_debug & mask){ printk(format, ##args);}


// data structure to store all the information we know about the device's bars
struct mcedev_bar_type {
  unsigned long start;
  unsigned long size;
  unsigned long pci_flags;
};


/* generic data structure to capture all we know about a device that's supported by this driver. 
 * Device specific data will be stored in the device specific pointer (e.g. data about bbdev)
 */


struct mcedev_data {
  struct mcedev_bar_type bar[MAXNUM_PCI_BAR];
  unsigned int irq;
  int mcedev_type;
  void (*cleanup) (struct mcedev_data *mcedev);
  struct pci_dev *dev;
  struct class *mcedev_class;
  void *specific_data;
};
  


#endif // ifndef MCEDEV_H
