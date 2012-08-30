/********************************************************************************
 *
 *
 *
 *
 * $Date: 2010-03-31 13:19:28 +0200 (Wed, 31 Mar 2010) $
 * $Revision: 16496 $
 * $Author: tlehnig $
 * $HeadURL: https://subversion.jf.intel.com/ctg/mtl/mc/mcemu/software/trunk/linuxdrv-2.6/mcedev/crbdev_misc.c $
 * $Id: crbdev_misc.c 16496 2010-03-31 11:19:28Z tlehnig $
 *
 ********************************************************************************
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>

#include <asm/io.h>

// #include <asm/msr.h>

#include "mcedev.h"
#include "mcedev_common.h"
#include "crbdev.h"


extern int mcedev_debug;
extern int tlpsmax;


/*
 * simple bitfield printout and test, initialization of global settings
 */

void rckcrb_init_bitfield_test(struct rckcrb_data *crb_appl) {
  u32 BitSID;
  u64 grbtest;
  u32 tmp;

  BitSID = ioread32(RRA(RCK_ID0_REG));
  printk(KERN_INFO "rckcrb_init: BITSID: 0x%08x \n", BitSID);

  grbtest = readl(RRA(RCK_ID1_REG));
  grbtest |= ((u64)readl(RRA(RCK_ID2_REG))) << 32;
  MPRINTK(MCEDBG_INIT, KERN_DEBUG "rckcrb_init: grbtest: 0x%016llx \n", grbtest);

  writel(0x35377353, RRA(RCK_ID2_REG));
  writel((grbtest & 0xffffffff) + 1, RRA(RCK_ID1_REG));

  grbtest = readl(RRA(RCK_ID1_REG));
  grbtest |= ((u64)readl(RRA(RCK_ID2_REG))) << 32;
  MPRINTK(MCEDBG_INIT, KERN_DEBUG "rckcrb_init: grbtest: 0x%016llx \n", grbtest);

  tmp = readl(RRA(XLX_DCSR1));
  MPRINTK(MCEDBG_INIT, KERN_DEBUG "rckcrb_init: Xilinx DCSR1: BuildVersion: 0x%02x, DataPathWidth: 0x%02x, FpgaFamily: 0x%02x (0x%08x)\n", 
	  (tmp >> 8) & 0xff, (tmp >> 16) & 0xff, (tmp >> 24) & 0xff, tmp);

  tmp = readl(RRA(XLX_DLWSTAT));
  MPRINTK(MCEDBG_INIT, KERN_DEBUG "rckcrb_init: Xilinx DLWSTAT: cap. max width: 0x%02x, neg. max width: 0x%02x (0x%08x)\n", 
	  tmp & 0x3f, (tmp >> 8) & 0x3f, tmp);

  tmp = readl(RRA(XLX_DLTRSSTAT));
  MPRINTK(MCEDBG_INIT, KERN_DEBUG "rckcrb_init: Xilinx DLTRSSTAT: cap. max size: 0x%01x, prog. max size: 0x%01x, max rd req size: 0x%01x, (0x%08x)\n", 
	  tmp & 0x07, (tmp >> 8) & 0x07, (tmp >> 16) & 0x07, tmp);

  tmp = readl(RRA(XLX_DMISCCONT));
  MPRINTK(MCEDBG_INIT, KERN_DEBUG "Misc Control:                 0x%08x\n", tmp);
}



void rckcrb_disable_ASPM(struct mcedev_data *mcedev) {
  int i, err; 
  u32 tmp, tmp2;

  MPRINTK(MCEDBG_INIT, KERN_DEBUG "RCKCRB DBG: Device: V:0x%04x D:0x%04x S:0x%02x F:0x%02x\n", 
	  mcedev->dev->vendor,
	  mcedev->dev->device,
	  PCI_SLOT(mcedev->dev->devfn),
	  PCI_FUNC(mcedev->dev->devfn));

  MPRINTK(MCEDBG_INIT, KERN_DEBUG "RCKCRB DBG: Bus: n:%i, p:%i\n",
	  mcedev->dev->bus->number,
	  mcedev->dev->bus->primary);

  MPRINTK(MCEDBG_INIT, KERN_DEBUG "RCKCRB DBG: Bus->Self: V:0x%04x D:0x%04x S:0x%02x F:0x%02x\n", 
	  mcedev->dev->bus->self->vendor,
	  mcedev->dev->bus->self->device,
	  PCI_SLOT(mcedev->dev->bus->self->devfn),
	  PCI_FUNC(mcedev->dev->bus->self->devfn));

  MPRINTK(MCEDBG_INIT, KERN_DEBUG "RCKCRB DBG: Bus->Self->Bus: n:%i, p:%i\n",
	  mcedev->dev->bus->self->bus->number,
	  mcedev->dev->bus->self->bus->primary);


  printk(KERN_DEBUG "rckcrb_init: Checking and disabling ASPM on root port %04x:%04x (%02x:%02x.%02x) for rckcrb device\n",
	 mcedev->dev->bus->self->vendor,  mcedev->dev->bus->self->device,
	 mcedev->dev->bus->self->bus->number, PCI_SLOT(mcedev->dev->bus->self->devfn), 
	 PCI_FUNC(mcedev->dev->bus->self->devfn));

	 
  i = pci_find_capability(mcedev->dev->bus->self, 0x10);
  printk(KERN_DEBUG "    Found PCIe Capability structure @ 0x%02x\n", i);

  err = pci_read_config_dword(mcedev->dev->bus->self, i + 0x10, &tmp);
  MPRINTK(MCEDBG_INIT, KERN_DEBUG "RCKCRB DBG: Read Config @ 0x%02x: 0x%08x (%i)\n", i + 0x10, tmp, err);

  if (tmp & 0x03) {
    tmp2 = (tmp & (~0x0003));
    printk(KERN_DEBUG "    Link Control Reg @ 0x%02x: 0x%08x, setting to 0x%08x\n", i + 0x10, tmp, tmp2);

    err =  pci_write_config_byte(mcedev->dev->bus->self, i + 0x10, (tmp2 & 0xff));

    err = pci_read_config_dword(mcedev->dev->bus->self, i + 0x10, &tmp);
    printk(KERN_DEBUG "    Link Control Reg @ 0x%02x changed to: 0x%08x\n", i + 0x10, tmp);
    MPRINTK(MCEDBG_INIT, KERN_DEBUG "RCKCRB DBG: Read Config @ 0x%02x: 0x%08x (%i)\n", i + 0x10, tmp, err);
  }
  else {
    printk(KERN_DEBUG "    Link Control Reg @ 0x%02x: 0x%08x --> ASPM already disabled!\n", i + 0x10, tmp);
  }
}


int rckcrb_set_tlpsmax(struct rckcrb_data *crb_appl) {
  u32 tmp, plSize, retval;
  
  tmp = readl(RRA(XLX_DLTRSSTAT));

  plSize = 1 << (7 + ((tmp >> 8) & 0x07));

  if ((plSize < 128) || (plSize > 4096)) {
    printk(KERN_ERR "rckcrb_set_tlpsmax(): Invalid programmed maximum payload size (Xilinx DLTRSSTAT): %i\n",
	   plSize);
    return plSize * (-1);
  }

  // TLP size is in 32bit words, payload size in bytes
  retval = plSize / 4; 
  
  MPRINTK(MCEDBG_INIT, KERN_DEBUG "rckcrb_init: Setting tlpsmax to %i (programmed max payload size is %i).\n",
	  retval, plSize);

  return retval;
}
