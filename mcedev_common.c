/********************************************************************************
 *
 *
 *
 *
 * $Date: 2009-06-11 14:51:39 +0200 (Thu, 11 Jun 2009) $
 * $Revision: 15400 $
 * $Author: tlehnig $
 * $HeadURL: https://subversion.jf.intel.com/ctg/mtl/mc/mcemu/software/trunk/linuxdrv-2.6/mcedev/mcedev_common.c $
 * $Id: mcedev_common.c 15400 2009-06-11 12:51:39Z tlehnig $
 *
 ********************************************************************************
 */



#include <linux/pci.h>
#include <linux/fs.h>
#include <linux/cdev.h>

#include "mcedev.h"
#include "mcedev_common.h"


extern int mcedev_debug;


int mcedev_no_init(struct mcedev_data *mcedev, int dev_found, int major) {

  printk(KERN_INFO "No driver functionality implemented yet for this device. Not initializing.\n");
  
  return -ENODEV;
}


void mcedev_set_resources(struct mcedev_data *mcedev, struct pci_dev *dev) {
  int i;

  for (i = 0; i < MAXNUM_PCI_BAR; i++) {
    mcedev->bar[i].start = dev->resource[i].start;
    mcedev->bar[i].size =  dev->resource[i].end - dev->resource[i].start + 1;
    mcedev->bar[i].pci_flags = dev->resource[i].flags;
  } 
  mcedev->irq = dev->irq;
}


void mcedev_print_resources(struct mcedev_data *mcedev) {
  int i;
  for (i = 0; i < MAXNUM_PCI_BAR; i++) {
    MPRINTK(0x02, KERN_DEBUG "BAR %i (%s) at 0x%08lx, size 0x%08lx, flags 0x%08lx\n", 
	   i, mcedev->dev->resource[i].name, mcedev->bar[i].start, 
	   mcedev->bar[i].size, mcedev->bar[i].pci_flags);
  } 
  MPRINTK(0x02, KERN_DEBUG "IRQ: %i\n", mcedev->irq);
}
  

int allocate_device_node(int devid, int devcount, struct file_operations *myfops, struct cdev *cdev) {
  int err;

  // TODO: Add check to make sure devcount doesn't spill over into next MINOR_DEVNUM space
  cdev_init(cdev, myfops);
  cdev->ops = myfops;
  cdev->owner = THIS_MODULE;
  err = cdev_add(cdev, devid, devcount);
  if (err)
    printk(KERN_INFO  "Error %d adding device: 0x%08x (%d:%d(%d:%d))\n", 
	   err, devid, MAJOR(devid), MINOR(devid),
	   MINOR_DEVNUM(MINOR(cdev->dev)), MINOR_FUNCTION(MINOR(cdev->dev)));
  else
    MPRINTK(0x02, KERN_DEBUG "cdev add ok (%i) device: 0x%08x (%d:%d:(%d:%d)), count: %d\n", 
	   err, cdev->dev, MAJOR(cdev->dev), MINOR(cdev->dev), 
	   MINOR_DEVNUM(MINOR(cdev->dev)), MINOR_FUNCTION(MINOR(cdev->dev)), cdev->count);
  return err;

}


int map_to_kernelmem(unsigned long phys_addr, unsigned long size, void **target, char *desc) {

  MPRINTK(0x02, KERN_DEBUG "map_to_kernelmem: %s for access by the kernel (0x%08lx, 0x%08lx) ...\n",
	 desc, phys_addr, size);
  
  *target = ioremap_nocache(phys_addr, size);
  if (target) {
    MPRINTK(0x02, KERN_DEBUG "map_to_kernelmem: ... Success, mapped to 0x%08lx\n", (unsigned long) *target);
  }
  else {
    MPRINTK(0x02, KERN_DEBUG "map_to_kernelmem: ... Failed!\n");
    return -ENODEV;
  }

  return 0;
}


/*
void mcedev_probe_bars(struct pci_dev *dev) {
  int i;
  u16 val1, val2;
  u32 bar[6];
  u32 bar_r;
  u16 d_control;

  pci_read_config_word(dev, PCI_DEVICE_ID, &val1);
  pci_read_config_word(dev, PCI_VENDOR_ID, &val2);
  pci_read_config_word(dev, PCI_COMMAND, &d_control);

  printk(KERN_DEBUG "Device ID: 0x%04x, Vendor ID: 0x%04x, Control: 0x%04x\n", val1, val2, d_control);

  pci_write_config_word(dev, PCI_COMMAND, 0x0000);

  for (i=0; i < 6; i++) {
    pci_read_config_dword(dev, PCI_BASE_ADDRESS_0 + i*4, &bar[i]);
    pci_write_config_dword(dev, PCI_BASE_ADDRESS_0 + i*4, 0xffffffff);
    pci_read_config_dword(dev, PCI_BASE_ADDRESS_0 + i*4, &bar_r);
    pci_write_config_dword(dev, PCI_BASE_ADDRESS_0 + i*4, bar[i]);
    printk(KERN_DEBUG "BAR%i: set to 0x%08x - Address mask: 0x%08x\n", i, bar[i], bar_r);
  }

  pci_write_config_word(dev, PCI_COMMAND, d_control);
}

*/
