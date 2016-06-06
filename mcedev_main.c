/********************************************************************************
 *
 * Main functions of the MCEMU device driver. This driver registers itself for 
 * all MCEMU related PCI devices and calls system specific initialization 
 * functions.
 *
 * $Date: 2010-06-08 16:25:34 +0200 (Tue, 08 Jun 2010) $
 * $Revision: 16545 $
 * $Author: jbrummer $
 * $HeadURL: https://subversion.jf.intel.com/ctg/mtl/mc/mcemu/software/trunk/linuxdrv-2.6/mcedev/mcedev_main.c $
 * $Id: mcedev_main.c 16545 2010-06-08 14:25:34Z jbrummer $
 *
 ********************************************************************************
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/sysfs.h>
#include <linux/device.h>

#include "mcedev.h"
#include "mcedev_common.h"
#include "mcedev_pciids.h"

#include "mcedev_build.h"

#if defined (MCEDEV_MODULE_MCECD) || defined (MCEDEV_MODULE_ALL)
#include "bbdev.h"
#endif

#if defined (MCEDEV_MODULE_MCEAPPL) || defined (MCEDEV_MODULE_ALL)
#include "bbappl.h"
#endif

#if defined (MCEDEV_MODULE_DINI)
#include "dndev.h"
#endif

#if defined (MCEDEV_MODULE_RCKCRB) || defined (MCEDEV_MODULE_CRBIF) || defined (MCEDEV_MODULE_ALL)
#include "crbdev.h"
#endif

#include "mcedev_ver.h"


const char MCEIDSTRING[] = "$Id: mcedev_main.c 16545 2010-06-08 14:25:34Z jbrummer $";


static struct pci_device_id mcedev_ids[] = {
#if defined (MCEDEV_MODULE_MCECD) || defined (MCEDEV_MODULE_ALL)
  { PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_BB_CD), 
    .driver_data = (kernel_ulong_t)&bb_init},
#endif
#if defined (MCEDEV_MODULE_MCEAPPL) || defined (MCEDEV_MODULE_ALL)
  { PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_BB_APPL), 
    .driver_data = (kernel_ulong_t)&bb_init},
#endif
#if defined (MCEDEV_MODULE_RCKCRB) || defined (MCEDEV_MODULE_ALL)
  { PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_CRB),
    .driver_data = (kernel_ulong_t)&rckcrb_init},
#endif
#if defined (MCEDEV_MODULE_CRBIF)
  { PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_CRB),
    .driver_data = (kernel_ulong_t)&crbif_init},
  { PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_RLB),
    .driver_data = (kernel_ulong_t)&crbif_init},
#endif
#if defined (MCEDEV_MODULE_DINI)
  { PCI_DEVICE(PCI_VENDOR_ID_DINI, PCI_DEVICE_ID_DINI_8K), 
    .driver_data = (kernel_ulong_t)&mcedev_no_init},
#endif
#if defined (MCEDEV_MODULE_ML505)
  { PCI_DEVICE(PCI_VENDOR_ID_XILINX, PCI_DEVICE_ID_XILINX_ML505),
    .driver_data = (kernel_ulong_t)&mcedev_no_init},
#endif
  { 0, }
};


MODULE_DEVICE_TABLE(pci, mcedev_ids);

struct mcedev_data *mcemu_device_table[MAXNUM_MCE_DEV];

struct class *mcedev_class;

int mcedev_debug = 0x0;

int mcedev_found = 0;
int mcedev_major = MCEDEV_MAJOR;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 37)
struct semaphore mcedev_sem;
DEFINE_SEMAPHORE(mcedev_sem);
#else
DECLARE_MUTEX(mcedev_sem);
#endif


static int mcedev_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
  int result, index;
  int (*startup)(struct mcedev_data *mcedev, int dev_found, int major);
  struct mcedev_data *mce_dev; // = &mcemu_devices[mcedev_found];

  // Print information

  printk(KERN_INFO "mcedev_probe found device: VID 0x%04x, DID 0x%04x, SVID 0x%04x, SDID 0x%04x, BUS 0x%02x, DEVFN 0x%02x.%01i \n",
	 dev->vendor, dev->device, dev->subsystem_vendor, dev->subsystem_device, 
	 dev->bus->number, PCI_SLOT(dev->devfn), PCI_FUNC(dev->devfn));

  // enable PCI device - needed to access it further on
  result = pci_enable_device(dev);
  if (result) {
    pci_disable_device(dev);
    return -ENODEV;
  }

  // Allocate memory for device/driver data
  mce_dev = kmalloc(sizeof(*mce_dev), GFP_KERNEL);
  if (!mce_dev) {
    printk(KERN_INFO "mcedev_probe: Out of memory ... \n");
    pci_disable_device(dev);
    return -ENOMEM;
  }
  
  // fill initial data structures
  mce_dev->dev = dev;
  mce_dev->specific_data = NULL;
  mce_dev->mcedev_type = MCEDEV_TYPE_UNDEFINED;
  mce_dev->mcedev_class = mcedev_class;


  // Get mutex on device table
  down(&mcedev_sem);
  
  // More devices already registered than we can handle? ... Should never happen. ;-)
  if (mcedev_found >= MAXNUM_MCE_DEV) {
    printk(KERN_INFO "mcedev_probe: reached maximum number of supported devices (%i), not initializing device\n",
	   mcedev_found);
    up(&mcedev_sem);
    pci_disable_device(dev);
    kfree(mce_dev);
    return -ENODEV;
  }
  
  // This is our slot in table
  index = mcedev_found;

  // Table clean?
  if (mcemu_device_table[index] != NULL) {
    MPRINTK(0x01, KERN_DEBUG "mcedev_probe: WARNING: table entry %i is not NULL!\n", index);
  }

  // Insert our structure into table
  mcemu_device_table[index] = mce_dev;
  MPRINTK(0x01, KERN_DEBUG "mcedev_probe: Added mce_dev into slot %i of device table\n", index);

  // Call driver initialization function
  if (id->driver_data != 0) {
    startup = (void *) id->driver_data;
  }
  else {
    startup = mcedev_no_init;
  }
   
  result = startup(mce_dev, index, mcedev_major);
  MPRINTK(0x01, KERN_DEBUG "mcedev_probe: startup result = %i\n", result);

  if (result <= 0) {
    printk(KERN_INFO "mcedev_probe: device specific startup failed (%i)\n", result);
    mcemu_device_table[index] = NULL;
    up(&mcedev_sem);
    pci_disable_device(dev);
    kfree(mce_dev);
    return -ENODEV;
  }

  // Increment total and release semaphore, our structure is now in global table
  mcedev_found++;

  up(&mcedev_sem);

  return 0;
}



static void mcedev_remove(struct pci_dev *dev)
{
  int i;

  // Walk through the list of device table entries and clean up any allocated resources.

  MPRINTK(0x01, KERN_DEBUG "mcedev_remove: %i active devices found\n", mcedev_found);

  if (mcedev_found > 0) {
    for (i = 0; i < MAXNUM_MCE_DEV; i++) {
      MPRINTK(0x01, KERN_DEBUG "mcedev_remove: device table slot %i ... ", i);
      if ((mcemu_device_table[i] != NULL) && (dev == mcemu_device_table[i]->dev)) {
	MPRINTK(0x01, "struct pci_dev matches\n");
	  if (mcemu_device_table[i]->cleanup != NULL) {
	    mcemu_device_table[i]->cleanup(mcemu_device_table[i]);
	  }
	  else 
	    MPRINTK(0x01, KERN_DEBUG "... not calling cleanup (is NULL!!)\n");
	  down(&mcedev_sem);
	  pci_disable_device(mcemu_device_table[i]->dev);
	  kfree(mcemu_device_table[i]);
	  mcemu_device_table[i] = NULL;
	  mcedev_found--;
	  up(&mcedev_sem);
       
      }
      else MPRINTK(0x01, "no match\n");
    }
  }
}


static struct pci_driver pci_driver = {
  .name = "mcedev",
  .id_table = mcedev_ids,
  .probe = mcedev_probe,
  .remove = mcedev_remove,
};



static int __init mcedev_init(void)
{ 
  int result = 0;
  dev_t mcedev_dev;

  printk(KERN_INFO "mcedev Id: %s, Version %s\n", MCEIDSTRING, MODVERSTRING);

  if (mcedev_major) {
    mcedev_dev = MKDEV(mcedev_major, 0);
    MPRINTK(MCEDBG_GLOBAL, KERN_DEBUG "mcedev: registering %i minors for major %i\n", 
	   (MAXNUM_MCE_DEV + 1) * (1U << MINOR_FUNCTIONBITS), mcedev_major);
    result = register_chrdev_region(mcedev_dev, (MAXNUM_MCE_DEV + 1) * (1U << MINOR_FUNCTIONBITS), "mcedev");
    if (result != 0) {
      printk(KERN_ERR "mcedev: register_chrdev_region() failed: %i, Trying alloc\n", result);
      mcedev_major = 0;
    }
  }
  if (mcedev_major == 0) {
    result = alloc_chrdev_region(&mcedev_dev, 0,(MAXNUM_MCE_DEV + 1) * (1U << MINOR_FUNCTIONBITS), "mcedev");
    if (result != 0) {
      printk(KERN_ERR "mcedev: alloc_chrdev_region() failed: %i\n", result);
      return -ENODEV;
    }
    mcedev_major = MAJOR(mcedev_dev);
    MPRINTK(MCEDBG_GLOBAL, KERN_DEBUG "mcedev: succesfully allocated %i minors for major: %i\n", 
	   (MAXNUM_MCE_DEV + 1) * (1U << MINOR_FUNCTIONBITS), mcedev_major);
  }

  mcedev_class = class_create(THIS_MODULE, "mcedev");
  if (IS_ERR(mcedev_class)) {
    printk(KERN_ERR "mcedev: Error creating mcedev class.\n");
    return -ENODEV;
  }
 
  return pci_register_driver(&pci_driver);
}



static void __exit mcedev_exit(void)
{
  dev_t mcedev_dev = MKDEV(mcedev_major, 0);

  pci_unregister_driver(&pci_driver); 
  class_destroy(mcedev_class);
  unregister_chrdev_region(mcedev_dev, (MAXNUM_MCE_DEV + 1) * (1U << MINOR_FUNCTIONBITS));

  printk(KERN_INFO "mcedev: driver unregistered\n");
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Thomas Lehnig");
MODULE_VERSION(MODVERSTRING);


module_init(mcedev_init);
module_exit(mcedev_exit);

module_param(mcedev_debug, int, 0644);
MODULE_PARM_DESC(mcedev_debug, "debug mask: 0x01:global, 0x02:common, 0x04:open(), 0x08:release(), 0x10:mmap(), 0x20:init(), 0x40:cleanup(), 0x80:interrupt, 0x100:ioctl, 0x200:i2c, 0x400:serial, 0x800:DMA, 0x1000:read/write");
