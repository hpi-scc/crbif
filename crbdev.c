/********************************************************************************
 *
 *
 *
 *
 * $Date: 2010-04-29 10:54:15 +0200 (Thu, 29 Apr 2010) $
 * $Revision: 16530 $
 * $Author: jbrummer $
 * $HeadURL: https://subversion.jf.intel.com/ctg/mtl/mc/mcemu/software/trunk/linuxdrv-2.6/mcedev/crbdev.c $
 * $Id: crbdev.c 16530 2010-04-29 08:54:15Z jbrummer $
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
#include <linux/interrupt.h>
#include <linux/device.h>
#include <asm/io.h>
#include <asm/uaccess.h>

#include "mcedev.h"
#include "mcedev_common.h"
#include "crbdev.h"

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18) 
#define IRQF_SHARED SA_SHIRQ
#define IRQF_DISABLED SA_INTERRUPT
#endif


const char RCKCRBIDSTRING[] = "$Id: crbdev.c 16530 2010-04-29 08:54:15Z jbrummer $";

extern struct mcedev_data *mcemu_device_table[];

extern int mcedev_debug;
extern int tlpsmax;


int usePioForRest = 0;
int usePioOnly = 0;
int disableASPM = 1;

module_param(usePioForRest, int, 0644);
MODULE_PARM_DESC(usePioForRest, "Use PIO mode to tranfer data that didn't fit into DMA (TLPC * TLPS), use second DMA otherwise");

module_param(usePioOnly, int, 0644);
MODULE_PARM_DESC(usePioOnly, "Use PIO mode to tranfer all data (test and debug only, very slow!)");

module_param(disableASPM, int, 0644);
MODULE_PARM_DESC(disableASPM, "Disable Active State Power Management of link partner.");


/*
 * device function to BAR transaltion.
 * device function is index into array, 
 * first element is BAR number
 * second element is Busregion size (actually should be how much is really used - 0 means unused - do not use!)
 */

// Xilinx PCIe I/F
const struct crbappl_func2bar func2barXilinx[MAXNUM_RCKCRB_FUNCTIONS+1] = {
  {0, 1024 * 1024 },			// 1MB of memory mapped IO in BAR0
  {-1, 0},				// end of list
};


// Forward Declarations of file functions
int rckcrb_open(struct inode *inode, struct file *filp);
int rckcrb_release(struct inode *inode, struct file *filp);
ssize_t rckcrb_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
ssize_t rckcrb_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);
int rckcrb_mmap(struct file *filp, struct vm_area_struct *vma);


struct file_operations rckcrb_fops[MAXNUM_RCKCRB_FUNCTIONS] = {
  {.owner = THIS_MODULE, 
   .open = rckcrb_open, 
   .read = rckcrb_read, 
   .write = rckcrb_write, 
   .mmap = rckcrb_mmap, 
   .release = rckcrb_release },
};



/*
 * Implementation of the open(), release() and mmap() functions as required for the
 * device nodes of MCEMU application PCIe interface.
 */

int rckcrb_open(struct inode *inode, struct file *filp) {
  int minor = iminor(inode);
  int major = imajor(inode);
  int devnum = MINOR_DEVNUM(minor);
  int function = MINOR_FUNCTION(minor);
  
  struct rckcrb_data *crb_appl;

  MPRINTK(MCEDBG_OPEN, KERN_DEBUG "rckcrb_open: maj: %i, min: %i, (devnum: %i, function: %i)\n", 
	 major, minor, devnum, function);

  if (function >= MAXNUM_RCKCRB_FUNCTIONS) {
    printk(KERN_ERR "rckcrb_open: Invalid function: %i\n", function);
    return -ENODEV;
  }

  if (mcemu_device_table[devnum] == NULL) {
    printk(KERN_ERR "rckcrb_open: mcemu_devices[%i] is NULL pointer\n",	
	   devnum);
    return -ENXIO; 
  }

  // set private data of filepointer to point to mcemu_devices structure for this device instance
  filp->private_data = (void*)mcemu_device_table[devnum];

  crb_appl = ((struct mcedev_data*)filp->private_data)->specific_data;

  if (crb_appl == NULL) {
    printk(KERN_ERR "rckcrb_open: crb_appl specific data not set in filp->private_data->specific_data\n");
    return -ENXIO; 
  }

  rckcrb_reset_scemi(crb_appl); 

  // The device file may only be opened once - TBD. to do that atomically
  if (crb_appl->crb_function[function].use_count != 0) {
    printk(KERN_ERR "rckcrb_open: use_count for function %i != 0 (%i)\n", 
	   function, crb_appl->crb_function[function].use_count);	
    return -EPERM;
  }

  crb_appl->crb_function[function].use_count++;
  MPRINTK(MCEDBG_OPEN, KERN_DEBUG "rckcrb_open: usage count for this function (%i): %i\n", 
	 function, crb_appl->crb_function[function].use_count);

  return 0;
}


/*
 * release - called when user calls close() on fd.
 */

int rckcrb_release(struct inode *inode, struct file *filp) {
  int minor = iminor(inode);
  int major = imajor(inode);
  int devnum = MINOR_DEVNUM(minor);
  int function = MINOR_FUNCTION(minor);

  struct rckcrb_data *crb_appl;

  int i, usage = 0; //, clean = 0;

  MPRINTK(MCEDBG_RELEASE, KERN_DEBUG "rckcrb_release: maj: %i, min: %i, (devnum: %i, function: %i)\n", 
	 major, minor, devnum, function);

  crb_appl = ((struct mcedev_data*)filp->private_data)->specific_data;
  if (crb_appl == NULL) {
    printk(KERN_ERR "rckcrb_release: rckcrb specific data not set in filp->private_data->specific_data\n");
    return -ENXIO;
  }

  if (crb_appl->crb_function[function].use_count > 0) {
    crb_appl->crb_function[function].use_count--;
  }
  else {
    MPRINTK(MCEDBG_RELEASE, KERN_DEBUG "rckcrb_release: WARNING: use count is already at 0!\n");
  }

  MPRINTK(MCEDBG_RELEASE, KERN_DEBUG "rckcrb_release: usage count for this function (%i): %i\n", 
	 function, crb_appl->crb_function[function].use_count);

  // Calculate usage count of all functions
  usage = 0;

  for (i = 0; i < MAXNUM_RCKCRB_FUNCTIONS; i++) {
    usage += crb_appl->crb_function[i].use_count;
  }

  // Do some cleanups if usage of all functions is 0
  MPRINTK(MCEDBG_RELEASE, KERN_DEBUG "rckcrb_release: total usage count at %i\n", usage);

  return 0;
}


/*
 * read - takes care of MOP level / buffer management
 */

ssize_t rckcrb_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos) {
  int minor = iminor(filp->f_dentry->d_inode);
  int major = imajor(filp->f_dentry->d_inode);
  int devnum = MINOR_DEVNUM(minor);
  int function = MINOR_FUNCTION(minor);

  u32 tmp, moplvl;
  unsigned int data_avail, packets_to_read, retval, left1 = 0, left2 = 0, reploop;

  struct mcedev_data *mce_dev = (struct mcedev_data *)filp->private_data;
  struct rckcrb_data *crb_appl = (struct rckcrb_data *)mce_dev->specific_data;

  MPRINTK(MCEDBG_READWRITE, KERN_DEBUG "rckcrb_read: maj: %i, min: %i, (devnum: %i, function: %i)\n", 
	 major, minor, devnum, function);

  // check MOP level
  tmp = readl(RRA(RCK_TRNCT4_REG));
  moplvl = tmp & MOP_LVL_MSK;

  if (moplvl & MOP_LVLERR_MSK) {
    // error state in MOP FIFO
    printk(KERN_ERR "rckcrb_read: Error in MOP FIFO: 0x%04x\n", moplvl);
    return -EIO;
  }

  // calculate available data in bytes
  data_avail = moplvl * MOP_FIFO_WIDTH;

  // calculate number of packets minimum(available data in MOP FIFO, size of return buffer)
  if (count >= data_avail)
    packets_to_read = data_avail / MOP_BYTE_PACKET_SIZE;
  else
    packets_to_read = count / MOP_BYTE_PACKET_SIZE;

  if (packets_to_read <= 0) {
    MPRINTK((MCEDBG_READWRITE | MCEDBG_DMA), 
	    KERN_DEBUG "rckcrb_read: No packet to read (data_avail: %i, count: %i)\n", data_avail, (int)count);

    return packets_to_read;
  }

  MPRINTK((MCEDBG_READWRITE | MCEDBG_DMA), 
	  KERN_DEBUG "rckcrb_read: moplvl: 0x%04x, data_avail: %i Bytes, count: %i Bytes, packets_to_read: %i\n", 
	  moplvl, data_avail, (int)count, packets_to_read);

  // read from device with read_dma
  if (!usePioOnly)  
    left1 = rckcrb_read_dma_w(crb_appl, packets_to_read * MOP_WORD_PACKET_SIZE, 0);
  else
    left1 = rckcrb_read_pio_w(crb_appl, packets_to_read * MOP_WORD_PACKET_SIZE, 0);

  if (left1 != 0) {
    reploop = 2;
    left2 = left1;
    do {
      MPRINTK((MCEDBG_READWRITE | MCEDBG_DMA), 
	      KERN_DEBUG "rckcrb_read: Leftover after DMA: %i, reading in %i.Try\n", left2, reploop);
      if (!usePioForRest)
	left2 = rckcrb_read_dma_w(crb_appl, left2, (packets_to_read * MOP_WORD_PACKET_SIZE) - left2);
      else
	left2 = rckcrb_read_pio_w(crb_appl, left2, (packets_to_read * MOP_WORD_PACKET_SIZE) - left2);
      if (left2 != 0) {
	MPRINTK((MCEDBG_READWRITE | MCEDBG_DMA), 
		KERN_DEBUG "rckcrb_read: Leftover after %i.Try: %i, should not happen!\n", reploop, left2);
      }
      reploop++;
    } while ((reploop <= 4) && (left2 != 0));
  }

  if (left2 != 0) {
    printk(KERN_ERR "rckcrb_read: Error: Could not read all data after %i repetitions, leftover: %i\n", reploop, left2);
    return -EFAULT;
  }

  // copy packets from rx_dma_buffer to userspace
  retval = copy_to_user(buf, crb_appl->dma_tx_kernelptr, packets_to_read * MOP_BYTE_PACKET_SIZE);
  if (retval != 0) {
    printk(KERN_ERR "rckcrb_read: Error: copy_to_user() has %i remaining Bytes\n", retval);
    return -EFAULT;
  }

  // increment f_pos by amount written
  *f_pos += packets_to_read * MOP_BYTE_PACKET_SIZE;

  // return count
  return packets_to_read * MOP_BYTE_PACKET_SIZE;
}



/*
 * write - takes care of MIP level / buffer management
 */

ssize_t rckcrb_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos) {
  int minor = iminor(filp->f_dentry->d_inode);
  int major = imajor(filp->f_dentry->d_inode);
  int devnum = MINOR_DEVNUM(minor);
  int function = MINOR_FUNCTION(minor);

  u32 tmp, miplvl;
  unsigned int room_avail, packets_to_write, retval, left1 = 0, left2 = 0, reploop;

  struct mcedev_data *mce_dev = (struct mcedev_data *)filp->private_data;
  struct rckcrb_data *crb_appl = (struct rckcrb_data *)mce_dev->specific_data;

  MPRINTK(MCEDBG_READWRITE, KERN_DEBUG "rckcrb_write: maj: %i, min: %i, (devnum: %i, function: %i)\n", 
	 major, minor, devnum, function);

  // check MIP level
  tmp = readl(RRA(RCK_TRNCT4_REG));
  miplvl = (tmp >> MIP_LVL_SHIFT) & MIP_LVL_MSK;

  if (miplvl & MIP_LVLERR_MSK) {
    // error state in MIP FIFO
    printk(KERN_ERR "rckcrb_write: Error in MIP FIFO: 0x%04x\n", miplvl);
    return -EIO;
  }

  // calculate available room in FIFO in BYtes
  room_avail = (MIP_FIFO_DEPTH - miplvl) * MIP_FIFO_WIDTH;

  // calculate number of packets minimum(available data in buffer, available room in MIP FIFO)
  if (count >= room_avail)
    packets_to_write = room_avail / MIP_BYTE_PACKET_SIZE;
  else
    packets_to_write = count / MIP_BYTE_PACKET_SIZE;

  if (packets_to_write <= 0) {
     MPRINTK((MCEDBG_READWRITE | MCEDBG_DMA), 
	     KERN_DEBUG "rckcrb_write: No packet to write (room_avail: %i, count: %i)\n", room_avail, (int)count);
    return packets_to_write;
  }

  MPRINTK((MCEDBG_READWRITE | MCEDBG_DMA), 
	  KERN_DEBUG "rckcrb_write: miplvl: 0x%04x, room_avail: %i Bytes, count: %zu Bytes, packets_to_write: %i\n", 
	  miplvl, room_avail, count, packets_to_write);

  // copy 256K - MIP lvl packets to rx_dma_buffer
  retval = copy_from_user(crb_appl->dma_rx_kernelptr, buf, packets_to_write * MIP_BYTE_PACKET_SIZE);
  if (retval != 0) {
    printk(KERN_ERR "rckcrb_write: Error: copy_from_user() has %i remaining Bytes\n", retval);
    return -EFAULT;
  }

  // write to device with write_dma
  if (!usePioOnly)  
    left1 = rckcrb_write_dma_w(crb_appl, packets_to_write * MIP_WORD_PACKET_SIZE, 0);
  else
    left1 = rckcrb_write_pio_w(crb_appl, packets_to_write * MIP_WORD_PACKET_SIZE, 0);
  
  if (left1 != 0) {
    reploop = 2;
    left2 = left1;
    do {
      MPRINTK((MCEDBG_READWRITE | MCEDBG_DMA),
	      KERN_DEBUG "rckcrb_write: Leftover after DMA: %i, writing in %i.Try\n", left1, reploop);
      if (!usePioForRest)
	left2 = rckcrb_write_dma_w(crb_appl, left2, (packets_to_write * MIP_WORD_PACKET_SIZE) - left2);
      else
	left2 = rckcrb_write_pio_w(crb_appl, left2, (packets_to_write * MIP_WORD_PACKET_SIZE) - left2);
      if (left2 != 0) {
	MPRINTK((MCEDBG_READWRITE | MCEDBG_DMA), 
		"rckcrb_write: Leftover after %i.Try: %i, should not happen!\n", reploop, left2);
      }
      reploop++;
    } while ((reploop <= 4) && (left2 != 0));
  }

  if (left2 != 0) {
    printk(KERN_ERR "rckcrb_write: Error: Could not write all data after %i repetitions, leftover: %i\n", reploop, left2);
    return -EFAULT;
  }

  // increment f_pos by amount written
  *f_pos += packets_to_write * MIP_BYTE_PACKET_SIZE;

  // return count
  return packets_to_write * MIP_BYTE_PACKET_SIZE;
}



/*
 * mmap - connection between physical MMIO and virtual address of userspace
 */

int rckcrb_mmap(struct file *filp, struct vm_area_struct *vma) {
  int minor = iminor(filp->f_dentry->d_inode);
  int major = imajor(filp->f_dentry->d_inode);
  int devnum = MINOR_DEVNUM(minor);
  int function = MINOR_FUNCTION(minor);
  unsigned long psize=0, physical=0; // base=0,

  if (function < MAXNUM_RCKCRB_FUNCTIONS) {
    struct mcedev_data *mce_dev = (struct mcedev_data *)filp->private_data;
    struct rckcrb_data *crb_appl = (struct rckcrb_data *)mce_dev->specific_data;

    if (crb_appl->func2bar[function].size > 0) {
      unsigned long off = vma->vm_pgoff << PAGE_SHIFT;  			// offset
      unsigned long vsize = vma->vm_end - vma->vm_start;			// requested virt. size

      psize = crb_appl->func2bar[function].size;
      physical = mce_dev->bar[crb_appl->func2bar[function].bar].start;		// bar physical start
   
      MPRINTK(MCEDBG_MMAP, KERN_DEBUG "rckcrb_mmap: device 0x%08lx (%d:%d:(%d:%d),\n",
	     (unsigned long)filp->f_dentry->d_inode->i_rdev, major, minor, devnum, function);
      MPRINTK(MCEDBG_MMAP, KERN_DEBUG " pstart: 0x%08lx, poff: 0x%08lx, psize: 0x%08lx\n",
	     physical, off, psize);
      MPRINTK(MCEDBG_MMAP, KERN_DEBUG " vstart: 0x%08lx, vsize: 0x%08lx\n",
	     vma->vm_start, vsize);

      if (vsize > (psize - off))
	return -EINVAL;

      if (io_remap_pfn_range(vma, vma->vm_start, (physical + off) >> PAGE_SHIFT, vsize, vma->vm_page_prot)) {
	printk(KERN_ERR "rckcrb_mmap: EAGAIN\n");
	return -EAGAIN;
      }
    
      return 0;
    }
  }

  return -ENODEV;
}



/*
 * Initialization function called by mcedev_probe() from mcedev_main.c
 */

int rckcrb_init(struct mcedev_data *mcedev, int dev_found, int major) {
  int i, err, devno, irq; 

  // Initialize data structure for application specific interface
  struct rckcrb_data *crb_appl = kmalloc(sizeof(struct rckcrb_data), GFP_KERNEL);

  printk(KERN_INFO "rckcrb_init: %s (%i)\n", RCKCRBIDSTRING, mcedev_debug);

  if (!crb_appl) {
    printk(KERN_ERR "Failed to kmalloc memory for rckcrb data\n");
    return -ENOMEM;
  }

  if (disableASPM) {
    rckcrb_disable_ASPM(mcedev);
  }


  memset(crb_appl, 0, sizeof(struct rckcrb_data));

  mcedev->specific_data = crb_appl;

  mcedev->mcedev_type = MCEDEV_TYPE_RCK_CRB;
  mcedev_set_resources(mcedev, mcedev->dev);
  mcedev_print_resources(mcedev);
  mcedev->cleanup = rckcrb_cleanup;

  crb_appl->func2bar = &func2barXilinx[0];
  
  // map memory of BAR0 for device register programming by driver
  if ((err = map_to_kernelmem(mcedev->bar[0].start, mcedev->bar[0].size, &crb_appl->bar0_kernelptr, "bar 0"))) {
    printk(KERN_ERR "rckcrb_init: Error: Failed to ioremap BAR0 region, error: %i\n", err);
    kfree(crb_appl);
    return err;
  }
  
  // printout FPGA register information
  rckcrb_init_bitfield_test(crb_appl);

  // Set Maximum TLP Size based on configured values in FPGA 
  if (tlpsmax == 0) {
    tlpsmax = rckcrb_set_tlpsmax(crb_appl);
    if (tlpsmax < 0) {
      kfree(crb_appl);
      return -ENXIO;
    }
  }

  // Allocate buffer memory for DMA
  err = rckcrb_alloc_dmabuffers(crb_appl, mcedev, 1024 * 256);
  if (err != 0) {
    printk(KERN_ERR "rckcrb_init: Error: Failed to allocate DMA buffers, error: %i\n", err);
    kfree(crb_appl);
    return err;
  }

  rckcrb_reset_scemi(crb_appl);

  irq =  mcedev->dev->irq;
  MPRINTK((MCEDBG_INIT | MCEDBG_INTERRUPT), KERN_DEBUG "Interrupt pre MSI: %i\n", irq);

  err = pci_enable_msi(mcedev->dev);
  if (err != 0) {
    printk(KERN_ERR "rckcrb_init: Error: Failed to switch into MSI interrupt mode, error: %i\n", err);
    kfree(crb_appl);
    return err;
  }
  else {
    irq =  mcedev->dev->irq;
    MPRINTK((MCEDBG_INIT | MCEDBG_INTERRUPT), KERN_DEBUG "Enable MSI returned %i, using IRQ %i\n", err, irq);
  }
  
  crb_appl->irq_num = irq;

  err = request_irq(irq, rckcrb_isr, IRQF_SHARED | IRQF_DISABLED, 
		    "crbdev", (void *) crb_appl); 

  if (err != 0) {
    printk(KERN_ERR "rckcrb_init: Error: request_irq() failed, error: %i\n", err);
    kfree(crb_appl);
    return err;
  }

  // Set interrupt usage to 1
  crb_appl->int_usage = 1;

  // allocate device nodes
  for (i = 0; i < MAXNUM_RCKCRB_FUNCTIONS; i++) {
    if (crb_appl->func2bar[i].size > 0) {
      // check assumed with available size
      if (crb_appl->func2bar[i].size > mcedev->bar[crb_appl->func2bar[i].bar].size) {
	printk(KERN_ERR "rckcrb_init: Error: Driver is assuming size of %ld in BAR %d, but it only provides %ld Bytes.\n",
	       crb_appl->func2bar[i].size, crb_appl->func2bar[i].bar, mcedev->bar[crb_appl->func2bar[i].bar].size);
        kfree(crb_appl);
	return -ENOMEM;
      }
      // add device to kernel
      devno = MKDEV(major, MKMINOR(dev_found, i));
      if ((err = allocate_device_node(devno, 1, &rckcrb_fops[i], &crb_appl->crb_function[i].cdev))) {
        kfree(crb_appl);
	return err;
      }
      crb_appl->crb_function[i].devno = devno;
      crb_appl->crb_function[i].fops = &rckcrb_fops[i];
      crb_appl->crb_function[i].use_count = 0;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
      class_device_create(mcedev->mcedev_class, NULL, devno, &(mcedev->dev->dev), "rckcrb%drb%d", dev_found, i);
#else
      device_create(mcedev->mcedev_class, NULL, devno, &(mcedev->dev->dev), "rckcrb%drb%d", dev_found, i);
#endif
    }
  }

  MPRINTK(MCEDBG_INIT, KERN_DEBUG "rckcrb_init: ... done\n");

  return 1;
}



/*
 * bbappl_cleanup() gets called from mcedev_remove() in mcedev_main.c if the cleanup pointer of
 * the mcedev structure was set to point to this function.
 */

void rckcrb_cleanup(struct mcedev_data *mcedev) {

  struct rckcrb_data *crb_appl = (struct rckcrb_data*)mcedev->specific_data;
  int i;

  MPRINTK(MCEDBG_CLEANUP, KERN_DEBUG "rckcrb_cleanup: Releasing IO memory mappings for crb_appl\n");
  /*
  // TODO: really disable interrupts here
  if (bb_appl->QL_int_usage != 0)
    MPRINTK(0x40, KERN_DEBUG "bbappl: WARNING: QL_int_usage is not 0\n");
  */


  // unregister device files
  for (i = 0; i < MAXNUM_RCKCRB_FUNCTIONS; i++) {
    if (crb_appl->crb_function[i].use_count != 0) {	
      MPRINTK(0x40, KERN_DEBUG "function use count != 0: function %i, count %i\n",
	      i, crb_appl->crb_function[i].use_count);	
    }
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
    class_device_destroy(mcedev->mcedev_class, crb_appl->crb_function[i].devno);
#else
    device_destroy(mcedev->mcedev_class, crb_appl->crb_function[i].devno);
#endif
    cdev_del(&crb_appl->crb_function[i].cdev);
    crb_appl->crb_function[i].use_count = 0;
  }


  writel(0x01, RRA(XLX_DCSR1));				// Assert reset 
  writel(0x00, RRA(XLX_DCSR1));				// Deassert reset

  if (crb_appl->int_usage > 0)
    free_irq(crb_appl->irq_num, (void *) crb_appl); 

  pci_disable_msi(mcedev->dev);

  rckcrb_free_dmabuffers(crb_appl, mcedev);
  
  if (crb_appl->bar0_kernelptr != 0)
    iounmap(crb_appl->bar0_kernelptr);

  kfree(mcedev->specific_data);
  
  MPRINTK(MCEDBG_CLEANUP, KERN_DEBUG "rckcrb_cleanup: ... done\n");
}


MODULE_DESCRIPTION("Driver for Rockcreek Copperridge Application PCIe interface");
