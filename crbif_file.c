/***************************************************************************
 *
 * $Date: 2010-08-24 16:35:55 +0200 (Tue, 24 Aug 2010) $
 * $Revision: 16585 $
 * $Author: jbrummer $
 * $HeadURL: https://subversion.jf.intel.com/ctg/mtl/mc/mcemu/software/trunk/linuxdrv-2.6/mcedev/crbif_file.c $
 * $Id: crbif_file.c 16585 2010-08-24 14:35:55Z jbrummer $
 *
 ***************************************************************************
 */


#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/uaccess.h>        /* copy_to/from_user() */
#include <linux/compat.h>
#include <linux/poll.h>

#include "mcedev.h"
#include "mcedev_common.h"      /* MINOR_* macros (requires mcedev.h) */
#include "crbdev.h"             /* CopperRidge specifics */



extern struct mcedev_data* mcemu_device_table[];

/* mcedev_debug controls MPRINTK debug output messages (see mcedev.h).
 * Used values are  MCEDBG_OPEN       => 0x0004 (   4)
 *                  MCEDBG_RELEASE    => 0x0008 (   8)
 *                  MCEDBG_READWRITE  => 0x0010 (  16)
 *                  MCEDBG_MMAP       => 0x1000 (4096)
 */
extern int mcedev_debug;
extern unsigned dmaBufferSize;
extern int scemiBufferSize;
extern int maxTransId;

int enableMmap = 0;
module_param(enableMmap, int, 0644);
MODULE_PARM_DESC(enableMmap, "Enable direct access to CRB device via mmap()");

int oldBehaviour = 0;
module_param(oldBehaviour, int, 0644);
MODULE_PARM_DESC(oldBehaviour, "Use old behaviour: return 0 on read error");



/*
 * Implementation of the open(), release() and mmap() functions as required
 * for the device nodes of MCEMU application PCIe interface.
 */
int crbif_open(struct inode* inode, struct file* filp) {
  int                 minor     = iminor(inode);
  int                 major     = imajor(inode);
  int                 devnum    = MINOR_DEVNUM(minor);
  int                 function  = MINOR_FUNCTION(minor);
  unsigned long       flags;
  struct rckcrb_data* crb_appl;

  MPRINTK(MCEDBG_OPEN, KERN_DEBUG 
          "crbif_open: maj: %i, min: %i (devnum: %i, func: %i)\n",
          major, minor, devnum, function);

  
  if (function >= MAXNUM_RCKCRB_FUNCTIONS) {
    printk(KERN_ERR "crbif_open: Invalid function: %i\n", function);
    return -ENODEV;
  }

  if (mcemu_device_table[devnum] == NULL) {
    printk(KERN_ERR "crbif_open: mcemu_devices[%i] is NULL pointer\n", devnum);
    return -ENXIO; 
  }

  
  /* Set private data of filepointer to point to mcemu_devices structure 
   * for this device instance
   */
  filp->private_data = (void*)mcemu_device_table[devnum];

  /* Get the CopperRidge specific data structure holding addresses, 
   * interrupt numbers, etc.
   */
  crb_appl = ((struct mcedev_data*)filp->private_data)->specific_data;

  if (crb_appl == NULL) {
    printk(KERN_ERR "crbif_open: crb_appl specific data not set\n");
    return -ENXIO; 
  }

  
  /* Make sure that the device is only opened once when initializing the
   * user interface
   */
  spin_lock_irqsave(&(crb_appl->mip_fifo.lock), flags);

  if (crb_appl->crb_function[function].use_count != 0) {
    printk(KERN_ERR "crbif_open: use_count for function %i != 0 (%i)\n", 
                    function, crb_appl->crb_function[function].use_count);
    spin_unlock_irqrestore(&(crb_appl->mip_fifo.lock), flags); 
    return -EPERM;
  }
  crb_appl->crb_function[function].use_count++;
  
  /* Reset the MIP FIFO */
  crb_appl->mip_fifo.readPointer  = 0;
  crb_appl->mip_fifo.writePointer = 0;
  crb_appl->mip_fifo.level        = 0;
  
  spin_unlock_irqrestore(&(crb_appl->mip_fifo.lock), flags);
  
  /* Also reset the MOP FIFO which is protected by its own spinlock */
  spin_lock_irqsave(&(crb_appl->mop_fifo.lock), flags);  
  crb_appl->mop_fifo.readPointer  = 0;
  crb_appl->mop_fifo.writePointer = 0;
  crb_appl->mop_fifo.level        = 0;
  spin_unlock_irqrestore(&(crb_appl->mop_fifo.lock), flags);
  

  MPRINTK(MCEDBG_OPEN, KERN_DEBUG 
          "crbif_open: usage count for this function (%i): %i\n", 
          function, crb_appl->crb_function[function].use_count);

  return 0;
}


/*
 * release is called through close() on the file descriptor.
 */
int crbif_release(struct inode* inode, struct file* filp) {
  int minor     = iminor(inode);
  int major     = imajor(inode);
  int devnum    = MINOR_DEVNUM(minor);
  int function  = MINOR_FUNCTION(minor);
  int usage     = 0;
  int i;
  
  struct rckcrb_data* crb_appl;

  MPRINTK(MCEDBG_RELEASE, KERN_DEBUG 
          "crbif_release: maj: %i, min: %i (devnum: %i, func: %i)\n",
          major, minor, devnum, function);

  
  /* Get the CopperRidge specific data structure holding addresses, 
   * interrupt numbers, etc.
   */
  crb_appl = ((struct mcedev_data*)filp->private_data)->specific_data;

  if (crb_appl == NULL) {
    printk(KERN_ERR "crbif_release: crb_appl specific data not set\n");
    return -ENXIO; 
  }


  /* There is 1 less user of of this function */  
  if (crb_appl->crb_function[function].use_count > 0) {
    crb_appl->crb_function[function].use_count--;
  }
  else {
    MPRINTK(MCEDBG_RELEASE, KERN_DEBUG 
            "crbif_release: WARNING: use count is already at 0!\n");
  }

  
  /* Print usage info for debug purposes */
  MPRINTK(MCEDBG_RELEASE, KERN_DEBUG 
          "crbif_release: usage count for function %i: %i\n",
          function, crb_appl->crb_function[function].use_count);

  /* Count the usage across all functions */
  for (i=0; i<MAXNUM_RCKCRB_FUNCTIONS; i++) {
    usage += crb_appl->crb_function[i].use_count;
  }
  MPRINTK(MCEDBG_RELEASE, KERN_DEBUG 
          "crbif_release: total usage count at %i\n", usage);

  return 0;
}



/*
 * read - returns data from our local MOP buffer
 */
ssize_t crbif_read(struct file* filp, char __user* buf, 
                   size_t count, loff_t* f_pos) 
{
  struct mcedev_data* mce_dev   = (struct mcedev_data*)filp->private_data;
  struct rckcrb_data* crb_appl  = (struct rckcrb_data*)mce_dev->specific_data;
  unsigned      packetCount     = 0;
  unsigned      totalBytes      = 0;
  int           result;

  if (crb_appl->mop_fifo.level < 0 ) {
    MPRINTK(MCEDBG_READWRITE, KERN_DEBUG "Warning: MOP fifo level < 0, (is %d)\n", crb_appl->mop_fifo.level);
    return -EIO;
  }

  if (crb_appl->mop_fifo.level == 0 ) {
    //MPRINTK(MCEDBG_READWRITE, KERN_DEBUG "MOP fifo level == 0, (is %d), returning -EAGAIN\n", crb_appl->mop_fifo.level);
    if (!oldBehaviour) {
      return -EAGAIN;
    }
  }
  

  //if (crb_appl->mop_fifo.level < count ) {
   // MPRINTK(MCEDBG_READWRITE, KERN_DEBUG "Warning: MOP fifo level <= count, (is %d < %ld)\n", crb_appl->mop_fifo.level, count);
    //return -EIO;
  //}
  MPRINTK(MCEDBG_READWRITE, KERN_DEBUG "MOP level: %d, Count: %ld\n", crb_appl->mop_fifo.level, count);
  
  
  /* First calculate how much data we can actually return
   * Note that we always return entire packets
   */
  if (count > crb_appl->mop_fifo.level) 
    packetCount = crb_appl->mop_fifo.level / MOP_BYTE_PACKET_SIZE;
  else
    packetCount = count / MOP_BYTE_PACKET_SIZE;
  totalBytes = packetCount * MOP_BYTE_PACKET_SIZE;

  
  if (!totalBytes) {
    return 0;
  }
  MPRINTK(MCEDBG_READWRITE, KERN_DEBUG 
          "crbif_read: MOP: %i bytes, requested: %i bytes (%i packets)\n",
          crb_appl->mop_fifo.level, (int)count, packetCount);
  
  
  result = crbif_fifo_read(&crb_appl->mop_fifo, buf, totalBytes, USERSPACE);
  if (result) {
    printk(KERN_ERR "crbif_read: FIFO error detected (%i)\n", result);
    return -EFAULT;
  } else {
  MPRINTK(MCEDBG_READWRITE, KERN_DEBUG 
          "crbif_read: got %d. %i bytes wanted\n", result, totalBytes );
  }

  /* Increment f_pos by the amount written */
  *f_pos += totalBytes;
  
  return totalBytes;
}



/*
 * write - store data in our local MIP buffer
 */
ssize_t crbif_write(struct file* filp, const char __user* buf, 
                    size_t count, loff_t* f_pos)
{
  struct mcedev_data* mce_dev   = (struct mcedev_data*)filp->private_data;
  struct rckcrb_data* crb_appl  = (struct rckcrb_data*)mce_dev->specific_data;
  unsigned      totalBytes      = 0;
  unsigned      packetCount     = 0;
  unsigned      freeSpace;
  int           result;

  /* First calculate how much data we can actually store
   * Note that we always write entire packets
   */
  freeSpace = dmaBufferSize - crb_appl->mip_fifo.level;
  if (count > freeSpace) 
    packetCount = freeSpace / MIP_BYTE_PACKET_SIZE;
  else
    packetCount = count / MIP_BYTE_PACKET_SIZE;
  totalBytes = packetCount * MIP_BYTE_PACKET_SIZE;
    
    
  MPRINTK(MCEDBG_READWRITE, KERN_DEBUG 
          "crbif_write: MIP: %i free, requested: %i bytes (%i packets)\n",
          freeSpace, (int)count, packetCount);

  
  if (!packetCount) {
    MPRINTK(MCEDBG_READWRITE, KERN_INFO "crbif_write: No packets to write\n");
    return 0;
  }

  
  /* Copy the packet data from userspace into the MIP buffer */
  result = crbif_fifo_write(&crb_appl->mip_fifo, buf, totalBytes, USERSPACE);
  if (result) {
    printk(KERN_ERR "crbif_write: FIFO error detected (%i)\n", result);
    return -EFAULT;
  }

  /* Increment f_pos by the amount written */
  *f_pos += totalBytes;

  return totalBytes;
}



/*
 * mmap - connection between physical MMIO and virtual address of userspace
 */
int crbif_mmap(struct file* filp, struct vm_area_struct* vma) {
  struct mcedev_data* mce_dev   = (struct mcedev_data*)filp->private_data;
  struct rckcrb_data* crb_appl  = (struct rckcrb_data*)mce_dev->specific_data;
  int           minor           = iminor(filp->f_dentry->d_inode);
  int           major           = imajor(filp->f_dentry->d_inode);
  int           devnum          = MINOR_DEVNUM(minor);
  int           function        = MINOR_FUNCTION(minor);
  unsigned long offset;
  unsigned long vsize;
  unsigned long psize;
  unsigned long physical;
  int           errorCode;
  
  MPRINTK(MCEDBG_MMAP, KERN_DEBUG 
          "crbif_mmap: device 0x%08lx (%d, %d (%d:%d))\n",
          (unsigned long)filp->f_dentry->d_inode->i_rdev, 
          major, minor, devnum, function);

  
  if (function >= MAXNUM_RCKCRB_FUNCTIONS)    return -ENODEV;
  if (crb_appl->func2bar[function].size <= 0) return -ENODEV;
  
  
  if (enableMmap == 0) {
    MPRINTK(MCEDBG_MMAP, KERN_DEBUG "crbif_mmap: operation not enabled\n");
    return -EAGAIN;
  }
  
  
  offset    = vma->vm_pgoff << PAGE_SHIFT;
  vsize     = vma->vm_end - vma->vm_start;  /* requested virtual size */
  psize     = crb_appl->func2bar[function].size;
  physical  = mce_dev->bar[crb_appl->func2bar[function].bar].start;


  MPRINTK(MCEDBG_MMAP, KERN_DEBUG 
          " pstart: 0x%08lx, poff: 0x%08lx, psize: 0x%08lx\n",
          physical, offset, psize);
  MPRINTK(MCEDBG_MMAP, KERN_DEBUG 
          " vstart: 0x%08lx, vsize: 0x%08lx\n",
          vma->vm_start, vsize);


  if (vsize > (psize-offset)) return -EINVAL;
  
  errorCode = io_remap_pfn_range(vma, vma->vm_start, 
                                 (physical+offset) >> PAGE_SHIFT, 
                                 vsize, vma->vm_page_prot);
  if (errorCode) {
    printk(KERN_ERR "crbif_mmap: io_remap() failed with %i\n", errorCode);
    return -EAGAIN;
  }

  return 0;
}

/**
 * \brief internal crbif ioctl function, which will be called by crbif_unlocked_ioctl and crbif_compat_ioctl
 * \param psFile file structure
 * \param nCommand ioctl command
 * \param nValue value
 * \return error code
 */
long crbif_intern_ioctl(struct file* psFile, unsigned int nCommand, unsigned long nValue) {
  struct mcedev_data* mce_dev   = (struct mcedev_data*)psFile->private_data;
  struct rckcrb_data* crb_appl  = (struct rckcrb_data*)mce_dev->specific_data;
  unsigned long flags;

  MPRINTK(MCEDBG_READWRITE, KERN_DEBUG "crbif_intern_ioctl(): command 0x%x, value 0x%lx\n", nCommand, nValue);
  switch (nCommand) {
    case CRBIF_SET_SUBNET_SIZE:
      setSubnetSize(nValue);
      break;
    case CRBIF_SET_TX_BASE_ADDRESS:
      nValue <<= 2;
      setTxBaseAddress(nValue);
      break;
    case CRBIF_RESET:
      /* Reset the MIP FIFO */
      spin_lock_irqsave(&(crb_appl->mip_fifo.lock), flags);
      crb_appl->mip_fifo.readPointer  = 0;
      crb_appl->mip_fifo.writePointer = 0;
      crb_appl->mip_fifo.level        = 0;
      spin_unlock_irqrestore(&(crb_appl->mip_fifo.lock), flags);

      /* Also reset the MOP FIFO which is protected by its own spinlock */
      spin_lock_irqsave(&(crb_appl->mop_fifo.lock), flags);
      crb_appl->mop_fifo.readPointer  = 0;
      crb_appl->mop_fifo.writePointer = 0;
      crb_appl->mop_fifo.level        = 0;
      spin_unlock_irqrestore(&(crb_appl->mop_fifo.lock), flags);

      crb_appl->dma_state = 0;
      crb_appl->dma_doTx = 0;
      crb_appl->dma_doFilter = 0;

      rckcrb_reset_scemi(crb_appl);
      break;
    case CRBIF_GET_BUFFER_SIZE: {
      int nErr = 0;
      int nCopy = 0;
      MPRINTK(MCEDBG_READWRITE, KERN_DEBUG "get buffer size:\n" );
      if ( nValue == 0 ) {
        MPRINTK(MCEDBG_READWRITE, KERN_DEBUG "value is zero!\n" );
        return -EINVAL;
      }
      nErr = access_ok( VERIFY_WRITE, nValue, sizeof( long ) );
      if ( nErr != 1 ) {
        MPRINTK(MCEDBG_READWRITE, KERN_DEBUG "no write access!\n" );
        return -EFAULT;
      }
      MPRINTK(MCEDBG_READWRITE, KERN_DEBUG "from %d\n", MIP_FIFO_SIZE );
      nCopy = MIP_FIFO_SIZE;
      MPRINTK(MCEDBG_READWRITE, KERN_DEBUG "result is: %d\n", nCopy );
      if ( copy_to_user( ( long * ) nValue, &nCopy, sizeof( long ) ) ) {
        MPRINTK(MCEDBG_READWRITE, KERN_DEBUG "copy_to_user failed\n" );
        return -EFAULT;
      }
      break;
    }
    case CRBIF_SET_MAX_TRANSID: {
      MPRINTK(MCEDBG_READWRITE, KERN_DEBUG "set max transid size to %ld\n", nValue );
      if ( nValue == 0 ) {
        MPRINTK(MCEDBG_READWRITE, KERN_DEBUG "value is zero!\n" );
        return -EINVAL;
      }
      maxTransId = nValue;
      break;
    }
  }

  return 0;
}

unsigned int crbif_poll(struct file* psFile, poll_table* psWait) {
  struct mcedev_data* mce_dev   = (struct mcedev_data*)psFile->private_data;
  struct rckcrb_data* crb_appl  = (struct rckcrb_data*)mce_dev->specific_data;

  return crb_appl->mop_fifo.level <= 0 ? 0 : DEFAULT_POLLMASK;
}

/**
 * \brief crbif ioctl function
 * \param psFile file structure
 * \param nCommand ioctl command
 * \param nValue value
 * \return error code
 */
long crbif_unlocked_ioctl(struct file* psFile, unsigned int nCommand, unsigned long nValue) {
  MPRINTK(MCEDBG_READWRITE, KERN_DEBUG "crbif_unlocked_ioctl(): command %x, value 0x%lx\n", nCommand, nValue);
  return crbif_intern_ioctl(psFile, nCommand, nValue);
}

#ifdef CONFIG_COMPAT
/**
 * \brief crbif compatible ioctl
 * \param file file structure
 * \param command ioctl command
 * \param value value
 * \return error code
 */
long crbif_compat_ioctl(struct file* psFile, unsigned int nCommand, unsigned long nValue) {
  MPRINTK(MCEDBG_READWRITE, KERN_DEBUG "crbif_compat_ioctl(): command %x, value 0x%lx\n", nCommand, nValue);
  return crbif_intern_ioctl(psFile, nCommand, nValue);
}
#endif
