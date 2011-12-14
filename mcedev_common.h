/********************************************************************************
 *
 *
 *
 *
 * $Date: 2009-06-11 14:51:39 +0200 (Thu, 11 Jun 2009) $
 * $Revision: 15400 $
 * $Author: tlehnig $
 * $HeadURL: https://subversion.jf.intel.com/ctg/mtl/mc/mcemu/software/trunk/linuxdrv-2.6/mcedev/mcedev_common.h $
 * $Id: mcedev_common.h 15400 2009-06-11 12:51:39Z tlehnig $
 *
 ********************************************************************************
 */


#ifndef MCEDEV_COMMON_H
#define MCEDEV_COMMON_H

#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>

#define MINOR_FUNCTIONBITS      (MINORBITS / 2)
#define MINOR_FUNCTIONMASK      ((1U << MINOR_FUNCTIONBITS) - 1)

#define MINOR_DEVNUM(dev)     ((unsigned int) ((dev) >> MINOR_FUNCTIONBITS))
#define MINOR_FUNCTION(dev)   ((unsigned int) ((dev) & MINOR_FUNCTIONMASK))
#define MKMINOR(num, func)    (((num) << MINOR_FUNCTIONBITS) | (func))


int mcedev_no_init(struct mcedev_data *mcedev, int dev_found, int major);
void mcedev_set_resources(struct mcedev_data *mcedev, struct pci_dev *dev);
void mcedev_print_resources(struct mcedev_data *mcedev);

int allocate_device_node(int nodeid, int nodenums, struct file_operations *myfops, struct cdev *cdev);
int map_to_kernelmem(unsigned long phys_addr, unsigned long size, void **target, char *desc);

void mcedev_probe_bars(struct pci_dev *dev);


static inline void iosetbit32(u32 bits, void *addr) {
  iowrite32(ioread32(addr) | bits, addr);
}


static inline void ioclearbit32(u32 bits, void *addr) {
  iowrite32(ioread32(addr) & (~bits), addr);
}


static inline u32 iogetbit32(u32 bits, void *addr) {
  return ioread32(addr) & bits;
}



#endif // ifndef MCEDEV_COMMON_H
