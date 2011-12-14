/********************************************************************************
 *
 *
 *
 *
 * $Date: 2010-08-24 16:35:55 +0200 (Tue, 24 Aug 2010) $
 * $Revision: 16585 $
 * $Author: jbrummer $
 * $HeadURL: https://subversion.jf.intel.com/ctg/mtl/mc/mcemu/software/trunk/linuxdrv-2.6/mcedev/crbdev.h $
 * $Id: crbdev.h 16585 2010-08-24 14:35:55Z jbrummer $
 *
 ********************************************************************************
 */


#ifndef CRBDEV_H
#define CRBDEV_H

#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>

#include <linux/in.h>
#include <linux/netdevice.h>    /* struct device, and other headers */
#include <linux/etherdevice.h>  /* eth_type_trans */
#include <linux/skbuff.h>
#include <linux/compat.h>

#include "mcedev.h"
#include "mcedev_build.h"


// Register defines - GRB

#define RCK_ID0_REG		0x0300
#define RCK_ID0_BITSID		0xffffffff

#define RCK_ID1_REG		0x0304
#define RCK_ID1_GRBTEST		0xffffffff

#define RCK_ID2_REG		0x0308
#define RCK_ID2_GRBTEST		0xffffffff

#define RCK_TRNCT0_REG		0x030C
#define RCK_TRNCT0_BARWIN	0xffffffff

#define RCK_TRNCT1_REG		0x0310
#define RCK_TRNCT1_BARWIN	0xffffffff

#define RCK_TRNCT2_REG		0x314
#define RCK_TRNCT2_FOOWIN	0xffffffff

#define RCK_TRNCT3_REG		0x0318
#define RCK_TRNCT3_FOOWIN	0xffffffff

#define RCK_TRNCT4_REG		0x031C
#define RCK_TRNCT4_MOPFIFOLVL	0x00007fff
#define RCK_TRNCT4_MOPERROR	0x00008000
#define RCK_TRNCT4_MIPFIFOLVL	0x7fff0000
#define RCK_TRNCT4_MIPERROR	0x80000000

#define RCK_TRNCT5_REG		0x0320
#define RCK_TRNCT5_INIT		0x00000001
#define RCK_TRNCT5_TRNCTE	0x00000002

#define RCK_CONFIG_REG		0x0324
#define RCK_CONFIG_SOFTRESET	0x00000001
#define RCK_CONFIG_GEN_MEMCOMP	0x00000002
#define RCK_CONFIG_DROP_MEMCOMP	0x00000004
#define RCK_CONFIG_RCINITDONE	0x00000008

#define RCK_DEBUG_REG		0x0328
#define RCK_DEBUG_MIPDISABLERD	0x00000001
#define RCK_DEBUG_SCEMILOOPBACK	0x00000002
#define RCK_DEBUG_MIPRDTRIG     0x00000004

#define RCK_SCEMIDATA0_REG	0x032C
#define RCK_SCEMIDATA1_REG	0x0330

#define RCK_SCEMI_BUFFER_SIZE	0x03B4

#define RCK_RTRCntP0I 		(205 * 4)
#define RCK_RTRCntP0O 		(206 * 4)
#define RCK_RTRCntP1I 		(207 * 4)
#define RCK_RTRCntP1O 		(208 * 4)
#define RCK_RTRCntP2I 		(209 * 4)
#define RCK_RTRCntP2O 		(210 * 4)
#define RCK_RTRCntP3I 		(211 * 4)
#define RCK_RTRCntP3O 		(212 * 4)

#define XLX_DCSR1		0x0000	// Device Control Status Register
#define XLX_DDMACR		0x0004	// Device DMA Control Status Register
#define XLX_WDMATLPA		0x0008	// Write DMA TLP Address
#define XLX_WDMATLPS		0x000C	// Write DMA TLP Size
#define XLX_WDMATLPC		0x0010	// Write DMA TLP Count
#define XLX_WDMATLPP		0x0014	// Write DMA TLP Data Pattern
#define XLX_RDMATLPP		0x0018	// Read DMA TLP Expected Pattern
#define XLX_RDMATLPA		0x001C	// Read DMA TLP Address
#define XLX_RDMATLPS		0x0020	// Read DMA TLP Size
#define XLX_RDMATLPC		0x0024	// Read DMA TLP Count
#define XLX_WDMAPERF		0x0028	// Write DMA Performance
#define XLX_RDMAPERF		0x002C	// Read DMA Performance
#define XLX_RDMASTAT		0x0030	// Read DMA Status
#define XLX_NRDCOMP		0x0034	// Number of Rad Completion w/ data
#define XLX_RCOMPDSIZW		0x0038	// Read Completion Data Size
#define XLX_DLWSTAT		0x003C	// Device Link Width Status
#define XLX_DLTRSSTAT		0x0040	// Device Link Transaction Size Status
#define XLX_DMISCCONT		0x0044	// Device Miscellaneous Control


// Rck Register Address makro
#define RRA(x) crb_appl->bar0_kernelptr + x

// device functions
#define MAXNUM_RCKCRB_FUNCTIONS		1

// MIP / MIP FIFO size convenience functions

// width of MIP / MOP depth - to be read from register
#define MIP_FIFO_DEPTH_BITS		scemiBufferSize // mipFifoDepthBits 15 for 256KB, 13 for 64KB
#define MOP_FIFO_DEPTH_BITS		scemiBufferSize // mopFifoDepthBits 15 for 256KB, 13 for 64KB

// MIP/MOP FIFO depth (of 8Bytes)
#define MIP_FIFO_DEPTH			(1 << MIP_FIFO_DEPTH_BITS)
#define MOP_FIFO_DEPTH			(1 << MOP_FIFO_DEPTH_BITS)

// MIP / MOP data width 8 Bytes
#define MIP_FIFO_WIDTH 			8
#define MOP_FIFO_WIDTH 			8

// MIP/MOP Max FIFO Size in bytes
#define MIP_FIFO_SIZE			(MIP_FIFO_WIDTH * MIP_FIFO_DEPTH)
#define MOP_FIFO_SIZE			(MOP_FIFO_WIDTH * MOP_FIFO_DEPTH)


// MIP / MOP Masks to access FIFO levels in TRNCT4 correctly
#define MIP_LVL_MSK			((2 << MIP_FIFO_DEPTH_BITS) - 1)
#define MOP_LVL_MSK			((2 << MOP_FIFO_DEPTH_BITS) - 1)

#define MIP_LVLERR_MSK			(1 << MIP_FIFO_DEPTH_BITS)
#define MOP_LVLERR_MSK			(1 << MOP_FIFO_DEPTH_BITS)

#define MIP_LVL_SHIFT			16
#define MOP_LVL_SHIFT			0


// Packet Size 48 Bytes
#define MIP_BYTE_PACKET_SIZE 		48
#define MOP_BYTE_PACKET_SIZE 		48

// Packet Size 48 Bytes in words
#define MIP_WORD_PACKET_SIZE 		(MIP_BYTE_PACKET_SIZE / 4)
#define MOP_WORD_PACKET_SIZE 		(MOP_BYTE_PACKET_SIZE / 4)

// Max number of loops to wait for IRQ from DMA
#define RCK_MAXWAIT_LOOPS		1000000


/* Number of cores on a Rock Creek chip */
#define RCK_CORECOUNT       48

/** Ioctl function calls */
#define CRBIF_SET_SUBNET_SIZE		_IOWR(MCEDEV_MAJOR, 1, unsigned long)
#define CRBIF_SET_TX_BASE_ADDRESS	_IOWR(MCEDEV_MAJOR, 2, unsigned long)
#define CRBIF_RESET			_IOWR(MCEDEV_MAJOR, 3, unsigned long)
#define CRBIF_GET_BUFFER_SIZE			_IOR(MCEDEV_MAJOR, 4, unsigned long)
#define CRBIF_SET_MAX_TRANSID			_IOWR(MCEDEV_MAJOR, 5, unsigned long)


struct crbappl_function {
  unsigned long devno;
  struct cdev cdev;
  struct file_operations *fops;
  int use_count;
};


struct crbappl_func2bar {
  int bar;
  unsigned long size;
};



/* Data structure and access functions for the FIFOs between the DMA engine
 * and the kernel/user space.
 */
#define KERNELSPACE   0
#define USERSPACE     1

struct dma_fifo {
  spinlock_t  lock;
  void*       data;
  int    level;
  unsigned    readPointer;
  unsigned    writePointer;
};

int crbif_fifo_read(struct dma_fifo* fifo, void* destination, unsigned size, 
                    int userSpace);
int crbif_fifo_write(struct dma_fifo* fifo, const void* source, unsigned size, 
                     int userSpace);



struct rckcrb_data {
  unsigned long major, minor; //, numfunctions;
  struct crbappl_function crb_function[MAXNUM_RCKCRB_FUNCTIONS];
  void *bar0_kernelptr;
  //  int user_pid;
  //  struct task_struct *task;
  unsigned int int_usage;
  unsigned int irq_num;
  //  unsigned long mem_physical_start;
  //  unsigned long mem_physical_size;
  const struct crbappl_func2bar *func2bar;
  size_t dmabuf_size;
  void *dma_rx_kernelptr, *dma_tx_kernelptr;
  dma_addr_t dma_rx_deviceptr, dma_tx_deviceptr;

  struct work_struct crbif_work;
  struct pci_dev *dev;

  struct semaphore app_sema;

  /* Additional elements for the crbif kernel module
   * They are appended to the existing rckcrb_data structure so the
   * existing CRB functions can be reused
   */
#if defined (MCEDEV_MODULE_CRBIF)
  volatile unsigned          dma_state;
  unsigned          dma_doTx;
  unsigned          dma_doFilter;
  unsigned          dma_txOffset;
  unsigned          dma_txSize;
  unsigned          dma_rxOffset;
  unsigned          dma_rxSize;

  struct dma_fifo   mip_fifo;
  struct dma_fifo   mop_fifo;
  struct dma_fifo   net_fifo;
#endif
};



/*
 * Private data of the network device
 */
struct crbnet_priv {
  struct net_device_stats     stats;
  spinlock_t                  lock;
  struct sk_buff*             rxInFlight[RCK_CORECOUNT];
  int                         nextTxSlot[RCK_CORECOUNT];
  unsigned                    txSlotBusy[RCK_CORECOUNT];
  unsigned long               lastTx[RCK_CORECOUNT];
  struct rckcrb_data*         crb_appl;
};



int rckcrb_init(struct mcedev_data *mcedev, int dev_found, int major);
void rckcrb_cleanup(struct mcedev_data *mcedev);

int crbif_init(struct mcedev_data *mcedev, int dev_found, int major);
void crbif_cleanup(struct mcedev_data *mcedev);

void rckcrb_init_bitfield_test(struct rckcrb_data *crb_appl);
void rckcrb_disable_ASPM(struct mcedev_data *mcedev);
int rckcrb_set_tlpsmax(struct rckcrb_data *crb_appl);

int rckcrb_alloc_dmabuffers(struct rckcrb_data *crb_appl, struct mcedev_data *mcedev, size_t dmabuf_size);
void rckcrb_free_dmabuffers(struct rckcrb_data *crb_appl, struct mcedev_data *mcedev);

int rckcrb_write_dma_w(struct rckcrb_data *crb_appl, unsigned int numwords, unsigned int offsetwords);
int rckcrb_write_pio_w(struct rckcrb_data *crb_appl, unsigned int numwords, unsigned int offsetwords);

int rckcrb_read_dma_w(struct rckcrb_data *crb_appl, unsigned int numwords, unsigned int offsetwords);
int rckcrb_read_pio_w(struct rckcrb_data *crb_appl, unsigned int numwords, unsigned int offsetwords);

void rckcrb_reset_scemi(struct rckcrb_data *crb_appl);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
irqreturn_t rckcrb_isr(int irq, void *dev_id, struct pt_regs *regs);
#else
irqreturn_t rckcrb_isr(int irq, void *dev_id);
#endif

void setSubnetSize(unsigned long nSize);
void setTxBaseAddress(unsigned long nAddress);

#endif // ifndef CRBDEV_H

