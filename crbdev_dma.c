/********************************************************************************
 *
 *
 *
 *
 * $Date: 2010-08-24 16:35:55 +0200 (Tue, 24 Aug 2010) $
 * $Revision: 16585 $
 * $Author: jbrummer $
 * $HeadURL: https://subversion.jf.intel.com/ctg/mtl/mc/mcemu/software/trunk/linuxdrv-2.6/mcedev/crbdev_dma.c $
 * $Id: crbdev_dma.c 16585 2010-08-24 14:35:55Z jbrummer $
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
extern int scemiBufferSize;

volatile int wait_for_dma = 0;

// Module parameters
int tlps64 = 2048;
int tlps32 = 1024;
int tlps16 = 512;
int tlps8  = 64;
int tlpsfixed = 0;
int tlpsmax = 0;


module_param(tlps64, int, 0644);
MODULE_PARM_DESC(tlps64, "Size in words (32bit) when TLP size 64 starts to be used for DMA");

module_param(tlps32, int, 0644);
MODULE_PARM_DESC(tlps32, "Size in words (32bit) when TLP size 32 starts to be used for DMA");

module_param(tlps16, int, 0644);
MODULE_PARM_DESC(tlps16, "Size in words (32bit) when TLP size 16 starts to be used for DMA");

module_param(tlps8, int, 0644);
MODULE_PARM_DESC(tlps8, "Size in words (32bit) when TLP size 8 starts to be used for DMA");

module_param(tlpsfixed, int, 0644);
MODULE_PARM_DESC(tlpsfixed, "If != 0 use only this fixed TLP size (test and debug only!)");

module_param(tlpsmax, int, 0644);
MODULE_PARM_DESC(tlpsmax, "Maximum TLP size allowable to be used by driver. Set during init, can be overridden");





/*
 * Print FPGA DMA state before write
 */

void rckcrb_write_stat_bef(struct rckcrb_data *crb_appl) {
  u32 tmp;
  if (mcedev_debug & MCEDBG_DMA) {
    tmp = readl(RRA(RCK_TRNCT4_REG));
    printk(KERN_DEBUG "before DMA: MIP FIFO lvl: 0x%04x, MOP FIFO lvl:     0x%04x\n", 
	   (tmp >> MIP_LVL_SHIFT) & MIP_LVL_MSK, tmp & MOP_LVL_MSK);
    tmp = readl(RRA(XLX_DDMACR));
    printk(KERN_DEBUG "Status before DMA:            0x%08x\n", tmp);
    tmp = readl(RRA(XLX_RDMATLPS));
    printk(KERN_DEBUG "Read DMA TLP Size:            0x%08x\n", tmp);
    tmp = readl(RRA(XLX_RDMATLPC));
    printk(KERN_DEBUG "Read DMA TLP Count:           0x%08x\n", tmp);
  }
}



/*
 * Print FPGA DMA state after write
 */

void rckcrb_write_stat_after(struct rckcrb_data *crb_appl, u64 tsc1, u64 tsc2, unsigned int i) {
  u32 tmp;
  if (mcedev_debug & MCEDBG_DMA) {
    tmp = readl(RRA(XLX_DDMACR));
    printk(KERN_DEBUG "Status after DMA:             0x%08x\n", tmp);
    tmp = readl(RRA(XLX_RDMAPERF));
    printk(KERN_DEBUG "Read DMA Performance:         0x%08x\n", tmp);
    tmp = readl(RRA(XLX_RDMASTAT));
    printk(KERN_DEBUG "Read DMA Status:              0x%08x\n", tmp);
    tmp = readl(RRA(XLX_NRDCOMP));
    printk(KERN_DEBUG "Number Read completions:      0x%08x\n", tmp);
    tmp = readl(RRA(XLX_RCOMPDSIZW));
    printk(KERN_DEBUG "Read Completion Data Size:    0x%08x\n", tmp);
    tmp = readl(RRA(RCK_TRNCT4_REG));
    printk(KERN_DEBUG "after DMA: MIP FIFO lvl: 0x%04x, MOP FIFO lvl: 0x%04x\n", 
	   (tmp >> MIP_LVL_SHIFT) & MIP_LVL_MSK, tmp & MOP_LVL_MSK);
    printk(KERN_DEBUG "Duration: tsc2 - tsc1 = %lli (i: %i)\n", tsc2 - tsc1, i);
  }
}



/*
 * Print FPGA DMA state before read
 */

void rckcrb_read_stat_bef(struct rckcrb_data *crb_appl) {
  u32 tmp;
  if (mcedev_debug & MCEDBG_DMA) {
    tmp = readl(RRA(RCK_TRNCT4_REG));
    printk(KERN_DEBUG "before DMA: MIP FIFO lvl: 0x%04x, MOP FIFO lvl: 0x%04x, packets: %i\n", 
	   (tmp >> MIP_LVL_SHIFT) & MIP_LVL_MSK, tmp & MOP_LVL_MSK, (tmp & MOP_LVL_MSK) / 6);
    tmp = readl(RRA(XLX_DDMACR));
    printk(KERN_DEBUG "Status before DMA:            0x%08x\n", tmp);
    tmp = readl(RRA(XLX_WDMATLPS));
    printk(KERN_DEBUG "Write DMA TLP Size:           0x%08x\n", tmp);
    tmp = readl(RRA(XLX_WDMATLPC));
    printk(KERN_DEBUG "Write DMA TLP Count:          0x%08x\n", tmp);
  }
}



/*
 * Print FPGA DMA state after read
 */

void rckcrb_read_stat_after(struct rckcrb_data *crb_appl, u64 tsc1, u64 tsc2, unsigned int i) {
  u32 tmp;
  if (mcedev_debug & MCEDBG_DMA) {
    tmp = readl(RRA(XLX_DDMACR));
    printk(KERN_DEBUG "Status after DMA:             0x%08x\n", tmp);
    tmp = readl(RRA(XLX_WDMAPERF));
    printk(KERN_DEBUG "Write DMA Performance:        0x%08x\n", tmp);
    tmp = readl(RRA(RCK_TRNCT4_REG));
    printk(KERN_DEBUG "after DMA: MIP FIFO lvl: 0x%04x, MOP FIFO lvl: 0x%04x\n", 
	   (tmp >> MIP_LVL_SHIFT) & MIP_LVL_MSK, tmp & MOP_LVL_MSK);
    printk(KERN_DEBUG "Duration: tsc2 - tsc1 = %lli (i: %i)\n", tsc2 - tsc1, i);
  }
}


void rckcrb_read_RTRCnt(struct rckcrb_data *crb_appl) {
  u32 tmp;
  tmp = readl(RRA(RCK_RTRCntP0I));
  printk(KERN_DEBUG "RTRCntP0I:  0x%08x\n", tmp);
  tmp = readl(RRA(RCK_RTRCntP0O));
  printk(KERN_DEBUG "RTRCntP0O:  0x%08x\n", tmp);
  tmp = readl(RRA(RCK_RTRCntP1I));
  printk(KERN_DEBUG "RTRCntP1I:  0x%08x\n", tmp);
  tmp = readl(RRA(RCK_RTRCntP1O));
  printk(KERN_DEBUG "RTRCntP1O:  0x%08x\n", tmp);
  tmp = readl(RRA(RCK_RTRCntP2I));
  printk(KERN_DEBUG "RTRCntP2I:  0x%08x\n", tmp);
  tmp = readl(RRA(RCK_RTRCntP2O));
  printk(KERN_DEBUG "RTRCntP2O:  0x%08x\n", tmp);
  tmp = readl(RRA(RCK_RTRCntP3I));
  printk(KERN_DEBUG "RTRCntP3I:  0x%08x\n", tmp);
  tmp = readl(RRA(RCK_RTRCntP3O));
  printk(KERN_DEBUG "RTRCntP3O:  0x%08x\n", tmp);
}


void rckcrb_dump_loopback(struct rckcrb_data *crb_appl) {
  u32 tmp, mip;
  u32 mopData[12];
  int i;
  
  mip = (readl(RRA(RCK_TRNCT4_REG)) >> MIP_LVL_SHIFT) & MIP_LVL_MSK;

  // switch to loopback mode
  printk(KERN_DEBUG "Switching to Loopback mode for debug (0x%04x)\n", mip);
  tmp = readl(RRA(RCK_DEBUG_REG));
  tmp |= RCK_DEBUG_SCEMILOOPBACK;
  writel(tmp, RRA(RCK_DEBUG_REG));

  // Read out and dump all MIP data through MOP loopback
  while (mip >= 6) {
  // read data
    for (i = 0; i < 12; i++) {
      if ((i % 2) == 0)
	mopData[i] = readl(RRA(RCK_TRNCT0_REG)); 	// lw
      else
	mopData[i] = readl(RRA(RCK_TRNCT1_REG));	// hw
    }
    printk(KERN_DEBUG "miplvl: 0x%04x, 0:0x%08x 1:0x%08x 2:0x%08x 3:0x%08x 4:0x%08x 5:0x%08x 6:0x%08x 7:0x%08x 8:0x%08x 9:0x%08x 10:0x%08x 11:0x%08x\n",
	   mip, mopData[0], mopData[1], mopData[2], mopData[3], mopData[4], mopData[5], 
	   mopData[6], mopData[7], mopData[8], mopData[9], mopData[10], mopData[11]);
    udelay(10);
    mip = (readl(RRA(RCK_TRNCT4_REG)) >> MIP_LVL_SHIFT) & MIP_LVL_MSK;
  }
  // Disable loopback again ...
  tmp = readl(RRA(RCK_DEBUG_REG));
  tmp &= ~RCK_DEBUG_SCEMILOOPBACK;
  writel(tmp, RRA(RCK_DEBUG_REG));
}


/*
 * Write specified number of words into MIP using PIO mode from offset in dma_rx buffer.
 * Returns number of words not written.
 */

int rckcrb_write_pio_w(struct rckcrb_data *crb_appl, unsigned int numwords, unsigned int offsetwords) {
  int i = 0;
  u32 tmp;

  if (mcedev_debug & MCEDBG_PIO) {
    // Get FIFO level 
    tmp = readl(RRA(RCK_TRNCT4_REG));
    printk(KERN_DEBUG "rckcrb_write_pio: before: MIP FIFO lvl: 0x%04x, MOP FIFO lvl: 0x%04x, numwords: %i, offsetwords: %i\n", 
	   (tmp >> MIP_LVL_SHIFT) & MIP_LVL_MSK, tmp & MOP_LVL_MSK, numwords, offsetwords);
  }

  // write data
  for (i = 0; i < numwords; i++) {
    if ((i % 2) == 0)  
      writel(*((unsigned int*)(crb_appl->dma_rx_kernelptr + i * 4 + offsetwords * 4)), RRA(RCK_TRNCT2_REG)); // lw
    else
      writel(*((unsigned int*)(crb_appl->dma_rx_kernelptr + i * 4 + offsetwords * 4)), RRA(RCK_TRNCT3_REG)); // hw
  }

  if (mcedev_debug & MCEDBG_PIO) {
    // Get FIFO level 
    tmp = readl(RRA(RCK_TRNCT4_REG));
    printk( KERN_DEBUG "rckcrb_write_pio: after: MIP FIFO lvl: 0x%04x, MOP FIFO lvl: 0x%04x\n", 
	   (tmp >> MIP_LVL_SHIFT) & MIP_LVL_MSK, tmp & MOP_LVL_MSK);
  }
  
  // all data can always be written (FIFO level checking done in filp->write)
  return 0;
}



/*
 * Calculate TLP Size
 */

unsigned int rckcrb_calc_TLPS_inner(unsigned int numwords) {
  unsigned int tmp;

  if (tlpsfixed == 0) {
    if (numwords >= tlps64) 
      return 64;
    tmp = numwords % 64;
    if (tmp == 0)
      return 64;
    if ((numwords > (tlps64 / 2)) && (tmp <= 16))
      return 64;
    if ((numwords > (tlps64 / 4)) && (tmp <= 8))
      return 64;

    if (numwords >= tlps32)
      return 32;
    tmp = numwords % 32;
    if (tmp == 0)
      return 32;
    if ((numwords > (tlps32 / 2)) && (tmp <= 8))
      return 32;
    if ((numwords > (tlps32 / 4)) && (tmp <= 4))
      return 32;

    if (numwords >= tlps16)
      return 16;
    tmp = numwords % 16;
    if (tmp == 0)
      return 16;
    if ((numwords > (tlps16 / 2)) && (tmp == 4))
      return 16;

    if (numwords >= tlps8)
      return 8;
    if ((numwords % 8) == 0)
      return 8;

    return 4;
  }
  else {
    return tlpsfixed;
  }

  return 4;
}


unsigned int rckcrb_calc_TLPS(unsigned int numwords) {
  unsigned int tmp;

  tmp =  rckcrb_calc_TLPS_inner(numwords);

  if (tmp > tlpsmax)
    tmp = tlpsmax;

  return tmp;
}



/*
 * Write specified number of words into MIP using DMA mode from offset in dma_rx buffer.
 * Returns number of words not written.
 */

volatile u32 lastmip = 0xffffffff;

int rckcrb_write_dma_w(struct rckcrb_data *crb_appl, unsigned int numwords, unsigned int offsetwords) {
#if !defined (MCEDEV_MODULE_CRBIF)
  u64 tsc1, tsc2;
#endif
  unsigned int i = 0, tlps = 0, tlpc = 0;
  u32 tmp;


  // Reset Xilinx DMA engine
  // TBD acquire lock
  writel(0x01, RRA(XLX_DCSR1));			// Assert reset
  writel(0x00, RRA(XLX_DCSR1));			// Deassert reset

  MPRINTK(MCEDBG_DMA, KERN_DEBUG "rckcrb_write_dma_w: numwords: %i, offsetwords: %i\n", numwords, offsetwords);

  // determine TLP size based on thresholds
  tlps = rckcrb_calc_TLPS(numwords);

  tlpc = numwords / tlps;

  if (tlpc == 0)
    return numwords;
  
  // setup and configure DMA engine
  writel(crb_appl->dma_rx_deviceptr + (offsetwords * 4), RRA( XLX_RDMATLPA));	// Write TLP Address;
  writel(tlps, RRA(XLX_RDMATLPS));		// TLP Size
  writel(tlpc, RRA(XLX_RDMATLPC));		// TLP count

  rckcrb_write_stat_bef(crb_appl);

  i = 0;

  tmp = readl(RRA(RCK_TRNCT4_REG));
  
  while ((((tmp >> MIP_LVL_SHIFT) & MIP_LVL_MSK) > 0) && (i < 100000)) {
    i++;
    udelay(1);
    tmp = readl(RRA(RCK_TRNCT4_REG));
  }

  //if (printk_ratelimit()) {
    MPRINTK(MCEDBG_LLDMA, KERN_DEBUG "write_dma_w(%5i, %5i), MIP: 0x%04x, MOP: 0x%04x, tlps: %2i, tlpc: %4i, packets: %4i, incompl: %i, leftover: %i, i: %i\n",
	    numwords, offsetwords, (tmp >> MIP_LVL_SHIFT) & MIP_LVL_MSK, tmp & MOP_LVL_MSK, tlps, tlpc, 
	    numwords / 12, numwords % 12, numwords - (tlps * tlpc), i);

    if ((i == 100000) || (((tmp >> MIP_LVL_SHIFT) & MIP_LVL_MSK) >= lastmip)) {
      printk(KERN_DEBUG "write_dma_w(): Error, MIP Assumption not true (currmip: 0x%04x, lastmip: 0x%04x)!!\n", 
	     (tmp >> MIP_LVL_SHIFT) & MIP_LVL_MSK, lastmip);
      rckcrb_read_RTRCnt(crb_appl);
      // rckcrb_dump_loopback(crb_appl);
    }
  //}

  lastmip = ((tmp >> MIP_LVL_SHIFT) & MIP_LVL_MSK) + (numwords / 2);

  MPRINTK(MCEDBG_DMA, KERN_DEBUG "Starting write() DMA\n");
 
  wait_for_dma = 1;
  i = 0;

  writel((1 << 16) | (0 << 23), RRA(XLX_DDMACR));	// Start with interrupt not disabled

#if !defined (MCEDEV_MODULE_CRBIF)
  rdtscll(tsc1);

  // wait until DMA finished
  while ((wait_for_dma) && (i < RCK_MAXWAIT_LOOPS)) {
    i++;
    udelay(1);
  }

  rdtscll(tsc2);

  rckcrb_write_stat_after(crb_appl, tsc1, tsc2, i);
#endif
  
  // calculate and return leftover
  return numwords - (tlps * tlpc);
}



/*
 * Read specified number of words out of MOP using PIO mode storing at offset in dma_tx buffer.
 * Returns number of words not read.
 */

int rckcrb_read_pio_w(struct rckcrb_data *crb_appl, unsigned int numwords, unsigned int offsetwords) {
  int i = 0;
  u32 tmp;

  if (mcedev_debug & MCEDBG_PIO) {
    // Get FIFO level 
    tmp = readl(RRA(RCK_TRNCT4_REG));
    printk(KERN_DEBUG "rckcrb_read_pio: before: MIP FIFO lvl: 0x%04x, MOP FIFO lvl: 0x%04x, numwords: %i offsetwords: %i\n", 
	   (tmp >> MIP_LVL_SHIFT) & MIP_LVL_MSK, tmp & MOP_LVL_MSK, numwords, offsetwords);
  }

  // read data
  for (i = 0; i < numwords; i++) {
    if ((i % 2) == 0)
      *((unsigned int*)(crb_appl->dma_tx_kernelptr + i * 4 + offsetwords * 4)) = readl(RRA(RCK_TRNCT0_REG)); 	// lw
    else
      *((unsigned int*)(crb_appl->dma_tx_kernelptr + i * 4 + offsetwords * 4)) = readl(RRA(RCK_TRNCT1_REG));	// hw
  }

  if (mcedev_debug & MCEDBG_PIO) {
    // Get FIFO level 
    tmp = readl(RRA(RCK_TRNCT4_REG));
    printk(KERN_DEBUG "rckcrb_read_pio: after: MIP FIFO lvl: 0x%04x, MOP FIFO lvl: 0x%04x\n", 
	   (tmp >> MIP_LVL_SHIFT) & MIP_LVL_MSK, tmp & MOP_LVL_MSK);
  }

  // all data can always be read (FIFO level checking done in filp->read function)
  return 0;
}



/*
 * Read specified number of words out of MOP using DMA mode storing at offset in dma_tx buffer.
 * Returns number of words not written.
 */

int rckcrb_read_dma_w(struct rckcrb_data *crb_appl, unsigned int numwords, unsigned int offsetwords) {
#if !defined (MCEDEV_MODULE_CRBIF)
  u64 tsc1, tsc2;
#endif
  unsigned int i = 0, tlps, tlpc;
  u32 tmp;
  
  // Reset Xilinx DMA machine
  // TBD: aquire Lock 
  writel(0x01, RRA(XLX_DCSR1));				// Assert reset
  writel(0x00, RRA(XLX_DCSR1));				// Deassert reset

  MPRINTK(MCEDBG_DMA, "rckcrb_read_dma_w: numwords: %i, offsetwords: %i\n", numwords, offsetwords);

  // determine TLP size based on thresholds
  tlps = rckcrb_calc_TLPS(numwords);
  
  tlpc = numwords / tlps;

  if (tlpc == 0)
    return numwords;

  // setup and configure DMA engine
  writel(crb_appl->dma_tx_deviceptr + (offsetwords * 4), RRA(XLX_WDMATLPA));	// Write TLP Address;
  writel(tlps, RRA(XLX_WDMATLPS));		// TLP Size
  writel(tlpc, RRA(XLX_WDMATLPC));		// TLP count

  rckcrb_read_stat_bef(crb_appl);

  tmp = readl(RRA(RCK_TRNCT4_REG));
  MPRINTK(MCEDBG_LLDMA, KERN_DEBUG " read_dma_w(%5i, %5i), MIP: 0x%04x, MOP: 0x%04x, tlps: %2i, tlpc: %4i, packets: %4i, incompl: %i, leftover: %i\n",
	  numwords, offsetwords, (tmp >> MIP_LVL_SHIFT) & MIP_LVL_MSK, tmp & MOP_LVL_MSK, tlps, tlpc, 
	  numwords / 12, numwords % 12, numwords - (tlps * tlpc));


  MPRINTK(MCEDBG_DMA, "Starting read() DMA\n");
 
  wait_for_dma = 1;
  i = 0;

  writel((1 << 0) | (0 << 7), RRA(XLX_DDMACR));		// Start with interrupt

#if !defined (MCEDEV_MODULE_CRBIF)
  rdtscll(tsc1);

  // wait until DMA finished
  while((wait_for_dma) && (i < RCK_MAXWAIT_LOOPS)) {
    i++;
    udelay(1);
  }

  rdtscll(tsc2);

  rckcrb_read_stat_after(crb_appl, tsc1, tsc2, i);
#endif
  
  // calculate and return leftover
  return numwords - (tlps * tlpc);
}



/*
 * Reset SCEMI infrastructure and FPGA
 */

void rckcrb_reset_scemi(struct rckcrb_data *crb_appl) {
  u32 tmp;
  if (mcedev_debug & (MCEDBG_DMA | MCEDBG_OPEN)) {
    tmp = readl(RRA(RCK_TRNCT4_REG));
    printk(KERN_DEBUG "rckcrb_reset_scemi: MIP FIFO lvl: 0x%04x, MOP FIFO lvl: 0x%04x\n", 
	   (tmp >> MIP_LVL_SHIFT) & MIP_LVL_MSK, tmp & MOP_LVL_MSK);
  }

  MPRINTK((MCEDBG_DMA | MCEDBG_OPEN), "rckcrb_reset_scemi: Resetting FPGA and SCEMI\n");

  // Pull / trigger init of MIP/MOP
  writel(RCK_CONFIG_SOFTRESET, RRA(RCK_CONFIG_REG));
  writel(readl(RRA(RCK_CONFIG_REG)) & (!RCK_CONFIG_SOFTRESET), RRA(RCK_CONFIG_REG));
  writel(RCK_TRNCT5_INIT, RRA(RCK_TRNCT5_REG));

  if (mcedev_debug & (MCEDBG_DMA | MCEDBG_OPEN)) {
    tmp = readl(RRA(RCK_TRNCT4_REG));
    printk(KERN_DEBUG "rckcrb_reset_scemi: MIP FIFO lvl: 0x%04x, MOP FIFO lvl: 0x%04x\n", 
	   (tmp >> MIP_LVL_SHIFT) & MIP_LVL_MSK, tmp & MOP_LVL_MSK);
  }
}



/*
 * Allocate DMA buffer space in kernel memory
 */

int rckcrb_alloc_dmabuffers(struct rckcrb_data *crb_appl, struct mcedev_data *mcedev, size_t dmabuf_size) {

  // allocate RX buffer
  crb_appl->dma_rx_kernelptr = dma_alloc_coherent(&(mcedev->dev->dev), dmabuf_size, &(crb_appl->dma_rx_deviceptr), GFP_KERNEL);

  // success?
  if ((crb_appl->dma_rx_kernelptr == NULL) || (crb_appl->dma_rx_deviceptr == 0))
    return -ENOMEM;

  MPRINTK(MCEDBG_DMA, KERN_DEBUG "rckcrb_alloc_dmabuffers: DMA rx: virtual addr: %p, device addr: 0x%08x, size 0x%08zx\n", 
	  crb_appl->dma_rx_kernelptr, (unsigned int)crb_appl->dma_rx_deviceptr, dmabuf_size);

  // allocate TX buffer
  crb_appl->dma_tx_kernelptr = dma_alloc_coherent(&(mcedev->dev->dev), dmabuf_size, &(crb_appl->dma_tx_deviceptr), GFP_KERNEL);

  // success?
  if ((crb_appl->dma_tx_kernelptr == NULL) || (crb_appl->dma_tx_deviceptr == 0)) {
    // deallocate rx buffer
    dma_free_coherent(&(mcedev->dev->dev), dmabuf_size, crb_appl->dma_rx_kernelptr, crb_appl->dma_rx_deviceptr);
    return -ENOMEM;
  }

  MPRINTK(MCEDBG_DMA, KERN_DEBUG "rckcrb_alloc_dmabuffers: DMA tx: virtual addr: %p, device addr: 0x%08x, size 0x%08zx\n", 
	  crb_appl->dma_tx_kernelptr, (unsigned int)crb_appl->dma_tx_deviceptr, dmabuf_size);

  crb_appl->dmabuf_size = dmabuf_size;

  return 0;
}



/*
 * Free DMA buffer space in kernel memory
 */

void rckcrb_free_dmabuffers(struct rckcrb_data *crb_appl, struct mcedev_data *mcedev) {

  dma_free_coherent(&(mcedev->dev->dev), crb_appl->dmabuf_size, crb_appl->dma_rx_kernelptr, crb_appl->dma_rx_deviceptr);

  dma_free_coherent(&(mcedev->dev->dev), crb_appl->dmabuf_size, crb_appl->dma_tx_kernelptr, crb_appl->dma_tx_deviceptr);

  crb_appl->dmabuf_size = 0;
  crb_appl->dma_rx_kernelptr = NULL;
  crb_appl->dma_tx_kernelptr = NULL;
  crb_appl->dma_rx_deviceptr = 0;
  crb_appl->dma_tx_deviceptr = 0;

  MPRINTK(MCEDBG_DMA, "rckcrb_free_dmabuffers: done\n");
}


/*
 * DMA ISR - just reset the global "wait for DMA bit". 
 * TBD: implement waitqueue
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
irqreturn_t rckcrb_isr(int irq, void *dev_id, struct pt_regs *regs)
#else
irqreturn_t rckcrb_isr(int irq, void *dev_id)
#endif
{
  struct rckcrb_data *crb_appl = (struct rckcrb_data *) dev_id;
  u32 tmp;

  if (mcedev_debug & (MCEDBG_DMA & MCEDBG_INTERRUPT)) {
    printk(KERN_DEBUG "Got crbdev int\n");
    tmp = readl(RRA(XLX_DDMACR));
    printk("IRQ STATUS :            0x%08x\n", tmp);
  }

  // reset global wait flag
  wait_for_dma = 0;

  return IRQ_HANDLED;
}

