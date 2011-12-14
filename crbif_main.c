/***************************************************************************
 *
 * $Date: 2010-08-24 16:35:55 +0200 (Tue, 24 Aug 2010) $
 * $Revision: 16585 $
 * $Author: jbrummer $
 * $HeadURL: https://subversion.jf.intel.com/ctg/mtl/mc/mcemu/software/trunk/linuxdrv-2.6/mcedev/crbif_main.c $
 * $Id: crbif_main.c 16585 2010-08-24 14:35:55Z jbrummer $
 *
 ***************************************************************************
 */


#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/kthread.h>
#include <linux/fs.h>           /* inode */

#include <linux/workqueue.h>    /* Work queue for the packet filter */

#include <linux/in.h>
#include <linux/netdevice.h>    /* struct device, and other headers */
#include <linux/etherdevice.h>  /* eth_type_trans */
#include <linux/skbuff.h>
#include <linux/aer.h>
#include <linux/poll.h>

#include "mcedev.h"
#include "mcedev_common.h"
#include "crbdev.h"             /* CopperRidge specifics */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
#define IRQF_SHARED SA_SHIRQ
#define IRQF_DISABLED SA_INTERRUPT
#endif


const char RCKCRBIDSTRING[] = "$Id: crbif_main.c 16585 2010-08-24 14:35:55Z jbrummer $";
MODULE_DESCRIPTION("Driver for network/user access to the CopperRidge platform");



extern int mcedev_debug;
extern int tlpsmax;


int scemiBufferSize = 32;
module_param(scemiBufferSize, int, 0444);
MODULE_PARM_DESC(scemiBufferSize, "SCEMI buffer size");

int enableLoopback = 0;
module_param(enableLoopback, int, 0644);
MODULE_PARM_DESC(enableLoopback, "Enable the MIP->MOP loopback in the SIF FPGA");

int disableASPM = 1;
module_param(disableASPM, int, 0644);
MODULE_PARM_DESC(disableASPM, "Disable Active State Power Management of PCIe");

int dmaBufferSize = 0;
module_param(dmaBufferSize, int, 0444);
MODULE_PARM_DESC(dmaBufferSize, "Size of the local buffers for DMA transfers");

int traceDma = 0;
module_param(traceDma, int, 0644);
MODULE_PARM_DESC(traceDma, "Trace the raw data of DMA transfers");

int usePioOnlyWrite = 0;
module_param(usePioOnlyWrite, int, 0644);
MODULE_PARM_DESC(usePioOnlyWrite, "usePioOnlyWrite");

int usePioOnlyRead = 0;
module_param(usePioOnlyRead, int, 0644);
MODULE_PARM_DESC(usePioOnlyRead, "usePioOnlyRead");

static int dmaState = 0;
module_param(dmaState, int, 0444);
MODULE_PARM_DESC(dmaState, "Provide access to the DMA state of the daemon");


/*
 * Device function to BAR translation.
 * Device function is index into array:
 * - first element is BAR number
 * - second element is Busregion size 
 *   (actually should be how much is really used - 0 means unused - do not use!)
 */
const struct crbappl_func2bar func2barXilinx[MAXNUM_RCKCRB_FUNCTIONS+1] = {
  {0,   1024 * 1024},       /* 1MB of memory mapped IO in BAR0  */
  {-1,  0},                 /* end of list                      */
};



/* Declaration of file functions */
int     crbif_open(struct inode* inode, struct file* filp);
int     crbif_release(struct inode* inode, struct file* filp);
ssize_t crbif_read(struct file* filp, char __user* buf, 
                   size_t count, loff_t* f_pos);
ssize_t crbif_write(struct file* filp, const char __user* buf,
                    size_t count, loff_t *f_pos);
int     crbif_mmap(struct file* filp, struct vm_area_struct* vma);
long    crbif_unlocked_ioctl(struct file* file, unsigned int command, unsigned long value);
unsigned crbif_poll(struct file* file, poll_table* wait);
#ifdef CONFIG_COMPAT
long    crbif_compat_ioctl(struct file* file, unsigned int command, unsigned long value);
#endif


struct file_operations crbif_fops[MAXNUM_RCKCRB_FUNCTIONS] = {
  { .owner    = THIS_MODULE, 
    .open     = crbif_open, 
    .read     = crbif_read, 
    .write    = crbif_write, 
    .mmap     = crbif_mmap, 
    .release  = crbif_release,
    .unlocked_ioctl    = crbif_unlocked_ioctl,
    .poll     = crbif_poll,
#ifdef CONFIG_COMPAT
    .compat_ioctl = crbif_compat_ioctl
#endif
  },
};



/* The network interface of the CopperRidge connection */
struct net_device* crbnet_dev;

/* A work queue for filtering the incoming packet data */
struct workqueue_struct*  crbif_wq;


/* Symbolic names for DMA operational states */
const unsigned DMA_IDLE         = 0;
const unsigned DMA_TX_PENDING   = 1;
const unsigned DMA_RX_PENDING   = 2;
const unsigned DMA_RX_DONE      = 3;



/*
 * Generic access functions for the DMA FIFOs
 */
int crbif_fifo_read(struct dma_fifo* fifo, void* destination, unsigned size, 
                    int userSpace)
{
  void*       copyTo;
  void*       copyFrom;
  unsigned    bytesToCopy;
  unsigned    headRoom;
  unsigned long flags;
  int         result = 0;
  
  /* Because the buffer is organized as a ring we first copy the part up to
   * the end and then the remaining bytes starting from the beginning. The
   * headroom is the space from the current read pointer to the end of the
   * buffer
   */
  headRoom = dmaBufferSize - fifo->readPointer;


  /* Copy up to headroom bytes from the current read position */
  copyFrom    = (void*)(fifo->data + fifo->readPointer);
  copyTo      = destination;
  
  bytesToCopy = size;
  if (bytesToCopy > headRoom) bytesToCopy = headRoom;

  if (userSpace) {
    result = copy_to_user(copyTo, copyFrom, bytesToCopy);
    if (result) {
      MPRINTK(MCEDBG_READWRITE, KERN_DEBUG 
              "crbif_fifo_read: 1. error while copying data, %d bytes left (from: %p/to: %p/readPointer: %d)\n", 
              result, copyFrom, copyTo, fifo->readPointer);
    }
  }
  else
    memcpy(copyTo, copyFrom, bytesToCopy);
  
    
  /* We are done if we copied everything and abort in case of error */
  if (bytesToCopy == size) goto finish;
  if (result) goto finish;


  /* Copy the rest, starting from the begin of the FIFO */
  fifo->readPointer = 0;
  copyFrom          = (void*)fifo->data;
  copyTo           += bytesToCopy;
  bytesToCopy       = size - bytesToCopy;

  if (userSpace) {
    result = copy_to_user(copyTo, copyFrom, bytesToCopy);
    if (result) {
      MPRINTK(MCEDBG_READWRITE, KERN_DEBUG 
              "crbif_fifo_read: 2. error while copying data, %d bytes left (from: %p/to: %p/readPointer: %d)\n", 
              result, copyFrom, copyTo, fifo->readPointer);
    }
  }
  else 
    memcpy(copyTo, copyFrom, bytesToCopy);


finish:
  /* Now lock the data structure so we can safely update the FIFO status
   * Note that we have to make sure that we are not getting interrupted as
   * the DMA thread might also want to update the buffer status!
   */
  /* FIXME: errors from copy_to_user() are not handled properly */
  spin_lock_irqsave(&fifo->lock, flags);

  /* We just took a couple of bytes out of the buffer */
  fifo->level -= size;
  if (fifo->level<0){
      MPRINTK(MCEDBG_READWRITE, KERN_DEBUG
        "Warning: fifo level set below zero (%d)!!\n", fifo->level );
  }
  
  /* If the chunk of the last copy() is not the total number of bytes
   * we had a wrap-around, i.e. we start from the beginning.
   */
  if (bytesToCopy != size)  fifo->readPointer  = bytesToCopy;
  else                      fifo->readPointer += bytesToCopy;

  spin_unlock_irqrestore(&fifo->lock, flags);

  return result;
}


int crbif_fifo_write(struct dma_fifo* fifo, const void* source, unsigned size, 
                     int userSpace)
{
  const void* copyFrom;
  void*       copyTo;
  unsigned    bytesToCopy;
  unsigned    headRoom;
  unsigned long flags;
  int         result = 0;
  
  /* Because the buffer is organized as a ring we first copy the part up to
   * the end and then the remaining bytes starting from the beginning. The
   * headroom is the space from the current write pointer to the end of the
   * buffer
   */
  headRoom = dmaBufferSize - fifo->writePointer;

  
  /* Copy up to headroom bytes to the current write position */
  copyFrom  = source;
  copyTo    = (void*)(fifo->data + fifo->writePointer);
  
  bytesToCopy = size;
  if (bytesToCopy > headRoom) bytesToCopy = headRoom;
  
  if (userSpace)
    result = copy_from_user(copyTo, copyFrom, bytesToCopy);
  else
    memcpy(copyTo, copyFrom, bytesToCopy);
  
  /* We are done if we copied everything and abort in case of error */
  if (bytesToCopy == size) goto finish;
  if (result) goto finish;
  

  /* Copy the rest */
  fifo->writePointer  = 0;
  copyFrom           += bytesToCopy;
  copyTo              = (void*)fifo->data;
  bytesToCopy         = size - bytesToCopy;

  if (userSpace)
    result = copy_from_user(copyTo, copyFrom, bytesToCopy);
  else
    memcpy(copyTo, copyFrom, bytesToCopy);


finish:
  /* Now lock the data structure so we can safely update the FIFO status
   * Note that we have to make sure that we are not getting interrupted as
   * the DMA thread might also want to update the buffer status!
   */
  /* FIXME: errors from copy_to_user() are not handled properly */
  spin_lock_irqsave(&fifo->lock, flags);

  /* We just stored a couple of bytes in the buffer */
  fifo->level += size;
  
  /* If the chunk of the last copy() is not the total number of bytes
   * we had a wrap-around, i.e. we start from the beginning.
   */
  if (bytesToCopy != size)  fifo->writePointer  = bytesToCopy;
  else                      fifo->writePointer += bytesToCopy;

  spin_unlock_irqrestore(&fifo->lock, flags);
  
  return 0;
}



/*
 * Copy data from the MIP and network Tx buffers to the DMA range so it
 * can be transferred to the CopperRidge platform. The DMA memory is 
 * organized in 2 halfes so one can be processed by the FPGA and the other
 * one by the host.
 */
unsigned crbif_fifo2dma(void* dmaBuffer, struct dma_fifo* fifo, 
                        unsigned maxSize) 
{
  unsigned  bytesToCopy;
  unsigned  packetCount;
  
  /* Determine how many bytes to copy: max(FIFO size, available bytes) */
  bytesToCopy = maxSize;
  if (fifo->level < maxSize) bytesToCopy = fifo->level;
  
  /* Always transfer entire packets to the FPGA */
  packetCount = bytesToCopy / MIP_BYTE_PACKET_SIZE;
  bytesToCopy = packetCount * MIP_BYTE_PACKET_SIZE;

  /* Source and destination memory range are both in kernel space */
  if (packetCount) crbif_fifo_read(fifo, dmaBuffer, bytesToCopy, KERNELSPACE);
  
  return bytesToCopy;
}

int crbif_prepareDma(struct rckcrb_data* crb_appl) 
{
  void*     dmaBuffer;
  int       freeSpace;
  unsigned  bytesCopied;
  
  
  /* This function is called from the daemon thread, only, i.e. we need not
   * worry about concurrent accesses to the Tx DMA because nobody else 
   * modifies them.
   */
  dmaBuffer = crb_appl->dma_rx_kernelptr + 
              crb_appl->dma_txOffset + crb_appl->dma_txSize;
  
  /* Calculate how much free space is left. Only half the buffer size is
   * available for preparation.
   */
  freeSpace = (dmaBufferSize/2 - crb_appl->dma_txSize);
  if (freeSpace < 0) {
    printk(KERN_ERR "crbif_prepareDma: Tx DMA buffer overflow\n");
    return -EIO;
  }
  
  /* Handle network packets first */
  bytesCopied = crbif_fifo2dma(dmaBuffer, &crb_appl->net_fifo, freeSpace);
  
  /* Adjust the variables for the next write */
  crb_appl->dma_txSize  += bytesCopied;
  dmaBuffer             += bytesCopied;
  freeSpace             -= bytesCopied;

  /* And finish with user space MIP packets */
  bytesCopied = crbif_fifo2dma(dmaBuffer, &crb_appl->mip_fifo, freeSpace);
  crb_appl->dma_txSize += bytesCopied;

  
  if (bytesCopied) MPRINTK(MCEDBG_DMA, KERN_DEBUG 
                           "crbif_prepareDma: copied %i bytes\n", bytesCopied);
  return 0;
}

void crbif_traceDma(void* dmaBase, unsigned dmaOffset, unsigned numDWords)
{
  unsigned i;
  
  
  /* Return immediately if tracing is not enabled */
  if (!traceDma) return;

    
  printk(KERN_DEBUG "Raw DMA data @0x%p+%d\n", dmaBase, dmaOffset);
  for (i=0; i<numDWords; i++) {
    printk("%4d: %08X\n", i+1, *(unsigned*)(dmaBase + dmaOffset + 4*i));
  }
}



int crbif_doDma(struct rckcrb_data* crb_appl)
{
  unsigned  tmp;
  unsigned  mipStatus;
  unsigned  mopStatus;
  unsigned  numDWords;
  unsigned  offset;
  unsigned  availableBytes;
  unsigned  copiedBytes;
  unsigned  leftover;
  
  
  /* Provide access to the DMA state via a module parameter as first debug
   * hint in case of driver issues.
   */
  dmaState = crb_appl->dma_state;
  
  /* Only 1 DMA transfer may be active at any time */
  if (crb_appl->dma_state != DMA_IDLE) return 0;
  
  
  /* Determine the MIP/MOP status */
  tmp = readl(RRA(RCK_TRNCT4_REG));
  mopStatus = tmp & MOP_LVL_MSK;
  mipStatus = (tmp >> MIP_LVL_SHIFT) & MIP_LVL_MSK;

  if ((mipStatus & MIP_LVLERR_MSK) || (mopStatus & MOP_LVLERR_MSK)) {
    //if (printk_ratelimit()) {
      printk(KERN_ERR "crbif_doDma: Error in MIP/MOP FIFOs: 0x%08x\n", tmp);
    //}
    return -EIO;
  }
  
  
  /* Write to the MIP if the flag is set */
  if (crb_appl->dma_doTx) {
    /* Check how much space is available in the FPGA */
    availableBytes = (MIP_FIFO_DEPTH - mipStatus) * MIP_FIFO_WIDTH;
    
    /* Calculate the number of DWords from the number of bytes we may
     * transmit, limited by the available space. Also convert the offset
     * value to dword units.
     */
    numDWords = crb_appl->dma_txSize/4;
    if (availableBytes < crb_appl->dma_txSize) numDWords = availableBytes/4;
    offset = crb_appl->dma_txOffset/4;

    /* Trigger the actual data transfer if there is data to send */        
    if (numDWords) {
      /* Already set the DMA state before starting the actual transfer so
       * the interrupt service routine sees the right state in case it is
       * quicker than this code.
       */
      crb_appl->dma_state = DMA_TX_PENDING;
// DBG TLE
      if (usePioOnlyWrite)
	leftover = numDWords;
      else
	leftover = rckcrb_write_dma_w(crb_appl, numDWords, offset);
      
      /* Check whether a DMA transfer was actually started */
      if (leftover == numDWords) {
        /* No, i.e. send the data through PIO accesses and prevent waiting
         * for the DMA interrupt.
         */
        crb_appl->dma_state = DMA_IDLE;
        leftover = rckcrb_write_pio_w(crb_appl, numDWords, offset);
      }
      
      /* Calculate how much data is actually transferred */
      copiedBytes = 4*(numDWords - leftover);
      
      /* Generate debug information */
      MPRINTK(MCEDBG_DMA, KERN_DEBUG 
              "crbif_doDma: write %i dwords from %i (%i remaining)\n", 
              numDWords, offset, leftover);
      crbif_traceDma(crb_appl->dma_rx_kernelptr, crb_appl->dma_txOffset,
                     numDWords-leftover);
      
      
      /* Update the DMA data structure so we can prepare the next transfer,
       * i.e. switch the Tx offset to the other half and initialise it with
       * the leftover bytes.
       */
      if (crb_appl->dma_txOffset == 0) 
        crb_appl->dma_txOffset = dmaBufferSize/2;
      else
        crb_appl->dma_txOffset = 0;
      
      /* If there was not enough space in the MIP FIFO in the FPGA we have
       * to copy the remaining bytes in addition to the DMA leftover
       */
      leftover = (crb_appl->dma_txSize - 4*numDWords) + 4*leftover;
      
      /* Copy the bytes to the beginning for the next DMA transfer */
      if (leftover) {
        /* Copy to the new Tx offset using the DMA parameters */
        /* Note that the DMA function unit is dwords! */
        memcpy(crb_appl->dma_rx_kernelptr + crb_appl->dma_txOffset,
               crb_appl->dma_rx_kernelptr + 4*offset + copiedBytes,
               leftover);
      }
      crb_appl->dma_txSize = leftover;
      
      MPRINTK(MCEDBG_DMA, KERN_DEBUG 
              "crbif_doDma: next transmission starting at offset %i\n", 
              crb_appl->dma_txOffset);
    }
    
    /* Read data from the FPGA in the next round */
    crb_appl->dma_doTx = 0;
  }
  
  /* Fetch data from the MOP if the DMA logic is idle and we are not filtering 
   * old data.
   * Note: crbif_filter() should not be called again before processing is
   * complete as otherwise the packet order cannot be maintained!
   */
  if (crb_appl->dma_state == DMA_IDLE && !crb_appl->dma_doFilter) {
    /* Calculate how many dwords we may ask the DMA functions to fetch */
    availableBytes = mopStatus * MOP_FIFO_WIDTH;
    if (availableBytes > dmaBufferSize/2) availableBytes = dmaBufferSize/2;
    numDWords = availableBytes/4;

    /* Trigger the actual data transfer if we can read something. Note that
     * we use a fixed offset of 4kB = 1k dword to keep the DMA address
     * nicely aligned. Packet fragments from previous transfers are copied
     * into this 4k space and processed by the filter function.
     */
    if (numDWords) {
      /* Already set the DMA state before starting the actual transfer so
       * the interrupt service routine sees the right state in case it is
       * quicker than this code.
       */
      crb_appl->dma_state = DMA_RX_PENDING;
// DBG TLE
      if (usePioOnlyRead)
	leftover = numDWords;
      else
	leftover = rckcrb_read_dma_w(crb_appl, numDWords, 1024);
      
      /* Check whether a DMA transfer was actually started */
      if (leftover == numDWords) {
        /* No, i.e. fetch the data through PIO accesses and prevent waiting
         * for interrupts.
         */
        leftover = rckcrb_read_pio_w(crb_appl, numDWords, 1024);
        crb_appl->dma_state = DMA_RX_DONE;
      }
      MPRINTK(MCEDBG_DMA, KERN_DEBUG 
              "crbif_doDma: read %i dwords (%i remaining)\n", 
              numDWords, leftover);


      /* Update the DMA data structure */
      crb_appl->dma_rxSize   += 4*(numDWords - leftover);
      crb_appl->dma_doFilter  = 1;
    }
    
    /* Send data to the FPGA in the next round */
    crb_appl->dma_doTx = 1;
  }
  
  return 0;
}



/* Filtering the incoming DMA data stream is implemented in a dedicated
 * workqueue. Hence the function declaration must match the work queue
 * handler requirements.
 */
void crbif_decodeHeader(u8* packet, u8* core, unsigned* cmd, 
                        unsigned* address, u8* memType, u8* byteEnable)
{
  int x, y, z;
  int i;
  
  
  /* The packet layout is as follows:
   * Byte   0-31  Payload           256b
   *          32  Byteenable          8b
   *          33  Transaction ID      8b  -> unused
   *          34  Source ID           8b  -> fixed 1
   *          35  Destination ID      8b  -> fixed 0
   *       36-39  Address[31:0]      32b
   *          40  Address[33:32]      2b
   *       40-41  Command            12b
   *       41-42  RockCreek ID        8b
   *       42-43  RockCreek sub-ID    3b  -> 0b000/1 = core 0/1
   *       43-47  Reserved           39b
   */
  *byteEnable = packet[32];
  
  *address = 0;
  for (i=0; i<4; i++) *address |= (packet[36+i] << (8*i));
  
  *memType = (packet[40] & 0x03);
  
  *cmd  = ((packet[40] >> 2) & 0x3F);   /* cmd[ 5: 0] */
  *cmd |= ((packet[41] & 0x3F) << 6);   /* cmd[11: 6] */
  
  x = ((packet[41] >> 6) & 0x03) + ((packet[42] & 0x03) << 2);
  y = ((packet[42] >> 2) & 0x0F);
  z = ((packet[42] >> 6) & 0x03);
  *core = 12*y + 2*x + z;
  
  MPRINTK(MCEDBG_DMA, KERN_DEBUG 
          "crbif_decodeHeader: CMD %03X @ 0x%08X (%i) from core #%i\n", 
          *cmd, *address, *memType, *core);
}

int crbif_isNetPacket(unsigned cmd, u8 memType)
{
  /* Network packets are write requests from Rock Creek. The command
   * encoding is as follows:
   * 0x*4*  => Responses, ACK, Abort
   * 0x*2*  => Write
   * 0x*0*  => Read
   */
  if (cmd & 0x020) {
    /* Also decode the address in case of writes, i.e. the memory type */
    if (memType == 0x02) return 1;
  }
  
  return 0;
}

void crbif_extractData(unsigned cmd, unsigned address, u8 byteEnable, 
                       unsigned* offset, unsigned* size)
{
  int i;
  
  
  /* Usually the payload starts at the beginning of the buffer */
  *offset = 0;
  
  /* In case of entire cache lines (WBI) all 32B of the payload range
   * are valid.
   */
  if (cmd == 0x02C) {
    *size = 32;
    return;
  }
  
  /* Else calculate the offset so it points to the 8B range within the 32B
   * payload which is covered by the byte enables.
   */
  *size   = 0;
  
  /* Walk through the byte enables and increase the offset until we found
   * the first valid byte. Also count the number of valid data bytes.
   * Note this assumes the byte enables match the following expression:
   * byteEnable ~= "0*1+0*"
   */
  for (i=0; i<8; i++) {
    /* Count valid bytes */
    if (((byteEnable >> i) & 0x01) == 1) *size += 1;
    /* Else increment the offset while we have not found the data */
    else if (*size == 0) *offset += 1;
  }
}

void crbnet_pktHandler(u8 core, unsigned address, void* data, unsigned size);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
void crbif_filter(void* arg) 
{
  struct rckcrb_data*   crb_appl = (struct rckcrb_data*) arg;
#else
void crbif_filter(struct work_struct *work) 
{
  struct rckcrb_data*   crb_appl = container_of(work, struct rckcrb_data, crbif_work);
#endif
  void*                 dmaBuffer;
  u8                    core;
  unsigned              cmd;
  unsigned              address;
  unsigned              offset;
  unsigned              size;
  u8                    memType;
  u8                    byteEnable;
  int                   freeSpace;
  unsigned              numPackets;
  int                   leftover;
  unsigned              i;
  
  
  /* Calculate the start of the data area. DMA transfer addresses are 4k
   * aligned and in case the previous filter run left some packet fragments
   * they were copied into this 4k range.
   */
  dmaBuffer = crb_appl->dma_tx_kernelptr + 4096 - crb_appl->dma_rxOffset;
  
  /* Process all packets */
  freeSpace = dmaBufferSize - crb_appl->mop_fifo.level;
  numPackets = crb_appl->dma_rxSize / MOP_BYTE_PACKET_SIZE;
  if (crb_appl->dma_rxSize > freeSpace) 
    numPackets = freeSpace / MOP_BYTE_PACKET_SIZE;
  
  
  /* Dump the raw data to the kernel log */
  if (numPackets) crbif_traceDma(crb_appl->dma_tx_kernelptr, 
                                 4096-crb_appl->dma_rxOffset, numPackets*12);

  
  for (i=0; i<numPackets; i++) {
    /* Decode the packet header */
    crbif_decodeHeader(dmaBuffer, &core, &cmd, &address, &memType, &byteEnable);
    
    /* Hand network packets over to the network data filter and copy all
     * other data to the MOP FIFO
     */
    if (crbif_isNetPacket(cmd, memType)) {
      /* Extract the valid payload in case it is not an entire cacheline and
       * let the network function handle the data.
       */
      crbif_extractData(cmd, address, byteEnable, &offset, &size);
      crbnet_pktHandler(core, address, dmaBuffer+offset, size);
    }
    else {
      crbif_fifo_write(&crb_appl->mop_fifo, dmaBuffer, 
                       MOP_BYTE_PACKET_SIZE, KERNELSPACE);
    }
    
    /* Move on to the next packet */
    dmaBuffer += MOP_BYTE_PACKET_SIZE;
  }
  
  /* Setup the DMA data structure for the next transfer, i.e. copy
   * any packet fragments into the space before the 4k DMA start address
   * so they are not lost.
   */
  leftover = crb_appl->dma_rxSize - numPackets*MOP_BYTE_PACKET_SIZE;
  memcpy(crb_appl->dma_tx_kernelptr+4096-leftover, dmaBuffer, leftover);
  crb_appl->dma_rxOffset  = leftover;
  crb_appl->dma_rxSize    = leftover;
  crb_appl->dma_doFilter  = 0;
}



/* Kernel thread for the actual data transfer between CopperRidge and host */
static struct task_struct* commThread;

/* Get our own wait queue and completion interface */
static DECLARE_WAIT_QUEUE_HEAD(crbifd_wait);
static DECLARE_COMPLETION(crbifd_exit);

static int crbif_daemon(void* arg) 
{
  struct rckcrb_data* crb_appl = (struct rckcrb_data*) arg;
  static long last = 0;
  
  DECLARE_WAITQUEUE(wait, current);
  add_wait_queue(&crbifd_wait, &wait);

  last = jiffies;
  
  /* Continue working until we are told to stop */
  while (!kthread_should_stop() ) {
    /* Trigger the packet filter if new data is available */
    if (crb_appl->dma_state == DMA_RX_DONE) {
      if (!queue_work(crbif_wq, &crb_appl->crbif_work) ) {
        printk(KERN_ERR "crbif_daemon: Could not trigger packet filter\n");
      }
      
      /* The last DMA request is now truly completed */
      crb_appl->dma_state = DMA_IDLE;
    }
    
    /* Copy data from the MIP and network transmit buffers to the DMA range */
    crbif_prepareDma(crb_appl);
    
    /* Trigger DMA transfers only if the previous request has been completed */
    if (crbif_doDma(crb_appl)) {
      u16 nId;

      //if (printk_ratelimit()) {
        printk(KERN_ERR "crbif_daemon: Could not trigger DMA transfer\n");
      //}
      pci_read_config_word(crb_appl->dev, PCI_DEVICE_ID, &nId);
      if (nId == 0xFFFF) {
        printk(KERN_INFO "crbif_daemin: Board power off detected! (PCI ID: %x).\n", nId);
        do {
          msleep(500);
          crb_appl->dma_state = DMA_IDLE;
          pci_read_config_word(crb_appl->dev, PCI_DEVICE_ID, &nId);
          if (nId != 0xFFFF) {
            printk(KERN_INFO "crbif_daemin: Board power on detected! (PCI ID: %x).\n", nId);
            break;
          }
          schedule();
        } while (1);
        pci_restore_state(crb_appl->dev);
        /* Reset the device via the Device Control Status Register */
        writel(0x01, RRA(XLX_DCSR1));
        writel(0x00, RRA(XLX_DCSR1));
      }
    }
    
    /* Let the processor do other work if DMA is inactive. Note that the
     * daemon does not enter a sleep state, i.e. its CPU utilization will
     * stay at 100%. However if e.g. schedule_timeout_interruptible() is
     * called the ping latencies will increase at least by an order of
     * magnitude.
     */
    if (crb_appl->dma_state == DMA_IDLE) {
      //msleep(500);
      if ( jiffies - last > 1000 ) {
        //printk( "interruptible - sleep (%ld/%ld)\n", jiffies, last );
        set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(HZ);
        set_current_state(TASK_INTERRUPTIBLE);
        //printk( "going on\n" );
      } else {
        schedule();
      }
    } else {
      last = jiffies;
    }//schedule();
    /*if (crb_appl->dma_state == DMA_IDLE) {
      __set_current_state(TASK_UNINTERRUPTIBLE);
      schedule_timeout_interruptible(HZ);
    } */
  }
  
  __set_current_state(TASK_RUNNING);
  remove_wait_queue(&crbifd_wait, &wait);
  
  return 0;
}



/* Interrupt service routine handling end-of-DMA transfer interrupts */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
irqreturn_t crbif_isr(int irq, void* dev_id, struct pt_regs* regs)
#else
irqreturn_t crbif_isr(int irq, void* dev_id)
#endif
{
  struct rckcrb_data* crb_appl = (struct rckcrb_data*) dev_id;
  u32 tmp;

  if (mcedev_debug & (MCEDBG_DMA & MCEDBG_INTERRUPT)) {
    tmp = readl(RRA(XLX_DDMACR));
    printk(KERN_DEBUG "Got crbdev IRQ, status = 0x%08X\n", tmp);
  }

  /* Update the DMA state according to the operation which just completed */
  MPRINTK(MCEDBG_DMA, KERN_DEBUG "crbif_isr() state=%i\n", crb_appl->dma_state);
  if (crb_appl->dma_state == DMA_TX_PENDING) crb_appl->dma_state = DMA_IDLE;
  if (crb_appl->dma_state == DMA_RX_PENDING) crb_appl->dma_state = DMA_RX_DONE;
  
  /* Wake up the daemon process */
  wake_up_interruptible(&crbifd_wait);
  
  return IRQ_HANDLED;
}



/* Forward declaration of the netdevice initialization functions */
void crbnet_init(struct net_device* dev);
void crbnet_bind(struct net_device* dev, struct rckcrb_data* crb_appl);

static inline int bit(int num) {
  return __builtin_ffs(num)-1;
/*  int n;

  while (num>1) {
    n++;
    num >>= 1;
  }

  return n;*/
}


/*
 * Initialization function called by mcedev_probe() from mcedev_main.c
 */
int crbif_init(struct mcedev_data* mcedev, int dev_found, int major) 
{
  struct rckcrb_data* crb_appl;
  
  int errorCode;
  int devno;
  int irq; 
  int i;

  printk(KERN_INFO "crbif_init: %s (debuglevel %04X)\n",
	 RCKCRBIDSTRING, mcedev_debug);


  /* Disable Active State Power Management first, if applicable */
  if (disableASPM) rckcrb_disable_ASPM(mcedev);

  /* Enable PCIE error reporting */
  pci_enable_pcie_error_reporting(mcedev->dev);

  pci_set_master(mcedev->dev);
  pci_save_state(mcedev->dev);

  /* Initialize the data structure for the application specific interface */
  crb_appl = kmalloc(sizeof(struct rckcrb_data), GFP_KERNEL);
  if (!crb_appl) {
    printk(KERN_ERR "Failed to kmalloc memory for rckcrb_data structure \n");
    return -ENOMEM;
  }
  memset(crb_appl, 0, sizeof(struct rckcrb_data));
  crb_appl->func2bar = &func2barXilinx[0];
  spin_lock_init(&crb_appl->mip_fifo.lock);
  spin_lock_init(&crb_appl->mop_fifo.lock);
  spin_lock_init(&crb_appl->net_fifo.lock);
  
  /* Tailor the mcedev structure for CopperRidge */
  mcedev->mcedev_type     = MCEDEV_TYPE_RCK_CRB;
  mcedev->specific_data   = crb_appl;
  mcedev->cleanup         = crbif_cleanup;

  mcedev_set_resources(mcedev, mcedev->dev);
  mcedev_print_resources(mcedev);

  
  /* Map memory of BAR0 for device register programming by the driver */
  errorCode = map_to_kernelmem(mcedev->bar[0].start, mcedev->bar[0].size, 
                               &crb_appl->bar0_kernelptr, "bar 0");
  if (errorCode) {
    printk(KERN_ERR "crbif_init: Failed to ioremap BAR0 region, error: %i\n", 
                    errorCode);
    kfree(crb_appl);
    return errorCode;
  }
  
  
  /* Printout FPGA register information and reset the communication system */
  rckcrb_init_bitfield_test(crb_appl);
  rckcrb_reset_scemi(crb_appl);

  /* Read SCEMI buffer size */
  scemiBufferSize = bit(ioread32(RRA(RCK_SCEMI_BUFFER_SIZE)) << 7);
  if (scemiBufferSize <= 0) {
    printk(KERN_WARNING "Could not detect fifo size, setting default to 64k\n" );
    scemiBufferSize = 13;
  }
  

  printk(KERN_INFO "MIP/MOP size: %d/%dKB (%d)\n", MIP_FIFO_SIZE >> 10, MOP_FIFO_SIZE >> 10, scemiBufferSize);

  dmaBufferSize = 2 * MIP_FIFO_SIZE;

  /* Configure the SIF-FPGA loopback mode
   * Note that the bit gets only cleared through a hardware reset so it
   * makes sense to configure it at startup.
   */
  i = readl(RRA(RCK_DEBUG_REG));
  if (enableLoopback) i |= RCK_DEBUG_SCEMILOOPBACK;
  else                i &= ~RCK_DEBUG_SCEMILOOPBACK;
  writel(i, RRA(RCK_DEBUG_REG));

  
  /* Set Maximum TLP Size based on configured values in FPGA */
  if (tlpsmax == 0) {
    tlpsmax = rckcrb_set_tlpsmax(crb_appl);
    if (tlpsmax < 0) {
      printk(KERN_ERR "crbif_init: Illegal max TLP size %i\n", tlpsmax);
      crbif_cleanup(mcedev);
      return -ENXIO;
    }
  }

  
  /* Allocate buffer memory for DMA */
  errorCode = rckcrb_alloc_dmabuffers(crb_appl, mcedev, dmaBufferSize);
  if (errorCode) {
    printk(KERN_ERR "crbif_init: Failed to allocate DMA buffers, error: %i\n", 
                    errorCode);
    crbif_cleanup(mcedev);
    return errorCode;
  }
  crb_appl->mip_fifo.data = vmalloc(dmaBufferSize);
  crb_appl->mop_fifo.data = vmalloc(dmaBufferSize);
  crb_appl->net_fifo.data = vmalloc(dmaBufferSize);
  if (!crb_appl->mip_fifo.data || 
      !crb_appl->mop_fifo.data ||
      !crb_appl->net_fifo.data)
  {
    printk(KERN_ERR "crbif_init: Failed to allocate FIFO memory\n");
    crbif_cleanup(mcedev);
    return -ENOMEM;
  }
  

  /* Configure the interrupt as MSI */
  irq =  mcedev->dev->irq;
  MPRINTK((MCEDBG_INIT | MCEDBG_INTERRUPT), 
          KERN_DEBUG "Interrupt pre MSI: %i\n", irq);

  errorCode = pci_enable_msi(mcedev->dev);
  if (errorCode) {
    printk(KERN_ERR "crbif_init: Failed to switch into MSI mode, error: %i\n", 
                    errorCode);
    crbif_cleanup(mcedev);
    return errorCode;
  }
  else {
    irq =  mcedev->dev->irq;
    MPRINTK((MCEDBG_INIT | MCEDBG_INTERRUPT), KERN_DEBUG 
            "Enable MSI returned %i, using IRQ %i\n", errorCode, irq);
  }
  
  crb_appl->irq_num = irq;
  errorCode = request_irq(irq, crbif_isr, IRQF_SHARED | IRQF_DISABLED,
                          "crbif", (void*) crb_appl);

  if (errorCode) {
    printk(KERN_ERR "crbif_init: request_irq() failed, error: %i\n", errorCode);
    crbif_cleanup(mcedev);
    return errorCode;
  }

  /* We finally got the interrupt we wanted so enable its usage */
  crb_appl->int_usage = 1;

  
  /* Allocate device nodes */
  for (i=0; i < MAXNUM_RCKCRB_FUNCTIONS; i++) {
    if (crb_appl->func2bar[i].size > 0) {
      /* Compare assumed with available size */
      if (crb_appl->func2bar[i].size > 
          mcedev->bar[crb_appl->func2bar[i].bar].size)
      {
        printk(KERN_ERR "crbif_init: Driver is assuming size of %ld in BAR %d, but it only provides %ld Bytes.\n",
                        crb_appl->func2bar[i].size, 
                        crb_appl->func2bar[i].bar, 
                        mcedev->bar[crb_appl->func2bar[i].bar].size);
        crbif_cleanup(mcedev);
        return -ENOMEM;
      }
      
      /* Add device to kernel */
      devno = MKDEV(major, MKMINOR(dev_found, i));
      errorCode = allocate_device_node(devno, 1, &crbif_fops[i], 
                                       &crb_appl->crb_function[i].cdev);
      if (errorCode) {
        printk(KERN_ERR "crbif_init: allocate_device_node() failed, error: %i\n", 
                        errorCode);
        crbif_cleanup(mcedev);
        return errorCode;
      }
      
      /* Update our application data */
      crb_appl->crb_function[i].devno     = devno;
      crb_appl->crb_function[i].fops      = &crbif_fops[i];
      crb_appl->crb_function[i].use_count = 0;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
      class_device_create(mcedev->mcedev_class, NULL, devno, 
                          &(mcedev->dev->dev), "crbif%drb%d", dev_found, i);
#else
      device_create(mcedev->mcedev_class, NULL, devno, 
                          &(mcedev->dev->dev), "crbif%drb%d", dev_found, i);
#endif

    }
  }


  /* Create a work queue for the packet filter and the work structure */
  crbif_wq = create_singlethread_workqueue("crbfilter");
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
  INIT_WORK(&crb_appl->crbif_work, crbif_filter, crb_appl);
#else
  INIT_WORK(&crb_appl->crbif_work, crbif_filter);
#endif

  /* Start the thread for the CRB<->host data transfer */
  crb_appl->dma_state     = DMA_IDLE;
  crb_appl->dma_doFilter  = 0;

  crb_appl->dev = mcedev->dev;

  commThread = kthread_run(crbif_daemon, crb_appl, "%s", "crbifd");
  if (IS_ERR(commThread) ) {
    printk(KERN_ERR "crbif_init: Failed to start crbif daemon\n");
    commThread = NULL;
    crbif_cleanup(mcedev);
    return -EIO;
  }

  /* Allocate the network device */
  crbnet_dev = alloc_netdev(sizeof(struct crbnet_priv), "crb0", crbnet_init);
  if (!crbnet_dev) {
    printk(KERN_ERR "crbif_init: alloc_netdev() failed\n"); 
    crbif_cleanup(mcedev);
    return -ENOMEM;
  }

  errorCode = register_netdev(crbnet_dev);
  if (errorCode) {
    printk(KERN_ERR "crbif: error %i registering device \"%s\"\n", 
                    errorCode, crbnet_dev->name);
    crbif_cleanup(mcedev);
    return -ENODEV;
  }

  /* Bind the network device to the CRB application */
  crbnet_bind(crbnet_dev, crb_appl);
  
  
  MPRINTK(MCEDBG_INIT, KERN_DEBUG "crbif_init: ...done\n");

  return 1;
}



/* Cleanup function called from mcedev_remove() in mcedev_main.c via
 * the pointer in the mcedev structure.
 */
void crbif_cleanup(struct mcedev_data* mcedev) 
{
  struct rckcrb_data* crb_appl = (struct rckcrb_data*)mcedev->specific_data;
  int i;

  MPRINTK(MCEDBG_CLEANUP, KERN_DEBUG 
          "crbif_cleanup: Releasing device resources...\n");


  /* Reset the device via the Device Control Status Register */
  writel(0x01, RRA(XLX_DCSR1));
  writel(0x00, RRA(XLX_DCSR1));

  
  /* Remove the network device if it exists */
  if (crbnet_dev) {
    MPRINTK(MCEDBG_CLEANUP, KERN_DEBUG "  crbif_cleanup: netdev\n");
    unregister_netdev(crbnet_dev);
    free_netdev(crbnet_dev);
  }

  
  /* Stop the data transfer thread */
  if (commThread) {
    MPRINTK(MCEDBG_CLEANUP, KERN_DEBUG "  crbif_cleanup: crbifd\n");
    kthread_stop(commThread);
  }

  /* Stop the work queue */
  if (crbif_wq){
    destroy_workqueue(crbif_wq);
  }

  /* Unregister device files */
  for (i=0; i < MAXNUM_RCKCRB_FUNCTIONS; i++) {
    if (crb_appl->crb_function[i].devno == 0) {
      continue;
    }

    if (crb_appl->crb_function[i].use_count != 0) {  
      MPRINTK(MCEDBG_CLEANUP, KERN_DEBUG 
              "function use count != 0: function %i, count %i\n",
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


  /* Free the resources */
  vfree(crb_appl->mip_fifo.data);
  vfree(crb_appl->mop_fifo.data);
  vfree(crb_appl->net_fifo.data);

  if (crb_appl->int_usage > 0) free_irq(crb_appl->irq_num, (void*) crb_appl); 

  /* disable msi */
  pci_disable_msi(mcedev->dev);

  rckcrb_free_dmabuffers(crb_appl, mcedev);
  if (crb_appl->bar0_kernelptr != NULL) iounmap(crb_appl->bar0_kernelptr);
  kfree(mcedev->specific_data);

  pci_disable_pcie_error_reporting(mcedev->dev);

  MPRINTK(MCEDBG_CLEANUP, KERN_DEBUG "crbif_cleanup: done\n");
}
