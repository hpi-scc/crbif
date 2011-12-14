/***************************************************************************
 *
 * $Date: 2010-08-24 16:35:55 +0200 (Tue, 24 Aug 2010) $
 * $Revision: 16585 $
 * $Author: jbrummer $
 * $HeadURL: https://subversion.jf.intel.com/ctg/mtl/mc/mcemu/software/trunk/linuxdrv-2.6/mcedev/crbif_net.c $
 * $Id: crbif_net.c 16585 2010-08-24 14:35:55Z jbrummer $
 *
 ***************************************************************************
 */


#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/in.h>
#include <linux/netdevice.h>    /* struct device, and other headers */
#include <linux/etherdevice.h>  /* eth_type_trans */
#include <linux/ip.h>           /* struct iphdr */
#include <linux/tcp.h>          /* struct tcphdr */
#include <linux/skbuff.h>

#include "mcedev.h"
#include "mcedev_common.h"      /* MINOR_* macros (requires mcedev.h) */
#include "crbdev.h"             /* CopperRidge specifics */



/* RCK-Host network specific constants which have to match the #defines
 * of the rckpc driver!
 */
#define MAX_PKTSIZE         4096    /* Maximum size of data transfers to RCK */

/* Register offsets of the host mailbox registers
 * Note that we hae to use different cachelines to use the write
 * combining buffer (WCB) efficiently.
 */
#define MBX_NULL            0x00 /* Dummy address to flush WCB */
#define MBX_CONFIG          0x20
#define MBX_PACKETSTART     0x40
#define MBX_PACKETDATA      0x60
#define MBX_RXDONE          0x80

/* Register offset of the CONFIG registers */
#define RCK_GLCFG0          0x10
#define RCK_GLCFG1          0x18

/* Mask of the interrupt bits */
#define RCK_INTR_MASK       0x00000002
#define RCK_NMI_MASK        0x00000001

/* Symbolic names for RockCreek commands */
#define CMD_WBI             0x02C   /* cacheline write */
#define CMD_NCWR            0x022   /* uncacheable write with byte enables */



/* mcedev_debug controls MPRINTK debug output messages (see mcedev.h).
 * Used values are  MCEDBG_NET    => 0x2000 (8192)
 */
extern int mcedev_debug;

/* dmaBufferSize is the size of the network FIFO and is used to check
 * that we can send an entire packet.
 */
extern int dmaBufferSize;

/* Make the number of packets in flight configurable. Note that an unsigend
 * variable is used to store the status, i.e. the value should be <= 32!
 */
static int txPacketSlots = 16;
module_param(txPacketSlots, int, 0644);
MODULE_PARM_DESC(txPacketSlots, "Number of packet slots at the receiver");

/* Local address at the destination DDR3 memory controller where the shared
 * memory is located. By default the 16 MB at 0xF000.0000 of each MC are 
 * mapped to 0x8000.0000-0x8300.0000. Note that the first 2 MB are resered
 * for the shared memory TTYs.
 */
static ulong txBaseAddress = 0xF0200000UL;
module_param(txBaseAddress, ulong, 0664);
MODULE_PARM_DESC(txBaseAddress, "Start address of the packet space at the MC");

static int timeout = 2;
module_param(timeout, int, 0644);
MODULE_PARM_DESC(timeout, "Timeout in jiffies of the Tx watchdog.\
 Packets are dropped after timeout*HZ.");

/* In case interrupts are enabled, the host first clears the two IRQ bits
 * of the CRB register in RCK and then asserts the NMI/LINT1 bit
 */
static int enableIrq = 1;
module_param(enableIrq, int, 0644);
MODULE_PARM_DESC(enableIrq, "Enable triggering the LINT1 interrupt on RCK");


static int subnetSize = 12;
module_param(subnetSize, int, 0644);
MODULE_PARM_DESC(subnetSize, "Size of subnets for routing, 0 to disable");

int maxTransId = 0x40;
module_param(maxTransId, int, 0644);
MODULE_PARM_DESC(maxTransId, "Maximal transaction id");


/* The network interface of the CopperRidge connection */
extern struct net_device* crbnet_dev;



/*
 * Generic helper functions
 */
int isBusy(struct crbnet_priv* priv, u8 destIp)
{
  u8        coreNr    = destIp - 1;
  int       slot      = priv->nextTxSlot[coreNr];
  unsigned  bitMask   = (1 << (slot-1));
  unsigned  txStatus  = priv->txSlotBusy[coreNr];

  if (txStatus & bitMask) return 1;
  return 0;
}

void setBusy(struct crbnet_priv* priv, u8 destIp, int slot, u8 value)
{
  spinlock_t*   lock      = &(priv->lock);
  u8            coreNr    = destIp - 1;
  unsigned      bitMask   = (1 << (slot-1));
  unsigned long flags;

  /* Prevent interference as the flags are modified by hw_tx() in transmit
   * and tx_complete() in receive direction.
   */
  spin_lock_irqsave(lock, flags);
  
  if (value)  priv->txSlotBusy[coreNr] |= bitMask;
  else        priv->txSlotBusy[coreNr] &= ~bitMask;
  
  spin_unlock_irqrestore(lock, flags);
}

int getMc(u8 destIp)
{
  u8 coreNr = destIp - 1;
  u8 tileNr = coreNr / 2;
  u8 row    = tileNr / 6;
  u8 col    = tileNr % 6;
  int mc;

  /* The memory controllers serve the cores in their quadrant and are
   * located at
   *    W y=2 x=0       y=2 x=5 E
   *    W y=0 x=0       y=0 x=5 E
   * The return value is as follows:
   * 10: 8  => rc_subid
   *  7: 4  => rc_id(y)
   *  3: 0  => rc_id(x)
   */
  if (col < 3)  mc = 0x600; /* WEST port 0/0 */
  else          mc = 0x400; /* EAST port 0/0 */

  if (row > 1) mc |= 0x020;
  if (col > 2) mc |= 0x005;

  return mc;
}

ulong getAddress(struct crbnet_priv* priv, u8 destIp)
{
  u8        coreNr  = destIp - 1;
  u8        z       = coreNr % 2;
  u8        y       = coreNr / 12;
  u8        x       = (coreNr % 12) / 2;
  ulong     address = txBaseAddress;

  /* y=3  x=0..2->7..11   x=3..5->7..11
   * y=2  x=0..2->0..6    x=3..5->0..6
   * y=1  x=0..2->7..11   x=3..5->7..11
   * y=0  x=0..2->0..6    x=3..5->0..6
   */
  if (x > 2) x -= 3;

  if (y==0 || y==2) address += ((2*x+z)   * txPacketSlots*MAX_PKTSIZE);
  else              address += ((2*x+z+6) * txPacketSlots*MAX_PKTSIZE);

  address += (priv->nextTxSlot[coreNr] * MAX_PKTSIZE);

  return address;
}



/*
 * Open and close
 * These functions are called when an interface is activated/stopped. Thus,
 * any system resources should be registered and the device itself should
 * be initialized.
 */
int crbnet_open(struct net_device* dev)
{
  MPRINTK(MCEDBG_NET, KERN_DEBUG "crbnet_open()\n");


  /* Assign the hardware address of the board (6 octets for Ethernet)
   * Note that the first octet of ethernet multicast addresses is odd
   */
  memcpy(dev->dev_addr, "\0CRB_0", ETH_ALEN);

  netif_start_queue(dev);

  return 0;
}



int crbnet_close(struct net_device* dev)
{
  MPRINTK(MCEDBG_NET, KERN_DEBUG "crbnet_close()\n");


  netif_stop_queue(dev);

  return 0;
}



/*
 * Configuration changes (passed on by ifconfig)
 */
int crbnet_config(struct net_device* dev, struct ifmap* map)
{
  MPRINTK(MCEDBG_NET, KERN_DEBUG "crbnet_config()\n");


  /* Can't act on a running interface */
  if (dev->flags & IFF_UP) return -EBUSY;

  /* Don't allow changing the I/O address */
  if (map->base_addr != dev->base_addr) {
    printk(KERN_WARNING "crbnet: Can't change I/O address\n");
    return -EOPNOTSUPP;
  }

  /* Ignore other fields */
  return 0;
}



/*
 * Low level data transfer of 32B chunks to the destination core memory
 */
void crbnet_hw_header(struct rckcrb_data* crb_appl, unsigned cmd, 
                      unsigned dest, ulong address) 
{
  unsigned i;
  
  /* The packet header expected by the SIF FPGA is as follows.
   * Note that we always write complete cache lines!
   *
   * Bit      Byte    Content
   *   7:  0     0    Byte enable       (unused: 0xFF)
   *  15:  8     1    Transaction ID    (transId)
   *  23: 16     2    Source ID         (host->RCK: 0x00)
   *  31: 24     3    Destination ID    (host->RCK: 0x01)
   *
   *  63: 32  7: 4    Address[31: 0]    (address)
   *
   *  65: 64     8    Address[33:32]    (fixed: 0b00)
   *  77: 66  9: 8    Command type
   *  85: 78 10: 9    Rock Creek ID     (dest[ 7:0])
   *  88: 86 11:10    Rock Creek sub-Id (dest[10:8])
   *  95: 89    11    Reserved
   *
   * 127: 96 15:12    Reserved
   */
  static u8 transId     = 0;
  static u8 header[16]  = { 0xFF, 0x00, 0x00, 0x01,
                            0x00, 0x00, 0x00, 0x00,
                            0xB0, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00 };
  
  
  /* Add the 128b header and update its variable fields */
  /* Transaction ID */
  header[1] = transId;
  transId++;
  if (transId == maxTransId) {
    transId = 0;
  }
  
  /* Address */
  for(i=0; i<4; i++) header[4+i] = (address >> 8*i) & 0xFF;

  /* Set the mixed-meaning fields of the packet header */
  header[8]   = ((address >> 4 * 8) & 0x03); /* [1:0] = address[33:32] */
  header[8]  |= ((cmd & 0x3F) << 2);    /* [7:2] = cmd[5:0]       */
  header[9]   = ((cmd >> 6) & 0x3F);    /* [5:0] = cmd[11:6]      */
  header[9]  |= ((dest & 0x03) << 6);   /* [7:6] = dest[1:0]      */
  header[10]  = ((dest >> 2) & 0xFF);   /* [7:0] = dest[9:2]      */
  header[11]  = ((dest >>10) & 0x01);   /* [0]   = dest[10]       */

  crbif_fifo_write(&crb_appl->net_fifo, header, 16, KERNELSPACE);
}
    
int crbnet_hw_tx(struct net_device* dev, u8 destIp, struct sk_buff* skb)
{
  struct crbnet_priv*         priv = netdev_priv(crbnet_dev);
  struct rckcrb_data*         crb_appl = priv->crb_appl;
  int                         mc = getMc(destIp);
  ulong                       txBuffer = getAddress(priv, destIp);
  unsigned                    bytesToWrite;
  unsigned                    packetsToSend;
  unsigned                    x;
  unsigned                    y;
  int                         dest;
  ulong                       address;
  
  /* The default value of the GLCFG register with the interrupt bits is
   * 0x0DF8
   */
  static u8 regValue[4] = { 0xF8, 0x0D, 0x00, 0x00 };
  
  
  MPRINTK(MCEDBG_NET, KERN_DEBUG
          "crbnet_hw_tx: %i bytes to 0x%09lX @ %03X\n", skb->len, txBuffer, mc);


  /* Check that there is sufficient space in the network FIFO. We have to add
   * 128b=16B for each 32B data chunk so for simplicity just multiply by 2.
   */
  if (dmaBufferSize - crb_appl->net_fifo.level < 2*skb->len) 
    return NETDEV_TX_BUSY;

  /* Write the packet data to the network FIFO. Since the first cache line
   * holds the packet length information which is used by RCK to detect new
   * data it has to be written last, i.e. we start from the end!
   */
  
  /* Calculate how much valid data we have in the last packet and how many
   * packets we have to send in total. Note that the count has to be
   * incremented in case of partial fills.
   */
  bytesToWrite  = skb->len % 32;
  packetsToSend = skb->len / 32;
  if (bytesToWrite == 0)  bytesToWrite = 32;
  else                    packetsToSend++;
  
  while (packetsToSend) {
    /* Decrement the number of packets so we can use the variable to
     * calculate the offset of the packet data.
     */
    packetsToSend--;
    
    if (bytesToWrite < 32) {
      /* First write the remaining bytes from the packet */
      crbif_fifo_write(&crb_appl->net_fifo, skb->data+packetsToSend*32,
                       bytesToWrite, KERNELSPACE);
      /* And fill up the 32B cacheline with garbage data */
      crbif_fifo_write(&crb_appl->net_fifo, skb->data,
                       32-bytesToWrite, KERNELSPACE);
      /* Subsequent writes will all span entire cache lines */
      bytesToWrite = 32;
    }
    else {
      crbif_fifo_write(&crb_appl->net_fifo, skb->data+packetsToSend*32,
                       32, KERNELSPACE);
    }

    /* Add the 128b header */
    crbnet_hw_header(crb_appl, CMD_WBI, mc, txBuffer+packetsToSend*32);
  }

  /* Also generate an edge on the LINT1 interrupt pin if enabled */
  if (enableIrq) {
    /* Send the packet to the CRB of the core's tile */
    y =  (destIp - 1) / 12;
    x = ((destIp - 1) % 12) / 2;
    dest = ((0x2 << 8) | (y << 4) | (x << 0));
    
    /* The address depends on the destination core */
    if (destIp % 2) address = RCK_GLCFG0;
    else            address = RCK_GLCFG1;
    
    /* First send a NCWR to clear the 2 interrupt bits */
    regValue[0] = 0xF8;
    for (x=0; x<8; x++) crbif_fifo_write(&crb_appl->net_fifo, regValue, 4,
                                         KERNELSPACE);
    crbnet_hw_header(crb_appl, CMD_NCWR, dest, address);
      
    /* Then assert NMI/LINT1 to generate the edge for the APIC */
    regValue[0] |= RCK_NMI_MASK;
    for (x=0; x<8; x++) crbif_fifo_write(&crb_appl->net_fifo, regValue, 4,
                                         KERNELSPACE);
    crbnet_hw_header(crb_appl, CMD_NCWR, dest, address);
  }
  
  
  /* Transmission succeeded so save the timestamp of the transmission,
   * update the statistics and free the socket buffer
   */
  dev->trans_start        = jiffies;
  priv->lastTx[destIp-1]  = jiffies;

  priv->stats.tx_packets++;
  priv->stats.tx_bytes += skb->len;

  setBusy(priv, destIp, priv->nextTxSlot[destIp-1], 1);
  priv->nextTxSlot[destIp-1] = (priv->nextTxSlot[destIp-1] + 1)%txPacketSlots;

  dev_kfree_skb_any(skb);

  return NETDEV_TX_OK;
}



/*
 * Transmit a packet (low level interface)
 * This function deals with HW details, i.e. it writes the packet
 * into the message buffer and informs the destination.
 */
void crbnet_tx_reset(unsigned core, u8 slot)
{
  struct crbnet_priv*         priv = netdev_priv(crbnet_dev);

  MPRINTK(MCEDBG_NET, KERN_DEBUG 
          "crbnet_tx_reset: using slot %i at core #%i\n", slot, core);

  /* Reset the transmit state */
  priv->nextTxSlot[core]  = slot;
  priv->txSlotBusy[core]  = 0;
  priv->lastTx[core]      = 0;
}

void crbnet_tx_complete(unsigned core, u8 slot)
{
  struct crbnet_priv*         priv = netdev_priv(crbnet_dev);

  MPRINTK(MCEDBG_NET, KERN_DEBUG 
          "crbnet_tx_complete: slot %i at core #%i\n", slot, core);

  /* Clear the corresponding busy flag */
  setBusy(priv, core+1, slot, 0);
}

int crbnet_tx(struct sk_buff* skb, struct net_device* dev)
{
  struct crbnet_priv*         priv = netdev_priv(crbnet_dev);
  struct iphdr*               ipHeader;
  unsigned long               waitPeriod;
  u32*                        destAddr;
  u8                          destIp;


  /* Extract the destination address from the IP header.
   * The last (4th) octet is interpreted as core ID.
   */
  ipHeader  = (struct iphdr*)(skb->data + dev->hard_header_len);
  destAddr  = &(ipHeader->daddr);
  destIp    = ((u8*)destAddr)[3];

  if (subnetSize)
    destIp = ((destIp - 1) / subnetSize) * subnetSize + 1;

  MPRINTK(MCEDBG_NET, KERN_DEBUG
          "crbnet_tx: %i bytes to %i\n", skb->len, destIp);


  /* Perform a sanity check on the packet data, i.e. silently drop packets
   * that are either too short or send to an illegal IP address by indicating
   * success without executing any data transfer operations.
   */
  if ((skb->len < dev->hard_header_len+sizeof(struct iphdr) ) ||
      (destIp == 0) || ((destIp - 1) > RCK_CORECOUNT) )
  {
    printk(KERN_NOTICE "crbnet_tx: Illegal packet (%i octets to IP %d)\n",
                       skb->len, destIp);
    goto drop_packet;
  }


  /* Check if we have a Tx slot available */
  if (isBusy(priv, destIp) ) {
    /* Make sure that we waited at least a little */
    waitPeriod = jiffies - priv->lastTx[destIp-1];
    if (waitPeriod > 1) {
      /* Simply drop the packet if the destination seems to be unreachable,
       * i.e. if too much time elapsed since the last transmission
       */
      if (waitPeriod > timeout*HZ) {
        MPRINTK(MCEDBG_NET, KERN_NOTICE "crbnet_tx: Timeout at destination %d\n", destIp);
        goto drop_packet;
      }
      /* Stop the transmit queue as the RCK cores apparently cannot cope 
       * with the data.
       */
      netif_stop_queue(dev);
    }
    
    return NETDEV_TX_BUSY;
  }


  /* Perform the low level data transfer */
  return crbnet_hw_tx(dev, destIp, skb);


drop_packet:
  priv->stats.tx_errors++;
  dev_kfree_skb_any(skb);
  return NETDEV_TX_OK;
}



void crbnet_tx_timeout(struct net_device* dev)
{
  /* A timeout occurs if the queue was stopped because too many packet
   * transfers were pending. Let's simply resume regular packet processing
   * in the hope that the receivers have cleared their backlog.
   */
  netif_wake_queue(dev);

  return;
}



/*
 * Receive a packet: retrieve, encapsulate and pass over to upper levels
 */
void crbnet_rx_complete(unsigned core)
{
  struct crbnet_priv*         priv = netdev_priv(crbnet_dev);
  struct sk_buff*             skb;
  int                         len;


  /* Fetch the packet and remove it from our internal data structure */
  skb = priv->rxInFlight[core];
  priv->rxInFlight[core] = NULL;

  /* Update the interface statistics (also count the header bytes) */
  len = 256*skb->head[0] + skb->head[1];
  priv->stats.rx_packets++;
  priv->stats.rx_bytes += 2+len;
  crbnet_dev->last_rx = jiffies;

  /* Write metadata, and then pass to the receive level */
  skb->dev        = crbnet_dev;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
  skb->h.raw      = skb->data;
  skb->nh.raw     = skb->data;
  skb->mac.raw    = skb->data;
#else
  skb_reset_transport_header(skb);
  skb_reset_network_header(skb);
  skb_reset_mac_header(skb);
#endif

  skb->protocol   = htons(ETH_P_IP);
  skb->pkt_type   = PACKET_HOST;
  skb->ip_summed  = CHECKSUM_UNNECESSARY;

  MPRINTK(MCEDBG_NET, KERN_DEBUG 
          "crbnet_rx_complete: %i bytes received from core #%i\n", len, core);
      
  netif_rx(skb);
}

void crbnet_rx_start(unsigned core, unsigned char* payload, unsigned size)
{
  struct crbnet_priv*         priv = netdev_priv(crbnet_dev);
  struct sk_buff*             skb;
  unsigned char*              ip_data;
  int                         len;

  MPRINTK(MCEDBG_NET, KERN_DEBUG 
          "crbnet_rx_start: packet header received from core #%i\n", core);


  /* The packet length is stored in the first 2 bytes but does not include
   * the header. Check for reasonable sizes before processing the data to
   * prevent nasty memory overflow errors.
   */
  len = 256*payload[0] + payload[1];
  if ((len < sizeof(struct iphdr)) || (len > crbnet_dev->mtu) ) {
    printk(KERN_NOTICE "crbnet_rx: illegal packet length - packet dropped\n");
    priv->stats.rx_dropped++;
    return;
  }

  /* Build a skb for the packet data so upper layers can handle it
   * Note that IP headers should be aligned on 16B boundaries. Let's thus
   * reserve 16B additional descriptor space.
   */
  skb = dev_alloc_skb(len+16);
  if (!skb) {
    //if (printk_ratelimit() )
      printk(KERN_NOTICE "crbnet_rx: low on mem - packet dropped\n");

    priv->stats.rx_dropped++;
    return;
  }
  skb_reserve(skb, 16);

  /* Calculate the how much valid payload data we got. If we received a full
   * cacheline (size=32) it may be bigger than the IP packet + its 2 header
   * bytes!
   */
  if (size >= len+2)  size  = len;
  else                size -= 2;

  /* Initialize the descriptor data:
   * 2B: IP packet length
   * 2B: offset, i.e. where to write the next chunk of data
   * 2B: Outstanding payload bytes
   */
  skb->head[0] = len/256;
  skb->head[1] = len%256;
  skb->head[2] = size/256;
  skb->head[3] = size%256;
  skb->head[4] = (len-size)/256;
  skb->head[5] = (len-size)%256;

  /* Store the packet in our internal data structure but check if other
   * incomplete transmissions exist.
   */
  if (priv->rxInFlight[core]) {
    printk(KERN_NOTICE "crbnet_rx: dropping incomplete packet from %i\n", core);
    dev_kfree_skb_any(priv->rxInFlight[core]);
    priv->stats.rx_dropped++;
  }
  priv->rxInFlight[core] = skb;

  /* Copy the packet data (without the header) into the payload area */
  memcpy(skb->data, payload+2, size);

  /* Set the tail pointer to the end of the packet data */
  ip_data = skb_put(skb, len);

  /* Check if the packet is already complete */
  if (size == len) crbnet_rx_complete(core);
}

void crbnet_rx_data(unsigned core, unsigned char* payload, unsigned size)
{
  struct crbnet_priv*         priv = netdev_priv(crbnet_dev);
  struct sk_buff*             skb = priv->rxInFlight[core];
  unsigned                    offset;
  unsigned                    outstanding;

  MPRINTK(MCEDBG_NET, KERN_DEBUG 
          "crbnet_rx_data: %i bytes from core #%i\n", size, core);


  /* Make sure that we already have started the receive process */
  if (!skb) {
    printk(KERN_NOTICE "crbnet_rx: received invalid data from %i\n", core);
    priv->stats.rx_errors++;
    return;
  }


  offset      = 256*skb->head[2] + skb->head[3];
  outstanding = 256*skb->head[4] + skb->head[5];

  /* Adjust the usable data size in case we got more data than needed to
   * complete the IP packet
   */
  if (size > outstanding) size = outstanding;

  /* Copy the packet data and update the descriptor fields */
  memcpy((void*)(skb->data + offset), payload, size);
  skb->head[2] = (offset+size)/256;
  skb->head[3] = (offset+size)%256;
  skb->head[4] = (outstanding-size)/256;
  skb->head[5] = (outstanding-size)%256;

  /* Check if the packet is complete now */
  if (size == outstanding) crbnet_rx_complete(core);
}



/*
 * The crbif daemon fetches data from the FPGA and its filter function
 * hands us the packets which are intended for the network interface.
 * Preprocessing has already been done, i.e. we only have to deal with
 * the data.
 */
void crbnet_pktHandler(u8 core, unsigned address, void* data, unsigned size)
{
  MPRINTK(MCEDBG_NET, KERN_DEBUG 
          "crbnet_pktHandler: core #%i: %i bytes to 0x%04X\n",
          core, size, address);
  
  
  /* Data consistency check */
  if (core >= RCK_CORECOUNT) {
    printk(KERN_NOTICE 
           "crbnet_pktHandler: Illegal packet from core #%i\n", core);
    return;
  }
  
  /* Decode the different mailbox addresses 
   * (align to cachelines in case we get partial writes)
   */
  address &= 0xFFE0;
  if (address == MBX_CONFIG)      crbnet_tx_reset(core, *((u8*)data) );
  if (address == MBX_RXDONE)      crbnet_tx_complete(core, *((u8*)data) );
  if (address == MBX_PACKETSTART) crbnet_rx_start(core, data, size);
  if (address == MBX_PACKETDATA)  crbnet_rx_data(core, data, size);
}



/*
 * This function is called to fill up an eth header, since ARP is not
 * available on the interface.
 */
int crbnet_rebuild_header(struct sk_buff* skb)
{
  printk(KERN_WARNING "crbnet_rebuild_header() called - ignoring\n");

  return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
int crbnet_header(struct sk_buff* skb, struct net_device* dev,
                  unsigned short type, void* daddr, void* saddr,
                  unsigned int len)
#else
int crbnet_header(struct sk_buff* skb, struct net_device* dev,
                  unsigned short type, const void* daddr, const void* saddr,
                  unsigned len)
#endif
{
  /* Prepend 2 header bytes containing the packet length */
  u8* header = skb_push(skb, 2);

  /* Store the length starting with the MSBs */
  header[0] = len/256;
  header[1] = len%256;

  return (dev->hard_header_len);
}



/*
 * ioctl commands
 */
int crbnet_ioctl(struct net_device* dev, struct ifreq* rq, int cmd)
{
  MPRINTK(MCEDBG_NET, KERN_DEBUG "crbnet_ioctl()\n");

  return 0;
}



/*
 * Return statistics to the caller
 */
struct net_device_stats* crbnet_stats(struct net_device* dev)
{
  struct crbnet_priv* priv = netdev_priv(dev);

  return &(priv->stats);
}



/*
 * The "change_mtu" method is usually not needed.
 */
int crbnet_change_mtu(struct net_device* dev, int new_mtu)
{
  struct crbnet_priv*         priv = netdev_priv(dev);
  unsigned long               flags;
  spinlock_t*                 lock = &(priv->lock);

  /* check ranges */
  if ((new_mtu < 68) || (new_mtu > MAX_PKTSIZE)) return -EINVAL;

  /*
   * Simply accept the value
   */
  spin_lock_irqsave(lock, flags);
  dev->mtu = new_mtu;
  spin_unlock_irqrestore(lock, flags);

  return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29)
static const struct net_device_ops crbnet_device_ops = {
  .ndo_open = crbnet_open,
  .ndo_stop = crbnet_close,
  .ndo_start_xmit = crbnet_tx,
  .ndo_get_stats = crbnet_stats,
  .ndo_tx_timeout = crbnet_tx_timeout,
  .ndo_change_mtu = crbnet_change_mtu,
  .ndo_do_ioctl = crbnet_ioctl,
  .ndo_set_config = crbnet_config,
};

static const struct header_ops crbnet_header_ops = {
  .create = crbnet_header,
  .rebuild = crbnet_rebuild_header,
};
#endif


/*
 * The init function (sometimes called probe).
 * It is invoked by register_netdev()
 */
void crbnet_init(struct net_device* dev)
{
  struct crbnet_priv* priv;

  MPRINTK(MCEDBG_NET, KERN_DEBUG "crbnet_init()\n");


  /* Get meaningful default values */
  ether_setup(dev);

  /* Set the correct function pointers */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
  dev->open               = crbnet_open;
  dev->stop               = crbnet_close;
  dev->set_config         = crbnet_config;
  dev->hard_start_xmit    = crbnet_tx;
  dev->do_ioctl           = crbnet_ioctl;
  dev->get_stats          = crbnet_stats;
  dev->change_mtu         = crbnet_change_mtu;
  dev->rebuild_header     = crbnet_rebuild_header;
  dev->hard_header        = crbnet_header;
  dev->tx_timeout         = crbnet_tx_timeout;
#else
  dev->netdev_ops         = &crbnet_device_ops;
  dev->header_ops         = &crbnet_header_ops;
#endif

  dev->watchdog_timeo     = timeout;

  /* This network device does not need an interrupt as the packet
   * receive function is triggered by data from the CRB polling thread.
   */
  dev->irq                = 0;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
  dev->poll               = NULL;
  dev->weight             = 0;
  /* Disable caching of (nonexistent) ARP replies */
  dev->hard_header_cache  = NULL;
#endif

  /* Reset the flags set by ether_setup() and set NOARP */
  dev->flags              = IFF_NOARP;
  /* Checksum checks are not required */
  dev->features          |= NETIF_F_NO_CSUM;
  /* Change the hardware header as there is no need for an Ethernet format */
  dev->hard_header_len    = 2;
  dev->addr_len           = 0;

  /*
   * Then initialize the priv field. This encloses the statistics
   * and a few private fields.
   */
  priv = netdev_priv(dev);
  memset(priv, 0, sizeof(struct crbnet_priv));

  spin_lock_init(&priv->lock);
}



/*
 * The link function connects the network device to the CRB application
 */
void crbnet_bind(struct net_device* dev, struct rckcrb_data* crb_appl)
{
  struct crbnet_priv*         priv = netdev_priv(dev);

  priv->crb_appl = crb_appl;
}

/**
 * \brief Set subnet size
 * \param nSize new subnet size
 */
void setSubnetSize(unsigned long nSize){
  subnetSize = nSize;
}

/**
 * \brief Set tx base address
 * \param nAddress new base address
 */
void setTxBaseAddress(unsigned long nAddress) {
  txBaseAddress = nAddress;
}
