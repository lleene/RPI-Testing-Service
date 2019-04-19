//
// C lib Proceedures for servicing i2s/spi/dma/pwm/gpclk on RPI3 over LAN
// Using bcm2835 C lib
//
// 2019 Lieuwe B. Leene
//

#ifndef __BCM2835_DMA_H__
#define __BCM2835_DMA_H__

#include <assert.h>
#include <bcm2835.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#define MAJOR_NUM 100
#define IOCTL_MBOX_PROPERTY _IOWR(MAJOR_NUM, 0, char *)
#define DEVICE_FILE_NAME "/dev/vcio"

// Physical Address defines for BCM2835_DMA
// ---- Memory mappping defines
#define BUS_TO_PHYS(x) ((x) & ~0xC0000000)

// ---- Memory allocating defines
// https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
#define MEM_FLAG_DIRECT (1 << 2)
#define MEM_FLAG_COHERENT (2 << 2)
#define MEM_FLAG_L1_NONALLOCATING (MEM_FLAG_DIRECT | MEM_FLAG_COHERENT)

#define SPI_FIFO_PHYLOC 0x7E204004 ///< Physical Adress Location of SPI FIFO
#define I2S_FIFO_PHYLOC 0x7E203004 ///< Physical Adress Location of I2S FIFO

#define BCM2835_DMA_BASE                                                       \
  0x007000 ///< Adress Offset of DMA BASE w.r.t Peripherals
#define BCM2835_SPI0_BASE                                                      \
  0x204000 ///< Adress Offset of SPI BASE w.r.t Peripherals
#define BCM2835_I2S_BASE                                                       \
  0x203000 ///< Adress Offset of I2S BASE w.r.t Peripherals

// Register offsets from BCM2835_DMA_BASE.
#define BCM2835_DMA(n) ((n)*0x100) ///< DMA #N CB Source Register Location
#define BCM2835_DMA_CB(n) ((n)*0x100 + 0x4) ///< DMA #N Control Block Offset
#define BCM2835_DMA_TI(n)                                                      \
  ((n)*0x100 + 0x8) ///< DMA #N Transfer Information Offset
#define BCM2835_DMA_SA(n) ((n)*0x100 + 0xc)  ///< DMA #N Source Offset
#define BCM2835_DMA_DA(n) ((n)*0x100 + 0x10) ///< DMA #N Destination Offset
#define BCM2835_DMA_LN(n) ((n)*0x100 + 0x14) ///< DMA #N Transfer Length
#define BCM2835_DMA_ST(n) ((n)*0x100 + 0x18) ///< DMA #N 2D Stride
#define BCM2835_DMA_NB(n) ((n)*0x100 + 0x1c) ///< DMA #N Next CB Location
#define BCM2835_DMA_DB(n) ((n)*0x100 + 0x20) ///< DMA #N Debug Information
#define BCM2835_DMA_ENABLE (0xff0)           /// DMA Enable Status

// Register masks for DMA Transfer Information Register
#define BCM2835_DMA_CB_TI_NO_WIDE_BURSTS (1 << 26) ///< No Wide bursts
#define BCM2835_DMA_CB_TI_WAIT_RESP (1 << 3)       ///< Wait for AXI pipeline
#define BCM2835_DMA_CB_TI_WAITS(n) ((n) << 21)     ///< Wait n Cycles afer write
#define BCM2835_DMA_CB_TI_WAIT_MASK (0x1F << 21)   ///< Wait n Cycles afer write
#define BCM2835_DMA_CB_TI_PERMAP_MASK (0x1F << 16) ///< Mask for sel. peripheral
#define BCM2835_DMA_CB_TI_PERMAP_NONE (0 << 16)    ///< sel. no peripheral
#define BCM2835_DMA_CB_TI_PERMAP_DSI (1 << 16)     ///< sel. DSI peripheral
#define BCM2835_DMA_CB_TI_PERMAP_TXPCM (2 << 16)   ///< sel. TXSPI peripheral
#define BCM2835_DMA_CB_TI_PERMAP_RXPCM (3 << 16)   ///< sel. RXSPI peripheral
#define BCM2835_DMA_CB_TI_PERMAP_PWM (5 << 16)     ///< sel. PWDM peripheral
#define BCM2835_DMA_CB_TI_PERMAP_TXSPI (6 << 16)   ///< sel. TXSPI peripheral
#define BCM2835_DMA_CB_TI_PERMAP_RXSPI (7 << 16)   ///< sel. RXSPI peripheral
#define BCM2835_DMA_CB_TI_SRC_IGR (1 << 11)        ///< Src. write is ignored
#define BCM2835_DMA_CB_TI_SRC_DREQ (1 << 10)       ///< Source is driven by DREQ
#define BCM2835_DMA_CB_TI_SRC_128b (1 << 9)        ///< 128b Dest. read width
#define BCM2835_DMA_CB_TI_SRC_INC                                              \
  (1 << 8) ///< Src. Add. incremented after write
#define BCM2835_DMA_CB_TI_DEST_IGR (1 << 7)  ///< Dest. write is ignored
#define BCM2835_DMA_CB_TI_DEST_DREQ (1 << 6) ///< Destination is driven by DREQ
#define BCM2835_DMA_CB_TI_DEST_128b (1 << 5) ///< 128b Dest. read width
#define BCM2835_DMA_CB_TI_DEST_INC (1 << 4)  ///< Destination is incremented
#define BCM2835_DMA_CB_TI_TDMODE (1 << 1)    ///< Enable 2D stride
#define BCM2835_DMA_CB_TI_INTRPT (1 << 0)    ///< Enable Interrupt on Complete

// Register masks for DMA Control and Status Register
#define BCM2835_DMA_CS_INTRPT (1 << 2)
#define BCM2835_DMA_CS_ERROR (1 << 8)
#define BCM2835_DMA_CS_STOP (1 << 5)
#define BCM2835_DMA_CS_PAUSE (1 << 4)
#define BCM2835_DMA_CS_DREQ (1 << 3)
#define BCM2835_DMA_CS_RESET (1 << 31)
#define BCM2835_DMA_CS_ABORT (1 << 30)
#define BCM2835_DMA_CS_WAIT (1 << 28)
#define BCM2835_DMA_CS_END (1 << 1)
#define BCM2835_DMA_CS_ENABLE (1 << 0)

// Register masks for DMA Debug Register
#define BCM2835_DMA_DEBUG_CLR (0x7)
#define BCM2835_DMA_DEBUG_LITE (1 << 28)
#define BCM2835_DMA_DEBUG_RDERR (1 << 2)
#define BCM2835_DMA_DEBUG_FIFOERR (1 << 1)

// #define BCM2835_PAGE_SIZE (4 * 1024)

struct UncachedMemBlock {
  void *mem; // User visible value: the memory to use.
  //-- Internal representation.
  uint32_t bus_addr;
  uint32_t mem_handle;
  size_t size;
};

// @4.2.1.1
struct dma_cb {    // 32 bytes.
  uint32_t info;   // transfer information.
  uint32_t src;    // physical source address.
  uint32_t dst;    // physical destination address.
  uint32_t length; // transfer length.
  uint32_t stride; // stride mode.
  uint32_t next;   // next control block; Physical address. 32 byte aligned.
  uint32_t pad[2];
};

#ifdef __cplusplus
extern "C" {
#endif

// Mailbox Utility
int mbox_open();
void mbox_close(int file_desc);

// Mailbox Memory Management Functions
unsigned get_version(int file_desc);
unsigned mem_alloc(int file_desc, unsigned size, unsigned align,
                   unsigned flags);
unsigned mem_free(int file_desc, unsigned handle);
unsigned mem_lock(int file_desc, unsigned handle);
unsigned mem_unlock(int file_desc, unsigned handle);
void *mapmem(unsigned base, unsigned size);
void unmapmem(void *addr, unsigned size);

// Memory management for DMA interfacing
struct UncachedMemBlock UncachedMemBlock_alloc(unsigned size); // uses the
                                                               // mailbox to
                                                               // allocate
                                                               // memory that
                                                               // can be seen by
                                                               // the DMA
void UncachedMemBlock_free(struct UncachedMemBlock *block); // uses the mailbox
                                                            // to free memory
                                                            // that can be seen
                                                            // by the DMA
uint32_t UncachedMemBlock_to_physical(
    const struct UncachedMemBlock *blk,
    void *p); // uses the mailbox to fetch the physical address used by the DMA

// DMA Initialization
int bcm2835_dma_init(void); // sets the pointers for accessing SPI & DMA

// DMA Debug functions
void bcm2835_dma_status(); // Helper function to check active DMA channels
void bcm2835_dma_debug(
    int channel); // Helper function to check the DMA Debug Internals
void bcm2835_dma_info(int channel); // Helper function to check the DMA status
void bcm2835_dma_test(int channel); // Helper function for full DMA BEBUG + INFO
void bcm2835_check_cblk(struct dma_cb *cblk);

// DMA RUN CONTROL TAKES 100 us to complete!!
void bcm2835_solo_dma_start(int channel,
                            uint32_t cblk_phyaddrss); // resets the DMA and
                                                      // initializes with the
                                                      // given control block
void bcm2835_solo_dma_end(int channel);               // resets the DMA
void bcm2835_dual_dma_start(int channel_src, uint32_t cblk_src_phyaddrss,
                            int channel_dst,
                            uint32_t cblk_dst_phyaddrss); // resets two DMAs and
                                                          // initializes them
                                                          // with the given
                                                          // control blocks
void bcm2835_dual_dma_end(int channel_src, int channel_dst); // resets two DMAs

// DMA Clean up funtions
void bcm2835_solo_dma_clean_up(int channel, uint32_t *data, uint32_t *cblk,
                               struct UncachedMemBlock *data_ptr,
                               struct UncachedMemBlock *cblk_ptr);

// DMA Test Setup
void bcm2835_dma_spi_setup(struct dma_cb *cblk_src, struct dma_cb *cblk_dst,
                           uint32_t transfer_size); // Helper function to setup
                                                    // a simple spi DMA
                                                    // operation
void bcm2835_dma_i2s_setup(struct dma_cb *cblk_src, struct dma_cb *cblk_dst,
                           uint32_t transfer_size); // Helper function to setup
                                                    // a simple i2s DMA
                                                    // operation

// SPI Utility Functions
void bcm2835_spi_config_for_dma(
    uint32_t CLOCK_DIVIDER); // Configures the SPI register for DMA access
uint32_t bcm2835_dma_get_length(int channel);
uint32_t bcm2835_dma_get_cblk(int channel);

#ifdef __cplusplus
}
#endif

#endif /* BCM2835_DMA_H */
