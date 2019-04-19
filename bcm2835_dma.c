//
// C lib Proceedures for servicing i2s/spi/dma/pwm/gpclk on RPI3 over LAN
// Using bcm2835 C lib
//
// 2019 Lieuwe B. Leene
//

#include "bcm2835_dma.h"

static int mbox_fd = -1; // used internally by the UncachedMemBlock-functions.
extern uint32_t *bcm2835_peripherals;
volatile uint32_t *bcm2835_dma = (uint32_t *)MAP_FAILED;

void *mapmem(unsigned base, unsigned size) {
  int mem_fd;
  unsigned offset = base % BCM2835_PAGE_SIZE;
  base = base - offset;
  /* open /dev/mem */
  if ((mem_fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
    printf("can't open /dev/mem\nThis program should be run as root. Try "
           "prefixing command with: sudo\n");
    exit(-1);
  }
  void *mem = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED /*|MAP_FIXED*/,
                   mem_fd, base);
#ifdef DEBUG
  printf("base=0x%x, mem=%p\n", base, mem);
#endif
  if (mem == MAP_FAILED) {
    printf("mmap error %d\n", (int)mem);
    exit(-1);
  }
  close(mem_fd);
  return (char *)mem + offset;
}

void unmapmem(void *addr, unsigned size) {
  int s = munmap(addr, size);
  if (s != 0) {
    printf("munmap error %d\n", s);
    exit(-1);
  }
}

/*
 * use ioctl to send mbox property message
 */

static int mbox_property(int file_desc, void *buf) {
  int ret_val = ioctl(file_desc, IOCTL_MBOX_PROPERTY, buf);

  if (ret_val < 0) {
    printf("ioctl_set_msg failed:%d\n", ret_val);
  }

#ifdef DEBUG
  unsigned *p = buf;
  int i;
  unsigned size = *(unsigned *)buf;
  for (i = 0; i < size / 4; i++)
    printf("%04x: 0x%08x\n", i * sizeof *p, p[i]);
#endif
  return ret_val;
}

unsigned mem_alloc(int file_desc, unsigned size, unsigned align,
                   unsigned flags) {
  int i = 0;
  unsigned p[32];
  p[i++] = 0;          // size
  p[i++] = 0x00000000; // process request

  p[i++] = 0x3000c; // (the tag id)
  p[i++] = 12;      // (size of the buffer)
  p[i++] = 12;      // (size of the data)
  p[i++] = size;    // (num bytes? or pages?)
  p[i++] = align;   // (alignment)
  p[i++] = flags;   // (MEM_FLAG_L1_NONALLOCATING)

  p[i++] = 0x00000000;  // end tag
  p[0] = i * sizeof *p; // actual size

  mbox_property(file_desc, p);
  return p[5];
}

unsigned mem_free(int file_desc, unsigned handle) {
  int i = 0;
  unsigned p[32];
  p[i++] = 0;          // size
  p[i++] = 0x00000000; // process request

  p[i++] = 0x3000f; // (the tag id)
  p[i++] = 4;       // (size of the buffer)
  p[i++] = 4;       // (size of the data)
  p[i++] = handle;

  p[i++] = 0x00000000;  // end tag
  p[0] = i * sizeof *p; // actual size

  mbox_property(file_desc, p);
  return p[5];
}

unsigned mem_lock(int file_desc, unsigned handle) {
  int i = 0;
  unsigned p[32];
  p[i++] = 0;          // size
  p[i++] = 0x00000000; // process request

  p[i++] = 0x3000d; // (the tag id)
  p[i++] = 4;       // (size of the buffer)
  p[i++] = 4;       // (size of the data)
  p[i++] = handle;

  p[i++] = 0x00000000;  // end tag
  p[0] = i * sizeof *p; // actual size

  mbox_property(file_desc, p);
  return p[5];
}

unsigned mem_unlock(int file_desc, unsigned handle) {
  int i = 0;
  unsigned p[32];
  p[i++] = 0;          // size
  p[i++] = 0x00000000; // process request

  p[i++] = 0x3000e; // (the tag id)
  p[i++] = 4;       // (size of the buffer)
  p[i++] = 4;       // (size of the data)
  p[i++] = handle;

  p[i++] = 0x00000000;  // end tag
  p[0] = i * sizeof *p; // actual size

  mbox_property(file_desc, p);
  return p[5];
}

int mbox_open() {
  int file_desc;

  // open a char device file used for communicating with kernel mbox driver
  file_desc = open(DEVICE_FILE_NAME, 0);
  if (file_desc < 0) {
    printf("Can't open device file: %s\n", DEVICE_FILE_NAME);
    printf("Try creating a device file with: sudo mknod %s c %d 0\n",
           DEVICE_FILE_NAME, MAJOR_NUM);
    exit(-1);
  }
  return file_desc;
}

void mbox_close(int file_desc) { close(file_desc); }

// Allocate a block of memory of the given size (which is rounded up to the next
// full page). The memory will be aligned on a page boundary and zeroed out.
struct UncachedMemBlock UncachedMemBlock_alloc(unsigned size) {
  if (mbox_fd < 0) {
    mbox_fd = mbox_open();
    assert(mbox_fd >= 0); // Uh, /dev/vcio not there ?
  }
  // Round up to next full page.
  size = size % BCM2835_PAGE_SIZE == 0 ? size : (size + BCM2835_PAGE_SIZE) &
                                                    ~(BCM2835_PAGE_SIZE - 1);

  struct UncachedMemBlock result;
  result.size = size;
  result.mem_handle =
      mem_alloc(mbox_fd, size, BCM2835_PAGE_SIZE, MEM_FLAG_L1_NONALLOCATING);
  result.bus_addr = mem_lock(mbox_fd, result.mem_handle);
  result.mem = mapmem(BUS_TO_PHYS(result.bus_addr), size);
  fprintf(stderr, "Alloc: %6d bytes;  %p (bus=0x%08x, phys=0x%08x)\n",
          (int)size, result.mem, result.bus_addr, BUS_TO_PHYS(result.bus_addr));
  assert(result.bus_addr); // otherwise: couldn't allocate contiguous block.
  memset(result.mem, 0x00, size);

  return result;
}

// Free block previously allocated with UncachedMemBlock_alloc()
void UncachedMemBlock_free(struct UncachedMemBlock *block) {
  if (block->mem == NULL)
    return;
  assert(mbox_fd >= 0); // someone should've initialized that on allocate.
  unmapmem(block->mem, block->size);
  mem_unlock(mbox_fd, block->mem_handle);
  mem_free(mbox_fd, block->mem_handle);
  block->mem = NULL;
}

// Given a pointer to memory that is in the allocated block, return the
// physical bus address needed by DMA operations.
uint32_t UncachedMemBlock_to_physical(const struct UncachedMemBlock *blk,
                                      void *p) {
  uint32_t offset = (uint8_t *)p - (uint8_t *)blk->mem;
  assert(offset < blk->size); // pointer not within our block.
  return blk->bus_addr + offset;
}

// sets the pointers for accessing SPI & DMA
int bcm2835_dma_init(void) {
  if (bcm2835_peripherals == MAP_FAILED) {
    fprintf(stderr,
            "bcm2835 library must be initialized before calling dma_init");
    return 0; // bcm2835_init() failed, or not root //
  }
  bcm2835_dma = bcm2835_peripherals + BCM2835_DMA_BASE / 4;
  return 1; // success //
}

void bcm2835_solo_dma_start(int channel, uint32_t cblk_phyaddrss) {
  bcm2835_peri_set_bits(bcm2835_dma + BCM2835_DMA(channel) / 4,
                        BCM2835_DMA_CS_ABORT,
                        BCM2835_DMA_CS_ABORT); // Abort Current Operation
  bcm2835_delayMicroseconds(100);              // Wait for abort
  bcm2835_peri_set_bits(bcm2835_dma + BCM2835_DMA(channel) / 4,
                        BCM2835_DMA_CS_RESET,
                        BCM2835_DMA_CS_RESET); // Reset Current Operation
  bcm2835_peri_set_bits(bcm2835_dma + BCM2835_DMA(channel) / 4,
                        BCM2835_DMA_CS_END,
                        BCM2835_DMA_CS_END); // Clear End bit
  bcm2835_peri_set_bits(bcm2835_dma + BCM2835_DMA(channel) / 4,
                        BCM2835_DMA_DEBUG_CLR,
                        BCM2835_DMA_DEBUG_CLR); // Clear Debug Status

  // DMA Activation (Write to CB in CS Register according to p158 in BCM2835 ARM
  // Peripherals)
  bcm2835_peri_write(bcm2835_dma + BCM2835_DMA_CB(channel) / 4,
                     cblk_phyaddrss); // Write physical address of CB1 for fetch
                                      // but must be uncached
  bcm2835_peri_set_bits(bcm2835_dma + BCM2835_DMA(channel) / 4,
                        BCM2835_DMA_CS_ENABLE,
                        BCM2835_DMA_CS_ENABLE); // ENABLE DMA
}

void bcm2835_solo_dma_end(int channel) {
  bcm2835_peri_set_bits(bcm2835_dma + BCM2835_DMA(channel) / 4,
                        BCM2835_DMA_CS_ABORT,
                        BCM2835_DMA_CS_ABORT); // Abort Current Operation
  bcm2835_delayMicroseconds(100);              // Wait for abort
  bcm2835_peri_set_bits(bcm2835_dma + BCM2835_DMA(channel) / 4, 0x0,
                        BCM2835_DMA_CS_ENABLE); // ENABLE DMA
  bcm2835_peri_set_bits(bcm2835_dma + BCM2835_DMA(channel) / 4,
                        BCM2835_DMA_CS_RESET,
                        BCM2835_DMA_CS_RESET); // Reset Current Operation
}

void bcm2835_dual_dma_start(int channel_src, uint32_t cblk_src_phyaddrss,
                            int channel_dst, uint32_t cblk_dst_phyaddrss) {
  bcm2835_peri_set_bits(bcm2835_dma + BCM2835_DMA(channel_src) / 4,
                        BCM2835_DMA_CS_ABORT,
                        BCM2835_DMA_CS_ABORT); // Abort Current Operation
  bcm2835_peri_set_bits(bcm2835_dma + BCM2835_DMA(channel_dst) / 4,
                        BCM2835_DMA_CS_ABORT,
                        BCM2835_DMA_CS_ABORT); // Abort Current Operation
  bcm2835_delayMicroseconds(100);              // Wait for abort
  bcm2835_peri_set_bits(bcm2835_dma + BCM2835_DMA(channel_src) / 4,
                        BCM2835_DMA_CS_RESET,
                        BCM2835_DMA_CS_RESET); // Reset Current Operation
  bcm2835_peri_set_bits(bcm2835_dma + BCM2835_DMA(channel_dst) / 4,
                        BCM2835_DMA_CS_RESET,
                        BCM2835_DMA_CS_RESET); // Reset Current Operation
  bcm2835_peri_set_bits(bcm2835_dma + BCM2835_DMA(channel_src) / 4,
                        BCM2835_DMA_CS_END,
                        BCM2835_DMA_CS_END); // Clear End bit
  bcm2835_peri_set_bits(bcm2835_dma + BCM2835_DMA(channel_dst) / 4,
                        BCM2835_DMA_CS_END,
                        BCM2835_DMA_CS_END); // Clear End bit
  bcm2835_peri_set_bits(bcm2835_dma + BCM2835_DMA(channel_src) / 4,
                        BCM2835_DMA_DEBUG_CLR,
                        BCM2835_DMA_DEBUG_CLR); // Clear Debug Status
  bcm2835_peri_set_bits(bcm2835_dma + BCM2835_DMA(channel_dst) / 4,
                        BCM2835_DMA_DEBUG_CLR,
                        BCM2835_DMA_DEBUG_CLR); // Clear Debug Status

  // DMA Activation (Write to CB in CS Register according to p158 in BCM2835 ARM
  // Peripherals)
  bcm2835_peri_write(bcm2835_dma + BCM2835_DMA_CB(channel_src) / 4,
                     cblk_src_phyaddrss); // Write physical address of CB1 for
                                          // fetch but must be uncached
  bcm2835_peri_write(bcm2835_dma + BCM2835_DMA_CB(channel_dst) / 4,
                     cblk_dst_phyaddrss); // Write physical address of CB1 for
                                          // fetch but must be uncached
  bcm2835_peri_set_bits(bcm2835_dma + BCM2835_DMA(channel_src) / 4,
                        BCM2835_DMA_CS_ENABLE,
                        BCM2835_DMA_CS_ENABLE); // ENABLE DMA
  bcm2835_peri_set_bits(bcm2835_dma + BCM2835_DMA(channel_dst) / 4,
                        BCM2835_DMA_CS_ENABLE,
                        BCM2835_DMA_CS_ENABLE); // ENABLE DMA
}

void bcm2835_dual_dma_end(int channel_src, int channel_dst) {
  bcm2835_peri_set_bits(bcm2835_dma + BCM2835_DMA(channel_src) / 4,
                        BCM2835_DMA_CS_ABORT,
                        BCM2835_DMA_CS_ABORT); // Abort Current Operation
  bcm2835_peri_set_bits(bcm2835_dma + BCM2835_DMA(channel_dst) / 4,
                        BCM2835_DMA_CS_ABORT,
                        BCM2835_DMA_CS_ABORT); // Abort Current Operation
  bcm2835_delayMicroseconds(100);              // Wait for abort
  bcm2835_peri_set_bits(bcm2835_dma + BCM2835_DMA(channel_src) / 4, 0x0,
                        BCM2835_DMA_CS_ENABLE); // ENABLE DMA
  bcm2835_peri_set_bits(bcm2835_dma + BCM2835_DMA(channel_src) / 4,
                        BCM2835_DMA_CS_RESET,
                        BCM2835_DMA_CS_RESET); // Reset Current Operation
  bcm2835_peri_set_bits(bcm2835_dma + BCM2835_DMA(channel_dst) / 4, 0x0,
                        BCM2835_DMA_CS_ENABLE); // ENABLE DMA
  bcm2835_peri_set_bits(bcm2835_dma + BCM2835_DMA(channel_dst) / 4,
                        BCM2835_DMA_CS_RESET,
                        BCM2835_DMA_CS_RESET); // Reset Current Operation
}

// BCM2835_DMA_CB_TI_NO_WIDE_BURSTS May not be required &&
// BCM2835_DMA_CB_TI_WAIT_RESP may not be required
void bcm2835_dma_spi_setup(struct dma_cb *cblk_src, struct dma_cb *cblk_dst,
                           uint32_t transfer_size) {
  cblk_src->info =
      BCM2835_DMA_CB_TI_PERMAP_TXSPI | BCM2835_DMA_CB_TI_DEST_DREQ |
      BCM2835_DMA_CB_TI_SRC_INC | BCM2835_DMA_CB_TI_WAIT_RESP |
      BCM2835_DMA_CB_TI_NO_WIDE_BURSTS | BCM2835_DMA_CB_TI_SRC_128b;
  cblk_src->src = 0x0;
  cblk_src->dst = SPI_FIFO_PHYLOC;
  cblk_src->length = transfer_size + 1;
  cblk_src->stride = 0x00000000;
  cblk_src->next = 0x00000000;
  cblk_src->pad[0] = 0x00000000;
  cblk_src->pad[1] = 0x00000000;

  cblk_dst->info = BCM2835_DMA_CB_TI_PERMAP_RXSPI | BCM2835_DMA_CB_TI_SRC_DREQ |
                   BCM2835_DMA_CB_TI_DEST_INC | BCM2835_DMA_CB_TI_WAIT_RESP |
                   BCM2835_DMA_CB_TI_NO_WIDE_BURSTS |
                   BCM2835_DMA_CB_TI_DEST_128b;
  cblk_dst->src = SPI_FIFO_PHYLOC;
  cblk_dst->dst = 0x0;
  cblk_dst->length = transfer_size;
  cblk_dst->stride = 0x00000000;
  cblk_dst->next = 0x00000000;
  cblk_dst->pad[0] = 0x00000000;
  cblk_dst->pad[1] = 0x00000000;
}

// BCM2835_DMA_CB_TI_NO_WIDE_BURSTS Required && BCM2835_DMA_CB_TI_WAIT_RESP may
// not be required
void bcm2835_dma_i2s_setup(struct dma_cb *cblk_src, struct dma_cb *cblk_dst,
                           uint32_t transfer_size) {
  cblk_src->info = BCM2835_DMA_CB_TI_PERMAP_TXPCM |
                   BCM2835_DMA_CB_TI_DEST_DREQ | BCM2835_DMA_CB_TI_SRC_INC |
                   BCM2835_DMA_CB_TI_WAIT_RESP | BCM2835_DMA_CB_TI_SRC_128b |
                   BCM2835_DMA_CB_TI_NO_WIDE_BURSTS;
  cblk_src->src = 0x0;
  cblk_src->dst = I2S_FIFO_PHYLOC;
  cblk_src->length = transfer_size;
  cblk_src->stride = 0x00000000;
  cblk_src->next = 0x00000000;
  cblk_src->pad[0] = 0x00000000;
  cblk_src->pad[1] = 0x00000000;

  cblk_dst->info = BCM2835_DMA_CB_TI_PERMAP_RXPCM | BCM2835_DMA_CB_TI_SRC_DREQ |
                   BCM2835_DMA_CB_TI_DEST_INC | BCM2835_DMA_CB_TI_WAIT_RESP |
                   BCM2835_DMA_CB_TI_DEST_128b |
                   BCM2835_DMA_CB_TI_NO_WIDE_BURSTS;
  cblk_dst->src = I2S_FIFO_PHYLOC;
  cblk_dst->dst = 0x0;
  cblk_dst->length = transfer_size;
  cblk_dst->stride = 0x00000000;
  cblk_dst->next = 0x00000000;
  cblk_dst->pad[0] = 0x00000000;
  cblk_dst->pad[1] = 0x00000000;
}

void bcm2835_dma_status() {
  fprintf(stdout, "DMA Enable Info: %p\n",
          (void *)(bcm2835_peri_read(bcm2835_dma + BCM2835_DMA_ENABLE / 4)));
  int i;
  for (i = 0; i < 16; i++) {
    if (bcm2835_peri_read(bcm2835_dma + BCM2835_DMA(i) / 4) &
        BCM2835_DMA_CS_ENABLE) {
      fprintf(stderr, "DMA channel %d is running\n", i);
    }
  }
}

// Helper function to check the DMA Debug Internals
void bcm2835_dma_debug(int channel) {
  volatile uint32_t *paddr;
  paddr = bcm2835_dma + BCM2835_DMA(channel) / 4; // Config Register
  uint32_t stat = bcm2835_peri_read(paddr);
  fprintf(stdout, "DMA Status Info: %p\n", (void *)stat);
  if (stat & BCM2835_DMA_CS_INTRPT)
    fprintf(stderr, "Interrupt \n");
  if (stat & BCM2835_DMA_CS_ERROR)
    fprintf(stderr, "Error \n");
  if (stat & BCM2835_DMA_CS_STOP)
    fprintf(stderr, "No DREQ \n");
  if (stat & BCM2835_DMA_CS_PAUSE)
    fprintf(stderr, "Paused \n");
  if (stat & BCM2835_DMA_CS_DREQ)
    fprintf(stderr, "Inactive \n");
  if (stat & BCM2835_DMA_CS_ABORT)
    fprintf(stderr, "Aborted \n");
  if (stat & BCM2835_DMA_CS_WAIT)
    fprintf(stderr, "Waiting \n");
  if (stat & BCM2835_DMA_CS_ENABLE)
    fprintf(stderr, "DMA Enabled \n");
  if (stat & BCM2835_DMA_CS_END)
    fprintf(stderr, "DMA has Ended \n");

  paddr = bcm2835_dma + BCM2835_DMA_DB(channel) / 4; // Debug Register
  uint32_t dbug = bcm2835_peri_read(paddr);
  fprintf(stdout, "DMA Debug Info: %p\n", (void *)dbug);
  if (stat & BCM2835_DMA_DEBUG_LITE)
    fprintf(stderr, "LITE Enabled \n");
  if (stat & BCM2835_DMA_DEBUG_RDERR)
    fprintf(stderr, "Read Error \n");
  if (stat & BCM2835_DMA_DEBUG_FIFOERR)
    fprintf(stderr, "FIFO Error \n");
}

void bcm2835_check_cblk(struct dma_cb *cblk) {
  uint32_t ctrl = cblk->info;
  fprintf(stdout, "CBLK Control Info: %p\n", (void *)ctrl);
  if (ctrl & BCM2835_DMA_CB_TI_NO_WIDE_BURSTS)
    fprintf(stderr, "No Wide Bursts \n");
  if (ctrl & BCM2835_DMA_CB_TI_WAIT_RESP)
    fprintf(stderr, "Wait for AXI \n");
  if (ctrl & BCM2835_DMA_CB_TI_WAITS(0x1F))
    fprintf(stderr, "Extra Wait Cycles %p\n", (void *)((ctrl >> 21) & 0x1F));
  if (ctrl & BCM2835_DMA_CB_TI_PERMAP_MASK)
    fprintf(stderr, "DREQ Peripheral %p\n", (void *)((ctrl >> 16) & 0x1F));
  if (ctrl & BCM2835_DMA_CB_TI_SRC_IGR)
    fprintf(stderr, "Ignore Source Reads \n");
  if (ctrl & BCM2835_DMA_CB_TI_SRC_DREQ)
    fprintf(stderr, "Source DREQ \n");
  if (ctrl & BCM2835_DMA_CB_TI_SRC_128b)
    fprintf(stderr, "Source is 128b Wide \n");
  if (ctrl & BCM2835_DMA_CB_TI_SRC_INC)
    fprintf(stderr, "Inc. Source \n");
  if (ctrl & BCM2835_DMA_CB_TI_DEST_IGR)
    fprintf(stderr, "Ignore Destination Reads \n");
  if (ctrl & BCM2835_DMA_CB_TI_DEST_DREQ)
    fprintf(stderr, "Destination DREQ \n");
  if (ctrl & BCM2835_DMA_CB_TI_DEST_128b)
    fprintf(stderr, "Destination is 128b Wide \n");
  if (ctrl & BCM2835_DMA_CB_TI_DEST_INC)
    fprintf(stderr, "Inc. Destination \n");
  if (ctrl & BCM2835_DMA_CB_TI_TDMODE)
    fprintf(stderr, "Strides Enabled \n");
  if (ctrl & BCM2835_DMA_CB_TI_INTRPT)
    fprintf(stderr, "Interrupt on Complete \n");

  uint32_t leng = cblk->length;
  fprintf(stdout, "CBLK Transfer Length: %p\n", (void *)leng);
  uint32_t dest = cblk->dst;
  fprintf(stdout, "CBLK Destination Add: %p\n", (void *)dest);
  uint32_t src = cblk->src;
  fprintf(stdout, "CBLK Source Add: %p\n", (void *)src);
}

uint32_t bcm2835_dma_get_length(int channel) {
  volatile uint32_t *paddr;
  paddr = bcm2835_dma + BCM2835_DMA_LN(channel) / 4;
  return bcm2835_peri_read(paddr); // Read Length Register
}

uint32_t bcm2835_dma_get_cblk(int channel) {
  volatile uint32_t *paddr;
  paddr = bcm2835_dma + BCM2835_DMA_CB(channel) / 4;
  return bcm2835_peri_read(paddr); // Read Next Register
}

// Helper function to check the DMA info
void bcm2835_dma_info(int channel) // this is buggy for some reason
{
  volatile uint32_t *paddr;
  paddr = bcm2835_dma + BCM2835_DMA_CB(channel) / 4; // Control Register
  uint32_t ctrl = bcm2835_peri_read(paddr);
  fprintf(stdout, "DMA Control Info: 0x%08x\n", ctrl);
  if (ctrl & BCM2835_DMA_CB_TI_NO_WIDE_BURSTS)
    fprintf(stderr, "No Wide Bursts \n");
  if (ctrl & BCM2835_DMA_CB_TI_WAIT_RESP)
    fprintf(stderr, "Wait for AXI \n");
  if (ctrl & BCM2835_DMA_CB_TI_WAIT_MASK)
    fprintf(stderr, "Extra Wait Cycles %p\n",
            (void *)(ctrl & BCM2835_DMA_CB_TI_WAIT_MASK));
  if (ctrl & BCM2835_DMA_CB_TI_PERMAP_MASK)
    fprintf(stderr, "DREQ Peripheral %p\n",
            (void *)(ctrl & BCM2835_DMA_CB_TI_PERMAP_MASK));
  if (ctrl & BCM2835_DMA_CB_TI_SRC_IGR)
    fprintf(stderr, "Ignore Source Reads \n");
  if (ctrl & BCM2835_DMA_CB_TI_SRC_DREQ)
    fprintf(stderr, "Source DREQ \n");
  if (ctrl & BCM2835_DMA_CB_TI_SRC_128b)
    fprintf(stderr, "Source is 128b Wide \n");
  if (ctrl & BCM2835_DMA_CB_TI_SRC_INC)
    fprintf(stderr, "Inc. Source \n");
  if (ctrl & BCM2835_DMA_CB_TI_DEST_IGR)
    fprintf(stderr, "Ignore Destination Reads \n");
  if (ctrl & BCM2835_DMA_CB_TI_DEST_DREQ)
    fprintf(stderr, "Destination DREQ \n");
  if (ctrl & BCM2835_DMA_CB_TI_DEST_128b)
    fprintf(stderr, "Destination is 128b Wide \n");
  if (ctrl & BCM2835_DMA_CB_TI_DEST_INC)
    fprintf(stderr, "Inc. Destination \n");
  if (ctrl & BCM2835_DMA_CB_TI_TDMODE)
    fprintf(stderr, "Strides Enabled \n");
  if (ctrl & BCM2835_DMA_CB_TI_INTRPT)
    fprintf(stderr, "Interrupt on Complete \n");

  paddr = bcm2835_dma + BCM2835_DMA_LN(channel) / 4; // Length Register
  uint32_t leng = bcm2835_peri_read(paddr);
  fprintf(stdout, "DMA Transfer Length: %p\n", (void *)leng);
  paddr = bcm2835_dma + BCM2835_DMA_DA(channel) / 4; // Length Register
  uint32_t dest = bcm2835_peri_read(paddr);
  fprintf(stdout, "DMA Destination Add: %p\n", (void *)dest);
  paddr = bcm2835_dma + BCM2835_DMA_SA(channel) / 4; // Length Register
  uint32_t src = bcm2835_peri_read(paddr);
  fprintf(stdout, "DMA Source Add: %p\n", (void *)src);
  paddr = bcm2835_dma + BCM2835_DMA_NB(channel) / 4;
  uint32_t next = bcm2835_peri_read(paddr);
  fprintf(stdout, "DMA Next BLK Add: %p\n", (void *)next);
}

void bcm2835_dma_test(int channel) {
  bcm2835_dma_debug(channel);
  bcm2835_dma_info(channel);
}

void bcm2835_solo_dma_clean_up(int channel, uint32_t *data, uint32_t *cblk,
                               struct UncachedMemBlock *data_ptr,
                               struct UncachedMemBlock *cblk_ptr) {
  bcm2835_solo_dma_end(channel); // end channel
  if (data)
    UncachedMemBlock_free(data_ptr); // if allocated free
  if (cblk)
    UncachedMemBlock_free(cblk_ptr); // if allocated free
}
