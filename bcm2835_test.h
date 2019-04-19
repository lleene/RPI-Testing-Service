//
// C lib Proceedures for servicing i2s/spi/dma/pwm/gpclk on RPI3 over LAN
// Using bcm2835 C lib
//
// 2019 Lieuwe B. Leene
//

#ifndef __BCM2835_TEST_H__
#define __BCM2835_TEST_H__

#include <bcm2835.h>
#include <stdio.h>

#include "bcm2835_dma.h"
#include "bcm2835_i2s.h"
#include "sdm_core.h"

// Test & DMA Setup
#define DATA_SEGMENTS 4        // Number of Data segments
#define DMA_PAGE_SIZE (0x3F00) // 0x3FFF is max 8 is min
#define channel_src 11         // 11/13 is lite dma - always available
#define channel_dst 12         // 12/14 is lite dma - always available
// TEST SIZE Cannot Exceed 14b DMA-Lite TXLen for Pre-Compute

// VYGR2 SYSTEM CLK ENABLE PIN
#define i2s_en RPI_V2_GPIO_P1_36 // GP-OUT

// VYGR2 SPI Control Register
#define 	spi_miso 			RPI_V2_GPIO_P1_21 // ALT0
#define 	spi_mosi 			RPI_V2_GPIO_P1_19 // ALT0
#define 	spi_ce0				RPI_V2_GPIO_P1_24 // ALT0
#define 	spi_clk				RPI_V2_GPIO_P1_23 //
#define 	vygr_dt				RPI_V2_GPIO_P1_33 //
#define 	vygr_ck				RPI_V2_GPIO_P1_05 //
#define 	vygr_ax				RPI_V2_GPIO_P1_37 //

// GPIO Allocation using I2S + PWM + SPI
#define gp_clk0 RPI_V2_GPIO_P1_07 // ALT0 - ?
#define gp_clk1 RPI_V2_GPIO_P1_29 // ALT0 - 25MHz Ref
#define gp_clk2 RPI_V2_GPIO_P1_31 // ALT0 - 32.77kHz Ref

#define pwm_clk0 RPI_V2_GPIO_P1_32 // ALT0 Generate Master Bit-CLK
#define pwm_clk1 RPI_V2_GPIO_P1_33 // ALT0 DT Control signal

typedef struct {
  // Configuration & data pointers
  uint32_t *data_src;
  uint32_t *data_dst;
  struct dma_cb *cblk_src;
  struct dma_cb *cblk_dst;

  // Uncached Data Structures
  struct UncachedMemBlock data_memblock_src;
  struct UncachedMemBlock data_memblock_dst;
  struct UncachedMemBlock cblk_memblock_src;
  struct UncachedMemBlock cblk_memblock_dst;
} dma_mem_alloc_t;

#ifdef __cplusplus
extern "C" {
#endif

// Standard Operation
void *bcm2835_test_sdmdma_4ch_proc(void *args);
void *bcm2835_test_sdmdma_1ch_proc(void *args);
void bcm2835_test_configure_akira(char *test_config, syth_cnfg_t *configuration);
void bcm2835_test_start_clk(char* test_config);
void bcm2835_test_stop_clk(char* test_config);

int bcm2835_test_init_cache(dma_mem_alloc_t *dma_alloc_obj);
void bcm2835_test_setup_cblocks(dma_mem_alloc_t *dma_alloc_obj,
                                bool circular_transfer);
void bcm2835_test_start_i2sdma(dma_mem_alloc_t *dma_alloc_obj);
void bcm2835_test_end_transfer(dma_mem_alloc_t *dma_alloc_obj);

// Initialisation Proceedures
int bcm2835_test_VYGR2_init(void);
int bcm2835_test_AKIRA_init(void);
void bcm2835_test_VYGR2_end(void);
void bcm2835_test_stop_dma(dma_mem_alloc_t *dma_alloc_obj);
void bcm2835_test_end_full(dma_mem_alloc_t *dma_alloc_obj);
void bcm2835_test_init_full(void);


// Data Proceedures
void bcm2835_test_generate_segment(dma_mem_alloc_t *dma_alloc_obj, int segment);
void bcm2835_test_print_segment(dma_mem_alloc_t *dma_alloc_obj, int segment);
int bcm2835_test_output_file(dma_mem_alloc_t *dma_alloc_obj,  int segment_offset, int segment_number);
int bcm2835_test_dump_csv(dma_mem_alloc_t *dma_alloc_obj);
int bcm2835_test_sdm2csv(dma_mem_alloc_t *dma_alloc_obj);
void bcm2835_test_print_config(char *test_config);

// Test Proceedures
// void i2s_send_config(char test_config[32]);
// int bcm2835_test_dma_transfer(void);
// int bcm2835_test_frequency_ratio(void);

#ifdef __cplusplus
}
#endif

#endif /* BCM2835_TEST_H */
