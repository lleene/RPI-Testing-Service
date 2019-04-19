//
// C lib Proceedures for servicing i2s/spi/dma/pwm/gpclk on RPI3 over LAN
// Using bcm2835 C lib
//
// 2019 Lieuwe B. Leene
//

#ifndef __BCM2835_SPI_H__
#define __BCM2835_SPI_H__

#include <bcm2835.h>
#include <inttypes.h>
#include <stdio.h>
#include <sys/mman.h>

#define BCM2835_SPI_BASE 0x204000 // 0x7E204000

#define spi_miso RPI_V2_GPIO_P1_21 // ALT0
#define spi_mosi RPI_V2_GPIO_P1_19 // ALT0
#define spi_ce0 RPI_V2_GPIO_P1_24  // ALT0
#define spi_ce1 RPI_V2_GPIO_P1_26  // ALT0
#define spi_clk RPI_V2_GPIO_P1_23  // ALT0

#ifdef __cplusplus
extern "C" {
#endif

int bcm2835_spi_init();
void bcm2835_spi_enable_output();
void bcm2835_spi_disable_output();
void bcm2835_spi_master_setup();

#ifdef __cplusplus
}
#endif

#endif /* BCM2835_SPI_H */
