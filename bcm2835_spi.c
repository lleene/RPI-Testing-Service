//
// C lib Proceedures for servicing i2s/spi/dma/pwm/gpclk on RPI3 over LAN
// Using bcm2835 C lib
//
// 2019 Lieuwe B. Leene
//

#include "bcm2835_spi.h"

extern uint32_t *bcm2835_peripherals;
volatile uint32_t *bcm2835_spi = (uint32_t *)MAP_FAILED;

int bcm2835_spi_init() {
  if (bcm2835_peripherals == MAP_FAILED) {
    fprintf(stderr,
            "bcm2835 library must be initialized before calling dma_init");
    return 0; // bcm2835_init() failed, or not root //
  }
  bcm2835_spi = bcm2835_peripherals + BCM2835_SPI_BASE / 4;

  /* Reset registers - just to make sure */
  // bcm2835_peri_write(bcm2835_spi + BCM2835_I2S_MODE_A/4, 0x0); TBC
  bcm2835_delayMicroseconds(10); // wait for data load
  return 1;
}

void bcm2835_spi_enable_output() {
  // Configure SPI outputs
  bcm2835_gpio_fsel(spi_miso, BCM2835_GPIO_FSEL_ALT0);
  bcm2835_gpio_fsel(spi_mosi, BCM2835_GPIO_FSEL_ALT0);
  bcm2835_gpio_fsel(spi_ce0, BCM2835_GPIO_FSEL_ALT0);
  bcm2835_gpio_fsel(spi_ce1, BCM2835_GPIO_FSEL_ALT0);
  bcm2835_gpio_fsel(spi_clk, BCM2835_GPIO_FSEL_ALT0);
}

void bcm2835_spi_disable_output() {
  // Configure SPI outputs
  bcm2835_gpio_fsel(spi_miso, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(spi_mosi, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(spi_ce0, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(spi_ce1, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(spi_clk, BCM2835_GPIO_FSEL_OUTP);
}

void bcm2835_spi_master_setup() {
  // Configure SPI Settings
  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_16);
  bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
  bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);
}
