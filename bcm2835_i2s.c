//
// C lib Proceedures for servicing i2s/spi/dma/pwm/gpclk on RPI3 over LAN
// Using bcm2835 C lib
//
// 2019 Lieuwe B. Leene
//

#include "bcm2835_i2s.h"

extern uint32_t *bcm2835_peripherals;
volatile uint32_t *bcm2835_i2s = (uint32_t *)MAP_FAILED;

int bcm2835_i2s_init() {
  if (bcm2835_peripherals == MAP_FAILED) {
    fprintf(stderr,
            "bcm2835 library must be initialized before calling dma_init");
    return 0; // bcm2835_init() failed, or not root //
  }
  bcm2835_i2s = bcm2835_peripherals + BCM2835_I2S_BASE / 4;

  /* Reset registers - just to make sure */
  bcm2835_peri_write(bcm2835_i2s + BCM2835_I2S_MODE_A / 4, 0x0);
  bcm2835_delayMicroseconds(10); // wait for data load
  return 1;
}

void bcm2835_i2s_enable_output() {
  bcm2835_gpio_fsel(i2s_bclk, BCM2835_GPIO_FSEL_ALT0);
  bcm2835_gpio_fsel(i2s_fclk, BCM2835_GPIO_FSEL_ALT0);
  bcm2835_gpio_fsel(i2s_dout, BCM2835_GPIO_FSEL_ALT0);
  bcm2835_gpio_fsel(i2s_din, BCM2835_GPIO_FSEL_ALT0);
}

void bcm2835_i2s_disable_output() {
  bcm2835_gpio_fsel(i2s_bclk, BCM2835_GPIO_FSEL_INPT);
  bcm2835_gpio_fsel(i2s_fclk, BCM2835_GPIO_FSEL_INPT);
  bcm2835_gpio_fsel(i2s_dout, BCM2835_GPIO_FSEL_INPT);
  bcm2835_gpio_fsel(i2s_din, BCM2835_GPIO_FSEL_INPT);
}

void bcm2835_i2s_8bslave_setup() {
  bcm2835_i2s_clock_config(
      BCM2835_I2S_INV_EXTERNAL_CLK); // Safe Setup for rising EDGE Clock Master
  bcm2835_i2s_frame_config(BCM2835_I2S_EXTERNAL_FS, 8,
                           4); // uint32_t frame_config, int flen, int fslen
  bcm2835_i2s_config_transmit(BCM2835_I2S_CHANNEL1, 8, 0,
                              0); // uint32_t channel_config, int length, int
                                  // channel1_pos, int channel2_pos
  bcm2835_i2s_config_recieve(BCM2835_I2S_CHANNEL2, 8, 0,
                             0); // uint32_t channel_config, int length, int
                                 // channel1_pos, int channel2_pos
  bcm2835_i2s_dma_en();
}

void bcm2835_i2s_pwm_setup(uint32_t frame_length) {
  bcm2835_i2s_clock_config(
      BCM2835_I2S_INV_EXTERNAL_CLK); // Safe Setup for rising EDGE Clock Master
  bcm2835_i2s_frame_config(BCM2835_I2S_INTERNAL_FS, frame_length, (frame_length>>1)); // uint32_t frame_config, int flen, int fslen
  bcm2835_i2s_config_transmit(BCM2835_I2S_CHANNEL1, frame_length, 0,
                              0); // uint32_t channel_config, int length, int
                                  // channel1_pos, int channel2_pos
  bcm2835_i2s_config_recieve(BCM2835_I2S_CHANNEL2, frame_length, 0,
                             0); // uint32_t channel_config, int length, int
                                 // channel1_pos, int channel2_pos
  bcm2835_i2s_dma_en();
  bcm2835_i2s_start();
  bcm2835_i2c_write('\0',0);
}

void bcm2835_i2s_8bpwm_setup() {
  bcm2835_i2s_clock_config(
      BCM2835_I2S_INV_EXTERNAL_CLK); // Safe Setup for rising EDGE Clock Master
  bcm2835_i2s_frame_config(BCM2835_I2S_INTERNAL_FS, 8, 4); // uint32_t frame_config, int flen, int fslen
  bcm2835_i2s_config_transmit(BCM2835_I2S_CHANNEL1, 8, 0,
                              0); // uint32_t channel_config, int length, int
                                  // channel1_pos, int channel2_pos
  bcm2835_i2s_config_recieve(BCM2835_I2S_CHANNEL2, 8, 0,
                             0); // uint32_t channel_config, int length, int
                                 // channel1_pos, int channel2_pos
  bcm2835_i2s_dma_en();
  bcm2835_i2s_start();
  bcm2835_i2c_write('\0',0);
}


void bcm2835_i2s_clock_config(
    uint32_t clock_config) // configure I2S clock control
{
  uint32_t mode_a = bcm2835_peri_read(bcm2835_i2s + BCM2835_I2S_MODE_A / 4);
  mode_a &= ~(BCM2835_I2S_MODEA_CLKM_MASK |
              BCM2835_I2S_MODEA_CLKI_MASK); // Clear Current Config
  mode_a |= ((clock_config << BCM2835_I2S_MODEA_CLK_OFFSET) &
             (BCM2835_I2S_MODEA_CLKM_MASK |
              BCM2835_I2S_MODEA_CLKI_MASK)); // Set New Config
  bcm2835_peri_write(bcm2835_i2s + BCM2835_I2S_MODE_A / 4, mode_a);
  // fprintf(stderr,"Mode A Info: %p\n", (void*)mode_a);
}

void bcm2835_i2s_frame_config(uint32_t frame_config, int flen,
                              int fslen) // configure I2S clock control
{
  /* set MODE_A */
  uint32_t mode_a = bcm2835_peri_read(bcm2835_i2s + BCM2835_I2S_MODE_A / 4);
  mode_a &= ~(BCM2835_I2S_MODEA_FSM_MASK |
              BCM2835_I2S_MODEA_FSI_MASK); // Clear Current Config
  mode_a &= ~BCM2835_I2S_MODEA_FSLEN_MASK;
  mode_a &= ~BCM2835_I2S_MODEA_FLEN_MASK;
  mode_a |= ((frame_config << BCM2835_I2S_MODEA_FS_OFFSET) &
             (BCM2835_I2S_MODEA_FSM_MASK |
              BCM2835_I2S_MODEA_FSI_MASK)); // Set New Config
  mode_a |=
      (fslen << BCM2835_I2S_MODEA_FSLEN_OFFSET) & BCM2835_I2S_MODEA_FSLEN_MASK;
  mode_a |= ((flen - 1) << BCM2835_I2S_MODEA_FLEN_OFFSET) &
            BCM2835_I2S_MODEA_FLEN_MASK;
  bcm2835_peri_write(bcm2835_i2s + BCM2835_I2S_MODE_A / 4, mode_a);
  // fprintf(stderr,"Mode A Info: %p\n", (void*)mode_a);
}

void bcm2835_i2s_config_transmit(uint32_t channel_config, int length,
                                 int channel1_pos, int channel2_pos) {
  if (length < 8)
    length = 8;
  /* set TXC_A */
  uint32_t txc_a = 0x0; // No need to load - we are going to change everything
  uint32_t ch_wid = (length - 8) & 0xf;
  uint32_t ch_wex = ((length - 8) & (0x10)) >> 4;

  if (ch_wex) {
    txc_a |= BCM2835_I2S_TXCA_CH1WEX_MASK;
    txc_a |= BCM2835_I2S_TXCA_CH2WEX_MASK;
  }

  if (channel_config & BCM2835_I2S_CHANNEL1)
    txc_a |= (ch_wid << BCM2825_I2S_TXCA_CH1WID_OFFSET) &
             BCM2825_I2S_TXCA_CH1WID_MASK;
  if (channel_config & BCM2835_I2S_CHANNEL2)
    txc_a |= (ch_wid << BCM2825_I2S_TXCA_CH2WID_OFFSET) &
             BCM2825_I2S_TXCA_CH2WID_MASK;
  if (channel_config & BCM2835_I2S_CHANNEL1)
    txc_a |= BCM2825_I2S_TXCA_CH1EN_MASK;
  if (channel_config & BCM2835_I2S_CHANNEL2)
    txc_a |= BCM2825_I2S_TXCA_CH2EN_MASK;
  txc_a |= (channel1_pos << BCM2825_I2S_TXCA_CH1POS_OFFSET) &
           BCM2825_I2S_TXCA_CH1POS_MASK;
  txc_a |= (channel2_pos << BCM2825_I2S_TXCA_CH2POS_OFFSET) &
           BCM2825_I2S_TXCA_CH2POS_MASK;
  bcm2835_peri_write(bcm2835_i2s + BCM2835_I2S_TXC_A / 4, txc_a);
  // fprintf(stderr,"TXC_A Info: %p\n", (void*)txc_a);
}

void bcm2835_i2s_config_recieve(uint32_t channel_config, int length,
                                int channel1_pos, int channel2_pos) {
  if (length < 8)
    length = 8;
  /* set RXC_A */
  uint32_t rxc_a = 0x0; // No need to load - we are going to change everything
  uint32_t ch_wid = (length - 8) & 0xf;
  uint32_t ch_wex = ((length - 8) & (0x10)) >> 4;

  if (ch_wex) {
    rxc_a |= BCM2835_I2S_RXCA_CH1WEX_MASK;
    rxc_a |= BCM2835_I2S_RXCA_CH2WEX_MASK;
  }

  if (channel_config & BCM2835_I2S_CHANNEL1)
    rxc_a |= (ch_wid << BCM2825_I2S_RXCA_CH1WID_OFFSET) &
             BCM2825_I2S_RXCA_CH1WID_MASK;
  if (channel_config & BCM2835_I2S_CHANNEL2)
    rxc_a |= (ch_wid << BCM2825_I2S_RXCA_CH2WID_OFFSET) &
             BCM2825_I2S_RXCA_CH2WID_MASK;
  if (channel_config & BCM2835_I2S_CHANNEL1)
    rxc_a |= BCM2825_I2S_RXCA_CH1EN_MASK;
  if (channel_config & BCM2835_I2S_CHANNEL2)
    rxc_a |= BCM2825_I2S_RXCA_CH2EN_MASK;
  rxc_a |= (channel1_pos << BCM2825_I2S_RXCA_CH1POS_OFFSET) &
           BCM2825_I2S_RXCA_CH1POS_MASK;
  rxc_a |= (channel2_pos << BCM2825_I2S_RXCA_CH2POS_OFFSET) &
           BCM2825_I2S_RXCA_CH2POS_MASK;
  bcm2835_peri_write(bcm2835_i2s + BCM2835_I2S_RXC_A / 4, rxc_a);
  // fprintf(stderr,"RXC_A Info: %p\n", (void*)rxc_a);
}

void bcm2835_i2s_en_strobe() // configure I2S clock control
{
  /* set MODE_A */
  uint32_t mode_a = bcm2835_peri_read(bcm2835_i2s + BCM2835_I2S_MODE_A / 4);
  mode_a = mode_a + (0x1 << BCM2835_I2S_MODEA_FLEN_OFFSET);
  bcm2835_peri_write(bcm2835_i2s + BCM2835_I2S_MODE_A / 4, mode_a);
  // fprintf(stderr,"Mode A Info: %p\n", (void*)mode_a);
}

void bcm2835_i2s_dis_strobe() // configure I2S clock control
{
  /* set MODE_A */
  uint32_t mode_a = bcm2835_peri_read(bcm2835_i2s + BCM2835_I2S_MODE_A / 4);
  mode_a = mode_a - (0x1 << BCM2835_I2S_MODEA_FLEN_OFFSET);
  bcm2835_peri_write(bcm2835_i2s + BCM2835_I2S_MODE_A / 4, mode_a);
  // fprintf(stderr,"Mode A Info: %p\n", (void*)mode_a);
}

void bcm2835_i2s_decimation(uint32_t decimation_config) {
  uint32_t mode_a = bcm2835_peri_read(bcm2835_i2s + BCM2835_I2S_MODE_A / 4);
  mode_a &= ~(BCM2835_I2S_MODEA_PDME_MASK |
              BCM2835_I2S_MODEA_PDMN_MASK); // Clear Current Setting
  mode_a |= (decimation_config << 26) &
            (BCM2835_I2S_MODEA_PDME_MASK | BCM2835_I2S_MODEA_PDMN_MASK);
  bcm2835_peri_write(bcm2835_i2s + BCM2835_I2S_MODE_A / 4, mode_a);
  // fprintf(stderr,"Mode A Info: %p\n", (void*)mode_a);
}

void bcm2835_i2s_dma_en() {
  /* set CS_A */
  uint32_t cs_a = bcm2835_peri_read(bcm2835_i2s + BCM2835_I2S_CS_A / 4);
  cs_a |= BCM2835_I2S_CSA_DMAEN_MASK;
  bcm2835_peri_write(bcm2835_i2s + BCM2835_I2S_CS_A / 4, cs_a);
  // fprintf(stderr,"CS_A Info: %p\n", (void*)cs_a);

  /* set DREQ_A */
  uint32_t dreq_a = 0x0; // No need to load - we are going to change everything
  dreq_a |= (0x10 << 24) & BCM2835_I2S_DRQ_TPANIC_MASK;
  dreq_a |= (0x30 << 16) & BCM2835_I2S_DRQ_RPANIC_MASK;
  dreq_a |= (0x30 << 8) & BCM2835_I2S_DRQ_TDREQ_MASK;
  dreq_a |= (0x10 << 0) & BCM2835_I2S_DRQ_RDREQ_MASK;
  bcm2835_peri_write(bcm2835_i2s + BCM2835_I2S_DREQ_A / 4, dreq_a);
  // fprintf(stderr,"DREQ_A Info: %p\n", (void*)dreq_a);
}

void bcm2835_i2s_enable() {
  /* set CS_A */
  uint32_t cs_a = bcm2835_peri_read(bcm2835_i2s + BCM2835_I2S_CS_A / 4);
  cs_a |= BCM2835_I2S_CSA_EN_MASK;
  cs_a &= ~BCM2835_I2S_CSA_RXON_MASK;
  cs_a &= ~BCM2835_I2S_CSA_TXON_MASK;
  bcm2835_peri_write(bcm2835_i2s + BCM2835_I2S_CS_A / 4, cs_a);
  // fprintf(stderr,"CS_A Info: %p\n", (void*)cs_a);
}

void bcm2835_i2s_start_rx() {
  uint32_t cs_a = bcm2835_peri_read(bcm2835_i2s + BCM2835_I2S_CS_A / 4);
  cs_a |= BCM2835_I2S_CSA_RXON_MASK;
  bcm2835_peri_write(bcm2835_i2s + BCM2835_I2S_CS_A / 4, cs_a);
}

void bcm2835_i2s_start_tx() {
  uint32_t cs_a = bcm2835_peri_read(bcm2835_i2s + BCM2835_I2S_CS_A / 4);
  cs_a |= BCM2835_I2S_CSA_TXON_MASK;
  bcm2835_peri_write(bcm2835_i2s + BCM2835_I2S_CS_A / 4, cs_a);
}

void bcm2835_i2s_start() {
  uint32_t cs_a = bcm2835_peri_read(bcm2835_i2s + BCM2835_I2S_CS_A / 4);
  cs_a |= BCM2835_I2S_CSA_TXON_MASK;
  cs_a |= BCM2835_I2S_CSA_RXON_MASK;
  bcm2835_peri_write(bcm2835_i2s + BCM2835_I2S_CS_A / 4, cs_a);
  // fprintf(stderr,"CS_A Info: %p\n", (void*)cs_a);
}

void bcm2835_i2s_stop() {
  uint32_t cs_a = bcm2835_peri_read(bcm2835_i2s + BCM2835_I2S_CS_A / 4);
  cs_a |= BCM2835_I2S_CSA_TXCLR_MASK;
  cs_a |= BCM2835_I2S_CSA_RXCLR_MASK;
  cs_a &= ~BCM2835_I2S_CSA_TXON_MASK;
  cs_a &= ~BCM2835_I2S_CSA_RXON_MASK;
  bcm2835_peri_write(bcm2835_i2s + BCM2835_I2S_CS_A / 4, cs_a);
  // fprintf(stderr,"CS_A Info: %p\n", (void*)cs_a);
}

void bcm2835_i2s_stop_rx() {
  uint32_t cs_a = bcm2835_peri_read(bcm2835_i2s + BCM2835_I2S_CS_A / 4);
  cs_a &= ~BCM2835_I2S_CSA_RXON_MASK;
  bcm2835_peri_write(bcm2835_i2s + BCM2835_I2S_CS_A / 4, cs_a);
}

void bcm2835_i2s_stop_tx() {
  uint32_t cs_a = bcm2835_peri_read(bcm2835_i2s + BCM2835_I2S_CS_A / 4);
  cs_a &= ~BCM2835_I2S_CSA_TXON_MASK;
  bcm2835_peri_write(bcm2835_i2s + BCM2835_I2S_CS_A / 4, cs_a);
}

uint32_t bcm2835_i2s_is_tx_fifo_empty() {
  uint32_t cs_a = bcm2835_peri_read(bcm2835_i2s + BCM2835_I2S_CS_A / 4);
  return !(cs_a & BCM2835_I2S_CSA_TXEMPTY_MASK);
}

void bcm2835_i2s_write_fifo(uint32_t sample) {
  bcm2835_peri_write(bcm2835_i2s + BCM2835_I2S_FIFO_A / 4, sample);
}

void bcm2835_i2s_clear_tx_fifo() // Takes 2 PCM cycles
{
  uint32_t cs_a = bcm2835_peri_read(bcm2835_i2s + BCM2835_I2S_CS_A / 4);
  cs_a |= BCM2835_I2S_CSA_TXCLR_MASK;
  bcm2835_peri_write(bcm2835_i2s + BCM2835_I2S_CS_A / 4, cs_a);
  // fprintf(stderr,"CS_A Info: %p\n", (void*)cs_a);
}

uint32_t bcm2835_i2s_is_rx_fifo_full() {
  uint32_t cs_a = bcm2835_peri_read(bcm2835_i2s + BCM2835_I2S_CS_A / 4);
  return !(cs_a & BCM2835_I2S_CSA_RXFULL_MASK);
}

uint32_t bcm2835_i2s_read_fifo() // Takes 2 PCM cycles
{
  return bcm2835_peri_read(bcm2835_i2s + BCM2835_I2S_FIFO_A / 4);
}

void bcm2835_i2s_clear_rx_fifo() {
  uint32_t cs_a = bcm2835_peri_read(bcm2835_i2s + BCM2835_I2S_CS_A / 4);
  cs_a |= BCM2835_I2S_CSA_RXCLR_MASK;
  bcm2835_peri_write(bcm2835_i2s + BCM2835_I2S_CS_A / 4, cs_a);
  // fprintf(stderr,"CS_A Info: %p\n", (void*)cs_a);
}

void bcm2835_i2s_wait_for_sync() {
  uint32_t cs_a = bcm2835_peri_read(bcm2835_i2s + BCM2835_I2S_CS_A / 4);
  cs_a &= ~BCM2835_I2S_CSA_SYNC_MASK;
  bcm2835_peri_write(bcm2835_i2s + BCM2835_I2S_CS_A / 4, cs_a);
  while (bcm2835_peri_read(bcm2835_i2s + BCM2835_I2S_CS_A / 4) &
         BCM2835_I2S_CSA_SYNC_MASK) {
  }
}

// Helper function to check TX/RX I2S Debug Internals
void bcm2835_i2s_debug() {
  uint32_t cs_a = bcm2835_peri_read(bcm2835_i2s + BCM2835_I2S_CS_A / 4);
  if (cs_a & BCM2835_I2S_CSA_EN_MASK)
    fprintf(stderr, "Enabled \n");
  if (cs_a & BCM2835_I2S_CSA_TXON_MASK)
    fprintf(stderr, "TX ON \n");
  if (cs_a & BCM2835_I2S_CSA_TXEMPTY_MASK)
    fprintf(stderr, "TX is Empty \n");
  if (cs_a & BCM2835_I2S_CSA_TXD_MASK)
    fprintf(stderr, "TX RDY \n");
  if (cs_a & BCM2835_I2S_CSA_TXERR_MASK)
    fprintf(stderr, "TX Error \n");
  if (cs_a & BCM2835_I2S_CSA_RXON_MASK)
    fprintf(stderr, "RX ON \n");
  if (cs_a & BCM2835_I2S_CSA_RXFULL_MASK)
    fprintf(stderr, "RX is Full \n");
  if (cs_a & BCM2835_I2S_CSA_RXD_MASK)
    fprintf(stderr, "RX RDY \n");
  if (cs_a & BCM2835_I2S_CSA_RXERR_MASK)
    fprintf(stderr, "RX Error \n");
}

// Helper function to check all I2S Debug Internals
void bcm2835_i2s_info() {
  bcm2835_i2s_debug();
  uint32_t cs_a = bcm2835_peri_read(bcm2835_i2s + BCM2835_I2S_CS_A / 4);
  if (cs_a & BCM2835_I2S_CSA_DMAEN_MASK)
    fprintf(stderr, "DMA Enabled \n");
  if (cs_a & BCM2835_I2S_CSA_RXSEX_MASK)
    fprintf(stderr, "RX Sign Extended \n");
  if (cs_a & BCM2835_I2S_CSA_RXR_MASK)
    fprintf(stderr, "RX Needs Read / RXTHR\n");
  if (cs_a & BCM2835_I2S_CSA_TXW_MASK)
    fprintf(stderr, "TX Needs Write / TXTHR \n");
  if (cs_a & BCM2835_I2S_CSA_RXSYNC_MASK)
    fprintf(stderr, "RX FIFO is in Sync\n");
  if (cs_a & BCM2835_I2S_CSA_TXSYNC_MASK)
    fprintf(stderr, "TX FIFO is in Sync\n");
  fprintf(stderr, "PCM Status Info: %p\n", (void *)cs_a);

  uint32_t mode_a = bcm2835_peri_read(bcm2835_i2s + BCM2835_I2S_MODE_A / 4);
  if (mode_a & BCM2835_I2S_MODEA_CLK_DIS)
    fprintf(stderr, "Clock Disabled \n");
  if (mode_a & BCM2835_I2S_MODEA_PDMN_MASK)
    fprintf(stderr, "Increased Decimation \n");
  if (mode_a & BCM2835_I2S_MODEA_PDME_MASK)
    fprintf(stderr, "Input Decimation En \n");
  if (mode_a & BCM2835_I2S_MODEA_FRXP_MASK)
    fprintf(stderr, "RX 2x16 bit packed mode \n");
  if (mode_a & BCM2835_I2S_MODEA_FTXP_MASK)
    fprintf(stderr, "TX 2x16 bit packed mode \n");
  if (mode_a & BCM2835_I2S_MODEA_CLKM_MASK)
    fprintf(stderr, "PCM CLK is Slave \n");
  if (mode_a & BCM2835_I2S_MODEA_CLKI_MASK)
    fprintf(stderr, "PCM CLK is Inverted \n");
  if (mode_a & BCM2835_I2S_MODEA_FSM_MASK)
    fprintf(stderr, "PCM FRAME is Slave \n");
  if (mode_a & BCM2835_I2S_MODEA_FSI_MASK)
    fprintf(stderr, "PCM FRAME is Inverted \n");
  fprintf(stderr, "FLEN is %x \n", (mode_a & BCM2835_I2S_MODEA_FLEN_MASK) >>
                                       BCM2835_I2S_MODEA_FLEN_OFFSET);
  fprintf(stderr, "FSLEN is %x \n", (mode_a & BCM2835_I2S_MODEA_FSLEN_MASK) >>
                                        BCM2835_I2S_MODEA_FSLEN_OFFSET);
  fprintf(stderr, "Mode A Info: %p\n", (void *)mode_a);
}
