//
// C lib Proceedures for servicing i2s/spi/dma/pwm/gpclk on RPI3 over LAN
// Using bcm2835 C lib
//
// 2019 Lieuwe B. Leene
//

#ifndef __BCM2835_I2S_H__
#define __BCM2835_I2S_H__

#include <bcm2835.h>
#include <inttypes.h>
#include <stdio.h>
#include <sys/mman.h>

#define BCM2835_I2S_BASE 0x203000 // 0x7E203000
#define BCM2835_PWM_OSC_FRQ_MHZ 19

/* I2S Pin Definitions */
#define i2s_bclk RPI_V2_GPIO_P1_12 // ALT0
#define i2s_fclk RPI_V2_GPIO_P1_35 // ALT0
#define i2s_dout RPI_V2_GPIO_P1_40 // ALT0
#define i2s_din RPI_V2_GPIO_P1_38  // ALT0

/* I2S Register Off-Sets */
#define BCM2835_I2S_CS_A 0x0
#define BCM2835_I2S_FIFO_A 0x4
#define BCM2835_I2S_MODE_A 0x8
#define BCM2835_I2S_RXC_A 0xc
#define BCM2835_I2S_TXC_A 0x10
#define BCM2835_I2S_DREQ_A 0x14
#define BCM2835_I2S_INTEN_A 0x18
#define BCM2835_I2S_INTSTC_A 0x1c
#define BCM2835_I2S_GRAY 0x20

/* I2S_CS_A Contents */
#define BCM2835_I2S_CSA_STBY_MASK (0x1 << 25)
#define BCM2835_I2S_CSA_SYNC_MASK (0x1 << 24)
#define BCM2835_I2S_CSA_RXSEX_MASK (0x1 << 23)
#define BCM2835_I2S_CSA_RXFULL_MASK (0x1 << 22)
#define BCM2835_I2S_CSA_TXEMPTY_MASK (0x1 << 21)
#define BCM2835_I2S_CSA_RXD_MASK (0x1 << 20)
#define BCM2835_I2S_CSA_TXD_MASK (0x1 << 19)
#define BCM2835_I2S_CSA_RXR_MASK (0x1 << 18)
#define BCM2835_I2S_CSA_TXW_MASK (0x1 << 17)
#define BCM2835_I2S_CSA_RXERR_MASK (0x1 << 16)
#define BCM2835_I2S_CSA_TXERR_MASK (0x1 << 15)
#define BCM2835_I2S_CSA_RXSYNC_MASK (0x1 << 14)
#define BCM2835_I2S_CSA_TXSYNC_MASK (0x1 << 13)
#define BCM2835_I2S_CSA_DMAEN_MASK (0x1 << 9)
#define BCM2835_I2S_CSA_RXTHR_MASK (0x3 << 7)
#define BCM2835_I2S_CSA_TXTHR_MASK (0x3 << 5)
#define BCM2835_I2S_CSA_RXCLR_MASK (0x1 << 4)
#define BCM2835_I2S_CSA_TXCLR_MASK (0x1 << 3)
#define BCM2835_I2S_CSA_TXON_MASK (0x1 << 2)
#define BCM2835_I2S_CSA_RXON_MASK (0x1 << 1)
#define BCM2835_I2S_CSA_EN_MASK (0x1 << 0)

/* I2S_MODE_A Contents */
#define BCM2835_I2S_MODEA_CLK_DIS (0x1 << 28)
#define BCM2835_I2S_MODEA_PDMN_MASK (0x1 << 27)
#define BCM2835_I2S_MODEA_PDME_MASK (0x1 << 26)
#define BCM2835_I2S_MODEA_FRXP_MASK (0x1 << 25)
#define BCM2835_I2S_MODEA_FTXP_MASK (0x1 << 24)
#define BCM2835_I2S_MODEA_CLKM_MASK (0x1 << 23)
#define BCM2835_I2S_MODEA_CLKI_MASK (0x1 << 22)
#define BCM2835_I2S_MODEA_CLK_OFFSET 22
#define BCM2835_I2S_MODEA_FSM_MASK (0x1 << 21)
#define BCM2835_I2S_MODEA_FSI_MASK (0x1 << 20)
#define BCM2835_I2S_MODEA_FS_OFFSET 20
#define BCM2835_I2S_MODEA_FLEN_OFFSET 10
#define BCM2835_I2S_MODEA_FLEN_MASK 0xFFC00
#define BCM2835_I2S_MODEA_FSLEN_OFFSET 0x0
#define BCM2835_I2S_MODEA_FSLEN_MASK (0x3FF << 0)

/* I2S_TXC_A Contents */
#define BCM2835_I2S_RXCA_CH1WEX_MASK (0x1 << 31)
#define BCM2825_I2S_RXCA_CH1EN_MASK (0x1 << 30)
#define BCM2825_I2S_RXCA_CH1POS_OFFSET 20
#define BCM2825_I2S_RXCA_CH1POS_MASK (0x3FF << 20)
#define BCM2825_I2S_RXCA_CH1WID_OFFSET 16
#define BCM2825_I2S_RXCA_CH1WID_MASK (0xF << 16)
#define BCM2835_I2S_RXCA_CH2WEX_MASK (0x1 << 15)
#define BCM2825_I2S_RXCA_CH2EN_MASK (0x1 << 14)
#define BCM2825_I2S_RXCA_CH2POS_OFFSET 4
#define BCM2825_I2S_RXCA_CH2POS_MASK (0x3FF << 4)
#define BCM2825_I2S_RXCA_CH2WID_OFFSET 0
#define BCM2825_I2S_RXCA_CH2WID_MASK (0xF << 0)

/* I2S_TXC_A Contents */
#define BCM2835_I2S_TXCA_CH1WEX_MASK (0x1 << 31)
#define BCM2825_I2S_TXCA_CH1EN_MASK (0x1 << 30)
#define BCM2825_I2S_TXCA_CH1POS_OFFSET 20
#define BCM2825_I2S_TXCA_CH1POS_MASK (0x3FF << 20)
#define BCM2825_I2S_TXCA_CH1WID_OFFSET 16
#define BCM2825_I2S_TXCA_CH1WID_MASK (0xF << 16)
#define BCM2835_I2S_TXCA_CH2WEX_MASK (0x1 << 15)
#define BCM2825_I2S_TXCA_CH2EN_MASK (0x1 << 14)
#define BCM2825_I2S_TXCA_CH2POS_OFFSET 4
#define BCM2825_I2S_TXCA_CH2POS_MASK (0x3FF << 4)
#define BCM2825_I2S_TXCA_CH2WID_OFFSET 0
#define BCM2825_I2S_TXCA_CH2WID_MASK (0xF << 0)

/* I2S_DREQ_A Contents */
#define BCM2835_I2S_DRQ_TPANIC_MASK (0x7F << 24) //// Write Panic Threshold
#define BCM2835_I2S_DRQ_RPANIC_MASK (0x7F << 16) //// Read Panic Threshold
#define BCM2835_I2S_DRQ_TDREQ_MASK (0x7F << 8)   //// Write Request Threshold
#define BCM2835_I2S_DRQ_RDREQ_MASK (0x7F)        //// Read Request Threshold

/* elinux.org/BCM2835_datasheet_errata */
/*
* REF1
*   BCM2835 ARM Peripherals 6 Feb 2012 Broadcom Europe
*   BCM2835-ARM-Peripherals.pdf
*
* REF2
*   BCM2835_Audio_PWM_Clocks_errata_Geert_Van_Loo.doc which is images
*   captured from http://www.scribd.com/doc/127599939/BCM2835-Audio-clocks
*
* REF3
*  http://raspberrypi.stackexchange.com/questions/1153/what-are-the-different-
*      clock-sources-for-the-general-purpose-clocks
*
*  which reports the following:
*   0     0 Hz     Ground
*   1     19.2 MHz oscillator
*   2     0 Hz     testdebug0
*   3     0 Hz     testdebug1
*   4     0 Hz     PLLA
*   5     1000 MHz PLLC (changes with overclock settings)
*   6     500 MHz  PLLD
*   7     216 MHz  HDMI auxiliary
*   8-15  0 Hz     Ground
*
*  The REF1 table 6-34 doesn't report the clock frequencies
*
* REF4
*  i2s_test4_test_vector_vy.yy.xls computes values for test vectors
*  see docs dir for latest version
*/

typedef enum {
  BCM2835_I2S_INV_EXTERNAL_CLK = 0x3, /*!< PCM/I2S Uses External !CLK 0b11  */
  BCM2835_I2S_EXTERNAL_CLK = 0x2,     /*!< PCM/I2S Uses External CLK  0b10  */
  BCM2835_I2S_INV_INTERNAL_CLK = 0x1, /*!< PCM/I2S Uses Internal !CLK 0b01  */
  BCM2835_I2S_INTERNAL_CLK = 0x0,     /*!< PCM/I2S Uses Internal CLK  0b00  */
} bcm2835I2SClockConfig;

typedef enum {
  BCM2835_I2S_INV_EXTERNAL_FS =
      0x3,                       /*!< PCM/I2S Uses External !FRAME_SYNC 0b11  */
  BCM2835_I2S_EXTERNAL_FS = 0x2, /*!< PCM/I2S Uses External FRAME_SYNC  0b10  */
  BCM2835_I2S_INV_INTERNAL_FS =
      0x1,                       /*!< PCM/I2S Uses Internal !FRAME_SYNC 0b01  */
  BCM2835_I2S_INTERNAL_FS = 0x0, /*!< PCM/I2S Uses Internal FRAME_SYNC  0b00  */
} bcm2835I2SFrameConfig;

typedef enum {
  BCM2835_I2S_ALL_CHANNELS = 0x3, /*!< PCM/I2S Uses Both ChHannels   0b11  */
  BCM2835_I2S_CHANNEL2 = 0x2,     /*!< PCM/I2S Uses Channel 2        0b10  */
  BCM2835_I2S_CHANNEL1 = 0x1,     /*!< PCM/I2S Uses Channel 1        0b01  */
  BCM2835_I2S_NO_CHANNELS = 0x0,  /*!< PCM/I2S Uses Neither Channel  0b00  */
} bcm2835I2SChannelConfig;

typedef enum {
  BCM2835_I2S_DECIMATE_32 = 0x3, /*!< Enable Decimation 16x 0b11  */
  BCM2835_I2S_DECIMATE_16 = 0x1, /*!< Enable Decimation 16x 0b01  */
  BCM2835_I2S_DECIMATE_0 = 0x0,  /*!< Disable Decimation 0b00  */
} bcm2835I2SDecimation;

#ifdef __cplusplus
extern "C" {
#endif

int bcm2835_i2s_init();
void bcm2835_i2s_enable_output();
void bcm2835_i2s_disable_output();
void bcm2835_i2s_8bslave_setup();
void bcm2835_i2s_8bpwm_setup();
void bcm2835_i2s_pwm_setup(uint32_t frame_length);
void bcm2835_i2s_clock_config(uint32_t clock_config);
void bcm2835_i2s_frame_config(uint32_t frame_config, int flen, int fslen);
void bcm2835_i2s_config_transmit(uint32_t channel_config, int length,
                                 int channel1_pos, int channel2_pos);
void bcm2835_i2s_config_recieve(uint32_t channel_config, int length,
                                int channel1_pos, int channel2_pos);
void bcm2835_i2s_decimation(uint32_t decimation_config);
void bcm2835_i2s_dma_en();
void bcm2835_i2s_enable();
void bcm2835_i2s_start();
void bcm2835_i2s_start_tx();
void bcm2835_i2s_start_rx();
void bcm2835_i2s_stop();
void bcm2835_i2s_stop_tx();
void bcm2835_i2s_stop_rx();

void bcm2835_i2s_en_strobe();
void bcm2835_i2s_dis_strobe();
void bcm2835_i2s_write_fifo(uint32_t sample);
uint32_t bcm2835_i2s_read_fifo();

uint32_t bcm2835_i2s_is_tx_fifo_empty();
void bcm2835_i2s_clear_tx_fifo();
uint32_t bcm2835_i2s_is_rx_fifo_full();
void bcm2835_i2s_clear_rx_fifo();

void bcm2835_i2s_wait_for_sync();
void bcm2835_i2s_debug();
void bcm2835_i2s_info();

#ifdef __cplusplus
}
#endif

#endif /* BCM2835_I2S_H */
