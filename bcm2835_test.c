//
// C lib Proceedures for servicing i2s/spi/dma/pwm/gpclk on RPI3 over LAN
// Using bcm2835 C lib
//
// 2019 Lieuwe B. Leene
//

#include "bcm2835_test.h"

int bcm2835_test_VYGR2_init(void) {
  if (!bcm2835_init())
    return 0;
  if (!bcm2835_i2s_init())
    return 0;
  if (!bcm2835_dma_init())
    return 0;

  //Setup I2S PIN configuration
  bcm2835_i2s_enable_output();
  bcm2835_i2s_8bslave_setup();

  //Setup BB-SPI PIN configuration
  bcm2835_gpio_fsel(spi_ce0, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(spi_mosi, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(spi_miso, BCM2835_GPIO_FSEL_INPT);

  //Setup Enable CLK PIN
  bcm2835_gpio_fsel(i2s_en, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_clr(i2s_en);
  bcm2835_gpio_fsel(vygr_ck, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_clr(vygr_ck);

  //Setup Enable PWM HIGH Pass
  bcm2835_gpio_fsel(vygr_dt, BCM2835_GPIO_FSEL_ALT0);

  //Setup Enable HIGH Pass
  bcm2835_pwm_set_clock(BCM2835_PWM_CLOCK_DIVIDER_2);
  bcm2835_pwm_set_range(0, 2);
  bcm2835_pwm_set_data(0, 1);
  bcm2835_pwm_set_mode(0, 1, 0);
  bcm2835_pwm_set_clock(BCM2835_PWM_CLOCK_DIVIDER_2); // 4
  bcm2835_pwm_set_range(1, 16);
  bcm2835_pwm_set_data(1, 0);
  bcm2835_pwm_set_mode(1, 1, 0);

  return 1;
}

int bcm2835_test_AKIRA_init(void) {
  if (!bcm2835_init())
    return 0;
  if (!bcm2835_i2s_init())
    return 0;
  if (!bcm2835_dma_init())
    return 0;

  //Setup I2S PIN configuration
  bcm2835_i2s_enable_output();
  bcm2835_i2s_8bslave_setup();

  //Setup BB-SPI PIN configuration
  bcm2835_gpio_set_pud(spi_clk, BCM2835_GPIO_PUD_OFF);
  bcm2835_gpio_fsel(spi_clk, BCM2835_GPIO_FSEL_INPT);
  bcm2835_gpio_fsel(spi_ce0, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_set(spi_ce0);
  bcm2835_gpio_fsel(spi_mosi, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(spi_miso, BCM2835_GPIO_FSEL_INPT);

  //Setup Enable CLK PIN
  bcm2835_gpio_fsel(i2s_en, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_clr(i2s_en);
  bcm2835_gpio_fsel(vygr_ck, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_clr(vygr_ck);
  bcm2835_gpio_fsel(pwm_clk1, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_clr(pwm_clk1);

  //Setup CLK Source
  bcm2835_pwm_set_clock(BCM2835_PWM_CLOCK_DIVIDER_8); // 1/2=4.25Mhz
  bcm2835_pwm_set_range(0, 2);
  bcm2835_pwm_set_data(0, 1);
  bcm2835_pwm_set_mode(0, 1, 0);
  bcm2835_pwm_set_range(1, 2);
  bcm2835_pwm_set_data(1, 1);
  bcm2835_pwm_set_mode(1, 1, 0);

  return 1;
}

void bcm2835_test_start_clk(char* test_config){  // need to check for illegal clock state
  if(test_config[0] == '0'){
    bcm2835_gpio_fsel(pwm_clk0, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set(i2s_en);
  }
  else{
    bcm2835_gpio_clr(i2s_en);
    bcm2835_gpio_fsel(pwm_clk0, BCM2835_GPIO_FSEL_ALT0);
  }
}

void bcm2835_test_stop_clk(char* test_config){
  bcm2835_gpio_fsel(pwm_clk0, BCM2835_GPIO_FSEL_INPT);
  bcm2835_gpio_clr(i2s_en);
}

void bcm2835_test_VYGR2_end(void) { // make sure dma stopped
  bcm2835_gpio_clr(i2s_en);
  bcm2835_close();
}

void bcm2835_test_wait_cycle(uint8_t pin) // Cycle Pin untill next falling edge
{
  while(!bcm2835_gpio_lev(pin)){}
  while(bcm2835_gpio_lev(pin)){}
}

void bcm2835_test_toggle(uint8_t pin) // Cycle Pin untill next falling edge
{
  bcm2835_gpio_clr(pin);
  bcm2835_delayMicroseconds(1);
  bcm2835_gpio_set(pin);
  bcm2835_delayMicroseconds(1);
  bcm2835_gpio_clr(pin);
}


void bcm2835_test_configure_akira(char* test_config, syth_cnfg_t *configuration){ // SPI CLK ALWAYS RDY
  int i;
  if(test_config[0] == '1'){ // Master Mode
    bcm2835_gpio_clr(i2s_en);
    bcm2835_i2s_disable_output();
    bcm2835_pwm_set_mode(0, 1, 1);
    bcm2835_gpio_fsel(pwm_clk0, BCM2835_GPIO_FSEL_ALT0);
    // bcm2835_i2s_8bpwm_setup(); // go ahead and start i2s
  }
  else {
    bcm2835_i2s_enable_output();
    bcm2835_gpio_fsel(pwm_clk0, BCM2835_GPIO_FSEL_INPT);
    bcm2835_pwm_set_mode(0, 1, 0);
    bcm2835_i2s_8bslave_setup();
  }

  bcm2835_test_toggle(vygr_ck);
  // Proceed to load configuration data
  for(i=0;i<32;i++){
    if(test_config[31-i] == '1') bcm2835_gpio_set(spi_mosi);
    else bcm2835_gpio_clr(spi_mosi);
    bcm2835_test_toggle(vygr_ck);
  }
  bcm2835_delayMicroseconds(1);
  bcm2835_gpio_clr(spi_ce0);
  bcm2835_gpio_clr(spi_ce0);
  bcm2835_gpio_clr(spi_ce0);
  bcm2835_gpio_set(spi_ce0);
  bcm2835_delayMicroseconds(1);
  bcm2835_test_toggle(vygr_ck);
  bcm2835_delayMicroseconds(5);
  if(test_config[14] == '1'){ // Generate PPulse
    bcm2835_gpio_set(pwm_clk1);
    bcm2835_delayMicroseconds(1);
    bcm2835_gpio_clr(pwm_clk1);
  }
}

void *bcm2835_test_sdmdma_4ch_proc(void *args) {
  int storage_index = 0;
  syth_cnfg_t *configuration = args;
  dma_mem_alloc_t dma_alloc_obj;
  SDM3_4ch_object_t modulator;

  //for(int i=0;i<4;i++) fprintf(stdout,"FRQ:%d AMP:%d\n",configuration->frequency[i],(configuration->amplitude[i] << (8*sizeof(int)-13)));

  if (!bcm2835_test_init_cache(&dma_alloc_obj)) {
    fprintf(stderr, "Allocation Failed... \n");
    return NULL;
  }

  bcm2835_test_setup_cblocks(&dma_alloc_obj, true);
  generate_sdm3_4ch_data(configuration, &(dma_alloc_obj.data_src[0]), DATA_SEGMENTS*DMA_PAGE_SIZE, &modulator);
  bcm2835_test_start_i2sdma(&dma_alloc_obj);

  while (configuration->status_code != SYNTHCORE_STOP){ // Do we need to stop?
    while (bcm2835_dma_get_cblk(channel_src) != (dma_alloc_obj.cblk_src + DATA_SEGMENTS/2 * 8)->next)  // X/2 + 1
      usleep(DMA_PAGE_SIZE / 8); // Wait untill we are halfway
    generate_sdm3_4ch_data(configuration, &(dma_alloc_obj.data_src[0]), DATA_SEGMENTS*DMA_PAGE_SIZE/2, &modulator);
    if(configuration->recording_index > storage_index) {
      bcm2835_test_output_file(&dma_alloc_obj, 0, DATA_SEGMENTS/2);
      storage_index++;
    }
    while (bcm2835_dma_get_cblk(channel_src) != (dma_alloc_obj.cblk_src)->next)   // 0/2 + 1
      usleep(DMA_PAGE_SIZE / 8); // Wait untill we are back at the beginning
    generate_sdm3_4ch_data(configuration, &(dma_alloc_obj.data_src[DATA_SEGMENTS*DMA_PAGE_SIZE/2]), DATA_SEGMENTS*DMA_PAGE_SIZE/2, &modulator);
    if(configuration->recording_index > storage_index) {
      bcm2835_test_output_file(&dma_alloc_obj, DATA_SEGMENTS/2, DATA_SEGMENTS/2);
      storage_index++;
    }
  }

  bcm2835_test_end_transfer(&dma_alloc_obj);
  bcm2835_test_stop_dma(&dma_alloc_obj);
  return NULL;
}

void *bcm2835_test_sdmdma_1ch_proc(void *args) {
  int storage_index = 0;
  syth_cnfg_t *configuration = args;
  dma_mem_alloc_t dma_alloc_obj;
  SDM3_1ch_object_t modulator;

  //for(int i=0;i<4;i++) fprintf(stdout,"FRQ:%d AMP:%d\n",configuration->frequency[i],(configuration->amplitude[i] << (8*sizeof(int)-13)));

  if (!bcm2835_test_init_cache(&dma_alloc_obj)) {
    fprintf(stderr, "Allocation Failed... \n");
    return NULL;
  }

  bcm2835_test_setup_cblocks(&dma_alloc_obj, true);
  generate_sdm3_1ch_data(configuration, &(dma_alloc_obj.data_src[0]), DATA_SEGMENTS*DMA_PAGE_SIZE, &modulator);
  bcm2835_test_start_i2sdma(&dma_alloc_obj);

  while (configuration->status_code != SYNTHCORE_STOP){ // Do we need to stop?
    while (bcm2835_dma_get_cblk(channel_src) != (dma_alloc_obj.cblk_src + DATA_SEGMENTS/2 * 8)->next)  // X/2 + 1
      usleep(DMA_PAGE_SIZE / 8); // Wait untill we are halfway
    generate_sdm3_1ch_data(configuration, &(dma_alloc_obj.data_src[0]), DATA_SEGMENTS*DMA_PAGE_SIZE/2, &modulator);
    if(configuration->recording_index > storage_index) {
      bcm2835_test_output_file(&dma_alloc_obj, 0, DATA_SEGMENTS/2);
      storage_index++;
    }
    while (bcm2835_dma_get_cblk(channel_src) != (dma_alloc_obj.cblk_src)->next)   // 0/2 + 1
      usleep(DMA_PAGE_SIZE / 8); // Wait untill we are back at the beginning
    generate_sdm3_1ch_data(configuration, &(dma_alloc_obj.data_src[DATA_SEGMENTS*DMA_PAGE_SIZE/2]), DATA_SEGMENTS*DMA_PAGE_SIZE/2, &modulator);
    if(configuration->recording_index > storage_index) {
      bcm2835_test_output_file(&dma_alloc_obj, DATA_SEGMENTS/2, DATA_SEGMENTS/2);
      storage_index++;
    }
  }

  bcm2835_test_end_transfer(&dma_alloc_obj);
  bcm2835_test_stop_dma(&dma_alloc_obj);
  return NULL;
}

void bcm2835_test_print_config(char *test_config) {
  int j;
  uint32_t config_word = 0;
  for (j = 0; j < 32; j++) {
    if (test_config[j] == '1')
      config_word |= 0x1 << j;
  }

  for (j = 0; j < 32; j++) {
    if (config_word & (0x1 << j))
      printf("1");
    else
      printf("0");
  }
  printf("\n");
}

int bcm2835_test_init_cache(dma_mem_alloc_t *dma_alloc_obj) {
  // initialize data structres
  dma_alloc_obj->data_memblock_src =
      UncachedMemBlock_alloc(DATA_SEGMENTS * DMA_PAGE_SIZE * sizeof(uint32_t));
  dma_alloc_obj->data_src = (uint32_t *)dma_alloc_obj->data_memblock_src.mem;
  if (!dma_alloc_obj->data_src)
    return 0;
  dma_alloc_obj->data_memblock_dst =
      UncachedMemBlock_alloc(DATA_SEGMENTS * DMA_PAGE_SIZE * sizeof(uint32_t));
  dma_alloc_obj->data_dst = (uint32_t *)dma_alloc_obj->data_memblock_dst.mem;
  if (!dma_alloc_obj->data_dst)
    return 0;

  // initialize configuration structres
  dma_alloc_obj->cblk_memblock_src =
      UncachedMemBlock_alloc(DATA_SEGMENTS * sizeof(struct dma_cb));
  dma_alloc_obj->cblk_src =
      (struct dma_cb *)dma_alloc_obj->cblk_memblock_src.mem;
  if (!dma_alloc_obj->cblk_src)
    return 0;
  dma_alloc_obj->cblk_memblock_dst =
      UncachedMemBlock_alloc(DATA_SEGMENTS * sizeof(struct dma_cb));
  dma_alloc_obj->cblk_dst =
      (struct dma_cb *)dma_alloc_obj->cblk_memblock_dst.mem;
  if (!dma_alloc_obj->cblk_dst)
    return 0;

  return 1; // Setup is successful
}

// Configure Load configuration with circular CBLOCKs
void bcm2835_test_setup_cblocks(dma_mem_alloc_t *dma_alloc_obj,
                                bool circular_transfer) {
  int i;
  for (i = 0; i < DATA_SEGMENTS; i++) {
    bcm2835_dma_i2s_setup((dma_alloc_obj->cblk_src + i * 8),
                          (dma_alloc_obj->cblk_dst + i * 8),
                          (uint32_t)DMA_PAGE_SIZE * 4);
    (dma_alloc_obj->cblk_src + i * 8)->src = UncachedMemBlock_to_physical(
        &dma_alloc_obj->data_memblock_src,
        &dma_alloc_obj->data_src[i * DMA_PAGE_SIZE]);
    (dma_alloc_obj->cblk_dst + i * 8)->dst = UncachedMemBlock_to_physical(
        &dma_alloc_obj->data_memblock_dst,
        &dma_alloc_obj->data_dst[i * DMA_PAGE_SIZE]);
    (dma_alloc_obj->cblk_src + i * 8)->next = UncachedMemBlock_to_physical(
        &dma_alloc_obj->cblk_memblock_src,
        (dma_alloc_obj->cblk_src) + ((i + 1) % DATA_SEGMENTS) * 8);
    (dma_alloc_obj->cblk_dst + i * 8)->next = UncachedMemBlock_to_physical(
        &dma_alloc_obj->cblk_memblock_dst,
        (dma_alloc_obj->cblk_dst) + ((i + 1) % DATA_SEGMENTS) * 8);
  }
  if (circular_transfer == false) {
    (dma_alloc_obj->cblk_src + DATA_SEGMENTS * 8 - 8)->next = 0;
    (dma_alloc_obj->cblk_dst + DATA_SEGMENTS * 8 - 8)->next = 0;
  }
}

void bcm2835_test_end_transfer(
    dma_mem_alloc_t *dma_alloc_obj) { // Assume transfer is circular
  if (DATA_SEGMENTS > 1 &&
      bcm2835_dma_get_cblk(channel_dst) !=
          0) { // Wait untill we are operating on the first CBLK
    while (bcm2835_dma_get_cblk(channel_dst) != dma_alloc_obj->cblk_dst->next) {
      usleep(DMA_PAGE_SIZE / 8);
    }
    (dma_alloc_obj->cblk_src + DATA_SEGMENTS * 8 - 8)->next = 0;
    (dma_alloc_obj->cblk_dst + DATA_SEGMENTS * 8 - 8)->next = 0;
  }

  else { // Else wait untill more than half the transfer is qued
    while (bcm2835_dma_get_length(channel_dst) <= DMA_PAGE_SIZE / 2) {
      usleep(DMA_PAGE_SIZE / 8);
    }
    (dma_alloc_obj->cblk_src + DATA_SEGMENTS * 8 - 8)->next = 0;
    (dma_alloc_obj->cblk_dst + DATA_SEGMENTS * 8 - 8)->next = 0;
  }

  // wait for completion
  while (bcm2835_dma_get_cblk(channel_dst) != 0) {
    usleep(DMA_PAGE_SIZE / 8);
  }
}

// Start DMA-I2S-DMA Data Pipeline
void bcm2835_test_start_i2sdma(dma_mem_alloc_t *dma_alloc_obj) {
  bcm2835_i2s_clear_tx_fifo();
  bcm2835_i2s_clear_rx_fifo();
  bcm2835_i2s_enable();

  // Use the first control block for DMA control
  bcm2835_dual_dma_start(
      channel_src,
      UncachedMemBlock_to_physical(&dma_alloc_obj->cblk_memblock_src,
                                   dma_alloc_obj->cblk_src),
      channel_dst,
      UncachedMemBlock_to_physical(&dma_alloc_obj->cblk_memblock_dst,
                                   dma_alloc_obj->cblk_dst));

  bcm2835_i2s_start();
}

void bcm2835_test_stop_dma(dma_mem_alloc_t *dma_alloc_obj) {
  // Stop I2S Engine
  bcm2835_i2s_stop();

  // Clean Up DMA Allocation
  bcm2835_dual_dma_end(channel_src, channel_dst);

  bcm2835_test_sdm2csv(dma_alloc_obj);

  if (dma_alloc_obj->data_src)
    UncachedMemBlock_free(&dma_alloc_obj->data_memblock_src);
  if (dma_alloc_obj->data_dst)
    UncachedMemBlock_free(&dma_alloc_obj->data_memblock_dst);
  if (dma_alloc_obj->cblk_src)
    UncachedMemBlock_free(&dma_alloc_obj->cblk_memblock_src);
  if (dma_alloc_obj->cblk_dst)
    UncachedMemBlock_free(&dma_alloc_obj->cblk_memblock_dst);
}

void bcm2835_test_end_full(dma_mem_alloc_t *dma_alloc_obj) {
  // Stop I2S Engine
  bcm2835_i2s_stop();
  bcm2835_gpio_clr(i2s_en);

  // Clean Up DMA Allocation
  bcm2835_dual_dma_end(channel_src, channel_dst);

  if (dma_alloc_obj->data_src)
    UncachedMemBlock_free(&dma_alloc_obj->data_memblock_src);
  if (dma_alloc_obj->data_dst)
    UncachedMemBlock_free(&dma_alloc_obj->data_memblock_dst);
  if (dma_alloc_obj->cblk_src)
    UncachedMemBlock_free(&dma_alloc_obj->cblk_memblock_src);
  if (dma_alloc_obj->cblk_dst)
    UncachedMemBlock_free(&dma_alloc_obj->cblk_memblock_dst);

  // Configure I2S outputs
  bcm2835_i2s_disable_output();

  // Disable PWM Clocks
  bcm2835_gpio_fsel(pwm_clk0, BCM2835_GPIO_FSEL_INPT);
  bcm2835_gpio_fsel(pwm_clk1, BCM2835_GPIO_FSEL_INPT);

  bcm2835_gpio_fsel(gp_clk0, BCM2835_GPIO_FSEL_INPT);
  bcm2835_gpio_fsel(gp_clk1, BCM2835_GPIO_FSEL_INPT);
  bcm2835_gpio_fsel(gp_clk2, BCM2835_GPIO_FSEL_INPT);
}

void bcm2835_test_init_full(void) {
  // Configure ENABLE output
  bcm2835_gpio_set_pud(i2s_en, BCM2835_GPIO_PUD_OFF);
  bcm2835_gpio_fsel(i2s_en, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_set(i2s_en);

  // Configure I2S outputs
  bcm2835_i2s_enable_output();

  // Configure PWM outputs
  bcm2835_gpio_fsel(pwm_clk0, BCM2835_GPIO_FSEL_ALT0);
  bcm2835_gpio_fsel(pwm_clk1, BCM2835_GPIO_FSEL_ALT0);

  // Configure CLK sources
  bcm2835_gpio_fsel(gp_clk0, BCM2835_GPIO_FSEL_ALT0);
  bcm2835_gpio_fsel(gp_clk1, BCM2835_GPIO_FSEL_ALT0);
  bcm2835_gpio_fsel(gp_clk2, BCM2835_GPIO_FSEL_ALT0);

  // Configure PWM Settings
  bcm2835_pwm_set_clock(BCM2835_PWM_CLOCK_DIVIDER_16);
  bcm2835_pwm_set_range(0, 2);
  bcm2835_pwm_set_data(0, 1);
  bcm2835_pwm_set_mode(0, 1, 1); // USE MARK SPACE MODE TO DECREASE TRANSISIONS
  bcm2835_pwm_set_range(1, 2);
  bcm2835_pwm_set_data(1, 1);
  bcm2835_pwm_set_mode(1, 1, 1); // USE MARK SPACE MODE TO DECREASE TRANSISIONS

  // Configure I2S/PCM Settings
  bcm2835_i2s_8bslave_setup();
}

void bcm2835_test_generate_segment(dma_mem_alloc_t *dma_alloc_obj,
                                   int segment) {
  // We are going to fill one segment
  int i;
  for (i = 0; i < DMA_PAGE_SIZE; i++) {
    dma_alloc_obj->data_src[i + (segment * DMA_PAGE_SIZE)] =
        DMA_PAGE_SIZE * segment + i;
  }
}

void bcm2835_test_print_segment(dma_mem_alloc_t *dma_alloc_obj, int segment) {
  // We are going to print one segment
  int i;
  for (i = 0; i < DMA_PAGE_SIZE; i++) {
    fprintf(stdout, "SRC: %d DST: %d \n",
            dma_alloc_obj->data_src[i + (segment * DMA_PAGE_SIZE)],
            dma_alloc_obj->data_dst[i + (segment * DMA_PAGE_SIZE)]);
  }
}

int bcm2835_test_output_file(dma_mem_alloc_t *dma_alloc_obj, int segment_offset, int segment_number) {
  FILE *fp;
  fp = fopen("dataout.bin", "a");
  if(fp > 0 && DATA_SEGMENTS >= (segment_offset+segment_number) ) {
    fwrite(&dma_alloc_obj->data_dst[segment_offset*DMA_PAGE_SIZE], sizeof(uint32_t), segment_number*DMA_PAGE_SIZE, fp);
  }
  else return 0;
  fclose(fp);
  return 1;
}

int bcm2835_test_dump_csv(dma_mem_alloc_t *dma_alloc_obj) {
  // We are going to print src + dst data to file
  FILE *fp;
  fp = fopen("file_src.csv", "w");
  if (fp <= 0)
    return 0;
  fwrite(dma_alloc_obj->data_src, sizeof(uint32_t),
         DATA_SEGMENTS * DMA_PAGE_SIZE, fp);
  fclose(fp);
  fp = fopen("file_dst.csv", "w");
  if (fp <= 0)
    return 0;
  fwrite(dma_alloc_obj->data_dst, sizeof(uint32_t),
         DATA_SEGMENTS * DMA_PAGE_SIZE, fp);
  fclose(fp);
  return 1;
}

int bcm2835_test_sdm2csv(dma_mem_alloc_t *dma_alloc_obj) {
  // We are going to print src + dst data to csv
  FILE *fp;
  fp = fopen("file_src.csv", "w");
  if (fp <= 0)
    return 0;

  int i;
  char byte[] = "0,0,0,0";
  for(i=0;i<DATA_SEGMENTS*DMA_PAGE_SIZE; i++){
    if(dma_alloc_obj->data_src[i] & ch1_pos_key) byte[0] = '1'; else byte[0] = '0';
    if(dma_alloc_obj->data_src[i] & ch2_pos_key) byte[2] = '1'; else byte[2] = '0';
    if(dma_alloc_obj->data_src[i] & ch3_pos_key) byte[4] = '1'; else byte[4] = '0';
    if(dma_alloc_obj->data_src[i] & ch4_pos_key) byte[6] = '1'; else byte[6] = '0';
    fprintf(fp,"%s\n",byte);
  }

  fclose(fp);
  fp = fopen("file_dst.csv", "w");
  if (fp <= 0)
    return 0;
  for(i=0;i<DATA_SEGMENTS*DMA_PAGE_SIZE; i++){
    if(dma_alloc_obj->data_dst[i] & ch1_pos_key) byte[0] = '1'; else byte[0] = '0';
    if(dma_alloc_obj->data_dst[i] & ch2_pos_key) byte[2] = '1'; else byte[2] = '0';
    if(dma_alloc_obj->data_dst[i] & ch3_pos_key) byte[4] = '1'; else byte[4] = '0';
    if(dma_alloc_obj->data_dst[i] & ch4_pos_key) byte[6] = '1'; else byte[6] = '0';
    fprintf(fp,"%s\n",byte);
  }
  fclose(fp);
  return 1;
}

/*
// Configure I2S/PCM
bcm2835_i2s_clock_config(BCM2835_I2S_INV_EXTERNAL_CLK); // Safe Setup for rising
EDGE Clock Master
bcm2835_i2s_frame_config(BCM2835_I2S_CHANNEL1, BCM2835_I2S_INV_INTERNAL_FS, 0,
CMD_LEN, 0);
bcm2835_i2s_config_transmit(BCM2835_I2S_CHANNEL1, 0, CMD_LEN, 0);
bcm2835_i2s_config_recieve(BCM2835_I2S_CHANNEL1, 0, CMD_LEN, 0);

#define		CMD_LEN			(32)
#define		PWM_DIV
(BCM2835_PWM_CLOCK_DIVIDER_4) // 19.2 /4 /2

void bcm2835_test_i2s_config(char test_config[32]){ 	// Configure SPI
register
        int j;
        uint32_t config_word=0;
        for(j=0;j<32;j++){
                if( test_config[j] == '1' ) config_word |= 0x1<<j;
        }

        if(test_config[10] == '1' ) bcm2835_gpio_set(i2s_en);
        else bcm2835_gpio_clr(i2s_en);

        bcm2835_i2s_start_tx();
        bcm2835_delayMicroseconds(100);
        bcm2835_i2s_write_fifo(config_word);
        bcm2835_i2s_write_fifo(config_word);
        bcm2835_i2s_write_fifo(config_word);
        bcm2835_i2s_write_fifo(config_word);
        bcm2835_i2s_write_fifo(config_word);
        bcm2835_i2s_write_fifo(config_word);
        bcm2835_delayMicroseconds(40+(int)(1.5*PWM_DIV*CMD_LEN/BCM2835_PWM_OSC_FRQ_MHZ));
// data to preamble
        bcm2835_i2s_en_strobe();
        bcm2835_delayMicroseconds((int)(1.5*PWM_DIV*CMD_LEN/BCM2835_PWM_OSC_FRQ_MHZ));
// strobe for one data frame
        bcm2835_i2s_dis_strobe();
        bcm2835_delayMicroseconds((int)(2*PWM_DIV*CMD_LEN/BCM2835_PWM_OSC_FRQ_MHZ));
// let data to tail on
}

int bcm2835_test_dma_transfer(void) // MUST WIRE UP: FSCLK = DIN && BCLK == PWM0
{
        fprintf(stdout,"Initializing Test... \n");
        // Prepare Periphery

        bcm2835_gpio_fsel(pwm_clk0, BCM2835_GPIO_FSEL_ALT0);
        bcm2835_pwm_set_clock(BCM2835_PWM_CLOCK_DIVIDER_8);
        bcm2835_pwm_set_range(0, 2);
        bcm2835_pwm_set_data(0, 1);
        bcm2835_pwm_set_mode(0, 1, 1);  // USE MARK SPACE MODE TO DECREASE
TRANSISIONS

        bcm2835_gpio_fsel(i2s_bclk, BCM2835_GPIO_FSEL_ALT0);
        bcm2835_gpio_fsel(i2s_fclk, BCM2835_GPIO_FSEL_ALT0);
        bcm2835_gpio_fsel(i2s_dout, BCM2835_GPIO_FSEL_ALT0);
        bcm2835_gpio_fsel(i2s_din, BCM2835_GPIO_FSEL_ALT0);

        bcm2835_i2s_clock_config(BCM2835_I2S_INV_EXTERNAL_CLK); // Safe Setup
for rising EDGE Clock Master
        bcm2835_i2s_frame_config(BCM2835_I2S_INTERNAL_FS, 8, 4); // uint32_t
frame_config, int flen, int fslen
        bcm2835_i2s_config_transmit(BCM2835_I2S_CHANNEL1, 8, 0, 0); // uint32_t
channel_config, int length, int channel1_pos, int channel2_pos
        bcm2835_i2s_config_recieve(BCM2835_I2S_CHANNEL2, 8, 0, 0); // uint32_t
channel_config, int length, int channel1_pos, int channel2_pos
        bcm2835_i2s_dma_en();

        // Perpare data allocations
        if(!bcm2835_test_init_cache()) return 0;

        // Generate & Prepare test data
        int i;
        for(i=0;i<DATA_SEGMENTS;i++){
                bcm2835_test_generate_segment(i);
        }

        bcm2835_test_setup_cblocks();

        // We don't point to the first CB if we arre not performing circular
transfer
        if!(DMA_CIRC_TRSFR){
                (dma_alloc_obj.cblk_src+DATA_SEGMENTS*8-8)->next =0;
                (dma_alloc_obj.cblk_dst+DATA_SEGMENTS*8-8)->next =0;
        }

        fprintf(stdout,"Starting Test... \n");
        bcm2835_test_start_i2sdma();

        if( DMA_CIRC_TRSFR == 0 && DATA_SEGMENTS > 1 ) {
                while(bcm2835_dma_get_cblk(channel_dst) ==
dma_alloc_obj->cblk_dst->next){usleep(DMA_PAGE_SIZE/8);}
                while(bcm2835_dma_get_cblk(channel_dst) !=
dma_alloc_obj->cblk_dst->next){usleep(DMA_PAGE_SIZE/8);}
                while(bcm2835_dma_get_cblk(channel_dst) ==
dma_alloc_obj->cblk_dst->next){usleep(DMA_PAGE_SIZE/8);}
                while(bcm2835_dma_get_cblk(channel_dst) !=
dma_alloc_obj->cblk_dst->next){usleep(DMA_PAGE_SIZE/8);}
                (dma_alloc_obj.cblk_src+DATA_SEGMENTS*8-8)->next =0;
                (dma_alloc_obj.cblk_dst+DATA_SEGMENTS*8-8)->next =0;
        }

        if( DMA_CIRC_TRSFR == 0 && DATA_SEGMENTS == 1 ) {
                while(bcm2835_dma_get_length(channel_dst) >=
DMA_PAGE_SIZE/2){usleep(DMA_PAGE_SIZE/8);}
                while(bcm2835_dma_get_length(channel_dst) <=
DMA_PAGE_SIZE/2){usleep(DMA_PAGE_SIZE/8);}
                while(bcm2835_dma_get_length(channel_dst) >=
DMA_PAGE_SIZE/2){usleep(DMA_PAGE_SIZE/8);}
                while(bcm2835_dma_get_length(channel_dst) <=
DMA_PAGE_SIZE/2){usleep(DMA_PAGE_SIZE/8);}
                (dma_alloc_obj->cblk_src+DATA_SEGMENTS*8-8)->next =0;
                (dma_alloc_obj->cblk_dst+DATA_SEGMENTS*8-8)->next =0;
        }

        while(bcm2835_dma_get_cblk(channel_dst) != 0){usleep(DMA_PAGE_SIZE/8);}
        usleep(100);

        // Test output procedure TBC
        bcm2835_test_output_file();

        // Clean Up
        bcm2835_test_end_full();
        return 1;
}

int bcm2835_test_frequency_ratio(void) // MUST WIRE UP: FSCLK = DIN && BCLK IS
SLAVE
{
        // start up I2S GPIOs
        bcm2835_gpio_set_pud(i2s_en, BCM2835_GPIO_PUD_OFF);
        bcm2835_gpio_fsel(i2s_en, BCM2835_GPIO_FSEL_OUTP);
        bcm2835_gpio_set(i2s_en);
        bcm2835_gpio_fsel(i2s_bclk, BCM2835_GPIO_FSEL_ALT0);
        bcm2835_gpio_fsel(i2s_fclk, BCM2835_GPIO_FSEL_INPT);
        bcm2835_gpio_fsel(i2s_din, BCM2835_GPIO_FSEL_ALT0);
        bcm2835_gpio_fsel(i2s_dout, BCM2835_GPIO_FSEL_ALT0);

        // First need to check if input frame is valid
        bcm2835_i2s_clock_config(BCM2835_I2S_INV_EXTERNAL_CLK); // Safe Setup
for rising EDGE Clock Master
        bcm2835_i2s_frame_config(BCM2835_I2S_INTERNAL_FS, 8, 0); // uint32_t
frame_config, int flen > int fslen
        bcm2835_i2s_config_transmit(BCM2835_I2S_CHANNEL2, 8, 0, 0); // uint32_t
channel_config, int length, int channel1_pos, int channel2_pos
        bcm2835_i2s_config_recieve(BCM2835_I2S_CHANNEL1, 8, 1, 1); // uint32_t
channel_config, int length, int channel1_pos, int channel2_pos

        bcm2835_i2s_clear_rx_fifo();
        bcm2835_i2s_clear_tx_fifo();
        bcm2835_i2s_enable();
        bcm2835_i2s_start();


        bcm2835_delayMicroseconds(50);  // Wait for FIFO to fill
        int data;
        data = bcm2835_i2s_read_fifo();
        data = bcm2835_i2s_read_fifo();
        int cz = 0;
        while(data & (data >> cz) ) cz++;
        bcm2835_i2s_stop();
        bcm2835_i2s_clear_rx_fifo();
        bcm2835_i2s_clear_tx_fifo();
        if(cz >= 4){ // Undefined requirements for CZ > 4 TBD
                bcm2835_i2s_frame_config(BCM2835_I2S_EXTERNAL_FS, 8, 0);
                bcm2835_gpio_fsel(i2s_fclk, BCM2835_GPIO_FSEL_ALT0);
        }
        //else bcm2835_i2s_frame_config(BCM2835_I2S_INTERNAL_FS, 8, 0);

        return cz;
        bcm2835_gpio_clr(i2s_en);
}

*/
