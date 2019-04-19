//
// C lib Proceedures for performing SDM modulation
//
// 2019 Lieuwe B. Leene
//

#include "sdm_core.h"

void print_sdm1_state(sdm1_modulator_state_t *sdm) {
  fprintf(stdout, "S1=%d, Q=%d\n", sdm->S1, sdm->Q);
}

void print_sdm2_state(sdm2_modulator_state_t *sdm) {
  fprintf(stdout, "S1=%d, S2=%d, Q=%d\n", sdm->S1, sdm->S2, sdm->Q);
}

void print_sdm3_state(sdm3_modulator_state_t *sdm) {
  fprintf(stdout, "S1=%d, S2=%d, S3=%d, Q=%d\n", sdm->S1, sdm->S2, sdm->S3,
          sdm->Q);
}

void update_sdm1(sdm1_modulator_state_t *sdm, int input) {
  int S; // New State Values
  // Calculate feed-forward component
  S = input + sdm->S1; // 1
  // Update Noise Shaping states & output Q
  if (S > 0) {
    sdm->Q = true;
    sdm->S1 = sdm->S1 + input / 2 - INPUT_FDBK;
  } else {
    sdm->Q = false;
    sdm->S1 = sdm->S1 + input / 2 + INPUT_FDBK;
  }
}

void update_sdm2(sdm2_modulator_state_t *sdm, int input) {
  int S; // New State Values
  // Calculate feed-forward component
  S = input + 2 * sdm->S1 + sdm->S2; // 21
  sdm->S2 = sdm->S2 + sdm->S1 / 2;
  if (S > 0) {
    sdm->Q = true;
    sdm->S1 = sdm->S1 + input / 2 - INPUT_FDBK;
  } else {
    sdm->Q = false;
    sdm->S1 = sdm->S1 + input / 2 + INPUT_FDBK;
  }
}

void update_sdm3(sdm3_modulator_state_t *sdm, int input) {
  int S, F; // New State Values
  // Calculate feed-forward component
  S = input + 3 * sdm->S1 + 3 * sdm->S2 + sdm->S3; // 331
  // Calculate Resonator component
  F = sdm->S3 / 256; // divide by 256 for NTF zero;
  // Update Noise Shaping states & output Q
  sdm->S3 = sdm->S3 + sdm->S2 / 2;
  sdm->S2 = sdm->S2 + sdm->S1 / 2 - F;
  if (S > 0) {
    sdm->Q = true;
    sdm->S1 = sdm->S1 + input / 2 - INPUT_FDBK;
  } else {
    sdm->Q = false;
    sdm->S1 = sdm->S1 + input / 2 + INPUT_FDBK;
  }
}

void generate_sdm3_4ch_data(syth_cnfg_t *configuration, uint32_t *mem_ptr,
                        int count, SDM3_4ch_object_t *modulator) {
  // We are going to fill [count] elements
  int i = 0;
  while (i < count) {
    int j, k;
    int delta[4];
    for (j = 0; j < 4; j++) {
      // Need to use variables from active configuration to generate signals
      double signal =
          (double)((configuration->amplitude[j]) << (8*sizeof(int)-18)) *
          sin((double)(configuration->frequency[j] >> 2) *
              (double)modulator->time_keeper / ((double)SAMPLE_RATE));
      delta[j] = ((int)signal) / 2;
      delta[j] = (delta[j] - modulator->sdm_input[j]) / INTERP_RATE;
    }
    for (j = 0; (j < INTERP_RATE && i < count); j++) {
      // Run Quantisation Process for Interpolated points
      for (k = 0; k < 4; k++) {
        modulator->sdm_input[k] = modulator->sdm_input[k] + delta[k];
        update_sdm3(&(modulator->sdm[k]), modulator->sdm_input[k]);
      }
      // Save Quantisation Results
      uint32_t data_point;
      if (modulator->sdm[0].Q)
        data_point = ch1_pos_key;
      else
        data_point = ch1_neg_key;
      if (modulator->sdm[1].Q)
        data_point |= ch2_pos_key;
      else
        data_point |= ch2_neg_key;
      if (modulator->sdm[2].Q)
        data_point |= ch3_pos_key;
      else
        data_point |= ch3_neg_key;
      if (modulator->sdm[3].Q)
        data_point |= ch4_pos_key;
      else
        data_point |= ch4_neg_key;
      mem_ptr[i] = data_point;
      modulator->time_keeper++;
      i++;
    }
  }
}

void generate_sdm3_1ch_data(syth_cnfg_t *configuration, uint32_t *mem_ptr,
                        int count, SDM3_1ch_object_t *modulator) {
  // We are going to fill [count] elements
  int i = 0;
  while (i < count) {
    int j;
    int delta;

    // Need to use variables from active configuration to generate signals
    double signal =
        (double)((configuration->amplitude[0]) << (8*sizeof(int)-18)) *
        sin((double)(configuration->frequency[0] >> 2) *
            (double)modulator->time_keeper / ((double)SAMPLE_RATE));
    delta = ((int)signal) / 2;
    delta = (delta - modulator->sdm_input) / INTERP_RATE;


    for (j = 0; (j < INTERP_RATE && i < count); j++) {
      // Run Quantisation Process for Interpolated points
      modulator->sdm_input = modulator->sdm_input + delta;
      update_sdm3(&(modulator->sdm), modulator->sdm_input);

      // Save Quantisation Results
      uint32_t data_point;
      if (modulator->sdm.Q)
        data_point = ch1_pos_key | ch2_pos_key | ch3_pos_key | ch4_pos_key;
      else
        data_point = ch1_neg_key | ch2_neg_key | ch3_neg_key | ch4_neg_key;

      mem_ptr[i] = data_point;
      modulator->time_keeper++;
      i++;
    }
  }
}
