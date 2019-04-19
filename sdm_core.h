//
// C lib Proceedures for performing SDM modulation
//
// 2019 Lieuwe B. Leene
//

#ifndef __SDM_CORE_H__
#define __SDM_CORE_H__

#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// PROCEDURE Paramters
#define INPUT_RANGE (INT_MAX >> 4)
#define INPUT_FDBK (INT_MAX >> 5)

// 25MHz Fs -> 100kHz @ 256 OSR
#define INTERP_RATE 32
#define INPUT_RANGE (INT_MAX >> 4)
#define SAMPLE_RATE (0.5e6)

// VYGR-DDWS Pairs A/B(4/1) C/D(3/2) E/F(5/6) G/H(8/7)
#define ch1_pos_key 0b00000001
#define ch1_neg_key 0b00000010
#define ch2_pos_key 0b00000100
#define ch2_neg_key 0b00001000
#define ch3_pos_key 0b00010000
#define ch3_neg_key 0b00100000
#define ch4_pos_key 0b01000000
#define ch4_neg_key 0b10000000

typedef enum {
  SYNTHCORE_PAUSE = 0x2,  // Pause SDM process   //
  SYNTHCORE_ENABLE = 0x1, // Enable SDM process  //
  SYNTHCORE_STOP = 0x0,   // Stop SDM process    //
} ProcStatusCode;

// Strongly typed boolean
typedef enum { false, true } bool;

// Strongly typed Modulator States
typedef struct { // 1st Order SDM
  int S1;        // First Integrator
  bool Q;        // Quantiser Output
} sdm1_modulator_state_t;

typedef struct { // 2nd Order SDM
  int S1;        // First Integrator
  int S2;        // Second Integrator
  bool Q;        // Quantiser Output
} sdm2_modulator_state_t;

typedef struct { // 3rd Order SDM
  int S1;        // First Integrator
  int S2;        // Second Integrator
  int S3;        // Third Integrator
  bool Q;        // Quantiser Output
} sdm3_modulator_state_t;

typedef struct {
  int recording_index;
  ProcStatusCode status_code; // Start/Stop/Pause Indicator
  int amplitude[4];
  int frequency[4];
} syth_cnfg_t;

typedef struct {
  int sdm_input[4];
  int time_keeper;
  sdm3_modulator_state_t sdm[4];
} SDM3_4ch_object_t;

typedef struct {
  int sdm_input;
  int time_keeper;
  sdm3_modulator_state_t sdm;
} SDM3_1ch_object_t;

#ifdef __cplusplus
extern "C" {
#endif

// Helper function to print current modulator state
void print_sdm1_state(sdm1_modulator_state_t *sdm);
void print_sdm2_state(sdm2_modulator_state_t *sdm);
void print_sdm3_state(sdm3_modulator_state_t *sdm);

// Update proceedure to process input to track noise components
void update_sdm1(sdm1_modulator_state_t *sdm, int input);
void update_sdm2(sdm2_modulator_state_t *sdm, int input);
void update_sdm3(sdm3_modulator_state_t *sdm, int input);

// Signal Generation Proceedure for QUAD channel setup using keys
void generate_sdm3_4ch_data(syth_cnfg_t *configuration, uint32_t *mem_ptr,
                        int count, SDM3_4ch_object_t *modulator);
void generate_sdm3_1ch_data(syth_cnfg_t *configuration, uint32_t *mem_ptr,
                        int count, SDM3_1ch_object_t *modulator);


#ifdef __cplusplus
}
#endif

#endif /* SDM_CORE_H */
