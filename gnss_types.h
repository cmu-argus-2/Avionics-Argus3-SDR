#ifndef GNSS_TYPES_H
#define GNSS_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#define GPS_L1_CA_CHIPS   1023
#define MAX_SAMPLES_MS    5000
#define MAX_TRACKS        12

/* Fixed-point formats */
#define GNSS_Q15_SHIFT    15
#define GNSS_Q16_SHIFT    16
#define GNSS_PHASE_SHIFT  32

typedef struct {
    int16_t i;
    int16_t q;
} iq16_t;

typedef struct {
    int32_t i;
    int32_t q;
} iq32_t;

typedef struct {
    int32_t i;
    int32_t q;
} gnss_corr32_t;

typedef struct {
    bool found;
    int prn;
    int32_t doppler_hz;
    int32_t code_phase;
    int32_t metric;
} acq_result_t;

typedef struct {
    bool locked;
    int prn;

    int32_t carrier_hz;
    int32_t code_phase;

    int32_t carrier_phase_q16;
    int32_t code_rate_q16;

    int32_t early_i,  early_q;
    int32_t prompt_i, prompt_q;
    int32_t late_i,   late_q;

    int32_t early_mag;
    int32_t prompt_mag;
    int32_t late_mag;
    int32_t cn0_like;
} track_state_t;

typedef struct {
    uint32_t tow_ms;
    uint8_t prn;
    int32_t code_phase_q16;
    int32_t doppler_hz;
    int32_t carrier_phase_q16;
    int32_t prompt_i;
    int32_t prompt_q;
    int32_t cn0_like;
    uint8_t lock;
} gnss_measurement_t;

/* Saturation / integer helpers */
int16_t gnss_sat16(int32_t x);
int32_t gnss_iabs32(int32_t x);
int32_t gnss_mag2_iq32(int32_t i, int32_t q);

/* Fixed-point helpers */
int32_t gnss_q15_mul(int32_t a, int32_t b);
int32_t gnss_q15_mul_round(int32_t a, int32_t b);
int32_t gnss_q16_from_int(int32_t x);
int32_t gnss_q16_to_int_floor(int32_t x);
int32_t gnss_q16_mul_int(int32_t q16, int32_t x);

/* Phase / frequency helpers */
uint32_t gnss_phase_step_from_hz(int32_t f_hz, uint32_t fs_hz);
uint32_t gnss_phase_q16_to_q32(int32_t phase_q16);
int32_t gnss_carrier_phase_advance_q16(int32_t carrier_hz);

/* Correlator helpers */
int32_t gnss_corr_abs_metric(const gnss_corr32_t *c);
void gnss_corr_set(gnss_corr32_t *c, int32_t i, int32_t q);

#endif