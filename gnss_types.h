#ifndef GNSS_TYPES_H
#define GNSS_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* ---------------- configuration ---------------- */

#define GPS_L1_CA_CHIPS 1023
#define MAX_SAMPLES_MS  5000
#define MAX_TRACKS      12

/* ---------------- fixed-point / sample datatypes ---------------- */

typedef struct {
    int16_t i;
    int16_t q;
} iq16_t;

typedef struct {
    bool    found;
    int     prn;
    int     doppler_hz;
    int     code_phase;
    int32_t metric;
} acq_result_t;

typedef struct {
    bool    locked;
    int     prn;

    /* tracking state */
    int     carrier_hz;
    int     code_phase;

    /* correlation outputs */
    int32_t early_i,  early_q;
    int32_t prompt_i, prompt_q;
    int32_t late_i,   late_q;

    /* derived metrics */
    int32_t early_mag;
    int32_t prompt_mag;
    int32_t late_mag;
    int32_t cn0_like;

    /* optional observables */
    int32_t carrier_phase_q16;
    int32_t code_rate_q16;
} track_state_t;

typedef struct {
    uint32_t tow_ms;
    uint8_t  prn;
    int32_t  code_phase_q16;
    int32_t  doppler_hz;
    int32_t  carrier_phase_q16;
    int32_t  prompt_i;
    int32_t  prompt_q;
    int32_t  cn0_like;
    uint8_t  lock;
} gnss_measurement_t;

/* ---------------- datatype helper functions ---------------- */

int16_t gnss_sat16(int32_t x);
int32_t gnss_iabs32(int32_t x);
int32_t gnss_mag2_iq32(int32_t i, int32_t q);

#endif