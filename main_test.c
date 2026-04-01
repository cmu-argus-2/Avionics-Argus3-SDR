#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "gnss_types.h"

#define MAX_PRNS_TO_SEARCH   12
#define ACQ_DOPPLER_MIN_HZ  (-10000)
#define ACQ_DOPPLER_MAX_HZ   (10000)
#define ACQ_DOPPLER_STEP_HZ  (500)
#define ACQ_METRIC_THRESHOLD 1000000

#define NCO_LUT_BITS         8
#define NCO_LUT_SIZE         (1 << NCO_LUT_BITS)
#define NCO_LUT_MASK         (NCO_LUT_SIZE - 1)

#define TEST_ITERATIONS      10
#define TEST_SEED            12345

#define DEBUG_PRINT
#ifdef DEBUG_PRINT
#define DBG_PRINTF(...) printf(__VA_ARGS__)
#else
#define DBG_PRINTF(...)
#endif

/* Q15 cosine/sine LUT, 256 samples over 2*pi */
static const int16_t cos_lut_q15[NCO_LUT_SIZE] = {
    32767,32757,32728,32678,32609,32521,32412,32285,
    32137,31971,31785,31580,31356,31113,30852,30571,
    30273,29956,29621,29268,28898,28510,28105,27683,
    27245,26790,26319,25832,25329,24811,24279,23731,
    23170,22594,22005,21403,20787,20159,19519,18868,
    18204,17530,16846,16151,15446,14732,14010,13279,
    12539,11793,11039,10278,9512,8739,7962,7179,
    6393,5602,4808,4011,3212,2410,1608,804,
    0,-804,-1608,-2410,-3212,-4011,-4808,-5602,
    -6393,-7179,-7962,-8739,-9512,-10278,-11039,-11793,
    -12539,-13279,-14010,-14732,-15446,-16151,-16846,-17530,
    -18204,-18868,-19519,-20159,-20787,-21403,-22005,-22594,
    -23170,-23731,-24279,-24811,-25329,-25832,-26319,-26790,
    -27245,-27683,-28105,-28510,-28898,-29268,-29621,-29956,
    -30273,-30571,-30852,-31113,-31356,-31580,-31785,-31971,
    -32137,-32285,-32412,-32521,-32609,-32678,-32728,-32757,
    -32767,-32757,-32728,-32678,-32609,-32521,-32412,-32285,
    -32137,-31971,-31785,-31580,-31356,-31113,-30852,-30571,
    -30273,-29956,-29621,-29268,-28898,-28510,-28105,-27683,
    -27245,-26790,-26319,-25832,-25329,-24811,-24279,-23731,
    -23170,-22594,-22005,-21403,-20787,-20159,-19519,-18868,
    -18204,-17530,-16846,-16151,-15446,-14732,-14010,-13279,
    -12539,-11793,-11039,-10278,-9512,-8739,-7962,-7179,
    -6393,-5602,-4808,-4011,-3212,-2410,-1608,-804,
    0,804,1608,2410,3212,4011,4808,5602,
    6393,7179,7962,8739,9512,10278,11039,11793,
    12539,13279,14010,14732,15446,16151,16846,17530,
    18204,18868,19519,20159,20787,21403,22005,22594,
    23170,23731,24279,24811,25329,25832,26319,26790,
    27245,27683,28105,28510,28898,29268,29621,29956,
    30273,30571,30852,31113,31356,31580,31785,31971,
    32137,32285,32412,32521,32609,32678,32728,32757
};

static const int16_t sin_lut_q15[NCO_LUT_SIZE] = {
    0,804,1608,2410,3212,4011,4808,5602,
    6393,7179,7962,8739,9512,10278,11039,11793,
    12539,13279,14010,14732,15446,16151,16846,17530,
    18204,18868,19519,20159,20787,21403,22005,22594,
    23170,23731,24279,24811,25329,25832,26319,26790,
    27245,27683,28105,28510,28898,29268,29621,29956,
    30273,30571,30852,31113,31356,31580,31785,31971,
    32137,32285,32412,32521,32609,32678,32728,32757,
    32767,32757,32728,32678,32609,32521,32412,32285,
    32137,31971,31785,31580,31356,31113,30852,30571,
    30273,29956,29621,29268,28898,28510,28105,27683,
    27245,26790,26319,25832,25329,24811,24279,23731,
    23170,22594,22005,21403,20787,20159,19519,18868,
    18204,17530,16846,16151,15446,14732,14010,13279,
    12539,11793,11039,10278,9512,8739,7962,7179,
    6393,5602,4808,4011,3212,2410,1608,804,
    0,-804,-1608,-2410,-3212,-4011,-4808,-5602,
    -6393,-7179,-7962,-8739,-9512,-10278,-11039,-11793,
    -12539,-13279,-14010,-14732,-15446,-16151,-16846,-17530,
    -18204,-18868,-19519,-20159,-20787,-21403,-22005,-22594,
    -23170,-23731,-24279,-24811,-25329,-25832,-26319,-26790,
    -27245,-27683,-28105,-28510,-28898,-29268,-29621,-29956,
    -30273,-30571,-30852,-31113,-31356,-31580,-31785,-31971,
    -32137,-32285,-32412,-32521,-32609,-32678,-32728,-32757,
    -32767,-32757,-32728,-32678,-32609,-32521,-32412,-32285,
    -32137,-31971,-31785,-31580,-31356,-31113,-30852,-30571,
    -30273,-29956,-29621,-29268,-28898,-28510,-28105,-27683,
    -27245,-26790,-26319,-25832,-25329,-24811,-24279,-23731,
    -23170,-22594,-22005,-21403,-20787,-20159,-19519,-18868,
    -18204,-17530,-16846,-16151,-15446,-14732,-14010,-13279,
    -12539,-11793,-11039,-10278,-9512,-8739,-7962,-7179,
    -6393,-5602,-4808,-4011,-3212,-2410,-1608,-804
};

/* ---------------- output helper ---------------- */

static void print_measurement(const gnss_measurement_t *m) {
    printf("MEAS,%lu,%u,%ld,%ld,%ld,%ld,%ld,%ld,%u\n",
           (unsigned long)m->tow_ms,
           (unsigned)m->prn,
           (long)m->code_phase_q16,
           (long)m->doppler_hz,
           (long)m->carrier_phase_q16,
           (long)m->prompt_i,
           (long)m->prompt_q,
           (long)m->cn0_like,
           (unsigned)m->lock);
}

/* ---------------- local code helpers ---------------- */

static void generate_ca_code_prn(int prn, int8_t code[GPS_L1_CA_CHIPS]) {
    uint16_t lfsr = (uint16_t)(0x3FFu ^ (uint16_t)(prn * 37));
    for (int i = 0; i < GPS_L1_CA_CHIPS; i++) {
        int bit = ((lfsr >> 2) ^ (lfsr >> 9)) & 1;
        lfsr = (uint16_t)(((lfsr << 1) | bit) & 0x3FFu);
        code[i] = (lfsr & 1u) ? 1 : -1;
    }
}

static void make_oversampled_code(const int8_t ca_code[GPS_L1_CA_CHIPS],
                                  int8_t *dst,
                                  int samples_per_ms,
                                  int code_phase_offset)
{
    for (int n = 0; n < samples_per_ms; n++) {
        int chip = ((n * GPS_L1_CA_CHIPS) / samples_per_ms + code_phase_offset) % GPS_L1_CA_CHIPS;
        if (chip < 0) {
            chip += GPS_L1_CA_CHIPS;
        }
        dst[n] = ca_code[chip];
    }
}

/* ---------------- DSP kernels ---------------- */

static void mix_down_nco_kernel(const iq16_t *in,
                                iq16_t *out,
                                int n,
                                uint32_t phase0,
                                uint32_t phase_step)
{
    uint32_t phase = phase0;

    for (int k = 0; k < n; k++) {
        uint32_t idx = (phase >> (32 - NCO_LUT_BITS)) & NCO_LUT_MASK;

        int32_t ir = in[k].i;
        int32_t iq = in[k].q;
        int32_t nr = cos_lut_q15[idx];
        int32_t nq = -sin_lut_q15[idx];

        int32_t orr = gnss_q15_mul(ir, nr) - gnss_q15_mul(iq, nq);
        int32_t orq = gnss_q15_mul(ir, nq) + gnss_q15_mul(iq, nr);

        out[k].i = gnss_sat16(orr);
        out[k].q = gnss_sat16(orq);

        phase += phase_step;
    }
}

static void correlate_1ms_kernel(const iq16_t *samples,
                                 const int8_t *ca_oversampled,
                                 int n,
                                 int32_t *acc_i,
                                 int32_t *acc_q)
{
    int32_t si = 0;
    int32_t sq = 0;

    for (int k = 0; k < n; k++) {
        int32_t c = (int32_t)ca_oversampled[k];
        si += samples[k].i * c;
        sq += samples[k].q * c;
    }

    *acc_i = si;
    *acc_q = sq;
}

static void correlate_epl_kernel(const iq16_t *samples,
                                 const int8_t *code_e,
                                 const int8_t *code_p,
                                 const int8_t *code_l,
                                 int n,
                                 int32_t *e_i, int32_t *e_q,
                                 int32_t *p_i, int32_t *p_q,
                                 int32_t *l_i, int32_t *l_q)
{
    int32_t ei = 0, eq = 0, pi = 0, pq = 0, li = 0, lq = 0;

    for (int k = 0; k < n; k++) {
        int32_t sr = samples[k].i;
        int32_t sq = samples[k].q;

        ei += sr * code_e[k];
        eq += sq * code_e[k];
        pi += sr * code_p[k];
        pq += sq * code_p[k];
        li += sr * code_l[k];
        lq += sq * code_l[k];
    }

    *e_i = ei; *e_q = eq;
    *p_i = pi; *p_q = pq;
    *l_i = li; *l_q = lq;
}

/* ---------------- acquisition ---------------- */

static acq_result_t acquire_one_prn(const iq16_t *raw_1ms,
                                    int samples_per_ms,
                                    uint32_t fs_hz,
                                    int prn)
{
    static int8_t ca_code[GPS_L1_CA_CHIPS];
    static int8_t local_code[MAX_SAMPLES_MS];
    static iq16_t mixed[MAX_SAMPLES_MS];

    acq_result_t best;
    memset(&best, 0, sizeof(best));
    best.prn = prn;

    generate_ca_code_prn(prn, ca_code);

    for (int dop = ACQ_DOPPLER_MIN_HZ; dop <= ACQ_DOPPLER_MAX_HZ; dop += ACQ_DOPPLER_STEP_HZ) {
        uint32_t phase_step = gnss_phase_step_from_hz(dop, fs_hz);
        mix_down_nco_kernel(raw_1ms, mixed, samples_per_ms, 0u, phase_step);

        for (int code_phase = 0; code_phase < GPS_L1_CA_CHIPS; code_phase++) {
            int32_t ci;
            int32_t cq;
            int32_t metric;

            make_oversampled_code(ca_code, local_code, samples_per_ms, code_phase);
            correlate_1ms_kernel(mixed, local_code, samples_per_ms, &ci, &cq);
            metric = gnss_mag2_iq32(ci, cq);

            if (!best.found || metric > best.metric) {
                best.found = true;
                best.prn = prn;
                best.doppler_hz = dop;
                best.code_phase = code_phase;
                best.metric = metric;
            }
        }
    }

    return best;
}

/* ---------------- tracking ---------------- */

static void tracking_step(const iq16_t *raw_1ms,
                          int samples_per_ms,
                          uint32_t fs_hz,
                          track_state_t *st)
{
    static int8_t ca_code[GPS_L1_CA_CHIPS];
    static int8_t code_e[MAX_SAMPLES_MS];
    static int8_t code_p[MAX_SAMPLES_MS];
    static int8_t code_l[MAX_SAMPLES_MS];
    static iq16_t mixed[MAX_SAMPLES_MS];

    generate_ca_code_prn(st->prn, ca_code);

    {
        uint32_t phase0 = gnss_phase_q16_to_q32(st->carrier_phase_q16);
        uint32_t phase_step = gnss_phase_step_from_hz(st->carrier_hz, fs_hz);
        mix_down_nco_kernel(raw_1ms, mixed, samples_per_ms, phase0, phase_step);
    }

    make_oversampled_code(ca_code, code_e, samples_per_ms, st->code_phase - 1);
    make_oversampled_code(ca_code, code_p, samples_per_ms, st->code_phase);
    make_oversampled_code(ca_code, code_l, samples_per_ms, st->code_phase + 1);

    correlate_epl_kernel(mixed, code_e, code_p, code_l, samples_per_ms,
                         &st->early_i, &st->early_q,
                         &st->prompt_i, &st->prompt_q,
                         &st->late_i, &st->late_q);

    st->early_mag  = gnss_iabs32(st->early_i)  + gnss_iabs32(st->early_q);
    st->prompt_mag = gnss_iabs32(st->prompt_i) + gnss_iabs32(st->prompt_q);
    st->late_mag   = gnss_iabs32(st->late_i)   + gnss_iabs32(st->late_q);

    st->cn0_like = st->prompt_mag - ((st->early_mag + st->late_mag) >> 1);

    {
        int32_t den = st->early_mag + st->late_mag + 1;
        int32_t dll_err = ((st->early_mag - st->late_mag) * 128) / den;
        st->code_phase += (dll_err > 8) ? 1 : (dll_err < -8 ? -1 : 0);
    }

    if (st->prompt_q > 1000) {
        st->carrier_hz += 1;
    } else if (st->prompt_q < -1000) {
        st->carrier_hz -= 1;
    }

    st->carrier_phase_q16 += gnss_carrier_phase_advance_q16(st->carrier_hz);
    st->code_rate_q16 = gnss_q16_from_int(1023);
}

/* ---------------- measurement packaging ---------------- */

static void build_measurement(uint32_t tow_ms,
                              const track_state_t *st,
                              gnss_measurement_t *m)
{
    m->tow_ms = tow_ms;
    m->prn = (uint8_t)st->prn;
    m->code_phase_q16 = gnss_q16_from_int(st->code_phase);
    m->doppler_hz = st->carrier_hz;
    m->carrier_phase_q16 = st->carrier_phase_q16;
    m->prompt_i = st->prompt_i;
    m->prompt_q = st->prompt_q;
    m->cn0_like = st->cn0_like;
    m->lock = st->locked ? 1 : 0;
}

/* ---------------- random testbench input ---------------- */

static int16_t rand_s16_in_range(int min_v, int max_v)
{
    int span = max_v - min_v + 1;
    return (int16_t)(min_v + (rand() % span));
}

static int read_gnss_samples_1ms(iq16_t *dst, int samples_per_ms)
{
    for (int i = 0; i < samples_per_ms; i++) {
        dst[i].i = rand_s16_in_range(-2000, 2000);
        dst[i].q = rand_s16_in_range(-2000, 2000);
    }
    return 1;
}

/* ---------------- main testbench ---------------- */

int main(void)
{
    const uint32_t fs_hz = 5000000u;
    const int samples_per_ms = (int)(fs_hz / 1000u);

    static iq16_t raw_1ms[MAX_SAMPLES_MS];
    static track_state_t tracks[MAX_TRACKS];

    int track_count = 0;
    uint32_t tow_ms = 0;

    srand(TEST_SEED);
    memset(tracks, 0, sizeof(tracks));

    DBG_PRINTF("[TB] GNSS testbench start\n");
    DBG_PRINTF("[TB] fs_hz=%lu samples_per_ms=%d seed=%d iterations=%d\n",
               (unsigned long)fs_hz, samples_per_ms, TEST_SEED, TEST_ITERATIONS);

    if (samples_per_ms > MAX_SAMPLES_MS) {
        DBG_PRINTF("[TB] ERROR: samples_per_ms exceeds MAX_SAMPLES_MS\n");
        return -1;
    }

    if (!read_gnss_samples_1ms(raw_1ms, samples_per_ms)) {
        DBG_PRINTF("[TB] ERROR: initial sample generation failed\n");
        return -1;
    }

    DBG_PRINTF("[TB] Starting acquisition...\n");
    for (int prn = 1; prn <= MAX_PRNS_TO_SEARCH && track_count < MAX_TRACKS; prn++) {
        acq_result_t acq = acquire_one_prn(raw_1ms, samples_per_ms, fs_hz, prn);

        DBG_PRINTF("[TB] ACQ PRN=%d found=%d metric=%ld doppler=%ld code_phase=%d\n",
                   prn,
                   acq.found ? 1 : 0,
                   (long)acq.metric,
                   (long)acq.doppler_hz,
                   acq.code_phase);

        if (acq.found && acq.metric > ACQ_METRIC_THRESHOLD) {
            tracks[track_count].locked = true;
            tracks[track_count].prn = acq.prn;
            tracks[track_count].carrier_hz = acq.doppler_hz;
            tracks[track_count].code_phase = acq.code_phase;
            tracks[track_count].carrier_phase_q16 = 0;
            tracks[track_count].code_rate_q16 = 0;

            DBG_PRINTF("[TB] LOCK slot=%d PRN=%d doppler=%ld code_phase=%d metric=%ld\n",
                       track_count,
                       acq.prn,
                       (long)acq.doppler_hz,
                       acq.code_phase,
                       (long)acq.metric);

            track_count++;
        }
    }

    DBG_PRINTF("[TB] Acquisition complete, track_count=%d\n", track_count);

    for (int iter = 0; iter < TEST_ITERATIONS; iter++) {
        if (!read_gnss_samples_1ms(raw_1ms, samples_per_ms)) {
            DBG_PRINTF("[TB] ERROR: sample generation failed at iter=%d\n", iter);
            continue;
        }

        tow_ms++;
        DBG_PRINTF("[TB] ---- ITER %d / TOW %lu ----\n", iter, (unsigned long)tow_ms);

        for (int i = 0; i < track_count; i++) {
            gnss_measurement_t meas;

            if (!tracks[i].locked) {
                DBG_PRINTF("[TB] track %d not locked, skipping\n", i);
                continue;
            }

            tracking_step(raw_1ms, samples_per_ms, fs_hz, &tracks[i]);

            DBG_PRINTF("[TB] TRACK=%d PRN=%d prompt_i=%ld prompt_q=%ld cn0_like=%ld carrier_hz=%ld code_phase=%d\n",
                       i,
                       tracks[i].prn,
                       (long)tracks[i].prompt_i,
                       (long)tracks[i].prompt_q,
                       (long)tracks[i].cn0_like,
                       (long)tracks[i].carrier_hz,
                       tracks[i].code_phase);

            build_measurement(tow_ms, &tracks[i], &meas);
            print_measurement(&meas);
        }
    }

    DBG_PRINTF("[TB] GNSS testbench done\n");
    return 0;
}