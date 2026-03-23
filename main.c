
// #include <eff.h>
// #include <stdio.h>
// #include "gnss_types.h"

// int main() {
//     int i = 0;

//     eff_pinmux_set(PINMUX_11, PINMUX_GPIO);
//     eff_gpio_dir_set(GPIO_11, GPIO_PIN_2, EFF_GPIO_OUT);

//     while (1) {
//         printf("Energy is everything! %i\r\n", i);

//         printf("Try GNSS type library: %x\r\n", gnss_sat16(0x0000FFFF));

//         if (i % 2)
//             eff_gpio_set(GPIO_11, GPIO_PIN_2);
//         else
//             eff_gpio_clear(GPIO_11, GPIO_PIN_2);

//         i++;
//         sleep(1);
//     }
// }

#include <eff.h>
#include <eff/drivers/uart.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "gnss_types.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define UART_DEV             UART_4
#define UART_PINMUX          PINMUX_4

#define MAX_PRNS_TO_SEARCH   12
#define ACQ_DOPPLER_MIN_HZ  (-10000)
#define ACQ_DOPPLER_MAX_HZ   (10000)
#define ACQ_DOPPLER_STEP_HZ  (500)
#define ACQ_METRIC_THRESHOLD 1000000
#define Q15_SHIFT            15

/* ---------------- UART ---------------- */

static int uart_init_for_gnss(void) {
    eff_uart_cfg_t cfg = EFF_UART_DEFAULTS;

    if (eff_uart_init(UART_DEV, cfg)) {
        return -1;
    }
    if (eff_pinmux_set(UART_PINMUX, PINMUX_UART)) {
        return -1;
    }
    return 0;
}

static void uart_send_measurement(const gnss_measurement_t *m) {
    char buf[256];
    snprintf(buf, sizeof(buf),
             "MEAS,%lu,%u,%ld,%ld,%ld,%ld,%ld,%ld,%u\r\n",
             (unsigned long)m->tow_ms,
             (unsigned)m->prn,
             (long)m->code_phase_q16,
             (long)m->doppler_hz,
             (long)m->carrier_phase_q16,
             (long)m->prompt_i,
             (long)m->prompt_q,
             (long)m->cn0_like,
             (unsigned)m->lock);
    eff_uart_puts(UART_DEV, buf);
}

/* ---------------- local code placeholders ---------------- */

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
        if (chip < 0) chip += GPS_L1_CA_CHIPS;
        dst[n] = ca_code[chip];
    }
}

static void make_nco_table(iq16_t *nco, int n, double fs_hz, double f_hz) {
    for (int k = 0; k < n; k++) {
        double ph = 2.0 * M_PI * f_hz * (double)k / fs_hz;
        int32_t ci = (int32_t)lrint(cos(ph) * 32767.0);
        int32_t cq = (int32_t)lrint(-sin(ph) * 32767.0);
        nco[k].i = gnss_sat16(ci);
        nco[k].q = gnss_sat16(cq);
    }
}

/* ---------------- Efficient kernels ---------------- */

__efficient__
void mix_down_kernel(const iq16_t *in, const iq16_t *nco, iq16_t *out, int n) {
    for (int k = 0; k < n; k++) {
        int32_t ir = in[k].i;
        int32_t iq = in[k].q;
        int32_t nr = nco[k].i;
        int32_t nq = nco[k].q;

        int32_t orr = (ir * nr - iq * nq) >> Q15_SHIFT;
        int32_t orq = (ir * nq + iq * nr) >> Q15_SHIFT;

        out[k].i = gnss_sat16(orr);
        out[k].q = gnss_sat16(orq);
    }
}

__efficient__
void correlate_1ms_kernel(const iq16_t *samples,
                          const int8_t *ca_oversampled,
                          int n,
                          int32_t *acc_i,
                          int32_t *acc_q)
{
    int32_t si = 0, sq = 0;
    for (int k = 0; k < n; k++) {
        int32_t c = (int32_t)ca_oversampled[k];
        si += samples[k].i * c;
        sq += samples[k].q * c;
    }
    *acc_i = si;
    *acc_q = sq;
}

__efficient__
void correlate_epl_kernel(const iq16_t *samples,
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
                                    double fs_hz,
                                    int prn)
{
    static int8_t ca_code[GPS_L1_CA_CHIPS];
    static int8_t local_code[MAX_SAMPLES_MS];
    static iq16_t nco[MAX_SAMPLES_MS];
    static iq16_t mixed[MAX_SAMPLES_MS];

    acq_result_t best;
    memset(&best, 0, sizeof(best));
    best.prn = prn;

    generate_ca_code_prn(prn, ca_code);

    for (int dop = ACQ_DOPPLER_MIN_HZ; dop <= ACQ_DOPPLER_MAX_HZ; dop += ACQ_DOPPLER_STEP_HZ) {
        make_nco_table(nco, samples_per_ms, fs_hz, (double)dop);
        mix_down_kernel(raw_1ms, nco, mixed, samples_per_ms);

        for (int code_phase = 0; code_phase < GPS_L1_CA_CHIPS; code_phase++) {
            int32_t ci, cq;
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
                          double fs_hz,
                          track_state_t *st)
{
    static int8_t ca_code[GPS_L1_CA_CHIPS];
    static int8_t code_e[MAX_SAMPLES_MS];
    static int8_t code_p[MAX_SAMPLES_MS];
    static int8_t code_l[MAX_SAMPLES_MS];
    static iq16_t nco[MAX_SAMPLES_MS];
    static iq16_t mixed[MAX_SAMPLES_MS];

    generate_ca_code_prn(st->prn, ca_code);
    make_nco_table(nco, samples_per_ms, fs_hz, (double)st->carrier_hz);
    mix_down_kernel(raw_1ms, nco, mixed, samples_per_ms);

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

    st->carrier_phase_q16 += (st->carrier_hz << 4);
    st->code_rate_q16 = (1023 << 16);
}

/* ---------------- measurement packaging ---------------- */

static void build_measurement(uint32_t tow_ms,
                              const track_state_t *st,
                              gnss_measurement_t *m)
{
    m->tow_ms = tow_ms;
    m->prn = (uint8_t)st->prn;
    m->code_phase_q16 = st->code_phase << 16;
    m->doppler_hz = st->carrier_hz;
    m->carrier_phase_q16 = st->carrier_phase_q16;
    m->prompt_i = st->prompt_i;
    m->prompt_q = st->prompt_q;
    m->cn0_like = st->cn0_like;
    m->lock = st->locked ? 1 : 0;
}

/* ---------------- sample source hook ---------------- */

static int read_gnss_samples_1ms(iq16_t *dst, int samples_per_ms) {
    for (int i = 0; i < samples_per_ms; i++) {
        dst[i].i = 0;
        dst[i].q = 0;
    }
    return 1;
}

/* ---------------- main ---------------- */

int main(void) {
    const double fs_hz = 5000000.0;
    const int samples_per_ms = (int)(fs_hz / 1000.0);

    static iq16_t raw_1ms[MAX_SAMPLES_MS];
    static track_state_t tracks[MAX_TRACKS];

    int track_count = 0;
    uint32_t tow_ms = 0;

    memset(tracks, 0, sizeof(tracks));

    if (samples_per_ms > MAX_SAMPLES_MS) {
        return -1;
    }

    if (uart_init_for_gnss()) {
        return -1;
    }

    eff_uart_puts(UART_DEV, "GNSS SDR start\r\n");

    if (!read_gnss_samples_1ms(raw_1ms, samples_per_ms)) {
        eff_uart_puts(UART_DEV, "sample input failed\r\n");
        return -1;
    }

    for (int prn = 1; prn <= MAX_PRNS_TO_SEARCH && track_count < MAX_TRACKS; prn++) {
        acq_result_t acq = acquire_one_prn(raw_1ms, samples_per_ms, fs_hz, prn);

        if (acq.found && acq.metric > ACQ_METRIC_THRESHOLD) {
            tracks[track_count].locked = true;
            tracks[track_count].prn = acq.prn;
            tracks[track_count].carrier_hz = acq.doppler_hz;
            tracks[track_count].code_phase = acq.code_phase;
            tracks[track_count].carrier_phase_q16 = 0;
            tracks[track_count].code_rate_q16 = 0;
            track_count++;
        }
    }

    while (1) {
        if (!read_gnss_samples_1ms(raw_1ms, samples_per_ms)) {
            eff_uart_puts(UART_DEV, "sample input failed\r\n");
            continue;
        }

        tow_ms++;

        for (int i = 0; i < track_count; i++) {
            gnss_measurement_t meas;

            if (!tracks[i].locked) {
                continue;
            }

            tracking_step(raw_1ms, samples_per_ms, fs_hz, &tracks[i]);
            build_measurement(tow_ms, &tracks[i], &meas);
            uart_send_measurement(&meas);
        }
    }

    return 0;
}
