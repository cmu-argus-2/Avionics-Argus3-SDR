#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include <eff.h>
#include <eff/drivers/spi.h>
#include "gnss_types.h"

#define MAX_PRNS_TO_SEARCH   1
#define ACQ_DOPPLER_MIN_HZ  (-10000)
#define ACQ_DOPPLER_MAX_HZ   (10000)
#define ACQ_DOPPLER_STEP_HZ  (500)
#define ACQ_METRIC_THRESHOLD 1000000

#define NCO_LUT_BITS         8
#define NCO_LUT_SIZE         (1 << NCO_LUT_BITS)
#define NCO_LUT_MASK         (NCO_LUT_SIZE - 1)

#define FRAME_MAGIC          0x49515130u
#define FRAME_VERSION        1u
#define FRAME_DATA_BYTES     2048u
#define FRAME_SAMPLES        (FRAME_DATA_BYTES / 2u)
#define TEST_ITERATIONS      0   /* unused in streaming build */

#define SPI_DEV             SPI_0
#define SPI_PINMUX          PINMUX_0
#define SPI_PINMUX_CFG      PINMUX_SPI

#define TS_SEC(us_)   ((unsigned long)((us_) / 1000000ULL))
#define TS_USEC(us_)  ((unsigned long)((us_) % 1000000ULL))

typedef struct {
    uint64_t acq_prns_tested;
    uint64_t acq_doppler_bins;
    uint64_t acq_code_phases;
    uint64_t acq_trials;

    uint64_t mix_calls;
    uint64_t corr1_calls;
    uint64_t corr3_calls;

    uint64_t mix_samples;
    uint64_t corr1_samples;
    uint64_t corr3_samples;

    uint64_t input_samples_generated;
    uint64_t measurements_emitted;

    uint64_t t_prog_start_us;
    uint64_t t_acq_start_us;
    uint64_t t_acq_end_us;
    uint64_t t_track_start_us;
    uint64_t t_track_end_us;
    uint64_t t_prog_end_us;
} perf_stats_t;

static perf_stats_t g_stats;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t payload_bytes;
    uint32_t sequence;
    uint32_t sample_rate_hz;
    uint32_t center_freq_hz;
    uint8_t  iq_u8[FRAME_DATA_BYTES];
} iq_spi_frame_t;

static uint32_t g_last_seq = 0;
static bool g_have_last_seq = false;
static uint32_t g_last_center_freq_hz = 0;

/* ---------------- SPI0 raw-IQ input ---------------- */


/* ---------------- Q15 LUTs ---------------- */

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

/* ---------------- timing / logging ---------------- */

static uint64_t now_us(void)
{
    return uptime_us();
}

static uint64_t rel_us(void)
{
    return now_us() - g_stats.t_prog_start_us;
}

static void print_measurement(const gnss_measurement_t *m)
{
    uint64_t t = rel_us();
    printf("[T=%lu.%06lu s] [MEAS] %lu,%u,%ld,%ld,%ld,%ld,%ld,%ld,%u\r\n",
           TS_SEC(t), TS_USEC(t),
           (unsigned long)m->tow_ms,
           (unsigned)m->prn,
           (long)m->code_phase_q16,
           (long)m->doppler_hz,
           (long)m->carrier_phase_q16,
           (long)m->prompt_i,
           (long)m->prompt_q,
           (long)m->cn0_like,
           (unsigned)m->lock);
    g_stats.measurements_emitted++;
}

/* ---------------- sample source from SPI0 ---------------- */

static int spi0_init_for_iq(void)
{
    eff_spi_cfg_t spi_cfg = EFF_SPI_DEFAULTS;

    spi_cfg.xfer_mode = SPI_XFER_READ_ONLY;
    spi_cfg.bus_size = SPI_BUS_SINGLE;

    if (eff_spi_init(SPI_DEV, &spi_cfg)) {
        return -1;
    }
    if (eff_pinmux_set(SPI_PINMUX, SPI_PINMUX_CFG)) {
        return -1;
    }
    return 0;
}

static int spi0_read_frame(iq_spi_frame_t *frame)
{
    if (eff_spi_xfer(SPI_DEV, 0, 0, NULL, 0,
                     (uint8_t *)frame, (uint32_t)sizeof(*frame))) {
        return 0;
    }
    return 1;
}

static void unpack_u8iq_to_iq16(const uint8_t *src, iq16_t *dst, uint32_t n_samples)
{
    for (uint32_t k = 0; k < n_samples; k++) {
        uint8_t i_u8 = src[2u * k + 0u];
        uint8_t q_u8 = src[2u * k + 1u];

        dst[k].i = (int16_t)(((int32_t)i_u8 - 128) << 8);
        dst[k].q = (int16_t)(((int32_t)q_u8 - 128) << 8);
    }
}

static int read_gnss_samples_1ms(iq16_t *dst, int samples_per_ms, uint32_t expected_fs_hz)
{
    iq_spi_frame_t frame;

    if (samples_per_ms != (int)FRAME_SAMPLES) {
        printf("[ERR] samples_per_ms=%d but SPI frame holds %u samples\r\n",
               samples_per_ms, (unsigned)FRAME_SAMPLES);
        return 0;
    }

    if (!spi0_read_frame(&frame)) {
        printf("[ERR] SPI frame read failed\r\n");
        return 0;
    }

    if (frame.magic != FRAME_MAGIC ||
        frame.version != FRAME_VERSION ||
        frame.payload_bytes != FRAME_DATA_BYTES) {
        printf("[ERR] bad SPI frame hdr: magic=0x%08lx ver=%u bytes=%u\r\n",
               (unsigned long)frame.magic,
               (unsigned)frame.version,
               (unsigned)frame.payload_bytes);
        return 0;
    }

    if (frame.sample_rate_hz != expected_fs_hz) {
        printf("[ERR] frame fs_hz=%lu expected=%lu\r\n",
               (unsigned long)frame.sample_rate_hz,
               (unsigned long)expected_fs_hz);
        return 0;
    }

    if (g_have_last_seq && frame.sequence != (g_last_seq + 1u)) {
        printf("[WARN] SPI seq jump prev=%lu curr=%lu\r\n",
               (unsigned long)g_last_seq,
               (unsigned long)frame.sequence);
    }

    g_last_seq = frame.sequence;
    g_have_last_seq = true;
    g_last_center_freq_hz = frame.center_freq_hz;

    unpack_u8iq_to_iq16(frame.iq_u8, dst, FRAME_SAMPLES);
    g_stats.input_samples_generated += (uint64_t)FRAME_SAMPLES;
    return 1;
}

/* ---------------- rest of your GNSS code unchanged ---------------- */

static void generate_ca_code_prn(int prn, int8_t code[GPS_L1_CA_CHIPS])
{
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

static void mix_down_nco_kernel(const iq16_t *in,
                                iq16_t *out,
                                int n,
                                uint32_t phase0,
                                uint32_t phase_step)
{
    uint32_t phase = phase0;
    g_stats.mix_calls++;
    g_stats.mix_samples += (uint64_t)n;

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
    g_stats.corr1_calls++;
    g_stats.corr1_samples += (uint64_t)n;

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
    g_stats.corr3_calls++;
    g_stats.corr3_samples += (uint64_t)(3ULL * (uint64_t)n);

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
    g_stats.acq_prns_tested++;

    for (int dop = ACQ_DOPPLER_MIN_HZ; dop <= ACQ_DOPPLER_MAX_HZ; dop += ACQ_DOPPLER_STEP_HZ) {
        uint32_t phase_step = gnss_phase_step_from_hz(dop, fs_hz);
        g_stats.acq_doppler_bins++;

        mix_down_nco_kernel(raw_1ms, mixed, samples_per_ms, 0u, phase_step);

        for (int code_phase = 0; code_phase < GPS_L1_CA_CHIPS; code_phase++) {
            int32_t ci;
            int32_t cq;
            int32_t metric;

            g_stats.acq_code_phases++;
            g_stats.acq_trials++;

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

static void print_evaluation(uint32_t fs_hz, int samples_per_ms, int track_count)
{
    uint64_t total_time_us = g_stats.t_prog_end_us  - g_stats.t_prog_start_us;
    uint64_t acq_time_us   = g_stats.t_acq_end_us   - g_stats.t_acq_start_us;
    uint64_t trk_time_us   = g_stats.t_track_end_us - g_stats.t_track_start_us;

    uint64_t input_bandwidth_Bps  = (uint64_t)fs_hz * 4ULL;
    uint64_t input_bandwidth_kBps = input_bandwidth_Bps / 1000ULL;

    uint64_t sample_throughput_sps = 0ULL;
    uint64_t acq_trials_per_s = 0ULL;
    uint64_t corr_contribs_total = 0ULL;
    uint64_t corr_contribs_per_s = 0ULL;

    uint64_t est_mix_ops = 0ULL;
    uint64_t est_corr1_ops = 0ULL;
    uint64_t est_corr3_ops = 0ULL;
    uint64_t est_total_ops = 0ULL;
    uint64_t est_ops_per_s = 0ULL;

    if (total_time_us > 0ULL) {
        sample_throughput_sps =
            (g_stats.input_samples_generated * 1000000ULL) / total_time_us;
    }

    if (acq_time_us > 0ULL) {
        acq_trials_per_s =
            (g_stats.acq_trials * 1000000ULL) / acq_time_us;
    }

    corr_contribs_total = g_stats.corr1_samples + g_stats.corr3_samples;
    if (total_time_us > 0ULL) {
        corr_contribs_per_s =
            (corr_contribs_total * 1000000ULL) / total_time_us;
    }

    est_mix_ops   = g_stats.mix_samples * 10ULL;
    est_corr1_ops = g_stats.corr1_samples * 4ULL;
    est_corr3_ops = (g_stats.corr3_samples / 3ULL) * 12ULL;
    est_total_ops = est_mix_ops + est_corr1_ops + est_corr3_ops;

    if (total_time_us > 0ULL) {
        est_ops_per_s = (est_total_ops * 1000000ULL) / total_time_us;
    }

    printf("\r\n================ GNSS PERFORMANCE EVALUATION ================\r\n");
    printf("CONFIG\r\n");
    printf("  fs_hz                         : %lu\r\n", (unsigned long)fs_hz);
    printf("  spi_frame_samples            : %u\r\n", (unsigned)FRAME_SAMPLES);
    printf("  samples_per_ms               : %d\r\n", samples_per_ms);
    printf("  center_freq_hz(last)         : %lu\r\n", (unsigned long)g_last_center_freq_hz);
    printf("  track_count                  : %d\r\n", track_count);
    printf("  acquisition correlator bank  : 1\r\n");
    printf("  tracking correlator bank     : 3\r\n");

    printf("\r\nTIMING\r\n");
    printf("  total_runtime_us             : %llu\r\n", (unsigned long long)total_time_us);
    printf("  acquisition_runtime_us       : %llu\r\n", (unsigned long long)acq_time_us);
    printf("  tracking_runtime_us          : %llu\r\n", (unsigned long long)trk_time_us);

    printf("\r\nWORK COUNTS\r\n");
    printf("  input_samples_generated      : %llu\r\n", (unsigned long long)g_stats.input_samples_generated);
    printf("  acquisition_trials           : %llu\r\n", (unsigned long long)g_stats.acq_trials);
    printf("  mix_samples                  : %llu\r\n", (unsigned long long)g_stats.mix_samples);
    printf("  corr1_samples                : %llu\r\n", (unsigned long long)g_stats.corr1_samples);
    printf("  corr3_samples                : %llu\r\n", (unsigned long long)g_stats.corr3_samples);
    printf("  measurements_emitted         : %llu\r\n", (unsigned long long)g_stats.measurements_emitted);

    printf("\r\nTHROUGHPUT / BANDWIDTH\r\n");
    printf("  input_sample_throughput_sps  : %llu\r\n", (unsigned long long)sample_throughput_sps);
    printf("  acquisition_trials_per_s     : %llu\r\n", (unsigned long long)acq_trials_per_s);
    printf("  corr_contribs_per_s          : %llu\r\n", (unsigned long long)corr_contribs_per_s);
    printf("  input_bandwidth_Bps          : %llu\r\n", (unsigned long long)input_bandwidth_Bps);
    printf("  input_bandwidth_kBps         : %llu\r\n", (unsigned long long)input_bandwidth_kBps);

    printf("\r\nOPS ESTIMATION\r\n");
    printf("  est_total_ops                : %llu\r\n", (unsigned long long)est_total_ops);
    printf("  est_ops_per_s                : %llu\r\n", (unsigned long long)est_ops_per_s);
    printf("============================================================\r\n");
}

int main(void)
{
    const uint32_t fs_hz = 1024000u;
    const int samples_per_ms = (int)FRAME_SAMPLES;

    static iq16_t raw_1ms[MAX_SAMPLES_MS];
    static track_state_t tracks[MAX_TRACKS];

    int track_count = 0;
    uint32_t tow_ms = 0;

    memset(&g_stats, 0, sizeof(g_stats));
    memset(tracks, 0, sizeof(tracks));
    g_stats.t_prog_start_us = now_us();

    {
        uint64_t t = rel_us();
        printf("[T=%lu.%06lu s] [SPI] GNSS SPI0 raw-IQ receiver start\r\n", TS_SEC(t), TS_USEC(t));
    }

    if (samples_per_ms > MAX_SAMPLES_MS) {
        printf("[ERR] samples_per_ms exceeds MAX_SAMPLES_MS\r\n");
        return -1;
    }

    if (spi0_init_for_iq()) {
        printf("[ERR] failed to init SPI0 raw-IQ input\r\n");
        return -1;
    }

    if (!read_gnss_samples_1ms(raw_1ms, samples_per_ms, fs_hz)) {
        printf("[ERR] failed to read initial SPI IQ frame\r\n");
        return -1;
    }

    g_stats.t_acq_start_us = now_us();
    printf("[ACQ] Starting acquisition\r\n");

    for (int prn = 1; prn <= MAX_PRNS_TO_SEARCH && track_count < MAX_TRACKS; prn++) {
        acq_result_t acq = acquire_one_prn(raw_1ms, samples_per_ms, fs_hz, prn);

        printf("[ACQ] PRN=%d found=%d metric=%ld doppler=%ld code_phase=%d\r\n",
               prn, acq.found ? 1 : 0, (long)acq.metric,
               (long)acq.doppler_hz, acq.code_phase);

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

    g_stats.t_acq_end_us = now_us();
    g_stats.t_track_start_us = now_us();

    while (1) {
        if (!read_gnss_samples_1ms(raw_1ms, samples_per_ms, fs_hz)) {
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
            print_measurement(&meas);
        }

        if ((tow_ms % 1000u) == 0u) {
            g_stats.t_track_end_us = now_us();
            g_stats.t_prog_end_us = now_us();
            print_evaluation(fs_hz, samples_per_ms, track_count);
        }
    }

    return 0;
}