#include "gnss_types.h"

int16_t gnss_sat16(int32_t x)
{
    if (x > 32767) return 32767;
    if (x < -32768) return -32768;
    return (int16_t)x;
}

int32_t gnss_iabs32(int32_t x)
{
    return (x < 0) ? -x : x;
}

int32_t gnss_mag2_iq32(int32_t i, int32_t q)
{
    return i * i + q * q;
}

int32_t gnss_q15_mul(int32_t a, int32_t b)
{
    return (a * b) >> GNSS_Q15_SHIFT;
}

int32_t gnss_q15_mul_round(int32_t a, int32_t b)
{
    int32_t prod = a * b;
    if (prod >= 0) {
        prod += (1 << (GNSS_Q15_SHIFT - 1));
    } else {
        prod -= (1 << (GNSS_Q15_SHIFT - 1));
    }
    return prod >> GNSS_Q15_SHIFT;
}

int32_t gnss_q16_from_int(int32_t x)
{
    return x << GNSS_Q16_SHIFT;
}

int32_t gnss_q16_to_int_floor(int32_t x)
{
    return x >> GNSS_Q16_SHIFT;
}

int32_t gnss_q16_mul_int(int32_t q16, int32_t x)
{
    return q16 * x;
}

uint32_t gnss_phase_step_from_hz(int32_t f_hz, uint32_t fs_hz)
{
    int64_t num = ((int64_t)f_hz << 32);
    return (uint32_t)(num / (int64_t)fs_hz);
}

uint32_t gnss_phase_q16_to_q32(int32_t phase_q16)
{
    return ((uint32_t)phase_q16) << 16;
}

int32_t gnss_carrier_phase_advance_q16(int32_t carrier_hz)
{
    return carrier_hz << 4;
}

int32_t gnss_corr_abs_metric(const gnss_corr32_t *c)
{
    return gnss_iabs32(c->i) + gnss_iabs32(c->q);
}

void gnss_corr_set(gnss_corr32_t *c, int32_t i, int32_t q)
{
    c->i = i;
    c->q = q;
}