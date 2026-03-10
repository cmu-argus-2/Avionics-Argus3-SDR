#include "gnss_types.h"

int16_t gnss_sat16(int32_t x) {
    if (x > 32767) return 32767;
    if (x < -32768) return -32768;
    return (int16_t)x;
}

int32_t gnss_iabs32(int32_t x) {
    return (x < 0) ? -x : x;
}

int32_t gnss_mag2_iq32(int32_t i, int32_t q) {
    return i * i + q * q;
}