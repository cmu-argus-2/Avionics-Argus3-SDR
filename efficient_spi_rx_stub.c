#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "gnss_types.h"

#define FRAME_MAGIC      0x49515130u
#define FRAME_VERSION    1u
#define FRAME_DATA_BYTES 2048u
#define FRAME_SAMPLES    (FRAME_DATA_BYTES / 2u)  /* U8 I,Q interleaved */

#pragma pack(push, 1)
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t payload_bytes;
    uint32_t sequence;
    uint32_t sample_rate_hz;
    uint32_t center_freq_hz;
    uint8_t  iq_u8[FRAME_DATA_BYTES];
} iq_spi_frame_t;
#pragma pack(pop)

/*
 * TODO: replace this with the Efficient SDK SPI receive path.
 * It must block until exactly len bytes are received from the Pi.
 * Return true on success, false on link/protocol error.
 */
static bool spi_read_exact(void *dst, uint32_t len)
{
    (void)dst;
    (void)len;
    return false;
}

static void unpack_u8iq_to_iq16(const uint8_t *src, iq16_t *dst, uint32_t n_samples)
{
    for (uint32_t k = 0; k < n_samples; k++) {
        uint8_t i_u8 = src[2u * k + 0u];
        uint8_t q_u8 = src[2u * k + 1u];

        /* Convert RTL-SDR unsigned-8 IQ into signed 16-bit centered at 0. */
        dst[k].i = (int16_t)(((int32_t)i_u8 - 128) << 8);
        dst[k].q = (int16_t)(((int32_t)q_u8 - 128) << 8);
    }
}

/*
 * Drop-in replacement idea for your current read_gnss_samples_1ms().
 * One SPI frame carries 1024 complex samples == 1 ms at 1.024 Msps.
 */
int read_gnss_samples_1ms_from_spi(iq16_t *dst,
                                   int samples_per_ms,
                                   uint32_t *sample_rate_hz,
                                   uint32_t *center_freq_hz,
                                   uint32_t *sequence)
{
    iq_spi_frame_t frame;

    if ((uint32_t)samples_per_ms != FRAME_SAMPLES) {
        return 0;
    }

    if (!spi_read_exact(&frame, (uint32_t)sizeof(frame))) {
        return 0;
    }

    if (frame.magic != FRAME_MAGIC ||
        frame.version != FRAME_VERSION ||
        frame.payload_bytes != FRAME_DATA_BYTES) {
        return 0;
    }

    unpack_u8iq_to_iq16(frame.iq_u8, dst, FRAME_SAMPLES);

    if (sample_rate_hz)  *sample_rate_hz = frame.sample_rate_hz;
    if (center_freq_hz)  *center_freq_hz = frame.center_freq_hz;
    if (sequence)        *sequence = frame.sequence;

    return 1;
}
