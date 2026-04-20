#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <eff.h>
#include <eff/drivers/spi.h>

#define SPI_DEV        SPI_0
#define SPI_PINMUX     PINMUX_0
#define SPI_PINMUX_CFG PINMUX_SPI

#define FRAME_MAGIC       0x49515130u
#define FRAME_VERSION     1u
#define FRAME_DATA_BYTES  2048u

#define DEBUG_PRINT
#ifdef DEBUG_PRINT
#define DBG_PRINTF(...) printf(__VA_ARGS__)
#else
#define DBG_PRINTF(...)
#endif

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t payload_bytes;
    uint32_t sequence;
    uint32_t sample_rate_hz;
    uint32_t center_freq_hz;
    uint8_t  iq_u8[FRAME_DATA_BYTES];
} iq_spi_frame_t;

static int spi0_init(void)
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
    memset(frame, 0, sizeof(*frame));
    if (eff_spi_xfer(SPI_DEV, 0, 0, NULL, 0,
                     (uint8_t *)frame, (uint32_t)sizeof(*frame))) {
        return 0;
    }
    return 1;
}

int main(void)
{
    iq_spi_frame_t frame;

    DBG_PRINTF("[DEBUG] spi0 smoke start\r\n");

    if (spi0_init()) {
        DBG_PRINTF("[DEBUG] spi0 init failed\r\n");
        return -1;
    }

    DBG_PRINTF("[DEBUG] spi0 init ok, waiting for frames\r\n");

    while (1) {
        if (!spi0_read_frame(&frame)) {
            DBG_PRINTF("[DEBUG] spi0 read failed\r\n");
            continue;
        }

        DBG_PRINTF("[DEBUG] hdr magic=0x%08lx ver=%u bytes=%u seq=%lu fs=%lu cf=%lu i0=%u q0=%u\r\n",
                   (unsigned long)frame.magic,
                   (unsigned)frame.version,
                   (unsigned)frame.payload_bytes,
                   (unsigned long)frame.sequence,
                   (unsigned long)frame.sample_rate_hz,
                   (unsigned long)frame.center_freq_hz,
                   (unsigned)frame.iq_u8[0],
                   (unsigned)frame.iq_u8[1]);

        if (frame.magic != FRAME_MAGIC) {
            DBG_PRINTF("[DEBUG] bad magic\r\n");
        }
        if (frame.version != FRAME_VERSION) {
            DBG_PRINTF("[DEBUG] bad version\r\n");
        }
        if (frame.payload_bytes != FRAME_DATA_BYTES) {
            DBG_PRINTF("[DEBUG] bad payload_bytes\r\n");
        }
    }

    return 0;
}
