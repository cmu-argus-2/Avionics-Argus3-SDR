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

    /* Pinmux BEFORE the controller so the pins are already in SPI mode
       when eff_spi_init samples / drives them. */
    if (eff_pinmux_set(SPI_PINMUX, SPI_PINMUX_CFG)) {
        return -1;
    }
    if (eff_spi_init(SPI_DEV, &spi_cfg)) {
        return -2;
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
    /* CRITICAL: the Efficient libc leaves stdout block-buffered when it is
       attached to a UART (no tty detection). This program only emits a few
       banner lines before blocking forever inside eff_spi_xfer() waiting
       for the Pi master to clock out a frame, so the buffer never fills
       and nothing ever reaches /dev/ttyACM2. Force unbuffered stdio. */
    setvbuf(stdout, NULL, _IONBF, 0);

    iq_spi_frame_t frame;
    int rc;

    DBG_PRINTF("Hello from spi0 smoke test\r\n");
    fflush(stdout);
    DBG_PRINTF("[DEBUG] spi0 smoke start\r\n");
    fflush(stdout);

    rc = spi0_init();
    if (rc) {
        DBG_PRINTF("[DEBUG] spi0 init failed rc=%d\r\n", rc);
        fflush(stdout);
        return -1;
    }

    DBG_PRINTF("[DEBUG] spi0 init ok, waiting for frames\r\n");
    fflush(stdout);

    uint32_t iter = 0;
    while (1) {
        /* Heartbeat so you can tell the CPU is alive even if the SPI
           transfer is blocking or returning garbage. */
        if ((iter++ & 0x3Fu) == 0u) {
            DBG_PRINTF("[DEBUG] spi0 loop iter=%lu\r\n",
                       (unsigned long)iter);
            fflush(stdout);
        }

        if (!spi0_read_frame(&frame)) {
            DBG_PRINTF("[DEBUG] spi0 read failed\r\n");
            fflush(stdout);
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
        fflush(stdout);

        if (frame.magic != FRAME_MAGIC) {
            DBG_PRINTF("[DEBUG] bad magic\r\n");
        }
        if (frame.version != FRAME_VERSION) {
            DBG_PRINTF("[DEBUG] bad version\r\n");
        }
        if (frame.payload_bytes != FRAME_DATA_BYTES) {
            DBG_PRINTF("[DEBUG] bad payload_bytes\r\n");
        }
        fflush(stdout);
    }

    return 0;
}
