#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>

#include <eff.h>
#include <eff/drivers/spi.h>
#include <eff/drivers/uart.h>

/* ---------------- UART (debug output) ----------------
 *
 * Board port layout (per user):
 *   /dev/ttyACM0 -> flashing (eff-flash)
 *   /dev/ttyACM1 -> power monitor
 *   /dev/ttyACM2 -> monitoring (this is what minicom should attach to)
 *
 * /dev/ttyACM2 is physically wired to UART_4 on the Efficient chip.
 * Per the UART + pinmux docs, any UART other than the boot UART must be
 * BOTH eff_uart_init()'d AND have its pin group switched to UART mode
 * via eff_pinmux_set(..., PINMUX_UART) before anything you write can
 * reach the pins.
 *
 * run_simple.sh now builds with -DEFF_STDIO_PORT=4, so printf() is also
 * routed to UART_4. We use BOTH printf() and eff_uart_puts() for the
 * early banner so that whichever path the runtime prefers, at least one
 * message reaches minicom. This is the same strategy main.c uses.
 */
#define UART_DEV        UART_4
#define UART_PINMUX     PINMUX_4

/* ---------------- SPI -----------------
 *
 * Pin group selection, per the pinmux PDF:
 *   - Only GPIO_0..GPIO_5 support alternate peripheral functions, so
 *     PINMUX_0..PINMUX_5 are the only valid pin-group handles. PINMUX_0
 *     for SPI_0 is fine. (The SDK SPI example itself pairs SPI_3 with
 *     PINMUX_3, confirming the index-matches-peripheral convention.)
 *   - PINMUX_SPI = "SPI only" mode for that pin group, which is what we
 *     want — no I2C/UART/RTC sharing on pin group 0.
 *   - Pin group 0 (SPI) and pin group 4 (UART debug) are independent,
 *     so configuring SPI_0 cannot disturb the UART_4 debug output.
 *
 * WARNING: eff_spi_xfer is master-only in the published SDK. The Pi
 * bridge (/dev/spidev0.0) is also master, so once this board runs, both
 * sides will be driving SCK/CS -> bus contention, no valid frames. The
 * smoke test intentionally still runs so we can at least confirm the
 * CPU + UART are alive; the link direction needs to be redesigned
 * before any real IQ frame will arrive.
 */
#define SPI_DEV         SPI_0
#define SPI_PINMUX      PINMUX_0
#define SPI_PINMUX_CFG  PINMUX_SPI

#define FRAME_MAGIC       0x49515130u
#define FRAME_VERSION     1u
#define FRAME_DATA_BYTES  2048u

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t payload_bytes;
    uint32_t sequence;
    uint32_t sample_rate_hz;
    uint32_t center_freq_hz;
    uint8_t  iq_u8[FRAME_DATA_BYTES];
} iq_spi_frame_t;

/* ---------------- debug output helpers ---------------- */

static int uart_debug_init(void)
{
    eff_uart_cfg_t cfg = EFF_UART_DEFAULTS;

    if (eff_uart_init(UART_DEV, cfg)) {
        return -1;
    }
    if (eff_pinmux_set(UART_PINMUX, PINMUX_UART)) {
        return -2;
    }
    return 0;
}

/* snprintf into a local buffer and shove it out UART_4. Avoids any
 * dependency on the stdio port the CMake build picked. */
static void dbg(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    eff_uart_puts(UART_DEV, buf);
}

/* ---------------- SPI ---------------- */

static int spi0_init(void)
{
    eff_spi_cfg_t spi_cfg = EFF_SPI_DEFAULTS;
    spi_cfg.xfer_mode = SPI_XFER_READ_ONLY;
    spi_cfg.bus_size  = SPI_BUS_SINGLE;

    /* IMPORTANT: order matches the canonical SPI example in the
     * Efficient docs (and the UART bring-up in main.c):
     *
     *     eff_spi_init(SPI_3, &spi_cfg);
     *     eff_pinmux_set(PINMUX_3, PINMUX_QSPI);
     *
     * i.e. configure the controller FIRST, then flip the pin group into
     * SPI mode. The previous version of this file did pinmux first,
     * which deviates from the docs and may have been contributing to
     * the "no output at all on minicom" symptom if the SPI bring-up
     * faulted before the first heartbeat could print. */
    if (eff_spi_init(SPI_DEV, &spi_cfg)) {
        return -1;
    }
    if (eff_pinmux_set(SPI_PINMUX, SPI_PINMUX_CFG)) {
        return -2;
    }
    return 0;
}

static int spi0_read_frame(iq_spi_frame_t *frame)
{
    memset(frame, 0, sizeof(*frame));
    /* eff_spi_xfer returns 0 on success, nonzero on error. */
    if (eff_spi_xfer(SPI_DEV, 0, 0, NULL, 0,
                     (uint8_t *)frame, (uint32_t)sizeof(*frame))) {
        return 0;
    }
    return 1;
}

/* ---------------- main ---------------- */

int main(void)
{
    /* printf() before any explicit init, mirroring main.c. With
     * -DEFF_STDIO_PORT=4 (see run_simple.sh) the stdio runtime will
     * route this to UART_4 and — empirically from main.c — handle
     * whatever implicit bring-up stdio needs. If the earlier smoke test
     * produced NO output at all, this line will at least tell us
     * whether the stdio path is working. */
    printf("\r\n[DEBUG] spi0 smoke: entering main (stdio path)\r\n");

    /* Bring up UART_4 FIRST, before anything else could try to talk. */
    if (uart_debug_init()) {
        /* Nothing we can do if this fails — there's no other output
         * path. Fall through silently so the CPU doesn't hang here. */
    }

    /* Immediate sanity print so we can see the board is alive before
     * any peripheral configuration touches pins. This uses the explicit
     * eff_uart_puts() path, which is what main.c actually relies on for
     * its visible output. */
    eff_uart_puts(UART_DEV, "\r\n=== spi0 smoke: UART_4 alive ===\r\n");

    iq_spi_frame_t frame;
    int rc;

    dbg("[DEBUG] spi0 smoke start\r\n");

    rc = spi0_init();
    if (rc) {
        dbg("[DEBUG] spi0 init failed rc=%d\r\n", rc);
        /* Don't return — keep the heartbeat going so you can confirm
         * the board hasn't silently crashed. */
    } else {
        dbg("[DEBUG] spi0 init ok, waiting for frames\r\n");
    }

    dbg("[DEBUG] NOTE: eff_spi_xfer is master-only per SDK docs;\r\n"
        "        with the Pi also as master you have bus contention.\r\n"
        "        Expect garbage frames or hangs until this is resolved.\r\n");

    uint32_t iter = 0;
    while (1) {
        /* Heartbeat every ~64 iterations so you can see the CPU is
         * alive even if eff_spi_xfer never returns valid data. */
        if ((iter++ & 0x3Fu) == 0u) {
            dbg("[DEBUG] spi0 loop iter=%lu\r\n", (unsigned long)iter);
        }

        if (rc) {
            /* Don't hammer the broken driver; just loop so the
             * heartbeat keeps coming out. */
            continue;
        }

        if (!spi0_read_frame(&frame)) {
            dbg("[DEBUG] spi0 read failed\r\n");
            continue;
        }

        dbg("[DEBUG] hdr magic=0x%08lx ver=%u bytes=%u seq=%lu fs=%lu cf=%lu i0=%u q0=%u\r\n",
            (unsigned long)frame.magic,
            (unsigned)frame.version,
            (unsigned)frame.payload_bytes,
            (unsigned long)frame.sequence,
            (unsigned long)frame.sample_rate_hz,
            (unsigned long)frame.center_freq_hz,
            (unsigned)frame.iq_u8[0],
            (unsigned)frame.iq_u8[1]);

        if (frame.magic != FRAME_MAGIC) {
            dbg("[DEBUG] bad magic\r\n");
        }
        if (frame.version != FRAME_VERSION) {
            dbg("[DEBUG] bad version\r\n");
        }
        if (frame.payload_bytes != FRAME_DATA_BYTES) {
            dbg("[DEBUG] bad payload_bytes\r\n");
        }
    }

    return 0;
}
