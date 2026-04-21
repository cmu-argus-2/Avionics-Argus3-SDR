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
 * /dev/ttyACM2 is physically wired to UART_3 on the Efficient chip.
 * This was confirmed empirically: with -DEFF_STDIO_PORT=3 the printf()
 * banner at the top of main() appears in minicom; with =4 it does not.
 * Earlier comments in this file (and in main.c) claimed UART_4 was the
 * monitoring UART — that was wrong, and is why nothing ever showed up
 * when we pushed output through eff_uart_puts(UART_4, ...).
 *
 * Per the UART + pinmux docs, any UART other than the boot UART must be
 * BOTH eff_uart_init()'d AND have its pin group switched to UART mode
 * via eff_pinmux_set(PINMUX_3, PINMUX_UART) before anything you write
 * can reach the pins. We do both below in uart_debug_init().
 *
 * run_simple.sh therefore builds with -DEFF_STDIO_PORT=3 so that
 * printf() and our explicit eff_uart_puts(UART_3, ...) both hit the
 * same physical wire, and we use both paths as a belt-and-suspenders
 * early sanity check.
 */
#define UART_DEV        UART_3
#define UART_PINMUX     PINMUX_3

/* ---------------- SPI -----------------
 *
 * Pin group selection, per the pinmux PDF and board wiring (per user):
 *   - SPI_0's physical pads on this board sit in pin group PINMUX_4,
 *     which is the pin group normally shared with UART_4. Since our
 *     debug UART is UART_3 (pin group PINMUX_3), UART_4 is unused, so
 *     we can safely hand all four pins of pin group 4 to SPI.
 *   - PINMUX_SPI = "SPI only" mode for that pin group. This is the
 *     right enum value because we do NOT want UART_4 multiplexed on
 *     the same group (PINMUX_SPI_UART would keep UART_4 wired up too).
 *   - Pin group 3 (UART debug, UART_3) and pin group 4 (SPI_0) are
 *     independent, so configuring SPI_0 cannot disturb UART_3 output.
 *   - Only GPIO_0..GPIO_5 support alternate functions; PINMUX_4 is
 *     well within that range.
 *
 * Link direction (per user): the Raspberry Pi is the SPI master and
 * clocks IQ frames *into* the Efficient board; the Efficient board is
 * the receiver. The published SDK's eff_spi_xfer is documented as
 * master-only, so driving it with SPI_XFER_READ_ONLY here will also
 * generate SCK/CS from the Efficient side and collide with the Pi.
 * That still needs to be resolved (slave-mode config on Efficient, or
 * flipping master/slave roles) before real frames will land; the smoke
 * test is only here to confirm CPU + UART are alive and that the SPI
 * bring-up call itself doesn't hang.
 */
#define SPI_DEV         SPI_0
#define SPI_PINMUX      PINMUX_4
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

/* snprintf into a local buffer and shove it out UART_3. Avoids any
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
    /* printf() before any explicit init. With -DEFF_STDIO_PORT=3 (see
     * run_simple.sh) the stdio runtime routes this to UART_3, which is
     * physically wired to /dev/ttyACM2 (monitoring). This line is the
     * first thing the user will actually see in minicom and confirms
     * the board booted and reached main(). */
    printf("\r\n[DEBUG] spi0 smoke: entering main (stdio path)\r\n");

    /* Bring up UART_3 FIRST, before anything else could try to talk. */
    if (uart_debug_init()) {
        /* Nothing we can do if this fails — there's no other output
         * path. Fall through silently so the CPU doesn't hang here. */
    }

    /* Immediate sanity print so we can see the board is alive before
     * any peripheral configuration touches pins. This uses the explicit
     * eff_uart_puts() path on UART_3 (the monitoring UART). */
    eff_uart_puts(UART_DEV, "\r\n=== spi0 smoke: UART_3 alive ===\r\n");

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
