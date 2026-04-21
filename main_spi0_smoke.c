#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>

#include <eff.h>
#include <eff/drivers/spi.h>
#include <eff/drivers/uart.h>

/* Direct register access to the ATCSPI200 controller behind SPI_0.
 * The SDK's eff_spi_cfg_t does not expose a master/slave field, but
 * the underlying IP has TRANSFMT.SLVMODE (bit 2) which flips the
 * controller into slave mode. We poke that bit manually after
 * eff_spi_init() has set up everything else. See
 * /home/argus/effcc/sdk/include/eff/atc/atcspi200.h lines 126-128. */
#include <eff/atc/atcspi200.h>

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
 *   - PINMUX_SPI = "SPI only" mode for that pin group.
 *
 * Link direction: the Raspberry Pi is the SPI master and clocks IQ
 * frames *into* the Efficient board; the Efficient board is the
 * receiver. Since the SDK's eff_spi_cfg_t cannot express master-vs-
 * slave, we let eff_spi_init() configure the controller as master
 * (the default), then manually set TRANSFMT.SLVMODE=1 on the raw
 * ATCSPI200 register block to switch to slave mode. In slave mode SCK
 * and CS are inputs driven by the Pi, and our MOSI becomes the data-in
 * line. No bus contention — only one device drives the clock.
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
    /* clk_div is irrelevant in slave mode — the Pi master drives SCK.
     * We leave it at the default from EFF_SPI_DEFAULTS. */

    /* Init order matches the canonical SPI example in the Efficient
     * docs (controller first, then pinmux). */
    if (eff_spi_init(SPI_DEV, &spi_cfg)) {
        return -1;
    }
    if (eff_pinmux_set(SPI_PINMUX, SPI_PINMUX_CFG)) {
        return -2;
    }

    /* Flip the controller into SLAVE mode by setting TRANSFMT.SLVMODE.
     *
     * The SDK wrapper has already written TRANSFMT with everything
     * else (data length, CPOL/CPHA, etc.) at defaults and left SLVMODE
     * at 0 (master). We OR in the SLVMODE bit as the very last step of
     * bring-up, before any data transfer is requested, so the hardware
     * sees a coherent slave-mode config when the Pi first asserts CS.
     *
     * SPI_0 is a handle to eff_spi_t, whose base_address points at the
     * ATCSPI200 register block (see sdk/drivers/spi/spi.c). */
    {
        ATCSPI200_RegDef *atc = (ATCSPI200_RegDef *)SPI_0->base_address;
        atc->TRANSFMT |= ATCSPI200_TRANSFMT_SLVMODE_MASK;
    }

    return 0;
}

static int spi0_read_frame(iq_spi_frame_t *frame)
{
    memset(frame, 0, sizeof(*frame));

    /* Bracket the xfer with debug prints so we can tell from minicom
     * whether eff_spi_xfer is blocking forever vs. returning an error.
     * Without these prints we can't distinguish "the driver hung on the
     * first call" from "everything is fine, just no frames yet". */
    dbg("[DEBUG] spi_xfer: enter (rx %u bytes)\r\n",
        (unsigned)sizeof(*frame));

    /* eff_spi_xfer returns 0 on success, nonzero on error. */
    int8_t rc = eff_spi_xfer(SPI_DEV, 0, 0, NULL, 0,
                             (uint8_t *)frame, (uint32_t)sizeof(*frame));

    dbg("[DEBUG] spi_xfer: exit rc=%d\r\n", (int)rc);

    if (rc) {
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

    /* IMPORTANT: the IQ frame is 2068 bytes. Allocating it on main()'s
     * stack is risky on this core — a stack overflow during SPI xfer
     * would look exactly like a hang (which is one of the candidate
     * causes for why we only see "iter=1" today). Move it to static
     * storage so we can rule that out as a confound. */
    static iq_spi_frame_t frame;
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

    dbg("[DEBUG] SPI_0 running in SLAVE mode (TRANSFMT.SLVMODE=1).\r\n"
        "        Waiting for Pi master to clock in frames on MOSI.\r\n");

    uint32_t iter = 0;
    while (1) {
        iter++;

        /* Heartbeat on EVERY iteration during diagnosis. Previously
         * this only fired every 64 iterations, which made it
         * impossible to tell whether the loop had advanced at all past
         * a blocking eff_spi_xfer. Once we know the SPI path is
         * behaving, we can throttle this back. */
        dbg("[DEBUG] spi0 loop iter=%lu\r\n", (unsigned long)iter);

        if (rc) {
            /* SPI init failed — don't hammer the broken driver, just
             * keep the heartbeat going so we know the CPU is alive. */
            continue;
        }

        /* Paranoia: if the xfer call is blocking, the line above
         * ("iter=N") will be the last thing minicom ever sees. That's
         * the diagnostic signal we're looking for. */
        dbg("[DEBUG] about to call spi0_read_frame\r\n");

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
