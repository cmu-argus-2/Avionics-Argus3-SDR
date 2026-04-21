#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>

#include <eff.h>
#include <eff/drivers/spi.h>
#include <eff/drivers/uart.h>

/* ---------------- UART (debug output) ----------------
 *
 * IMPORTANT: per the Efficient docs, only UART_0 (the boot UART) is
 * usable without configuration. Every other UART must be both
 * eff_uart_init()'d AND have its pinmux set to PINMUX_UART before
 * anything you write to it can reach the wire.
 *
 * Your run_simple.sh builds with -DEFF_STDIO_PORT=3, which routes
 * printf() to UART_3, but nothing in the code ever inits UART_3 or
 * pinmuxes PINMUX_3. That is why you see absolutely nothing in minicom.
 * main.c only appears to work because it pushes its visible output via
 * eff_uart_puts(UART_4, ...) after explicitly configuring UART_4, which
 * is what /dev/ttyACM2 is actually wired to on your setup.
 *
 * We mirror that exact known-good path here and use eff_uart_puts()
 * directly instead of relying on stdio at all.
 */
#define UART_DEV        UART_4
#define UART_PINMUX     PINMUX_4

/* ---------------- SPI -----------------
 *
 * WARNING: the Efficient SDK SPI driver (eff_spi_xfer) is MASTER ONLY.
 * There is no documented slave-mode configuration; the driver generates
 * SCK/CS and clocks tx_data out / rx_data in itself. Your Pi bridge
 * (/dev/spidev0.0) is also configured as master, so once this board
 * runs, both sides will be driving SCK/CS → bus contention, and no
 * valid frames will be received.
 *
 * We keep the SPI code here so you can at least see the smoke test
 * start printing now, but the architecture needs to change before any
 * frame will actually arrive: either flip roles (Pi becomes slave,
 * Efficient master — Linux spidev supports slave mode but it's
 * awkward), use a different link (UART/I2C/DMA-fed GPIO), or check the
 * actual eff/drivers/spi.h header on your dev machine for any
 * undocumented slave config fields.
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

    /* Pinmux first so pins are in SPI mode before the controller
     * samples / drives them. */
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
    /* Bring up UART_4 FIRST, before anything else could try to talk. */
    if (uart_debug_init()) {
        /* Nothing we can do if this fails — there's no other output
         * path. Fall through silently so the CPU doesn't hang here. */
    }

    /* Immediate sanity print so we can see the board is alive before
     * any peripheral configuration touches pins. */
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
