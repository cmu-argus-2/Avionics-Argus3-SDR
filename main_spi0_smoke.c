#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <stddef.h>
#include <string.h>

#include <eff.h>
#include <eff/drivers/spi.h>
#include <eff/drivers/uart.h>

/* Direct register access to the ATCSPI200 controller behind SPI_0.
 *
 * Two things we can't do through the SDK wrapper and therefore do
 * ourselves against the raw register block:
 *
 *   1. Put the controller into SLAVE mode. eff_spi_cfg_t exposes no
 *      master/slave field; eff_spi_init() leaves TRANSFMT.SLVMODE=0.
 *      We OR in SLVMODE=1 manually so SCK/CS become inputs driven by
 *      the Pi master.
 *
 *   2. Receive bytes. eff_spi_xfer() hangs forever in slave mode —
 *      empirically confirmed: the debug print immediately before the
 *      call fires, the one after never does. The implementation
 *      almost certainly polls a master-side completion flag
 *      (STATUS.SPIACTIVE or TX-done) that is never asserted when the
 *      Pi is the one driving SCK. Slave RX doesn't need any of that:
 *      the hardware autonomously pushes received bytes into the RX
 *      FIFO as SCK edges arrive, and we just drain them by polling
 *      STATUS.RXEMPTY + reading DATA. See spi0_poll_byte() and
 *      spi0_read_frame() below. */
#include <eff/atc/atcspi200.h>

/* ---------------- UART (debug output) ----------------
 *
 * Board port layout (per user):
 *   /dev/ttyACM0 -> flashing (eff-flash)
 *   /dev/ttyACM1 -> power monitor
 *   /dev/ttyACM2 -> monitoring (this is what minicom should attach to)
 *
 * /dev/ttyACM2 is physically wired to UART_3 on the Efficient chip.
 * Confirmed empirically: with -DEFF_STDIO_PORT=3 the printf() banner
 * at the top of main() appears in minicom; with =4 it does not.
 *
 * Per the UART + pinmux docs, any UART other than the boot UART must
 * be BOTH eff_uart_init()'d AND have its pin group switched to UART
 * mode via eff_pinmux_set(PINMUX_3, PINMUX_UART) before anything you
 * write can reach the pins. We do both below in uart_debug_init().
 */
#define UART_DEV        UART_3
#define UART_PINMUX     PINMUX_3

/* ---------------- SPI -----------------
 *
 * Pin group selection, per the pinmux PDF and board wiring:
 *   - SPI_0's physical pads on this board sit in pin group PINMUX_4.
 *     (Same group that would otherwise be UART_4, but we use UART_3
 *      for debug, so pin group 4 is free.)
 *   - PINMUX_SPI = "SPI only" mode for that pin group.
 *
 * Link direction: the Raspberry Pi is the SPI MASTER and clocks IQ
 * frames *into* the Efficient board; the Efficient board is the
 * SLAVE receiver. No bus contention — only the Pi drives SCK/CS.
 */
#define SPI_DEV         SPI_0
#define SPI_PINMUX      PINMUX_4
#define SPI_PINMUX_CFG  PINMUX_SPI

#define FRAME_MAGIC       0x49515130u
#define FRAME_VERSION     1u
#define FRAME_DATA_BYTES  2048u

/* Little-endian byte order of FRAME_MAGIC as it appears on the wire.
 * The Pi writes the struct byte-for-byte; on a little-endian ARM host
 * that means MAGIC is sent as 0x30, 0x51, 0x51, 0x49. */
#define MAGIC_B0 0x30u
#define MAGIC_B1 0x51u
#define MAGIC_B2 0x51u
#define MAGIC_B3 0x49u

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

/* ---------------- SPI: slave-mode direct-register driver ----------------
 *
 * What spi0_init() does beyond eff_spi_init():
 *
 *   1. Set TRANSFMT.SLVMODE=1 so SCK/CS become inputs.
 *   2. Clear TRANSFMT.DATAMERGE (default is 1, which packs 4 received
 *      bytes into each 32-bit DATA word). With DATAMERGE=0 each read
 *      of DATA returns exactly one received byte in the low 8 bits —
 *      no packing, no alignment bookkeeping, no tricky resync.
 *   3. Pulse CTRL.RXFIFORST to flush any junk left over from init.
 *   4. Clear INTRST (W1C) so a later RXFIFOORINT unambiguously means
 *      "new overrun, not stale state from init."
 *
 * spi0_poll_byte() implements the "wait for one byte" primitive by
 * polling STATUS.RXEMPTY, with RX-FIFO-overrun detection via
 * INTRST.RXFIFOORINT.
 *
 * spi0_read_frame() does:
 *   (a) Slide a 4-byte window over arriving bytes until it matches
 *       the wire byte pattern of FRAME_MAGIC (0x30 51 51 49).
 *   (b) Copy those four bytes into *frame and then pull the remaining
 *       sizeof(*frame)-4 bytes with a tight per-byte deadline.
 *
 * Budgets:
 *   SEARCH_BUDGET is loose — while we're waiting for the Pi to begin
 *   sending, we want one heartbeat every ~second or two. The exact
 *   timing doesn't matter, just that it doesn't go silent.
 *
 *   INBAND_BUDGET is tight — once we're locked onto magic, each
 *   subsequent byte should arrive within one master clock period
 *   (microseconds at 20 MHz). A stall means the Pi stopped or CS
 *   went idle mid-frame, and the rest of the frame is junk.
 */

static int spi0_init(void)
{
    eff_spi_cfg_t spi_cfg = EFF_SPI_DEFAULTS;
    spi_cfg.xfer_mode = SPI_XFER_READ_ONLY;
    spi_cfg.bus_size  = SPI_BUS_SINGLE;
    /* clk_div is irrelevant in slave mode — the Pi drives SCK. */

    if (eff_spi_init(SPI_DEV, &spi_cfg)) {
        return -1;
    }
    if (eff_pinmux_set(SPI_PINMUX, SPI_PINMUX_CFG)) {
        return -2;
    }

    ATCSPI200_RegDef *atc = (ATCSPI200_RegDef *)SPI_0->base_address;

    /* (1) + (2): flip to slave mode AND disable DATAMERGE in a single
     * RMW so the hardware never sees a transient "slave + merged"
     * state (which the IP may or may not support). */
    uint32_t fmt = atc->TRANSFMT;
    fmt |=  ATCSPI200_TRANSFMT_SLVMODE_MASK;
    fmt &= ~ATCSPI200_TRANSFMT_DATAMERGE_MASK;
    atc->TRANSFMT = fmt;

    /* (3) flush anything stale from the RX FIFO. RXFIFORST self-clears
     * after the hardware completes the reset; bounded wait so a stuck
     * bit can't hang us here. */
    atc->CTRL |= ATCSPI200_CTRL_RXFIFORST_MASK;
    for (int i = 0; i < 10000 &&
         (atc->CTRL & ATCSPI200_CTRL_RXFIFORST_MASK); i++) {
        /* spin */
    }

    /* (4) clear all W1C interrupt-status bits. Writing the current
     * value back clears exactly whichever bits were set. */
    atc->INTRST = atc->INTRST;

    return 0;
}

/* Counters exposed to the outer loop for periodic summary prints.
 * We deliberately do NOT print anything from inside the hot poll loop —
 * every dbg() is a synchronous UART transmit and at high SPI rates we
 * don't have the time. */
static volatile uint32_t g_midframe_err = 0;
static volatile uint32_t g_bytes_seen   = 0;  /* total bytes drained from RX FIFO */

/* Poll the RX FIFO for one byte.
 *
 * IMPORTANT: we do NOT read INTRST here. INTRST is declared `__O`
 * (write-only) in the SDK header, and reads of write-only registers
 * return undefined values on this IP — which caused the false-positive
 * overrun storm in earlier builds (`ovr = 5M/heartbeat` with `ok = 0`).
 * Overrun detection is handled implicitly instead: if we miss bytes
 * mid-frame, the per-byte INBAND_BUDGET will time out and the caller
 * will abort + resync. During magic search, dropped bytes are harmless
 * because we're scanning a stream for a 4-byte pattern that will
 * appear again on the next frame.
 *
 * Returns:
 *    0  - got a byte, written to *b
 *   -1  - timeout: no byte arrived within `budget` polls */
static int spi0_poll_byte(uint8_t *b, uint32_t budget)
{
    ATCSPI200_RegDef *atc = (ATCSPI200_RegDef *)SPI_0->base_address;

    /* ONE-SHOT probes. A pack of static flags, each pinning down a
     * specific place we might hang. Each fires at most once across
     * all calls to this function, so they're cheap — and they give
     * us a precise minicom trace of how far we got on the very first
     * byte attempt. */
    static volatile int p_entry      = 0;  /* function entered */
    static volatile int p_status_1st = 0;  /* first STATUS read returned */
    static volatile int p_notempty   = 0;  /* RXEMPTY cleared → about to read DATA */
    static volatile int p_data_1st   = 0;  /* first DATA read returned */
    static volatile int p_ret_byte   = 0;  /* returning a byte to caller */
    static volatile int p_loop_i1    = 0;  /* entered loop body past i=0 */
    static volatile int p_loop_i1k   = 0;  /* reached i=1000 with FIFO empty */

    if (!p_entry) {
        p_entry = 1;
        dbg("[DEBUG] poll: entry, STATUS @ %p DATA @ %p\r\n",
            (void *)&atc->STATUS, (void *)&atc->DATA);
    }

    for (uint32_t i = 0; i < budget; i++) {
        if (i == 1u && !p_loop_i1) {
            p_loop_i1 = 1;
            dbg("[DEBUG] poll: reached i=1 (loop body executes)\r\n");
        }
        if (i == 1000u && !p_loop_i1k) {
            p_loop_i1k = 1;
            dbg("[DEBUG] poll: reached i=1000 (FIFO empty so far)\r\n");
        }
        /* Diagnostic tap: every 100K polls, emit a progress mark so
         * we can tell from minicom whether this loop is actually
         * running or whether the STATUS read hung us. */
        if ((i % 100000u) == 0u && i > 0u) {
            dbg("[DEBUG] poll i=%lu\r\n", (unsigned long)i);
        }
        uint32_t st = atc->STATUS;
        if (!p_status_1st) {
            p_status_1st = 1;
            dbg("[DEBUG] poll: first STATUS = 0x%08lx\r\n",
                (unsigned long)st);
        }
        if (!(st & ATCSPI200_STATUS_RXEMPTY_MASK)) {
            if (!p_notempty) {
                p_notempty = 1;
                dbg("[DEBUG] poll: RXEMPTY=0, about to read DATA\r\n");
            }
            /* DATAMERGE=0, so low byte of DATA is the next received
             * byte on MOSI. Upper bytes are undefined; ignore them. */
            uint32_t dv = atc->DATA;
            if (!p_data_1st) {
                p_data_1st = 1;
                dbg("[DEBUG] poll: first DATA = 0x%08lx\r\n",
                    (unsigned long)dv);
            }
            *b = (uint8_t)(dv & 0xFFu);
            g_bytes_seen++;
            if (!p_ret_byte) {
                p_ret_byte = 1;
                dbg("[DEBUG] poll: returning first byte = 0x%02x\r\n",
                    (unsigned)*b);
            }
            return 0;
        }
    }
    return -1;
}

/* Read one IQ frame from the SPI bus. Returns:
 *    1  - full frame received and copied into *frame
 *    0  - no frame yet (poll budget exhausted before finding magic).
 *         Caller should loop and heartbeat.
 *   -1  - bus error (overrun or stall mid-frame). Caller should
 *         resync on the next call.
 *
 * Note: this function is SILENT. No dbg() calls, no prints. Every
 * UART TX in here used to push us over the byte-time budget at 20 MHz
 * SPI and caused the exact overruns we were trying to diagnose. */
static int spi0_read_frame(iq_spi_frame_t *frame)
{
    /* Much smaller than before so heartbeats fire frequently even
     * when the Pi is silent — we'd rather get 10 heartbeats/sec that
     * say "wcnt=X, bytes=Y" than wait seconds between them. */
    const uint32_t SEARCH_BUDGET =   500000u;
    const uint32_t INBAND_BUDGET =   200000u; /* tight mid-frame deadline */

    ATCSPI200_RegDef *atc = (ATCSPI200_RegDef *)SPI_0->base_address;

    uint8_t w0 = 0, w1 = 0, w2 = 0, w3 = 0;
    uint32_t remaining = SEARCH_BUDGET;

    memset(frame, 0, sizeof(*frame));

    while (remaining > 0) {
        uint8_t b;
        int rc = spi0_poll_byte(&b, remaining);
        if (rc == -1) {
            /* No byte in this window. Let outer loop heartbeat. */
            return 0;
        }
        if (remaining > 1) remaining--;

        w0 = w1; w1 = w2; w2 = w3; w3 = b;

        if (w0 == MAGIC_B0 && w1 == MAGIC_B1 &&
            w2 == MAGIC_B2 && w3 == MAGIC_B3) {
            uint8_t *dst = (uint8_t *)frame;
            dst[0] = MAGIC_B0;
            dst[1] = MAGIC_B1;
            dst[2] = MAGIC_B2;
            dst[3] = MAGIC_B3;

            for (size_t n = 4; n < sizeof(*frame); n++) {
                /* Tight per-byte deadline: once locked on magic, every
                 * subsequent byte should arrive within one master clock
                 * period. A timeout here means the Pi stopped or we
                 * got desynced. */
                rc = spi0_poll_byte(&dst[n], INBAND_BUDGET);
                if (rc) {
                    g_midframe_err++;
                    atc->CTRL |= ATCSPI200_CTRL_RXFIFORST_MASK;
                    return -1;
                }
            }
            return 1;
        }
    }
    return 0;
}

/* ---------------- main ---------------- */

int main(void)
{
    /* printf() before any explicit init. With -DEFF_STDIO_PORT=3 the
     * stdio runtime routes this to UART_3, which is physically wired
     * to /dev/ttyACM2 (monitoring). First thing the user sees in
     * minicom — confirms the board booted and reached main(). */
    printf("\r\n[DEBUG] spi0 smoke: entering main (stdio path)\r\n");

    /* Bring up UART_3 before anything else could try to talk. */
    if (uart_debug_init()) {
        /* Nothing we can do — no other output path. Fall through. */
    }

    eff_uart_puts(UART_DEV, "\r\n=== spi0 smoke: UART_3 alive ===\r\n");

    /* Static storage: 2068-byte struct on main's stack is borderline
     * risky on this core and has masked earlier diagnostics. */
    static iq_spi_frame_t frame;
    int rc;

    dbg("[DEBUG] spi0 smoke start (direct-register slave RX) [build-v8 probe-data-read]\r\n");

    rc = spi0_init();
    if (rc) {
        dbg("[DEBUG] spi0 init failed rc=%d\r\n", rc);
    } else {
        dbg("[DEBUG] spi0 init ok: SLVMODE=1, DATAMERGE=0, RX FIFO reset\r\n");
        dbg("[DEBUG] waiting for Pi master to clock frames onto MOSI\r\n");
    }

    uint32_t iter = 0;
    uint32_t frames_ok = 0;
    uint32_t frames_bad = 0;

    while (1) {
        iter++;

        /* One heartbeat per outer loop. `wcnt` is a hardware counter
         * of bytes the master has clocked to us, masked to the 10-bit
         * field in SLVDATACNT (wraps every 1024 bytes). If wcnt is
         * advancing between heartbeats, bytes ARE reaching the slave
         * even if we can't drain them fast enough. If wcnt stays
         * constant, nothing is on the wire. */
        uint32_t wcnt = 0;
        {
            ATCSPI200_RegDef *atc =
                (ATCSPI200_RegDef *)SPI_0->base_address;
            wcnt = (atc->SLVDATACNT & ATCSPI200_SLVDATACNT_WCNT_MASK)
                   >> ATCSPI200_SLVDATACNT_WCNT_OFFSET;
        }
        dbg("[DEBUG] iter=%lu ok=%lu bad=%lu mid=%lu wcnt=%lu bytes=%lu\r\n",
            (unsigned long)iter,
            (unsigned long)frames_ok,
            (unsigned long)frames_bad,
            (unsigned long)g_midframe_err,
            (unsigned long)wcnt,
            (unsigned long)g_bytes_seen);

        if (rc) {
            /* SPI init failed — keep heartbeat, don't touch driver. */
            continue;
        }

        /* One-shot probe: did we actually reach the call site? */
        static volatile int entered_read_frame = 0;
        if (!entered_read_frame) {
            entered_read_frame = 1;
            dbg("[DEBUG] main: about to call spi0_read_frame\r\n");
        }
        int r = spi0_read_frame(&frame);
        static volatile int exited_read_frame = 0;
        if (!exited_read_frame) {
            exited_read_frame = 1;
            dbg("[DEBUG] main: spi0_read_frame returned r=%d\r\n", r);
        }
        if (r == 0) {
            /* Outer heartbeat already prints counters; don't spam
             * additional lines here. */
            continue;
        }
        if (r < 0) {
            frames_bad++;
            continue;
        }

        frames_ok++;
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
