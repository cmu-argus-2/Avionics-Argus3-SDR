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
 *   - SPI_0's physical pads on this board sit in pin group PINMUX_2.
 *     (Corrected from earlier PINMUX_4 attempt — board routes SPI_0
 *      through pin group 2 on this revision.)
 *   - PINMUX_SPI = "SPI only" mode for that pin group.
 *
 * Link direction: the Raspberry Pi is the SPI MASTER and clocks IQ
 * frames *into* the Efficient board; the Efficient board is the
 * SLAVE receiver. No bus contention — only the Pi drives SCK/CS.
 */
#define SPI_DEV         SPI_0
#define SPI_PINMUX      PINMUX_2
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

/* vprintf-backed debug print. printf via stdio (routed to UART_3
 * by -DEFF_STDIO_PORT=3) appears to be synchronous — it blocks
 * until each byte has actually been transmitted, so consecutive
 * dbg() calls don't race the UART TX FIFO and we don't see
 * interleaved/garbled output. Empirically the first printf line
 * at boot prints cleanly while same-content eff_uart_puts calls
 * later do not. */
static void dbg(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
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

    /* Log the post-eff_spi_init register state so we can see what the
     * SDK left us with before we override it. */
    uint32_t fmt_before = atc->TRANSFMT;
    printf("[INIT] TRANSFMT before = 0x%08lx\r\n", (unsigned long)fmt_before);

    /* Write TRANSFMT explicitly with ALL fields — don't RMW. The
     * previous OR-in-SLVMODE approach didn't stick: readback showed
     * 0x0a (SLVMODE clear, DATALEN=0 i.e. 1-bit words). The cause is
     * unclear, but a clean full-word write should be unambiguous.
     *
     * Field plan for SPI mode 0, MSB first, 8-bit slave RX:
     *   bit 0  CPHA      = 0
     *   bit 1  CPOL      = 0
     *   bit 2  SLVMODE   = 1   ← we are the slave
     *   bit 3  LSB       = 0   (MSB first — matches the Pi)
     *   bit 4  MOSIBIT   = 0
     *   bit 7  DATAMERGE = 0   ← one received byte per DATA read
     *   bits 12:8 DATALEN= 7   ← 8-bit words (length = N+1)
     * Everything else 0.
     *
     * Encoded value: 0x00000704 */
    uint32_t fmt = (1u << 2)        /* SLVMODE */
                 | (7u << 8);       /* DATALEN = 7 → 8-bit data */
    atc->TRANSFMT = fmt;

    /* Read back and shout if the write didn't land. */
    uint32_t fmt_after = atc->TRANSFMT;
    printf("[INIT] TRANSFMT wrote 0x%08lx  read 0x%08lx %s\r\n",
           (unsigned long)fmt, (unsigned long)fmt_after,
           (fmt_after == fmt) ? "(OK)" : "(MISMATCH!)");

    /* Flush anything stale from the RX FIFO. RXFIFORST self-clears
     * after the hardware completes the reset; bounded wait so a stuck
     * bit can't hang us here. */
    atc->CTRL |= ATCSPI200_CTRL_RXFIFORST_MASK;
    for (int i = 0; i < 10000 &&
         (atc->CTRL & ATCSPI200_CTRL_RXFIFORST_MASK); i++) {
        /* spin */
    }

    /* Post-reset snapshot. STATUS should now show RXEMPTY=1 (bit 14,
     * mask 0x4000) since we just flushed the FIFO. If it doesn't,
     * either our STATUS layout is wrong or the FIFO reset didn't take. */
    uint32_t st_after = atc->STATUS;
    printf("[INIT] post-reset STATUS = 0x%08lx (RXEMPTY %s)\r\n",
           (unsigned long)st_after,
           (st_after & ATCSPI200_STATUS_RXEMPTY_MASK) ? "set" : "CLEAR");

    return 0;
}

/* Counters exposed to the outer loop for periodic summary prints.
 * We deliberately do NOT print anything from inside the hot poll loop —
 * every dbg() is a synchronous UART transmit and at high SPI rates we
 * don't have the time. */
static volatile uint32_t g_midframe_err = 0;
static volatile uint32_t g_bytes_seen   = 0;  /* total bytes drained from RX FIFO */

/* First-bytes buffer: capture the first 32 bytes the FIFO ever gives
 * us, so on the first heartbeat after any RX activity we can print
 * them and see what the hardware is actually delivering. This cuts
 * through the guesswork: if we see 32 consecutive 0x00s, the stream
 * is all zeros (or RXEMPTY is stuck and DATA returns 0); if we see
 * meaningful bytes (including 0x30 0x51 0x51 0x49 somewhere), the
 * Pi is actually sending frames. */
#define FIRST_BYTES_CAP 32u
static volatile uint32_t g_first_bytes_count = 0;
static uint8_t           g_first_bytes[FIRST_BYTES_CAP];

/* Rolling last-16 bytes. Overwritten as we drain — shows whether the
 * FIFO is giving us one repeated ghost value or actually new data. */
#define LAST_BYTES_CAP 16u
static volatile uint32_t g_last_bytes_idx = 0;
static uint8_t           g_last_bytes[LAST_BYTES_CAP];

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
/* Re-assert TRANSFMT every time we want to read. The hardware
 * empirically reverts TRANSFMT between transactions on this part
 * (SLVMODE seen to clear between init and the very next heartbeat;
 * TRANSFMT seen to reach 0x00 after a few hundred bytes are clocked).
 * Easier to just rewrite it than to chase the trigger. */
static void spi0_force_transfmt(void)
{
    ATCSPI200_RegDef *atc = (ATCSPI200_RegDef *)SPI_0->base_address;
    uint32_t fmt = (1u << 2)        /* SLVMODE */
                 | (7u << 8);       /* DATALEN = 7 → 8-bit data */
    atc->TRANSFMT = fmt;
}

static int spi0_read_frame(iq_spi_frame_t *frame)
{
    /* Force TRANSFMT every entry — hardware loses our SLVMODE setting. */
    spi0_force_transfmt();
    /* Separate the three budgets so each has clear meaning:
     *   SEARCH_MAX_BYTES — how many bytes we'll consume while
     *                      searching for magic before giving up and
     *                      letting the outer loop heartbeat. Bounded
     *                      so a stuck-RXEMPTY FIFO (delivers bytes
     *                      infinitely fast) can't lock us in here.
     *   SEARCH_POLL_BUDGET — per-call timeout on spi0_poll_byte while
     *                        searching. If no byte arrives in this
     *                        many polls, we bail out and heartbeat.
     *   INBAND_BUDGET — tight per-byte deadline once we're locked on
     *                   magic and reading the rest of the frame. */
    const uint32_t SEARCH_MAX_BYTES   =   10000u;
    const uint32_t SEARCH_POLL_BUDGET =  500000u;
    const uint32_t INBAND_BUDGET      =  200000u;

    ATCSPI200_RegDef *atc = (ATCSPI200_RegDef *)SPI_0->base_address;

    uint8_t w0 = 0, w1 = 0, w2 = 0, w3 = 0;
    uint32_t bytes_consumed = 0;

    memset(frame, 0, sizeof(*frame));

    while (bytes_consumed < SEARCH_MAX_BYTES) {
        uint8_t b;
        int rc = spi0_poll_byte(&b, SEARCH_POLL_BUDGET);
        if (rc == -1) {
            /* No byte in this window. Let outer loop heartbeat. */
            return 0;
        }
        bytes_consumed++;

        /* Capture the first few bytes ever drained so main()'s next
         * heartbeat can print them. Tells us what the FIFO is
         * actually giving us vs. what we think it should give us. */
        if (g_first_bytes_count < FIRST_BYTES_CAP) {
            g_first_bytes[g_first_bytes_count++] = b;
        }
        /* Also keep a rolling window of the last LAST_BYTES_CAP bytes,
         * so we can tell stuck-ghost-value from actual data. */
        g_last_bytes[g_last_bytes_idx % LAST_BYTES_CAP] = b;
        g_last_bytes_idx++;

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
    /* Burned SEARCH_MAX_BYTES without finding magic. Return and let
     * outer loop heartbeat (and print g_first_bytes so we can see
     * what we actually got). */
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

    /* Use printf (synchronous) instead of eff_uart_puts so this
     * line is guaranteed to flush before the next ones. */
    printf("\r\n=== spi0 smoke: UART_3 alive ===\r\n");

    /* Static storage: 2068-byte struct on main's stack is borderline
     * risky on this core and has masked earlier diagnostics. */
    static iq_spi_frame_t frame;
    int rc;

    dbg("[DEBUG] spi0 smoke start (direct-register slave RX) [build-v16 force-transfmt]\r\n");

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
        uint32_t st_snap = 0;
        uint32_t fmt_snap = 0;
        {
            ATCSPI200_RegDef *atc =
                (ATCSPI200_RegDef *)SPI_0->base_address;
            wcnt = (atc->SLVDATACNT & ATCSPI200_SLVDATACNT_WCNT_MASK)
                   >> ATCSPI200_SLVDATACNT_WCNT_OFFSET;
            st_snap  = atc->STATUS;
            fmt_snap = atc->TRANSFMT;
        }
        /* Single compact line per heartbeat. printf is synchronous
         * so no garbling. Anything noteworthy (real frame, error)
         * prints its own additional line elsewhere. */
        dbg("[H] it=%lu ok=%lu bad=%lu w=%lu b=%lu st=%08lx fmt=%08lx\r\n",
            (unsigned long)iter,
            (unsigned long)frames_ok,
            (unsigned long)frames_bad,
            (unsigned long)wcnt,
            (unsigned long)g_bytes_seen,
            (unsigned long)st_snap,
            (unsigned long)fmt_snap);

        /* Dump the first bytes we ever saw on the wire, once, as soon
         * as we have at least 8. Gives us a "what did the FIFO
         * actually give us" snapshot that's independent of magic
         * matching. */
        static volatile int first_bytes_dumped = 0;
        if (!first_bytes_dumped && g_first_bytes_count >= 8u) {
            first_bytes_dumped = 1;
            /* Use printf one byte at a time — synchronous, no race. */
            dbg("[DEBUG] first %lu bytes:",
                (unsigned long)g_first_bytes_count);
            for (uint32_t k = 0; k < g_first_bytes_count; k++) {
                dbg(" %02x", (unsigned)g_first_bytes[k]);
            }
            dbg("\r\n");
        }

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

        /* Rate-limit heartbeats so the UART TX FIFO doesn't overflow
         * and garble output. ~1 heartbeat/sec is plenty for human
         * monitoring; real frames will print their own header. The
         * exact spin count is core-speed dependent — tune if needed. */
        for (volatile uint32_t s = 0; s < 2000000u; s++) {
            /* spin */
        }
    }

    return 0;
}
