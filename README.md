# Avionics-Argus3-SDR

A Software-Defined Radio (SDR) GNSS receiver running on the **Efficient Computer Argus3** evaluation board. The long-term goal is a full GPS L1 C/A receiver pipeline — sample input, acquisition, tracking, and per-satellite measurement output for PVT (position / velocity / time) — implemented in fixed-point C, suitable for a CubeSat avionics stack.

## What this project is

There are two pieces of hardware:

1. **Raspberry Pi + RTL-SDR (host side).** The Pi runs `rpi_rtlsdr_spi_bridge.c`, which pulls IQ samples from the RTL-SDR (default: GPS L1 at 1575.42 MHz, 1.024 Msps) and clocks them out over `/dev/spidev0.0` as the SPI **master**. Each frame on the wire is `iq_spi_frame_t`: a 16-byte header (magic `IQQ0` / `0x49515130`, version, payload size, sequence, sample rate, center freq) followed by 2048 bytes (1024 complex U8 IQ samples).
2. **Efficient Argus3 board (target side).** The board runs `main_spi0_smoke.c` as the SPI **slave**, drains frames from MOSI, and (eventually) feeds them into the GNSS DSP pipeline.

Wiring (Pi -> Efficient):

```
Pi SPI0_SCLK (GPIO 11, pin 23)  -> Efficient SPI0 SCK   (Pi drives)
Pi SPI0_CE0  (GPIO 8,  pin 24)  -> Efficient SPI0 SS    (Pi drives)
Pi SPI0_MOSI (GPIO 10, pin 19)  -> Efficient SPI0 MOSI  (Pi sends IQ)
Pi SPI0_MISO (GPIO 9,  pin 21)  <- Efficient SPI0 MISO  (unused)
Common ground.
```

UART layout on the Efficient dev board:

```
/dev/ttyACM0 -> flashing (eff-flash)
/dev/ttyACM1 -> power monitor
/dev/ttyACM2 -> debug / monitoring (UART_3 on the chip; this is what minicom should attach to)
```

Pin groups: `PINMUX_3` is set to UART for debug; `PINMUX_2` is set to SPI for the IQ link (so SPI_0's pads are live and UART_2 is unused).

## Current stage

We are **not yet running the GNSS pipeline end-to-end**. The pipeline DSP code (`gnss_types.{c,h}`, `main.c`, `main_sdr.c`, `main_spi0_sdr.c`) exists from earlier iterations, but the active build target is the SPI smoke test:

```
CMakeLists.txt -> SOURCE main_spi0_smoke.c
```

`main_spi0_smoke.c` is the bring-up program that:

- Configures `SPI_0` as a slave by directly poking `ATCSPI200_TRANSFMT.SLVMODE = 1` (the SDK's `eff_spi_cfg_t` doesn't expose that bit).
- Disables `DATAMERGE` so each `DATA` read returns one received byte (in theory; see caveats below).
- Drains the RX FIFO by polling the raw ATCSPI200 registers, not via `eff_spi_xfer()` (which hangs forever in slave mode — it appears to poll a master-side completion flag that's never asserted when the Pi drives SCK).
- Slides a 4-byte window looking for the frame magic, then captures the rest of the frame with a tight per-byte deadline.
- Emits one compact heartbeat per outer iteration over UART_3: `[H] it=N ok=N bad=N w=N b=N st=XXXXXXXX fmt=XXXXXXXX` where `w` is `SLVDATACNT.WCNT` (10-bit master-write count, wraps every 1024 bytes), `b` is total bytes drained, `st` is `STATUS`, `fmt` is `TRANSFMT`.

What works:
- Boot + UART_3 debug path is reliable.
- SPI_0 slave init no longer hangs; the heartbeat loop runs steadily.
- `SLVDATACNT.WCNT` advances when the Pi clocks data, confirming bytes physically reach the slave through PINMUX_2.

What's still being chased (open issues, in priority order):

1. **Pi-side: `rpi_bridge` cannot open the RTL-SDR.** `rtlsdr_open` returns `-1` (`LIBUSB_ERROR_IO`) on most runs because the kernel keeps loading `rtl2832_sdr` and `dvb_usb_rtl28xxu`. The blacklist file under `/etc/modprobe.d/` needs to include both module names plus `dvb_usb_v2`, followed by `update-initramfs -u` and a reboot. `rtl_test` is misleading here: it succeeds even when `rpi_bridge` can't open the device, because `rtl_test` calls `libusb_detach_kernel_driver` to forcibly steal the dongle.
2. **Efficient-side: ATCSPI200 register layout is not fully understood on this part.** Empirically:
   - `TRANSFMT` only partially accepts writes. We write `0x00000704` (SLVMODE=1, DATALEN=7) and read back `0x00000025` — bits 8–12 (DATALEN) silently refuse to take, and the hardware also reverts SLVMODE between transactions (TRANSFMT seen to drift from `0x025` → `0x0a` → `0x00` over a few hundred bytes). Force-rewriting TRANSFMT on every frame attempt does not help.
   - `STATUS.RXEMPTY` (claimed bit 14 in the header) is never observed set, even directly after `CTRL.RXFIFORST`. Likely the bit position is different on this variant. Until that's resolved, the polling loop treats an empty FIFO as "not empty" and reads ghost zeros forever — bounded to 10000 bytes/iter so the heartbeat still flows.
   - Reading `INTRST` returns garbage (it's `__O` write-only in the SDK header). Do not read it.
3. **End-to-end frame capture.** A full `iq_spi_frame_t` arriving with correct magic (`30 51 51 49` on the wire), version (1), payload_bytes (2048). Blocked on (1) and (2) above.

UART output gotcha: `eff_uart_puts(UART_DEV, ...)` is **non-blocking** on this chip. Back-to-back calls overflow the TX FIFO and visibly interleave/garble output. `printf` via stdio (with `-DEFF_STDIO_PORT=3`) is synchronous and does not. **Use `printf`/`dbg()` (which wraps `vprintf`) for all debug output, not `eff_uart_puts` directly.**

Other artifacts in the folder:
- `power.csv`, `power_SDR.csv`, `report.txt`, `report_sdr.txt`, `sample_SDR.log`, `parsed_iq.txt`, `raw.iq` — captures from earlier offline experiments.
- `*.pdf` — Efficient Computer reference docs (GPIO, I2C, SPI, UART, Pinmux, Eval Kit getting started). Keep these handy when modifying low-level driver code.

## How to run

### On the Argus3 board (this repo)

`run_simple.sh` is the one-shot build + flash script. It assumes:

- `EFFCC_DIR=/home/argus/effcc` (the Efficient toolchain install).
- This repo is checked out at `<effcc-tree>/apps/Avionics-Argus3-SDR` (so `../../build` lands in the right place).
- `eff-flash` is on `/home/argus/effcc/bin/`.
- The board is plugged in and `/dev/ttyACM0` is the flashing port.

Run it:

```bash
./run_simple.sh
```

What it does:

```bash
export EFFCC_DIR="/home/argus/effcc/"
rm -rf ../../build
mkdir ../../build
cd ../../build
cmake -G Ninja .. -DEFF_STDIO_PORT=3      # route printf() to UART_3 == /dev/ttyACM2
ninja Avionics-Argus3-SDR
sudo /home/argus/effcc/bin/eff-flash apps/Avionics-Argus3-SDR/scalar/Avionics-Argus3-SDR.hex sram
cd apps/Avionics-Argus3-SDR
```

`-DEFF_STDIO_PORT=3` is important: with `=3`, `printf()` and the `dbg()` helpers land on UART_3 (i.e. `/dev/ttyACM2`); with anything else you'll see no output in minicom.

Note: the trailing `5` on the last line of `run_simple.sh` is junk and should be removed (it produces a "command not found" at the end of the run; harmless but noisy).

After flashing, attach to the monitoring UART:

```bash
minicom -D /dev/ttyACM2 -b 115200
```

You should see the boot banner `=== spi0 smoke: UART_3 alive ===` followed by per-iteration heartbeats.

### On the Raspberry Pi (host side, separate machine)

One-time setup: blacklist the kernel DVB modules so they don't steal the RTL-SDR dongle. Without this, `rpi_bridge` fails with `rtlsdr_open failed: -1` (`LIBUSB_ERROR_IO`) on most runs because the kernel's `dvb_usb_rtl28xxu` driver has claimed the USB interface.

```bash
sudo tee /etc/modprobe.d/blacklist-rtl.conf <<'EOF'
blacklist rtl2832_sdr
blacklist dvb_usb_rtl28xxu
blacklist dvb_usb_v2
blacklist rtl2832
blacklist rtl2830
EOF
sudo update-initramfs -u
sudo reboot
```

After reboot, verify the modules really are gone before running anything else:

```bash
lsmod | grep -iE 'rtl|dvb'   # expect EMPTY output
lsusb                          # should still show Realtek RTL2838
```

Build and run the bridge:

```bash
gcc -O2 -Wall -Wextra -o rpi_bridge rpi_rtlsdr_spi_bridge.c -lrtlsdr -lpthread
sudo ./rpi_bridge /dev/spidev0.0 1575420000 1024000 1000000 0
#                  ^spi dev      ^center Hz  ^fs Hz   ^SPI Hz  ^gain (0=auto)
```

SPI clock is currently held at 1 MHz while bringing up the slave RX path; once that's stable we'll push it back toward 20 MHz. Start the Pi bridge **after** the Argus3 board is flashed and printing heartbeats — the slave needs to be drained and listening before the master starts clocking, otherwise the first frame's magic gets eaten.

Quick diagnostic if `rpi_bridge` fails: `rtl_test -t` will succeed even when `rpi_bridge` cannot (it force-detaches the kernel driver). Don't take a passing `rtl_test` as proof the bridge will work — only `lsmod` showing zero rtl/dvb modules confirms the blacklist is in effect.

## TODOs

Near-term (unblock the data path):
- [ ] **Pi**: confirm post-reboot `lsmod | grep -iE 'rtl|dvb'` is empty and `rpi_bridge` reaches the per-frame send loop.
- [ ] **Efficient**: get the actual `STATUS` and `TRANSFMT` bit-field definitions out of the on-disk `atcspi200.h` (the values we've been using do not match the hardware's behavior — `RXEMPTY` is never observed set, `DATALEN` writes are silently dropped, SLVMODE reverts between transactions). Until this is grounded in the real header, the slave RX path is guesswork.
- [ ] Get a clean `iq_spi_frame_t` from Pi -> Efficient end-to-end (correct magic, version, payload). Blocked on the two items above.
- [ ] Once frames are stable, strip the one-shot probe instrumentation (`p_entry`, `p_status_1st`, …) from `spi0_poll_byte` and the bulky heartbeat dump — every UART TX costs us bytes at higher SPI clocks.
- [ ] Decide on overrun handling: today we rely on per-byte timeouts; if the FIFO genuinely overflows we want explicit detection (not via `INTRST` reads — that register is `__O` write-only on this IP and gave false positives).
- [ ] Fix the stray trailing `5` in `run_simple.sh`.

Pipeline integration:
- [ ] Wire the SPI RX path into the GNSS pipeline (`main_spi0_sdr.c` was the previous attempt; needs to be revisited once the SPI smoke is solid).
- [ ] Convert U8 IQ samples from the RTL-SDR into the `iq16_t` format the DSP code expects (subtract 127, scale).
- [ ] Bring up acquisition (PRN search over Doppler / code phase) on live samples.
- [ ] Bring up tracking loops (DLL + PLL/FLL) and emit `gnss_measurement_t` over UART.
- [ ] Pseudorange + Doppler -> PVT solver (likely off-board on the Pi to start).

Housekeeping:
- [ ] Move stale `main*.c` variants (`main.c`, `main_sdr.c`, `main_spi0_sdr.c`, `main_test.c`) into an `archive/` folder so it's obvious which file is the live target.
- [ ] Drop binary artifacts (`iq_convert.o`, `raw.iq`, big CSVs) from git or move them to a `data/` folder with a `.gitignore`.

## Pipeline reference (target architecture)

The intended GNSS DSP pipeline, once the SPI plumbing is finished:

```
Raw IQ samples (iq16_t)
    -> Mix down (carrier wipeoff to baseband)
    -> Correlation against PRN replica (Early / Prompt / Late)
    -> Metric computation (I^2 + Q^2)
    -> Tracking loops (DLL for code phase, PLL/FLL for carrier)
    -> track_state_t (code phase, doppler, lock status)
    -> Measurement extraction (gnss_measurement_t: pseudorange, doppler, C/N0)
    -> UART output for off-board PVT
```

Key types live in `gnss_types.h`:

| Type | Description |
|------|------------|
| `iq16_t` | 16-bit complex I/Q samples |
| `iq32_t` / `gnss_corr32_t` | 32-bit complex (correlator accumulators) |
| `acq_result_t` | Acquisition output (PRN, Doppler, code phase, metric) |
| `track_state_t` | Per-channel tracking loop state |
| `gnss_measurement_t` | Per-satellite measurement for PVT |

Design choices: fixed-point throughout (no FPU on the target), streaming pipeline (one sample in, one update out), all buffers statically sized.
