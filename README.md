# Avionics-Argus3-SDR

A Software-Defined Radio (SDR) GNSS receiver running on the **Efficient Computer Argus3** evaluation board. The long-term goal is a full GPS L1 C/A receiver pipeline — sample input, acquisition, tracking, and per-satellite measurement output for PVT (position / velocity / time) — implemented in fixed-point C, suitable for a CubeSat avionics stack.

## What this project is

There are two pieces of hardware:

1. **Raspberry Pi + RTL-SDR (host side).** The Pi runs `rpi_rtlsdr_spi_bridge.c`, which pulls IQ samples from the RTL-SDR (default: GPS L1 at 1575.42 MHz, 1.024 Msps) and clocks them out over `/dev/spidev0.0` as the SPI **master**. Each frame on the wire is `iq_spi_frame_t`: a 16-byte header (magic `IQQ0` / `0x49515130`, version, payload size, sequence, sample rate, center freq) followed by 2048 bytes (1024 complex U8 IQ samples).
2. **Efficient Argus3 board (target side).** The board runs `main_spi0_smoke.c` as the SPI **slave**, drains frames from MOSI, and (eventually) feeds them into the GNSS DSP pipeline.

Wiring (Pi -> Efficient):

```
Pi SPI0_SCLK (GPIO 11, pin 23)  -> Efficient SPI0 SCK   (Pi drives)
Pi SPI0_CE0  (GPIO 8,  pin 24)  -> Efficient SPI0 CS    (Pi drives)
Pi SPI0_MOSI (GPIO 10, pin 19)  -> Efficient SPI0 MOSI  (Pi sends IQ)
Pi SPI0_MISO (GPIO 9,  pin 21)  <- Efficient SPI0 MISO  (unused)
Common ground.
```

UART layout on the Argus3 dev board:

```
/dev/ttyACM0 -> flashing (eff-flash)
/dev/ttyACM1 -> power monitor
/dev/ttyACM2 -> debug / monitoring (UART_3 on the chip; this is what minicom should attach to)
```

Pin groups: `PINMUX_3` is set to UART for debug; `PINMUX_4` is set to SPI for the IQ link (so SPI_0's pads are live and UART_4 is unused).

## Current stage

We are **not yet running the GNSS pipeline end-to-end**. The pipeline DSP code (`gnss_types.{c,h}`, `main.c`, `main_sdr.c`, `main_spi0_sdr.c`) exists from earlier iterations, but the active build target is the SPI smoke test:

```
CMakeLists.txt -> SOURCE main_spi0_smoke.c
```

`main_spi0_smoke.c` is the bring-up program that:

- Configures `SPI_0` as a slave by directly poking `ATCSPI200_TRANSFMT.SLVMODE = 1` (the SDK's `eff_spi_cfg_t` doesn't expose that bit).
- Disables `DATAMERGE` so each `DATA` read returns one received byte.
- Drains the RX FIFO by polling `STATUS.RXEMPTY` (the SDK's `eff_spi_xfer()` hangs forever in slave mode).
- Slides a 4-byte window looking for the frame magic, then captures the rest of the frame with a tight per-byte deadline.
- Emits per-iteration heartbeats over UART_3 with counters (`ok / bad / mid-frame errors / bytes drained / SLVDATACNT.WCNT / STATUS / TRANSFMT`) plus a rolling last-16-bytes window so we can tell stuck-ghost-value vs. real traffic.

What works:
- Boot + UART_3 debug path is reliable.
- SPI_0 slave init no longer hangs; the heartbeat loop runs steadily.
- `SLVDATACNT.WCNT` advances when the Pi clocks data, confirming bytes physically reach the slave.

What's still being chased:
- End-to-end frame capture (a full `iq_spi_frame_t` arriving with correct magic/version/payload). The project is in the "stare at minicom output and chase the SPI slave RX path" phase.

Other artifacts in the folder:
- `Argus3_SDR_EndOfSemester.pptx` + `Argus3_SpeakerNotes.md` — end-of-semester deliverables.
- `tone_down_pptx.py`, `trim_words_add_notes.py`, `inspect_deck.mjs` — one-off scripts used while preparing the deck.
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

Build and run the bridge:

```bash
gcc -O2 -Wall -Wextra -o rpi_bridge rpi_rtlsdr_spi_bridge.c -lrtlsdr -lpthread
sudo ./rpi_bridge /dev/spidev0.0 1575420000 1024000 20000000 0
#                  ^spi dev      ^center Hz  ^fs Hz   ^SPI Hz   ^gain (0=auto)
```

Start the Pi bridge **after** the Argus3 board is flashed and printing heartbeats — the slave needs to be drained and listening before the master starts clocking, otherwise the first frame's magic gets eaten.

## TODOs

Near-term (unblock the data path):
- [ ] Get a clean `iq_spi_frame_t` from Pi -> Efficient end-to-end (correct magic, version, payload). Currently chasing this in `main_spi0_smoke.c`.
- [ ] Once frames are stable, strip the verbose `[DEBUG]` instrumentation from the hot path — every UART TX inside `spi0_poll_byte` costs us bytes at 20 MHz SPI.
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
