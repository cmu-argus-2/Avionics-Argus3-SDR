# 📡 SDR GNSS Receiver on Efficient Chip

## Overview
This project implements a **Software-Defined Radio (SDR) GNSS receiver** on the Efficient Chip platform.

The system processes **digitized GNSS RF samples** and outputs **per-satellite measurements** required to determine the CubeSat’s:

- Position  
- Velocity  
- Time (PVT)

The design focuses on:
- Fixed-point DSP (no floating point)
- Streaming pipeline architecture
- Embedded efficiency and scalability

---

## 🧠 System Pipeline
Raw Samples (iq16_t)
|
Mix Down (gnss_sat16)
|
Correlation
|
Metric Computation (gnss_mag2_iq32)
|
Tracking Loop (gnss_iabs32)
|
Tracking State (track_state_t)
|
Measurement Extraction (gnss_measurement_t)
|
UART Output

---

## 🔧 Processing Stages

### 1. Raw Sample Input
- **Type:** `iq16_t`
- Complex I/Q samples from ADC
- Represents digitized GNSS RF signal

---

### 2. Mix Down
- **Output Type:** `gnss_sat16`
- Performs carrier wipeoff using local oscillator
- Shifts signal to baseband

---

### 3. Correlation
- Matches signal with satellite PRN codes
- Generates correlation peaks
- Supports Early / Prompt / Late tracking

---

### 4. Metric Computation
- **Type:** `gnss_mag2_iq32`
- Computes signal power: I^2+Q^2
- Used for acquisition and signal strength estimation

---

### 5. Tracking Loop
- **Type:** `gnss_iabs32`
- Maintains lock on:
- Code phase (DLL)
- Carrier frequency (PLL / FLL)

---

### 6. Tracking State
- **Struct:** `track_state_t`
- Contains:
- Code phase
- Doppler frequency
- Lock status
- Loop filter state

---

### 7. Measurement Extraction
- **Struct:** `gnss_measurement_t`
- Outputs:
- Pseudorange
- Doppler (velocity)
- Signal strength

---

### 8. UART Output
- Sends measurements to external processor
- Used for:
- Navigation (PVT computation)
- Debugging / telemetry

---

## 📦 Data Types

| Type | Description |
|------|------------|
| `iq16_t` | 16-bit complex I/Q samples |
| `gnss_sat16` | Mixed-down signal |
| `gnss_mag2_iq32` | Magnitude squared (power) |
| `gnss_iabs32` | Absolute value representation |
| `track_state_t` | Tracking loop state |
| `gnss_measurement_t` | Final satellite measurement |

---

## ⚙️ Design Considerations

### Fixed-Point Arithmetic
- Eliminates floating-point overhead
- Improves performance on embedded hardware

### Streaming Architecture
- Continuous data flow
- Low latency processing

### Modular Design
- Each stage is independently testable
- Enables future FPGA / ASIC mapping

---

## 🚀 Output

The system generates **per-satellite measurements**:

- Code phase → pseudorange  
- Doppler → velocity  
- Timing → synchronization  

These measurements are used to compute:
Position (x, y, z)
Velocity (vx, vy, vz)
Time (clock bias)
