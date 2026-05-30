# HB100 Phased Array Radar System

A 4-element phased-array Doppler radar system using HB100 microwave modules (10.525 GHz) with ESP32-S3 firmware data collection and Raspberry Pi 4 host processor. Estimates target speed, angle of arrival, and performs multi-target tracking using an Extended Kalman Filter.

**See [HB100.md](../dchen/client/public/markdown/HB100.md) for detailed technical documentation and [SETUP_GUIDE.md](SETUP_GUIDE.md) for step-by-step setup instructions.**

## System Overview

```
4 × HB100 Modules (10.525 GHz Doppler)
    ↓
Analog Signal Conditioning (MCP6002 op-amp, 1092× gain)
    ↓
ESP32-S3 (12-bit ADC @ 10 kHz/channel via DMA)
    ↓
USB-CDC Serial → Raspberry Pi 4
    ↓
Processing Pipelines:
  • Radar Processor: MUSIC algorithm for angle-of-arrival + EKF tracking
  • Zone Classifier: Amplitude-based monopulse, robust to LO drift
```

## Key Features

- **4-Channel Receiver Array**: Independent HB100 Doppler modules with free-running Gunn oscillators
- **Dual Processing Pipelines**:
  - **Radar Processor** (`radar_processor.py`): Phase-coherent angle estimation via MUSIC pseudospectrum + Extended Kalman Filter state tracking
  - **Zone Classifier** (`zone_classifier.py`): Amplitude-based monopulse DOA, 5-zone classification (FAR_LEFT → CENTER → FAR_RIGHT), robust to oscillator drift
- **Analog Front-End**: 1092× total gain (52× + 21× stages) with virtual ground biasing to handle bipolar IF signals on unipolar ADC supply
- **Multi-Stage Filtering**: EMI rejection, anti-aliasing, power supply decoupling with optimized RC constants
- **Extended Kalman Filter**: Continuous 2D position and velocity tracking with adaptive observation noise
- **12-bit ADC Sampling**: 10 kHz per channel, 1024-sample blocks (~102.4 ms coherency windows)

## Quick Start

### Hardware Setup

1. **4-element HB100 array** with element spacing: 37 mm, 38 mm, 37 mm (center-to-center)
2. **Analog conditioning**: MCP6002 dual op-amp providing 52× + 21× gain stages with virtual ground at 1.65 V
3. **ESP32-S3-DevKitC-1**: Connected to op-amp outputs via GPIO 4, 5, 6, 7 (ADC channels)
4. **Raspberry Pi 4 Model B** (4 GB): Connected to ESP32-S3 via USB for host processing

### Build and Flash

Requires **ESP-IDF v5.1+**.

```bash
# Set target and build (desktop/Pi with ESP-IDF installed)
cd path/to/HB100
idf.py set-target esp32s3
idf.py build

# Flash to ESP32-S3 (hold BOOT, press RESET, release BOOT to enter download mode)
idf.py -p /dev/ttyACM0 flash
```

See [SETUP_GUIDE.md](SETUP_GUIDE.md) **Part 3** for detailed build instructions.

### Run Host Processor

On the Raspberry Pi 4 (or any Linux machine):

```bash
# Install dependencies
pip install numpy scipy pyserial

# Calibrate noise floor first (no moving targets, ~10 seconds)
python radar_processor.py --port /dev/ttyACM0 --calibrate --cal-blocks 100

# Run with full diagnostics (verbose mode)
python radar_processor.py --port /dev/ttyACM0 --threshold <VALUE> --verbose

# Run in compact mode with CSV logging
python radar_processor.py --port /dev/ttyACM0 --threshold <VALUE> --log measurements.csv
```

Replace `<VALUE>` with the "moderate" threshold from calibration output.

See [SETUP_GUIDE.md](SETUP_GUIDE.md) **Parts 5–9** for detailed host setup and first operation.

## Repository Structure

```
HB100/
├── main/
│   ├── main.c                    # ESP32-S3 firmware: ADC DMA, USB-CDC output
│   └── CMakeLists.txt            # Component build config
├── CMakeLists.txt                # Top-level ESP-IDF build
├── sdkconfig.defaults            # ESP32-S3 target configuration
├── radar_processor.py            # Host: MUSIC + Kalman filter tracking
├── zone_classifier.py            # Host: Amplitude-based zone classification
├── requirements.txt              # Python dependencies (numpy, scipy, pyserial)
├── README.md                     # This file
└── SETUP_GUIDE.md               # Comprehensive setup and tuning guide
```

## Signal Processing Pipelines

### Radar Processor (`radar_processor.py`)

Full-featured pipeline with phase-coherent angle-of-arrival estimation:

1. **USB Serial Reception**: Scans for 0xDEADBEEF sync word, reads 1024 × 4-channel × uint16 samples
2. **DC Offset Removal**: Per-channel mean subtraction to remove oscillator drift
3. **Bandpass Filtering**: 4th-order Butterworth, 10–800 Hz (SOS form for stability)
4. **FFT & Peak Detection**: Hanning windowed 1024-point FFT, identify Doppler peak bin
5. **Phase Extraction**: Complex FFT values at peak bin for each channel
6. **MUSIC (MUltiple SIgnal Classification)**: 
   - Correlation matrix from FFT bin range ±5 around peak
   - Forward-backward averaging for real-valued symmetry
   - Eigendecomposition → noise subspace → pseudospectrum peak = DOA
7. **Speed Estimation**: Doppler frequency converted to m/s (accounting for 2× radar round-trip)
8. **Extended Kalman Filter**: State [x_m, y_m, v_x, v_y]; measurement: [range, angle]; adaptive noise covariance

**Output**: Real-time display of Doppler, speed, angle, SNR, EKF state; CSV logging of all detections.

### Zone Classifier (`zone_classifier.py`)

Simplified, LO-drift-robust alternative using only amplitude (magnitude):

1. **Same front-end**: USB reception, DC removal, bandpass, FFT, peak detection
2. **Amplitude-based monopulse**:
   - Per-channel magnitude gains (corrected for hardware imbalance)
   - Left/right bias = (right_energy − left_energy) / (right_energy + left_energy)
   - Normalized centroid bias = (weighted_index − 1.5) / 1.5
3. **Zone Threshold Mapping**:
   - FAR_LEFT: bias ≤ −0.50
   - LEFT: −0.50 < bias ≤ −0.15
   - CENTER: −0.15 < bias ≤ +0.15
   - RIGHT: +0.15 < bias ≤ +0.50
   - FAR_RIGHT: bias > +0.50
4. **Speed Tracking**: 1-D alpha-beta filter on Doppler speed (α=0.35, β=0.10)

**Output**: Per-detection CSV: timestamp, zone, speed_mps, bias, snr_db, per-channel magnitudes.

## Hardware Design

### Analog Signal Conditioning

The HB100 IF output (10 µV–2 mV at ~200 Hz nominal Doppler) is amplified and referenced to a virtual ground at 1.65 V (half the 3.3 V ADC rail):

- **Stage 1 (Virtual Ground)**: 10 kΩ + 10 kΩ resistor divider → 1.65 V midpoint reference
- **Pre-Op-Amp EMI Filter**: 10 µF || 10 kΩ to virtual ground (1.6 Hz cutoff, rejects 50/60 Hz powerline hum)
- **Stage 2 (Non-Inverting, 52×)**: 1 kΩ input, 51 kΩ feedback; AC-coupled input
- **Stage 3 (Inverting, 21×)**: 1 kΩ input, 21 kΩ feedback
- **Total Gain**: 52 × 21 = **1092×** (60.8 dB)
- **Final Anti-Aliasing Filter**: 2 kΩ + 100 nF to real GND (~800 Hz cutoff)

**Signal Mapping**: 2 mV @ ADC input → 2.184 V amplified → maps to 0.56–2.74 V range (ADC codes ~683–5620), utilizing ~66% of full ADC range safely.

**Power Supply Filtering**: 3× 10 Ω in series with capacitor bank (100 µF + 10 µF + 0.1 µF) for broadband noise rejection (10 Hz–10 kHz).

### ESD Protection

Simple but effective EPA (Electrostatic Discharge Protected Area) using aluminum foil:
- Large aluminum foil sheet connected to circuit GND
- Ground body on foil for 3+ seconds before handling op-amps
- Maintain forearm contact during all component manipulation
- Drains accumulated static charge (5–15 kV potential) → near-zero volts

See [HB100.md](../dchen/client/public/markdown/HB100.md) **Schema** section for detailed circuit explanation and ESD setup procedures.

## Physical Array Layout

4-element **Uniform Linear Array (ULA)**:

| Element | Position | GPIO | ADC Channel | Spacing to Next |
|---------|----------|------|-------------|-----------------|
| #0 (left) | 0.000 m | GPIO 7 | ADC1_CH6 | 37 mm |
| #1 | 0.037 m | GPIO 6 | ADC1_CH5 | 38 mm |
| #2 | 0.075 m | GPIO 5 | ADC1_CH4 | 37 mm |
| #3 (right) | 0.112 m | GPIO 4 | ADC1_CH3 | — |

**Wavelength & DOA Resolution**:
- Carrier: f_c = 10.525 GHz → λ ≈ 28.5 mm
- Max baseline: d_max = 112 mm ≈ 3.93λ
- MUSIC pseudospectrum computed over ±80° range

See [HB100.md](../dchen/client/public/markdown/HB100.md) **Field-Deployable Antenna Array Layout** for detailed array geometry.

## Calibration and Tuning

### Calibration (Required First)

Run with no moving targets in a quiet RF environment:

```bash
python radar_processor.py --port /dev/ttyACM0 --calibrate --cal-blocks 100
```

Output provides recommended thresholds for **sensitive**, **moderate**, and **conservative** detection levels. Start with "moderate" threshold.

### Tuning

**Too many false detections?**
Increase threshold or re-calibrate in a quieter environment.

**Missing detections at range?**
Decrease threshold toward "sensitive" value. Monitor false positive rate.

**Kalman filter heading oscillates?**
Increase `KF_ANGLE_NOISE` in `radar_processor.py` (e.g., `np.radians(10)` or `np.radians(15)`).

**Kalman filter speed lags?**
Increase `KF_PROCESS_NOISE` (e.g., from 0.5 to 1.0 or 2.0).

**Channels show different Doppler frequencies?**
Check power supply filtering and HB100 ground plane connections (use short, thick wires to ESP32-S3 GND).

**MUSIC angle stuck or wrong?**
Verify `ELEMENT_POSITIONS` array in code matches real element spacing (measure center-to-center distances again).

See [SETUP_GUIDE.md](SETUP_GUIDE.md) **Part 10** for detailed tuning procedures.

## Deployment

### Run on Boot (Optional)

Create systemd service on Raspberry Pi:

```bash
sudo nano /etc/systemd/system/radar.service
```

```ini
[Unit]
Description=HB100 Phased Array Radar Processor
After=multi-user.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/radar
Environment=PATH=/home/pi/radar/venv/bin:/usr/bin
ExecStart=/home/pi/radar/venv/bin/python radar_processor.py --port /dev/ttyACM0 --threshold <YOUR_VALUE> --log /home/pi/radar/logs/radar.csv
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Enable and start:

```bash
sudo systemctl daemon-reload
sudo systemctl enable radar.service
sudo systemctl start radar.service
```

View logs:

```bash
journalctl -u radar.service -f
```

See [SETUP_GUIDE.md](SETUP_GUIDE.md) **Part 11** for full service setup.

## System Performance

| Specification | Value |
|---------------|-------|
| **RF Carrier** | 10.525 GHz (HB100 Gunn oscillator) |
| **IF Frequency Range** | ~70–700 Hz (typical Doppler) |
| **ADC Resolution** | 12-bit (0–4095 codes) |
| **Sampling Rate** | 10 kHz per channel |
| **Block Duration** | 102.4 ms (1024 samples) |
| **FFT Resolution** | ~9.77 Hz per bin |
| **DOA Resolution** | ±0.5–2° (depends on target SNR and element spacing) |
| **Tracking Update Rate** | ~9.8 Hz (one block per ~102 ms) |
| **Range Ambiguity** | None (Doppler-only system) |
| **Max Baseline** | 112 mm ≈ 3.93λ |

## External References

- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/)
- [MCP6002 Datasheet](https://www.microchip.com/en-us/product/MCP6002)
- [NumPy/SciPy Documentation](https://scipy.org/)

## License & Attribution

This project documents the design and implementation of a 4-element phased-array Doppler radar system. The analog signal conditioning, firmware architecture, and signal processing pipelines are original designs developed for this system.

## Getting Help

- **Firmware Issues**: Check [SETUP_GUIDE.md](SETUP_GUIDE.md) **Common build errors and fixes** (Part 3)
- **No USB Serial Connection**: See [SETUP_GUIDE.md](SETUP_GUIDE.md) **Part 6**
- **False Detections / Missing Targets**: See [SETUP_GUIDE.md](SETUP_GUIDE.md) **Part 10: Tune the system**
- **Technical Deep Dive**: See [HB100.md](../dchen/client/public/markdown/HB100.md) for signal conditioning, pipeline architecture, and performance rationale

---

**For comprehensive setup and operation instructions, see [SETUP_GUIDE.md](SETUP_GUIDE.md).**
