# HB100 Phased Array Radar

4-element HB100 Doppler radar array with ESP32-S3 data collection and
Raspberry Pi 4 host processing. Computes target speed, angle of arrival,
and tracks movement using an Extended Kalman Filter.

## Hardware

- 4x HB100 microwave Doppler modules (10.525 GHz)
- ESP32-S3-DevKitC-1
- Raspberry Pi 4 Model B (4GB)

### Wiring

| HB100 | IF Pin | 5V Pin | GND | ESP32-S3 GPIO |
|-------|--------|--------|-----|---------------|
| #0 (far left) | GPIO 7 | 5V | GND | ADC1_CH6 |
| #1 | GPIO 6 | 5V | GND | ADC1_CH5 |
| #2 | GPIO 5 | 5V | GND | ADC1_CH4 |
| #3 (far right) | GPIO 4 | 5V | GND | ADC1_CH3 |

### Element spacing (center to center)
- #0 to #1: 37mm
- #1 to #2: 38mm
- #2 to #3: 37mm

## Building the ESP32-S3 Firmware

Requires ESP-IDF v5.1+.

```bash
cd esp32_radar
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyUSB0 flash
```

## Running the Host Processor

On the Raspberry Pi 4 (or any Linux machine):

```bash
cd esp32_radar/host
pip install -r requirements.txt
python radar_processor.py --port /dev/ttyACM0
```

### Options

- `--port`: Serial port (default: /dev/ttyACM0)
- `--baud`: Baud rate (default: 921600)
- `--log measurements.csv`: Save all detections to CSV for analysis

## Binary Protocol

The ESP32 sends blocks over USB-CDC serial:

| Field | Size | Description |
|-------|------|-------------|
| Sync | 4 bytes | 0xDEADBEEF |
| Count | 4 bytes | uint32 LE, samples per block (1024) |
| Data | 8192 bytes | 1024 samples x 4 channels x uint16 LE |

Channel order per sample row: ch0, ch1, ch2, ch3.

## Processing Pipeline

1. Remove DC offset per channel
2. Bandpass filter (10-2000 Hz, 4th order Butterworth)
3. Hanning window + FFT
4. Peak Doppler bin detection with SNR threshold
5. MUSIC algorithm for angle of arrival (spatial smoothing, forward-backward averaging)
6. Extended Kalman Filter for target tracking (state: x, y, vx, vy)

## Future Expansion

- Second array at known offset for range triangulation
- FMCW mode using HB100 varactor tuning for direct range measurement
- Turret/laser pointing using EKF state output
