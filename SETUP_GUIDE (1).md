# HB100 Phased Array Radar: Complete Setup Guide

## What you need before starting

**Hardware (already built):**
- 4x HB100 modules in a row (37mm, 38mm, 37mm spacing)
- Double op-amp amplifier chain per channel (51x first stage, 20x second stage, 1.65V virtual ground, RC coupling between stages, 2kΩ + 100nF output filter)
- ESP32-S3-DevKitC-1 with GPIO 4, 5, 6, 7 connected to op-amp outputs
- Filtered 5V supply (3x 10R parallel + 3x capacitor LPF)
- USB cable (USB-C or Micro-USB depending on your DevKitC-1 revision)

**Hardware (host processor):**
- Raspberry Pi 4 Model B (4GB) with Raspberry Pi OS installed
- USB cable to connect ESP32-S3 to Pi
- Power supply for the Pi
- Monitor/keyboard for the Pi (or SSH access from your desktop)

**Software you will install:**
- ESP-IDF v5.1 or later (on your desktop or Pi for flashing)
- Python 3.9+ with pip (on the Pi)
- numpy, scipy, pyserial Python packages (on the Pi)


## Part 1: Install ESP-IDF on your development machine

You need ESP-IDF to compile and flash the firmware. You have two options for where to install it: your desktop (then flash over USB), or directly on the Pi (slower compilation but simpler workflow). The desktop is faster for compilation.

**Option A: Install on your desktop (recommended)**

Open a terminal and run the following commands. These download ESP-IDF and set up the toolchain for ESP32-S3 cross-compilation.

```bash
mkdir -p ~/esp
cd ~/esp
git clone -b v5.3 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32s3
```

This takes 10-20 minutes depending on your internet connection. The install script downloads the Xtensa compiler toolchain, CMake, Ninja, and other build dependencies.

After installation completes, activate the ESP-IDF environment. You need to run this command every time you open a new terminal to work with ESP-IDF:

```bash
source ~/esp/esp-idf/export.sh
```

Verify the installation:

```bash
idf.py --version
```

You should see something like `ESP-IDF v5.3`.

**Option B: Install on the Raspberry Pi**

Same commands as above, but compilation will take significantly longer on the Pi's ARM processor. A full build takes 5-10 minutes on the Pi vs. under a minute on a modern desktop.

```bash
sudo apt update
sudo apt install -y git wget flex bison gperf python3 python3-pip \
    python3-venv cmake ninja-build ccache libffi-dev libssl-dev \
    dfu-util libusb-1.0-0
mkdir -p ~/esp
cd ~/esp
git clone -b v5.3 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32s3
source ~/esp/esp-idf/export.sh
```


## Part 2: Create the firmware project

On the machine where you installed ESP-IDF, create the project directory structure:

```bash
mkdir -p ~/hb100_radar/main
cd ~/hb100_radar
```

Create the top-level CMakeLists.txt:

```bash
cat > CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(hb100_radar)
EOF
```

Create the main component CMakeLists.txt:

```bash
cat > main/CMakeLists.txt << 'EOF'
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
)
EOF
```

Create the sdkconfig defaults:

```bash
cat > sdkconfig.defaults << 'EOF'
# Target
CONFIG_IDF_TARGET="esp32s3"

# USB-CDC console
CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y

# CPU frequency
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y

# FreeRTOS tick rate (1ms resolution)
CONFIG_FREERTOS_HZ=1000
EOF
```

Now copy the main.c file from the downloaded project files into `~/hb100_radar/main/main.c`. If you are working over SSH or directly on the Pi, you transfer the file with scp:

```bash
scp /path/to/esp32_radar/main/main.c pi@<PI_IP>:~/hb100_radar/main/main.c
```

Or copy the content directly. The file is the `main.c` from the project download.


## Part 3: Build the firmware

Make sure your ESP-IDF environment is active:

```bash
source ~/esp/esp-idf/export.sh
```

Set the target to ESP32-S3 and build:

```bash
cd ~/hb100_radar
idf.py set-target esp32s3
idf.py build
```

The first build takes longer because it compiles the entire ESP-IDF framework. Subsequent builds are incremental and much faster.

If the build succeeds, you see output ending with:

```
Project build complete. To flash, run:
 idf.py flash
```

**Common build errors and fixes:**

If you see `No such file or directory: 'esp_adc/adc_continuous.h'`, your ESP-IDF version is too old. You need v5.1 or later.

If you see errors about `SOC_ADC_DIGI_RESULT_BYTES` being undefined, run `idf.py set-target esp32s3` again. This constant is only defined when the target is set to S3.


## Part 4: Flash the ESP32-S3

Connect the ESP32-S3-DevKitC-1 to your computer with a USB cable. The board has two USB ports on some revisions. Use the one labeled "USB" (not "UART"). On single-port boards, the one port handles both.

Put the board into download mode:

1. Hold the BOOT button on the ESP32-S3 board
2. While holding BOOT, press and release the RESET button
3. Release the BOOT button

The board is now in download mode. Flash the firmware:

```bash
idf.py -p /dev/ttyACM0 flash
```

On Linux, the ESP32-S3 shows up as `/dev/ttyACM0` or `/dev/ttyUSB0`. If you are unsure which port, run `ls /dev/tty*` before and after plugging in the board.

If you get a permission error, add your user to the dialout group:

```bash
sudo usermod -a -G dialout $USER
```

Then log out and log back in for the group change to take effect.

After flashing completes, press the RESET button on the board. The firmware starts running immediately and begins streaming ADC data over USB-CDC.

You verify it is running by checking for serial output:

```bash
idf.py -p /dev/ttyACM0 monitor
```

You should see log messages like:

```
I (300) RADAR: HB100 Phased Array Radar Collector
I (300) RADAR: Channels: GPIO7(ch0) GPIO6(ch1) GPIO5(ch2) GPIO4(ch3)
I (400) RADAR: Starting ADC DMA...
I (500) RADAR: Collecting data. Block size: 1024 samples, 4 channels, 10000 Hz
```

Press `Ctrl+]` to exit the monitor. The firmware keeps running after you disconnect the monitor.


## Part 5: Set up the Raspberry Pi

Connect to your Pi (either directly with a monitor or over SSH from your desktop):

```bash
ssh pi@<PI_IP>
```

Update the system and install Python dependencies:

```bash
sudo apt update
sudo apt install -y python3-pip python3-venv
```

Create a project directory and a virtual environment:

```bash
mkdir -p ~/radar
cd ~/radar
python3 -m venv venv
source venv/bin/activate
```

Install the Python packages:

```bash
pip install numpy scipy pyserial
```

This takes a few minutes on the Pi, especially numpy and scipy which compile native code. If scipy takes too long, install the system package instead:

```bash
sudo apt install -y python3-numpy python3-scipy
pip install pyserial
```

Copy the `radar_processor.py` file to the Pi. From your desktop:

```bash
scp /path/to/esp32_radar/host/radar_processor.py pi@<PI_IP>:~/radar/
```

Or place it directly in `~/radar/radar_processor.py`.


## Part 6: Connect the ESP32-S3 to the Pi

Unplug the ESP32-S3 from your desktop. Plug it into one of the Raspberry Pi's USB ports.

Verify the Pi sees the device:

```bash
ls /dev/ttyACM*
```

You should see `/dev/ttyACM0`. If you see nothing, try:

```bash
dmesg | tail -20
```

Look for lines mentioning `cdc_acm` or `USB Serial`. The device name appears in those messages.

If permissions block access:

```bash
sudo usermod -a -G dialout $USER
```

Log out and back in, or run:

```bash
newgrp dialout
```


## Part 7: Calibrate the noise floor

This step determines the detection threshold. You need a quiet RF environment with no moving objects in front of the array.

1. Power on the ESP32-S3 and the HB100 array
2. Make sure nothing is moving within 5-10 meters in front of the array
3. Stand still or leave the room

Run the calibration:

```bash
cd ~/radar
source venv/bin/activate
python radar_processor.py --port /dev/ttyACM0 --calibrate --cal-blocks 100
```

The script collects 100 blocks (about 10 seconds of data). It prints a progress bar while collecting. When done, it outputs noise floor statistics:

```
NOISE FLOOR STATISTICS
  Samples analyzed: 51100
  Mean magnitude:   8.42
  Std deviation:    3.17
  Median:           7.85
  95th percentile:  14.21
  99th percentile:  19.83
  Maximum:          31.47

RECOMMENDED THRESHOLDS
  Noise floor (mean + 3 std): 17.9
  Sensitive (3x p99):    --threshold 59
  Moderate  (5x p99):    --threshold 99
  Conservative (10x p99): --threshold 198
```

Your numbers will differ based on your op-amp noise, power supply quality, and RF environment. Write down the moderate threshold value. You use this in the next steps.

If the calibration fails to receive data (hangs with no progress), check:
- Is the ESP32-S3 powered and running? The onboard LED should be on.
- Is the USB cable a data cable, not charge-only?
- Is the port correct? Try `/dev/ttyUSB0` if `/dev/ttyACM0` doesn't work.
- Run `dmesg | tail` to check for USB errors.


## Part 8: First test with verbose diagnostics

This is where you verify everything works. Have someone walk back and forth in front of the array at 3-5 meters distance, roughly perpendicular to the array (walking left to right or right to left, not toward or away from the sensors).

```bash
python radar_processor.py --port /dev/ttyACM0 --threshold <YOUR_MODERATE_VALUE> --verbose
```

Replace `<YOUR_MODERATE_VALUE>` with the moderate threshold from calibration (for example, 99).

The terminal clears and redraws a full diagnostic panel on every detection. Here is what to check in each section:

**RAW ADC SIGNALS section:**
- All four channels should show a Mean(V) near 1.65V (your virtual ground bias). If a channel reads near 0V or 3.3V, the op-amp on that channel is saturated or disconnected.
- The P-P(V) column shows peak-to-peak voltage swing. With a person walking at 3-5 meters, you should see values in the range of 0.01 to 0.5V depending on distance and your gain. All four channels should show similar values. If one channel is significantly weaker, check that channel's op-amp wiring.

**FILTERED SIGNALS section:**
- RMS(V) shows the signal strength after bandpass filtering. When a person walks by, these values increase noticeably. The bar graphs should all light up roughly the same amount. If channel 0 shows strong signal but channel 3 shows nothing, it means only the far-left sensor is picking up the target (which implies the person is at an extreme angle, or channel 3's analog chain has a problem).

**FFT ANALYSIS section:**
- All four channels should show the same Peak Hz value (within 1-2 Hz). This is the Doppler frequency of the target. For a person walking at 1.5 m/s, the peak should be around 105 Hz. For a slow walk (0.8 m/s), about 56 Hz. For a jog (3 m/s), about 210 Hz.
- If different channels show different peak frequencies, you have interference on one channel. Check for ground loops or inadequate shielding.
- The Ch SNR column shows per-channel signal-to-noise ratio in dB. Anything above 10 dB is a clean detection. Below 5 dB is marginal.

**DOPPLER section:**
- Shows the combined peak frequency converted to speed in m/s, km/h, and mph.
- A walking person should register between 0.8 and 2.0 m/s (2.9 to 7.2 km/h).
- The threshold status shows ABOVE (green) or BELOW (red) to indicate whether the detection passed your threshold.

**PHASE EXTRACTION section:**
- Shows the raw phase angle at the Doppler peak for each channel and all six pairwise phase differences.
- The short baselines (0-1, 1-2, 2-3) will often show "ALIASED" because your 37-38mm spacing exceeds the half-wavelength limit at 10.525 GHz. This is expected.
- The longer baselines (0-2, 0-3, 1-3) will alias even more frequently. Also expected.
- The naive angle estimates from individual pairs will disagree with each other. This is normal. The MUSIC algorithm handles the ambiguity.

**MUSIC PSEUDOSPECTRUM section:**
- Shows an ASCII plot of the spatial spectrum from -80 to +80 degrees.
- You should see one strong peak at the angle where the person is standing.
- If you see multiple peaks of similar height, those are grating lobes from the over-spaced array. The tallest peak is the true target angle.
- As the person walks left to right, the peak angle should sweep smoothly from negative to positive degrees.

**KALMAN FILTER STATE section:**
- Position shows estimated x and y coordinates. Since you have no range measurement, the x value (range) is approximate. The y value (cross-range) is more reliable.
- Speed and heading show the filtered velocity estimate. This should be smoother than the raw Doppler readings.
- Direction shows an arrow and label (MOVING LEFT, MOVING RIGHT, etc.). Verify this matches what the person is doing.
- Health shows whether the filter is tracking correctly. "OK" means the filter model matches the measurements. "DIVERGING" means the filter needs tuning (adjust the process noise or measurement noise constants in the code).


## Part 9: Run in compact mode with logging

Once you have verified the system works in verbose mode, switch to compact mode for continuous operation. Compact mode prints one line per detection instead of clearing the screen.

```bash
python radar_processor.py --port /dev/ttyACM0 --threshold <YOUR_VALUE> --log measurements.csv
```

Output looks like:

```
Block  |  Doppler |     Speed |  Angle |    SNR | EKF Speed | EKF Heading |            Direction
----------------------------------------------------------------------------------------------------
   142 |  103.2Hz |  1.47 m/s | +12.3° |  18.2dB |   1.44m/s |      +15.2° |            -> RIGHT
   143 |  105.1Hz |  1.50 m/s | +14.1° |  17.8dB |   1.46m/s |      +14.8° |            -> RIGHT
   144 |  101.8Hz |  1.45 m/s | +15.8° |  16.5dB |   1.45m/s |      +15.1° |            -> RIGHT
```

The CSV file records every detection with timestamps, all phase values, and the full Kalman filter state. You load this in Python or Excel later for analysis.


## Part 10: Tune the system

After your first test session, you have data to tune with.

**If you get too many false detections (detections when nothing is moving):**
Increase the threshold. Try the conservative value from calibration, or re-run calibration. False detections often come from fans, HVAC systems, or vibrating objects in the environment.

**If you miss detections at longer range:**
Decrease the threshold toward the sensitive value. The tradeoff is more false positives.

**If the Kalman filter heading oscillates or jumps erratically:**
Open `radar_processor.py` and increase `KF_ANGLE_NOISE` from `np.radians(5)` to `np.radians(10)` or `np.radians(15)`. This tells the filter to trust the angle measurements less and rely more on the motion model.

**If the Kalman filter speed lags behind quick changes:**
Increase `KF_PROCESS_NOISE` from 0.5 to 1.0 or 2.0. This tells the filter the target accelerates more frequently, making it more responsive to speed changes.

**If all four channels show different Doppler frequencies:**
You have electromagnetic interference. Check that the 5V power filter is working. Try adding a ferrite bead on the USB cable. Make sure the HB100 modules' ground planes are connected to the ESP32-S3 ground with short, thick wires.

**If the MUSIC angle seems stuck or wrong:**
Verify your element positions in the code match reality. Measure the center-to-center distance between adjacent HB100 modules again. A 1mm error in the `ELEMENT_POSITIONS` array causes a few degrees of angle error at 10.525 GHz.


## Part 11: Run the system on boot (optional)

If you want the Pi to start the radar processor automatically when it boots:

Create a systemd service file:

```bash
sudo nano /etc/systemd/system/radar.service
```

Paste this content:

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

Replace `<YOUR_VALUE>` with your calibrated threshold.

Create the logs directory and enable the service:

```bash
mkdir -p ~/radar/logs
sudo systemctl daemon-reload
sudo systemctl enable radar.service
sudo systemctl start radar.service
```

Check that it is running:

```bash
sudo systemctl status radar.service
```

View live output:

```bash
journalctl -u radar.service -f
```


## Quick reference commands

```bash
# Activate Python environment
cd ~/radar && source venv/bin/activate

# Calibrate (do this first, no moving targets)
python radar_processor.py --port /dev/ttyACM0 --calibrate --cal-blocks 100

# Full diagnostics (for testing and debugging)
python radar_processor.py --port /dev/ttyACM0 --threshold 99 --verbose

# Compact mode with logging (for continuous operation)
python radar_processor.py --port /dev/ttyACM0 --threshold 99 --log data.csv

# Re-flash the ESP32-S3 (if you modify firmware)
cd ~/hb100_radar
source ~/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyACM0 flash

# Check if ESP32 is connected
ls /dev/ttyACM*
dmesg | tail -20

# Check service status (if running on boot)
sudo systemctl status radar.service
```


## File locations summary

| File | Location | Purpose |
|------|----------|---------|
| ESP32 firmware | `~/hb100_radar/main/main.c` | ADC DMA sampling, USB serial output |
| ESP32 project config | `~/hb100_radar/CMakeLists.txt` | ESP-IDF build config |
| ESP32 sdkconfig | `~/hb100_radar/sdkconfig.defaults` | ESP32-S3 target settings |
| Python processor | `~/radar/radar_processor.py` | DSP, MUSIC, Kalman filter |
| Python venv | `~/radar/venv/` | Isolated Python environment |
| CSV log files | `~/radar/logs/` or `~/radar/data.csv` | Measurement history |
