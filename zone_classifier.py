"""
HB100 Phased Array - Amplitude Monopulse Zone Classifier
=========================================================

Pragmatic processor for 4x independent HB100 modules that cannot achieve
phase coherence across channels (each HB100 has its own free-running Gunn
oscillator). Instead of phase-based DOA (MUSIC), this uses FFT magnitudes
per channel at the Doppler peak -- robust to LO drift because it uses only
power, not phase.

What works:
  - Speed (Doppler) measurement per channel -- LO drift << Doppler frequency
  - Per-channel FFT magnitude at peak bin -- no phase, so LO-independent
  - Left/right zone classification from amplitude ratio (monopulse)

What was dropped (needs phase coherence we don't have):
  - MUSIC pseudospectrum
  - Pairwise phase-difference DOA
  - 2-D (x,y) position tracking

Hardware layout (viewed from behind array, looking outward):
    ch0 (GPIO7)  |  ch1 (GPIO6)  |  ch2 (GPIO5)  |  ch3 (GPIO4)
    <-- 37mm -->  <-- 38mm -->    <-- 37mm -->
    farthest left                              closest / rightmost

Zone convention (from the array's perspective):
    FAR_LEFT | LEFT | CENTER | RIGHT | FAR_RIGHT
    ch0 strong <---------->  ch3 strong

Usage:
    # Calibrate noise floor first (no moving targets):
    python zone_classifier.py --port /dev/ttyACM0 --calibrate --cal-blocks 100

    # Run with zone classification:
    python zone_classifier.py --port /dev/ttyACM0 --threshold 5 --verbose

    # Log to CSV:
    python zone_classifier.py --port /dev/ttyACM0 --threshold 5 --log data.csv
"""

import argparse
import struct
import time
import sys
import os
import json
from collections import deque
from datetime import datetime

import numpy as np
from scipy.signal import butter, sosfilt
import serial


# --------------- Physical Constants ---------------

F_CARRIER       = 10.525e9
C_SPEED         = 3.0e8
WAVELENGTH      = C_SPEED / F_CARRIER  # ~0.0285 m

NUM_CHANNELS    = 4
SAMPLES_PER_BLOCK = 1024
SAMPLE_RATE     = 10000
ADC_MAX         = 4095
ADC_VREF        = 3.3

# DSP parameters
FFT_SIZE        = SAMPLES_PER_BLOCK
BANDPASS_LOW    = 10.0
BANDPASS_HIGH   = 800.0   # Matches 2k + 100nF hardware LPF rolloff
MIN_MAGNITUDE   = 5.0     # Use --calibrate to find real value

# --------------- Monopulse / Zone Parameters ---------------
# Bias is in [-1, +1]: -1 = all energy in ch0, +1 = all energy in ch3.
# These thresholds can be tuned empirically once you calibrate against
# targets at known angles.
ZONE_THRESHOLDS = {
    'FAR_LEFT':   -0.50,   # bias <= -0.50
    'LEFT':       -0.15,   # -0.50 < bias <= -0.15
    'CENTER':      0.15,   # -0.15 < bias <=  0.15
    'RIGHT':       0.50,   # 0.15 < bias <=  0.50
    # bias > 0.50 -> FAR_RIGHT
}

ZONE_LABELS = ['FAR_LEFT', 'LEFT', 'CENTER', 'RIGHT', 'FAR_RIGHT']

# --------------- Speed Tracker Parameters (alpha-beta filter) ---------------
# alpha: weight on new speed measurements (0 = ignore, 1 = fully trust)
# beta:  weight on acceleration update
SPEED_ALPHA  = 0.35
SPEED_BETA   = 0.10

SYNC_WORD = b'\xDE\xAD\xBE\xEF'


# --------------- Terminal Colors ---------------

class C:
    """ANSI color codes for terminal output."""
    RESET   = '\033[0m'
    BOLD    = '\033[1m'
    DIM     = '\033[2m'
    RED     = '\033[91m'
    GREEN   = '\033[92m'
    YELLOW  = '\033[93m'
    BLUE    = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN    = '\033[96m'
    WHITE   = '\033[97m'

    @staticmethod
    def bar(value, max_val, width=30, fill_char='\u2588', empty_char='\u2591'):
        """Render a horizontal bar."""
        filled = int(min(value / max(max_val, 1e-10), 1.0) * width)
        return fill_char * filled + empty_char * (width - filled)


# --------------- Serial Reader ---------------

class BlockReader:
    """Reads binary sample blocks from ESP32-S3 USB serial."""

    def __init__(self, port: str, baudrate: int = 921600):
        self.ser = serial.Serial(port, baudrate, timeout=1.0)
        self.ser.reset_input_buffer()
        self.bytes_received = 0
        self.sync_errors = 0

    def read_block(self) -> np.ndarray | None:
        buf = b''
        scan_bytes = 0
        while True:
            byte = self.ser.read(1)
            if len(byte) == 0:
                return None
            scan_bytes += 1
            if scan_bytes > 50000:
                self.sync_errors += 1
                return None
            buf += byte
            if len(buf) > 4:
                buf = buf[-4:]
            if buf == SYNC_WORD:
                break

        count_bytes = self.ser.read(4)
        if len(count_bytes) < 4:
            return None
        sample_count = struct.unpack('<I', count_bytes)[0]

        if sample_count != SAMPLES_PER_BLOCK:
            self.sync_errors += 1
            return None

        data_size = sample_count * NUM_CHANNELS * 2
        raw = self.ser.read(data_size)
        if len(raw) < data_size:
            return None

        self.bytes_received += 4 + 4 + data_size
        data = np.frombuffer(raw, dtype=np.uint16).reshape(sample_count, NUM_CHANNELS)
        return data.astype(np.float64) * (ADC_VREF / ADC_MAX)

    def close(self):
        self.ser.close()


# --------------- DSP Pipeline ---------------

def classify_zone(bias: float) -> str:
    """Map bias in [-1, +1] to a zone label."""
    if bias <= ZONE_THRESHOLDS['FAR_LEFT']:
        return 'FAR_LEFT'
    elif bias <= ZONE_THRESHOLDS['LEFT']:
        return 'LEFT'
    elif bias <= ZONE_THRESHOLDS['CENTER']:
        return 'CENTER'
    elif bias <= ZONE_THRESHOLDS['RIGHT']:
        return 'RIGHT'
    else:
        return 'FAR_RIGHT'


class DSPPipeline:
    """
    Amplitude-based DSP pipeline. No phase processing -- every output
    depends only on FFT magnitudes, so LO incoherence doesn't matter.
    """

    def __init__(self, magnitude_threshold: float = MIN_MAGNITUDE, gain_factors: dict = None):
        self.magnitude_threshold = magnitude_threshold

        # Per-channel gain correction factors [ch0, ch1, ch2, ch3]
        # Multiply channel_mags[i] by gain_factors[i] to equalize.
        # If None (no calibration), default to unity gains.
        if gain_factors is None:
            self.gain_factors = np.ones(NUM_CHANNELS)
        else:
            self.gain_factors = np.array([gain_factors.get(str(i), 1.0) for i in range(NUM_CHANNELS)])

        self.sos = butter(
            4,
            [BANDPASS_LOW, BANDPASS_HIGH],
            btype='bandpass',
            fs=SAMPLE_RATE,
            output='sos'
        )

        self.window = np.hanning(SAMPLES_PER_BLOCK)
        self.freqs = np.fft.rfftfreq(FFT_SIZE, d=1.0 / SAMPLE_RATE)

    def process_block(self, voltages: np.ndarray) -> dict:
        """
        Process one block. Always returns a dict with diagnostics.
        'detected' field indicates whether a target was found.
        """
        result = {
            'detected': False,
            'raw_stats': {},
            'filtered_stats': {},
            'fft_stats': {},
            'channel_mags': None,
            'bias': None,
            'centroid': None,
            'zone': None,
        }

        # --- Raw signal stats ---
        for ch in range(NUM_CHANNELS):
            v = voltages[:, ch]
            result['raw_stats'][ch] = {
                'mean_v': np.mean(v),
                'std_v': np.std(v),
                'min_v': np.min(v),
                'max_v': np.max(v),
                'pp_v': np.max(v) - np.min(v),
            }

        # 1. Remove DC offset
        centered = voltages - voltages.mean(axis=0, keepdims=True)

        # 2. Bandpass filter
        filtered = np.zeros_like(centered)
        for ch in range(NUM_CHANNELS):
            filtered[:, ch] = sosfilt(self.sos, centered[:, ch])

        for ch in range(NUM_CHANNELS):
            v = filtered[:, ch]
            result['filtered_stats'][ch] = {
                'std_v': np.std(v),
                'pp_v': np.max(v) - np.min(v),
                'rms_v': np.sqrt(np.mean(v**2)),
            }

        # 3. Window + FFT
        windowed = filtered * self.window[:, np.newaxis]
        spectra = np.fft.rfft(windowed, n=FFT_SIZE, axis=0)

        magnitudes = np.abs(spectra)
        avg_magnitude = magnitudes.mean(axis=1)

        # Per-channel FFT stats
        for ch in range(NUM_CHANNELS):
            ch_mag = magnitudes[:, ch]
            peak_bin = np.argmax(ch_mag[1:]) + 1
            result['fft_stats'][ch] = {
                'peak_bin': peak_bin,
                'peak_freq': self.freqs[peak_bin],
                'peak_mag': ch_mag[peak_bin],
                'noise_floor': np.median(ch_mag[1:]),
            }

        # Combined peak detection (common Doppler bin across channels)
        peak_bin = np.argmax(avg_magnitude[1:]) + 1
        peak_mag = avg_magnitude[peak_bin]
        noise_floor = np.median(avg_magnitude[1:])
        noise_std = np.std(avg_magnitude[1:])

        result['combined_peak_bin'] = peak_bin
        result['combined_peak_freq'] = self.freqs[peak_bin]
        result['combined_peak_mag'] = peak_mag
        result['combined_noise_floor'] = noise_floor
        result['combined_noise_std'] = noise_std
        result['combined_snr_db'] = 20 * np.log10(peak_mag / max(noise_floor, 1e-10))

        # Doppler speed
        doppler_freq = self.freqs[peak_bin]
        speed_mps = (doppler_freq * C_SPEED) / (2.0 * F_CARRIER)
        result['doppler_hz'] = doppler_freq
        result['speed_mps'] = speed_mps

        if peak_mag < self.magnitude_threshold:
            return result

        result['detected'] = True

        # --- Amplitude monopulse (the only DOA mechanism) ---
        # Per-channel magnitudes at the common Doppler peak
        channel_mags = magnitudes[peak_bin, :].copy()

        # Apply per-channel gain normalization to correct for hardware imbalances
        channel_mags *= self.gain_factors

        result['channel_mags'] = channel_mags

        # Simple left/right bias: -1 = all on left, +1 = all on right
        left = channel_mags[0] + channel_mags[1]
        right = channel_mags[2] + channel_mags[3]
        total = left + right
        bias = (right - left) / max(total, 1e-10)
        result['bias'] = bias

        # Centroid-based estimate (weighted channel index, normalized to [-1,+1])
        # 0 -> ch0, 3 -> ch3, map to bias range by (centroid - 1.5) / 1.5
        weights = channel_mags / max(np.sum(channel_mags), 1e-10)
        raw_centroid = np.sum(weights * np.arange(NUM_CHANNELS))
        centroid_bias = (raw_centroid - 1.5) / 1.5
        result['centroid'] = centroid_bias

        # Classify using the simple bias (or you could average bias and centroid)
        result['zone'] = classify_zone(bias)

        return result


# --------------- Speed Tracker (1-D alpha-beta filter) ---------------

class SpeedTracker:
    """
    Simple alpha-beta filter for speed. State: [speed, acceleration].
    Much simpler than an EKF since we only track a scalar.
    """

    def __init__(self, dt: float, alpha: float = SPEED_ALPHA, beta: float = SPEED_BETA):
        self.dt = dt
        self.alpha = alpha
        self.beta = beta
        self.speed = 0.0
        self.accel = 0.0
        self.initialized = False
        self.history = deque(maxlen=50)

    def predict(self):
        """Predict speed forward by dt (used when no detection)."""
        if self.initialized:
            self.speed = max(0.0, self.speed + self.accel * self.dt)

    def update(self, measured_speed: float):
        """Fuse a new speed measurement."""
        if not self.initialized:
            self.speed = measured_speed
            self.accel = 0.0
            self.initialized = True
            self.history.append(measured_speed)
            return

        # Predict
        pred_speed = self.speed + self.accel * self.dt

        # Residual
        residual = measured_speed - pred_speed

        # Update
        self.speed = pred_speed + self.alpha * residual
        self.accel = self.accel + (self.beta / self.dt) * residual
        self.speed = max(0.0, self.speed)  # speed is non-negative magnitude

        self.history.append(measured_speed)

    def get_state(self) -> dict:
        if len(self.history) > 5:
            innov_std = float(np.std(list(self.history)))
        else:
            innov_std = 0.0
        return {
            'speed_mps': self.speed,
            'accel_mps2': self.accel,
            'innov_std': innov_std,
        }


# --------------- Zone Tracker (debounced zone state) ---------------

class ZoneTracker:
    """
    Smooths the zone classification over recent blocks. A single noisy
    block shouldn't flip the reported zone.
    """

    def __init__(self, window_size: int = 5):
        self.window_size = window_size
        self.history = deque(maxlen=window_size)
        self.current_zone = 'CENTER'

    def update(self, zone: str):
        self.history.append(zone)
        # Majority vote
        if len(self.history) >= 3:
            counts = {}
            for z in self.history:
                counts[z] = counts.get(z, 0) + 1
            self.current_zone = max(counts, key=counts.get)
        else:
            self.current_zone = zone

    def get_zone(self) -> str:
        return self.current_zone

    def reset(self):
        self.history.clear()
        self.current_zone = 'CENTER'


# --------------- Display Functions ---------------

def clear_screen():
    os.system('clear' if os.name == 'posix' else 'cls')


def zone_arrow(zone: str) -> str:
    """Return a visual arrow for the zone."""
    return {
        'FAR_LEFT':  '<<<<<',
        'LEFT':      ' <<  ',
        'CENTER':    '  ^  ',
        'RIGHT':     '  >> ',
        'FAR_RIGHT': '>>>>>',
    }.get(zone, '  ?  ')


def zone_color(zone: str) -> str:
    return {
        'FAR_LEFT':  C.MAGENTA,
        'LEFT':      C.BLUE,
        'CENTER':    C.GREEN,
        'RIGHT':     C.YELLOW,
        'FAR_RIGHT': C.RED,
    }.get(zone, C.WHITE)


def render_zone_strip(zone: str, bias: float) -> str:
    """Render a horizontal strip showing all 5 zones with the current one highlighted."""
    strip = ""
    for label in ZONE_LABELS:
        if label == zone:
            strip += f"{zone_color(label)}{C.BOLD}[{label:^9}]{C.RESET}"
        else:
            strip += f"{C.DIM}[{label:^9}]{C.RESET}"
    return strip


def print_header(block_count, detect_count, elapsed, reader):
    rate = block_count / max(elapsed, 0.01)
    bandwidth = reader.bytes_received / max(elapsed, 0.01) / 1024
    print(f"{C.BOLD}{C.CYAN}+==============================================================================+{C.RESET}")
    print(f"{C.BOLD}{C.CYAN}|  HB100 AMPLITUDE-MONOPULSE ZONE CLASSIFIER{' ' * 34}|{C.RESET}")
    print(f"{C.BOLD}{C.CYAN}+==============================================================================+{C.RESET}")
    print(f"{C.CYAN}|{C.RESET}  Block: {C.WHITE}{block_count:6d}{C.RESET}  "
          f"Detections: {C.GREEN}{detect_count:5d}{C.RESET}  "
          f"Rate: {rate:5.1f} blk/s  "
          f"BW: {bandwidth:5.1f} KB/s  "
          f"Sync err: {C.RED if reader.sync_errors > 0 else C.DIM}{reader.sync_errors}{C.RESET}"
          f"  {C.CYAN}|{C.RESET}")
    print(f"{C.CYAN}+------------------------------------------------------------------------------+{C.RESET}")


def print_raw_signals(result):
    print(f"{C.CYAN}|{C.RESET} {C.BOLD}RAW ADC SIGNALS{C.RESET}")
    print(f"{C.CYAN}|{C.RESET}   {'Chan':>4}  {'Mean(V)':>8}  {'Std(V)':>8}  {'P-P(V)':>8}  Signal Level")
    for ch in range(NUM_CHANNELS):
        s = result['raw_stats'][ch]
        bar = C.bar(s['pp_v'], 3.3, width=20)
        color = C.GREEN if s['pp_v'] > 0.01 else C.DIM
        print(f"{C.CYAN}|{C.RESET}   {color}ch{ch}   "
              f"{s['mean_v']:8.4f}  {s['std_v']:8.5f}  {s['pp_v']:8.5f}{C.RESET}  {bar}")


def print_filtered_signals(result):
    print(f"{C.CYAN}|{C.RESET}")
    print(f"{C.CYAN}|{C.RESET} {C.BOLD}FILTERED SIGNALS (bandpass {BANDPASS_LOW}-{BANDPASS_HIGH} Hz){C.RESET}")
    print(f"{C.CYAN}|{C.RESET}   {'Chan':>4}  {'RMS(V)':>10}  {'P-P(V)':>10}  Activity")
    max_rms = max(result['filtered_stats'][ch]['rms_v'] for ch in range(NUM_CHANNELS))
    for ch in range(NUM_CHANNELS):
        s = result['filtered_stats'][ch]
        bar = C.bar(s['rms_v'], max(max_rms * 1.2, 0.001), width=25)
        color = C.GREEN if s['rms_v'] > 0.001 else C.DIM
        print(f"{C.CYAN}|{C.RESET}   {color}ch{ch}   "
              f"{s['rms_v']:10.6f}  {s['pp_v']:10.6f}{C.RESET}  {bar}")


def print_fft_and_doppler(result):
    print(f"{C.CYAN}|{C.RESET}")
    print(f"{C.CYAN}|{C.RESET} {C.BOLD}DOPPLER{C.RESET}  "
          f"freq={result['doppler_hz']:.1f}Hz  "
          f"speed={C.BOLD}{result['speed_mps']:.3f} m/s{C.RESET} "
          f"({result['speed_mps'] * 3.6:.1f} km/h, {result['speed_mps'] * 2.237:.1f} mph)")
    thresh_status = f"{C.GREEN}ABOVE{C.RESET}" if result['detected'] else f"{C.RED}BELOW{C.RESET}"
    print(f"{C.CYAN}|{C.RESET}   peak_bin={result['combined_peak_bin']}  "
          f"peak_mag={result['combined_peak_mag']:.2f}  "
          f"noise={result['combined_noise_floor']:.2f} +/- {result['combined_noise_std']:.2f}  "
          f"SNR={result['combined_snr_db']:.1f}dB  "
          f"threshold: {thresh_status}")


def print_channel_mags(result):
    """Core display for amplitude-monopulse DOA."""
    if result['channel_mags'] is None:
        return
    print(f"{C.CYAN}|{C.RESET}")
    print(f"{C.CYAN}|{C.RESET} {C.BOLD}PER-CHANNEL MAGNITUDES @ DOPPLER PEAK "
          f"({result['doppler_hz']:.1f} Hz){C.RESET}")
    mags = result['channel_mags']
    max_mag = float(np.max(mags))
    for ch in range(NUM_CHANNELS):
        bar = C.bar(mags[ch], max(max_mag * 1.1, 0.1), width=40)
        # Color the strongest channel green
        color = C.GREEN if mags[ch] == max_mag else C.WHITE
        side = "LEFT " if ch < 2 else "RIGHT"
        print(f"{C.CYAN}|{C.RESET}   {color}ch{ch} ({side}) "
              f"{mags[ch]:8.2f}{C.RESET}  {bar}")


def print_zone_classification(result, zone_tracker: 'ZoneTracker'):
    if result['bias'] is None:
        return
    print(f"{C.CYAN}|{C.RESET}")
    print(f"{C.CYAN}|{C.RESET} {C.BOLD}ZONE CLASSIFICATION{C.RESET}")

    bias = result['bias']
    centroid = result['centroid']
    instant_zone = result['zone']
    smoothed_zone = zone_tracker.get_zone()

    print(f"{C.CYAN}|{C.RESET}   bias (L/R ratio):   {bias:+.3f}   "
          f"(-1=all LEFT, +1=all RIGHT)")
    print(f"{C.CYAN}|{C.RESET}   centroid-bias:      {centroid:+.3f}")

    bias_bar_width = 40
    bias_pos = int((bias + 1.0) / 2.0 * bias_bar_width)
    bias_pos = max(0, min(bias_bar_width - 1, bias_pos))
    bias_bar = ['-'] * bias_bar_width
    bias_bar[bias_bar_width // 2] = '|'
    bias_bar[bias_pos] = '#'
    print(f"{C.CYAN}|{C.RESET}   [LEFT]{''.join(bias_bar)}[RIGHT]")

    print(f"{C.CYAN}|{C.RESET}")
    arrow = zone_arrow(smoothed_zone)
    zcolor = zone_color(smoothed_zone)
    print(f"{C.CYAN}|{C.RESET}   Instant zone:  {zone_color(instant_zone)}{instant_zone:>9}{C.RESET}")
    print(f"{C.CYAN}|{C.RESET}   Smoothed zone: {zcolor}{C.BOLD}{smoothed_zone:>9}{C.RESET} {C.BOLD}{zcolor}{arrow}{C.RESET}")
    print(f"{C.CYAN}|{C.RESET}   {render_zone_strip(smoothed_zone, bias)}")


def print_speed_tracker(state):
    print(f"{C.CYAN}|{C.RESET}")
    print(f"{C.CYAN}|{C.RESET} {C.BOLD}SPEED TRACKER (1-D alpha-beta){C.RESET}")
    print(f"{C.CYAN}|{C.RESET}   Tracked speed: {C.BOLD}{state['speed_mps']:.3f} m/s{C.RESET}  "
          f"({state['speed_mps'] * 3.6:.1f} km/h)")
    print(f"{C.CYAN}|{C.RESET}   Acceleration:  {state['accel_mps2']:+.3f} m/s^2")
    print(f"{C.CYAN}|{C.RESET}   Meas. std:     {state['innov_std']:.3f} m/s")


def print_footer():
    print(f"{C.CYAN}+==============================================================================+{C.RESET}")


def print_no_detection_summary(result, block_count):
    noise = result['combined_noise_floor']
    peak = result['combined_peak_mag']
    max_rms = max(result['filtered_stats'][ch]['rms_v'] for ch in range(NUM_CHANNELS))
    print(f"{C.DIM}[{block_count:6d}] No target  "
          f"peak={peak:.2f}  noise={noise:.2f}  "
          f"max_rms={max_rms:.6f}V{C.RESET}")


# --------------- Calibration Mode ---------------

def run_calibration(reader, dsp, num_blocks):
    print(f"\n{C.BOLD}{C.YELLOW}CALIBRATION MODE{C.RESET}")
    print(f"Collecting {num_blocks} blocks with NO moving targets present...")
    print(f"Keep the area in front of the array clear.\n")

    all_magnitudes = []
    per_ch_magnitudes = [[] for _ in range(NUM_CHANNELS)]
    block_count = 0

    # Find Doppler band frequency indices (10-800 Hz)
    doppler_low_idx = np.searchsorted(dsp.freqs, BANDPASS_LOW)
    doppler_high_idx = np.searchsorted(dsp.freqs, BANDPASS_HIGH)

    while block_count < num_blocks:
        block = reader.read_block()
        if block is None:
            continue

        block_count += 1

        centered = block - block.mean(axis=0, keepdims=True)
        filtered = np.zeros_like(centered)
        for ch in range(NUM_CHANNELS):
            filtered[:, ch] = sosfilt(dsp.sos, centered[:, ch])
        windowed = filtered * dsp.window[:, np.newaxis]
        spectra = np.fft.rfft(windowed, n=FFT_SIZE, axis=0)
        mags = np.abs(spectra)

        # Average magnitude across all frequencies for noise floor
        avg_mag = mags.mean(axis=1)
        all_magnitudes.extend(avg_mag[1:].tolist())

        # Per-channel magnitude in Doppler band for gain equalization
        for ch in range(NUM_CHANNELS):
            ch_doppler_band = mags[doppler_low_idx:doppler_high_idx, ch]
            per_ch_magnitudes[ch].extend(ch_doppler_band.tolist())

        pct = block_count / num_blocks * 100
        bar = C.bar(block_count, num_blocks, width=40)
        print(f"\r  {bar} {pct:5.1f}%  ({block_count}/{num_blocks})", end="", flush=True)

    print("\n")

    # Overall noise floor statistics
    mags = np.array(all_magnitudes)
    mean_mag = float(np.mean(mags))
    std_mag = float(np.std(mags))
    median_mag = float(np.median(mags))
    p95 = float(np.percentile(mags, 95))
    p99 = float(np.percentile(mags, 99))
    max_mag = float(np.max(mags))

    print(f"{C.BOLD}NOISE FLOOR STATISTICS{C.RESET}")
    print(f"  Samples analyzed: {len(mags)}")
    print(f"  Mean magnitude:   {mean_mag:.2f}")
    print(f"  Std deviation:    {std_mag:.2f}")
    print(f"  Median:           {median_mag:.2f}")
    print(f"  95th percentile:  {p95:.2f}")
    print(f"  99th percentile:  {p99:.2f}")
    print(f"  Maximum:          {max_mag:.2f}")
    print()

    # Per-channel RMS in Doppler band (for gain equalization)
    per_ch_rms = []
    print(f"{C.BOLD}PER-CHANNEL DOPPLER BAND RMS (gain calibration){C.RESET}")
    for ch in range(NUM_CHANNELS):
        ch_mags = np.array(per_ch_magnitudes[ch])
        ch_rms = np.sqrt(np.mean(ch_mags**2))
        per_ch_rms.append(ch_rms)
        print(f"  ch{ch}: {ch_rms:.4f}")

    # Compute gain factors: scale all channels to match ch0 (reference)
    ref_rms = per_ch_rms[0]
    gain_factors = {}
    print(f"\n{C.BOLD}GAIN CORRECTION FACTORS{C.RESET}")
    for ch in range(NUM_CHANNELS):
        if per_ch_rms[ch] > 0:
            gain = ref_rms / per_ch_rms[ch]
        else:
            gain = 1.0
        gain_factors[str(ch)] = float(gain)
        print(f"  ch{ch}: {gain:.4f}")

    # Save calibration to file
    with open('gain_calibration.json', 'w') as f:
        json.dump({'gain_factors': gain_factors}, f, indent=2)
    print(f"\n{C.BOLD}Saved to gain_calibration.json{C.RESET}")
    print()

    conservative = p99 * 10
    moderate = p99 * 5
    sensitive = p99 * 3

    floor_3sigma = mean_mag + 3 * std_mag

    print(f"{C.BOLD}RECOMMENDED THRESHOLDS{C.RESET}")
    print(f"  Noise floor (mean + 3 std): {floor_3sigma:.1f}")
    print(f"  {C.GREEN}Sensitive (3x p99):    --threshold {sensitive:.0f}{C.RESET}  (more detections, more false positives)")
    print(f"  {C.YELLOW}Moderate  (5x p99):    --threshold {moderate:.0f}{C.RESET}  (balanced)")
    print(f"  {C.RED}Conservative (10x p99): --threshold {conservative:.0f}{C.RESET}  (fewer false positives, misses weak targets)")
    print()
    print(f"Run with:  python zone_classifier.py --port {reader.ser.port} --threshold {moderate:.0f} --verbose")


# --------------- Main ---------------

def main():
    global MIN_MAGNITUDE

    parser = argparse.ArgumentParser(description='HB100 Amplitude-Monopulse Zone Classifier')
    parser.add_argument('--port', default='/dev/ttyACM0',
                        help='Serial port for ESP32-S3')
    parser.add_argument('--baud', type=int, default=921600,
                        help='Serial baud rate')
    parser.add_argument('--threshold', type=float, default=MIN_MAGNITUDE,
                        help='FFT magnitude threshold for detection')
    parser.add_argument('--calibrate', action='store_true',
                        help='Run noise floor calibration (no moving targets!)')
    parser.add_argument('--cal-blocks', type=int, default=100,
                        help='Number of blocks for calibration')
    parser.add_argument('--verbose', action='store_true',
                        help='Full diagnostic display (clears screen each block)')
    parser.add_argument('--log', default=None,
                        help='CSV file to log all measurements')
    parser.add_argument('--zone-window', type=int, default=5,
                        help='Zone-smoothing majority-vote window size')
    args = parser.parse_args()

    print(f"Connecting to ESP32-S3 on {args.port}...")
    reader = BlockReader(args.port, args.baud)

    MIN_MAGNITUDE = args.threshold

    # Load gain calibration if available
    gain_factors = None
    if os.path.exists('gain_calibration.json'):
        try:
            with open('gain_calibration.json', 'r') as f:
                cal_data = json.load(f)
                gain_factors = cal_data.get('gain_factors')
                print(f"{C.GREEN}Loaded gain calibration from gain_calibration.json{C.RESET}")
        except Exception as e:
            print(f"{C.YELLOW}Warning: could not load gain_calibration.json: {e}{C.RESET}")

    dsp = DSPPipeline(magnitude_threshold=args.threshold, gain_factors=gain_factors)

    if args.calibrate:
        run_calibration(reader, dsp, args.cal_blocks)
        reader.close()
        return

    dt = SAMPLES_PER_BLOCK / SAMPLE_RATE
    speed_tracker = SpeedTracker(dt=dt)
    zone_tracker = ZoneTracker(window_size=args.zone_window)

    log_file = None
    if args.log:
        log_file = open(args.log, 'w')
        log_file.write("timestamp,block,doppler_hz,speed_mps,snr_db,"
                       "ch0_mag,ch1_mag,ch2_mag,ch3_mag,"
                       "bias,centroid_bias,zone,smoothed_zone,"
                       "tracked_speed,acceleration\n")

    block_count = 0
    detect_count = 0
    start_time = time.time()
    last_detection_result = None

    print(f"Listening... threshold={args.threshold:.1f}  verbose={'ON' if args.verbose else 'OFF'}")
    if not args.verbose:
        print(f"{'Block':>6} | {'Doppler':>8} | {'Speed':>9} | {'Bias':>6} | "
              f"{'Zone':>10} | {'SNR':>6} | {'Tracked':>9} | Arrow")
        print("-" * 100)

    try:
        while True:
            block = reader.read_block()
            if block is None:
                continue

            block_count += 1
            result = dsp.process_block(block)
            elapsed = time.time() - start_time

            if not result['detected']:
                if speed_tracker.initialized:
                    speed_tracker.predict()
                if args.verbose and block_count % 5 == 0:
                    clear_screen()
                    print_header(block_count, detect_count, elapsed, reader)
                    print_raw_signals(result)
                    print_filtered_signals(result)
                    print_fft_and_doppler(result)
                    if last_detection_result:
                        print(f"{C.CYAN}|{C.RESET}")
                        print(f"{C.CYAN}|{C.RESET} {C.DIM}(showing last detection data){C.RESET}")
                        print_channel_mags(last_detection_result)
                        print_zone_classification(last_detection_result, zone_tracker)
                    print_speed_tracker(speed_tracker.get_state())
                    print_footer()
                elif not args.verbose and block_count % 20 == 0:
                    print_no_detection_summary(result, block_count)
                continue

            detect_count += 1
            last_detection_result = result
            speed = result['speed_mps']
            doppler = result['doppler_hz']
            snr = result['combined_snr_db']

            speed_tracker.update(speed)
            zone_tracker.update(result['zone'])
            state = speed_tracker.get_state()
            smoothed_zone = zone_tracker.get_zone()

            if args.verbose:
                clear_screen()
                print_header(block_count, detect_count, elapsed, reader)
                print_raw_signals(result)
                print_filtered_signals(result)
                print_fft_and_doppler(result)
                print_channel_mags(result)
                print_zone_classification(result, zone_tracker)
                print_speed_tracker(state)
                print_footer()
            else:
                arrow = zone_arrow(smoothed_zone)
                zcolor = zone_color(smoothed_zone)
                print(f"{C.GREEN}{block_count:6d}{C.RESET} | "
                      f"{doppler:7.1f}Hz | "
                      f"{speed:5.2f} m/s | "
                      f"{result['bias']:+5.2f} | "
                      f"{zcolor}{smoothed_zone:>10}{C.RESET} | "
                      f"{snr:5.1f}dB | "
                      f"{state['speed_mps']:6.2f}m/s | "
                      f"{zcolor}{C.BOLD}{arrow}{C.RESET}")

            if log_file:
                ts = time.time()
                mags = result['channel_mags']
                log_file.write(
                    f"{ts:.3f},{block_count},{doppler:.2f},{speed:.3f},{snr:.2f},"
                    f"{mags[0]:.3f},{mags[1]:.3f},{mags[2]:.3f},{mags[3]:.3f},"
                    f"{result['bias']:.4f},{result['centroid']:.4f},"
                    f"{result['zone']},{smoothed_zone},"
                    f"{state['speed_mps']:.3f},{state['accel_mps2']:.3f}\n"
                )
                log_file.flush()

    except KeyboardInterrupt:
        print(f"\n\n{C.BOLD}Stopped.{C.RESET} {block_count} blocks processed, {detect_count} detections "
              f"({detect_count/max(block_count,1)*100:.1f}% detection rate)")
        if speed_tracker.initialized:
            state = speed_tracker.get_state()
            print(f"Final tracked speed: {state['speed_mps']:.2f} m/s  "
                  f"zone: {zone_tracker.get_zone()}")
    finally:
        reader.close()
        if log_file:
            log_file.close()


if __name__ == '__main__':
    main()
