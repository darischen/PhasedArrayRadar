"""
HB100 Phased Array Radar - Host Processor
Runs on Raspberry Pi 4 (or any Linux host with Python 3.9+).

Reads raw ADC blocks from ESP32-S3 over USB serial.
Computes Doppler speed, angle of arrival (MUSIC), and tracks targets
with an Extended Kalman Filter.

Usage:
    pip install numpy scipy pyserial

    # Calibrate noise floor first (no moving targets):
    python radar_processor.py --port /dev/ttyACM0 --calibrate --cal-blocks 100

    # Run with tracking and diagnostics:
    python radar_processor.py --port /dev/ttyACM0 --threshold 150 --verbose

    # Log to CSV for post-analysis:
    python radar_processor.py --port /dev/ttyACM0 --threshold 150 --log data.csv

Hardware layout (viewed from behind the array, looking outward):
    ch0 (GPIO7)  |  ch1 (GPIO6)  |  ch2 (GPIO5)  |  ch3 (GPIO4)
    <-- 37mm -->  <-- 38mm -->    <-- 37mm -->
    farthest left                              closest / rightmost
"""

import argparse
import struct
import time
import sys
import os
from collections import deque
from datetime import datetime

import numpy as np
from scipy.signal import butter, sosfilt
import serial


# --------------- Physical Constants ---------------

F_CARRIER       = 10.525e9
C_SPEED         = 3.0e8
WAVELENGTH      = C_SPEED / F_CARRIER  # ~0.0285 m

# Element positions along array axis (meters), ch0 at origin
ELEMENT_POSITIONS = np.array([0.0, 0.037, 0.075, 0.112])

# Pairwise baselines for diagnostic display
BASELINES = [
    (0, 1, 0.037),
    (1, 2, 0.038),
    (2, 3, 0.037),
    (0, 2, 0.075),
    (0, 3, 0.112),
    (1, 3, 0.075),
]

NUM_CHANNELS    = 4
SAMPLES_PER_BLOCK = 1024
SAMPLE_RATE     = 10000
ADC_MAX         = 4095
ADC_VREF        = 3.3

# DSP parameters
FFT_SIZE        = SAMPLES_PER_BLOCK
BANDPASS_LOW    = 10.0
BANDPASS_HIGH   = 800.0   # Matches your 2k + 100nF hardware LPF rolloff
MIN_MAGNITUDE   = 50.0    # Placeholder, use --calibrate to find real value

# MUSIC parameters
MUSIC_ANGLE_RANGE = np.linspace(-80, 80, 321)
MUSIC_SMOOTHING   = 5

# Kalman filter parameters
KF_PROCESS_NOISE  = 0.5
KF_SPEED_NOISE    = 0.3
KF_ANGLE_NOISE    = np.radians(5)

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
    BG_RED  = '\033[41m'
    BG_GREEN = '\033[42m'

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

class DSPPipeline:

    def __init__(self, magnitude_threshold: float = MIN_MAGNITUDE):
        self.magnitude_threshold = magnitude_threshold

        self.sos = butter(
            4,
            [BANDPASS_LOW, BANDPASS_HIGH],
            btype='bandpass',
            fs=SAMPLE_RATE,
            output='sos'
        )

        self.window = np.hanning(SAMPLES_PER_BLOCK)
        self.freqs = np.fft.rfftfreq(FFT_SIZE, d=1.0 / SAMPLE_RATE)

        angles_rad = np.radians(MUSIC_ANGLE_RANGE)
        self.steering_vectors = np.exp(
            1j * 2 * np.pi * np.outer(np.sin(angles_rad), ELEMENT_POSITIONS) / WAVELENGTH
        )

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
            'channel_phases': None,
            'phase_diffs': None,
            'music_spectrum': None,
            'music_peak_idx': None,
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
                'mean_mag': np.mean(ch_mag[1:]),
            }

        # Combined peak detection
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

        # Top 5 frequency peaks
        sorted_bins = np.argsort(avg_magnitude[1:])[::-1] + 1
        result['top5_peaks'] = [
            (self.freqs[b], avg_magnitude[b]) for b in sorted_bins[:5]
        ]

        # Doppler speed from peak
        doppler_freq = self.freqs[peak_bin]
        speed_mps = (doppler_freq * C_SPEED) / (2.0 * F_CARRIER)
        result['doppler_hz'] = doppler_freq
        result['speed_mps'] = speed_mps

        if peak_mag < self.magnitude_threshold:
            return result

        result['detected'] = True

        # 4. Phase extraction at peak bin
        peak_complex = spectra[peak_bin, :]
        phases = np.angle(peak_complex)
        result['channel_phases'] = phases

        # Phase differences for all baselines
        phase_diffs = {}
        for i, j, d in BASELINES:
            dp = phases[i] - phases[j]
            dp = (dp + np.pi) % (2 * np.pi) - np.pi
            sin_val = (dp * WAVELENGTH) / (2 * np.pi * d)
            sin_val_clipped = np.clip(sin_val, -1.0, 1.0)
            angle_deg = np.degrees(np.arcsin(sin_val_clipped))
            phase_diffs[(i, j)] = {
                'delta_phi_rad': dp,
                'delta_phi_deg': np.degrees(dp),
                'sin_theta': sin_val,
                'angle_deg': angle_deg,
                'aliased': abs(sin_val) > 1.0,
            }
        result['phase_diffs'] = phase_diffs

        # 5. MUSIC angle estimation
        angle_deg, pseudospectrum = self._music_angle(filtered, peak_bin)
        result['angle_deg'] = angle_deg
        result['music_spectrum'] = pseudospectrum
        result['music_peak_idx'] = np.argmax(pseudospectrum)

        result['peak_complex'] = peak_complex
        result['snr_db'] = result['combined_snr_db']

        return result

    def _music_angle(self, filtered_data, peak_bin):
        bin_start = max(1, peak_bin - MUSIC_SMOOTHING)
        bin_end = min(len(self.freqs) - 1, peak_bin + MUSIC_SMOOTHING)

        spectra = np.fft.rfft(filtered_data * self.window[:, np.newaxis],
                              n=FFT_SIZE, axis=0)

        R = np.zeros((NUM_CHANNELS, NUM_CHANNELS), dtype=complex)
        count = 0
        for b in range(bin_start, bin_end + 1):
            x = spectra[b, :].reshape(-1, 1)
            R += x @ x.conj().T
            count += 1
        R /= count

        J = np.fliplr(np.eye(NUM_CHANNELS))
        R = 0.5 * (R + J @ R.conj() @ J)

        eigenvalues, eigenvectors = np.linalg.eigh(R)
        idx = np.argsort(eigenvalues)[::-1]
        eigenvalues = eigenvalues[idx]
        eigenvectors = eigenvectors[:, idx]

        noise_subspace = eigenvectors[:, 1:]

        pseudospectrum = np.zeros(len(MUSIC_ANGLE_RANGE))
        for i, sv in enumerate(self.steering_vectors):
            sv_col = sv.reshape(-1, 1)
            proj = noise_subspace.conj().T @ sv_col
            denom = np.real(np.sum(np.abs(proj) ** 2))
            pseudospectrum[i] = 1.0 / max(denom, 1e-20)

        peak_idx = np.argmax(pseudospectrum)
        return MUSIC_ANGLE_RANGE[peak_idx], pseudospectrum


# --------------- Extended Kalman Filter ---------------

class RadarEKF:
    def __init__(self, dt: float = 0.1):
        self.dt = dt
        self.x = np.array([5.0, 0.0, 0.0, 0.0])
        self.P = np.diag([10.0, 10.0, 5.0, 5.0])

        q = KF_PROCESS_NOISE
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt
        self.Q = q**2 * np.array([
            [dt4/4, 0,     dt3/2, 0    ],
            [0,     dt4/4, 0,     dt3/2],
            [dt3/2, 0,     dt2,   0    ],
            [0,     dt3/2, 0,     dt2  ],
        ])

        self.R = np.diag([KF_SPEED_NOISE**2, KF_ANGLE_NOISE**2])
        self.initialized = False
        self.innovation_history = deque(maxlen=50)

    def predict(self):
        F = np.array([
            [1, 0, self.dt, 0      ],
            [0, 1, 0,       self.dt],
            [0, 0, 1,       0      ],
            [0, 0, 0,       1      ],
        ])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update(self, speed_mps: float, angle_deg: float):
        angle_rad = np.radians(angle_deg)

        if not self.initialized:
            assumed_range = 5.0
            self.x[0] = assumed_range * np.cos(angle_rad)
            self.x[1] = assumed_range * np.sin(angle_rad)
            self.x[2] = -speed_mps * np.cos(angle_rad)
            self.x[3] = speed_mps * np.sin(angle_rad)
            self.initialized = True
            return

        self.predict()

        x, y, vx, vy = self.x
        r = np.sqrt(x**2 + y**2)
        if r < 0.01:
            r = 0.01

        pred_angle = np.arctan2(y, x)
        radial_v = (vx * x + vy * y) / r
        pred_speed = abs(radial_v)

        z = np.array([speed_mps, angle_rad])
        z_pred = np.array([pred_speed, pred_angle])
        innovation = z - z_pred
        innovation[1] = (innovation[1] + np.pi) % (2 * np.pi) - np.pi

        self.innovation_history.append(innovation.copy())

        sign_rv = np.sign(radial_v) if abs(radial_v) > 1e-6 else 1.0
        H = np.zeros((2, 4))
        H[0, 0] = sign_rv * (vx * r**2 - x * (vx*x + vy*y)) / r**3
        H[0, 1] = sign_rv * (vy * r**2 - y * (vx*x + vy*y)) / r**3
        H[0, 2] = sign_rv * x / r
        H[0, 3] = sign_rv * y / r
        H[1, 0] = -y / r**2
        H[1, 1] = x / r**2

        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ innovation
        I4 = np.eye(4)
        self.P = (I4 - K @ H) @ self.P @ (I4 - K @ H).T + K @ self.R @ K.T

    def get_state(self) -> dict:
        x, y, vx, vy = self.x
        speed = np.sqrt(vx**2 + vy**2)
        heading = np.degrees(np.arctan2(vy, vx))
        pos_uncertainty = np.sqrt(self.P[0, 0] + self.P[1, 1])
        vel_uncertainty = np.sqrt(self.P[2, 2] + self.P[3, 3])
        return {
            'x_m': x, 'y_m': y,
            'vx_mps': vx, 'vy_mps': vy,
            'speed_mps': speed,
            'heading_deg': heading,
            'pos_uncertainty_m': pos_uncertainty,
            'vel_uncertainty_mps': vel_uncertainty,
        }


# --------------- Display Functions ---------------

def clear_screen():
    os.system('clear' if os.name == 'posix' else 'cls')


def print_header(block_count, detect_count, elapsed, reader):
    rate = block_count / max(elapsed, 0.01)
    bandwidth = reader.bytes_received / max(elapsed, 0.01) / 1024
    print(f"{C.BOLD}{C.CYAN}+==============================================================================+{C.RESET}")
    print(f"{C.BOLD}{C.CYAN}|  HB100 PHASED ARRAY RADAR{' ' * 51}|{C.RESET}")
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
    print(f"{C.CYAN}|{C.RESET}   {'Chan':>4}  {'Mean(V)':>8}  {'Std(V)':>8}  {'P-P(V)':>8}  {'Min(V)':>8}  {'Max(V)':>8}  Signal Level")
    for ch in range(NUM_CHANNELS):
        s = result['raw_stats'][ch]
        bar = C.bar(s['pp_v'], 3.3, width=20)
        color = C.GREEN if s['pp_v'] > 0.01 else C.DIM
        print(f"{C.CYAN}|{C.RESET}   {color}ch{ch}   "
              f"{s['mean_v']:8.4f}  {s['std_v']:8.5f}  {s['pp_v']:8.5f}  "
              f"{s['min_v']:8.4f}  {s['max_v']:8.4f}{C.RESET}  {bar}")


def print_filtered_signals(result):
    print(f"{C.CYAN}|{C.RESET}")
    print(f"{C.CYAN}|{C.RESET} {C.BOLD}FILTERED SIGNALS (bandpass {BANDPASS_LOW}-{BANDPASS_HIGH} Hz){C.RESET}")
    print(f"{C.CYAN}|{C.RESET}   {'Chan':>4}  {'RMS(V)':>10}  {'P-P(V)':>10}  {'Std(V)':>10}  Activity")
    max_rms = max(result['filtered_stats'][ch]['rms_v'] for ch in range(NUM_CHANNELS))
    for ch in range(NUM_CHANNELS):
        s = result['filtered_stats'][ch]
        bar = C.bar(s['rms_v'], max(max_rms * 1.2, 0.001), width=25)
        color = C.GREEN if s['rms_v'] > 0.001 else C.DIM
        print(f"{C.CYAN}|{C.RESET}   {color}ch{ch}   "
              f"{s['rms_v']:10.6f}  {s['pp_v']:10.6f}  {s['std_v']:10.6f}{C.RESET}  {bar}")


def print_fft_analysis(result):
    print(f"{C.CYAN}|{C.RESET}")
    print(f"{C.CYAN}|{C.RESET} {C.BOLD}FFT ANALYSIS{C.RESET}")
    print(f"{C.CYAN}|{C.RESET}   {'Chan':>4}  {'Peak Bin':>8}  {'Peak Hz':>8}  {'Peak Mag':>9}  {'Noise Flr':>9}  {'Ch SNR':>8}")
    for ch in range(NUM_CHANNELS):
        s = result['fft_stats'][ch]
        snr = 20 * np.log10(s['peak_mag'] / max(s['noise_floor'], 1e-10))
        color = C.GREEN if snr > 10 else (C.YELLOW if snr > 5 else C.DIM)
        print(f"{C.CYAN}|{C.RESET}   {color}ch{ch}   "
              f"{s['peak_bin']:8d}  {s['peak_freq']:7.1f}Hz  {s['peak_mag']:9.2f}  "
              f"{s['noise_floor']:9.2f}  {snr:7.1f}dB{C.RESET}")

    print(f"{C.CYAN}|{C.RESET}")
    print(f"{C.CYAN}|{C.RESET}   {C.BOLD}Combined:{C.RESET}  peak bin={result['combined_peak_bin']}  "
          f"freq={result['combined_peak_freq']:.1f}Hz  "
          f"mag={result['combined_peak_mag']:.2f}  "
          f"noise={result['combined_noise_floor']:.2f} +/- {result['combined_noise_std']:.2f}  "
          f"SNR={result['combined_snr_db']:.1f}dB")

    # Top 5 peaks
    print(f"{C.CYAN}|{C.RESET}   {C.DIM}Top 5 peaks: ", end="")
    for i, (freq, mag) in enumerate(result['top5_peaks']):
        speed = (freq * C_SPEED) / (2.0 * F_CARRIER)
        print(f"{freq:.0f}Hz({speed:.1f}m/s, mag={mag:.0f})", end="  ")
    print(f"{C.RESET}")


def print_doppler(result):
    print(f"{C.CYAN}|{C.RESET}")
    thresh_status = f"{C.GREEN}ABOVE{C.RESET}" if result['detected'] else f"{C.RED}BELOW{C.RESET}"
    print(f"{C.CYAN}|{C.RESET} {C.BOLD}DOPPLER{C.RESET}  "
          f"freq={result['doppler_hz']:.1f}Hz  "
          f"speed={C.BOLD}{result['speed_mps']:.3f} m/s{C.RESET} "
          f"({result['speed_mps'] * 3.6:.1f} km/h, {result['speed_mps'] * 2.237:.1f} mph)  "
          f"threshold: {thresh_status} ({result['combined_peak_mag']:.0f} vs {MIN_MAGNITUDE:.0f})")


def print_phase_analysis(result):
    if result['channel_phases'] is None:
        return

    print(f"{C.CYAN}|{C.RESET}")
    print(f"{C.CYAN}|{C.RESET} {C.BOLD}PHASE EXTRACTION @ {result['doppler_hz']:.1f} Hz{C.RESET}")
    print(f"{C.CYAN}|{C.RESET}   Channel phases: ", end="")
    for ch in range(NUM_CHANNELS):
        p = result['channel_phases'][ch]
        print(f"ch{ch}={np.degrees(p):+7.2f}deg  ", end="")
    print()

    print(f"{C.CYAN}|{C.RESET}")
    print(f"{C.CYAN}|{C.RESET} {C.BOLD}PAIRWISE PHASE DIFFERENCES & NAIVE ANGLE ESTIMATES{C.RESET}")
    print(f"{C.CYAN}|{C.RESET}   {'Pair':>6}  {'Baseline':>8}  {'d_phi(rad)':>10}  {'d_phi(deg)':>10}  {'sin(th)':>8}  {'Angle':>8}  {'Status':>8}")
    for (i, j, d) in BASELINES:
        pd = result['phase_diffs'][(i, j)]
        aliased_flag = f"{C.RED}ALIASED{C.RESET}" if pd['aliased'] else f"{C.GREEN}OK{C.RESET}"
        color = C.RED if pd['aliased'] else C.WHITE
        print(f"{C.CYAN}|{C.RESET}   {color}{i}-{j}    "
              f"{d*1000:6.0f}mm  "
              f"{pd['delta_phi_rad']:+10.3f}  "
              f"{pd['delta_phi_deg']:+10.2f}  "
              f"{pd['sin_theta']:+8.4f}  "
              f"{pd['angle_deg']:+7.2f}deg{C.RESET}  {aliased_flag}")


def print_music(result):
    if result['music_spectrum'] is None:
        return

    print(f"{C.CYAN}|{C.RESET}")
    print(f"{C.CYAN}|{C.RESET} {C.BOLD}MUSIC PSEUDOSPECTRUM{C.RESET}  "
          f"Estimated angle: {C.BOLD}{C.GREEN}{result['angle_deg']:+.1f}deg{C.RESET}")

    spectrum = result['music_spectrum']
    spectrum_db = 10 * np.log10(spectrum / max(spectrum.max(), 1e-20))

    plot_width = 60
    db_range = 30

    indices = np.linspace(0, len(spectrum) - 1, plot_width).astype(int)
    plot_data = spectrum_db[indices]
    plot_max = plot_data.max()
    plot_data_norm = np.clip((plot_data - (plot_max - db_range)) / db_range, 0, 1)

    chars = " _.,:-=+*#@"
    print(f"{C.CYAN}|{C.RESET}   {C.DIM}{plot_max:.0f}dB{C.RESET} ", end="")
    for val in plot_data_norm:
        char_idx = int(val * (len(chars) - 1))
        if val > 0.8:
            print(f"{C.GREEN}{chars[char_idx]}{C.RESET}", end="")
        elif val > 0.4:
            print(f"{C.YELLOW}{chars[char_idx]}{C.RESET}", end="")
        else:
            print(f"{C.DIM}{chars[char_idx]}{C.RESET}", end="")
    print()
    print(f"{C.CYAN}|{C.RESET}         {C.DIM}-80deg{' ' * 18}0deg{' ' * 19}+80deg{C.RESET}")

    # Find and display all peaks above -15dB from maximum (grating lobes)
    from scipy.signal import find_peaks as _fp
    peaks, props = _fp(spectrum_db, height=plot_max - 15, distance=5)
    if len(peaks) > 1:
        print(f"{C.CYAN}|{C.RESET}   {C.YELLOW}Multiple peaks (possible grating lobes):{C.RESET}")
        for p in peaks:
            angle = MUSIC_ANGLE_RANGE[p]
            db = spectrum_db[p]
            marker = " << PRIMARY" if p == result['music_peak_idx'] else ""
            print(f"{C.CYAN}|{C.RESET}     {angle:+6.1f}deg at {db:.1f}dB{marker}")


def print_kalman(state, ekf):
    print(f"{C.CYAN}|{C.RESET}")
    print(f"{C.CYAN}|{C.RESET} {C.BOLD}KALMAN FILTER STATE{C.RESET}")
    print(f"{C.CYAN}|{C.RESET}   Position:  x={state['x_m']:+7.2f}m  y={state['y_m']:+7.2f}m  "
          f"(uncertainty: +/-{state['pos_uncertainty_m']:.2f}m)")
    print(f"{C.CYAN}|{C.RESET}   Velocity:  vx={state['vx_mps']:+6.3f}m/s  vy={state['vy_mps']:+6.3f}m/s  "
          f"(uncertainty: +/-{state['vel_uncertainty_mps']:.3f}m/s)")
    print(f"{C.CYAN}|{C.RESET}   Tracking:  speed={C.BOLD}{state['speed_mps']:.2f}m/s{C.RESET}  "
          f"heading={C.BOLD}{state['heading_deg']:+.1f}deg{C.RESET}")

    heading = state['heading_deg']
    compass = ""
    if -22.5 <= heading < 22.5:
        compass = "-> MOVING RIGHT"
    elif 22.5 <= heading < 67.5:
        compass = "/^ MOVING RIGHT+AWAY"
    elif 67.5 <= heading < 112.5:
        compass = "^  MOVING AWAY"
    elif 112.5 <= heading < 157.5:
        compass = "^\\ MOVING LEFT+AWAY"
    elif heading >= 157.5 or heading < -157.5:
        compass = "<- MOVING LEFT"
    elif -157.5 <= heading < -112.5:
        compass = "\\v MOVING LEFT+TOWARD"
    elif -112.5 <= heading < -67.5:
        compass = "v  MOVING TOWARD"
    elif -67.5 <= heading < -22.5:
        compass = "v/ MOVING RIGHT+TOWARD"
    print(f"{C.CYAN}|{C.RESET}   Direction: {C.BOLD}{C.MAGENTA}{compass}{C.RESET}")

    if len(ekf.innovation_history) > 5:
        innovations = np.array(list(ekf.innovation_history))
        speed_innov_std = np.std(innovations[:, 0])
        angle_innov_std = np.std(innovations[:, 1])
        speed_health = "OK" if speed_innov_std < KF_SPEED_NOISE * 3 else "DIVERGING"
        angle_health = "OK" if angle_innov_std < KF_ANGLE_NOISE * 3 else "DIVERGING"
        sc = C.GREEN if speed_health == "OK" else C.RED
        ac = C.GREEN if angle_health == "OK" else C.RED
        print(f"{C.CYAN}|{C.RESET}   Health:    speed innov std={speed_innov_std:.3f} [{sc}{speed_health}{C.RESET}]  "
              f"angle innov std={np.degrees(angle_innov_std):.2f}deg [{ac}{angle_health}{C.RESET}]")


def print_footer():
    print(f"{C.CYAN}+==============================================================================+{C.RESET}")


def print_no_detection_summary(result, block_count):
    """Compact single-line output when no target is detected."""
    noise = result['combined_noise_floor']
    peak = result['combined_peak_mag']
    max_rms = max(result['filtered_stats'][ch]['rms_v'] for ch in range(NUM_CHANNELS))
    print(f"{C.DIM}[{block_count:6d}] No target  "
          f"peak={peak:.1f}  noise={noise:.1f}  "
          f"max_rms={max_rms:.6f}V{C.RESET}")


# --------------- Calibration Mode ---------------

def run_calibration(reader, dsp, num_blocks):
    print(f"\n{C.BOLD}{C.YELLOW}CALIBRATION MODE{C.RESET}")
    print(f"Collecting {num_blocks} blocks with NO moving targets present...")
    print(f"Keep the area in front of the array clear.\n")

    all_magnitudes = []
    block_count = 0

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
        avg_mag = np.abs(spectra).mean(axis=1)
        all_magnitudes.extend(avg_mag[1:].tolist())

        pct = block_count / num_blocks * 100
        bar = C.bar(block_count, num_blocks, width=40)
        print(f"\r  {bar} {pct:5.1f}%  ({block_count}/{num_blocks})", end="", flush=True)

    print("\n")

    mags = np.array(all_magnitudes)
    mean_mag = np.mean(mags)
    std_mag = np.std(mags)
    median_mag = np.median(mags)
    p95 = np.percentile(mags, 95)
    p99 = np.percentile(mags, 99)
    max_mag = np.max(mags)

    print(f"{C.BOLD}NOISE FLOOR STATISTICS{C.RESET}")
    print(f"  Samples analyzed: {len(mags)}")
    print(f"  Mean magnitude:   {mean_mag:.2f}")
    print(f"  Std deviation:    {std_mag:.2f}")
    print(f"  Median:           {median_mag:.2f}")
    print(f"  95th percentile:  {p95:.2f}")
    print(f"  99th percentile:  {p99:.2f}")
    print(f"  Maximum:          {max_mag:.2f}")
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
    print(f"Run with:  python radar_processor.py --port {reader.ser.port} --threshold {moderate:.0f} --verbose")


# --------------- Main ---------------

def main():
    parser = argparse.ArgumentParser(description='HB100 Phased Array Radar Processor')
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
    args = parser.parse_args()

    print(f"Connecting to ESP32-S3 on {args.port}...")
    reader = BlockReader(args.port, args.baud)
    dsp = DSPPipeline(magnitude_threshold=args.threshold)

    global MIN_MAGNITUDE
    MIN_MAGNITUDE = args.threshold

    if args.calibrate:
        run_calibration(reader, dsp, args.cal_blocks)
        reader.close()
        return

    ekf = RadarEKF(dt=SAMPLES_PER_BLOCK / SAMPLE_RATE)

    log_file = None
    if args.log:
        log_file = open(args.log, 'w')
        log_file.write("timestamp,block,doppler_hz,speed_mps,angle_deg,snr_db,"
                       "ch0_phase,ch1_phase,ch2_phase,ch3_phase,"
                       "ekf_x,ekf_y,ekf_vx,ekf_vy,ekf_speed,ekf_heading,"
                       "pos_uncertainty,vel_uncertainty\n")

    block_count = 0
    detect_count = 0
    start_time = time.time()
    last_detection_result = None

    print(f"Listening... threshold={args.threshold:.0f}  verbose={'ON' if args.verbose else 'OFF'}")
    if not args.verbose:
        print(f"{'Block':>6} | {'Doppler':>8} | {'Speed':>9} | {'Angle':>7} | "
              f"{'SNR':>6} | {'EKF Speed':>9} | {'EKF Heading':>11} | {'Direction':>20}")
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
                if ekf.initialized:
                    ekf.predict()
                if args.verbose and block_count % 5 == 0:
                    clear_screen()
                    print_header(block_count, detect_count, elapsed, reader)
                    print_raw_signals(result)
                    print_filtered_signals(result)
                    print_fft_analysis(result)
                    print_doppler(result)
                    if last_detection_result:
                        print(f"{C.CYAN}|{C.RESET}")
                        print(f"{C.CYAN}|{C.RESET} {C.DIM}(showing last detection data){C.RESET}")
                        print_phase_analysis(last_detection_result)
                        print_music(last_detection_result)
                        if ekf.initialized:
                            print_kalman(ekf.get_state(), ekf)
                    print_footer()
                elif not args.verbose and block_count % 20 == 0:
                    print_no_detection_summary(result, block_count)
                continue

            detect_count += 1
            last_detection_result = result
            speed = result['speed_mps']
            angle = result['angle_deg']
            doppler = result['doppler_hz']
            snr = result['snr_db']

            ekf.update(speed, angle)
            state = ekf.get_state()

            if args.verbose:
                clear_screen()
                print_header(block_count, detect_count, elapsed, reader)
                print_raw_signals(result)
                print_filtered_signals(result)
                print_fft_analysis(result)
                print_doppler(result)
                print_phase_analysis(result)
                print_music(result)
                print_kalman(state, ekf)
                print_footer()
            else:
                heading = state['heading_deg']
                if -22.5 <= heading < 22.5:
                    direction = "-> RIGHT"
                elif 22.5 <= heading < 67.5:
                    direction = "/^ RIGHT+AWAY"
                elif 67.5 <= heading < 112.5:
                    direction = "^  AWAY"
                elif 112.5 <= heading < 157.5:
                    direction = "^\\ LEFT+AWAY"
                elif heading >= 157.5 or heading < -157.5:
                    direction = "<- LEFT"
                elif -157.5 <= heading < -112.5:
                    direction = "\\v LEFT+TOWARD"
                elif -112.5 <= heading < -67.5:
                    direction = "v  TOWARD"
                else:
                    direction = "v/ RIGHT+TOWARD"

                print(f"{C.GREEN}{block_count:6d}{C.RESET} | "
                      f"{doppler:7.1f}Hz | "
                      f"{speed:5.2f} m/s | "
                      f"{angle:+6.1f}deg | "
                      f"{snr:5.1f}dB | "
                      f"{state['speed_mps']:7.2f}m/s | "
                      f"{state['heading_deg']:+8.1f}deg | "
                      f"{C.MAGENTA}{direction:>20}{C.RESET}")

            if log_file:
                ts = time.time()
                phases = result['channel_phases']
                if phases is None:
                    phases = [0, 0, 0, 0]
                log_file.write(
                    f"{ts:.3f},{block_count},{doppler:.2f},{speed:.3f},"
                    f"{angle:.2f},{snr:.2f},"
                    f"{phases[0]:.4f},{phases[1]:.4f},{phases[2]:.4f},{phases[3]:.4f},"
                    f"{state['x_m']:.3f},{state['y_m']:.3f},"
                    f"{state['vx_mps']:.3f},{state['vy_mps']:.3f},"
                    f"{state['speed_mps']:.3f},{state['heading_deg']:.2f},"
                    f"{state['pos_uncertainty_m']:.3f},{state['vel_uncertainty_mps']:.3f}\n"
                )
                log_file.flush()

    except KeyboardInterrupt:
        print(f"\n\n{C.BOLD}Stopped.{C.RESET} {block_count} blocks processed, {detect_count} detections "
              f"({detect_count/max(block_count,1)*100:.1f}% detection rate)")
        if ekf.initialized:
            state = ekf.get_state()
            print(f"Final state: x={state['x_m']:.2f}m y={state['y_m']:.2f}m "
                  f"speed={state['speed_mps']:.2f}m/s heading={state['heading_deg']:.1f}deg")
    finally:
        reader.close()
        if log_file:
            log_file.close()


if __name__ == '__main__':
    main()
