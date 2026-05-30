"""
Signal processing utilities for HB100 radar simulation.
Reusable DSP functions (filtering, FFT, windowing).
"""

import numpy as np
from scipy.signal import butter, sosfilt


class DSPPipeline:
    """Bandpass filtering, FFT, peak detection utilities."""

    def __init__(self, sample_rate=10000, fft_size=1024):
        """
        Initialize DSP pipeline.

        Args:
            sample_rate (float): ADC sample rate in Hz
            fft_size (int): FFT size (should match block size)
        """
        self.sample_rate = sample_rate
        self.fft_size = fft_size
        self.window = np.hanning(fft_size)
        self.freqs = np.fft.fftfreq(fft_size, d=1.0 / sample_rate)

        # Bandpass filter: 10–800 Hz
        self.sos = butter(
            4,
            [10.0, 800.0],
            btype='bandpass',
            fs=sample_rate,
            output='sos'
        )

    def remove_dc(self, signal):
        """
        Remove DC offset per channel (mean subtraction).

        Args:
            signal (np.ndarray): Shape (num_samples, num_channels)

        Returns:
            np.ndarray: DC-removed signal, same shape
        """
        return signal - np.mean(signal, axis=0, keepdims=True)

    def bandpass_filter(self, signal):
        """
        Apply 4th-order Butterworth bandpass filter (10–800 Hz).

        Args:
            signal (np.ndarray): Shape (num_samples, num_channels)

        Returns:
            np.ndarray: Filtered signal, same shape
        """
        return np.array([sosfilt(self.sos, signal[:, ch])
                        for ch in range(signal.shape[1])]).T

    def compute_fft(self, signal):
        """
        Apply Hanning window and compute FFT.

        Args:
            signal (np.ndarray): Shape (num_samples, num_channels)

        Returns:
            np.ndarray: Complex FFT output, shape (num_freq_bins, num_channels)
        """
        windowed = signal * self.window[:, np.newaxis]
        # Use full FFT for complex signals, rfft for real signals
        if np.iscomplexobj(windowed):
            return np.fft.fft(windowed, axis=0)
        else:
            return np.fft.rfft(windowed, axis=0)

    def find_doppler_peak(self, fft_output):
        """
        Find peak Doppler bin from average magnitude spectrum.

        Args:
            fft_output (np.ndarray): Shape (num_freq_bins, num_channels)

        Returns:
            tuple: (peak_bin_index, peak_frequency_hz, average_magnitude)
        """
        avg_magnitude = np.mean(np.abs(fft_output), axis=1)
        # Exclude DC (bin 0)
        peak_bin = np.argmax(avg_magnitude[1:]) + 1
        peak_freq = self.freqs[peak_bin]
        peak_mag = avg_magnitude[peak_bin]
        return peak_bin, peak_freq, peak_mag

    def compute_snr(self, fft_output, peak_bin, noise_bins=50):
        """
        Estimate SNR from FFT magnitude spectrum.

        Args:
            fft_output (np.ndarray): Shape (num_freq_bins, num_channels)
            peak_bin (int): Index of Doppler peak
            noise_bins (int): Number of bins to average for noise floor

        Returns:
            float: SNR in dB
        """
        avg_mag = np.mean(np.abs(fft_output), axis=1)
        peak_power = avg_mag[peak_bin] ** 2

        # Noise floor from edges
        noise_power = np.mean(avg_mag[:noise_bins] ** 2)

        snr_db = 10 * np.log10(peak_power / (noise_power + 1e-10))
        return snr_db


# Physical constants
F_CARRIER = 10.525e9  # Hz
C_SPEED = 3.0e8  # m/s
WAVELENGTH = C_SPEED / F_CARRIER  # ~0.0285 m

# Element positions (meters)
ELEMENT_POSITIONS = np.array([0.0, 0.037, 0.075, 0.112])
NUM_CHANNELS = 4
