"""
Synthetic radar signal generation and processing pipeline.

Generates 4-channel complex Doppler signals with configurable target angles, SNR,
and oscillator drift. Runs MUSIC and monopulse processing pipelines.
"""

import numpy as np
from dsp_utils import DSPPipeline, WAVELENGTH, ELEMENT_POSITIONS
from music_algorithm import MUSICProcessor
from kalman_filter import ExtendedKalmanFilter


class SyntheticRadarSimulator:
    """Generate synthetic signals and run processing pipelines."""

    def __init__(self, config):
        """
        Initialize simulator.

        Args:
            config (dict): Configuration dict with simulation parameters
        """
        self.config = config

        # DSP
        self.dsp = DSPPipeline(
            sample_rate=config.get('sample_rate', 10000),
            fft_size=config.get('fft_size', 1024)
        )

        # MUSIC
        self.music = MUSICProcessor(
            wavelength=WAVELENGTH,
            element_positions=ELEMENT_POSITIONS
        )

        # Kalman filter
        self.ekf = ExtendedKalmanFilter(dt=0.1024)  # ~102.4 ms per block

        # Precompute steering vectors for MUSIC angle search
        angle_range = config.get('music_angle_range', (-80, 80))
        num_angles = config.get('music_num_angles', 321)
        self.music_angles, self.steering_vectors = self.music.compute_steering_vectors(
            angle_range, num_angles
        )

    def generate_synthetic_signals(self, target_angles_deg, snr_db, num_samples=1024,
                                    oscillator_drift_hz=0.0, element_spacing_error_mm=0.0):
        """
        Generate 4-channel synthetic Doppler signals with AWGN.

        Args:
            target_angles_deg (list): Target angles in degrees
            snr_db (float): Signal-to-noise ratio in dB
            num_samples (int): Number of samples per block
            oscillator_drift_hz (float): Free-running LO drift magnitude
            element_spacing_error_mm (float): Systematic element spacing error

        Returns:
            np.ndarray: Complex baseband signals, shape (num_samples, 4)
        """
        num_channels = 4

        # Signal power
        signal_power = 10 ** (snr_db / 10)
        noise_power = 1.0  # Normalized

        # Oscillator phase drift per channel (random initial phase + slow drift)
        phase_drifts = np.random.uniform(0, 2*np.pi, num_channels)
        drift_rate = oscillator_drift_hz * 2 * np.pi / self.config.get('sample_rate', 10000)

        # Adjusted element positions with spacing error
        element_pos = ELEMENT_POSITIONS.copy()
        if element_spacing_error_mm != 0:
            spacing_error = element_spacing_error_mm / 1000.0  # Convert to meters
            element_pos += np.random.normal(0, abs(spacing_error), num_channels)

        # Generate signals
        signals = np.zeros((num_samples, num_channels), dtype=complex)

        for target_angle_deg in target_angles_deg:
            target_angle_rad = np.radians(target_angle_deg)

            # Steering vector at target angle
            a = np.exp(1j * 2 * np.pi * np.sin(target_angle_rad) * element_pos / WAVELENGTH)

            # Generate complex baseband signal at all channels
            for n in range(num_samples):
                # Phase delay due to steering
                phase_offset = 2 * np.pi * np.sin(target_angle_rad) * element_pos / WAVELENGTH

                # Oscillator drift per channel
                drift_phase = phase_drifts + drift_rate * n

                # Complex baseband signal (normalized)
                signal_value = np.sqrt(signal_power) * np.exp(1j * (phase_offset + drift_phase))
                signals[n, :] += signal_value

        # Add AWGN
        noise = np.random.normal(0, np.sqrt(noise_power / 2), (num_samples, num_channels)) + \
                1j * np.random.normal(0, np.sqrt(noise_power / 2), (num_samples, num_channels))
        signals += noise

        return signals

    def process_music_pipeline(self, voltages):
        """
        Run MUSIC angle-of-arrival pipeline.

        Args:
            voltages (np.ndarray): Complex signals, shape (num_samples, num_channels)

        Returns:
            dict: Results with 'angle_deg', 'pseudospectrum', 'fft_output', 'snr_db'
        """
        # DC removal
        filtered = self.dsp.remove_dc(voltages)

        # Bandpass filtering
        filtered = self.dsp.bandpass_filter(filtered)

        # FFT
        fft_output = self.dsp.compute_fft(filtered)

        # Peak detection
        peak_bin, peak_freq, peak_mag = self.dsp.find_doppler_peak(fft_output)

        # SNR
        snr_db = self.dsp.compute_snr(fft_output, peak_bin)

        # MUSIC angle estimation
        angle_deg, pseudospectrum, angles_deg = self.music.estimate_angle(
            fft_output, peak_bin,
            angle_range_deg=tuple(self.config.get('music_angle_range', (-80, 80))),
            num_angles=self.config.get('music_num_angles', 321),
            smoothing_bins=self.config.get('music_smoothing_bins', 5)
        )

        # Speed estimation
        doppler_freq = peak_freq
        speed_mps = (doppler_freq * 3.0e8) / (2 * 10.525e9)

        return {
            'angle_deg': angle_deg,
            'speed_mps': speed_mps,
            'snr_db': snr_db,
            'pseudospectrum': pseudospectrum,
            'angles_deg': angles_deg,
            'fft_output': fft_output,
            'peak_bin': peak_bin,
            'peak_freq': doppler_freq
        }

    def process_monopulse_pipeline(self, voltages):
        """
        Run monopulse (amplitude-only) baseline pipeline.

        Args:
            voltages (np.ndarray): Complex signals, shape (num_samples, num_channels)

        Returns:
            dict: Results with 'angle_bias', 'snr_db', 'fft_output'
        """
        # DC removal
        filtered = self.dsp.remove_dc(voltages)

        # Bandpass filtering
        filtered = self.dsp.bandpass_filter(filtered)

        # FFT
        fft_output = self.dsp.compute_fft(filtered)

        # Peak detection
        peak_bin, peak_freq, peak_mag = self.dsp.find_doppler_peak(fft_output)

        # SNR
        snr_db = self.dsp.compute_snr(fft_output, peak_bin)

        # Monopulse: left/right amplitude ratio
        channel_mags = np.abs(fft_output[peak_bin, :])
        left_energy = channel_mags[0] + channel_mags[1]
        right_energy = channel_mags[2] + channel_mags[3]
        total_energy = left_energy + right_energy + 1e-10

        bias = (right_energy - left_energy) / total_energy

        # Convert bias to angle (heuristic)
        # bias ∈ [-1, 1], map to ~[-80, +80] degrees
        angle_deg = bias * 80.0

        return {
            'angle_deg': angle_deg,
            'angle_bias': bias,
            'snr_db': snr_db,
            'fft_output': fft_output,
            'peak_bin': peak_bin
        }


def run_monte_carlo_trial(config, ground_truth_angle_deg):
    """
    Run a single Monte Carlo trial: generate signal, process with both pipelines.

    Args:
        config (dict): Simulation config
        ground_truth_angle_deg (float): True target angle

    Returns:
        dict: Results from MUSIC and monopulse pipelines
    """
    simulator = SyntheticRadarSimulator(config)

    # Generate synthetic signals
    snr_db = config.get('snr_db', 15)
    signals = simulator.generate_synthetic_signals(
        target_angles_deg=[ground_truth_angle_deg],
        snr_db=snr_db,
        num_samples=config.get('fft_size', 1024),
        oscillator_drift_hz=config.get('oscillator_drift_hz', 0.1),
        element_spacing_error_mm=config.get('element_spacing_error_mm', 0)
    )

    # Process with both pipelines
    music_result = simulator.process_music_pipeline(signals)
    monopulse_result = simulator.process_monopulse_pipeline(signals)

    # Compute errors
    music_angle_error = music_result['angle_deg'] - ground_truth_angle_deg
    monopulse_angle_error = monopulse_result['angle_deg'] - ground_truth_angle_deg

    return {
        'ground_truth_angle': ground_truth_angle_deg,
        'snr_db': snr_db,
        'music_angle': music_result['angle_deg'],
        'music_angle_error': music_angle_error,
        'monopulse_angle': monopulse_result['angle_deg'],
        'monopulse_angle_error': monopulse_angle_error,
        'music_snr': music_result['snr_db'],
        'monopulse_snr': monopulse_result['snr_db']
    }
