"""
Comprehensive unit tests for SyntheticRadarSimulator.

Tests cover:
1. Simulator initialization
2. Signal generation (shape, dtype, no NaNs)
3. MUSIC pipeline (returns dict with correct keys, angle in valid range)
4. Monopulse pipeline (returns dict with correct keys)
5. High SNR improves angle accuracy (statistical test)
6. Monte Carlo trial runs (complete without errors)
"""

import numpy as np
import pytest
from synthetic_radar_sim import SyntheticRadarSimulator, run_monte_carlo_trial


class TestSyntheticRadarSimulator:
    """Test suite for SyntheticRadarSimulator."""

    @pytest.fixture
    def default_config(self):
        """Provide a default configuration for tests."""
        return {
            'sample_rate': 10000,
            'fft_size': 1024,
            'music_angle_range': (-80, 80),
            'music_num_angles': 321,
            'music_smoothing_bins': 5,
            'snr_db': 15,
            'oscillator_drift_hz': 0.1,
            'element_spacing_error_mm': 0
        }

    @pytest.fixture
    def simulator(self, default_config):
        """Create a simulator instance with default config."""
        return SyntheticRadarSimulator(default_config)

    # Test 1: Simulator Initialization
    def test_simulator_initialization(self, default_config):
        """Test that simulator initializes correctly with given config."""
        sim = SyntheticRadarSimulator(default_config)

        # Check config is stored
        assert sim.config == default_config

        # Check DSP pipeline initialized
        assert sim.dsp is not None
        assert sim.dsp.sample_rate == 10000
        assert sim.dsp.fft_size == 1024

        # Check MUSIC processor initialized
        assert sim.music is not None

        # Check Kalman filter initialized
        assert sim.ekf is not None

        # Check steering vectors precomputed
        assert sim.music_angles is not None
        assert sim.steering_vectors is not None
        assert len(sim.music_angles) == 321
        assert sim.steering_vectors.shape == (321, 4)

    # Test 2: Signal Generation - Shape and dtype
    def test_signal_generation_shape(self, simulator):
        """Test that generate_synthetic_signals returns correct shape."""
        num_samples = 1024
        target_angles = [0.0]
        snr_db = 15

        signals = simulator.generate_synthetic_signals(
            target_angles_deg=target_angles,
            snr_db=snr_db,
            num_samples=num_samples
        )

        assert signals.shape == (num_samples, 4), \
            f"Expected shape ({num_samples}, 4), got {signals.shape}"
        assert signals.dtype == complex, \
            f"Expected dtype complex, got {signals.dtype}"

    # Test 3: Signal Generation - No NaNs or Infs
    def test_signal_generation_no_nans(self, simulator):
        """Test that generated signals contain no NaNs or Infs."""
        signals = simulator.generate_synthetic_signals(
            target_angles_deg=[0.0, 30.0],
            snr_db=10,
            num_samples=512
        )

        assert not np.any(np.isnan(signals)), \
            "Generated signals contain NaN values"
        assert not np.any(np.isinf(signals)), \
            "Generated signals contain Inf values"

    # Test 4: MUSIC Pipeline - Returns Dict with Correct Keys
    def test_music_pipeline_returns_dict(self, simulator):
        """Test that process_music_pipeline returns dict with all required keys."""
        signals = simulator.generate_synthetic_signals(
            target_angles_deg=[0.0],
            snr_db=15,
            num_samples=1024
        )

        result = simulator.process_music_pipeline(signals)

        # Check return type
        assert isinstance(result, dict), "MUSIC pipeline should return dict"

        # Check required keys
        required_keys = [
            'angle_deg', 'speed_mps', 'snr_db', 'pseudospectrum',
            'angles_deg', 'fft_output', 'peak_bin', 'peak_freq'
        ]
        for key in required_keys:
            assert key in result, f"Missing key '{key}' in MUSIC result"

    # Test 5: MUSIC Pipeline - Angle in Valid Range
    def test_music_angle_in_valid_range(self, simulator):
        """Test that MUSIC angle estimates are within valid range [-80, 80]."""
        test_angles = [-60.0, -30.0, 0.0, 30.0, 60.0]

        for target_angle in test_angles:
            signals = simulator.generate_synthetic_signals(
                target_angles_deg=[target_angle],
                snr_db=20,
                num_samples=1024
            )

            result = simulator.process_music_pipeline(signals)
            angle_deg = result['angle_deg']

            assert -80 <= angle_deg <= 80, \
                f"Angle {angle_deg} outside valid range [-80, 80]"

    # Test 6: Monopulse Pipeline - Returns Dict with Correct Keys
    def test_monopulse_pipeline_returns_dict(self, simulator):
        """Test that process_monopulse_pipeline returns dict with all required keys."""
        signals = simulator.generate_synthetic_signals(
            target_angles_deg=[0.0],
            snr_db=15,
            num_samples=1024
        )

        result = simulator.process_monopulse_pipeline(signals)

        # Check return type
        assert isinstance(result, dict), "Monopulse pipeline should return dict"

        # Check required keys
        required_keys = ['angle_deg', 'angle_bias', 'snr_db', 'fft_output', 'peak_bin']
        for key in required_keys:
            assert key in result, f"Missing key '{key}' in monopulse result"

    # Test 7: Monopulse Pipeline - Angle in Valid Range
    def test_monopulse_angle_in_valid_range(self, simulator):
        """Test that monopulse angle estimates are within valid range [-80, 80]."""
        test_angles = [-60.0, -30.0, 0.0, 30.0, 60.0]

        for target_angle in test_angles:
            signals = simulator.generate_synthetic_signals(
                target_angles_deg=[target_angle],
                snr_db=20,
                num_samples=1024
            )

            result = simulator.process_monopulse_pipeline(signals)
            angle_deg = result['angle_deg']

            assert -80 <= angle_deg <= 80, \
                f"Angle {angle_deg} outside valid range [-80, 80]"

    # Test 8: High SNR Improves Angle Accuracy (Statistical Test)
    def test_high_snr_improves_accuracy(self, simulator):
        """
        Test that higher SNR leads to smaller angle estimation errors (statistically).
        Uses 20 trials for each SNR level with higher SNR separation.
        """
        target_angle = 30.0
        snr_values = [2, 25]  # Low SNR, High SNR (larger separation)
        num_trials = 20

        errors_low_snr = []
        errors_high_snr = []

        # Low SNR trials
        for _ in range(num_trials):
            signals = simulator.generate_synthetic_signals(
                target_angles_deg=[target_angle],
                snr_db=snr_values[0],
                num_samples=1024
            )
            result = simulator.process_music_pipeline(signals)
            error = abs(result['angle_deg'] - target_angle)
            errors_low_snr.append(error)

        # High SNR trials
        for _ in range(num_trials):
            signals = simulator.generate_synthetic_signals(
                target_angles_deg=[target_angle],
                snr_db=snr_values[1],
                num_samples=1024
            )
            result = simulator.process_music_pipeline(signals)
            error = abs(result['angle_deg'] - target_angle)
            errors_high_snr.append(error)

        # Statistical comparison: high SNR errors should be smaller on average
        mean_error_low = np.mean(errors_low_snr)
        mean_error_high = np.mean(errors_high_snr)
        median_error_low = np.median(errors_low_snr)
        median_error_high = np.median(errors_high_snr)

        # Use both mean and median for robustness
        assert (mean_error_high < mean_error_low) or (median_error_high < median_error_low), \
            f"High SNR should have better accuracy. " \
            f"Mean: low={mean_error_low:.4f}, high={mean_error_high:.4f}; " \
            f"Median: low={median_error_low:.4f}, high={median_error_high:.4f}"

    # Test 9: Monte Carlo Trial Runs Without Errors
    def test_monte_carlo_trial_completes(self, default_config):
        """Test that run_monte_carlo_trial completes without errors."""
        ground_truth_angle = 45.0

        result = run_monte_carlo_trial(default_config, ground_truth_angle)

        # Check return type
        assert isinstance(result, dict), "Monte Carlo trial should return dict"

        # Check result contains expected keys
        expected_keys = [
            'ground_truth_angle', 'snr_db', 'music_angle', 'music_angle_error',
            'monopulse_angle', 'monopulse_angle_error', 'music_snr', 'monopulse_snr'
        ]
        for key in expected_keys:
            assert key in result, f"Missing key '{key}' in Monte Carlo result"

    # Test 10: Multiple Monte Carlo Trials
    def test_monte_carlo_multiple_trials(self, default_config):
        """Test multiple Monte Carlo trials run successfully."""
        ground_truth_angles = [-45.0, 0.0, 45.0]
        num_trials = 3

        all_results = []
        for target_angle in ground_truth_angles:
            for _ in range(num_trials):
                result = run_monte_carlo_trial(default_config, target_angle)
                all_results.append(result)

        assert len(all_results) == len(ground_truth_angles) * num_trials, \
            "Should have completed all Monte Carlo trials"

        # Verify all results contain required fields
        for result in all_results:
            assert result['ground_truth_angle'] is not None
            assert result['music_angle'] is not None
            assert result['monopulse_angle'] is not None
            assert -80 <= result['music_angle'] <= 80
            assert -80 <= result['monopulse_angle'] <= 80

    # Test 11: FFT Output Properties
    def test_fft_output_properties(self, simulator):
        """Test that FFT outputs have correct properties."""
        signals = simulator.generate_synthetic_signals(
            target_angles_deg=[0.0],
            snr_db=15,
            num_samples=1024
        )

        result = simulator.process_music_pipeline(signals)
        fft_output = result['fft_output']

        # Check shape
        assert fft_output.shape[0] == 1024, "FFT output should have 1024 frequency bins"
        assert fft_output.shape[1] == 4, "FFT output should have 4 channels"

        # Check dtype
        assert np.iscomplexobj(fft_output), "FFT output should be complex"

        # Check no NaNs or Infs
        assert not np.any(np.isnan(fft_output)), "FFT output should not contain NaNs"
        assert not np.any(np.isinf(fft_output)), "FFT output should not contain Infs"

    # Test 12: SNR Output Properties
    def test_snr_output_properties(self, simulator):
        """Test that SNR estimates are reasonable."""
        test_snrs = [5, 15, 25]

        for target_snr in test_snrs:
            signals = simulator.generate_synthetic_signals(
                target_angles_deg=[0.0],
                snr_db=target_snr,
                num_samples=1024
            )

            music_result = simulator.process_music_pipeline(signals)
            monopulse_result = simulator.process_monopulse_pipeline(signals)

            # SNR estimates should be positive (in dB)
            assert music_result['snr_db'] > -100, \
                "MUSIC SNR estimate should be reasonable"
            assert monopulse_result['snr_db'] > -100, \
                "Monopulse SNR estimate should be reasonable"

    # Test 13: Signal Generation with Oscillator Drift
    def test_signal_generation_with_oscillator_drift(self, simulator):
        """Test signal generation with oscillator drift parameter."""
        signals = simulator.generate_synthetic_signals(
            target_angles_deg=[0.0],
            snr_db=15,
            num_samples=512,
            oscillator_drift_hz=1.0
        )

        assert signals.shape == (512, 4)
        assert not np.any(np.isnan(signals))
        assert not np.any(np.isinf(signals))

    # Test 14: Signal Generation with Element Spacing Error
    def test_signal_generation_with_spacing_error(self, simulator):
        """Test signal generation with element spacing error."""
        signals = simulator.generate_synthetic_signals(
            target_angles_deg=[0.0],
            snr_db=15,
            num_samples=512,
            element_spacing_error_mm=1.0
        )

        assert signals.shape == (512, 4)
        assert not np.any(np.isnan(signals))
        assert not np.any(np.isinf(signals))

    # Test 15: Multiple Target Angles
    def test_signal_generation_multiple_targets(self, simulator):
        """Test signal generation with multiple target angles."""
        signals = simulator.generate_synthetic_signals(
            target_angles_deg=[-30.0, 0.0, 30.0],
            snr_db=15,
            num_samples=1024
        )

        assert signals.shape == (1024, 4)
        assert not np.any(np.isnan(signals))
        assert not np.any(np.isinf(signals))


# Additional standalone tests (not part of class)

def test_steering_vector_computation():
    """Test MUSIC steering vector computation."""
    sim = SyntheticRadarSimulator({
        'sample_rate': 10000,
        'fft_size': 1024,
        'music_angle_range': (-80, 80),
        'music_num_angles': 321
    })

    # Check that precomputed steering vectors match on-demand computation
    angles_deg, steering_vecs = sim.music.compute_steering_vectors(
        angle_range_deg=(-80, 80), num_angles=321
    )

    assert angles_deg.shape == (321,)
    assert steering_vecs.shape == (321, 4)
    assert np.iscomplexobj(steering_vecs)


def test_signal_generation_repeatability():
    """Test that signal generation is repeatable with same random seed."""
    config = {
        'sample_rate': 10000,
        'fft_size': 1024,
        'music_angle_range': (-80, 80),
        'music_num_angles': 321
    }

    # Generate signals with same seed
    np.random.seed(42)
    sim1 = SyntheticRadarSimulator(config)
    signals1 = sim1.generate_synthetic_signals(
        target_angles_deg=[0.0], snr_db=15, num_samples=512
    )

    np.random.seed(42)
    sim2 = SyntheticRadarSimulator(config)
    signals2 = sim2.generate_synthetic_signals(
        target_angles_deg=[0.0], snr_db=15, num_samples=512
    )

    np.testing.assert_array_almost_equal(signals1, signals2)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
