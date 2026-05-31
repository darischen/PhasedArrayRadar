"""
MUSIC (MUltiple SIgnal Classification) algorithm for angle-of-arrival estimation.
"""

import numpy as np


class MUSICProcessor:
    """MUSIC pseudospectrum and angle-of-arrival estimation."""

    def __init__(self, wavelength=0.0285, element_positions=None):
        """
        Initialize MUSIC processor.

        Args:
            wavelength (float): RF wavelength in meters
            element_positions (np.ndarray): Antenna element positions (meters)
        """
        self.wavelength = wavelength
        if element_positions is None:
            element_positions = np.array([0.0, 0.037, 0.075, 0.112])
        self.element_positions = element_positions
        self.num_elements = len(element_positions)

    def compute_steering_vectors(self, angle_range_deg, num_angles):
        """
        Precompute steering vectors for angle search range.

        Args:
            angle_range_deg (tuple): (min_angle, max_angle) in degrees
            num_angles (int): Number of angles to sample

        Returns:
            tuple: (angles_deg, steering_vectors)
                angles_deg: np.ndarray, shape (num_angles,)
                steering_vectors: np.ndarray, shape (num_angles, num_elements)
        """
        angles_deg = np.linspace(angle_range_deg[0], angle_range_deg[1], num_angles)
        angles_rad = np.radians(angles_deg)

        # Steering vector: a(θ) = exp(j * 2π * sin(θ) * d / λ)
        steering_vectors = np.exp(
            1j * 2 * np.pi * np.outer(np.sin(angles_rad), self.element_positions) / self.wavelength
        )
        return angles_deg, steering_vectors

    def compute_correlation_matrix(self, fft_output, peak_bin, smoothing_bins=5):
        """
        Compute spatial correlation matrix from FFT outputs.

        Args:
            fft_output (np.ndarray): Complex FFT, shape (num_freq_bins, num_channels)
            peak_bin (int): Doppler peak bin index
            smoothing_bins (int): Number of bins around peak to include

        Returns:
            np.ndarray: Spatial correlation matrix, shape (num_channels, num_channels)
        """
        # Extract complex FFT samples around Doppler peak
        start_bin = max(0, peak_bin - smoothing_bins)
        end_bin = min(fft_output.shape[0], peak_bin + smoothing_bins + 1)

        fft_bin_range = fft_output[start_bin:end_bin, :]  # Shape: (num_bins, num_channels)

        # Correlation: R = (1/N) Σ x * x^H
        R = np.zeros((self.num_elements, self.num_elements), dtype=complex)
        for x in fft_bin_range:
            R += np.outer(x, np.conj(x))
        R /= fft_bin_range.shape[0]

        # Forward-backward averaging for real-valued symmetry
        # R_avg = 0.5 * (R + J * conj(R) * J^T)
        J = np.fliplr(np.eye(self.num_elements))
        R_fb = 0.5 * (R + J @ np.conj(R) @ J.T)

        return R_fb

    def compute_pseudospectrum(self, R, steering_vectors):
        """
        Compute MUSIC pseudospectrum.

        Args:
            R (np.ndarray): Spatial correlation matrix, shape (num_channels, num_channels)
            steering_vectors (np.ndarray): Steering vectors, shape (num_angles, num_channels)

        Returns:
            np.ndarray: Pseudospectrum, shape (num_angles,)
        """
        # Eigendecomposition: R = U * Λ * U^H
        eigenvalues, eigenvectors = np.linalg.eigh(R)

        # Noise subspace = eigenvectors of smallest eigenvalues
        # (typically num_elements - 1 for single target)
        noise_subspace = eigenvectors[:, :-1]

        # MUSIC: P(θ) = 1 / |a^H(θ) * E_n * E_n^H * a(θ)|
        pseudospectrum = np.zeros(len(steering_vectors))
        for i, a in enumerate(steering_vectors):
            proj = np.abs(a @ noise_subspace @ np.conj(noise_subspace.T) @ np.conj(a))
            pseudospectrum[i] = 1.0 / (proj + 1e-10)

        return pseudospectrum

    def estimate_angle(self, fft_output, peak_bin, angle_range_deg=(-80, 80), num_angles=321, smoothing_bins=5):
        """
        Estimate angle-of-arrival using MUSIC.

        Args:
            fft_output (np.ndarray): Complex FFT, shape (num_freq_bins, num_channels)
            peak_bin (int): Doppler peak bin
            angle_range_deg (tuple): Angle search range
            num_angles (int): Number of angles to sample
            smoothing_bins (int): Bins around peak for correlation matrix

        Returns:
            tuple: (estimated_angle_deg, pseudospectrum, angles_deg)
        """
        angles_deg, steering_vectors = self.compute_steering_vectors(angle_range_deg, num_angles)
        R = self.compute_correlation_matrix(fft_output, peak_bin, smoothing_bins)
        pseudospectrum = self.compute_pseudospectrum(R, steering_vectors)

        # Peak of pseudospectrum = estimated angle
        peak_idx = np.argmax(pseudospectrum)
        estimated_angle = angles_deg[peak_idx]

        return estimated_angle, pseudospectrum, angles_deg
