# Simulation Reproduction Guide

This guide provides comprehensive instructions for running, configuring, and modifying the simulation environment.

## Quick Start

### Installation
```bash
pip install -e .
```

### Run Example Simulations

Example 1: Basic simulation with default parameters
```bash
cd simulation
python examples/basic_simulation.py
```

Example 2: Run with custom configuration
```bash
cd simulation
python -m hydra.main config_path=configs config_name=high_speed_scenario
```

Example 3: Batch processing with multirun
```bash
cd simulation
python examples/batch_simulation.py --multirun scenario=low_speed,high_speed num_particles=100,500
```

## Configuration Guide

### Overview
The simulation uses YAML configuration files to manage parameters and experimental setups.

### defaults.yaml
The `configs/defaults.yaml` file contains baseline parameters:
- **Simulation Parameters**: Duration, timestep, sample rate
- **Sensor Configuration**: Number of channels, antenna spacing, noise levels
- **Algorithm Settings**: MUSIC resolution, monopulse gain, EKF process noise
- **Output Settings**: Log directory, figure generation flags

Example structure:
```yaml
simulation:
  duration: 10.0  # seconds
  timestep: 0.01  # seconds
  sample_rate: 1000  # Hz

sensor:
  num_channels: 8
  antenna_spacing: 0.05  # meters
  noise_level: 0.01

algorithms:
  music:
    resolution: 0.5  # degrees
  monopulse:
    gain: 1.5
  ekf:
    process_noise: 0.01
```

### Experiment Configurations
Four pre-configured experiment scenarios are provided:

1. **low_speed_scenario.yaml**: Tests algorithm performance with slow-moving targets
   - Speed range: 0-5 m/s
   - Static environment
   - SNR: 20 dB

2. **high_speed_scenario.yaml**: Tests algorithm robustness at high speeds
   - Speed range: 50-100 m/s
   - Dynamic environment
   - SNR: 15 dB

3. **urban_environment.yaml**: Simulates urban clutter and multipath
   - Multiple reflectors
   - Non-Gaussian noise
   - SNR: 10 dB

4. **validation_scenario.yaml**: Reproduces published benchmark conditions
   - Reference parameters from literature
   - Known ground truth
   - SNR: 25 dB

### Using Configuration Files
```bash
# Use a specific experiment configuration
python examples/run_scenario.py config_name=low_speed_scenario

# Override individual parameters
python examples/run_scenario.py config_name=high_speed_scenario sensor.noise_level=0.02
```

## Modifying Parameters

### Command-Line Parameter Override
Override any configuration parameter directly from the command line:

```bash
# Override single parameters
python examples/simulation.py simulation.duration=20.0 sensor.num_channels=16

# Override nested parameters
python examples/simulation.py algorithms.ekf.process_noise=0.05 algorithms.music.resolution=0.25

# Set boolean flags
python examples/simulation.py output.generate_figures=true output.save_raw_data=false
```

### Multirun Experiments
Run multiple parameter combinations in batch:

```bash
# Sweep over discrete values
python examples/simulation.py --multirun sensor.noise_level=0.01,0.05,0.1

# Cartesian product of parameters
python examples/simulation.py --multirun \
  algorithms.music.resolution=0.25,0.5,1.0 \
  sensor.num_channels=8,16,32

# Combine with config changes
python examples/simulation.py --multirun config_name=low_speed_scenario,high_speed_scenario
```

## Output Explanation

### Generated Files

The simulation generates output in the `outputs/` directory with the following structure:

```
outputs/
├── results/
│   ├── trajectory.json          # Target positions over time
│   ├── measurements.json         # Raw sensor measurements
│   ├── estimates.json            # Algorithm output (positions, velocities)
│   └── performance_metrics.json  # Accuracy, latency, convergence stats
│
├── figures/
│   ├── trajectory_2d.png         # 2D target trajectory
│   ├── trajectory_3d.png         # 3D position plot
│   ├── music_spectrum.png        # MUSIC spatial spectrum
│   ├── monopulse_pattern.png     # Monopulse error pattern
│   ├── ekf_convergence.png       # EKF state estimate convergence
│   └── error_over_time.png       # Position estimation error time series
│
└── hydra_logs/
    ├── .hydra/config.yaml        # Runtime configuration snapshot
    ├── .hydra/job.log            # Execution log
    └── .hydra/overrides.yaml      # Applied parameter overrides
```

### JSON Output Format

**trajectory.json**: Ground truth target motion
```json
{
  "timestamps": [0.0, 0.01, 0.02, ...],
  "positions": [[x1, y1, z1], [x2, y2, z2], ...],
  "velocities": [[vx1, vy1, vz1], [vx2, vy2, vz2], ...]
}
```

**estimates.json**: Algorithm output for comparison
```json
{
  "timestamps": [0.0, 0.01, 0.02, ...],
  "music_estimates": [[x1, y1, z1], [x2, y2, z2], ...],
  "monopulse_estimates": [[x1, y1, z1], [x2, y2, z2], ...],
  "ekf_estimates": [[x1, y1, z1], [x2, y2, z2], ...]
}
```

**performance_metrics.json**: Quantitative results
```json
{
  "music": {
    "mean_error": 0.25,  # meters
    "max_error": 1.5,
    "convergence_time": 2.3  # seconds
  },
  "monopulse": {
    "mean_error": 0.18,
    "max_error": 0.8,
    "convergence_time": 1.1
  },
  "ekf": {
    "mean_error": 0.12,
    "max_error": 0.6,
    "convergence_time": 0.9
  }
}
```

## Algorithms Section

### MUSIC (Multiple Signal Classification)
**Implementation**: `simulation/algorithms/music.py`

MUSIC performs spatial spectrum analysis to estimate target direction of arrival (DoA):
- Uses eigendecomposition of the sensor covariance matrix
- Computes spatial spectrum across azimuth and elevation angles
- Peak detection identifies target bearings
- **Resolution**: Configurable in `algorithms.music.resolution` (degrees)
- **Limitations**: Assumes stationary targets; sensitive to model mismatch

### Monopulse
**Implementation**: `simulation/algorithms/monopulse.py`

Monopulse provides real-time angle tracking with low latency:
- Compares amplitude and phase differences between antenna sub-arrays
- Produces error voltage proportional to angular deviation
- Fast feedback enables continuous tracking
- **Gain**: Configurable in `algorithms.monopulse.gain`
- **Advantages**: Low computational cost; continuous tracking capability

### Extended Kalman Filter (EKF)
**Implementation**: `simulation/algorithms/ekf.py`

EKF fuses all algorithm outputs with motion models:
- State: position (x, y, z), velocity (vx, vy, vz), acceleration
- Prediction: constant-velocity or constant-acceleration model
- Update: Incorporates MUSIC and monopulse measurements
- **Process Noise**: Configurable in `algorithms.ekf.process_noise`
- **Advantages**: Temporal smoothing; optimal in Gaussian noise

**State Transition Matrix**:
```
x_{k+1} = F * x_k + w_k
where F incorporates timestep and motion model
```

**Measurement Likelihood**:
```
z_k = H * x_k + v_k
where H projects state to measurement space
```

## Testing Instructions

### Running Tests
Execute the test suite with pytest:

```bash
cd simulation
pytest tests/ -v
```

### Test Coverage
Run tests with coverage analysis:

```bash
pytest tests/ --cov=simulation --cov-report=html
```

### Individual Test Modules

Test MUSIC algorithm:
```bash
pytest tests/test_algorithms/test_music.py -v
```

Test configuration loading:
```bash
pytest tests/test_config/test_defaults.py -v
```

Test output generation:
```bash
pytest tests/test_output/test_json_generation.py -v
```

### Expected Test Results

Successful test run output:
```
tests/test_algorithms/test_music.py::test_music_basic_case PASSED
tests/test_algorithms/test_music.py::test_music_with_noise PASSED
tests/test_algorithms/test_monopulse.py::test_monopulse_tracking PASSED
tests/test_algorithms/test_ekf.py::test_ekf_convergence PASSED
tests/test_config/test_defaults.py::test_load_defaults PASSED
tests/test_config/test_experiments.py::test_all_experiment_configs_valid PASSED
tests/test_output/test_json_generation.py::test_results_file_format PASSED

========== 7 passed in 0.42s ==========
```

### Test Categories

- **Algorithm Tests** (`test_algorithms/`): Unit tests for MUSIC, monopulse, EKF
- **Configuration Tests** (`test_config/`): Validation of YAML configs
- **Integration Tests** (`test_integration/`): End-to-end simulation runs
- **Output Tests** (`test_output/`): Verification of generated files and formats

## Troubleshooting

### Common Issues

**Issue**: Import errors when running examples
- **Solution**: Ensure installation: `pip install -e .`

**Issue**: Configuration not found
- **Solution**: Run from `simulation/` directory or specify full path: `--config-path=/absolute/path/configs`

**Issue**: OutOfMemory errors with large simulations
- **Solution**: Reduce `simulation.duration`, `sensor.num_channels`, or run with `--multirun job_num=0,1,2` to parallelize

**Issue**: Test failures
- **Solution**: Check Python version (3.8+), update dependencies: `pip install --upgrade -r requirements.txt`

## Additional Resources

- **Configuration Documentation**: See `configs/README.md`
- **API Reference**: See `docs/api.md`
- **Paper**: Reproduction of results from [reference paper if applicable]
