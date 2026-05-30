# HB100 Phased Array Radar: Research Paper & Reproducible Artifact Design

**Date:** 2025-05-30  
**Status:** Approved  
**Scope:** Transform HB100 Doppler radar codebase into arXiv-ready research paper with reproducible simulation framework

---

## 1. Problem Statement

The HB100 Phased Array Radar system currently has:
- ✅ Working signal processing (MUSIC, EKF, forward-backward averaging)
- ✅ Complete hardware + firmware + host processor implementation
- ✅ Comprehensive technical documentation (HB100.md, SETUP_GUIDE.md)
- ❌ **No evaluation or benchmarking**
- ❌ **No figures or visualizations**
- ❌ **No research framing or publication artifact**

**Goal:** Create a rigorous arXiv research paper establishing HB100 viability for phased array Doppler radar, with reproducible simulation and configuration management.

---

## 2. Research Contributions (Claimed)

1. **Novel Analog Signal Conditioning Design**
   - First documented circuit for handling bipolar IF outputs from free-running Gunn oscillators on unipolar ADC
   - Virtual ground biasing + multi-stage op-amp gain (1092×) architecture
   - Circuit photos and detailed design rationale

2. **Algorithm Comparison & Validation**
   - MUSIC achieves ~2–5° angle accuracy despite free-running oscillators
   - Quantified benefit of forward-backward averaging for phase coherence
   - Baseline comparison: MUSIC vs simpler monopulse (amplitude-only)
   - EKF tracking improves temporal smoothing

3. **Reproducible Open-Source Artifact**
   - Complete system design (hardware schematics, firmware, DSP)
   - Hydra-managed simulation framework for parameter sweeps
   - Scripted figure generation (all paper results reproducible via config files)
   - GitHub repository with clear reproduction instructions

---

## 3. Paper Structure & Content

### 3.1 Document Format & Length

- **Format:** LaTeX, arXiv-compatible (11–12 pages)
- **Template:** Standard IEEE/arXiv style (article class, 10pt, 1.15 line spacing)
- **Figures:** 4–5 publication-quality plots (matplotlib + seaborn, PDF output, 300 dpi)
- **References:** BibTeX, ~20–25 entries (phased arrays, MUSIC, HB100 applications, radar fundamentals)

### 3.2 Section Breakdown

| Section | Pages | Content |
|---------|-------|---------|
| **Abstract** | 0.25 | Problem, approach, contributions, reproducibility claim |
| **1. Introduction** | 1.0 | HB100 market (cheap), gap (no phased arrays), our contribution |
| **2. Related Work** | 1.5 | Phased array theory, MUSIC, monopulse, HB100 applications, budget radar systems |
| **3. System Design** | 2.5 | Hardware (array geometry, ADC, analog conditioning), firmware (DMA, USB-CDC), software pipeline |
| **4. Signal Processing** | 2.0 | MUSIC algorithm, monopulse baseline, EKF, forward-backward averaging, why each component matters |
| **5. Simulation & Validation** | 2.5 | Synthetic signal generation, metrics (RMS error, detection probability), results with 4 figures, sensitivity analysis |
| **6. Reproducibility** | 0.5 | Code structure, Hydra config examples, reproduction instructions |
| **7. Conclusion** | 0.5 | Summary, next steps (hardware validation), future extensions |
| **References** | 0.25 | ~25 citations |

### 3.3 Key Figures (Generated)

**Figure 1: Angle Estimation Error vs SNR**
- X-axis: SNR (0–30 dB)
- Y-axis: RMS angle error (degrees)
- Three curves: monopulse, MUSIC (standard), MUSIC + forward-backward averaging
- Error bands (mean ± std from 500 trials)
- Shows MUSIC advantage, especially at low SNR

**Figure 2: MUSIC vs Monopulse Comparison Table**
- Angle error at SNR=15 dB (best case vs realistic case)
- Detection probability at various SNRs
- Computational complexity
- Robustness to oscillator drift

**Figure 3: Example MUSIC Pseudospectrum**
- 3D surface or 2D heatmap of pseudospectrum P(θ)
- Clear peak at true target angle
- Grating lobes visible (expected with 3.93λ spacing)
- Annotations explaining key features

**Figure 4: Sensitivity to Oscillator Drift & Element Spacing Errors**
- Heatmap or contour plot
- X-axis: oscillator drift magnitude (0–2 Hz)
- Y-axis: element spacing error (−2 to +2 mm)
- Z-axis: angle RMS error
- Shows forward-backward averaging robustness region

**Figure 5 (Optional): Circuit Diagram & Photo**
- Schematic of analog signal conditioning (from HB100-assets or recreated in TikZ)
- Circuit photo (from HB100-assets/circuit-photo.jpg)
- Annotations of gain stages, virtual ground, filtering

---

## 4. Simulation Framework Design

### 4.1 Core Components

**`simulation/synthetic_radar_sim.py`** (Main simulation engine)

**Purpose:** Generate synthetic 4-channel radar signals and run MUSIC + monopulse pipelines

**Key functions:**
- `generate_synthetic_signals(target_angles, snr_db, num_samples)`: Create complex baseband signals using steering vectors + AWGN
- `run_music_pipeline(signals, config)`: MUSIC algorithm (DC removal, filter, FFT, MUSIC, pseudospectrum)
- `run_monopulse_pipeline(signals, config)`: Amplitude-only baseline (FFT magnitudes, left/right bias)
- `run_ekf_tracking(measurements, config)`: Extended Kalman Filter (constant-velocity model)
- `compute_metrics(estimates, ground_truth)`: RMS error, detection rate, convergence speed

**Key parameters (from Hydra config):**
- `target_angles`: List of angles to simulate (degrees)
- `snr_db`: Signal-to-noise ratio range
- `num_trials`: Monte Carlo trials per condition
- `oscillator_drift_hz`: Free-running LO drift
- `element_spacing_error_mm`: Systematic array calibration error
- MUSIC parameters: `smoothing_bins`, `angle_range`, `num_angles`
- Kalman filter parameters: `process_noise`, `angle_noise`, `speed_noise`

**Output:** Dictionary with estimated angles, pseudospectra, detection flags, metrics

**Dependencies:** Reuse code from `radar_processor.py` (MUSIC, EKF, DSP filters) where possible

---

**`simulation/generate_figures.py`** (Figure orchestration)

**Purpose:** Run experiments via Hydra configs, compute statistics, generate publication-quality plots

**Key functions:**
- `run_experiment(config)`: Load Hydra config, execute `synthetic_radar_sim.py`, collect results
- `compute_statistics(results)`: Mean, std, percentiles across trials
- `plot_angle_error_vs_snr(results)`: Matplotlib figure with error bands
- `plot_baseline_comparison_table(results)`: Formatted table (MUSIC vs monopulse)
- `plot_music_pseudospectrum(results)`: 3D or heatmap visualization
- `plot_sensitivity_heatmap(results)`: Oscillator drift × spacing error

**Workflow:**
1. Load `simulation/config/defaults.yaml` (base parameters)
2. Merge in experiment-specific config (e.g., `paper_angle_error_vs_snr.yaml`)
3. Run `synthetic_radar_sim.py` with merged config
4. Compute statistics (RMS error, detection rate)
5. Generate figure (matplotlib), save to `paper/figures/` as PDF
6. Log experiment metadata (Hydra auto-handles this)

**Output:** PDF figures in `paper/figures/`, experiment logs in `outputs/<timestamp>/`

---

### 4.2 Hydra Configuration Structure

**Base Config: `simulation/config/defaults.yaml`**

Contains all tunable parameters with sensible defaults:

```yaml
simulation:
  target_angles: [-80, -40, 0, 40, 80]
  snr_db: [5, 10, 15, 20, 25, 30]
  num_trials: 500
  oscillator_drift_hz: 0.1
  element_spacing_error_mm: 0
  random_seed: 42

dsp:
  bandpass_low: 10.0
  bandpass_high: 800.0
  fft_size: 1024
  sample_rate: 10000

music:
  angle_range: [-80, 80]
  num_angles: 321
  smoothing_bins: 5

kalman_filter:
  process_noise: 0.5
  speed_noise: 0.3
  angle_noise: 5.0  # degrees

monopulse:
  enabled: true

output:
  save_figures: true
  figure_format: pdf
  dpi: 300
  output_dir: ../paper/figures
```

**Experiment Configs: `simulation/config/paper_*.yaml`**

Override specific parameters for each figure:

```yaml
# paper_angle_error_vs_snr.yaml
defaults:
  - default

simulation:
  target_angles: [-80, -60, -40, -20, 0, 20, 40, 60, 80]
  snr_db: [0, 5, 10, 15, 20, 25, 30]
  num_trials: 1000

experiment_name: "figure_1_angle_error_vs_snr"
```

**Usage:**
```bash
# Single experiment
python generate_figures.py --config-name=paper_angle_error_vs_snr

# Multi-run (Hydra sweeps SNR automatically)
python generate_figures.py --config-name=paper_angle_error_vs_snr --multirun snr_db=0,5,10,15,20,25,30

# Reproduce all paper figures
python generate_figures.py --config-name=paper_results_full
```

---

## 5. Repository Structure

```
HB100/
├── README.md                              # Updated with link to paper
├── SETUP_GUIDE.md                         # (existing)
├── HB100.md                               # (existing, referenced in paper)
│
├── paper/                                 # NEW: Research paper
│   ├── paper.tex                          # Main document
│   ├── paper.bib                          # References
│   ├── preamble.tex                       # Macros, styling
│   ├── Makefile                           # Build PDF (latexmk)
│   ├── .gitignore                         # Ignore .pdf, .aux, .log, etc.
│   │
│   └── figures/                           # Generated by simulation/
│       ├── circuit_diagram.pdf
│       ├── circuit_photo.pdf
│       ├── array_geometry.pdf
│       ├── angle_error_vs_snr.pdf
│       ├── baseline_comparison.pdf
│       ├── music_pseudospectrum.pdf
│       └── sensitivity_drift.pdf
│
├── simulation/                            # NEW: Reproducible simulation
│   ├── synthetic_radar_sim.py             # Core engine
│   ├── generate_figures.py                # Orchestrator
│   ├── requirements.txt                   # Dependencies (numpy, scipy, hydra-core, matplotlib, seaborn)
│   ├── README.md                          # Reproduction instructions
│   │
│   └── config/                            # Hydra configs
│       ├── defaults.yaml
│       ├── paper_angle_error_vs_snr.yaml
│       ├── paper_baseline_comparison.yaml
│       ├── paper_sensitivity_drift.yaml
│       └── paper_sensitivity_spacing_error.yaml
│
├── main/                                  # (existing, firmware)
├── radar_processor.py                     # (existing)
├── zone_classifier.py                     # (existing)
└── requirements.txt                       # (existing)
```

---

## 6. Implementation Plan (High-Level)

**Phase 1: Paper LaTeX Framework (1–2 hours)**
- Create `paper/` directory structure
- Write `paper.tex` skeleton with sections, placeholders for figures
- Set up BibTeX references (`paper.bib`)
- Create Makefile for PDF build

**Phase 2: Simulation Engine (3–4 hours)**
- Extract/refactor MUSIC, EKF, DSP functions from `radar_processor.py` into reusable modules
- Write `synthetic_radar_sim.py` with signal generation and pipeline
- Validate against known targets (angle estimates should match inputs)
- Write unit tests for key functions

**Phase 3: Hydra Integration & Figure Generation (2–3 hours)**
- Set up Hydra configs in `simulation/config/`
- Write `generate_figures.py` orchestrator
- Create matplotlib plotting functions for each figure
- Verify all figures generate correctly and look publication-ready

**Phase 4: Paper Content (4–6 hours)**
- Write Introduction, Related Work, System Design, Signal Processing sections
- Integrate figures into LaTeX
- Polish references and cross-references
- Proofread for clarity and correctness

**Phase 5: Reproducibility Documentation (1–2 hours)**
- Write `simulation/README.md` with clear reproduction instructions
- Add inline code comments explaining algorithms
- Create example commands (how to run simulations, modify configs)
- Document all Hydra parameters

**Estimated Total Time:** 12–17 hours

---

## 7. Success Criteria

### Paper
- ✅ 11–12 pages, arXiv-ready formatting (no obvious issues)
- ✅ All figures generated reproducibly from `simulation/` configs
- ✅ Clear statement of contributions (analog design, algorithm comparison, reproducibility)
- ✅ References to HB100.md for detailed circuit explanation
- ✅ Conclusion frames future hardware validation

### Simulation Framework
- ✅ User can run `python generate_figures.py --config-name=paper_results_full` and get identical PDFs
- ✅ All Hydra configs are documented and self-explanatory
- ✅ Code reuses existing `radar_processor.py` where possible
- ✅ No hardcoded parameters in scripts (all in YAML configs)

### Reproducibility Artifact
- ✅ GitHub ready: paper source (LaTeX) + simulation code + configs
- ✅ Top-level README links to paper and reproduction instructions
- ✅ Clear entry points for users (single command to reproduce all figures)
- ✅ Code quality: docstrings, type hints where helpful, clear variable names

---

## 8. Assumptions & Constraints

**Assumptions:**
- Simulation with synthetic targets is sufficient for arXiv publication (no hardware data required)
- MUSIC algorithm implementation in `radar_processor.py` is correct and can be extracted
- HB100-assets folder contains usable circuit diagram/photo
- Hydra framework is familiar enough or documentation suffices for users

**Constraints:**
- No real hardware data available (simulation only)
- Paper must be self-contained (readers should understand HB100 context without external links)
- Figures must be publication-quality (clear, properly labeled, interpretable)
- All results must be reproducible via config files (no magic numbers or ad-hoc tweaks in code)

**Mitigation:**
- Clearly state in paper that results are simulation-based, hardware validation pending
- Include thorough system description + photos in paper (readers understand the physical setup)
- Write simulation carefully to avoid numerical artifacts
- Version control configs and commit generated figures so readers can verify

---

## 9. Deliverables

**By end of this work:**

1. **`paper/paper.pdf`** — Complete 12-page research paper ready for arXiv submission
2. **`paper/*.tex`** — Source files (paper.tex, preamble.tex, paper.bib)
3. **`simulation/synthetic_radar_sim.py`** — Core simulation engine
4. **`simulation/generate_figures.py`** — Figure generation orchestrator
5. **`simulation/config/*.yaml`** — All Hydra configs for reproducibility
6. **`simulation/README.md`** — Detailed reproduction instructions
7. **`paper/figures/*.pdf`** — All 4–5 publication-quality figures
8. **Updated `README.md`** — Link to paper, research artifact highlights

**GitHub State:**
- All source files committed
- Paper and simulation fully integrated
- Clear instructions for cloning and reproducing results

---

## 10. Open Questions / Decisions

**Resolved:**
- ✅ Simulation-based validation acceptable (no hardware required)
- ✅ Include circuit photos/diagrams (yes, from HB100-assets)
- ✅ Use Hydra for reproducibility (yes, standard practice in ML/signal processing research)

**Not Required:**
- Formal proof of MUSIC convergence (simulation results sufficient)
- Comparison to commercial systems (out of scope)
- Real-time performance benchmarks (not the focus)

---

## Notes for Implementation

- **Code reuse:** Extract MUSIC, EKF, DSP functions from `radar_processor.py` into a shared module that both the real system and simulation can import
- **Figure quality:** Matplotlib + seaborn with custom styling (sans-serif fonts, appropriate colors, grid, legends)
- **Reproducibility:** Every figure filename and config should be named clearly (e.g., `figure_1_angle_error_vs_snr.yaml` → `angle_error_vs_snr.pdf`)
- **Git hygiene:** Commit design doc first, then implement incrementally with clear commits for each phase

