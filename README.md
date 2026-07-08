# Power-Aware Time Scaling for DC-Bus Input-Power Budgeting

This repository contains the MATLAB source code used to support the revised manuscript:

**Power-Aware Time Scaling for DC-Bus Input-Power Budgeting and Power-Leveling in Multi-Axis Systems**

The code implements a digital-twin evaluation pipeline for fixed-path, power-aware time scaling in a multi-axis robotic system. It includes the DC-bus electrical model, the computed-torque inner-loop simulation, feasibility-search/reference-governor baselines, two learned progress-rate governor surrogates, robustness/sensitivity checks, latency decomposition, and manuscript table/figure exporters.

The package is intended for reproducibility of the reported simulation results and for review of the algorithmic implementation. It is not a hardware driver package and does not include physical robot I/O.

---

## 1. What this code reproduces

The scripts support the following components of the revised study:

- A fixed-path time-scaling architecture with a computed-torque inner loop and an outer progress-rate governor.
- A source-limited DC-bus model that distinguishes active sustained budget, hardware source rating, delivered source power, pre-limit requested demand, source shortfall, regenerative sharing, dump power, and energy accounting.
- Teacher-supervised learned PRG surrogates:
  - **MLP-PRG**: non-attention feedforward learned surrogate.
  - **TRF-PRG**: Transformer-based look-ahead sequence learned surrogate.
- Fair baseline comparisons among:
  - **NoGuard**: budget-agnostic reference.
  - **Feasible-RG**: online feasibility-search reference governor.
  - **MLP-PRG**: learned feedforward surrogate.
  - **TRF-PRG**: learned sequence surrogate.
  - **Bus-PTP-DP**: bus-power-constrained path time-parameterization reference.
  - **MPC-RG-grid**: receding-horizon/reference-governor grid-search reference.
- Manuscript-facing tables and figures for power, energy, ramp-rate, fair-baseline trade-offs, latency, robustness, and electrical-parameter sensitivity.

---

## 2. Repository contents

### Main one-click / staged scripts

| File | Purpose |
|---|---|
| `RUN_01_train_models_oneclick.m` | Generates the teacher dataset, trains the TRF-PRG model, and trains the MLP-PRG model from the same dataset. Also runs basic no-NN and one-case sanity checks. |
| `RUN_02_CHUNKED_CONTINUE.m` | Resumable paired-trial evaluator. It first completes the 4-method main batch and then completes the 6-method fair-baseline batch. |
| `RUN_02_CHECK_PROGRESS.m` | Prints progress and output locations for the resumable RUN_02 workflow. |
| `RUN_03_EXPORT_REVISION_RESULTS.m` | Compiles RUN_02 outputs, generates revision tables/figures, runs one-case latency traces, and optionally runs robustness/electrical-sensitivity checks. |
| `RUN_04_EXPORT_MANUSCRIPT_CORE.m` | Creates the compact manuscript-facing output set: main tables, appendix tables, figures, captions, and output index. |

### Core model and controller files

| File group | Purpose |
|---|---|
| `spinn3d_params_block.m` | Central parameter block. This is the main configuration file for timing, budget, bus model, training, payload, and evaluation settings. |
| `spinn3d_params_to_cfg.m`, `get_bus_from_cfg.m`, `spinn3d_bus_defaults.m` | Convert the parameter block to controller/evaluation configuration structures. |
| `spinn3d_robot.m`, `spinn3d_build_robot.m`, `spinn3d_features_alpha_plus*.m` | Robot model construction and feature extraction. |
| `spinn3d_controller_ctcgov.m` | Closed-loop computed-torque controller plus progress-rate governor implementation. |
| `governor_step.m`, `alpha_feasible.m`, `alpha_des_teacher.m` | Deterministic governor and feasibility-search teacher logic. |
| `spinn3d_bus_power_elec.m`, `spinn3d_bus_step_elec.m`, `spinn3d_bus_state_init.m`, `spinn3d_compute_power_metrics.m` | Electrical DC-bus model and telemetry metric computation. |
| `spinn3d_demo_ctcgov_online_*.m` | Closed-loop rollout wrappers for NoGuard, Feasible-RG, MLP-PRG, TRF-PRG, profile, and MPC/RG variants. |
| `spinn3d_bus_ptp_dp.m`, `spinn3d_mpc_rg_grid_step.m` | Bus-PTP-DP and MPC-RG-grid baseline components. |

### Training and evaluation helpers

| File group | Purpose |
|---|---|
| `spinn3d_make_dataset_alpha_gov_geom_seq.m` | Generates governor-labeled look-ahead training samples from the feasibility-search teacher. |
| `spinn3d_train_alpha_gov_trf.m` | Trains the TRF-PRG surrogate. |
| `spinn3d_train_alpha_gov_mlp.m` | Trains the MLP-PRG surrogate. |
| `load_model_alpha_any.m`, `load_model_alpha_strict.m` | Load learned surrogate model files. |
| `paper_power_eval.m`, `paper_fair_baseline_eval.m` | Main paired-trial and fair-baseline evaluation scripts. |
| `paper_policy_latency_trace_onecase.m`, `paper_latency_benchmark.m` | Policy-update latency and inference-decomposition support. |
| `paper_robustness_eval.m`, `paper_electrical_sensitivity_parallel.m`, `paper_budget_filter_policy_sensitivity_parallel.m` | Robustness and sensitivity support scripts. |
| `paper_fig2_representative_3methods_vertical_v2.m`, `paper_fig3_representative_trial.m`, `spinn3d_plot_bus_power_single_lpf.m` | Representative figure generation and power plotting. |

---

## 3. Software requirements

This package is written in MATLAB. The following MATLAB capabilities are used by the scripts:

- Robotics model utilities such as `rigidBodyTree` and inverse dynamics.
- Deep-learning utilities such as `dlnetwork`, `dlarray`, `selfAttentionLayer`, `trainNetwork`, and Adam optimization.
- Optional parallel execution through `parpool` and `parfor`.

Recommended toolboxes:

- Robotics System Toolbox.
- Deep Learning Toolbox.
- Parallel Computing Toolbox, optional but useful for dataset generation and paired-trial evaluation.

No physical robot hardware is required to run the provided digital-twin simulations.

---

## 4. Default study settings

The central settings are defined in `spinn3d_params_block.m`. The defaults correspond to the revised manuscript configuration unless intentionally modified.

Key defaults include:

| Setting | Default |
|---|---:|
| Servo/digital-twin integration period `PB.time.Ts` | `1e-3` s |
| Learned/governor policy update period `PB.policy.dt` | `0.01` s |
| Active sustained grid-input budget `PB.power.P_GRID_BUDGET` | `13` W |
| Nominal source voltage `PB.power.V_BUS_NOMINAL` | `12` V |
| Nominal source current rating `PB.power.I_SUPPLY_RATING` | `5` A |
| Nominal hardware source rating `PB.power.P_SUPPLY_MAX` | `60` W |
| Teacher trajectories | `360` |
| Training budget range | `12.7`-`13.3` W |
| Look-ahead tokens `K` | `81` |
| Look-ahead horizon `H` | `1` |
| Main paired-trial count | `100` |
| Main 4-method seed | `1` |
| Fair 6-method seed | `102` |

If any central parameters, trained-model paths, seeds, budget values, trajectory ranges, or payload ranges are changed after starting an experiment folder, delete the old generated output folders and restart from the first trial. Mixing results from different parameter settings can invalidate the reported tables.

---

## 5. Quick start

Place all `.m` files in one folder, open MATLAB in that folder, and run:

```matlab
cd('path/to/this/repository')
addpath(genpath(pwd))
```

### Step 1: train the learned surrogates

```matlab
RUN_01_train_models_oneclick
```

This creates the teacher dataset and trained model files under `data_alpha/`, and writes logs and sanity-check outputs under `training_outputs_oneclick/`.

Expected key outputs include:

```text
data_alpha/spinn3d_alpha_gov_seq_dataset_master.mat
data_alpha/model_alpha_gov_trf.mat
data_alpha/model_alpha_gov_mlp.mat
training_outputs_oneclick/training_manifest.mat
```

### Step 2: run the paired-trial evaluations

```matlab
RUN_02_CHUNKED_CONTINUE
```

`RUN_02_CHUNKED_CONTINUE.m` is resumable. By default it adds `RUN.add_trials = 10` new paired trials per execution. Run it repeatedly until both stages are complete:

1. `MAIN4`: NoGuard, Feasible-RG, MLP-PRG, TRF-PRG.
2. `FAIR6`: NoGuard, Feasible-RG, Bus-PTP-DP, MPC-RG-grid, MLP-PRG, TRF-PRG.

To check progress:

```matlab
RUN_02_CHECK_PROGRESS
```

Main output folders:

```text
paper_outputs_chunked_100/main_4methods_N100_seed1/
paper_outputs_chunked_100/fair_6methods_N100_seed102/
```

### Step 3: export revision tables, figures, latency, robustness, and sensitivity outputs

```matlab
RUN_03_EXPORT_REVISION_RESULTS
```

Main output folder:

```text
paper_outputs_chunked_100/RUN03_revision_exports/
```

### Step 4: export compact manuscript-facing tables and figures

```matlab
RUN_04_EXPORT_MANUSCRIPT_CORE
```

Main output folder:

```text
paper_outputs_chunked_100/RUN04_manuscript_core_outputs/
```

This folder contains:

```text
tables_main/
tables_appendix/
figures_main/
captions_and_index/
```

---

## 6. Minimal smoke checks

To run a quick NoGuard / Feasible-RG closed-loop check without learned models:

```matlab
PB = spinn3d_params_block();
cfg = spinn3d_params_to_cfg(PB, 'demo_alpha');
cfg.NO_FIGURE = true;
cfg.DISABLE_FIGURES = true;

out_noguard  = spinn3d_demo_ctcgov_online_nonn_nolimit(cfg);
out_feasible = spinn3d_demo_ctcgov_online_nonn(cfg);

disp(out_noguard.t_reach)
disp(out_feasible.t_reach)
```

To run learned-surrogate checks after training:

```matlab
PB = spinn3d_params_block();
cfg = spinn3d_params_to_cfg(PB, 'demo_alpha');
cfg.NO_FIGURE = true;
cfg.DISABLE_FIGURES = true;
cfg.MODEL_PATH = PB.paths.model_alpha;
cfg.MLP_MODEL_PATH = PB.paths.model_alpha_mlp;

out_mlp = spinn3d_demo_ctcgov_online_mlp(cfg);
out_trf = spinn3d_demo_ctcgov_online_nn(cfg);
```

---

## 7. Output interpretation

The most important generated artifacts are:

| Output | Meaning |
|---|---|
| `RUN02_MAIN4_trials_canonical.csv` | Trial-level main 4-method paired evaluation. |
| `RUN02_FAIR6_trials_canonical.csv` | Trial-level 6-method fair-baseline evaluation. |
| `Table3_key_setup_and_learned_surrogates.csv` | Compact training/evaluation setup and learned-surrogate summary. |
| `Table4_main_power_energy_and_ramp.csv` | Main power, energy, and ramp-rate statistics. |
| `Table5_fair_baseline_and_surrogate_comparison.csv` | Fair-baseline comparison. |
| `Table6_robustness_and_sensitivity_summary.csv` | Robustness and electrical-sensitivity summary. |
| `Figure02_representative_telemetry.png` | Representative NoGuard/TRF-PRG/MLP-PRG telemetry. |
| `Figure03_main_metric_reductions.png` | Main paired-trial reductions relative to NoGuard. |
| `Figure04_budget_feasible_vs_overdemanded.png` | Budget-feasible-like versus over-demanded stress-case split. |
| `Figure05_fair_baseline_tradeoff.png` | Fair-baseline reaching-time/exceedance trade-off. |
| `Figure06_latency_decomposition.png` | Policy-update latency decomposition. |
| `Figure07_robustness_exceedance_heatmap.png` | One-case perturbation robustness heatmap. |
| `Figure08_electrical_sensitivity_screening.png` | Electrical-parameter sensitivity screening. |

The code reports both source-stress metrics and energy metrics. Peak power, high-percentile power, exceedance, requested peak demand, ramp-rate, current, voltage, delivered grid energy, reused regenerative energy, and dumped energy are separate quantities and should not be collapsed into a single score.

---

## 8. Important reproducibility notes

1. `RUN_01_train_models_oneclick.m` trains models and generates the teacher dataset. It should be rerun whenever `spinn3d_params_block.m` is changed.
2. `RUN_02_CHUNKED_CONTINUE.m` writes manifests to detect accidental mixing of settings. Do not override manifest mismatch warnings unless you intentionally started a new experiment condition.
3. `RUN_03_EXPORT_REVISION_RESULTS.m` and `RUN_04_EXPORT_MANUSCRIPT_CORE.m` are exporters. They should be run after the required RUN_02 batches are complete.
4. Robustness and electrical-sensitivity scripts are intended as representative digital-twin stress checks and sensitivity screening. They are not a substitute for measured hardware voltage/current validation.
5. The code uses a source-limited engineering bus model initialized from public product-level specifications and engineering assumptions. Full hardware validation would require measured DC-bus voltage/current telemetry.

---

## 9. Suggested repository structure after running all scripts

```text
.
├── README.md
├── RUN_01_train_models_oneclick.m
├── RUN_02_CHUNKED_CONTINUE.m
├── RUN_02_CHECK_PROGRESS.m
├── RUN_03_EXPORT_REVISION_RESULTS.m
├── RUN_04_EXPORT_MANUSCRIPT_CORE.m
├── spinn3d_params_block.m
├── ... other source files ...
├── data_alpha/
│   ├── spinn3d_alpha_gov_seq_dataset_master.mat
│   ├── model_alpha_gov_trf.mat
│   └── model_alpha_gov_mlp.mat
├── training_outputs_oneclick/
└── paper_outputs_chunked_100/
    ├── main_4methods_N100_seed1/
    ├── fair_6methods_N100_seed102/
    ├── RUN03_revision_exports/
    └── RUN04_manuscript_core_outputs/
```

Generated data and model files can be large. They are intentionally separated from source files through the output folders above.

---

## 10. Troubleshooting

### Missing learned model files

If a script reports that `model_alpha_gov_trf.mat` or `model_alpha_gov_mlp.mat` is missing, run:

```matlab
RUN_01_train_models_oneclick
```

### RUN_02 seems slow

`RUN_02_CHUNKED_CONTINUE.m` intentionally runs a paired-trial protocol and may require multiple executions. Reduce or increase the number of new paired trials per execution by editing:

```matlab
RUN.add_trials = 10;
```

### Parallel pool issues

Parallel execution is optional. If parallel workers are unavailable or memory is limited, set:

```matlab
RUN.use_parallel_pool = false;
```

or reduce:

```matlab
RUN.num_workers = [];
```

in the relevant RUN script.

### Results changed after editing parameters

Delete the generated result folders and restart the pipeline from RUN_01. The most common cause of mismatched results is changing `spinn3d_params_block.m` after some trials have already been generated.

---

## 11. Citation

If this code is used, please cite the associated manuscript:

Peizhong Li, Zhenglin Li, Ting Zhu, Zhijie Fang, and Hao Fang, **Power-Aware Time Scaling for DC-Bus Input-Power Budgeting and Power-Leveling in Multi-Axis Systems**.

---

## 12. License

No license file is included in this source package. Before public release, add the license selected by the authors or required by the hosting repository.
