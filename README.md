# Power-Aware Time Scaling for DC-Bus Input-Power Budgeting and Power-Leveling in Multi-Axis Systems

MATLAB programs and trained models supporting the second major revision.

## Requirements

MATLAB R2026a with Deep Learning Toolbox, Robotics System Toolbox,
Signal Processing Toolbox, and Parallel Computing Toolbox.

## Run

Open MATLAB in the repository folder and run:

```matlab
run_all_results
```

This runs the main comparison, learning ablation, robustness, parameter
sensitivity, and filter analysis, then generates the report. Results are saved
under `results/`. Repeating a study reuses completed method trajectories.

To run a small subset of the main study:

```matlab
run_study('main', 1:3)
```

## Program organization

| File or folder | Role |
| --- | --- |
| `run_all_results.m` | Run all manuscript studies and generate the report. |
| `run_study.m` | Run one study or a selected task subset. |
| `reviewer_configuration.m`, `reviewer_conditions.m` | Set models, task parameters, and perturbation conditions. |
| `src/` | Robot and DC-link models, control methods, learning, and metrics. |
| `inputs/` | Frozen task definitions and inputs for evaluation and statistical summaries. |
| `models/r8_eq/` | Four trained networks used for evaluation. |
| `plots/` | MATLAB functions for drawing the numerical results. |
| `compute_report.m` | Summarize saved results and generate tables and figures. |
| `reviewer_computational_workload.m` | Calculate network operation counts and search bounds. |
| `reviewer_train_models.m` | Generate the teacher corpus and reproduce model training. |
| `reviewer_inference.m` | Measure network forward-inference time. |

## Models

| File | Model |
| --- | --- |
| `trf_phy.mat` | Transformer with physical regularization. |
| `mlp_phy.mat` | MLP with physical regularization. |
| `trf_sup.mat` | Supervised Transformer. |
| `mlp_sup.mat` | Supervised MLP. |

The default studies use the supplied networks. Optional training writes new
models under `results/training/`.
