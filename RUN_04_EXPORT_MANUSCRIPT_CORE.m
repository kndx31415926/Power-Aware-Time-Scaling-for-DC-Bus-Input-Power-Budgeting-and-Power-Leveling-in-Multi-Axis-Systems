%% RUN_04_EXPORT_MANUSCRIPT_CORE.m
% Compact manuscript-output exporter.
%
% Purpose
%   After RUN_01, RUN_02, and RUN_03 have completed, this script creates a
%   NEW folder containing a compact set of manuscript-facing tables and figures.
%   It does not rerun the 100 paired trials and does not change controller data.
%   Figure 2 is regenerated from one representative closed-loop rollout
%   using Baseline/NoGuard, TRF-PRG, and MLP-PRG if the helper
%   paper_fig2_representative_3methods_vertical_v2.m is on the path;
%   otherwise RUN04 falls back to the RUN03 representative figure.
%
% Recommended order
%   1) RUN_01_train_models_oneclick
%   2) RUN_02_CHUNKED_CONTINUE until MAIN4 and FAIR6 are complete
%   3) RUN_03_EXPORT_REVISION_RESULTS
%   4) RUN_04_EXPORT_MANUSCRIPT_CORE
%
% Output folder
%   paper_outputs_chunked_100/RUN04_manuscript_core_outputs/
%
% Design principle
%   Main manuscript: fewer tables, more figures.
%   Appendix/supplement: full numeric details for reproducibility.

clear; clc; close all;

RUN = struct();
RUN.project_dir = fileparts(mfilename('fullpath'));
if isempty(RUN.project_dir), RUN.project_dir = pwd; end
RUN.results_root = fullfile(RUN.project_dir,'paper_outputs_chunked_100');
RUN.run03_root = fullfile(RUN.results_root,'RUN03_revision_exports');
RUN.out_root = fullfile(RUN.results_root,'RUN04_manuscript_core_outputs');
RUN.fig_dpi = 450;
RUN.near_budget_threshold = 1.05;   % max filtered normalized power threshold used for split diagnostics
RUN.figure02_force_budget_W = 13;  % nominal active budget used in revised manuscript representative Fig. 2
RUN.figure02_lpf_tau_s = 0.05;     % evaluation LPF time constant used in Fig. 2 power trace
RUN.figure02_rng_seed = 1;         % matched representative trial seed across Baseline/TRF/MLP

cd(RUN.project_dir);
addpath(genpath(RUN.project_dir));
ensure_dir_local(RUN.out_root);
ensure_dir_local(fullfile(RUN.out_root,'tables_main'));
ensure_dir_local(fullfile(RUN.out_root,'tables_appendix'));
ensure_dir_local(fullfile(RUN.out_root,'figures_main'));
ensure_dir_local(fullfile(RUN.out_root,'captions_and_index'));

log_file = fullfile(RUN.out_root, sprintf('RUN_04_log_%s.txt', datestr(now,'yyyymmdd_HHMMSS')));
diary(log_file);
cDiary = onCleanup(@() diary('off')); %#ok<NASGU>

fprintf('\n============================================================\n');
fprintf('RUN_04_EXPORT_MANUSCRIPT_CORE started at %s\n', datestr(now));
fprintf('Project dir: %s\n', RUN.project_dir);
fprintf('Output dir : %s\n', RUN.out_root);
fprintf('============================================================\n');

% -------------------------------------------------------------------------
% Locate required data sources
% -------------------------------------------------------------------------
main_dir = fullfile(RUN.results_root,'main_4methods_N100_seed1');
fair_dir = fullfile(RUN.results_root,'fair_6methods_N100_seed102');

f_main_trials = fullfile(main_dir,'RUN02_MAIN4_trials_canonical.csv');
if exist(f_main_trials,'file')~=2
    f_main_trials = fullfile(main_dir,'RUN02_trials_canonical.csv');
end
f_fair_trials = fullfile(fair_dir,'RUN02_FAIR6_trials_canonical.csv');
if exist(f_fair_trials,'file')~=2
    f_fair_trials = fullfile(fair_dir,'RUN02_trials_canonical.csv');
end
f_fair_summary = fullfile(fair_dir,'RUN02_FAIR6_summary_canonical.csv');
if exist(f_fair_summary,'file')~=2
    f_fair_summary = fullfile(fair_dir,'RUN02_summary_canonical.csv');
end

f_t5 = fullfile(RUN.run03_root,'tables','Table5_learned_surrogate_training_validation.csv');
f_t6 = fullfile(RUN.run03_root,'tables','Table6_learned_surrogate_policy_latency_onecase.csv');
f_t9 = fullfile(RUN.run03_root,'tables','Table9_main_power_energy.csv');
f_t11 = fullfile(RUN.run03_root,'tables','Table11_fair_baseline_with_learned_latency.csv');
f_t12 = fullfile(RUN.run03_root,'tables','Table12_robustness_onecase.csv');
f_elec = fullfile(RUN.run03_root,'electrical_sensitivity','electrical_sensitivity_summary.csv');
f_repr = fullfile(RUN.run03_root,'figures','Fig_representative_trial.png');

assert(exist(f_main_trials,'file')==2, 'MAIN4 canonical trials not found. Run RUN03 first or standardize RUN02 outputs.');
assert(exist(f_fair_trials,'file')==2, 'FAIR6 canonical trials not found. Run RUN03 first or standardize RUN02 outputs.');
assert(exist(f_fair_summary,'file')==2, 'FAIR6 canonical summary not found. Run RUN03 first.');
assert(exist(f_t5,'file')==2, 'RUN03 learned surrogate table not found.');
assert(exist(f_t6,'file')==2, 'RUN03 latency table not found.');
assert(exist(f_t12,'file')==2, 'RUN03 robustness table not found.');

Tmain = readtable(f_main_trials);
Tfair = readtable(f_fair_trials);
Sfair = readtable(f_fair_summary);
Tlearn = readtable(f_t5);
Tlat = readtable(f_t6);
Trob = readtable(f_t12);
if exist(f_elec,'file')==2
    Telec = readtable(f_elec);
else
    Telec = table();
end

Tmain.method = string(Tmain.method);
Tfair.method = string(Tfair.method);
if ismember('method',Sfair.Properties.VariableNames), Sfair.method = string(Sfair.method); end

methods_main = {'NoGuard','Feasible-RG','MLP-PRG','TRF-PRG'};
methods_fair = {'NoGuard','Feasible-RG','Bus-PTP-DP','MPC-RG-grid','MLP-PRG','TRF-PRG'};

% -------------------------------------------------------------------------
% MAIN TABLES
% -------------------------------------------------------------------------
fprintf('\n[RUN04] Writing compact main manuscript tables...\n');

Table1 = make_related_work_template_local();
writetable(Table1, fullfile(RUN.out_root,'tables_main','Table1_related_work_positioning_compact.csv'));

Table2 = make_notation_table_local();
writetable(Table2, fullfile(RUN.out_root,'tables_main','Table2_notation_budget_definitions_compact.csv'));
write_abbreviation_note_local(fullfile(RUN.out_root,'captions_and_index','ABBREVIATION_NOTE_BEFORE_TABLE2.txt'));

Table3 = make_key_setup_table_local(Tmain,Tlearn,Tlat);
writetable(Table3, fullfile(RUN.out_root,'tables_main','Table3_key_setup_and_learned_surrogates.csv'));

Table4 = make_main_results_table_local(Tmain, methods_main);
writetable(Table4, fullfile(RUN.out_root,'tables_main','Table4_main_power_energy_and_ramp.csv'));

Table5 = make_fair_baseline_table_local(Tfair,Sfair,Tlat,methods_fair);
writetable(Table5, fullfile(RUN.out_root,'tables_main','Table5_fair_baseline_and_surrogate_comparison.csv'));

Table6 = make_robust_sensitivity_summary_local(Trob,Telec);
writetable(Table6, fullfile(RUN.out_root,'tables_main','Table6_robustness_and_sensitivity_summary.csv'));

% -------------------------------------------------------------------------
% APPENDIX / SUPPLEMENTARY TABLES
% -------------------------------------------------------------------------
fprintf('[RUN04] Writing appendix/supplementary tables...\n');
copyfile(f_t5, fullfile(RUN.out_root,'tables_appendix','TableA1_full_learned_surrogate_training_validation.csv'));
copyfile(f_t6, fullfile(RUN.out_root,'tables_appendix','TableA2_full_policy_update_latency.csv'));
if exist(f_t9,'file')==2
    copyfile(f_t9, fullfile(RUN.out_root,'tables_appendix','TableA3_RUN03_main_power_energy_full.csv'));
end
if exist(f_t11,'file')==2
    copyfile(f_t11, fullfile(RUN.out_root,'tables_appendix','TableA4_RUN03_fair_baseline_full.csv'));
end
copyfile(f_t12, fullfile(RUN.out_root,'tables_appendix','TableA5_full_robustness_onecase_numeric.csv'));
if exist(f_elec,'file')==2
    copyfile(f_elec, fullfile(RUN.out_root,'tables_appendix','TableA6_electrical_sensitivity_screening_numeric.csv'));
end

SplitTable = make_budget_split_table_local(Tmain, methods_main, RUN.near_budget_threshold);
writetable(SplitTable, fullfile(RUN.out_root,'tables_appendix','TableA7_budget_feasible_and_overdemanded_split.csv'));

% -------------------------------------------------------------------------
% FIGURES
% -------------------------------------------------------------------------
fprintf('[RUN04] Generating compact manuscript figures...\n');

make_figure02_representative_3methods_local(RUN, f_repr);

make_main_improvement_figure_local(Tmain, methods_main, fullfile(RUN.out_root,'figures_main','Figure03_main_metric_reductions.png'), RUN.fig_dpi);
make_budget_split_figure_local(Tmain, methods_main, RUN.near_budget_threshold, fullfile(RUN.out_root,'figures_main','Figure04_budget_feasible_vs_overdemanded.png'), RUN.fig_dpi);
make_fair_tradeoff_figure_local(Tfair, methods_fair, fullfile(RUN.out_root,'figures_main','Figure05_fair_baseline_tradeoff.png'), RUN.fig_dpi);
make_latency_figure_local(Tlat, fullfile(RUN.out_root,'figures_main','Figure06_latency_decomposition.png'), RUN.fig_dpi);
make_robustness_heatmap_local(Trob, fullfile(RUN.out_root,'figures_main','Figure07_robustness_exceedance_heatmap.png'), RUN.fig_dpi);
if ~isempty(Telec)
    make_electrical_sensitivity_figure_local(Telec, fullfile(RUN.out_root,'figures_main','Figure08_electrical_sensitivity_screening.png'), RUN.fig_dpi);
end

% -------------------------------------------------------------------------
% Captions and index
% -------------------------------------------------------------------------
fprintf('[RUN04] Writing caption guide and output index...\n');
write_caption_guide_local(fullfile(RUN.out_root,'captions_and_index','MANUSCRIPT_CORE_CAPTIONS.txt'));
Index = make_output_index_local(RUN.out_root);
writetable(Index, fullfile(RUN.out_root,'captions_and_index','MANUSCRIPT_CORE_OUTPUT_INDEX.csv'));

fprintf('\nRUN_04 complete. Compact outputs are in:\n  %s\n', RUN.out_root);
fprintf('Use tables_main/ for the reduced main-text tables and tables_appendix/ for supporting details.\n');
fprintf('Use figures_main/ for manuscript figures; rewrite captions using MANUSCRIPT_CORE_CAPTIONS.txt.\n');
fprintf('============================================================\n');

%% ========================================================================
% Local functions
% ========================================================================
function ensure_dir_local(p)
if exist(p,'dir')~=7, mkdir(p); end
end

function make_figure02_representative_3methods_local(RUN, f_repr)
% Generate the revised Fig. 2 directly in RUN04.
% This call creates a three-method representative telemetry figure:
%   row 1 = Baseline/NoGuard, row 2 = TRF-PRG, row 3 = MLP-PRG.
% If the helper function or trained models are unavailable, the old RUN03
% representative figure is copied as a fallback so RUN04 can still finish.
out_png = fullfile(RUN.out_root,'figures_main','Figure02_representative_telemetry.png');
out_fig = fullfile(RUN.out_root,'figures_main','Figure02_representative_telemetry.fig');
helper_name = 'paper_fig2_representative_3methods_vertical_v2';

if exist(helper_name,'file')==2
    try
        fprintf('[RUN04] Generating Figure02 representative telemetry with Baseline, TRF-PRG, and MLP-PRG...\n');
        PB = spinn3d_params_block();
        cfg = spinn3d_params_to_cfg(PB, 'demo_alpha');

        opts = struct();
        opts.save_png = out_png;
        opts.save_fig = out_fig;
        opts.force_budget_W = RUN.figure02_force_budget_W;
        opts.pgrid_tau_s = RUN.figure02_lpf_tau_s;
        opts.rng_seed = RUN.figure02_rng_seed;
        opts.close_demo_figs = true;
        opts.show_repeated_legends = false;
        opts.bus_ylim_focus_lpf = true;
        opts.joint_ylim_focus_quant = [];
        opts.fig_name = 'Figure02_representative_telemetry';
        opts.save_dpi = RUN.fig_dpi;        % supported by the updated helper; ignored by older helper versions
        opts.figure_visible = 'off';        % supported by the updated helper; ignored by older helper versions

        fig = paper_fig2_representative_3methods_vertical_v2(cfg, opts);
        if ~isempty(fig) && isvalid(fig)
            try, close(fig); catch, end
        end
        fprintf('[RUN04] Figure02 generated: %s\n', out_png);
        return;
    catch ME
        warning('[RUN04] Figure02 3-method generation failed: %s', ME.message);
        warning('[RUN04] Falling back to RUN03 representative figure if available.');
    end
else
    warning('[RUN04] Helper not found on path: %s.m', helper_name);
    warning('[RUN04] Falling back to RUN03 representative figure if available.');
end

if exist(f_repr,'file')==2
    copyfile(f_repr, out_png);
    fprintf('[RUN04] Copied fallback Figure02 from RUN03: %s\n', out_png);
else
    warning('[RUN04] RUN03 representative figure not found; Figure02 skipped.');
end
end

function write_abbreviation_note_local(path)
fid = fopen(path,'w');
assert(fid>0,'Could not write abbreviation note: %s', path);
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'Suggested abbreviation note before the notation table\n');
fprintf(fid,'===================================================\n');
fprintf(fid,'For clarity, the main abbreviations used in the manuscript are: DC-bus, direct-current bus; CTC, computed-torque control; PRG, progress-rate governor; RG, reference governor; MLP-PRG, multilayer-perceptron progress-rate governor; TRF-PRG, Transformer-based progress-rate governor; Bus-PTP-DP, bus-power-constrained path time-parameterization by dynamic programming; and MPC-RG-grid, receding-horizon/grid-search reference governor.\n');
end


function T = make_related_work_template_local()
Category = [
    "Energy-aware robot control";
    "Path time-parameterization and trajectory scaling";
    "DC-bus sharing, stability, and regeneration";
    "MPC, NMPC, and reference governors";
    "Learning-based control and sequence models";
    "This work"
];
RepresentativeIdea = [
    "Energy-efficient or energy-prioritized motion in hydraulic, aerial, and mobile robotic systems.";
    "Fixed-path timing laws enforce kinematic, acceleration, or torque constraints.";
    "Common-bus studies address energy exchange, voltage stability, and braking-energy handling.";
    "Optimization or reference management enforces constraints around a stabilized inner loop.";
    "Neural and sequence models can support control decisions when embedded with domain knowledge.";
    "DC-bus input-power budgeting for fixed-path robotic time scaling with two proposed learned PRG surrogates."
];
DCbusSourceModel = ["No"; "No"; "Yes"; "Partial"; "No/partial"; "Yes"];
RegenDumpHandling = ["Partial"; "Partial"; "Yes"; "Partial"; "No"; "Yes"];
LearnedSurrogate = ["No"; "No"; "No"; "No"; "Yes/partial"; "Yes"];
LatencyRobustness = ["Limited"; "Limited"; "Partial"; "Partial"; "Partial"; "Yes"];
FairBaselineCoverage = ["Limited"; "Partial"; "Limited"; "Partial"; "Limited"; "Yes"];
GapAddressed = [
    "Moves from total-energy emphasis to supply-side input-power budgeting and source-stress reduction.";
    "Extends path timing control to source-limited DC-bus power, regeneration, dump limits, and electrical diagnostics.";
    "Connects bus-side power quantities to manipulator progress-rate control rather than treating them only as drive-level variables.";
    "Uses learned surrogate inference plus deterministic governor post-processing to reduce online search in the learned variants.";
    "Uses bus-aware teacher labels, fixed-path preview features, and conservative post-processing instead of unconstrained end-to-end control.";
    "Reports energy accounting, ramp-rate behavior, latency, robustness, sensitivity, and fair baselines under shared telemetry definitions."
];
T = table(Category,RepresentativeIdea,DCbusSourceModel,RegenDumpHandling,LearnedSurrogate,LatencyRobustness,FairBaselineCoverage,GapAddressed);
end


function T = make_notation_table_local()
Symbol = [
    "q, qdot, qddot"; "s, Sf"; "a"; "a_des, a_post"; "Ts, Tpol"; ...
    "P_j"; "P_plus, P_minus"; "P_grid"; "P_grid_req"; "P_src_short"; ...
    "P_dump, P_spill"; "P_grid,bud"; "P_hw"; "P_grid,pk"; "rho"; "pbar"; ...
    "E_grid, E_reuse, E_dump"; "E_excess"
];
Unit = [
    "rad, rad/s, rad/s^2";
    "path unit";
    "path unit/s";
    "path unit/s";
    "s";
    "W";
    "W";
    "W";
    "W";
    "W";
    "W";
    "W";
    "W";
    "W";
    "-";
    "-";
    "J";
    "s"
];
Meaning = [
    "Joint position, velocity, and acceleration.";
    "Geometric path progress and terminal path parameter.";
    "Scalar progress rate commanded by the outer loop.";
    "Desired progress-rate command and applied command after deterministic rate limiting/filtering.";
    "Servo integration period and learned-policy update period.";
    "Mechanical power of joint j.";
    "Total positive mechanical power and available regenerative mechanical power.";
    "Delivered source-side DC-bus input power after reuse and source-current limiting.";
    "Pre-limit electrical demand requested by the servo/bus model.";
    "Unmet source-side demand after current limiting.";
    "Power dissipated by the braking branch and regenerative power not reused/dissipated.";
    "Active sustained grid-input budget used for normalization and evaluation.";
    "Hardware source rating based on nominal voltage and current rating.";
    "Instantaneous delivered-power guard.";
    "Evaluation tolerance for normalized budget exceedance.";
    "Low-pass filtered normalized delivered grid-input power.";
    "Integrated delivered grid energy, reused regenerative energy, and dumped energy.";
    "Integrated normalized budget exceedance area."
];
T = table(Symbol,Unit,Meaning);
end

function T = make_key_setup_table_local(Tmain,Tlearn,Tlat)
active_budget = safe_mean_local(Tmain,'P_grid_budget_W');
if ~isfinite(active_budget), active_budget = NaN; end
Nmain = numel(unique(Tmain.trial_id));
servo_ms = NaN; policy_ms = NaN;
try
    PB = spinn3d_params_block();
    servo_ms = PB.time.Ts*1e3;
    policy_ms = PB.policy.dt*1e3;
    hw_rating = PB.power.P_SUPPLY_MAX;
    vnom = PB.power.V_BUS_NOMINAL;
    irating = PB.power.I_SUPPLY_RATING;
catch
    hw_rating = 60; vnom = 12; irating = 5;
end
Rows = {};
Rows(end+1,:) = {"DC-bus budget", "Active sustained input-power budget", fmt_num_unit_local(active_budget,'W')};
Rows(end+1,:) = {"DC-bus budget", "Hardware source rating", sprintf('%.0f V x %.0f A = %.0f W', vnom, irating, hw_rating)};
Rows(end+1,:) = {"Timing", "Servo integration period", fmt_num_unit_local(servo_ms,'ms')};
Rows(end+1,:) = {"Timing", "Policy update period", fmt_num_unit_local(policy_ms,'ms')};
Rows(end+1,:) = {"Evaluation", "Main paired trials", sprintf('%d paired trials', Nmain)};
Rows(end+1,:) = {"Evaluation", "Near-budget split threshold", 'max filtered normalized power <= 1.05'};

for m = ["MLP-PRG","TRF-PRG"]
    idx = find(string(Tlearn.Method)==m,1);
    if ~isempty(idx)
        val = sprintf('MAE %.4g, RMSE %.4g, MaxAE %.4g; %s params, %.2f MB', ...
            getnum(Tlearn,'Validation_MAE',idx), getnum(Tlearn,'Validation_RMSE',idx), getnum(Tlearn,'Validation_MaxAE',idx), ...
            fmt_int_local(getnum(Tlearn,'Model_parameters',idx)), getnum(Tlearn,'Model_memory_MB',idx));
        Rows(end+1,:) = {"Learned surrogate", char(m), val}; %#ok<AGROW>
    end
end

for m = ["MLP-PRG","TRF-PRG"]
    p99inf = latency_component_local(Tlat,m,"Neural inference",'p99_us')/1000;
    p99tot = latency_component_local(Tlat,m,"Total learned-policy update",'p99_us')/1000;
    if isfinite(p99inf)
        Rows(end+1,:) = {"Latency", char(m + " neural inference p99"), sprintf('%.2f ms', p99inf)}; %#ok<AGROW>
    end
    if isfinite(p99tot)
        Rows(end+1,:) = {"Latency", char(m + " full prototype update p99"), sprintf('%.1f ms; feature construction dominates', p99tot)}; %#ok<AGROW>
    end
end

T = cell2table(Rows,'VariableNames',{'Block','Item','Value'});
end

function T = make_main_results_table_local(Tmain, methods)
Metric = {
    'Goal reaching time, t_reach (s)';
    'Peak filtered normalized power, max pbar (-)';
    '99th percentile filtered normalized power, p99(pbar) (-)';
    'Normalized exceedance energy, E_excess (s)';
    'Ramp-rate upper tail, p95(|dpbar/dt|) (s^-1)';
    'Delivered grid energy, E_grid (J)';
    'Reused regenerative energy, E_reuse (J)';
    'Dumped energy, E_dump (J)';
    'Requested peak power, max P_grid_req (W)';
    'Peak source current, I_peak (A)';
    'Minimum DC-link voltage, Vdc_min (V; larger is better)'};
Var = {'t_reach_s','pbar_peak_norm','pbar_p99_norm','E_excess_norm_s','pbar_ramp_p95_per_s', ...
    'E_grid_J','E_reuse_J','E_dump_J','P_grid_req_peak_W','I_grid_peak_A','Vdc_min_V'};
T = table(Metric,'VariableNames',{'Metric'});
for k=1:numel(methods)
    m = methods{k};
    vals = strings(numel(Var),1);
    for r=1:numel(Var)
        [mu,sd] = mean_std_method_local(Tmain,m,Var{r});
        vals(r) = fmt_pm_local(mu,sd,Var{r});
    end
    T.(safe_name_local(m)) = vals;
end
end

function T = make_fair_baseline_table_local(Tfair,Sfair,Tlat,methods)
Method = strings(numel(methods),1);
Role = strings(numel(methods),1);
t_reach = strings(numel(methods),1);
E_excess = strings(numel(methods),1);
E_grid = strings(numel(methods),1);
Ramp_p95 = strings(numel(methods),1);
Latency_note = strings(numel(methods),1);
for i=1:numel(methods)
    m = string(methods{i});
    Method(i) = m;
    Role(i) = method_role_sentence_local(m);
    t_reach(i) = fmt_pm_summary_local(Sfair,m,'t_reach');
    E_excess(i) = fmt_pm_summary_local(Sfair,m,'E_excess_norm_s');
    E_grid(i) = fmt_pm_summary_local(Sfair,m,'E_grid');
    Ramp_p95(i) = fmt_pm_summary_local(Sfair,m,'pbar_ramp_p95');
    if any(m == ["MLP-PRG","TRF-PRG"])
        inf_ms = latency_component_local(Tlat,m,"Neural inference",'p99_us')/1000;
        tot_ms = latency_component_local(Tlat,m,"Total learned-policy update",'p99_us')/1000;
        Latency_note(i) = sprintf('inference p99 %.2f ms; full prototype p99 %.1f ms', inf_ms, tot_ms);
    elseif m == "Bus-PTP-DP"
        Latency_note(i) = "offline profile; online lookup";
    else
        Latency_note(i) = "not neural inference";
    end
end
T = table(Method,Role,t_reach,E_excess,E_grid,Ramp_p95,Latency_note, ...
    'VariableNames',{'Method','Role','t_reach','E_excess','E_grid','Ramp_p95','Latency_note'});
end

function T = make_robust_sensitivity_summary_local(Trob,Telec)
Rows = {};
if ~isempty(Trob)
    all_success = all(Trob.MLP_PRG_success==1 & Trob.TRF_PRG_success==1);
    Rows(end+1,:) = {'Operating perturbations','friction, payload mass/CoM, sensing, voltage, budget, combined stress', ...
        bool_text_local(all_success), 'Both learned surrogates completed all one-case stress tests; isolated perturbations generally reduced normalized exceedance relative to NoGuard.'};
    [~,idxWorst] = max(Trob.Worst_exceedance_norm_s);
    Rows(end+1,:) = {'Most severe robustness case', char(string(Trob.Perturbation(idxWorst))), ...
        sprintf('worst E_excess %.3f s', Trob.Worst_exceedance_norm_s(idxWorst)), ...
        'Combined or over-demanded perturbations should be interpreted as stress cases rather than guaranteed budget-feasible cases.'};
end
if ~isempty(Telec)
    methods = {"NoGuard","Feasible-RG","MLP-PRG","TRF-PRG"};
    msg = strings(numel(methods),1);
    for i=1:numel(methods)
        ep = metric_from_elec_local(Telec,methods{i},'E_excess_norm_s');
        pk = metric_from_elec_local(Telec,methods{i},'pbar_peak');
        msg(i) = sprintf('%s: peak %.2f, Eex %.3f', methods{i}, pk, ep);
    end
    Rows(end+1,:) = {'Electrical-parameter screening','efficiency, voltage/source resistance, DC-link, braking/loss parameters', ...
        'screening result', strjoin(msg,'; ')};
end
Rows(end+1,:) = {'Interpretation','electrical model','sensitivity-supported', ...
    'The bus model is specification-initialized; measured voltage/current telemetry would be required for hardware validation.'};
T = cell2table(Rows,'VariableNames',{'Category','Coverage','Evidence','Main_observation'});
end

function T = make_budget_split_table_local(Tmain, methods, thr)
F = Tmain(Tmain.method=="Feasible-RG",:);
feas_ids = unique(F.trial_id(F.pbar_peak_norm <= thr));
all_ids = unique(Tmain.trial_id);
hard_ids = setdiff(all_ids,feas_ids);
subsets = {feas_ids,'Budget-feasible-like'; hard_ids,'Over-demanded stress cases'; all_ids,'All trials'};
Vars = {'pbar_peak_norm','pbar_p99_norm','E_excess_norm_s','t_reach_s','P_grid_req_peak_W'};
Metric = {'max pbar','p99(pbar)','E_excess','t_reach','max P_grid_req'};
Rows = {};
for s=1:size(subsets,1)
    ids = subsets{s,1}; label = subsets{s,2};
    for m=1:numel(methods)
        M = Tmain(Tmain.method==string(methods{m}) & ismember(Tmain.trial_id,ids),:);
        for v=1:numel(Vars)
            x = M.(Vars{v}); x = x(isfinite(x));
            Rows(end+1,:) = {label, methods{m}, Metric{v}, numel(x), mean(x,'omitnan'), std(x,'omitnan')}; %#ok<AGROW>
        end
    end
end
T = cell2table(Rows,'VariableNames',{'Subset','Method','Metric','N','Mean','SD'});
end


function make_main_improvement_figure_local(Tmain, methods, out_png, dpi)
ref = "NoGuard";
learned = {"Feasible-RG","MLP-PRG","TRF-PRG"};
vars = {'pbar_peak_norm','pbar_p99_norm','E_excess_norm_s','pbar_ramp_p95_per_s','P_grid_req_peak_W','I_grid_peak_A'};
labels = {'Peak pbar','p99 pbar','E_{excess}','Ramp p95','Req. peak power','Peak current'};
Y = nan(numel(vars),numel(learned));
E = nan(numel(vars),numel(learned));
for i=1:numel(vars)
    for j=1:numel(learned)
        [mu,ci95] = paired_reduction_stats_local(Tmain,ref,learned{j},vars{i});
        Y(i,j) = mu;
        E(i,j) = ci95;
    end
end
fig = figure('Visible','off','Color','w','Position',[100 100 1150 560]);
bh = bar(Y,'grouped'); hold on;
add_grouped_errorbars_local(Y,E,[0.20 0.20 0.20]);
yline(0,'k-','LineWidth',0.8);
grid on; box off;
set(gca,'XTick',1:numel(labels),'XTickLabel',labels,'XTickLabelRotation',22,'FontName','Arial','FontSize',10.5,'TickLabelInterpreter','tex');
ylabel('Reduction relative to budget-agnostic reference (%)');
legend(cellstr(learned),'Location','northoutside','Orientation','horizontal','Box','off');
title('Power-stress reductions in the 100 paired-trial evaluation','FontWeight','bold');
try
    ylim([min(0,min(Y(:)-E(:),[],'omitnan')-3), max(Y(:)+E(:),[],'omitnan')+5]);
catch
end
save_fig_local(fig,out_png,dpi); close(fig);
end


function make_budget_split_figure_local(Tmain, methods, thr, out_png, dpi)
F = Tmain(Tmain.method=="Feasible-RG",:);
feas_ids = unique(F.trial_id(F.pbar_peak_norm <= thr));
all_ids = unique(Tmain.trial_id);
hard_ids = setdiff(all_ids,feas_ids);
subsets = {feas_ids,'Budget-feasible-like'; hard_ids,'Over-demanded stress cases'};
metric_vars = {'pbar_peak_norm','E_excess_norm_s'};
metric_labels = {'Mean peak filtered budget ratio','Mean normalized exceedance energy (s)'};

% Fig. 4 is inserted relatively small in the manuscript.  Use shorter tick
% labels and keep all annotations away from the grouped bars.  The full
% subset names remain in the caption/text and in TableA7.
short_subset_labels = {'Feasible-like','Stress cases'};

fig = figure('Visible','off','Color','w','Position',[100 100 1320 620]);
tl = tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');

for p=1:2
    ax = nexttile(tl,p);
    Y = nan(size(subsets,1),numel(methods));
    E = nan(size(subsets,1),numel(methods));
    for s=1:size(subsets,1)
        ids = subsets{s,1};
        for m=1:numel(methods)
            M = Tmain(Tmain.method==string(methods{m}) & ismember(Tmain.trial_id,ids),:);
            x = M.(metric_vars{p}); x = x(isfinite(x));
            Y(s,m) = mean(x,'omitnan');
            E(s,m) = std(x,'omitnan');
        end
    end

    bar(ax,Y,'grouped','BarWidth',0.72); hold(ax,'on');
    axes(ax); %#ok<LAXES>  % keep legacy errorbar helper on the correct axes
    add_grouped_errorbars_local(Y,E,[0.25 0.25 0.25]);
    grid(ax,'on'); box(ax,'off');

    set(ax,'XTick',1:2,'XTickLabel',short_subset_labels, ...
        'XTickLabelRotation',0,'TickLabelInterpreter','none', ...
        'FontName','Arial','FontSize',10.0,'LineWidth',0.85, ...
        'TickDir','out','Layer','top');
    xlim(ax,[0.45 2.72]);
    ylabel(ax,metric_labels{p},'FontName','Arial','FontSize',10.5);
    title(ax,metric_labels{p},'FontWeight','bold','FontSize',11.0);

    finite_top = max(Y(:)+E(:),[],'omitnan');
    finite_bot = min(Y(:)-E(:),[],'omitnan');
    if ~isfinite(finite_top), finite_top = 1; end
    if ~isfinite(finite_bot), finite_bot = 0; end

    if p==1
        ylim(ax,[0 max(finite_top*1.12, thr*1.35)]);
        yline(ax,thr,'k--','LineWidth',0.85,'HandleVisibility','off');

        % Place the threshold label in the right margin, outside the purple
        % bar/error-bar region.  The x-limit above intentionally reserves
        % this blank space.
        yl = ylim(ax); yr = diff(yl);
        text(ax,2.50,thr+0.020*yr,sprintf('%.2f split',thr), ...
            'HorizontalAlignment','left','VerticalAlignment','bottom', ...
            'FontName','Arial','FontSize',9.4,'Color',[0.15 0.15 0.15], ...
            'BackgroundColor','w','Margin',1.2,'Clipping','off');
    else
        pad = 0.12*max(1e-9,finite_top-finite_bot);
        ylim(ax,[min(-0.05,finite_bot-pad) finite_top+pad]);
        legend(ax,methods,'Location','northoutside','Orientation','horizontal', ...
            'Box','off','FontSize',8.6,'Interpreter','none');
    end
end

save_fig_local(fig,out_png,dpi); close(fig);
end

function make_fair_tradeoff_figure_local(Tfair, methods, out_png, dpi)
% Publication-style paired trade-off plot.  This deliberately uses the same
% information layout as the paired-trial trade-off plots: each faint gray
% segment links the same trial under the budget-agnostic reference and a
% compared method.  Unlike the earlier bubble plot, no labels are placed on
% top of data points and no program/file names appear in the title.
required = {'trial_id','method','t_reach_s','E_excess_norm_s'};
for rr = 1:numel(required)
    assert(ismember(required{rr},Tfair.Properties.VariableNames), ...
        'FAIR6 trials table does not contain required variable: %s', required{rr});
end

panel_groups = {
    {'NoGuard','Feasible-RG','MLP-PRG','TRF-PRG'}, 'Learned surrogates and feasibility reference';
    {'NoGuard','MPC-RG-grid','Bus-PTP-DP','Feasible-RG'}, 'Optimization and path-timing references'};

fig = figure('Visible','off','Color','w','Position',[100 100 1280 560]);
tl = tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');

for p = 1:size(panel_groups,1)
    group = panel_groups{p,1};
    nexttile; hold on;
    ids = common_trial_ids_local(Tfair, group);
    [x0,y0,ids0] = xy_for_method_local(Tfair,'NoGuard',ids);

    % Paired gray segments from NoGuard to each compared method.
    for j = 2:numel(group)
        [xj,yj,idsj] = xy_for_method_local(Tfair,group{j},ids);
        ids_pair = intersect(ids0,idsj);
        [x0p,y0p] = xy_for_method_given_ids_local(Tfair,'NoGuard',ids_pair);
        [xjp,yjp] = xy_for_method_given_ids_local(Tfair,group{j},ids_pair);
        for k = 1:numel(ids_pair)
            if all(isfinite([x0p(k),xjp(k),y0p(k),yjp(k)]))
                plot([x0p(k),xjp(k)],[y0p(k),yjp(k)],'-','Color',[0.78 0.78 0.78], ...
                    'LineWidth',0.55,'HandleVisibility','off');
            end
        end
    end

    % Scatter all trial points plus a larger diamond for the mean.
    h = gobjects(numel(group),1);
    for j = 1:numel(group)
        m = group{j};
        [x,y,~] = xy_for_method_local(Tfair,m,ids);
        c = method_color_local(m);
        mk = method_marker_local(m);
        h(j) = plot(x,y,mk,'LineStyle','none','MarkerSize',4.8, ...
            'MarkerFaceColor',c,'MarkerEdgeColor',c,'Color',c, ...
            'DisplayName',m);
        plot(mean(x,'omitnan'),mean(y,'omitnan'),'d','MarkerSize',9, ...
            'MarkerFaceColor',c,'MarkerEdgeColor','k','LineWidth',0.8, ...
            'HandleVisibility','off');
    end

    grid on; box off;
    xlabel('Reaching time (s)');
    ylabel('Normalized exceedance energy (s)');
    title(panel_groups{p,2},'FontWeight','bold','FontSize',12);
    set(gca,'FontName','Arial','FontSize',10,'LineWidth',0.9);
    axis_padded_local(gca);
    legend(h, group,'Location','northoutside','Orientation','horizontal','Box','off','FontSize',9);
end

try
    title(tl,'Paired trade-off between completion time and budget exceedance', ...
        'FontWeight','bold','FontSize',13);
catch
    sgtitle('Paired trade-off between completion time and budget exceedance', ...
        'FontWeight','bold','FontSize',13);
end
save_fig_local(fig,out_png,dpi); close(fig);
end


function make_latency_figure_local(Tlat, out_png, dpi)
methods = {"MLP-PRG","TRF-PRG"};
components = {"Feature construction","Neural inference","Governor update"};
Y = nan(numel(methods),numel(components));
Inf = nan(numel(methods),1);
for i=1:numel(methods)
    for j=1:numel(components)
        Y(i,j) = latency_component_local(Tlat,methods{i},components{j},'p99_us')/1000;
    end
    Inf(i) = latency_component_local(Tlat,methods{i},"Neural inference",'p99_us')/1000;
end
fig = figure('Visible','off','Color','w','Position',[100 100 1180 520]);
subplot(1,2,1);
bar(Y,'stacked'); hold on; yline(10,'k--','10 ms policy target','LineWidth',1);
grid on; box off;
set(gca,'XTick',1:numel(methods),'XTickLabel',cellstr(methods),'TickLabelInterpreter','none','FontName','Arial','FontSize',10.5);
xlim([0.5 numel(methods)+0.5]);
ylabel('p99 latency (ms)'); title('Full prototype policy update','FontWeight','bold');
legend(components,'Location','northoutside','Orientation','horizontal','Box','off');
subplot(1,2,2);
bar(Inf); hold on; yline(10,'k--','10 ms policy target','LineWidth',1);
grid on; box off;
set(gca,'XTick',1:numel(methods),'XTickLabel',cellstr(methods),'TickLabelInterpreter','none','FontName','Arial','FontSize',10.5);
xlim([0.5 numel(methods)+0.5]); ylim([0 max(11,max(Inf)*1.18)]);
ylabel('p99 neural inference latency (ms)'); title('Neural forward inference only','FontWeight','bold');
for i=1:numel(Inf)
    if isfinite(Inf(i))
        text(i,Inf(i)+0.25,sprintf('%.2f',Inf(i)),'HorizontalAlignment','center','FontSize',9,'FontName','Arial');
    end
end
save_fig_local(fig,out_png,dpi); close(fig);
end


function make_robustness_heatmap_local(Trob, out_png, dpi)
methods = {'NoGuard','MLP-PRG','TRF-PRG'};
vars = {'NoGuard_E_excess_norm_s','MLP_PRG_E_excess_norm_s','TRF_PRG_E_excess_norm_s'};
M = nan(height(Trob),numel(vars));
for j=1:numel(vars), M(:,j) = Trob.(vars{j}); end
fig = figure('Visible','off','Color','w','Position',[100 100 930 540]);
imagesc(M); colormap(parula); cb = colorbar; ylabel(cb,'Normalized exceedance energy (s)');
set(gca,'XTick',1:numel(methods),'XTickLabel',methods,'YTick',1:height(Trob),'YTickLabel',cellstr(string(Trob.Perturbation)), ...
    'FontName','Arial','FontSize',10,'TickLabelInterpreter','none');
xtickangle(20);
title('One-case robustness stress tests (combined perturbations are limiting)','FontWeight','bold');
for i=1:size(M,1)
    for j=1:size(M,2)
        txtColor = 'w';
        if isfinite(M(i,j)) && M(i,j)>0.75, txtColor = 'k'; end
        text(j,i,sprintf('%.2f',M(i,j)),'HorizontalAlignment','center','Color',txtColor,'FontWeight','bold','FontSize',9);
    end
end
% Highlight the combined-perturbation row when present.
idxCombined = find(contains(lower(string(Trob.Perturbation)),'combined'),1,'last');
if ~isempty(idxCombined)
    rectangle('Position',[0.5 idxCombined-0.5 numel(vars) 1],'EdgeColor','w','LineWidth',1.5);
end
save_fig_local(fig,out_png,dpi); close(fig);
end


function make_electrical_sensitivity_figure_local(Telec, out_png, dpi)
methods = {"NoGuard","Feasible-RG","MLP-PRG","TRF-PRG"};
peak = nan(numel(methods),1); ex = peak;
for i=1:numel(methods)
    peak(i) = metric_from_elec_local(Telec,methods{i},'pbar_peak');
    ex(i) = metric_from_elec_local(Telec,methods{i},'E_excess_norm_s');
end
fig = figure('Visible','off','Color','w','Position',[100 100 1100 470]);
subplot(1,2,1); bar(peak); grid on; box off;
set(gca,'XTick',1:numel(methods),'XTickLabel',cellstr(methods),'XTickLabelRotation',22,'TickLabelInterpreter','none','FontName','Arial','FontSize',10);
ylabel('Mean peak filtered budget ratio'); title('Electrical sensitivity screening: peak','FontWeight','bold');
ylim([0 max(peak)*1.18]);
for i=1:numel(peak)
    if isfinite(peak(i))
        text(i,peak(i)+0.04*max(peak),sprintf('%.2f',peak(i)),'HorizontalAlignment','center','FontSize',9,'FontName','Arial');
    end
end
subplot(1,2,2); bar(ex); grid on; box off;
set(gca,'XTick',1:numel(methods),'XTickLabel',cellstr(methods),'XTickLabelRotation',22,'TickLabelInterpreter','none','FontName','Arial','FontSize',10);
ylabel('Mean normalized exceedance energy (s)'); title('Electrical sensitivity screening: exceedance','FontWeight','bold');
ylim([0 max(ex)*1.18]);
for i=1:numel(ex)
    if isfinite(ex(i))
        text(i,ex(i)+0.04*max(ex),sprintf('%.3f',ex(i)),'HorizontalAlignment','center','FontSize',9,'FontName','Arial');
    end
end
save_fig_local(fig,out_png,dpi); close(fig);
end

function write_caption_guide_local(path)
fid = fopen(path,'w');
assert(fid>0,'Could not write caption guide: %s', path);
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'Compact manuscript tables and figures generated by RUN_04\n');
fprintf(fid,'========================================================\n\n');
fprintf(fid,'Suggested abbreviation note\n');
fprintf(fid,'Use ABBREVIATION_NOTE_BEFORE_TABLE2.txt before the notation table if the main text needs a compact abbreviation declaration.\n\n');
fprintf(fid,'Suggested main-text tables\n');
fprintf(fid,'1. Table 1: Representative related work and positioning.\n');
fprintf(fid,'2. Table 2: Notation and units used for DC-bus budgeting, time scaling, and evaluation.\n');
fprintf(fid,'3. Table 3: Key experimental settings and learned-surrogate summary.\n');
fprintf(fid,'4. Table 4: Main paired-trial power, energy, and ramp-rate statistics.\n');
fprintf(fid,'5. Table 5: Fair-baseline and learned-surrogate comparison.\n');
fprintf(fid,'6. Table 6: Robustness and electrical-sensitivity summary.\n\n');
fprintf(fid,'Suggested main-text figures\n');
fprintf(fid,'Figure 1: Conceptual architecture and two-time-scale workflow. Use a manually drawn/vector figure, not a generated image.\n');
fprintf(fid,'Figure 2: Representative closed-loop telemetry for Baseline/NoGuard, TRF-PRG, and MLP-PRG under the same 13 W active budget; rows are methods and columns are EE trajectory/skeleton, DC-bus input power, and per-joint mechanical power.\n');
fprintf(fid,'Figure 3: Percentage reductions relative to the budget-agnostic reference with paired 95%% confidence intervals.\n');
fprintf(fid,'Figure 4: Budget-feasible-like versus over-demanded stress cases with standard-deviation error bars.\n');
fprintf(fid,'Figure 5: Fair-baseline trade-off between completion time, exceedance, and delivered grid energy.\n');
fprintf(fid,'Figure 6: Policy-update latency decomposition showing that neural inference is not the main bottleneck.\n');
fprintf(fid,'Figure 7: One-case robustness heatmap for normalized exceedance under perturbations; the combined perturbation row is highlighted.\n');
fprintf(fid,'Figure 8: Electrical-parameter sensitivity screening. Consider moving this figure to the appendix if space is tight.\n\n');
fprintf(fid,'Important interpretation notes\n');
fprintf(fid,'- MLP-PRG and TRF-PRG are both proposed learned PRG surrogates.\n');
fprintf(fid,'- Do not describe MLP-PRG as an external baseline.\n');
fprintf(fid,'- Do not claim strict budget satisfaction in all trials; retain the over-demanded stress-case interpretation.\n');
fprintf(fid,'- Do not use full closed-loop simulation wall time as neural inference latency.\n');
fprintf(fid,'- Current latency evidence supports neural inference at the policy period; feature construction is the prototype bottleneck.\n');
end

function Index = make_output_index_local(out_root)
files = {};
roles = {};
subdirs = {'tables_main','tables_appendix','figures_main','captions_and_index'};
for s=1:numel(subdirs)
    dd = fullfile(out_root,subdirs{s});
    L = dir(dd);
    for i=1:numel(L)
        if ~L(i).isdir
            files{end+1,1} = fullfile(subdirs{s},L(i).name); %#ok<AGROW>
            roles{end+1,1} = output_role_local(subdirs{s},L(i).name); %#ok<AGROW>
        end
    end
end
Index = table(string(files),string(roles),'VariableNames',{'RelativePath','RecommendedUse'});
end

function role = output_role_local(subdir,name)
if strcmp(subdir,'tables_main')
    role = 'candidate main-text table';
elseif strcmp(subdir,'tables_appendix')
    role = 'appendix or supplementary table';
elseif strcmp(subdir,'figures_main')
    role = 'candidate manuscript figure';
else
    role = 'caption guide or output index';
end
if contains(name,'Figure08'), role = 'candidate appendix or sensitivity figure'; end
end


function ids = common_trial_ids_local(T, group)
ids = unique(T.trial_id(string(T.method)==string(group{1})));
for j = 2:numel(group)
    ids = intersect(ids, unique(T.trial_id(string(T.method)==string(group{j}))));
end
ids = ids(:);
end

function [x,y,ids_out] = xy_for_method_local(T, method, ids)
R = T(string(T.method)==string(method),:);
[tf,loc] = ismember(ids,R.trial_id);
ids_out = ids(tf);
loc = loc(tf);
x = R.t_reach_s(loc);
y = R.E_excess_norm_s(loc);
valid = isfinite(x) & isfinite(y);
x = x(valid); y = y(valid); ids_out = ids_out(valid);
end

function [x,y] = xy_for_method_given_ids_local(T, method, ids)
[x,y,ids_found] = xy_for_method_local(T,method,ids);
% Preserve the requested ID order where possible.
x2 = nan(numel(ids),1); y2 = nan(numel(ids),1);
[tf,loc] = ismember(ids_found,ids);
x2(loc(tf)) = x(tf); y2(loc(tf)) = y(tf);
x = x2; y = y2;
end

function c = method_color_local(method)
switch char(method)
    case 'NoGuard'
        c = [0.20 0.20 0.20];
    case 'Feasible-RG'
        c = [0.00 0.45 0.74];
    case 'MLP-PRG'
        c = [0.85 0.33 0.10];
    case 'TRF-PRG'
        c = [0.49 0.18 0.56];
    case 'MPC-RG-grid'
        c = [0.47 0.67 0.19];
    case 'Bus-PTP-DP'
        c = [0.93 0.69 0.13];
    otherwise
        c = [0.35 0.35 0.35];
end
end

function mk = method_marker_local(method)
switch char(method)
    case 'NoGuard'
        mk = 'o';
    case 'Feasible-RG'
        mk = 's';
    case 'MLP-PRG'
        mk = '^';
    case 'TRF-PRG'
        mk = 'v';
    case 'MPC-RG-grid'
        mk = 'p';
    case 'Bus-PTP-DP'
        mk = 'h';
    otherwise
        mk = 'o';
end
end

function axis_padded_local(ax)
xl = xlim(ax); yl = ylim(ax);
if diff(xl)>0
    padx = 0.04*diff(xl); xlim(ax,[xl(1)-padx, xl(2)+padx]);
end
if diff(yl)>0
    pady = 0.06*diff(yl); ylim(ax,[max(0,yl(1)-pady), yl(2)+pady]);
end
end


function [mu,ci95] = paired_reduction_stats_local(T, refMethod, method, varname)
mu = NaN; ci95 = NaN;
if ~ismember(varname,T.Properties.VariableNames), return; end
ids = intersect(unique(T.trial_id(string(T.method)==string(refMethod))), unique(T.trial_id(string(T.method)==string(method))));
if isempty(ids), return; end
[xref,~] = values_for_method_ids_local(T, refMethod, ids, varname);
[x,~] = values_for_method_ids_local(T, method, ids, varname);
den = mean(xref,'omitnan');
if ~isfinite(den) || abs(den)<eps, return; end
r = 100*(xref - x)/abs(den);
r = r(isfinite(r));
if isempty(r), return; end
mu = mean(r,'omitnan');
if numel(r)>1
    ci95 = 1.96*std(r,'omitnan')/sqrt(numel(r));
else
    ci95 = 0;
end
end

function [x,ids_out] = values_for_method_ids_local(T, method, ids, varname)
R = T(string(T.method)==string(method),:);
x = nan(numel(ids),1); ids_out = ids(:);
[tf,loc] = ismember(ids,R.trial_id);
if any(tf)
    x(tf) = R.(varname)(loc(tf));
end
end

function add_grouped_errorbars_local(Y,E,color)
[ngroups, nbars] = size(Y);
groupwidth = min(0.8, nbars/(nbars + 1.5));
for j = 1:nbars
    x = (1:ngroups) - groupwidth/2 + (2*j-1) * groupwidth / (2*nbars);
    errorbar(x, Y(:,j), E(:,j), 'k', 'linestyle','none', 'LineWidth',0.8, 'Color',color, 'CapSize',4, 'HandleVisibility','off');
end
end

% -------------------- numeric helpers --------------------
function [mu,sd] = mean_std_method_local(T, method, var)
mu = NaN; sd = NaN;
if ~ismember(var,T.Properties.VariableNames), return; end
idx = string(T.method)==string(method) & isfinite(T.(var));
x = T.(var)(idx);
if isempty(x), return; end
mu = mean(x,'omitnan'); sd = std(x,'omitnan');
end

function txt = fmt_pm_local(mu,sd,var)
if ~isfinite(mu), txt = "n/a"; return; end
if contains(var,'E_grid') || contains(var,'P_grid_req')
    txt = sprintf('%.2f +/- %.2f',mu,sd);
elseif contains(var,'Vdc')
    txt = sprintf('%.3f +/- %.3f',mu,sd);
else
    txt = sprintf('%.3f +/- %.3f',mu,sd);
end
end

function txt = fmt_pm_summary_local(S, method, metric_raw)
txt = "n/a";
if isempty(S), return; end
if ~ismember('metric_raw',S.Properties.VariableNames), return; end
idx = string(S.method)==string(method) & string(S.metric_raw)==string(metric_raw);
if ~any(idx)
    idx = string(S.method)==string(method) & string(S.metric_canonical)==string(metric_raw);
end
if ~any(idx), return; end
mu = S.mean(find(idx,1)); sd = S.std(find(idx,1));
txt = sprintf('%.3f +/- %.3f',mu,sd);
end

function val = latency_component_local(Tlat, method, component, varname)
val = NaN;
if isempty(Tlat) || ~ismember(varname,Tlat.Properties.VariableNames), return; end
idx = string(Tlat.Method)==string(method) & string(Tlat.Component)==string(component);
if any(idx), val = Tlat.(varname)(find(idx,1)); end
end

function val = metric_from_elec_local(Telec, method, metric)
val = NaN;
if isempty(Telec), return; end
if ismember('method',Telec.Properties.VariableNames)
    idx = string(Telec.method)==string(method) & string(Telec.metric)==string(metric);
else
    idx = false(height(Telec),1);
end
if any(idx), val = Telec.mean(find(idx,1)); end
end

function val = safe_mean_local(T,var)
val = NaN;
if ismember(var,T.Properties.VariableNames)
    x = T.(var); x = x(isfinite(x));
    if ~isempty(x), val = mean(x); end
end
end

function val = getnum(T,var,idx)
val = NaN;
if ismember(var,T.Properties.VariableNames), val = T.(var)(idx); end
end

function s = fmt_num_unit_local(x,unit)
if ~isfinite(x), s = 'n/a'; else, s = sprintf('%.4g %s',x,unit); end
end

function s = fmt_int_local(x)
if ~isfinite(x), s = 'n/a'; else, s = sprintf('%.0f',x); end
end

function s = safe_name_local(name)
s = char(strrep(string(name),'-','_'));
s = matlab.lang.makeValidName(s);
end

function s = bool_text_local(tf)
if tf, s = 'all listed cases successful'; else, s = 'at least one case requires inspection'; end
end

function role = method_role_sentence_local(m)
switch char(m)
    case 'NoGuard'
        role = "Budget-agnostic reference case";
    case 'Feasible-RG'
        role = "Online feasibility-search reference governor";
    case 'MLP-PRG'
        role = "Proposed compact non-attention learned PRG surrogate";
    case 'TRF-PRG'
        role = "Proposed look-ahead sequence learned PRG surrogate";
    case 'Bus-PTP-DP'
        role = "Offline power-constrained path time-parameterization reference";
    case 'MPC-RG-grid'
        role = "Receding-horizon/reference-governor grid-search reference";
    otherwise
        role = "Comparison method";
end
end

function save_fig_local(fig, out_png, dpi)
ensure_dir_local(fileparts(out_png));
try
    exportgraphics(fig,out_png,'Resolution',dpi);
catch
    print(fig,out_png,'-dpng',sprintf('-r%d',dpi));
end
try
    [p,n,~] = fileparts(out_png);
    savefig(fig, fullfile(p,[n '.fig']));
catch
end
end
