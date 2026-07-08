%% RUN_03_EXPORT_REVISION_RESULTS.m
% One-click auxiliary exporter for the revised manuscript after RUN_02.
%
% Run order:
%   1) RUN_01_train_models_oneclick
%   2) RUN_02_CHUNKED_CONTINUE until MAIN4 and FAIR6 reach 100/100
%   3) RUN_03_EXPORT_REVISION_RESULTS
%
% RUN_03 does NOT add more 100-trial experiments. It compiles RUN_02 CSVs,
% generates paper-ready tables/figures, runs a one-case per-policy-update
% latency trace for both learned surrogates (MLP-PRG and TRF-PRG), and
% optionally runs light one-case robustness and electrical-sensitivity checks. It never uses RUN_02 rollout_wall_time_s as inference
% latency.

clear; clc; close all;

%% ================= USER SETTINGS =================
RUN = struct();
RUN.project_dir = fileparts(mfilename('fullpath'));
if isempty(RUN.project_dir), RUN.project_dir = pwd; end
RUN.results_root = fullfile(RUN.project_dir,'paper_outputs_chunked_100');
RUN.out_root = fullfile(RUN.results_root,'RUN03_revision_exports');

% Require completed RUN_02 tables before compiling manuscript tables.
RUN.require_MAIN4_100 = true;
RUN.require_FAIR6_100 = true;

% Lightweight auxiliary runs. These are single-case/smoke-size by default.
RUN.do_representative_figure = true;
RUN.do_csv_figures = true;
RUN.do_latency_trace = true;
RUN.latency_Nsteps = 200;   % one-case policy updates; not 100 rollouts
RUN.latency_warmup = 20;

RUN.do_robustness_onecase = true;    % 7 perturbation cases x NoGuard/MLP/TRF
RUN.do_electrical_smoke = true;      % small electrical sensitivity smoke
RUN.electrical_Nsens = 3;
RUN.electrical_Ntraj = 1;
RUN.use_parallel_aux = false;        % keep false unless you want to open parpool
%% =================================================

cd(RUN.project_dir);
addpath(genpath(RUN.project_dir));
ensure_dir_local(RUN.out_root);
ensure_dir_local(fullfile(RUN.out_root,'tables'));
ensure_dir_local(fullfile(RUN.out_root,'figures'));
ensure_dir_local(fullfile(RUN.out_root,'latency'));
ensure_dir_local(fullfile(RUN.out_root,'robustness'));
ensure_dir_local(fullfile(RUN.out_root,'electrical_sensitivity'));

log_file = fullfile(RUN.out_root, sprintf('RUN_03_log_%s.txt', datestr(now,'yyyymmdd_HHMMSS')));
diary(log_file);
cDiary = onCleanup(@() diary('off')); %#ok<NASGU>

PB  = spinn3d_params_block();
cfg = spinn3d_params_to_cfg(PB,'demo_alpha');
cfg.NO_FIGURE = true;
cfg.DISABLE_FIGURES = true;
cfg.MODEL_PATH = char(getfield_def_local(cfg,'MODEL_PATH',PB.paths.model_alpha));
cfg.MLP_MODEL_PATH = char(getfield_def_local(cfg,'MLP_MODEL_PATH',PB.paths.model_alpha_mlp));
assert(exist(cfg.MODEL_PATH,'file')==2, 'TRF model not found: %s. Run RUN_01 first.', cfg.MODEL_PATH);
assert(exist(cfg.MLP_MODEL_PATH,'file')==2, 'MLP model not found: %s. Run RUN_01 first.', cfg.MLP_MODEL_PATH);

main = struct('label','MAIN4','subdir','main_4methods_N100_seed1', ...
    'methods',{{'NoGuard','Feasible-RG','MLP-PRG','TRF-PRG'}},'N',100);
fair = struct('label','FAIR6','subdir','fair_6methods_N100_seed102', ...
    'methods',{{'NoGuard','Feasible-RG','TRF-PRG','MLP-PRG','Bus-PTP-DP','MPC-RG-grid'}},'N',100);

main_dir = fullfile(RUN.results_root, main.subdir);
fair_dir = fullfile(RUN.results_root, fair.subdir);
main_trials_csv = fullfile(main_dir,'fair_baseline_trials.csv');
main_summary_csv = fullfile(main_dir,'fair_baseline_summary.csv');
main_paired_csv = fullfile(main_dir,'fair_baseline_paired_vs_reference.csv');
fair_trials_csv = fullfile(fair_dir,'fair_baseline_trials.csv');
fair_summary_csv = fullfile(fair_dir,'fair_baseline_summary.csv');
fair_paired_csv = fullfile(fair_dir,'fair_baseline_paired_vs_reference.csv');

fprintf('\n============================================================\n');
fprintf('RUN_03_EXPORT_REVISION_RESULTS started at %s\n', datestr(now));
fprintf('Project dir: %s\n', RUN.project_dir);
fprintf('Output dir : %s\n', RUN.out_root);
fprintf('TRF model  : %s\n', cfg.MODEL_PATH);
fprintf('MLP model  : %s\n', cfg.MLP_MODEL_PATH);
fprintf('============================================================\n');

main_done = completed_ids_local(main_trials_csv, main.methods);
fair_done = completed_ids_local(fair_trials_csv, fair.methods);
fprintf('MAIN4 complete paired trials: %d/%d\n', numel(main_done), main.N);
fprintf('FAIR6 complete paired trials: %d/%d\n', numel(fair_done), fair.N);
if RUN.require_MAIN4_100 && numel(main_done) < main.N
    error('MAIN4 is not complete. Run RUN_02_CHUNKED_CONTINUE until MAIN4 is 100/100.');
end
if RUN.require_FAIR6_100 && numel(fair_done) < fair.N
    error('FAIR6 is not complete. Run RUN_02_CHUNKED_CONTINUE until FAIR6 is 100/100.');
end

% Ensure RUN_02 has canonical manuscript-facing aliases and variable dictionaries,
% even if the 100-trial CSVs were generated before this unified package.
try
    paper_standardize_run02_outputs(main_dir, main.label, main.methods);
    paper_standardize_run02_outputs(fair_dir, fair.label, fair.methods);
    paper_write_run02_output_index(RUN.results_root);
catch MEstd
    warning('[RUN03] Could not standardize existing RUN_02 outputs: %s', MEstd.message);
end

status = struct();
status.started = datestr(now);
status.steps = table(strings(0,1), strings(0,1), strings(0,1), ...
    'VariableNames',{'step','status','path'});

%% 1) Compile manuscript tables from RUN_02 CSVs.
fprintf('\n[RUN03] Compiling Tables 9--11 from RUN_02 CSV files...\n');
Table9 = make_table9_local(main_summary_csv);
Table10 = make_table10_local(main_summary_csv);
Table11 = make_table11_local(fair_summary_csv, table());
Table5learned = make_learned_surrogate_model_table_local(cfg);
writetable(Table9, fullfile(RUN.out_root,'tables','Table9_main_power_energy.csv'));
writetable(Table10, fullfile(RUN.out_root,'tables','Table10_ramp_power_leveling.csv'));
writetable(Table11, fullfile(RUN.out_root,'tables','Table11_fair_baseline_no_latency.csv'));
writetable(Table5learned, fullfile(RUN.out_root,'tables','Table5_learned_surrogate_training_validation.csv'));
status = add_status_local(status,'compile_RUN02_tables','done',fullfile(RUN.out_root,'tables'));
status = add_status_local(status,'learned_surrogate_model_table','done',fullfile(RUN.out_root,'tables','Table5_learned_surrogate_training_validation.csv'));

%% 2) CSV-only paper figures: no extra rollouts.
if RUN.do_csv_figures
    fprintf('\n[RUN03] Generating CSV-based summary/tradeoff figures...\n');
    try
        make_csv_figures_local(main_trials_csv, main_summary_csv, fair_summary_csv, fullfile(RUN.out_root,'figures'));
        status = add_status_local(status,'csv_figures','done',fullfile(RUN.out_root,'figures'));
    catch ME
        warning('CSV figures failed: %s', ME.message);
        status = add_status_local(status,'csv_figures',['failed: ' ME.message],fullfile(RUN.out_root,'figures'));
    end
end

%% 3) Representative time-domain figure: one paired case.
if RUN.do_representative_figure
    fprintf('\n[RUN03] Generating one representative paired-trial telemetry figure...\n');
    try
        cfg_fig = cfg;
        cfg_fig.NO_FIGURE = false;
        cfg_fig.DISABLE_FIGURES = false;
        paper_fig3_representative_trial(cfg_fig, struct( ...
            'rng_seed',1, ...
            'pgrid_tau_s',0.05, ...
            'save_png',fullfile(RUN.out_root,'figures','Fig_representative_trial.png'), ...
            'save_skeleton_png',fullfile(RUN.out_root,'figures','Fig_representative_skeleton.png'), ...
            'close_demo_figs',true));
        status = add_status_local(status,'representative_figure','done',fullfile(RUN.out_root,'figures','Fig_representative_trial.png'));
    catch ME
        warning('Representative figure failed: %s', ME.message);
        status = add_status_local(status,'representative_figure',['failed: ' ME.message],fullfile(RUN.out_root,'figures'));
    end
end

%% 4) One-case per-policy-update neural latency trace.
LatencySummary = table();
if RUN.do_latency_trace
    fprintf('\n[RUN03] Measuring one-case per-policy-update neural latency trace...\n');
    try
        Rlat = paper_policy_latency_trace_onecase(cfg, struct( ...
            'Nsteps',RUN.latency_Nsteps, ...
            'warmup',RUN.latency_warmup, ...
            'export_dir',fullfile(RUN.out_root,'latency')));
        LatencySummary = Rlat.summary;
        lat_csv = fullfile(RUN.out_root,'tables','Table6_policy_update_latency_onecase.csv');
        write_latency_table6_local(Rlat, lat_csv);
        % Symmetric learned-surrogate alias for the revised manuscript terminology.
        copyfile(lat_csv, fullfile(RUN.out_root,'tables','Table6_learned_surrogate_policy_latency_onecase.csv'));
        Table11lat = make_table11_local(fair_summary_csv, LatencySummary);
        writetable(Table11lat, fullfile(RUN.out_root,'tables','Table11_fair_baseline_with_nn_latency.csv'));
        writetable(Table11lat, fullfile(RUN.out_root,'tables','Table11_fair_baseline_with_learned_latency.csv'));
        status = add_status_local(status,'onecase_neural_latency','done',fullfile(RUN.out_root,'latency','latency_policy_steps.csv'));
    catch ME
        warning('Latency trace failed: %s', ME.message);
        status = add_status_local(status,'onecase_neural_latency',['failed: ' ME.message],fullfile(RUN.out_root,'latency'));
    end
end

%% 5) One-case robustness table: not 100-run average.
if RUN.do_robustness_onecase
    fprintf('\n[RUN03] Running one-case robustness perturbation table...\n');
    try
        Rob = robustness_onecase_local(cfg, fullfile(RUN.out_root,'robustness'));
        writetable(Rob, fullfile(RUN.out_root,'tables','Table12_robustness_onecase.csv'));
        status = add_status_local(status,'robustness_onecase','done',fullfile(RUN.out_root,'tables','Table12_robustness_onecase.csv'));
    catch ME
        warning('Robustness one-case failed: %s', ME.message);
        status = add_status_local(status,'robustness_onecase',['failed: ' ME.message],fullfile(RUN.out_root,'robustness'));
    end
end

%% 6) Light electrical sensitivity smoke: small by default, not full paper sweep.
if RUN.do_electrical_smoke
    fprintf('\n[RUN03] Running light electrical sensitivity smoke test...\n');
    try
        optsE = struct();
        optsE.Nsens = RUN.electrical_Nsens;
        optsE.Ntraj = RUN.electrical_Ntraj;
        optsE.use_parallel = RUN.use_parallel_aux;
        optsE.methods = {'NoGuard','Feasible-RG','MLP-PRG','TRF-PRG'};
        optsE.export_dir = fullfile(RUN.out_root,'electrical_sensitivity');
        optsE.mlp_model_path = cfg.MLP_MODEL_PATH;
        Relec = paper_electrical_sensitivity_parallel(cfg, optsE); %#ok<NASGU>
        status = add_status_local(status,'electrical_sensitivity_smoke','done',fullfile(RUN.out_root,'electrical_sensitivity'));
    catch ME
        warning('Electrical sensitivity smoke failed: %s', ME.message);
        status = add_status_local(status,'electrical_sensitivity_smoke',['failed: ' ME.message],fullfile(RUN.out_root,'electrical_sensitivity'));
    end
end

%% 7) Copy important source CSV paths into a manifest/report.
status.done = true;
status.finished = datestr(now);
status.main_trials_csv = main_trials_csv;
status.main_summary_csv = main_summary_csv;
status.main_paired_csv = main_paired_csv;
status.fair_trials_csv = fair_trials_csv;
status.fair_summary_csv = fair_summary_csv;
status.fair_paired_csv = fair_paired_csv;
status.out_root = RUN.out_root;
save(fullfile(RUN.out_root,'RUN03_STATUS.mat'),'status','RUN','cfg','PB','-v7.3');
% Legacy alias for older check scripts.
save(fullfile(RUN.out_root,'AUX_STATUS.mat'),'status','RUN','cfg','PB','-v7.3');
write_text_report_local(status, RUN, cfg, fullfile(RUN.out_root,'RUN03_README_RESULTS.txt'));

fprintf('\n============================================================\n');
fprintf('RUN_03 completed. Outputs are in:\n  %s\n', RUN.out_root);
fprintf('Key files:\n');
fprintf('  %s\n', fullfile(RUN.out_root,'tables','Table9_main_power_energy.csv'));
fprintf('  %s\n', fullfile(RUN.out_root,'tables','Table10_ramp_power_leveling.csv'));
fprintf('  %s\n', fullfile(RUN.out_root,'tables','Table5_learned_surrogate_training_validation.csv'));
fprintf('  %s\n', fullfile(RUN.out_root,'tables','Table11_fair_baseline_with_nn_latency.csv'));
fprintf('  %s\n', fullfile(RUN.out_root,'tables','Table6_policy_update_latency_onecase.csv'));
fprintf('  %s\n', fullfile(RUN.out_root,'latency','latency_policy_steps.csv'));
fprintf('  %s\n', fullfile(RUN.out_root,'RUN03_README_RESULTS.txt'));
fprintf('============================================================\n');

%% ================= LOCAL FUNCTIONS =================
function T = make_learned_surrogate_model_table_local(cfg)
% Model/training/validation summary for the two proposed learned PRG surrogates.
models = { ...
    'MLP-PRG', cfg.MLP_MODEL_PATH, 'Compact non-attention learned PRG surrogate'; ...
    'TRF-PRG', cfg.MODEL_PATH,     'Look-ahead sequence learned PRG surrogate'};
rows = cell(size(models,1),1);
for i=1:size(models,1)
    method = string(models{i,1});
    model_path = char(models{i,2});
    M0 = load_model_alpha_any(model_path);
    M = normalize_loaded_model_for_table_local(M0);
    W = whos('M0');
    ti = getfield_def_local(M,'train_info',struct());
    pp = getfield_def_local(M,'preproc',struct());
    vm = getfield_def_local(M,'val_metrics',struct());
    r = struct();
    r.Method = method;
    r.Role = string(models{i,3});
    r.Model_kind = string(getfield_def_local(M,'type',getfield_def_local(pp,'kind','unknown')));
    r.K_tokens = numeric_field_local(ti,'K',numeric_field_local(pp,'K',NaN));
    r.Input_dimension = numeric_field_local(ti,'Din',numel(getfield_def_local(pp,'mu',[])));
    r.Flattened_input_dimension = numeric_field_local(ti,'Dflat',NaN);
    r.N_samples = numeric_field_local(ti,'N_samples',NaN);
    r.N_train = numeric_field_local(ti,'n_train',NaN);
    r.N_validation = numeric_field_local(ti,'n_val',NaN);
    r.Epochs = numeric_field_local(ti,'MaxEpochs',NaN);
    r.Mini_batch = numeric_field_local(ti,'MiniBatchSize',NaN);
    r.Learn_rate = numeric_field_local(ti,'InitialLearnRate',NaN);
    r.Validation_MAE = numeric_field_local(vm,'MAE',NaN);
    r.Validation_RMSE = numeric_field_local(vm,'RMSE',NaN);
    r.Validation_MaxAE = numeric_field_local(vm,'MaxAE',NaN);
    r.Validation_N = numeric_field_local(vm,'N',NaN);
    r.Model_parameters = count_model_params_for_table_local(getfield_def_local(M,'net',[]));
    r.Model_memory_MB = W.bytes/1024/1024;
    r.Model_path = string(model_path);
    rows{i} = r;
end
T = struct2table(vertcat(rows{:}));
end

function M = normalize_loaded_model_for_table_local(M)
if isfield(M,'model'), M = M.model; end
if ~isfield(M,'type') || isempty(M.type)
    if isfield(M,'preproc'), M.type = getfield_def_local(M.preproc,'kind','trf_lookahead');
    else, M.type = 'unknown'; end
end
end

function v = numeric_field_local(S,fn,def)
v = def;
if isstruct(S) && isfield(S,fn) && ~isempty(S.(fn))
    try
        x = S.(fn);
        if isnumeric(x) || islogical(x), v = double(x(1)); end
    catch
    end
end
end

function n = count_model_params_for_table_local(net)
n = NaN;
try
    n = 0;
    L = net.Learnables;
    for ii = 1:size(L,1), n = n + numel(L.Value{ii}); end
catch
    try
        n = 0;
        layers = net.Layers;
        for ii=1:numel(layers)
            if isprop(layers(ii),'Weights') && ~isempty(layers(ii).Weights), n = n + numel(layers(ii).Weights); end
            if isprop(layers(ii),'Bias') && ~isempty(layers(ii).Bias), n = n + numel(layers(ii).Bias); end
        end
    catch
        n = NaN;
    end
end
end

function T9 = make_table9_local(summary_csv)
S = readtable(summary_csv);
methods = {'NoGuard','Feasible-RG','MLP-PRG','TRF-PRG'};
headers = matlab.lang.makeValidName(methods);
metrics = { ...
    't_reach (s)', {'t_reach'}; ...
    'max pbar (-)', {'pbar_peak','pbar_lpf_peak','P_peak_lpf_norm','P_peak_norm'}; ...
    'p99(pbar) (-)', {'pbar_p99','pbar_lpf_p99','P_p99_lpf_norm','P_p99_norm'}; ...
    'E_excess_norm (s)', {'E_excess_norm_s'}; ...
    'p95(|dpbar/dt|) (s^-1)', {'pbar_ramp_p95','ramp_p95_lpf_norm','ramp_p95_norm'}; ...
    'E_grid (J)', {'E_grid'}; ...
    'E_reuse (J)', {'E_reuse'}; ...
    'E_dump (J)', {'E_dump'}; ...
    'max P_grid_req / max P_src_short (W)', {'P_request_peak','Psource_shortfall_peak'}; ...
    'I_rms / I_peak (A)', {'I_rms','I_peak'}; ...
    'Vdc_min (V; larger is better)', {'Vdc_min'} };
rows = cell(size(metrics,1),1);
for i=1:size(metrics,1)
    r = struct(); r.Metric = string(metrics{i,1});
    for m=1:numel(methods)
        r.(headers{m}) = string(format_metric_cell_local(S, methods{m}, metrics{i,2}));
    end
    rows{i} = r;
end
T9 = struct2table(vertcat(rows{:}));
end

function T10 = make_table10_local(summary_csv)
S = readtable(summary_csv);
methods = {'NoGuard','Feasible-RG','MLP-PRG','TRF-PRG'};
rows = cell(numel(methods),1);
for i=1:numel(methods)
    r = struct();
    r.Method = string(methods{i});
    r.p95_dpbar_dt = string(format_metric_cell_local(S, methods{i}, {'pbar_ramp_p95','ramp_p95_lpf_norm','ramp_p95_norm'}));
    r.max_dpbar_dt = string(format_metric_cell_local(S, methods{i}, {'pbar_ramp_max','ramp_max_lpf_norm','ramp_max_norm'}));
    r.max_pbar = string(format_metric_cell_local(S, methods{i}, {'pbar_peak','pbar_lpf_peak','P_peak_lpf_norm','P_peak_norm'}));
    r.E_excess_norm_s = string(format_metric_cell_local(S, methods{i}, {'E_excess_norm_s'}));
    rows{i} = r;
end
T10 = struct2table(vertcat(rows{:}));
end

function T11 = make_table11_local(summary_csv, LatencySummary)
S = readtable(summary_csv);
methods = {'NoGuard','Feasible-RG','Bus-PTP-DP','MPC-RG-grid','MLP-PRG','TRF-PRG'};
notes = {'Budget-agnostic PRG','Online teacher/search','Offline path time-parameterization; online lookup', ...
    'Receding-horizon baseline','Proposed compact non-attention learned surrogate','Proposed look-ahead sequence learned surrogate'};
rows = cell(numel(methods),1);
for i=1:numel(methods)
    r = struct();
    r.Method = string(methods{i});
    r.t_reach = string(format_metric_cell_local(S, methods{i}, {'t_reach'}));
    r.E_excess_norm_s = string(format_metric_cell_local(S, methods{i}, {'E_excess_norm_s'}));
    r.E_grid_J = string(format_metric_cell_local(S, methods{i}, {'E_grid'}));
    r.Latency_p99_us = string(latency_p99_text_local(LatencySummary, methods{i}));
    r.Notes = string(notes{i});
    rows{i} = r;
end
T11 = struct2table(vertcat(rows{:}));
end

function txt = latency_p99_text_local(L, method)
txt = "N/A";
if isempty(L) || ~istable(L), return; end
m = string(method);
if any(strcmpi(method,{'TRF-PRG','MLP-PRG'}))
    idx = string(L.method)==m & string(L.component)=="Total learned-policy update";
    if any(idx)
        txt = sprintf('%.1f', L.p99_us(find(idx,1)));
    end
else
    txt = "not NN inference";
end
end

function write_latency_table6_local(Rlat, out_csv)
L = Rlat.summary;
rows = {};
comps = {'Feature construction','Neural inference','Governor update','Total learned-policy update'};
for im = 1:2
    method = string(Rlat.model_info.method(im));
    for c = 1:numel(comps)
        idx = string(L.method)==method & string(L.component)==string(comps{c});
        r = struct();
        r.Method = method;
        r.Component = string(comps{c});
        if any(idx)
            jj = find(idx,1);
            r.Mean_us = L.mean_us(jj); r.p95_us = L.p95_us(jj); r.p99_us = L.p99_us(jj); r.Max_us = L.max_us(jj);
        else
            r.Mean_us = NaN; r.p95_us = NaN; r.p99_us = NaN; r.Max_us = NaN;
        end
        r.Platform_detail = string(sprintf('policy dt %.3g ms, servo Ts %.3g ms',1e3*Rlat.policy_dt,1e3*Rlat.servo_Ts));
        rows{end+1,1} = r; %#ok<AGROW>
    end
end
for im = 1:height(Rlat.model_info)
    r = struct();
    r.Method = string(Rlat.model_info.method(im));
    r.Component = "Model memory / parameters";
    r.Mean_us = NaN; r.p95_us = NaN; r.p99_us = NaN; r.Max_us = NaN;
    r.Platform_detail = string(sprintf('%.3g MB, %.0f params, %s', Rlat.model_info.model_bytes(im)/1024/1024, Rlat.model_info.n_params(im), char(Rlat.model_info.kind(im))));
    rows{end+1,1} = r; %#ok<AGROW>
end
T = struct2table(vertcat(rows{:}));
writetable(T,out_csv);
end

function s = format_metric_cell_local(S, method, metricCandidates)
if ischar(metricCandidates) || isstring(metricCandidates), metricCandidates = cellstr(metricCandidates); end
if numel(metricCandidates)==2 && ~contains(metricCandidates{1}, '|')
    [mu1,sd1,ok1] = get_summary_metric_local(S,method,metricCandidates{1});
    [mu2,sd2,ok2] = get_summary_metric_local(S,method,metricCandidates{2});
    if ok1 || ok2
        s = sprintf('%s / %s', fmt_pm_local(mu1,sd1,ok1), fmt_pm_local(mu2,sd2,ok2));
        return;
    end
end
for k=1:numel(metricCandidates)
    [mu,sd,ok] = get_summary_metric_local(S,method,metricCandidates{k});
    if ok, s = fmt_pm_local(mu,sd,ok); return; end
end
s = 'N/A';
end

function [mu,sd,ok] = get_summary_metric_local(S,method,metric)
ok=false; mu=NaN; sd=NaN;
if isempty(S) || ~istable(S), return; end
if ~ismember('method',S.Properties.VariableNames) || ~ismember('metric',S.Properties.VariableNames), return; end
idx = string(S.method)==string(method) & string(S.metric)==string(metric);
if ~any(idx), return; end
j=find(idx,1);
mu=S.mean(j); sd=S.std(j); ok=true;
end

function s = fmt_pm_local(mu,sd,ok)
if nargin<3, ok=isfinite(mu); end
if ~ok || ~isfinite(mu), s='N/A'; return; end
if ~isfinite(sd), sd=NaN; end
if abs(mu) >= 100
    s = sprintf('%.2f ± %.2f',mu,sd);
elseif abs(mu) >= 10
    s = sprintf('%.3f ± %.3f',mu,sd);
else
    s = sprintf('%.4f ± %.4f',mu,sd);
end
end

function make_csv_figures_local(main_trials_csv, main_summary_csv, fair_summary_csv, figdir)
ensure_dir_local(figdir);
T = readtable(main_trials_csv); T.method = string(T.method);
S = readtable(main_summary_csv); S.method = string(S.method); S.metric = string(S.metric);
Sf = readtable(fair_summary_csv); Sf.method = string(Sf.method); Sf.metric = string(Sf.metric);

% Main metric bars, mean only.
methods = ["NoGuard","Feasible-RG","MLP-PRG","TRF-PRG"];
metrics = ["t_reach","pbar_peak","pbar_p99","E_excess_norm_s","pbar_ramp_p95","E_grid"];
for k=1:numel(metrics)
    vals = nan(1,numel(methods));
    for i=1:numel(methods)
        [mu,~,ok] = get_summary_metric_local(S,methods(i),metrics(k));
        if ok, vals(i)=mu; end
    end
    fig = figure('Visible','off','Name','RUN03_main_bar');
    bar(vals); grid on;
    set(gca,'XTickLabel',cellstr(methods),'XTickLabelRotation',20);
    ylabel(char(metrics(k)),'Interpreter','none');
    title(['MAIN4 summary: ' char(metrics(k))],'Interpreter','none');
    exportgraphics(fig, fullfile(figdir, sprintf('Fig_MAIN4_%s_bar.png',char(metrics(k)))),'Resolution',220);
    close(fig);
end

% Tradeoff scatter: t_reach vs E_excess_norm_s for NoGuard/TRF/Feasible.
fig = figure('Visible','off','Name','RUN03_tradeoff'); hold on; grid on;
plot_trade_pair_local(T,"NoGuard","TRF-PRG");
plot_trade_pair_local(T,"NoGuard","Feasible-RG");
xlabel('t_{reach} (s)'); ylabel('E_{excess} normalized (s)');
title('Paired trade-off from RUN\_02 MAIN4 CSV');
legend('Location','best');
exportgraphics(fig, fullfile(figdir,'Fig_MAIN4_tradeoff_treach_Eexcess.png'),'Resolution',220);
close(fig);

% FAIR6 reaching time and energy bars.
fair_methods = ["NoGuard","Feasible-RG","Bus-PTP-DP","MPC-RG-grid","MLP-PRG","TRF-PRG"];
fair_metrics = ["t_reach","E_excess_norm_s","E_grid"];
for k=1:numel(fair_metrics)
    vals = nan(1,numel(fair_methods));
    for i=1:numel(fair_methods)
        [mu,~,ok] = get_summary_metric_local(Sf,fair_methods(i),fair_metrics(k));
        if ok, vals(i)=mu; end
    end
    fig = figure('Visible','off','Name','RUN03_fair_bar');
    bar(vals); grid on;
    set(gca,'XTickLabel',cellstr(fair_methods),'XTickLabelRotation',25);
    ylabel(char(fair_metrics(k)),'Interpreter','none');
    title(['FAIR6 summary: ' char(fair_metrics(k))],'Interpreter','none');
    exportgraphics(fig, fullfile(figdir, sprintf('Fig_FAIR6_%s_bar.png',char(fair_metrics(k)))),'Resolution',220);
    close(fig);
end
end

function plot_trade_pair_local(T, methodA, methodB)
ids = unique(T.trial_id(:).');
for id = ids
    ia = find(T.trial_id==id & T.method==string(methodA) & T.success,1);
    ib = find(T.trial_id==id & T.method==string(methodB) & T.success,1);
    if isempty(ia) || isempty(ib), continue; end
    xa = T.t_reach(ia); xb = T.t_reach(ib);
    ya = T.E_excess_norm_s(ia); yb = T.E_excess_norm_s(ib);
    plot([xa xb],[ya yb],'-','Color',[0.75 0.75 0.75],'HandleVisibility','off');
end
idxA = T.method==string(methodA) & T.success;
idxB = T.method==string(methodB) & T.success;
scatter(T.t_reach(idxA),T.E_excess_norm_s(idxA),20,'DisplayName',char(methodA));
scatter(T.t_reach(idxB),T.E_excess_norm_s(idxB),20,'filled','DisplayName',char(methodB));
end

function Rob = robustness_onecase_local(cfg, outdir)
ensure_dir_local(outdir);
baseP = getfield_def_local(cfg,'P_GRID_BUDGET',getfield_def_local(cfg,'P_TOTAL_MAX',NaN));
cases = { ...
    'Friction perturbation', @perturb_friction_local; ...
    'Payload mass variation', @perturb_payload_mass_local; ...
    'Payload CoM error', @perturb_payload_com_local; ...
    'Sensor noise', @perturb_sensor_noise_local; ...
    'Voltage variation', @perturb_voltage_local; ...
    'Budget variation', @perturb_budget_local; ...
    'Combined perturbations', @perturb_combined_local};
rows = cell(size(cases,1),1);
for i=1:size(cases,1)
    cname = cases{i,1};
    f = cases{i,2};
    ci = cfg; ci.NO_FIGURE=true; ci.DISABLE_FIGURES=true;
    ci = f(ci,baseP);
    fprintf('[rob-onecase] %s\n', cname);
    [okB, MB, errB] = run_method_metrics_local('NoGuard',ci);
    [okM, MM, errM] = run_method_metrics_local('MLP-PRG',ci);
    [okT, MT, errT] = run_method_metrics_local('TRF-PRG',ci);
    r = struct();
    r.Perturbation = string(cname);
    r.NoGuard_success = okB;
    r.MLP_PRG_success = okM;
    r.TRF_PRG_success = okT;
    r.NoGuard_E_excess_norm_s = get_metric_field_local(MB,'E_excess_norm_s');
    r.MLP_PRG_E_excess_norm_s = get_metric_field_local(MM,'E_excess_norm_s');
    r.TRF_PRG_E_excess_norm_s = get_metric_field_local(MT,'E_excess_norm_s');
    r.NoGuard_E_grid_J = get_metric_field_local(MB,'E_grid');
    r.MLP_PRG_E_grid_J = get_metric_field_local(MM,'E_grid');
    r.TRF_PRG_E_grid_J = get_metric_field_local(MT,'E_grid');
    r.MLP_PRG_pbar_peak = get_metric_field_local(MM,'pbar_peak');
    r.TRF_PRG_pbar_peak = get_metric_field_local(MT,'pbar_peak');
    r.MLP_PRG_ramp_p95 = get_metric_field_local(MM,'pbar_ramp_p95');
    r.TRF_PRG_ramp_p95 = get_metric_field_local(MT,'pbar_ramp_p95');
    r.MLP_PRG_Vdc_min = get_metric_field_local(MM,'Vdc_min');
    r.TRF_PRG_Vdc_min = get_metric_field_local(MT,'Vdc_min');
    r.Worst_exceedance_norm_s = max([r.NoGuard_E_excess_norm_s, r.MLP_PRG_E_excess_norm_s, r.TRF_PRG_E_excess_norm_s],[],'omitnan');
    r.Key_observation = string(make_observation_local(r,errB,errM,errT));
    rows{i} = r;
end
Rob = struct2table(vertcat(rows{:}));
writetable(Rob, fullfile(outdir,'robustness_onecase_trials.csv'));
save(fullfile(outdir,'robustness_onecase.mat'),'Rob');
end

function [ok,M,err] = run_method_metrics_local(method,cfg)
ok=false; M=struct(); err="";
try
    switch lower(strrep(method,'_','-'))
        case {'noguard','baseline'}
            out = spinn3d_demo_ctcgov_online_nonn_nolimit(cfg);
        case {'trf-prg','trf'}
            out = spinn3d_demo_ctcgov_online_nn(cfg);
        case {'mlp-prg','mlp'}
            out = spinn3d_demo_ctcgov_online_mlp(cfg);
        otherwise
            error('Unsupported method for robustness onecase: %s',method);
    end
    M = spinn3d_compute_power_metrics(out, getfield_def_local(cfg,'P_GRID_BUDGET',getfield_def_local(cfg,'P_TOTAL_MAX',NaN)), ...
        struct('power_eval_mode','lpf','power_tau_s',0.05,'power_eval_tau_s',0.05,'Pbudget_margin_frac',0.05));
    ok = true;
catch ME
    err = string(ME.message);
end
try, close(findall(0,'Type','figure')); catch, end
end

function v = get_metric_field_local(M,fn)
if isstruct(M) && isfield(M,fn), v = M.(fn); else, v = NaN; end
end
function obs = make_observation_local(r,errB,errM,errT)
if ~r.MLP_PRG_success || ~r.TRF_PRG_success
    parts = strings(0,1);
    if ~r.MLP_PRG_success, parts(end+1) = "MLP failed: " + errM; end %#ok<AGROW>
    if ~r.TRF_PRG_success, parts(end+1) = "TRF failed: " + errT; end %#ok<AGROW>
    obs = strjoin(parts,'; '); return;
end
if ~r.NoGuard_success
    obs = "NoGuard failed: " + errB; return;
end
learned_excess = [r.MLP_PRG_E_excess_norm_s, r.TRF_PRG_E_excess_norm_s];
if all(isfinite(learned_excess)) && all(learned_excess <= 1e-6)
    obs = "Both learned surrogates retained near-zero normalized exceedance in this case.";
else
    obs = "Both learned surrogates remained executable; compare MLP/TRF exceedance and ramp metrics for this perturbation.";
end
end

function c = perturb_friction_local(c,~)
if isfield(c,'FRIC'), c.FRIC.B = c.FRIC.B*1.30; c.FRIC.Fc = c.FRIC.Fc*1.30; end
end
function c = perturb_payload_mass_local(c,~)
if isfield(c,'PAYLOAD') && isfield(c.PAYLOAD,'DEMO'), c.PAYLOAD.DEMO.mass = c.PAYLOAD.DEMO.mass*1.20; end
end
function c = perturb_payload_com_local(c,~)
if isfield(c,'PAYLOAD') && isfield(c.PAYLOAD,'DEMO'), c.PAYLOAD.DEMO.com = row_local(getfield_def_local(c.PAYLOAD.DEMO,'com',[0 0 0.05])) + [0.015 -0.015 0.015]; end
end
function c = perturb_sensor_noise_local(c,~)
c.NOISE.sigma_q = 1e-3; c.NOISE.sigma_dq = 1e-2;
end
function c = perturb_voltage_local(c,~)
if ~isfield(c,'BUS'), c.BUS=struct(); end
c.BUS.V_nom = 10.5; c.BUS.V_oc = 10.5; c.V_BUS_NOMINAL = 10.5;
end
function c = perturb_budget_local(c,baseP)
Pbud = 0.80*baseP; c.P_GRID_BUDGET=Pbud; c.P_TOTAL_MAX=Pbud;
if isfield(c,'BUS'), c.BUS.P_grid_budget=Pbud; c.BUS.P_total_max=Pbud; end
if isfield(c,'CAPS'), c.CAPS.P_grid_budget=Pbud; c.CAPS.P_total_max=Pbud; end
end
function c = perturb_combined_local(c,baseP)
c = perturb_friction_local(c,baseP); c = perturb_payload_mass_local(c,baseP); c = perturb_payload_com_local(c,baseP); c = perturb_sensor_noise_local(c,baseP); c = perturb_voltage_local(c,baseP); c = perturb_budget_local(c,baseP);
end

function ids = completed_ids_local(csv_trials, methods)
ids = [];
if exist(csv_trials,'file') ~= 2, return; end
try, T = readtable(csv_trials); catch, return; end
if ~ismember('trial_id',T.Properties.VariableNames) || ~ismember('method',T.Properties.VariableNames), return; end
if ~ismember('success',T.Properties.VariableNames), T.success = false(height(T),1); end
T.method = string(T.method);
all_ids = unique(T.trial_id(:).');
for id = all_ids
    ok = true;
    for k=1:numel(methods)
        if ~any(T.trial_id==id & T.method==string(methods{k}) & logical(T.success))
            ok=false; break;
        end
    end
    if ok, ids(end+1) = id; end %#ok<AGROW>
end
ids = sort(unique(ids));
end

function status = add_status_local(status,step,st,path)
new = table(string(step),string(st),string(path),'VariableNames',{'step','status','path'});
status.steps = [status.steps; new];
end

function write_text_report_local(status, RUN, cfg, fn)
fid = fopen(fn,'w');
if fid<0, return; end
fprintf(fid,'RUN03 auxiliary results\n');
fprintf(fid,'Generated: %s\n', datestr(now));
fprintf(fid,'Output root: %s\n\n', RUN.out_root);
fprintf(fid,'This folder compiles RUN_02 CSVs and adds one-case auxiliary outputs.\n');
fprintf(fid,'Latency files measure feature construction + neural inference + governor update per policy update. They do not use RUN_02 rollout_wall_time_s.\n');
fprintf(fid,'Servo Ts = %.6g s; policy dt = %.6g s.\n\n', getfield_def_local(cfg,'TS',NaN), getfield_def_local(cfg,'POLICY_DT',NaN));
fprintf(fid,'Important source CSVs:\n');
fprintf(fid,'  MAIN4 trials : %s\n', status.main_trials_csv);
fprintf(fid,'  MAIN4 summary: %s\n', status.main_summary_csv);
fprintf(fid,'  FAIR6 trials : %s\n', status.fair_trials_csv);
fprintf(fid,'  FAIR6 summary: %s\n\n', status.fair_summary_csv);
fprintf(fid,'Generated steps:\n');
for i=1:height(status.steps)
    fprintf(fid,'  - %s: %s -> %s\n', char(status.steps.step(i)), char(status.steps.status(i)), char(status.steps.path(i)));
end
fclose(fid);
end

function ensure_dir_local(d), if ~isempty(d) && exist(d,'dir')~=7, mkdir(d); end, end
function x = getfield_def_local(S,fn,def)
if isstruct(S) && isfield(S,fn) && ~isempty(S.(fn)), x=S.(fn); else, x=def; end
end
function r = row_local(x), r=x(:).'; end
