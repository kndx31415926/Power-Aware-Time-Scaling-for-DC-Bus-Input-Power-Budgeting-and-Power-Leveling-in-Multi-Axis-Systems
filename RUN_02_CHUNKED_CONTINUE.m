%% RUN_02_CHUNKED_CONTINUE.m
% Single RUN_02 entry point for resumable paper paired-trial evaluations.
%
% Use this one script only.
%
% Behavior:
%   - If MAIN4 is not complete, this run adds the next RUN.add_trials trials
%     for NoGuard / Feasible-RG / MLP-PRG / TRF-PRG.
%   - After MAIN4 reaches 100/100, the same script automatically switches to
%     FAIR6 and adds the next RUN.add_trials trials for the six-method fair
%     baseline comparison.
%   - It prints the global trial_id currently being run.
%   - It writes raw CSV files plus canonical manuscript-facing CSV aliases.
%
% Example:
%   First run  with RUN.add_trials = 10 -> runs MAIN4 trial 1:10.
%   Second run with RUN.add_trials = 20 -> runs MAIN4 trial 11:30.
%   After MAIN4 is complete, the next run automatically starts FAIR6.
%
% Do not change PB parameters, trained model files, seeds, q_margin_deg, or
% payload range after starting a result folder. If you change them, delete the
% corresponding folder under paper_outputs_chunked_100 and restart from trial 1.

clear; clc; close all;

%% ================= USER SETTINGS =================
RUN = struct();
RUN.project_dir = fileparts(mfilename('fullpath'));
if isempty(RUN.project_dir), RUN.project_dir = pwd; end

% Number of NEW paired trials to add in this MATLAB run.
% Change only this number for normal use.
RUN.add_trials = 10;

% Fixed deterministic trial universe.
RUN.total_N = 100;
RUN.seed_main4 = 1;
RUN.seed_fair6 = 102;
RUN.results_root = fullfile(RUN.project_dir, 'paper_outputs_chunked_100');

% Trial distribution. Keep unchanged after starting a folder.
RUN.q_margin_deg = 10;
RUN.payload_mass_range = [0.20 0.50];
RUN.pbudget_mode_eval = 'fixed';

% Parallel settings.
RUN.use_parallel_pool = true;
RUN.pool_type = 'local';
RUN.num_workers = 20;          % set [] to let MATLAB decide; reduce if memory is tight
RUN.force_pool_recreate = false;

% Expensive fair-baseline options, used only for FAIR6.
RUN.PTP_OPTS = struct('Ns',61,'Na',25);
RUN.MPC_OPTS = struct('Na',11,'H',4,'beamN',10,'allow_soft_violation',false, ...
                      'budget_lpf_ratio',0.98,'raw_guard_ratio',1.50);

% Safety: keep false. If model/budget/seed/config changes, start a new folder.
RUN.allow_manifest_mismatch = false;
%% =================================================

cd(RUN.project_dir);
addpath(genpath(RUN.project_dir));
ensure_dir_local(RUN.results_root);

PB  = spinn3d_params_block();
cfg = spinn3d_params_to_cfg(PB,'demo_alpha');
cfg.NO_FIGURE = true;
cfg.DISABLE_FIGURES = true;

trf_model_path = char(getfield_def_local(cfg,'MODEL_PATH',PB.paths.model_alpha));
mlp_model_path = char(PB.paths.model_alpha_mlp);
assert(exist(trf_model_path,'file')==2, 'TRF model not found: %s\nRun RUN_01_train_models_oneclick first.', trf_model_path);
assert(exist(mlp_model_path,'file')==2, 'MLP model not found: %s\nRun RUN_01_train_models_oneclick first.', mlp_model_path);
cfg.MODEL_PATH = trf_model_path;
cfg.MLP_MODEL_PATH = mlp_model_path;

main_def = make_mode_def_local('main4', RUN);
fair_def = make_mode_def_local('fair6', RUN);
main_done = completed_ids_for_def_local(RUN.results_root, main_def);
fair_done = completed_ids_for_def_local(RUN.results_root, fair_def);

fprintf('\n============================================================\n');
fprintf('RUN_02_CHUNKED_CONTINUE started at %s\n', datestr(now));
fprintf('Project dir   : %s\n', RUN.project_dir);
fprintf('Active budget : %.6g W\n', getfield_def_local(cfg,'P_GRID_BUDGET',NaN));
fprintf('TRF model     : %s\n', trf_model_path);
fprintf('MLP model     : %s\n', mlp_model_path);
fprintf('MAIN4 progress: %d/%d paired trials\n', numel(main_done), RUN.total_N);
fprintf('FAIR6 progress: %d/%d paired trials\n', numel(fair_done), RUN.total_N);
fprintf('This run will add at most %d paired trials to the next incomplete stage.\n', RUN.add_trials);
fprintf('============================================================\n');

if numel(main_done) < RUN.total_N
    run_def = main_def;
elseif numel(fair_done) < RUN.total_N
    run_def = fair_def;
else
    fprintf('\nAll MAIN4 and FAIR6 paired-trial evaluations are complete.\n');
    fprintf('Run RUN_02_CHECK_PROGRESS to view result paths.\n');
    return;
end

export_dir = fullfile(RUN.results_root, run_def.subdir);
ensure_dir_local(export_dir);

log_name = sprintf('RUN_02_%s_chunk_log_%s.txt', lower(run_def.label), datestr(now,'yyyymmdd_HHMMSS'));
diary(fullfile(RUN.results_root, log_name));
cDiary = onCleanup(@() diary('off')); %#ok<NASGU>

signature = make_signature_local(RUN, cfg, run_def, trf_model_path, mlp_model_path);
check_or_write_manifest_local(export_dir, signature, RUN.allow_manifest_mismatch);

csv_trials = fullfile(export_dir,'fair_baseline_trials.csv');
done_ids = completed_ids_local(csv_trials, run_def.methods);
missing_ids = setdiff(1:RUN.total_N, done_ids, 'stable');

if isempty(missing_ids)
    fprintf('[%s] Already complete. Regenerating summaries only.\n', run_def.label);
    next_ids = 1:RUN.total_N;
else
    next_ids = missing_ids(1:min(RUN.add_trials, numel(missing_ids)));
    fprintf('\n[%s] This run will add trial_id: %s\n', run_def.label, compact_id_list_local(next_ids));
    fprintf('[%s] This equals %d paired trials x %d methods = %d method runs.\n\n', ...
        run_def.label, numel(next_ids), numel(run_def.methods), numel(next_ids)*numel(run_def.methods));
end

if RUN.use_parallel_pool && exist('paper_open_parallel_pool','file')==2
    paper_open_parallel_pool(struct('use_parallel',true,'pool_type',RUN.pool_type, ...
        'num_workers',RUN.num_workers,'force_recreate',RUN.force_pool_recreate));
end

opts = struct();
opts.N_total = RUN.total_N;
opts.trial_ids = next_ids;
opts.resume = true;
opts.resume_force_rerun = false;
opts.seed = run_def.seed;
opts.q_margin_deg = RUN.q_margin_deg;
opts.payload_mass_range = RUN.payload_mass_range;
opts.pbudget_mode_eval = RUN.pbudget_mode_eval;
opts.power_eval_mode = 'lpf';
opts.power_eval_tau_s = 0.05;
opts.power_tau_s = 0.05;
opts.Pbudget_margin_frac = 0.05;
opts.reference_method = 'NoGuard';
opts.methods = run_def.methods;
opts.use_parallel = RUN.use_parallel_pool;
opts.pool_type = RUN.pool_type;
opts.num_workers = RUN.num_workers;
opts.force_recreate = RUN.force_pool_recreate;
opts.export_dir = export_dir;
opts.mlp_model_path = mlp_model_path;
opts.progress_print = true;
opts.progress_label = run_def.label;
opts.verbose = true;
Pbud = getfield_def_local(cfg,'P_GRID_BUDGET',getfield_def_local(cfg,'P_TOTAL_MAX',NaN));
opts.pbudget_range_W = [Pbud Pbud];
if strcmpi(run_def.key,'fair6')
    opts.PTP_OPTS = RUN.PTP_OPTS;
    opts.MPC_OPTS = RUN.MPC_OPTS;
end

R = paper_fair_baseline_eval(cfg, opts); %#ok<NASGU>

% Write canonical manuscript-facing CSVs with clear variable names/units.
% The raw fair_baseline_*.csv files are kept for backward compatibility.
try
    paper_standardize_run02_outputs(export_dir, run_def.label, run_def.methods);
    paper_write_run02_output_index(RUN.results_root);
catch MEstd
    warning('[RUN02] Could not write canonical output aliases: %s', MEstd.message);
end

done_ids_after = completed_ids_local(csv_trials, run_def.methods);
missing_after = setdiff(1:RUN.total_N, done_ids_after, 'stable');
fprintf('\n============================================================\n');
fprintf('[%s] After this run: %d/%d paired trials complete.\n', run_def.label, numel(done_ids_after), RUN.total_N);
if isempty(missing_after)
    fprintf('[%s] COMPLETE. Summary CSV:\n  %s\n', run_def.label, fullfile(export_dir,'fair_baseline_summary.csv'));
    fprintf('[%s] Paired stats CSV:\n  %s\n', run_def.label, fullfile(export_dir,'fair_baseline_paired_vs_reference.csv'));
    if strcmpi(run_def.key,'main4')
        fprintf('\nNext run of RUN_02_CHUNKED_CONTINUE will automatically start FAIR6.\n');
    else
        fprintf('\nMAIN4 and FAIR6 are complete. Use these CSV files to fill Tables 9--11.\n');
    end
else
    nNextDefault = min(RUN.add_trials, numel(missing_after));
    fprintf('[%s] Next missing starts at trial_id %d.\n', run_def.label, missing_after(1));
    fprintf('[%s] If you keep RUN.add_trials=%d, next run will execute: %s\n', ...
        run_def.label, RUN.add_trials, compact_id_list_local(missing_after(1:nNextDefault)));
end
fprintf('============================================================\n');

save(fullfile(RUN.results_root, sprintf('RUN_02_%s_last_status.mat', lower(run_def.label))), ...
    'RUN','cfg','PB','run_def','export_dir','done_ids_after','missing_after','signature','-v7.3');

%% ================= LOCAL FUNCTIONS =================
function def = make_mode_def_local(key, RUN)
def = struct();
def.key = lower(key);
switch def.key
    case 'main4'
        def.label = 'MAIN4';
        def.methods = {'NoGuard','Feasible-RG','MLP-PRG','TRF-PRG'};
        def.seed = RUN.seed_main4;
        def.subdir = sprintf('main_4methods_N%d_seed%d', RUN.total_N, def.seed);
    case 'fair6'
        def.label = 'FAIR6';
        def.methods = {'NoGuard','Feasible-RG','TRF-PRG','MLP-PRG','Bus-PTP-DP','MPC-RG-grid'};
        def.seed = RUN.seed_fair6;
        def.subdir = sprintf('fair_6methods_N%d_seed%d', RUN.total_N, def.seed);
    otherwise
        error('Unknown mode key: %s', key);
end
end

function ids = completed_ids_for_def_local(root, def)
ids = completed_ids_local(fullfile(root, def.subdir, 'fair_baseline_trials.csv'), def.methods);
end

function signature = make_signature_local(RUN, cfg, def, trf_model_path, mlp_model_path)
signature = struct();
signature.version = 'RUN_02_CHUNKED_CONTINUE_clean_v20260701';
signature.mode = def.key;
signature.export_subdir = def.subdir;
signature.total_N = RUN.total_N;
signature.seed = def.seed;
signature.methods = def.methods;
signature.q_margin_deg = RUN.q_margin_deg;
signature.payload_mass_range = RUN.payload_mass_range;
signature.pbudget_mode_eval = RUN.pbudget_mode_eval;
signature.P_GRID_BUDGET = getfield_def_local(cfg,'P_GRID_BUDGET',NaN);
signature.P_TOTAL_MAX = getfield_def_local(cfg,'P_TOTAL_MAX',NaN);
signature.ALPHA_MIN = getfield_def_local(cfg,'ALPHA_MIN',NaN);
signature.MODEL_PATH = char(trf_model_path);
signature.MLP_MODEL_PATH = char(mlp_model_path);
signature.params_file = which('spinn3d_params_block');
signature.controller_file = which('spinn3d_controller_ctcgov');
end

function check_or_write_manifest_local(export_dir, signature, allow_mismatch)
fn = fullfile(export_dir,'RUN_02_chunk_manifest.mat');
if exist(fn,'file') ~= 2
    save(fn,'signature','-v7.3');
    fprintf('[manifest] Created new manifest: %s\n', fn);
    return;
end
S = load(fn,'signature');
old = S.signature;
if isequaln(old, signature)
    fprintf('[manifest] Existing manifest matches current settings.\n');
    return;
end
msg = sprintf(['RUN_02 manifest mismatch in:\n  %s\n', ...
    'This means parameters/model/seed/trial distribution changed after the first chunk.\n', ...
    'Delete this export folder to restart cleanly; do not mix incompatible batches.'], export_dir);
if allow_mismatch
    warning('%s\nProceeding because RUN.allow_manifest_mismatch=true.', msg);
else
    fprintf('\n[manifest] OLD settings:\n'); disp(old);
    fprintf('[manifest] NEW settings:\n'); disp(signature);
    error('%s', msg);
end
end

function ids = completed_ids_local(csv_trials, methods)
ids = [];
if exist(csv_trials,'file') ~= 2, return; end
try
    T = readtable(csv_trials);
catch
    return;
end
if ~ismember('trial_id',T.Properties.VariableNames) || ~ismember('method',T.Properties.VariableNames)
    return;
end
if ~ismember('success',T.Properties.VariableNames)
    T.success = false(height(T),1);
end
mcol = string(T.method);
all_ids = unique(T.trial_id(:).');
for id = all_ids
    ok = true;
    for k = 1:numel(methods)
        idx = T.trial_id==id & mcol==string(methods{k}) & logical(T.success);
        if ~any(idx), ok = false; break; end
    end
    if ok, ids(end+1) = id; end %#ok<AGROW>
end
ids = sort(unique(ids));
end

function s = compact_id_list_local(ids)
ids = ids(:).';
if isempty(ids), s='(none)'; return; end
ids = sort(unique(ids));
out = {}; a = ids(1); b = ids(1);
for k=2:numel(ids)
    if ids(k)==b+1
        b = ids(k);
    else
        out{end+1} = range_str_local(a,b); %#ok<AGROW>
        a = ids(k); b = ids(k);
    end
end
out{end+1} = range_str_local(a,b);
s = strjoin(out, ',');
end

function s = range_str_local(a,b)
if a==b, s=sprintf('%d',a); else, s=sprintf('%d:%d',a,b); end
end


function write_output_index_local(results_root, main_def, fair_def)
rows = {};
defs = [main_def, fair_def];
for i=1:numel(defs)
    d = defs(i);
    stage_dir = fullfile(results_root, d.subdir);
    files = { ...
        'raw_trials','fair_baseline_trials.csv','Raw per-method per-trial metrics; contains rollout_wall_time_s for full simulated rollout wall time, not neural inference latency'; ...
        'raw_summary','fair_baseline_summary.csv','Raw method summary over completed paired trials'; ...
        'raw_paired','fair_baseline_paired_vs_reference.csv','Raw paired comparison versus NoGuard'; ...
        'canonical_trials',sprintf('RUN02_%s_trials_canonical.csv',d.label),'Canonical per-method per-trial table with clear units and manuscript-oriented names'; ...
        'canonical_summary',sprintf('RUN02_%s_summary_canonical.csv',d.label),'Canonical method summary table with metric dictionary fields'; ...
        'canonical_paired',sprintf('RUN02_%s_paired_vs_NoGuard_canonical.csv',d.label),'Canonical paired comparison table'; ...
        'variable_dictionary',sprintf('RUN02_%s_variable_dictionary.csv',d.label),'Variable dictionary and mapping from raw metric names to manuscript names'};
    for k=1:size(files,1)
        r = struct();
        r.stage = string(d.label);
        r.kind = string(files{k,1});
        r.path = string(fullfile(stage_dir, files{k,2}));
        r.exists = exist(char(r.path),'file')==2;
        r.description = string(files{k,3});
        rows{end+1,1} = r; %#ok<AGROW>
    end
end
if isempty(rows), return; end
T = struct2table(vertcat(rows{:}));
writetable(T, fullfile(results_root,'RUN02_OUTPUT_INDEX.csv'));
fprintf('[RUN02] Output index: %s\n', fullfile(results_root,'RUN02_OUTPUT_INDEX.csv'));
end

function x = getfield_def_local(S, f, d)
if isstruct(S) && isfield(S,f) && ~isempty(S.(f)), x = S.(f); else, x = d; end
end

function ensure_dir_local(d)
if ~isempty(d) && exist(d,'dir')~=7, mkdir(d); end
end
