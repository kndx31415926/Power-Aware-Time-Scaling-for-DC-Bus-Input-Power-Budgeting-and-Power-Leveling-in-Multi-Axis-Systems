%% RUN_01_train_models_oneclick.m
% One-click training entry for the paper experiments.
%
% This script does exactly three things:
%   1) run a no-NN smoke check;
%   2) generate the teacher dataset and train the TRF model;
%   3) train the compact MLP-PRG learned-surrogate variant from the same dataset.
%
% It intentionally does NOT run NN-rollout self-training or any extra
% experimental augmentation.  If you changed spinn3d_params_block.m, clean
% old data_alpha / output folders first, then run this file once.

clear; clc; close all;

%% ================= USER SETTINGS =================
TRAIN = struct();
TRAIN.project_dir = fileparts(mfilename('fullpath'));
if isempty(TRAIN.project_dir), TRAIN.project_dir = pwd; end

TRAIN.do_precheck = true;
TRAIN.do_onecase_check = true;
TRAIN.use_parallel_pool = true;
TRAIN.pool_type = 'local';
TRAIN.num_workers = 20;             % set [] if MATLAB should decide
TRAIN.force_pool_recreate = true;
TRAIN.rng_seed = 1;

TRAIN.train_mlp = true;
TRAIN.mlp_epochs = 80;
TRAIN.mlp_batch = 512;
TRAIN.mlp_lr = 1e-3;
TRAIN.mlp_hidden = [512 256 64];

TRAIN.output_dir = fullfile(TRAIN.project_dir,'training_outputs_oneclick');
%% =================================================

rng(TRAIN.rng_seed);
assert(exist(TRAIN.project_dir,'dir')==7, 'project_dir does not exist: %s', TRAIN.project_dir);
cd(TRAIN.project_dir);
addpath(genpath(TRAIN.project_dir));
ensure_dir_local(TRAIN.output_dir);

log_file = fullfile(TRAIN.output_dir, ['training_log_' datestr(now,'yyyymmdd_HHMMSS') '.txt']);
diary(log_file);
cleanup_diary = onCleanup(@() diary('off')); %#ok<NASGU>

fprintf('\n============================================================\n');
fprintf('RUN_01_train_models_oneclick started at %s\n', datestr(now));
fprintf('Project directory: %s\n', TRAIN.project_dir);
fprintf('Output directory : %s\n', TRAIN.output_dir);
fprintf('============================================================\n\n');

PB = spinn3d_params_block();
cfg = spinn3d_params_to_cfg(PB,'demo_alpha');
cfg.NO_FIGURE = true;
cfg.DISABLE_FIGURES = true;
print_config_local(cfg, PB);

if TRAIN.use_parallel_pool && exist('paper_open_parallel_pool','file')==2
    try
        paper_open_parallel_pool(struct('use_parallel',true,'pool_type',TRAIN.pool_type, ...
            'num_workers',TRAIN.num_workers,'force_recreate',TRAIN.force_pool_recreate));
    catch ME
        warning('Could not start parallel pool: %s', ME.message);
    end
end

if TRAIN.do_precheck
    fprintf('\n[1/5] Precheck without NN models ...\n');
    try
        out_base_pre = spinn3d_demo_ctcgov_online_nonn_nolimit(cfg); %#ok<NASGU>
        out_feas_pre = spinn3d_demo_ctcgov_online_nonn(cfg); %#ok<NASGU>
        save(fullfile(TRAIN.output_dir,'precheck_no_nn.mat'),'out_base_pre','out_feas_pre','cfg','PB','-v7.3');
        fprintf('Precheck OK: NoGuard t_reach=%.4f s | Feasible-RG t_reach=%.4f s\n', ...
            safe_field_local(out_base_pre,'t_reach',NaN), safe_field_local(out_feas_pre,'t_reach',NaN));
    catch ME
        fprintf(2,'Precheck failed. Fix this before training.\n%s\n', getReport(ME,'extended','hyperlinks','off'));
        rethrow(ME);
    end
    close all;
end

fprintf('\n[2/5] Teacher dataset generation + TRF training ...\n');
t_trf = tic;
out_train = spinn3d_run_alpha_gov_budget_trf();
trf_runtime_s = toc(t_trf);
assert(isfield(out_train,'dataset_master') && exist(out_train.dataset_master,'file')==2, 'Dataset master was not generated.');
assert(isfield(out_train,'model') && exist(out_train.model,'file')==2, 'TRF model was not generated.');

Smodel = load(out_train.model,'model');
trf_model = Smodel.model;
export_model_report_local(TRAIN.output_dir,'trf_final',trf_model,trf_runtime_s);
fprintf('TRF training complete in %.1f s. Model: %s\n', trf_runtime_s, out_train.model);

fprintf('\n[3/5] MLP-PRG compact learned-surrogate training from the same dataset ...\n');
model_mlp = [];
mlp_runtime_s = NaN;
mlp_model_path = resolve_mlp_path_local(PB);
if TRAIN.train_mlp
    mlp_opts = struct();
    mlp_opts.MaxEpochs = TRAIN.mlp_epochs;
    mlp_opts.MiniBatchSize = TRAIN.mlp_batch;
    mlp_opts.InitialLearnRate = TRAIN.mlp_lr;
    mlp_opts.Hidden1 = TRAIN.mlp_hidden(1);
    mlp_opts.Hidden2 = TRAIN.mlp_hidden(2);
    mlp_opts.Hidden3 = TRAIN.mlp_hidden(3);
    mlp_opts.save_model = mlp_model_path;
    mlp_opts.Verbose = true;
    mlp_opts.seed = TRAIN.rng_seed;
    mlp_opts.alpha_min = double(PB.alpha.alpha_min);
    t_mlp = tic;
    model_mlp = spinn3d_train_alpha_gov_mlp(out_train.dataset_master, mlp_opts);
    mlp_runtime_s = toc(t_mlp);
    export_model_report_local(TRAIN.output_dir,'mlp_final',model_mlp,mlp_runtime_s);
    assert(exist(mlp_model_path,'file')==2, 'MLP model was not generated: %s', mlp_model_path);
    fprintf('MLP training complete in %.1f s. Model: %s\n', mlp_runtime_s, mlp_model_path);
end

fprintf('\n[4/5] Checking model files and metadata ...\n');
check_model_files_local(PB, out_train.model, mlp_model_path);

if TRAIN.do_onecase_check
    fprintf('\n[5/5] One-case sanity check with final TRF/MLP models ...\n');
    try
        cfg = spinn3d_params_to_cfg(PB,'demo_alpha');
        cfg.NO_FIGURE = true;
        cfg.DISABLE_FIGURES = true;
        cfg.MODEL_PATH = out_train.model;
        cfg.MLP_MODEL_PATH = mlp_model_path;
        out_trf_one  = spinn3d_demo_ctcgov_online_nn(cfg); %#ok<NASGU>
        out_mlp_one  = spinn3d_demo_ctcgov_online_mlp(cfg); %#ok<NASGU>
        out_feas_one = spinn3d_demo_ctcgov_online_nonn(cfg); %#ok<NASGU>
        out_base_one = spinn3d_demo_ctcgov_online_nonn_nolimit(cfg); %#ok<NASGU>
        save(fullfile(TRAIN.output_dir,'onecase_after_training.mat'), ...
            'out_trf_one','out_mlp_one','out_feas_one','out_base_one','cfg','PB','-v7.3');
        Pbud_one = safe_field_local(cfg,'P_GRID_BUDGET',safe_field_local(cfg,'P_TOTAL_MAX',15));
        rows = [ ...
            onecase_metrics_local('NoGuard',out_base_one,Pbud_one); ...
            onecase_metrics_local('Feasible-RG',out_feas_one,Pbud_one); ...
            onecase_metrics_local('MLP-PRG',out_mlp_one,Pbud_one); ...
            onecase_metrics_local('TRF-PRG',out_trf_one,Pbud_one)];
        T_one = struct2table(rows);
        writetable(T_one, fullfile(TRAIN.output_dir,'onecase_after_training.csv'));
        disp(T_one);
    catch ME
        warning('One-case sanity check failed after training: %s', ME.message);
    end
    close all;
end

manifest = struct();
manifest.created_at = datestr(now);
manifest.project_dir = TRAIN.project_dir;
manifest.output_dir = TRAIN.output_dir;
manifest.dataset_master = out_train.dataset_master;
manifest.trf_model_path = out_train.model;
manifest.mlp_model_path = mlp_model_path;
manifest.trf_train_runtime_s = trf_runtime_s;
manifest.mlp_train_runtime_s = mlp_runtime_s;
manifest.TRAIN = TRAIN;
manifest.PB = PB;
save(fullfile(TRAIN.output_dir,'training_manifest.mat'),'manifest','out_train','trf_model','model_mlp','-v7.3');

fprintf('\n============================================================\n');
fprintf('TRAINING COMPLETE at %s\n', datestr(now));
fprintf('Dataset master : %s\n', manifest.dataset_master);
fprintf('Final TRF model: %s\n', manifest.trf_model_path);
fprintf('Final MLP model: %s\n', manifest.mlp_model_path);
fprintf('Training outputs: %s\n', TRAIN.output_dir);
fprintf('Log file: %s\n', log_file);
fprintf('============================================================\n');

%% ================= LOCAL FUNCTIONS =================
function p = resolve_mlp_path_local(PB)
    if isfield(PB,'paths') && isfield(PB.paths,'model_alpha_mlp') && ~isempty(PB.paths.model_alpha_mlp)
        p = PB.paths.model_alpha_mlp;
    else
        p = fullfile(PB.paths.root,'model_alpha_gov_mlp.mat');
    end
end

function check_model_files_local(PB, trf_model_path, mlp_model_path)
    dataset_master = PB.paths.dataset_alpha_gov_seq_master;
    assert(exist(dataset_master,'file')==2, 'Missing dataset master: %s', dataset_master);
    assert(exist(trf_model_path,'file')==2, 'Missing TRF model: %s', trf_model_path);
    assert(exist(mlp_model_path,'file')==2, 'Missing MLP model: %s', mlp_model_path);
    S = load(dataset_master,'alpha_min','K','ds','Ts_servo','policy_dt','teacher_diag','y');
    fprintf('Dataset master OK: %s\n', dataset_master);
    fprintf('  alpha_min = %.6g | K = %g | ds = %.6g | Ts = %.6g | policy_dt = %.6g\n', ...
        safe_field_local(S,'alpha_min',NaN), safe_field_local(S,'K',NaN), ...
        safe_field_local(S,'ds',NaN), safe_field_local(S,'Ts_servo',NaN), safe_field_local(S,'policy_dt',NaN));
    if isfield(S,'y') && isfield(S,'alpha_min')
        fprintf('  label-at-floor = %.2f %%\n', 100*mean(double(S.y(:)) <= double(S.alpha_min) + 1e-4));
    end
    if isfield(S,'teacher_diag') && isstruct(S.teacher_diag)
        fprintf('  teacher floor infeasible = %.3g | fallback = %.3g | noninterval = %.3g\n', ...
            safe_field_local(S.teacher_diag,'floor_infeasible_rate',NaN), ...
            safe_field_local(S.teacher_diag,'fallback_rate',NaN), ...
            safe_field_local(S.teacher_diag,'noninterval_rate',NaN));
    end
end

function print_config_local(cfg, PB)
    fprintf('[CONFIG]\n');
    fprintf('  Servo Ts                 = %.6g s\n', safe_field_local(cfg,'TS',NaN));
    if isfield(cfg,'POLICY') && isstruct(cfg.POLICY) && isfield(cfg.POLICY,'dt')
        fprintf('  Policy dt                = %.6g s\n', cfg.POLICY.dt);
    elseif isfield(cfg,'POLICY_DT')
        fprintf('  Policy dt                = %.6g s\n', cfg.POLICY_DT);
    end
    fprintf('  Active P_GRID_BUDGET      = %.6g W\n', safe_field_local(cfg,'P_GRID_BUDGET',safe_field_local(cfg,'P_TOTAL_MAX',NaN)));
    fprintf('  Hardware P_HW_RATING      = %.6g W\n', safe_field_local(cfg,'P_HW_RATING',NaN));
    fprintf('  alpha_min                 = %.6g\n', safe_field_local(cfg,'ALPHA_MIN',NaN));
    if isfield(PB,'alpha_budget') && isfield(PB.alpha_budget,'n_traj')
        fprintf('  teacher trajectories      = %d\n', round(PB.alpha_budget.n_traj));
    end
    if isfield(cfg,'MODEL_PATH'), fprintf('  TRF model path            = %s\n', cfg.MODEL_PATH); end
end

function export_model_report_local(outdir, prefix, model, runtime_s)
    if isfield(model,'train_info')
        Tinfo = struct_to_table_flat_local(model.train_info);
        writetable(Tinfo, fullfile(outdir,[prefix '_train_info.csv']));
    end
    if isfield(model,'val_metrics')
        Tval = struct_to_table_flat_local(model.val_metrics);
        writetable(Tval, fullfile(outdir,[prefix '_val_metrics.csv']));
    end
    Trun = table(string(prefix), runtime_s, 'VariableNames', {'model','training_runtime_s'});
    writetable(Trun, fullfile(outdir,[prefix '_training_runtime.csv']));
end

function rec = onecase_metrics_local(name,out,Pbud)
    rec = struct('method',string(name),'t_reach',NaN,'pbar_p99',NaN,'pbar_peak',NaN, ...
        'E_excess_norm_s',NaN,'E_grid',NaN,'E_reuse',NaN,'E_dump',NaN, ...
        'Psource_shortfall_peak',NaN,'I_peak',NaN,'Vdc_min',NaN);
    rec.t_reach = safe_field_local(out,'t_reach',NaN);
    try
        M = spinn3d_compute_power_metrics(out,Pbud,struct('power_eval_mode','lpf','power_tau_s',0.05,'Pbudget_margin_frac',0.05));
        fn = fieldnames(rec);
        for i=1:numel(fn)
            if isfield(M,fn{i}), rec.(fn{i}) = M.(fn{i}); end
        end
    catch
    end
end

function T = struct_to_table_flat_local(S)
    rows = struct('field',{},'value',{});
    add_fields('', S);
    T = struct2table(rows);
    function add_fields(prefix, X)
        if isstruct(X)
            f = fieldnames(X);
            for ii=1:numel(f)
                if isempty(prefix), name=f{ii}; else, name=[prefix '.' f{ii}]; end
                add_fields(name, X.(f{ii}));
            end
        else
            r.field = string(prefix);
            if isnumeric(X) || islogical(X)
                if isscalar(X), r.value = string(num2str(double(X),'%.12g'));
                else, r.value = string(mat2str(X)); end
            elseif ischar(X), r.value = string(X);
            elseif isstring(X), r.value = strjoin(X, ', ');
            else, r.value = string(class(X)); end
            rows(end+1)=r; %#ok<AGROW>
        end
    end
end

function x = safe_field_local(S,fn,def)
    if isstruct(S) && isfield(S,fn) && ~isempty(S.(fn))
        x = S.(fn);
    else
        x = def;
    end
end

function ensure_dir_local(d)
    if ~isempty(d) && exist(d,'dir')~=7, mkdir(d); end
end
