function R = paper_fair_baseline_eval(varargin)
%PAPER_FAIR_BASELINE_EVAL  Parallel fair-baseline evaluation for revision.
%
% Methods (same trajectory, plant, inner CTC, bus model, metrics):
%   NoGuard        : budget-agnostic PRG baseline
%   Feasible-RG    : online feasibility-search teacher/oracle baseline
%   TRF-PRG        : proposed look-ahead sequence learned PRG surrogate
%   MLP-PRG        : proposed compact non-attention learned PRG surrogate
%   Bus-PTP-DP     : offline power-constrained path time-parameterization
%   MPC-RG-grid    : receding-horizon RG grid-search baseline
%
% Usage:
%   R = paper_fair_baseline_eval();
%   R = paper_fair_baseline_eval(cfg, opts);
%
% Important options:
%   opts.use_parallel = true; opts.num_workers = [];
%   opts.N = 100; opts.methods = {'NoGuard','Feasible-RG','TRF-PRG','MLP-PRG','Bus-PTP-DP','MPC-RG-grid'};

[cfg_base, opts] = parse_inputs(varargin{:});
ensure_dir(opts.export_dir);

if opts.use_parallel
    paper_open_parallel_pool(opts);
end

trials = make_trials(cfg_base, opts);
methods = opts.methods;
metricList = default_metric_list();

csv_trials = fullfile(opts.export_dir,'fair_baseline_trials.csv');
csv_summary = fullfile(opts.export_dir,'fair_baseline_summary.csv');
csv_paired = fullfile(opts.export_dir,'fair_baseline_paired_vs_reference.csv');

existingT = table();
if opts.resume && exist(csv_trials,'file')==2
    try
        existingT = readtable(csv_trials);
        existingT = normalize_existing_table(existingT);
        fprintf('[fair/resume] loaded existing table: %s (%d rows)\n', csv_trials, height(existingT));
    catch ME
        warning('[fair/resume] Could not read existing table, starting from scratch: %s', ME.message);
        existingT = table();
    end
end

trials_to_run = trials;
if opts.resume && ~opts.resume_force_rerun && ~isempty(existingT)
    done = completed_trial_mask(existingT, trials, methods);
    trials_to_run = trials(~done);
    if opts.verbose
        fprintf('[fair/resume] requested trial ids: %s\n', compact_id_list([trials.trial_id]));
        fprintf('[fair/resume] completed trial ids: %s\n', compact_id_list([trials(done).trial_id]));
        fprintf('[fair/resume] missing trial ids:   %s\n', compact_id_list([trials_to_run.trial_id]));
    end
end

rowCells = cell(numel(trials_to_run),1);
tAll = tic;
if ~isempty(trials_to_run)
    nRun = numel(trials_to_run);
    label = char(getfield_def(opts,'progress_label','fair'));
    progressPrint = logical(getfield_def(opts,'progress_print',true));

    if opts.use_parallel && ~isempty(gcp('nocreate'))
        % With PARFOR, use DataQueue so the client command window still shows
        % which global paired-trial id has started/finished. Messages are
        % asynchronous, so the order can differ slightly from trial_id order.
        dq = [];
        useDQ = false;
        if progressPrint
            try
                dq = parallel.pool.DataQueue;
                afterEach(dq, @(msg) fprintf('%s\n', char(msg)));
                useDQ = true;
                fprintf('[%s] parallel progress enabled for %d paired trials.\n', label, nRun);
            catch ME
                warning('[%s] Could not create parallel DataQueue for progress printing: %s', label, ME.message);
            end
        end
        parfor ii = 1:nRun
            tid = trials_to_run(ii).trial_id;
            if useDQ
                send(dq, sprintf('[%s] START trial_id %d (%d/%d in this run)', label, tid, ii, nRun));
            end
            rowCells{ii} = run_one_trial(trials_to_run(ii), methods, metricList, opts);
            if useDQ
                send(dq, sprintf('[%s] DONE  trial_id %d (%d/%d in this run)', label, tid, ii, nRun));
            end
        end
    else
        for ii = 1:nRun
            tid = trials_to_run(ii).trial_id;
            if progressPrint || opts.verbose
                fprintf('[%s] START trial_id %d (%d/%d in this run)\n', label, tid, ii, nRun);
            end
            rowCells{ii} = run_one_trial(trials_to_run(ii), methods, metricList, opts);
            if progressPrint || opts.verbose
                fprintf('[%s] DONE  trial_id %d (%d/%d in this run)\n', label, tid, ii, nRun);
            end
        end
    end
    allRows = vertcat(rowCells{:});
    Tnew = struct2table(allRows);
    Tnew = normalize_existing_table(Tnew);
else
    Tnew = table();
    fprintf('[fair/resume] no missing trials in this chunk; only regenerating summaries.\n');
end

T = merge_trial_tables(existingT, Tnew);
if isempty(T)
    warning('[fair] no trial rows available.');
    S = table();
    PS = table();
else
    S = summarize_by_method(T, metricList);
    PS = paired_against_reference(T, metricList, opts.reference_method);
end

writetable(T,csv_trials);
writetable(S,csv_summary);
writetable(PS,csv_paired);
save(fullfile(opts.export_dir,'fair_baseline_eval.mat'),'T','S','PS','opts','cfg_base','trials','trials_to_run','methods','metricList');

R = struct('trials_table',T,'summary_table',S,'paired_stats_table',PS, ...
           'export_dir',opts.export_dir,'run02_script_wall_time_s',toc(tAll), ...
           'methods',{methods},'metricList',{metricList});
fprintf('[fair] done: requested=%d, ran_now=%d, total_rows=%d, methods=%d, script_wall_time=%.1f s\n', ...
    numel(trials), numel(trials_to_run), height(T), numel(methods), R.run02_script_wall_time_s);
fprintf('[fair] summary: %s\n', csv_summary);
end

function rows = run_one_trial(trial, methods, metricList, opts)
rows = repmat(init_row(metricList), numel(methods), 1);
for m = 1:numel(methods)
    method = methods{m};
    row = init_row(metricList);
    row.trial_id = trial.trial_id;
    row.method = string(method);
    row.payload_mass_kg = trial.payload_mass_kg;
    row.P_grid_budget_W = trial.P_grid_budget_W;
    for j = 1:numel(trial.q0_deg)
        row.(sprintf('q0_deg_j%d',j)) = trial.q0_deg(j);
        row.(sprintf('qf_deg_j%d',j)) = trial.qf_deg(j);
    end
    cfg_i = trial.cfg;
    cfg_i.NO_FIGURE = true;
    cfg_i.DISABLE_FIGURES = true;
    try
        t0 = tic;
        out = run_method(method, cfg_i, opts);
        row.rollout_wall_time_s = toc(t0);
        M = spinn3d_compute_power_metrics(out, trial.P_grid_budget_W, opts);
        for k=1:numel(metricList)
            fn = metricList{k};
            if isfield(M,fn), row.(fn) = M.(fn); end
        end
        row.success = true;
        row.error = string('');
        if isfield(out,'profile') && isfield(out.profile,'solve_time_s')
            row.offline_profile_solve_wall_time_s = out.profile.solve_time_s;
        end
    catch ME
        row.success = false;
        row.error = string(ME.message);
        row.rollout_wall_time_s = NaN;
    end
    rows(m) = row;
end
end

function out = run_method(method, cfg, opts)
switch lower(strrep(method,'_','-'))
    case {'noguard','baseline','budget-agnostic'}
        out = spinn3d_demo_ctcgov_online_nonn_nolimit(cfg);
    case {'feasible-rg','feasible','oracle'}
        out = spinn3d_demo_ctcgov_online_nonn(cfg);
    case {'trf-prg','trf','nn'}
        out = spinn3d_demo_ctcgov_online_nn(cfg);
    case {'mlp-prg','mlp'}
        if isfield(opts,'mlp_model_path') && ~isempty(opts.mlp_model_path)
            cfg.MLP_MODEL_PATH = opts.mlp_model_path;
        end
        out = spinn3d_demo_ctcgov_online_mlp(cfg);
    case {'bus-ptp-dp','ptp','topp-dp'}
        if isfield(opts,'PTP_OPTS'), cfg.PTP_OPTS = opts.PTP_OPTS; end
        out = spinn3d_demo_ctcgov_online_profile(cfg);
    case {'mpc-rg-grid','mpc-rg','mpc'}
        if isfield(opts,'MPC_OPTS'), cfg.MPC_OPTS = opts.MPC_OPTS; end
        out = spinn3d_demo_ctcgov_online_mpc_rg(cfg);
    otherwise
        error('Unknown method: %s', method);
end
end

function row = init_row(metricList)
row = struct();
row.trial_id = NaN;
row.method = "";
row.success = false;
row.error = string('');
row.rollout_wall_time_s = NaN;
row.offline_profile_solve_wall_time_s = NaN;
row.payload_mass_kg = NaN;
row.P_grid_budget_W = NaN;
for j=1:4
    row.(sprintf('q0_deg_j%d',j)) = NaN;
    row.(sprintf('qf_deg_j%d',j)) = NaN;
end
for k=1:numel(metricList)
    row.(metricList{k}) = NaN;
end
end

function trials = make_trials(cfg_base, opts)
rng(opts.seed);
nJ = numel(cfg_base.Q0_DEG(:));
q0c = cfg_base.Q0_DEG(:);
qfc = cfg_base.QF_DEG(:);
mg = opts.q_margin_deg;
if isscalar(mg), mg = repmat(mg,nJ,1); else, mg = mg(:); end
mass_range = opts.payload_mass_range;
if isempty(mass_range)
    mass_range = [0.20 0.50];
end
Pbud0 = getfield_def(cfg_base,'P_GRID_BUDGET',getfield_def(cfg_base,'P_TOTAL_MAX',15));
trial_ids = opts.trial_ids(:).';
if isempty(trial_ids)
    trial_ids = 1:opts.N;
end
Ngen = max([opts.N_total, opts.N, trial_ids]);
trials_all = repmat(struct(),Ngen,1);
for i=1:Ngen
    q0 = q0c + (2*rand(nJ,1)-1).*mg;
    qf = qfc + (2*rand(nJ,1)-1).*mg;
    m = mass_range(1) + (mass_range(2)-mass_range(1))*rand();
    cfg = cfg_base;
    cfg.Q0_DEG = q0; cfg.QF_DEG = qf;
    cfg = set_payload_mass(cfg,m);
    Pbud = Pbud0;
    if strcmpi(opts.pbudget_mode_eval,'sample')
        pr = opts.pbudget_range_W;
        Pbud = pr(1) + (pr(2)-pr(1))*rand();
    end
    cfg.P_GRID_BUDGET = Pbud; cfg.P_TOTAL_MAX = Pbud;
    if isfield(cfg,'BUS') && isstruct(cfg.BUS), cfg.BUS.P_grid_budget = Pbud; cfg.BUS.P_total_max = Pbud; end
    if isfield(cfg,'CAPS') && isstruct(cfg.CAPS), cfg.CAPS.P_grid_budget = Pbud; cfg.CAPS.P_total_max = Pbud; end
    trials_all(i).trial_id = i;
    trials_all(i).cfg = cfg;
    trials_all(i).q0_deg = q0;
    trials_all(i).qf_deg = qf;
    trials_all(i).payload_mass_kg = m;
    trials_all(i).P_grid_budget_W = Pbud;
end
trials = trials_all(trial_ids);
end

function cfg = set_payload_mass(cfg,m)
if ~isfield(cfg,'PAYLOAD') || ~isstruct(cfg.PAYLOAD), cfg.PAYLOAD = struct(); end
if ~isfield(cfg.PAYLOAD,'DEMO') || ~isstruct(cfg.PAYLOAD.DEMO), cfg.PAYLOAD.DEMO = struct(); end
p = cfg.PAYLOAD.DEMO;
base_m = getfield_def(p,'mass',m);
I6 = getfield_def(p,'inertia',[0 0 0 0 0 0]);
if base_m > 0, I6 = I6(:).'*m/base_m; end
p.mass = m;
p.inertia = I6;
if ~isfield(p,'com'), p.com=[0 0 0]; end
cfg.PAYLOAD.DEMO = p;
end

function S = summarize_by_method(T, metricList)
methods = unique(T.method,'stable');
rows = {};
for im=1:numel(methods)
    Mname = methods(im);
    idx = (T.method==Mname) & T.success;
    for k=1:numel(metricList)
        fn = metricList{k};
        v = T.(fn)(idx);
        r = struct('method',char(Mname),'metric',fn,'N',sum(idx), ...
            'mean',mean(v,'omitnan'),'std',std(v,'omitnan'),'median',pctile(v,50), ...
            'p05',pctile(v,5),'p95',pctile(v,95));
        rows{end+1,1}=r; %#ok<AGROW>
    end
end
S = struct2table(cell2mat(rows));
end

function PS = paired_against_reference(T, metricList, refMethod)
methods = unique(T.method,'stable');
rows = {};
refMethodS = string(refMethod);
for im=1:numel(methods)
    method = methods(im);
    if method==refMethodS, continue; end
    for k=1:numel(metricList)
        fn = metricList{k};
        [a,b] = paired_vectors(T,refMethod,method,fn);
        d = a-b; % positive: method lower than reference for smaller-is-better metrics
        r = struct('reference',char(refMethodS),'method',char(method),'metric',fn,'N',numel(d), ...
            'ref_minus_method_mean',mean(d,'omitnan'),'ref_minus_method_std',std(d,'omitnan'), ...
            'method_improved_count',sum(d>0),'method_worse_count',sum(d<0));
        rows{end+1,1}=r; %#ok<AGROW>
    end
end
if isempty(rows), PS = table(); else, PS = struct2table(cell2mat(rows)); end
end

function [a,b] = paired_vectors(T,refMethod,method,metric)
ids = unique(T.trial_id);
a = []; b = [];
for i=1:numel(ids)
    ir = T.trial_id==ids(i) & (T.method==string(refMethod)) & T.success;
    im = T.trial_id==ids(i) & (T.method==string(method)) & T.success;
    if any(ir) && any(im)
        a(end+1,1)=T.(metric)(find(ir,1)); %#ok<AGROW>
        b(end+1,1)=T.(metric)(find(im,1)); %#ok<AGROW>
    end
end
end

function [cfg, opts] = parse_inputs(varargin)
if nargin == 0
    PB = spinn3d_params_block(); cfg = spinn3d_params_to_cfg(PB,'demo_alpha'); opts = default_opts(struct(),cfg); return;
elseif nargin == 1
    if isstruct(varargin{1}) && isfield(varargin{1},'Q0_DEG')
        cfg = varargin{1}; opts = default_opts(struct(),cfg); return;
    else
        PB = spinn3d_params_block(); cfg = spinn3d_params_to_cfg(PB,'demo_alpha'); opts = default_opts(varargin{1},cfg); return;
    end
else
    cfg = varargin{1}; opts = default_opts(varargin{2},cfg); return;
end
end

function opts = default_opts(opts,cfg)
def = struct();
def.N = 20;              % use 20 for smoke test; set 100 for paper table
def.N_total = [];         % full deterministic trial population size for chunked/resume runs
def.trial_ids = [];       % subset of global trial ids to run, e.g., 1:10 or 21:40
def.resume = false;       % when true, reuse/append fair_baseline_trials.csv in export_dir
def.resume_force_rerun = false; % when true, rerun requested trials and replace duplicate rows
def.seed = 1;
def.verbose = true;
def.progress_print = true;       % print START/DONE messages for each paired trial
def.progress_label = 'fair';     % label used in progress messages
def.use_parallel = true;
def.num_workers = [];
def.q_margin_deg = 10;
def.payload_mass_range = [0.20 0.50];
def.pbudget_mode_eval = 'fixed';
def.pbudget_range_W = [getfield_def(cfg,'P_GRID_BUDGET',15), getfield_def(cfg,'P_GRID_BUDGET',15)];
def.power_eval_mode = 'lpf';
def.power_eval_tau_s = 0.05;
def.power_tau_s = 0.05;
def.Pbudget_margin_frac = 0.05;
def.reference_method = 'NoGuard';
def.export_dir = fullfile(pwd,'paper_results_fair_baselines');
def.methods = {'NoGuard','Feasible-RG','TRF-PRG','MLP-PRG','Bus-PTP-DP','MPC-RG-grid'};
def.PTP_OPTS = struct('Ns',81,'Na',31,'allow_soft_violation',false);
def.MPC_OPTS = struct('Na',13,'H',5,'beamN',15,'allow_soft_violation',false,'budget_lpf_ratio',0.98,'raw_guard_ratio',1.50);
try
    PB = spinn3d_params_block();
    def.mlp_model_path = fullfile(PB.paths.root,'model_alpha_gov_mlp.mat');
catch
    def.mlp_model_path = fullfile(pwd,'data_alpha','model_alpha_gov_mlp.mat');
end
f=fieldnames(def);
for i=1:numel(f)
    if ~isfield(opts,f{i}) || isempty(opts.(f{i})), opts.(f{i}) = def.(f{i}); end
end
if isempty(opts.trial_ids)
    opts.trial_ids = 1:opts.N;
else
    opts.trial_ids = unique(double(opts.trial_ids(:).'),'stable');
    opts.N = numel(opts.trial_ids);
end
if isempty(opts.N_total)
    opts.N_total = max(opts.trial_ids);
else
    opts.N_total = max(double(opts.N_total), max(opts.trial_ids));
end
try, opts.resume = logical(opts.resume); catch, opts.resume = false; end
try, opts.resume_force_rerun = logical(opts.resume_force_rerun); catch, opts.resume_force_rerun = false; end
end


function done = completed_trial_mask(T, trials, methods)
done = false(numel(trials),1);
if isempty(T) || ~istable(T) || ~ismember('trial_id',T.Properties.VariableNames) || ~ismember('method',T.Properties.VariableNames)
    return;
end
if ~ismember('success',T.Properties.VariableNames)
    T.success = false(height(T),1);
end
mcol = string(T.method);
for ii=1:numel(trials)
    id = trials(ii).trial_id;
    ok = true;
    for jj=1:numel(methods)
        idx = T.trial_id==id & mcol==string(methods{jj}) & logical(T.success);
        if ~any(idx)
            ok = false;
            break;
        end
    end
    done(ii) = ok;
end
end

function T = normalize_existing_table(T)
if isempty(T) || ~istable(T), return; end
if ismember('method',T.Properties.VariableNames)
    T.method = string(T.method);
end
if ismember('error',T.Properties.VariableNames)
    T.error = string(T.error);
end
if ismember('success',T.Properties.VariableNames)
    try, T.success = logical(T.success); catch, end
end
% Rename legacy ambiguous runtime_s to an explicit name.  This is the wall-clock
% time for the full MATLAB closed-loop rollout, not policy-update latency.
if ismember('runtime_s',T.Properties.VariableNames)
    if ~ismember('rollout_wall_time_s',T.Properties.VariableNames)
        T.rollout_wall_time_s = T.runtime_s;
    else
        try
            idx = ~isfinite(T.rollout_wall_time_s) & isfinite(T.runtime_s);
            T.rollout_wall_time_s(idx) = T.runtime_s(idx);
        catch
        end
    end
    T.runtime_s = [];
end
if ismember('offline_solve_time_s',T.Properties.VariableNames)
    if ~ismember('offline_profile_solve_wall_time_s',T.Properties.VariableNames)
        T.offline_profile_solve_wall_time_s = T.offline_solve_time_s;
    else
        try
            idx = ~isfinite(T.offline_profile_solve_wall_time_s) & isfinite(T.offline_solve_time_s);
            T.offline_profile_solve_wall_time_s(idx) = T.offline_solve_time_s(idx);
        catch
        end
    end
    T.offline_solve_time_s = [];
end
end

function T = merge_trial_tables(Told, Tnew)
if isempty(Told) && isempty(Tnew), T = table(); return; end
if isempty(Told), T = normalize_existing_table(Tnew); return; end
if isempty(Tnew), T = normalize_existing_table(Told); return; end
Told = normalize_existing_table(Told);
Tnew = normalize_existing_table(Tnew);
[Told,Tnew] = align_table_columns(Told,Tnew);
Tall = [Told; Tnew];
Tall = normalize_existing_table(Tall);
key = string(Tall.trial_id) + "|" + string(Tall.method);
[~,ia] = unique(key,'last');
T = Tall(sort(ia),:);
try, T = sortrows(T,{'trial_id','method'}); catch, end
end

function [A,B] = align_table_columns(A,B)
vars = unique([A.Properties.VariableNames, B.Properties.VariableNames],'stable');
for i=1:numel(vars)
    v = vars{i};
    if ~ismember(v,A.Properties.VariableNames)
        A.(v) = default_column_like(B.(v),height(A));
    end
    if ~ismember(v,B.Properties.VariableNames)
        B.(v) = default_column_like(A.(v),height(B));
    end
end
A = A(:,vars);
B = B(:,vars);
% Make common fragile text/logical columns compatible.
if ismember('method',vars), A.method = string(A.method); B.method = string(B.method); end
if ismember('error',vars), A.error = string(A.error); B.error = string(B.error); end
if ismember('success',vars), A.success = logical(A.success); B.success = logical(B.success); end
end

function c = default_column_like(ref,n)
if isstring(ref)
    c = strings(n,1);
elseif iscellstr(ref)
    c = repmat({''},n,1);
elseif islogical(ref)
    c = false(n,1);
elseif isnumeric(ref)
    c = NaN(n,1);
else
    c = strings(n,1);
end
end

function s = compact_id_list(ids)
ids = ids(:).';
if isempty(ids), s = '(none)'; return; end
ids = sort(unique(ids));
ranges = {};
a = ids(1); b = ids(1);
for k=2:numel(ids)
    if ids(k)==b+1
        b = ids(k);
    else
        ranges{end+1} = range_to_string(a,b); %#ok<AGROW>
        a = ids(k); b = ids(k);
    end
end
ranges{end+1} = range_to_string(a,b);
s = strjoin(ranges, ',');
end

function s = range_to_string(a,b)
if a==b, s = sprintf('%d',a); else, s = sprintf('%d:%d',a,b); end
end

function metricList = default_metric_list()
metricList = {'t_reach','P_peak','P_p99','P_peak_norm','P_p99_norm','pbar_peak','pbar_p99', ...
    'P_peak_raw','P_peak_fast','P_peak_slow','P_p99_fast','P_p99_slow', ...
    'P_peak_lpf_norm','P_p99_lpf_norm','pbar_lpf_peak','pbar_lpf_p99', ...
    'E_excess','E_excess_norm_s','E_grid','E_reuse','E_dump','E_mech_pos','E_mech_regen', ...
    'I_rms','I_peak','Vdc_min','P_loss_energy','E_spill', ...
    'P_request_peak','P_request_p99','Psource_shortfall_peak','E_source_shortfall','source_limited_frac', ...
    'P_std','P_std_norm','P_rms_to_Pbudget', ...
    'ramp_rms','ramp_p95','ramp_max','ramp_p95_norm','ramp_max_norm','pbar_ramp_p95','pbar_ramp_max', ...
    'ramp_p95_lpf_norm','ramp_max_lpf_norm','r_viol'};
end

function p = pctile(x,q)
x=x(isfinite(x)); x=sort(x(:)); if isempty(x), p=NaN; return; end
q=max(0,min(100,q)); pos=1+(numel(x)-1)*q/100; lo=floor(pos); hi=ceil(pos);
if lo==hi, p=x(lo); else, p=x(lo)+(pos-lo)*(x(hi)-x(lo)); end
end
function x = getfield_def(S,fn,def)
if isstruct(S) && isfield(S,fn) && ~isempty(S.(fn)), x=S.(fn); else, x=def; end
end
function ensure_dir(d), if exist(d,'dir')~=7, mkdir(d); end, end
