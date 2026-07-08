function R = paper_policy_latency_trace_onecase(cfg, opts)
%PAPER_POLICY_LATENCY_TRACE_ONECASE
% One-case, per-policy-update latency trace for TRF/MLP governor commands.
%
% This benchmark measures only the policy-update work needed by learned
% governors: look-ahead feature construction, neural forward inference, and
% deterministic governor post-processing. It deliberately does NOT use the
% end-to-end rollout_wall_time_s from RUN_02, because rollout_wall_time_s includes plant
% simulation, logging, table I/O, and MATLAB overhead.
%
% Output files:
%   latency_policy_steps.csv      one row per policy update
%   latency_policy_summary.csv    mean/p95/p99/max per method/component
%   latency_policy_trace.mat      raw MATLAB struct
%
% Usage:
%   PB = spinn3d_params_block();
%   cfg = spinn3d_params_to_cfg(PB,'demo_alpha');
%   cfg.MODEL_PATH = PB.paths.model_alpha;
%   cfg.MLP_MODEL_PATH = PB.paths.model_alpha_mlp;
%   R = paper_policy_latency_trace_onecase(cfg, struct('Nsteps',200));

if nargin < 1 || isempty(cfg)
    PB = spinn3d_params_block();
    cfg = spinn3d_params_to_cfg(PB,'demo_alpha');
end
if nargin < 2 || isempty(opts), opts = struct(); end

opts = defaults_local(opts, cfg);
if exist(opts.export_dir,'dir') ~= 7, mkdir(opts.export_dir); end

% Keep this benchmark figure-free and deterministic.
cfg.NO_FIGURE = true;
cfg.DISABLE_FIGURES = true;

assert(isfield(cfg,'MODEL_PATH') && exist(char(cfg.MODEL_PATH),'file')==2, ...
    'TRF model not found. Run RUN_01 first or set cfg.MODEL_PATH.');
assert(isfield(cfg,'MLP_MODEL_PATH') && exist(char(cfg.MLP_MODEL_PATH),'file')==2, ...
    'MLP model not found. Run RUN_01 first or set cfg.MLP_MODEL_PATH.');

models = struct();
models(1).method = "TRF-PRG";
models(1).path   = string(cfg.MODEL_PATH);
models(1).M      = normalize_model_local(load_model_alpha_any(char(cfg.MODEL_PATH)));
models(2).method = "MLP-PRG";
models(2).path   = string(cfg.MLP_MODEL_PATH);
models(2).M      = normalize_model_local(load_model_alpha_any(char(cfg.MLP_MODEL_PATH)));

robot = cfg.ROBOT;
try, robot.DataFormat = 'row'; end
limits = cfg.CAPS;
Pbud = getfield_def_local(cfg,'P_GRID_BUDGET',getfield_def_local(cfg,'P_TOTAL_MAX',NaN));
limits.P_grid_budget = Pbud;
limits.P_total_max = Pbud;

q0 = deg2rad_local(row_local(cfg.Q0_DEG));
qf = unwrap_to_near_local(q0, deg2rad_local(row_local(cfg.QF_DEG)));

pay_cfg = [];
if isfield(cfg,'PAYLOAD') && isstruct(cfg.PAYLOAD) && isfield(cfg.PAYLOAD,'DEMO')
    pay_cfg = cfg.PAYLOAD.DEMO;
end
[robot, pay_v10] = inject_payload_last_body_local(robot, pay_cfg);
payload = v10_to_struct_local(pay_v10);
traj = make_traj_cart_bezier_dls(robot, q0, qf, limits, cfg);

areg = cfg.ALPHA_REG;
alpha_min = getfield_def_local(cfg,'ALPHA_MIN',0.08);
Ts = getfield_def_local(cfg,'TS',getfield_def_local(cfg,'TS_SERVO',1e-3));
policy_dt = getfield_def_local(cfg,'POLICY_DT',getfield_def_local(getfield_def_local(cfg,'POLICY',struct()),'dt',0.01));
policyN = max(1,round(policy_dt/Ts));
policy_dt = policyN*Ts;

% Warm up networks so the timed trace is not dominated by first-call setup.
for im = 1:numel(models)
    M = models(im).M;
    a_prev = alpha_min;
    for k = 1:max(0,opts.warmup)
        s = min(traj.Tf, (k-1)*policy_dt);
        [q_r,dq_r,ddq_r] = traj.eval(s);
        [Z, meta] = build_policy_input_local(M, cfg, robot, traj, payload, Pbud, limits, areg, row_local(q_r), row_local(dq_r), row_local(ddq_r), s, a_prev); %#ok<ASGLU>
        a_des = forward_model_local(M,Z,alpha_min);
        a_prev = governor_step(a_des, a_prev, areg.BETA, areg.A_DOT_UP, areg.A_DOT_DN, Ts);
    end
end

rows = {};
platform = platform_info_local();

for im = 1:numel(models)
    method = models(im).method;
    M = models(im).M;
    a_prev = alpha_min;
    s = 0.0;
    for k = 1:opts.Nsteps
        [q_r,dq_r,ddq_r] = traj.eval(s);
        q_r=row_local(q_r); dq_r=row_local(dq_r); ddq_r=row_local(ddq_r);

        t_total = tic;
        t0 = tic;
        [Z, meta] = build_policy_input_local(M, cfg, robot, traj, payload, Pbud, limits, areg, q_r, dq_r, ddq_r, s, a_prev);
        feature_s = toc(t0);

        t0 = tic;
        a_des = forward_model_local(M,Z,alpha_min);
        inference_s = toc(t0);

        t0 = tic;
        a_post = governor_step(a_des, a_prev, areg.BETA, areg.A_DOT_UP, areg.A_DOT_DN, Ts);
        governor_s = toc(t0);
        total_s = toc(t_total);

        r = struct();
        r.method = method;
        r.policy_update = k;
        r.t_policy_s = (k-1)*policy_dt;
        r.s_path = s;
        r.s_norm = min(1,s/max(traj.Tf,eps));
        r.a_prev = a_prev;
        r.a_des = a_des;
        r.a_post = a_post;
        r.Klook = meta.Klook;
        r.input_dim = meta.input_dim;
        r.feature_us = 1e6*feature_s;
        r.neural_inference_us = 1e6*inference_s;
        r.governor_us = 1e6*governor_s;
        r.total_policy_update_us = 1e6*total_s;
        rows{end+1,1} = r; %#ok<AGROW>

        a_prev = a_post;
        s = min(traj.Tf, s + policy_dt*a_post);
        if s >= traj.Tf - 1e-12
            % Keep the requested number of rows by wrapping over the same path.
            s = 0.0;
        end
    end
end

T = struct2table(vertcat(rows{:}));
S = summarize_latency_local(T);

% Model sizes / parameter counts.
modelRows = repmat(struct('method',"",'model_path',"",'model_bytes',NaN,'n_params',NaN,'kind',""), numel(models),1);
for im = 1:numel(models)
    M = models(im).M; %#ok<NASGU>
    W = whos('M');
    modelRows(im).method = models(im).method;
    modelRows(im).model_path = models(im).path;
    modelRows(im).model_bytes = W.bytes;
    modelRows(im).n_params = count_params_local(models(im).M.net);
    modelRows(im).kind = string(getfield_def_local(models(im).M,'type',getfield_def_local(models(im).M.preproc,'kind','unknown')));
end
ModelInfo = struct2table(modelRows);

writetable(T, fullfile(opts.export_dir,'latency_policy_steps.csv'));
writetable(S, fullfile(opts.export_dir,'latency_policy_summary.csv'));
writetable(ModelInfo, fullfile(opts.export_dir,'latency_model_info.csv'));

R = struct();
R.steps = T;
R.summary = S;
R.model_info = ModelInfo;
R.platform = platform;
R.servo_Ts = Ts;
R.policy_dt = policy_dt;
R.policy_update_every_n = policyN;
R.note = "Per-policy-update trace. It excludes closed-loop simulation rollout_wall_time_s and table/logging overhead.";
R.export_dir = opts.export_dir;
save(fullfile(opts.export_dir,'latency_policy_trace.mat'),'R','-v7.3');

disp(S);
fprintf('[latency-onecase] wrote %d policy-update rows to %s\n', height(T), opts.export_dir);
fprintf('[latency-onecase] servo Ts = %.6g s, policy dt = %.6g s (%d servo ticks)\n', Ts, policy_dt, policyN);
end

% -------------------------------------------------------------------------
function opts = defaults_local(opts,cfg)
def = struct();
def.Nsteps = 200;
def.warmup = 20;
def.export_dir = fullfile(pwd,'paper_outputs_chunked_100','RUN03_revision_exports','latency');
f = fieldnames(def);
for i=1:numel(f)
    if ~isfield(opts,f{i}) || isempty(opts.(f{i})), opts.(f{i}) = def.(f{i}); end
end
opts.Nsteps = max(10,round(double(opts.Nsteps)));
opts.warmup = max(0,round(double(opts.warmup)));
if isfield(cfg,'POLICY_DT') && isfinite(cfg.POLICY_DT) && cfg.POLICY_DT <= 0
    cfg.POLICY_DT = 0.01; %#ok<NASGU>
end
end

function M = normalize_model_local(M)
if ~isfield(M,'type') || isempty(M.type)
    M.type = getfield_def_local(M.preproc,'kind','trf_lookahead');
end
M.preproc.mu = row_local(double(M.preproc.mu));
M.preproc.sig = row_local(double(M.preproc.sig));
if isfield(M.preproc,'keepBaseIdx') && ~isempty(M.preproc.keepBaseIdx)
    M.preproc.keepBaseIdx = logical(M.preproc.keepBaseIdx(:)).';
end
end

function [Z, meta] = build_policy_input_local(M, cfg, robot, traj, payload, Pbud, limits, areg, q_ref, dq_ref, ddq_ref, s, a_prev)
featureFcn = @spinn3d_features_alpha_plus_payload_gov;
Ts = getfield_def_local(cfg,'TS',getfield_def_local(cfg,'TS_SERVO',1e-3));
kb = logical(M.preproc.keepBaseIdx(:).');
Klook = max(1, round(double(getfield_def_local(M.preproc,'K',1))));
ds = getfield_def_local(M.preproc,'ds',Ts);
if ~isfinite(ds) || ds <= 0
    ds = max(traj.Tf,1e-9)/max(Klook-1,1);
end

s_norm0 = min(1.0, s/max(traj.Tf,1e-9));
x1 = featureFcn(q_ref,dq_ref,ddq_ref, robot, payload, Pbud, limits, ...
    a_prev, areg.BETA, areg.A_DOT_UP, areg.A_DOT_DN, Ts, s_norm0);
x1 = row_local(double(x1));
Dtot = numel(x1); Dtail = 6; Dbase = Dtot-Dtail;
assert(numel(kb)==Dbase, 'keepBaseIdx length mismatch for model kind=%s', char(getfield_def_local(M,'type','unknown')));
Xfull = zeros(Klook,Dtot);
Xfull(1,:) = x1;
for kk = 2:Klook
    s_i = min(traj.Tf, s + (kk-1)*ds);
    [qi,dqgi,ddqgi] = traj.eval(s_i);
    s_norm_i = min(1.0, s_i/max(traj.Tf,1e-9));
    Xfull(kk,:) = row_local(double(featureFcn(row_local(qi),row_local(dqgi),row_local(ddqgi), robot, payload, Pbud, limits, ...
        a_prev, areg.BETA, areg.A_DOT_UP, areg.A_DOT_DN, Ts, s_norm_i)));
end
Xbase = Xfull(:,1:Dbase);
Xtail = Xfull(:,Dbase+1:end);
Xc = [Xbase(:,kb), Xtail];

is_mlp = is_mlp_model_local(M);
if is_mlp
    Xflat = reshape(Xc.',1,[]);
    mu = row_local(double(M.preproc.mu));
    sig = row_local(double(M.preproc.sig));
    assert(numel(mu)==numel(Xflat), 'MLP normalization dimension mismatch.');
    Z = (Xflat - mu)./max(sig,1e-12);
else
    mu = row_local(double(M.preproc.mu));
    sig = row_local(double(M.preproc.sig));
    assert(numel(mu)==size(Xc,2), 'TRF normalization dimension mismatch.');
    Z = (Xc - mu)./max(sig,1e-12);
end
meta = struct('Klook',Klook,'input_dim',numel(Z),'is_mlp',is_mlp);
end

function y = forward_model_local(M,Z,alpha_min)
is_mlp = is_mlp_model_local(M);
if isa(M.net,'dlnetwork')
    if is_mlp
        dlX = dlarray(single(Z.'),'CB');
    else
        Din = size(Z,2); Klook = size(Z,1);
        if Klook > 1
            dlX = dlarray(single(reshape(Z.',[Din,1,Klook])),'CBT');
        else
            dlX = dlarray(single(Z(1,:).'),'CB');
        end
    end
    y = double(extractdata(forward(M.net,dlX)));
else
    try
        y = double(predict(M.net,single(Z)));
    catch
        y = double(predict(M.net,Z));
    end
end
if isempty(y) || ~isfinite(y(1)), y = alpha_min; else, y = y(1); end
if is_mlp && getfield_def_local(M.preproc,'target_normalized',false)
    amin_m = double(getfield_def_local(M.preproc,'alpha_min',alpha_min));
    y = amin_m + (1-amin_m)*y;
end
y = min(1.0,max(alpha_min,double(y)));
end

function tf = is_mlp_model_local(M)
kind = lower(char(getfield_def_local(M,'type',getfield_def_local(M.preproc,'kind',''))));
tf = ~isempty(strfind(kind,'mlp')) || logical(getfield_def_local(M.preproc,'flatten',false)); %#ok<STREMP>
end

function S = summarize_latency_local(T)
methods = unique(T.method,'stable');
components = {'feature_us','neural_inference_us','governor_us','total_policy_update_us'};
labels = {'Feature construction','Neural inference','Governor update','Total learned-policy update'};
rows = {};
for im = 1:numel(methods)
    idx = T.method == methods(im);
    for ic = 1:numel(components)
        v = T.(components{ic})(idx);
        r = struct();
        r.method = methods(im);
        r.component = string(labels{ic});
        r.N_updates = sum(isfinite(v));
        r.mean_us = mean(v,'omitnan');
        r.p95_us = pctile_local(v,95);
        r.p99_us = pctile_local(v,99);
        r.max_us = max(v,[],'omitnan');
        rows{end+1,1} = r; %#ok<AGROW>
    end
end
S = struct2table(vertcat(rows{:}));
end

function p = pctile_local(x,q)
x = sort(x(isfinite(x(:))));
if isempty(x), p = NaN; return; end
q = max(0,min(100,q));
pos = 1 + (numel(x)-1)*q/100;
lo = floor(pos); hi = ceil(pos);
if lo == hi, p = x(lo); else, p = x(lo) + (pos-lo)*(x(hi)-x(lo)); end
end

function n = count_params_local(net)
n = NaN;
try
    n = 0;
    L = net.Learnables;
    for i = 1:size(L,1)
        n = n + numel(L.Value{i});
    end
catch
    try
        n = 0;
        layers = net.Layers;
        for i=1:numel(layers)
            if isprop(layers(i),'Weights') && ~isempty(layers(i).Weights), n = n + numel(layers(i).Weights); end
            if isprop(layers(i),'Bias') && ~isempty(layers(i).Bias), n = n + numel(layers(i).Bias); end
        end
    catch
        n = NaN;
    end
end
end

function S = platform_info_local()
S = struct();
try, S.matlab_version = string(version); catch, S.matlab_version = ""; end
try, S.computer = string(computer); catch, S.computer = ""; end
try, S.maxNumCompThreads = maxNumCompThreads; catch, S.maxNumCompThreads = NaN; end
try
    if gpuDeviceCount > 0
        g = gpuDevice;
        S.gpu = string(g.Name);
    else
        S.gpu = "none";
    end
catch
    S.gpu = "unknown";
end
end

function [robot_out, pay_vec] = inject_payload_last_body_local(robot_in, pay)
robot_out = robot_in; pay_vec = zeros(1,10);
if isempty(pay), return; end
if isstruct(pay)
    m = getfield_def_local(pay,'mass',0);
    com = row_local(getfield_def_local(pay,'com',[0 0 0]));
    I6 = row_local(getfield_def_local(pay,'inertia',[0 0 0 0 0 0]));
else
    v = row_local(pay); v = [v,zeros(1,10-numel(v))];
    m = v(1); com = v(2:4); I6 = v(5:10);
end
pay_vec = [m,row_local(com),row_local(I6)];
try
    b = robot_out.Bodies{end};
    b.Mass = m;
    b.CenterOfMass = row_local(com);
    b.Inertia = row_local(I6);
catch
end
end

function S = v10_to_struct_local(v)
v = row_local(v); v = [v,zeros(1,10-numel(v))];
S = struct('mass',v(1),'com',v(2:4),'inertia',v(5:10));
end

function qf = unwrap_to_near_local(q0,qf)
d = qf-q0;
qf = q0 + atan2(sin(d),cos(d));
end
function x = deg2rad_local(x), x = (pi/180).*x; end
function r = row_local(x), r = x(:).'; end
function v = getfield_def_local(S,fn,def)
if isstruct(S) && isfield(S,fn) && ~isempty(S.(fn)), v = S.(fn); else, v = def; end
end
