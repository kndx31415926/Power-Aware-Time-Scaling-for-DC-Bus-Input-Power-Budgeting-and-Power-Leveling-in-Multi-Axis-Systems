function R = paper_latency_benchmark(cfg, opts)
%PAPER_LATENCY_BENCHMARK Measure high-level policy timing separately from 1 ms servo simulation.
%
% This script is intended for the revision/rebuttal. It explicitly reports that
% cfg.TS is the plant/CTC simulation step, whereas cfg.POLICY.dt is the NN / high
% level governor update period. It measures feature construction, Transformer
% forward inference, governor post-processing, and total policy update latency.
%
% Usage:
%   R = paper_latency_benchmark();
%   R = paper_latency_benchmark(cfg, struct('N',2000,'warmup',100));
%
% Output:
%   R.table, R.platform, R.policy_dt, R.servo_Ts, R.model_bytes, R.n_params

if nargin < 1 || isempty(cfg)
    PB = spinn3d_params_block();
    cfg = spinn3d_params_to_cfg(PB,'demo_alpha');
end
if nargin < 2 || isempty(opts), opts = struct(); end
N = get_opt_def(opts,'N',1000);
warmup = get_opt_def(opts,'warmup',50);
N = max(10, round(N)); warmup = max(0, round(warmup));

assert(isfield(cfg,'MODEL_PATH') && exist(cfg.MODEL_PATH,'file')==2, 'MODEL_PATH not found. Train/load the TRF model first.');
M = load_model_alpha_strict(cfg.MODEL_PATH);

robot = cfg.ROBOT; try, robot.DataFormat='row'; end
limits = cfg.CAPS;
limits.P_grid_budget = getfield_def(cfg,'P_GRID_BUDGET', getfield_def(cfg,'P_TOTAL_MAX', inf));
limits.P_total_max = limits.P_grid_budget;

q0 = deg2rad_local(row(cfg.Q0_DEG));
qf = unwrap_to_near(q0, deg2rad_local(row(cfg.QF_DEG)));

pay_cfg = [];
if isfield(cfg,'PAYLOAD') && isstruct(cfg.PAYLOAD) && isfield(cfg.PAYLOAD,'DEMO')
    pay_cfg = cfg.PAYLOAD.DEMO;
end
[robot, pay_v10] = inject_payload_last_body_local(robot, pay_cfg);
payload = v10_to_struct(pay_v10);
traj = make_traj_cart_bezier_dls(robot, q0, qf, limits, cfg);

policy = getfield_def(cfg,'POLICY',struct('dt',cfg.TS,'hold','zoh'));
policy.dt = max(getfield_def(policy,'dt',cfg.TS), cfg.TS);
policy.update_every_n = max(1, round(policy.dt/cfg.TS));
policy.dt_effective = policy.update_every_n * cfg.TS;

areg = cfg.ALPHA_REG;
Pbudget = limits.P_grid_budget;
featureFcn = @spinn3d_features_alpha_plus_payload_gov;

Klook = 1;
if isfield(M.preproc,'K') && ~isempty(M.preproc.K), Klook = round(double(M.preproc.K)); end
if ~isfinite(Klook) || Klook < 1, Klook = 1; end
kb = logical(M.preproc.keepBaseIdx(:).');

% Use the same look-ahead spacing saved at training time.
ds = getfield_def(M.preproc,'ds',cfg.TS);
if ~isfinite(ds) || ds <= 0, ds = cfg.TS; end

t_feat = zeros(N,1); t_inf = zeros(N,1); t_gov = zeros(N,1); t_total = zeros(N,1);
a_prev = cfg.ALPHA_MIN;

% Warmup
for i = 1:warmup
    s = mod((i-1)*policy.dt_effective, max(traj.Tf,1e-9));
    [~, ~, ~, a_prev] = one_policy_step(s, a_prev, false);
end

for i = 1:N
    s = mod((i-1)*policy.dt_effective, max(traj.Tf,1e-9));
    t0 = tic;
    [tf, ti, tg, a_prev] = one_policy_step(s, a_prev, true);
    t_total(i) = toc(t0);
    t_feat(i) = tf; t_inf(i) = ti; t_gov(i) = tg;
end

components = {'feature_construction'; 'trf_forward'; 'governor_update'; 'total_policy_update'};
X = [t_feat, t_inf, t_gov, t_total];
mean_us = 1e6*mean(X,1).';
p95_us  = 1e6*pctile_local(X,95).';
p99_us  = 1e6*pctile_local(X,99).';
max_us  = 1e6*max(X,[],1).';
T = table(components, mean_us, p95_us, p99_us, max_us);

R = struct();
R.table = T;
R.servo_Ts = cfg.TS;
R.policy_dt = policy.dt_effective;
R.policy_update_every_n = policy.update_every_n;
R.platform = local_platform_info();
Smodel = whos('M');
R.model_bytes = Smodel.bytes;
R.n_params = count_learnables(M.net);
R.note = 'Ts is the servo/digital-twin step. TRF inference is measured at cfg.POLICY.dt, not at every 1 ms servo tick.';

if get_opt_def(opts,'write_csv',true)
    outdir = get_opt_def(opts,'export_dir',fullfile(pwd,'latency_results'));
    if exist(outdir,'dir')~=7, mkdir(outdir); end
    writetable(T, fullfile(outdir,'latency_policy_update.csv'));
    save(fullfile(outdir,'latency_policy_update.mat'),'R');
end

disp(T);
fprintf('[latency] servo Ts = %.6g s, policy dt = %.6g s, update_every_n = %d\n', cfg.TS, policy.dt_effective, policy.update_every_n);

    function [tf, ti, tg, a_post] = one_policy_step(s, a_prev_in, do_timing)
        if do_timing, tt = tic; end
        [q_r,dq_r,ddq_r] = traj.eval(s); q_r=row(q_r); dq_r=row(dq_r); ddq_r=row(ddq_r);
        s_norm0 = min(1.0, s/max(traj.Tf,1e-9));
        x1 = featureFcn(q_r,dq_r,ddq_r, robot, payload, Pbudget, limits, ...
            a_prev_in, areg.BETA, areg.A_DOT_UP, areg.A_DOT_DN, cfg.TS, s_norm0);
        x1 = row(double(x1));
        Dtot = numel(x1); Dtail=6; Dbase=Dtot-Dtail;
        Xfull = zeros(Klook,Dtot); Xfull(1,:) = x1;
        for kk=2:Klook
            s_i = min(traj.Tf, s + (kk-1)*ds);
            [qi,dqgi,ddqgi] = traj.eval(s_i);
            s_norm_i = min(1.0, s_i/max(traj.Tf,1e-9));
            Xfull(kk,:) = row(double(featureFcn(row(qi),row(dqgi),row(ddqgi), robot, payload, Pbudget, limits, ...
                a_prev_in, areg.BETA, areg.A_DOT_UP, areg.A_DOT_DN, cfg.TS, s_norm_i)));
        end
        Xbase = Xfull(:,1:Dbase); Xtail = Xfull(:,Dbase+1:end);
        Xc = [Xbase(:,kb), Xtail];
        mu=row(double(M.preproc.mu)); sig=row(double(M.preproc.sig));
        Z = (Xc - mu)./max(sig,1e-12);
        if do_timing, tf = toc(tt); else, tf=0; end

        if do_timing, tt = tic; end
        Din = size(Z,2);
        if Klook > 1
            dlX = dlarray(single(reshape(Z.', [Din,1,Klook])), 'CBT');
        else
            dlX = dlarray(single(Z(1,:).'), 'CB');
        end
        a_des = double(extractdata(forward(M.net, dlX)));
        a_des = min(1.0, max(cfg.ALPHA_MIN, a_des));
        if do_timing, ti = toc(tt); else, ti=0; end

        if do_timing, tt = tic; end
        a_post = governor_step(a_des, a_prev_in, areg.BETA, areg.A_DOT_UP, areg.A_DOT_DN, cfg.TS);
        if do_timing, tg = toc(tt); else, tg=0; end
    end
end

% -------------------------------------------------------------------------
function S = local_platform_info()
S = struct();
try, S.version = version; catch, S.version = ''; end
try, S.computer = computer; catch, S.computer = ''; end
try, S.maxNumCompThreads = maxNumCompThreads; catch, S.maxNumCompThreads = NaN; end
end
function n = count_learnables(net)
n=0;
try
    L = net.Learnables;
    for i=1:size(L,1), n = n + numel(L.Value{i}); end
catch
    n = NaN;
end
end
function q = deg2rad_local(x), q=(pi/180).*x; end
function y = row(x), y=x(:).'; end
function v = get_opt_def(S,fn,def), if isstruct(S)&&isfield(S,fn)&&~isempty(S.(fn)), v=S.(fn); else, v=def; end, end
function v = getfield_def(S,fn,def), if isstruct(S)&&isfield(S,fn)&&~isempty(S.(fn)), v=S.(fn); else, v=def; end, end
function p = pctile_local(X,q)
p=zeros(1,size(X,2));
for j=1:size(X,2)
    x=sort(X(isfinite(X(:,j)),j));
    if isempty(x), p(j)=NaN; else, p(j)=x(max(1,min(numel(x),ceil(q/100*numel(x))))); end
end
end
function qf = unwrap_to_near(q0, qf), d=qf-q0; qf=q0+atan2(sin(d),cos(d)); end
function S = v10_to_struct(v)
v=row(v); v=[v,zeros(1,10-numel(v))];
S=struct('mass',v(1),'com',v(2:4),'inertia',v(5:10));
end
function [robot_out, pay_vec] = inject_payload_last_body_local(robot_in, pay)
% Minimal local payload injector for benchmark reproducibility.
robot_out = robot_in; pay_vec=zeros(1,10);
if isempty(pay), return; end
if isstruct(pay)
    m=getfield_def(pay,'mass',0); com=getfield_def(pay,'com',[0 0 0]); I6=getfield_def(pay,'inertia',[0 0 0 0 0 0]);
else
    v=row(pay); v=[v,zeros(1,10-numel(v))]; m=v(1); com=v(2:4); I6=v(5:10);
end
pay_vec=[m,row(com),row(I6)];
try
    b=robot_out.Bodies{end};
    b.Mass=m; b.CenterOfMass=row(com); b.Inertia=row(I6);
catch
end
end
