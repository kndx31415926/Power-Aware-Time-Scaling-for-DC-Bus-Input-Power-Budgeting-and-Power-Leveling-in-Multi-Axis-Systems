function R = paper_robustness_eval(cfg, opts)
%PAPER_ROBUSTNESS_EVAL  Parallel practical robustness stress test.
%
% Perturbs friction, payload CoM, sensor noise, bus voltage, and active
% power-budget variation.  Trial configurations are sampled serially for
% reproducibility, then rolled out in parfor when a pool is available.

if nargin < 1 || isempty(cfg)
    PB = spinn3d_params_block();
    cfg = spinn3d_params_to_cfg(PB,'demo_alpha');
end
if nargin < 2 || isempty(opts), opts = struct(); end
N = getfield_def(opts,'N',50);
seed = getfield_def(opts,'seed',2);
closeFigs = getfield_def(opts,'close_figs',true);
outdir = getfield_def(opts,'export_dir',fullfile(pwd,'robustness_eval'));
use_parallel = logical(getfield_def(opts,'use_parallel',false));
if ~exist(outdir,'dir'), mkdir(outdir); end

if use_parallel && exist('paper_open_parallel_pool','file')==2
    paper_open_parallel_pool(opts);
end

items = make_robustness_items(cfg,N,seed);
rows = cell(N,1);
tAll = tic;
if use_parallel && ~isempty(gcp('nocreate'))
    parfor k=1:N
        rows{k} = run_robustness_item(items(k), closeFigs);
    end
else
    for k=1:N
        rows{k} = run_robustness_item(items(k), closeFigs);
        fprintf('[robust] %d/%d ok=%d\n', k, N, rows{k}.ok);
    end
end
T = struct2table_robust(rows);
R = struct('table',T,'cfg',cfg,'opts',opts,'export_dir',outdir,'script_wall_time_s',toc(tAll));
try, writetable(T, fullfile(outdir,'robustness_trials.csv')); catch, end
try, save(fullfile(outdir,'robustness_eval.mat'),'R','items'); catch, end
fprintf('[robust] done: N=%d, script_wall_time=%.1f s, out=%s\n', N, R.script_wall_time_s, outdir);
end

function items = make_robustness_items(cfg,N,seed)
rng(seed);
items = repmat(struct('trial',NaN,'cfg',[],'Pbud',NaN,'V_nom',NaN,'fric_scale',NaN),N,1);
for k=1:N
    ci = cfg;
    ci.NO_FIGURE = true; ci.DISABLE_FIGURES = true;
    fs = unif(0.70,1.30);
    if isfield(ci,'FRIC')
        ci.FRIC.B  = ci.FRIC.B  * fs;
        ci.FRIC.Fc = ci.FRIC.Fc * unif(0.70,1.30);
    end
    ci.NOISE.sigma_q  = unif(1e-4, 1e-3);
    ci.NOISE.sigma_dq = unif(1e-4, 1e-2);

    if isfield(ci,'PAYLOAD') && isfield(ci.PAYLOAD,'DEMO')
        p = ci.PAYLOAD.DEMO;
        p.mass = getfield_def(p,'mass',0.4) * unif(0.80,1.20);
        p.com  = row(getfield_def(p,'com',[0 0 0.05])) + 0.015*randn(1,3);
        ci.PAYLOAD.DEMO = p;
    end

    if ~isfield(ci,'BUS') || ~isstruct(ci.BUS), ci.BUS = struct(); end
    ci.BUS.V_nom = unif(10.5,12.6);
    ci.BUS.V_oc  = ci.BUS.V_nom;
    Pbud = getfield_def(ci,'P_GRID_BUDGET',getfield_def(ci,'P_TOTAL_MAX',15)) * unif(0.80,1.20);
    ci.P_GRID_BUDGET = Pbud;
    ci.P_TOTAL_MAX   = Pbud;
    ci.BUS.P_grid_budget = Pbud;
    ci.BUS.P_total_max   = Pbud;
    if isfield(ci,'CAPS') && isstruct(ci.CAPS)
        ci.CAPS.P_grid_budget = Pbud;
        ci.CAPS.P_total_max = Pbud;
    end
    items(k).trial = k;
    items(k).cfg = ci;
    items(k).Pbud = Pbud;
    items(k).V_nom = ci.BUS.V_nom;
    items(k).fric_scale = fs;
end
end

function r = run_robustness_item(item, closeFigs)
ci = item.cfg;
try
    t0=tic;
    outT = run_demo_closefig(@spinn3d_demo_ctcgov_online_nn, ci, closeFigs);
    rt_trf = toc(t0);
    t0=tic;
    outB = run_demo_closefig(@spinn3d_demo_ctcgov_online_nonn_nolimit, ci, closeFigs);
    rt_base = toc(t0);
    mt = simple_metrics(outT, item.Pbud);
    mb = simple_metrics(outB, item.Pbud);
    r = struct('trial',item.trial,'ok',true,'message',string(''), ...
        'P_grid_budget',item.Pbud,'V_nom',item.V_nom,'fric_scale',item.fric_scale, ...
        'runtime_trf_s',rt_trf,'runtime_base_s',rt_base);
    f = fieldnames(mt); for i=1:numel(f), r.(['trf_' f{i}]) = mt.(f{i}); end
    f = fieldnames(mb); for i=1:numel(f), r.(['base_' f{i}]) = mb.(f{i}); end
catch ME
    r = struct('trial',item.trial,'ok',false,'message',string(ME.message), ...
        'P_grid_budget',item.Pbud,'V_nom',item.V_nom,'fric_scale',item.fric_scale);
end
end

function out = run_demo_closefig(fh,cfg,doClose)
fig_before = findall(0,'Type','figure');
out = fh(cfg);
fig_after = findall(0,'Type','figure');
if doClose
    new_figs = fig_after(~ismember(fig_after,fig_before));
    try, close(new_figs); catch, end
end
end
function M = simple_metrics(out,Pbud)
t = out.log.t(:); P = out.log.Pgrid(:);
if numel(t)<2, dt=1e-3; else, dt=median(diff(t)); end
if ~isfinite(dt) || dt<=0, dt=1e-3; end
tau_eval = 0.05;
if isfield(out,'cfg') && isfield(out.cfg,'EVAL') && isstruct(out.cfg.EVAL)
    tau_eval = getfield_def(out.cfg.EVAL,'power_tau_s', getfield_def(out.cfg.EVAL,'power_eval_tau_s',0.05));
end
Pfilt = spinn3d_lpf1_causal(P,dt,tau_eval,P(1));
thr = 1.05*Pbud;
M.t_reach = getfield_def(out,'t_reach',t(end));
M.P_peak = max(Pfilt);              % primary filtered-budget metric [W]
M.P_p99 = pctile(Pfilt,99);
M.P_peak_norm = M.P_peak / max(Pbud,eps);
M.P_p99_norm = M.P_p99 / max(Pbud,eps);
M.pbar_peak = M.P_peak_norm;
M.pbar_p99 = M.P_p99_norm;
M.P_peak_raw = max(P);
M.P_peak_fast = max(Pfilt);         % compatibility alias
M.E_grid = trapz(t,max(P,0));
if isfield(out.log,'Preuse_total'), M.E_reuse = trapz(t,max(out.log.Preuse_total(:),0)); else, M.E_reuse = NaN; end
if isfield(out.log,'Pdump'), M.E_dump = trapz(t,max(out.log.Pdump(:),0)); else, M.E_dump = NaN; end
M.E_excess = trapz(t,max(Pfilt-thr,0));
M.E_excess_norm_s = M.E_excess/max(Pbud,eps);
M.r_viol = mean(Pfilt>thr,'omitnan');
ramp = diff(Pfilt)/dt;
M.ramp_p95 = pctile(abs(ramp),95);
M.ramp_max = max(abs(ramp));
M.ramp_p95_norm = M.ramp_p95/max(Pbud,eps);
M.ramp_max_norm = M.ramp_max/max(Pbud,eps);
M.pbar_ramp_p95 = M.ramp_p95_norm;
M.pbar_ramp_max = M.ramp_max_norm;
if isfield(out.log,'Vdc'), M.Vdc_min = min(out.log.Vdc(:)); else, M.Vdc_min=NaN; end
if isfield(out.log,'Igrid')
    I = out.log.Igrid(:); M.I_peak=max(abs(I)); M.I_rms=sqrt(mean(I.^2,'omitnan'));
else
    M.I_peak=NaN; M.I_rms=NaN;
end
if isfield(out.log,'Pspill'), M.E_spill=trapz(t,max(out.log.Pspill(:),0)); else, M.E_spill=NaN; end
if isfield(out.log,'Pgrid_request')
    Preq = out.log.Pgrid_request(:); M.P_request_peak = max(Preq,[],'omitnan'); M.P_request_p99 = pctile(Preq,99);
else
    M.P_request_peak = NaN; M.P_request_p99 = NaN;
end
if isfield(out.log,'Psource_shortfall')
    Psf = out.log.Psource_shortfall(:); M.Psource_shortfall_peak = max(Psf,[],'omitnan'); M.E_source_shortfall = trapz(t,max(Psf,0));
else
    M.Psource_shortfall_peak = NaN; M.E_source_shortfall = NaN;
end
if isfield(out.log,'Psource_limited'), M.source_limited_frac = mean(double(out.log.Psource_limited(:))>0,'omitnan'); else, M.source_limited_frac = NaN; end
M.dt = dt;
M.power_tau_s = tau_eval;
M.power_slow_tau_s = tau_eval; % compatibility alias
M.power_fast_tau_s = tau_eval; % compatibility alias
end

function T = struct2table_robust(rows)
allf = {};
for i=1:numel(rows), allf = union(allf, fieldnames(rows{i})); end
S = repmat(cell2struct(cell(size(allf)), allf, 2), numel(rows),1);
for i=1:numel(rows)
    for j=1:numel(allf)
        f=allf{j};
        if isfield(rows{i},f), S(i).(f)=rows{i}.(f); else, S(i).(f)=missing; end
    end
end
T = struct2table(S);
end
function p = pctile(x,q)
x=sort(x(isfinite(x))); if isempty(x), p=NaN; return; end
pos=1+(numel(x)-1)*q/100; lo=floor(pos); hi=ceil(pos);
if lo==hi, p=x(lo); else, p=x(lo)+(pos-lo)*(x(hi)-x(lo)); end
end
function y=unif(a,b), y=a+(b-a)*rand(); end
function r=row(x), r=x(:)'; end
function v=getfield_def(S,fn,def), if isstruct(S)&&isfield(S,fn)&&~isempty(S.(fn)), v=S.(fn); else, v=def; end, end
