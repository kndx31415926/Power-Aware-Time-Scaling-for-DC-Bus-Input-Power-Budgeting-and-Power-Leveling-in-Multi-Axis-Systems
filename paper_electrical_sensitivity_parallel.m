function R = paper_electrical_sensitivity_parallel(varargin)
%PAPER_ELECTRICAL_SENSITIVITY_PARALLEL  Strong electrical-model sensitivity using parpool.
%
% Samples efficiency, inverter/copper loss, DC-link capacitance, internal
% resistance, voltage sag, braking limits, and budget variation.  Each worker
% runs complete closed-loop rollouts with the same metrics as the main paper.

[cfg_base, opts] = parse_inputs(varargin{:});
ensure_dir(opts.export_dir);
if opts.use_parallel, paper_open_parallel_pool(opts); end

items = make_items(cfg_base, opts);
metricList = {'t_reach','P_peak','P_p99','P_peak_norm','P_p99_norm','pbar_peak','pbar_p99', ...
    'P_peak_raw','P_peak_fast','P_peak_slow','P_p99_fast','P_p99_slow', ...
    'P_peak_lpf_norm','P_p99_lpf_norm','pbar_lpf_peak','pbar_lpf_p99', ...
    'E_excess','E_excess_norm_s','E_grid','E_reuse','E_dump','E_mech_pos','E_mech_regen', ...
    'I_rms','I_peak','Vdc_min','P_loss_energy','E_spill', ...
    'P_request_peak','P_request_p99','Psource_shortfall_peak','E_source_shortfall','source_limited_frac', ...
    'P_std','P_std_norm','P_rms_to_Pbudget', ...
    'ramp_rms','ramp_p95','ramp_max','ramp_p95_norm','ramp_max_norm','pbar_ramp_p95','pbar_ramp_max', ...
    'ramp_p95_lpf_norm','ramp_max_lpf_norm','r_viol'};
rows = cell(numel(items),1);
tAll = tic;
if opts.use_parallel && ~isempty(gcp('nocreate'))
    parfor i=1:numel(items)
        rows{i} = run_item(items(i), opts, metricList);
    end
else
    for i=1:numel(items)
        rows{i} = run_item(items(i), opts, metricList);
        if opts.verbose, fprintf('[elec-sens] item %d/%d\n',i,numel(items)); end
    end
end
T = struct2table(vertcat(rows{:}));
S = summarize(T,metricList);
writetable(T,fullfile(opts.export_dir,'electrical_sensitivity_trials.csv'));
writetable(S,fullfile(opts.export_dir,'electrical_sensitivity_summary.csv'));
save(fullfile(opts.export_dir,'electrical_sensitivity.mat'),'T','S','opts','cfg_base','items','metricList');
R = struct('trials_table',T,'summary_table',S,'export_dir',opts.export_dir,'script_wall_time_s',toc(tAll));
fprintf('[elec-sens] done: %s\n', opts.export_dir);
end

function row = run_item(item, opts, metricList)
row = init_row(metricList);
row.sample_id = item.sample_id;
row.trial_id = item.trial_id;
row.method = string(item.method);
row.P_grid_budget_W = item.cfg.P_GRID_BUDGET;
row.eta_mot = item.cfg.BUS.eta_mot;
row.eta_regen = item.cfg.BUS.eta_regen;
row.eta_share = item.cfg.BUS.eta_share;
row.R_batt = item.cfg.BUS.R_batt;
row.Cdc = item.cfg.BUS.Cdc;
row.V_nom = item.cfg.BUS.V_nom;
row.P_brk_peak = item.cfg.BUS.P_brk_peak;
try
    cfg = item.cfg; cfg.NO_FIGURE = true; cfg.DISABLE_FIGURES = true;
    t0=tic;
    out = run_method(item.method,cfg,opts);
    row.rollout_wall_time_s = toc(t0);
    M = spinn3d_compute_power_metrics(out,cfg.P_GRID_BUDGET,opts);
    for k=1:numel(metricList)
        if isfield(M,metricList{k}), row.(metricList{k}) = M.(metricList{k}); end
    end
    row.success = true; row.error = string('');
catch ME
    row.success = false; row.error = string(ME.message); row.rollout_wall_time_s = NaN;
end
end

function out = run_method(method,cfg,opts)
switch lower(strrep(method,'_','-'))
    case {'noguard','baseline'}, out = spinn3d_demo_ctcgov_online_nonn_nolimit(cfg);
    case {'trf-prg','trf'}, out = spinn3d_demo_ctcgov_online_nn(cfg);
    case {'mlp-prg','mlp'}
        if isfield(opts,'mlp_model_path'), cfg.MLP_MODEL_PATH = opts.mlp_model_path; end
        out = spinn3d_demo_ctcgov_online_mlp(cfg);
    case {'feasible-rg','feasible'}, out = spinn3d_demo_ctcgov_online_nonn(cfg);
    otherwise, error('Unsupported sensitivity method: %s',method);
end
end

function items = make_items(cfg_base, opts)
rng(opts.seed);
items = repmat(struct(),opts.Nsens*opts.Ntraj*numel(opts.methods),1);
idx=0;
for s=1:opts.Nsens
    bus_sample = sample_bus(cfg_base.BUS, opts);
    for tr=1:opts.Ntraj
        cfg = cfg_base;
        cfg.BUS = bus_sample;
        cfg.V_BUS_NOMINAL = bus_sample.V_nom;
        % random task/payload for each trajectory
        nJ = numel(cfg.Q0_DEG(:)); mg = opts.q_margin_deg; if isscalar(mg), mg=repmat(mg,nJ,1); end
        cfg.Q0_DEG = cfg.Q0_DEG(:) + (2*rand(nJ,1)-1).*mg(:);
        cfg.QF_DEG = cfg.QF_DEG(:) + (2*rand(nJ,1)-1).*mg(:);
        m = opts.payload_mass_range(1)+(opts.payload_mass_range(2)-opts.payload_mass_range(1))*rand();
        cfg = set_payload_mass(cfg,m);
        Pscale = opts.Pbudget_scale_range(1)+(opts.Pbudget_scale_range(2)-opts.Pbudget_scale_range(1))*rand();
        Pbud = getfield_def(cfg_base,'P_GRID_BUDGET',15)*Pscale;
        cfg.P_GRID_BUDGET = Pbud; cfg.P_TOTAL_MAX = Pbud; cfg.BUS.P_grid_budget = Pbud; cfg.BUS.P_total_max = Pbud;
        if isfield(cfg,'CAPS'), cfg.CAPS.P_grid_budget = Pbud; cfg.CAPS.P_total_max = Pbud; end
        for mth=1:numel(opts.methods)
            idx=idx+1;
            items(idx).sample_id = s;
            items(idx).trial_id = tr;
            items(idx).method = opts.methods{mth};
            items(idx).cfg = cfg;
        end
    end
end
items = items(1:idx);
end

function bus = sample_bus(bus0, opts)
bus = bus0;
bus.eta_mot = urand(opts.eta_mot_range);
bus.eta_regen = urand(opts.eta_regen_range);
bus.eta_share = urand(opts.eta_share_range);
bus.R_batt = urand(opts.R_batt_range);
bus.Cdc = urand(opts.Cdc_range);
bus.V_nom = urand(opts.V_nom_range);
% Keep the open-circuit/source voltage synchronized with the sampled nominal
% voltage.  Otherwise voltage-sensitivity runs can silently keep the old V_oc
% from cfg_base while only auxiliary losses see the sampled V_nom.
bus.V_oc = bus.V_nom;
if isfield(bus,'I_rating') && isfinite(bus.I_rating) && bus.I_rating > 0
    bus.P_hw_rating = bus.V_nom * bus.I_rating;
    bus.P_supply_rating = bus.P_hw_rating;
    bus.P_source_max_W = bus.P_hw_rating;
end
bus.P_brk_peak = urand(opts.P_brk_peak_range);
if isfield(bus,'P_inv0'), bus.P_inv0 = urand(opts.P_inv0_range); end
if isfield(bus,'k_inv'), bus.k_inv = urand(opts.k_inv_range); end
if isfield(bus,'R_phase')
    bus.R_phase = bus.R_phase * urand(opts.R_phase_scale_range);
end
end
function v = urand(r), v = r(1)+(r(2)-r(1))*rand(); end

function row = init_row(metricList)
row = struct('sample_id',NaN,'trial_id',NaN,'method',"",'success',false,'error',string(''), ...
    'rollout_wall_time_s',NaN,'P_grid_budget_W',NaN,'eta_mot',NaN,'eta_regen',NaN,'eta_share',NaN, ...
    'R_batt',NaN,'Cdc',NaN,'V_nom',NaN,'P_brk_peak',NaN);
for k=1:numel(metricList), row.(metricList{k}) = NaN; end
end
function S = summarize(T,metricList)
methods=unique(T.method,'stable'); rows={};
for im=1:numel(methods)
    idx=(T.method==methods(im)) & T.success;
    for k=1:numel(metricList)
        v=T.(metricList{k})(idx);
        r=struct('method',char(methods(im)),'metric',metricList{k},'N',sum(idx), ...
            'mean',mean(v,'omitnan'),'std',std(v,'omitnan'),'p05',pctile(v,5),'p95',pctile(v,95), ...
            'worst95',pctile(v,95));
        rows{end+1,1}=r; %#ok<AGROW>
    end
end
S=struct2table(cell2mat(rows));
end

function [cfg, opts] = parse_inputs(varargin)
if nargin==0
    PB=spinn3d_params_block(); cfg=spinn3d_params_to_cfg(PB,'demo_alpha'); opts=default_opts(struct(),cfg); return;
elseif nargin==1
    if isstruct(varargin{1}) && isfield(varargin{1},'Q0_DEG')
        cfg=varargin{1}; opts=default_opts(struct(),cfg); return;
    else
        PB=spinn3d_params_block(); cfg=spinn3d_params_to_cfg(PB,'demo_alpha'); opts=default_opts(varargin{1},cfg); return;
    end
else
    cfg=varargin{1}; opts=default_opts(varargin{2},cfg); return;
end
end
function opts = default_opts(opts,cfg)
def = struct('Nsens',30,'Ntraj',5,'seed',11,'verbose',true,'use_parallel',true,'num_workers',[], ...
    'methods',{{'NoGuard','Feasible-RG','MLP-PRG','TRF-PRG'}},'q_margin_deg',10,'payload_mass_range',[0.20 0.50], ...
    'power_eval_mode','lpf','power_tau_s',0.05,'power_eval_tau_s',0.05,'Pbudget_margin_frac',0.05, ...
    'Pbudget_scale_range',[0.8 1.2],'eta_mot_range',[0.70 0.95],'eta_regen_range',[0.0 0.80], ...
    'eta_share_range',[0.50 0.95],'R_batt_range',[0.05 0.30],'Cdc_range',[470e-6 4700e-6], ...
    'V_nom_range',[10.5 12.6],'P_brk_peak_range',[5 30],'P_inv0_range',[0 2], ...
    'k_inv_range',[0 0.10],'R_phase_scale_range',[0.5 2.0], ...
    'export_dir',fullfile(pwd,'paper_results_electrical_sensitivity'));
try
    PB=spinn3d_params_block(); def.mlp_model_path=fullfile(PB.paths.root,'model_alpha_gov_mlp.mat');
catch, def.mlp_model_path=fullfile(pwd,'data_alpha','model_alpha_gov_mlp.mat'); end
f=fieldnames(def); for i=1:numel(f), if ~isfield(opts,f{i})||isempty(opts.(f{i})), opts.(f{i})=def.(f{i}); end, end
end
function cfg = set_payload_mass(cfg,m)
if ~isfield(cfg,'PAYLOAD')||~isstruct(cfg.PAYLOAD), cfg.PAYLOAD=struct(); end
if ~isfield(cfg.PAYLOAD,'DEMO')||~isstruct(cfg.PAYLOAD.DEMO), cfg.PAYLOAD.DEMO=struct(); end
p=cfg.PAYLOAD.DEMO; p.mass=m; if ~isfield(p,'com'), p.com=[0 0 0]; end; if ~isfield(p,'inertia'), p.inertia=[0 0 0 0 0 0]; end; cfg.PAYLOAD.DEMO=p;
end
function p = pctile(x,q)
x=x(isfinite(x)); x=sort(x(:)); if isempty(x), p=NaN; return; end; q=max(0,min(100,q)); pos=1+(numel(x)-1)*q/100; lo=floor(pos); hi=ceil(pos); if lo==hi, p=x(lo); else, p=x(lo)+(pos-lo)*(x(hi)-x(lo)); end
end
function x=getfield_def(S,fn,def), if isstruct(S)&&isfield(S,fn)&&~isempty(S.(fn)), x=S.(fn); else, x=def; end, end
function ensure_dir(d), if exist(d,'dir')~=7, mkdir(d); end, end
