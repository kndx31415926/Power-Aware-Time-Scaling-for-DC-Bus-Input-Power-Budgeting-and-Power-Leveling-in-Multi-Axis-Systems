function R = paper_budget_filter_policy_sensitivity_parallel(varargin)
%PAPER_BUDGET_FILTER_POLICY_SENSITIVITY_PARALLEL
% Sequential condition sweep; each condition uses paper_fair_baseline_eval with
% a shared MATLAB parallel pool across trials.

[cfg_base, opts] = parse_inputs(varargin{:});
ensure_dir(opts.export_dir);
if opts.use_parallel, paper_open_parallel_pool(opts); end

conds = make_conditions(opts);
Rconds = cell(numel(conds),1);
summary_rows = {};
for c = 1:numel(conds)
    cfg = cfg_base;
    cfg.P_GRID_BUDGET = conds(c).P_grid_budget_W;
    cfg.P_TOTAL_MAX = conds(c).P_grid_budget_W;
    cfg.POLICY_DT = conds(c).policy_dt_s;
    cfg.POLICY_STEPS = max(1,round(cfg.POLICY_DT/cfg.TS));
    cfg.POLICY_DT = cfg.POLICY_STEPS*cfg.TS;
    cfg.POLICY = struct('dt',cfg.POLICY_DT,'steps',cfg.POLICY_STEPS,'hold','zoh');
    if isfield(cfg,'BUS')
        cfg.BUS.P_grid_budget = cfg.P_GRID_BUDGET; cfg.BUS.P_total_max = cfg.P_GRID_BUDGET;
        cfg.BUS.I_budget = cfg.P_GRID_BUDGET/max(cfg.BUS.V_nom,eps);
    end
    if isfield(cfg,'CAPS'), cfg.CAPS.P_grid_budget=cfg.P_GRID_BUDGET; cfg.CAPS.P_total_max=cfg.P_GRID_BUDGET; end

    fopts = struct();
    fopts.N = opts.N;
    fopts.seed = opts.seed + c;
    fopts.use_parallel = opts.use_parallel;
    fopts.num_workers = opts.num_workers;
    fopts.q_margin_deg = opts.q_margin_deg;
    fopts.payload_mass_range = opts.payload_mass_range;
    fopts.power_eval_mode = 'lpf';
    fopts.power_eval_tau_s = conds(c).tau_s;
    fopts.power_tau_s = conds(c).tau_s;  % keep metric helper from overriding the swept LPF time constant
    fopts.Pbudget_margin_frac = conds(c).rho;
    fopts.methods = opts.methods;
    fopts.PTP_OPTS = opts.PTP_OPTS;
    fopts.MPC_OPTS = opts.MPC_OPTS;
    fopts.export_dir = fullfile(opts.export_dir, sprintf('P%gW_tau%dms_rho%g_Tpol%dms', ...
        conds(c).P_grid_budget_W, round(1000*conds(c).tau_s), conds(c).rho, round(1000*conds(c).policy_dt_s)));
    if isfield(opts,'mlp_model_path'), fopts.mlp_model_path = opts.mlp_model_path; end
    fprintf('[sweep] condition %d/%d: P=%.1f W, tau=%.3f s, rho=%.3f, Tpol=%.0f ms\n', ...
        c,numel(conds),conds(c).P_grid_budget_W,conds(c).tau_s,conds(c).rho,1000*conds(c).policy_dt_s);
    Rconds{c} = paper_fair_baseline_eval(cfg, fopts);
    S = Rconds{c}.summary_table;
    for r=1:height(S)
        rr = table2struct(S(r,:));
        rr.condition_id = c;
        rr.P_grid_budget_W = conds(c).P_grid_budget_W;
        rr.tau_s = conds(c).tau_s;
        rr.rho = conds(c).rho;
        rr.policy_dt_s = conds(c).policy_dt_s;
        summary_rows{end+1,1} = rr; %#ok<AGROW>
    end
end
Sall = struct2table(cell2mat(summary_rows));
writetable(Sall,fullfile(opts.export_dir,'budget_filter_policy_sensitivity_summary.csv'));
save(fullfile(opts.export_dir,'budget_filter_policy_sensitivity.mat'),'Rconds','Sall','conds','opts','cfg_base');
R = struct('conditions',conds,'summary_table',Sall,'runs',{Rconds},'export_dir',opts.export_dir);
end

function conds = make_conditions(opts)
idx=0; conds=struct('P_grid_budget_W',{},'tau_s',{},'rho',{},'policy_dt_s',{});
for p=opts.Pbudget_list_W
    for tau=opts.tau_list_s
        for rho=opts.rho_list
            for tp=opts.policy_dt_list_s
                idx=idx+1;
                conds(idx).P_grid_budget_W = p;
                conds(idx).tau_s = tau;
                conds(idx).rho = rho;
                conds(idx).policy_dt_s = tp;
            end
        end
    end
end
end

function [cfg, opts] = parse_inputs(varargin)
if nargin==0
    PB=spinn3d_params_block(); cfg=spinn3d_params_to_cfg(PB,'demo_alpha'); opts=default_opts(struct()); return;
elseif nargin==1
    if isstruct(varargin{1}) && isfield(varargin{1},'Q0_DEG')
        cfg=varargin{1}; opts=default_opts(struct()); return;
    else
        PB=spinn3d_params_block(); cfg=spinn3d_params_to_cfg(PB,'demo_alpha'); opts=default_opts(varargin{1}); return;
    end
else
    cfg=varargin{1}; opts=default_opts(varargin{2}); return;
end
end
function opts = default_opts(opts)
def = struct('N',30,'seed',50,'use_parallel',true,'num_workers',[], ...
    'methods',{{'NoGuard','TRF-PRG','MLP-PRG','Feasible-RG'}},'q_margin_deg',10,'payload_mass_range',[0.20 0.50], ...
    'Pbudget_list_W',[10 15 20 30],'tau_list_s',[0.20 0.50 1.00],'rho_list',[0.02 0.05 0.10], ...
    'policy_dt_list_s',[0.010 0.020], 'export_dir',fullfile(pwd,'paper_results_budget_filter_policy_sensitivity'), ...
    'PTP_OPTS',struct('Ns',61,'Na',25),'MPC_OPTS',struct('Na',11,'H',4,'beamN',10));
try
    PB=spinn3d_params_block(); def.mlp_model_path=fullfile(PB.paths.root,'model_alpha_gov_mlp.mat');
catch, def.mlp_model_path=fullfile(pwd,'data_alpha','model_alpha_gov_mlp.mat'); end
f=fieldnames(def); for i=1:numel(f), if ~isfield(opts,f{i})||isempty(opts.(f{i})), opts.(f{i})=def.(f{i}); end, end
end
function ensure_dir(d), if exist(d,'dir')~=7, mkdir(d); end, end
