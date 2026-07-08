function cfg = spinn3d_params_to_cfg(PB, mode)
%SPINN3D_PARAMS_TO_CFG  Convert central PB block to cfg.
% mode is kept for compatibility with old scripts.
if nargin<2, mode='demo_alpha'; end %#ok<NASGU>
cfg = struct();

%% Trajectory / Time / Alpha / Power
cfg.Q0_DEG = PB.traj.q0_deg;
cfg.QF_DEG = PB.traj.qf_deg;
cfg.VMAX   = PB.traj.vmax;
cfg.AMAX   = PB.traj.amax;
cfg.PATH_MODE  = get_nested_def(PB,'traj.mode','joint_scurve');
cfg.PATH_SHAPE = get_nested_def(PB,'traj.shape','quintic_smootherstep');
cfg.TRAJ_MODE = get_nested_def(PB,'traj.mode','joint_scurve');

cfg.TS_SERVO  = get_nested_def(PB,'time.Ts_servo', get_nested_def(PB,'time.Ts',1e-3));
cfg.POLICY_DT = get_nested_def(PB,'policy.dt', get_nested_def(PB,'time.policy_dt', 10e-3));
cfg.POLICY_STEPS = max(1, round(cfg.POLICY_DT / cfg.TS_SERVO));
cfg.POLICY_DT = cfg.POLICY_STEPS * cfg.TS_SERVO;  % exact integer multiple
cfg.POLICY = struct('dt', cfg.POLICY_DT, 'steps', cfg.POLICY_STEPS, 'hold', 'zoh');
cfg.TS     = cfg.TS_SERVO;      % legacy alias: plant/servo integration step
cfg.DT     = PB.sample.dt;

% Use the same geometric path generator in online rollouts and dataset generation.
% These fields are consumed by make_traj_cart_bezier_dls_geom.m.
if isfield(PB,'alpha_budget') && isstruct(PB.alpha_budget)
    cfg.DLS_LAMBDA = get_nested_def(PB,'alpha_budget.dls_lambda',5e-3);
    cfg.DLS_W_ORI  = get_nested_def(PB,'alpha_budget.dls_w_ori',0.02);
    cfg.BEZ_LIFT_Z = get_nested_def(PB,'alpha_budget.bez_lift_z',0.15);
    cfg.BEZ_GAMMA  = get_nested_def(PB,'alpha_budget.bez_gamma',1/3);
    cfg.NS_MIN     = get_nested_def(PB,'alpha_budget.ns_min',200);
end
% Cartesian null-space IK path parameters (used when TRAJ_MODE='cartesian_ik_scurve').
cfg.CART_LIFT_Z      = get_nested_def(PB,'traj.cart_lift_z', get_nested_def(PB,'alpha_budget.bez_lift_z',0.06));
cfg.IK_NULL_WEIGHT   = get_nested_def(PB,'traj.ik_null_weight', 0.12);
cfg.IK_DAMPING       = get_nested_def(PB,'traj.ik_damping', get_nested_def(PB,'alpha_budget.dls_lambda',8e-3));
cfg.IK_MAX_ITER      = get_nested_def(PB,'traj.ik_max_iter', 15);
cfg.IK_TOL_M         = get_nested_def(PB,'traj.ik_tol_m', 1e-4);
cfg.IK_MAX_STEP_RAD  = get_nested_def(PB,'traj.ik_max_step_rad', 0.08);
cfg.TRAJ_DOMAIN = 'geom';

cfg.ALPHA_MIN   = PB.alpha.alpha_min;
cfg.ALPHA_ITMAX = PB.alpha.itmax;

% Clear power names.
cfg.P_GRID_BUDGET = get_nested_def(PB,'power.P_GRID_BUDGET', get_nested_def(PB,'power.P_TOTAL_MAX',15));
cfg.P_TOTAL_MAX   = cfg.P_GRID_BUDGET;  % legacy alias for old feature/training code
cfg.V_BUS_NOMINAL = get_nested_def(PB,'power.V_BUS_NOMINAL', 12.0);
cfg.VBUS_NOMINAL  = cfg.V_BUS_NOMINAL;  % compact compatibility alias
cfg.I_SUPPLY_RATING = get_nested_def(PB,'power.I_SUPPLY_RATING', 5.0);
cfg.IBUS_RATING     = cfg.I_SUPPLY_RATING;
cfg.P_HW_RATING   = get_nested_def(PB,'power.P_SUPPLY_MAX', get_nested_def(PB,'power.P_HW_RATING', cfg.V_BUS_NOMINAL*cfg.I_SUPPLY_RATING));
cfg.P_SUPPLY_RATING = cfg.P_HW_RATING;
cfg.P_GRID_PEAK_MAX = get_nested_def(PB,'power.P_GRID_PEAK_MAX', cfg.P_HW_RATING);
cfg.I_GRID_BUDGET = cfg.P_GRID_BUDGET / max(cfg.V_BUS_NOMINAL, eps);

% Evaluation filter defaults, used by paper_power_eval.
cfg.EVAL = struct();
if isfield(PB,'eval') && isstruct(PB.eval)
    cfg.EVAL = PB.eval;
end

if isfield(PB,'paths') && isfield(PB.paths,'model_alpha')
    cfg.MODEL_PATH = PB.paths.model_alpha;
end
if isfield(PB,'paths') && isfield(PB.paths,'model_alpha_mlp')
    cfg.MLP_MODEL_PATH = PB.paths.model_alpha_mlp;
end

%% Robot
cfg.ROBOT = resolve_robot(PB);

%% Limits / Gains / Stop
cfg.CAPS = PB.caps;
cfg.CAPS.P_grid_budget   = cfg.P_GRID_BUDGET;
cfg.CAPS.P_total_max     = cfg.P_GRID_BUDGET;  % legacy alias
cfg.CAPS.P_grid_peak_max = cfg.P_GRID_PEAK_MAX;
if isfield(PB,'bus') && isstruct(PB.bus)
    if isfield(PB.bus,'eta_share'),  cfg.CAPS.eta_share  = PB.bus.eta_share; end
    if isfield(PB.bus,'P_brk_peak'), cfg.CAPS.P_brk_peak = PB.bus.P_brk_peak; end
end

cfg.CTRL = struct('Kp', PB.control.Kp, 'Kd', PB.control.Kd);
cfg.STOP = struct('EE_TOL', PB.stop.ee_tol, ...
                  'NEAR_TOL', PB.stop.ee_tol, ...
                  'DQ_TOL_RAD', PB.stop.dq_tol_rad, ...
                  'VEL_TOL', PB.stop.dq_tol_rad, ...
                  'STABLE_TIME', PB.stop.stable_time);

%% Friction & Noise
cfg.FRIC  = PB.fric;
cfg.NOISE = PB.noise;

%% Alpha Regulator
if isfield(PB,'alpha_reg') && ~isempty(PB.alpha_reg)
    cfg.ALPHA_REG = struct( ...
        'BETA',     PB.alpha_reg.beta, ...
        'A_DOT_UP', PB.alpha_reg.a_dot_up, ...
        'A_DOT_DN', PB.alpha_reg.a_dot_dn ...
    );
    if isfield(PB.alpha_reg,'kinematic_cap_before_filter')
        cfg.ALPHA_REG.KINEMATIC_CAP_BEFORE_FILTER = PB.alpha_reg.kinematic_cap_before_filter;
    end
end

%% Budget-utilization feedback for TRF/MLP PRG
if isfield(PB,'budget_ctrl') && ~isempty(PB.budget_ctrl)
    cfg.BUDGET_CTRL = PB.budget_ctrl;
end

%% Bus
cfg.BUS = struct();
if isfield(PB,'bus') && ~isempty(PB.bus)
    cfg.BUS = PB.bus;
end
cfg.BUS.P_grid_budget   = cfg.P_GRID_BUDGET;
cfg.BUS.P_total_max     = cfg.P_GRID_BUDGET;  % legacy alias
cfg.BUS.P_grid_peak_max = cfg.P_GRID_PEAK_MAX;
if ~isfield(cfg.BUS,'P_hw_rating') || isempty(cfg.BUS.P_hw_rating)
    cfg.BUS.P_hw_rating = cfg.P_HW_RATING;
end
cfg.BUS = spinn3d_bus_defaults(cfg.BUS);

%% Debug flags
cfg.LOG_AFEAS = get_nested_def(PB,'debug.log_afeas',false);

%% Payload
cfg.PAYLOAD = struct();
if isfield(PB,'payload') && ~isempty(PB.payload)
    if isfield(PB.payload,'demo') && ~isempty(PB.payload.demo)
        cfg.PAYLOAD.DEMO = PB.payload.demo;
    end
    if isfield(PB.payload,'train') && ~isempty(PB.payload.train)
        cfg.PAYLOAD.TRAIN = PB.payload.train;
    end
    if isfield(PB.payload,'range') && ~isempty(PB.payload.range)
        cfg.PAYLOAD.RANGE = PB.payload.range;
    end
end
end

% ---- helpers ----
function robot = resolve_robot(PB)
robot = [];
if isfield(PB,'robot') && isfield(PB.robot,'object') && ~isempty(PB.robot.object)
    robot = PB.robot.object; return;
end
if isfield(PB,'robot') && isfield(PB.robot,'builder') && ~isempty(PB.robot.builder)
    robot = feval(PB.robot.builder); return;
end
try
    if evalin('base','exist(''robot'',''var'')'), robot = evalin('base','robot'); return; end
end
error('PB.robot must provide object or builder, or a robot variable must exist in the base workspace.');
end

function v = get_nested_def(S, path, def)
parts = strsplit(path,'.');
v = S;
for i=1:numel(parts)
    if isstruct(v) && isfield(v,parts{i}) && ~isempty(v.(parts{i}))
        v = v.(parts{i});
    else
        v = def; return;
    end
end
end
