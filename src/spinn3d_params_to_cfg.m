function cfg = spinn3d_params_to_cfg(PB, mode)
% mode: 'demo_alpha' | 'offline_nonn' | 'offline_nn' | 'alpha_run'
if nargin < 2, mode = 'demo_alpha'; end %#ok<NASGU>

soft_power_fields = {'enable','schema','target_W','internal_margin_W', ...
    'preview_horizon_s','candidate_count','speed_margin_s', ...
    'terminal_stable_time_s','soft_excess_tolerance','apply_methods', ...
    'alpha_max'};

stable_policy_ratio = double(PB.soft_power.terminal_stable_time_s)/ ...
    double(PB.policy.dt);

soft_contract_fields = {'schema','metric_schema','minimum_time_gain_s', ...
    'relative_excess_improvement_frac','zero_excess_tolerance', ...
    'absolute_budget_tolerance_W','power_tie_tolerance', ...
    'time_tie_tolerance_s','speed_violation_penalty', ...
    'relative_excess_violation_penalty','absolute_excess_penalty', ...
    'require_noguard_reference','no_dominance_action'};

soft_contract = PB.joint.soft_contract;
soft_numeric = [double(soft_contract.minimum_time_gain_s), ...
    double(soft_contract.relative_excess_improvement_frac), ...
    double(soft_contract.zero_excess_tolerance), ...
    double(soft_contract.absolute_budget_tolerance_W), ...
    double(soft_contract.power_tie_tolerance), ...
    double(soft_contract.time_tie_tolerance_s), ...
    double(soft_contract.speed_violation_penalty), ...
    double(soft_contract.relative_excess_violation_penalty), ...
    double(soft_contract.absolute_excess_penalty)];

alpha_feasible_fields = {'alpha_max','grid_min_points','grid_max_points', ...
    'grid_points_per_itmax','grid_point_offset','around_prev_min_step', ...
    'around_prev_step_multipliers','around_prev_geom_signs', ...
    'around_prev_geom_exp10','bisection_tolerance', ...
    'feasibility_tolerance','alpha_dot_time_epsilon_s', ...
    'soft_selection_schema'};

spinn3d_control_schedule( ...
    PB.time.Ts,PB.policy,PB.alpha_gate,PB.actual_guard);

cfg = struct();
cfg.REVISION_R6=struct('enabled',true,'reuse',true,'study_kind','paper', ...
    'execution_dir',fullfile(fileparts(mfilename('fullpath')),'r6_records'), ...
    'context',struct());

%% Trajectory / Time / Alpha / Power
trajectory_contract_fields = {'q0_deg','qf_deg','vmax','amax'};

% evaluation is allowed to replace Q0_DEG/QF_DEG inside the signed training
% jitter domain, but it must never mutate this central nominal contract.
cfg.CENTRAL_TRAJECTORY = PB.traj;
cfg.Q0_DEG = cfg.CENTRAL_TRAJECTORY.q0_deg;
cfg.QF_DEG = cfg.CENTRAL_TRAJECTORY.qf_deg;
cfg.VMAX   = PB.motion.qd_max_rad_s;
cfg.AMAX   = PB.motion.qdd_max_rad_s2;
cfg.TS     = PB.time.Ts;
cfg.DT     = PB.sample.dt;

%% Policy / inference cadence
cfg.POLICY = PB.policy;

cfg.FAIRNESS = PB.fairness;
cfg.SOFT_POWER = PB.soft_power;
execution_audit_fields = {'ratio_tolerance', ...
    'position_planning_tolerance_rad','position_tolerance_rad', ...
    'filtered_power_tolerance_W','physical_power_abs_tolerance_W', ...
    'physical_power_rel_tolerance','spill_power_tolerance_W', ...
    'telemetry_match_tolerance_W','energy_balance_tolerance_W', ...
    'joint_limit_viability','recovery'};

cfg.EXECUTION_AUDIT = PB.execution_audit;

cfg.ALPHA_MIN   = PB.alpha.alpha_min;

cfg.ALPHA_MAX   = PB.alpha.alpha_max;
cfg.ALPHA_ITMAX = PB.alpha.itmax;
cfg.ALPHA_FEASIBLE = PB.alpha_feasible;

NG=PB.noguard_policy;

for offset_name={'local_offsets','successor_offsets'}
    offsets=NG.(offset_name{1});

end
cfg.NOGUARD_POLICY=NG;

cfg.P_TOTAL_MAX = PB.power.P_TOTAL_MAX;
cfg.POWER_SEMANTICS = struct( ...
    'filtered_experimental_budget_W',PB.power.P_TOTAL_MAX, ...
    'filtered_budget_semantics',PB.power.PAVG_SEMANTICS, ...
    'budget_role','soft_learning_target_and_reported_KPI', ...
    'instantaneous_peak_role','physical_PSU_source_guard');

%% Model paths
if isfield(PB,'paths') && isfield(PB.paths,'model_alpha')
    cfg.MODEL_PATH = PB.paths.model_alpha;
    cfg.MODEL_PATH_ALPHA = PB.paths.model_alpha;
    cfg.MODEL_PATH_ALPHA_SUPERVISED = PB.paths.model_alpha_supervised;
    cfg.MODEL_PATH_ALPHA_MLP = PB.paths.model_alpha_mlp;
    cfg.MODEL_PATH_ALPHA_MLP_SUPERVISED = PB.paths.model_alpha_mlp_supervised;
    cfg.ALPHA_EXPECTED_ARCHITECTURE = 'transformer';
    cfg.ALPHA_METHOD_ID = 'nn';
end
if isfield(PB,'paths') && isfield(PB.paths,'model_theta')
    cfg.MODEL_PATH_THETA = PB.paths.model_theta;
end

%% Robot
robot_params = resolve_robot_params(PB);
cfg.ROBOT = resolve_robot(PB,robot_params);

cfg.EE_NAME = char(string(PB.robot.ee_name));

cfg.PLOT = struct('enable',logical(PB.demo.plot_enable));
if isfield(PB,'physics') && isstruct(PB.physics)
    cfg.PHYSICS = PB.physics;
else
    cfg.PHYSICS = struct();
end
if ~isempty(robot_params)
    if isfield(robot_params,'profile_id')
        cfg.PHYSICS.profile_id = robot_params.profile_id;
    end
    if isfield(robot_params,'calibration_status')
        cfg.PHYSICS.calibration_status = robot_params.calibration_status;
    end
    if isfield(robot_params,'hardware_map') && ...
            isfield(robot_params.hardware_map,'calibrated')
        cfg.PHYSICS.hardware_map_calibrated = ...
            logical(robot_params.hardware_map.calibrated);
    end
end
if ~isempty(robot_params)
    cfg.ROBOT_PARAMS = robot_params;
    if isfield(robot_params,'hardware_map')
        cfg.HARDWARE_MAP = robot_params.hardware_map;
    end
end

%% Limits / Gains / Stop
cfg.CAPS = PB.caps;

cfg.CAPS.alpha_feasible = cfg.ALPHA_FEASIBLE;
cfg.CAPS.P_total_max = PB.power.P_TOTAL_MAX;
[cfg.CAPS.q_min,cfg.CAPS.q_max] = robot_joint_limits_local( ...
    cfg.ROBOT,numel(cfg.CAPS.qd_max));
qd_expected = double(PB.motion.qd_max_rad_s);
qdd_expected = double(PB.motion.qdd_max_rad_s2);

cfg.CTRL = struct('Kp', PB.control.Kp, 'Kd', PB.control.Kd);
cfg.SERVO_HOLD_T = PB.control.servo_hold_time_s;
cfg.STOP = struct( ...
    'EE_TOL',      PB.stop.ee_tol, ...
    'DQ_TOL_RAD',  PB.stop.dq_tol_rad, ...
    'STABLE_TIME', PB.stop.stable_time, ...
    'NEAR_TOL',    PB.stop.ee_tol, ...
    'VEL_TOL',     PB.stop.dq_tol_rad );

%% Workspace / mounting convention
if isfield(PB,'workspace') && isstruct(PB.workspace)
    cfg.WORKSPACE = PB.workspace;
else
    cfg.WORKSPACE = struct('workplane_enable',false);
end
workspace_required = {'geometry_schema','capsule_body_names', ...
    'capsule_start_local_m','capsule_end_local_m','capsule_radius_m', ...
    'workplane_enable','workplane_z_m','clearance_m', ...
    'self_collision_enable','self_collision_pair_indices', ...
    'self_collision_margin_m','payload_body_name', ...
    'payload_box_dims_max_m','payload_check_requires_positive_mass', ...
    'payload_collision_capsule_indices'};

%% Friction / Noise
cfg.FRIC  = PB.fric;
cfg.NOISE = PB.noise;
if isfield(PB,'motor') && isstruct(PB.motor)
    cfg.MOTOR = PB.motor;
end

%% Alpha regulator
cfg.ALPHA_REG = struct( ...
    'BETA',     PB.alpha_reg.beta, ...
    'A_DOT_UP', PB.alpha_reg.a_dot_up, ...
    'A_DOT_DN', PB.alpha_reg.a_dot_dn );
cfg.ALPHA_GATE = PB.alpha_gate;

%% Payload
cfg.PAYLOAD = struct();
if isfield(PB,'payload') && ~isempty(PB.payload)
    if isfield(PB.payload,'demo'),  cfg.PAYLOAD.DEMO  = PB.payload.demo;  end
    if isfield(PB.payload,'train') && ~isempty(PB.payload.train), cfg.PAYLOAD.TRAIN = PB.payload.train; end
    if isfield(PB.payload,'range'), cfg.PAYLOAD.RANGE = PB.payload.range; end
    if isfield(PB.payload,'definition'), cfg.PAYLOAD.DEFINITION = PB.payload.definition; end
end

%% BUS
bus = struct('P_total_max', PB.power.P_TOTAL_MAX);
if isfield(PB,'bus') && isstruct(PB.bus)
    fn = fieldnames(PB.bus);
    for i = 1:numel(fn)
        bus.(fn{i}) = PB.bus.(fn{i});
    end
end
if isfield(PB,'motor') && isstruct(PB.motor) && ...
        isfield(PB.motor,'tau_dead') && ~isempty(PB.motor.tau_dead)
    bus.tau_dead_Nm = PB.motor.tau_dead;
end
cfg.BUS = spinn3d_bus_defaults(bus);
cfg.BUS.P_total_max = PB.power.P_TOTAL_MAX;
if ~strcmp(cfg.BUS.model_scope,'dynamic_capacitor_common_dc_v1')
    cfg.BUS.P_dump_max=cfg.BUS.P_brk_peak;
end

% Preserve every centrally defined electrical architecture so the formal
% bus-profile ablation can swap only the BUS contract while keeping the
% same robot, task manifest, controller and verifier.  Each profile is
% normalized independently; no runner is allowed to invent omitted fields.
cfg.BUS_PROFILES = struct();
if isfield(PB,'bus_profiles') && isstruct(PB.bus_profiles)
    profile_names = fieldnames(PB.bus_profiles);
    for ip = 1:numel(profile_names)
        profile_name = profile_names{ip};
        profile_bus = PB.bus_profiles.(profile_name);
        profile_bus.P_total_max = PB.power.P_TOTAL_MAX;
        if isfield(PB,'motor') && isstruct(PB.motor) && ...
                isfield(PB.motor,'tau_dead') && ~isempty(PB.motor.tau_dead)
            profile_bus.tau_dead_Nm = PB.motor.tau_dead;
        end
        profile_bus = spinn3d_bus_defaults(profile_bus);
        profile_bus.P_total_max = PB.power.P_TOTAL_MAX;
        if ~strcmp(profile_bus.model_scope,'dynamic_capacitor_common_dc_v1')
            profile_bus.P_dump_max=profile_bus.P_brk_peak;
        end
        cfg.BUS_PROFILES.(profile_name) = profile_bus;
    end
end
if isfield(PB,'experiment') && isstruct(PB.experiment)
    cfg.EXPERIMENT = PB.experiment;
end

% alpha_feasible callers that do not pass caps.bus still see them.
cfg.CAPS.P_grid_peak_max   = cfg.BUS.P_grid_peak_max;
cfg.CAPS.P_avg_window_s    = cfg.BUS.P_avg_window_s;
cfg.CAPS.P_avg_target_frac = cfg.BUS.P_avg_target_frac;
cfg.CAPS.P_avg_hard_enable = cfg.BUS.P_avg_hard_enable;
cfg.CAPS.bus = cfg.BUS;

%% Bézier defaults
cfg.BEZ_Z_CLEAR = PB.bez.z_clear;
cfg.BEZ_LAT     = PB.bez.lat;
cfg.BEZ_K1      = PB.bez.k1;
cfg.BEZ_K2      = PB.bez.k2;
cfg.BEZ_LIFT_Z  = PB.bez.z_clear;
cfg.BEZ_GAMMA   = PB.trajectory_builder.legacy_geom_bez_gamma;
cfg.NS_MIN      = PB.trajectory_builder.legacy_geom_ns_min;
cfg.DLS_LAMBDA  = PB.trajectory_builder.dls_lambda;
cfg.DLS_W_ORI   = PB.trajectory_builder.dls_w_ori;
cfg.DLS_QSTEP_DEG = PB.trajectory_builder.dls_qstep_deg;
cfg.DLS_POS_TOL = PB.trajectory_builder.dls_pos_tol_m;
cfg.DLS_INNER_MAX = PB.trajectory_builder.dls_inner_max;
cfg.DLS_LINESEARCH_MAX = PB.trajectory_builder.dls_linesearch_max;
cfg.DLS_POSTURE_GAIN = PB.trajectory_builder.dls_posture_gain;
cfg.TRAJ_MAX_TF_S = PB.trajectory_builder.max_trajectory_duration_s;
cfg.TRAJ_MAX_STORED_SAMPLES = PB.trajectory_builder.max_trajectory_samples;
cfg.TRAJECTORY_BUILDER = PB.trajectory_builder;

%% Warm-start / Joint planner
if isfield(PB,'plan') && isstruct(PB.plan)
    cfg.PLAN = PB.plan;
end
if isfield(PB,'warmstart') && isstruct(PB.warmstart)
    cfg.WARMSTART = PB.warmstart;
end
cfg.ACTUAL_GUARD = PB.actual_guard;

cfg.DIAGNOSTICS = PB.diagnostics;

cfg.METHODS = PB.methods;

cfg.METHOD_GUARDS = PB.method_guards;
method_names = fieldnames(cfg.METHODS);
for i_method = 1:numel(method_names)
    method_name = method_names{i_method};
    method_cfg = cfg.METHODS.(method_name);
    if isfield(method_cfg,'bus_state_enable')
        state_flag=method_cfg.bus_state_enable;

    end

    if isfield(cfg.METHOD_GUARDS,method_name)

    end
end
method_guard_names = fieldnames(cfg.METHOD_GUARDS);
for i_guard = 1:numel(method_guard_names)
    guard_i = cfg.METHOD_GUARDS.(method_guard_names{i_guard});

end

cfg.EXECUTION_HORIZON = PB.execution_horizon;
cfg.REVISION_R2 = PB.revision;
cfg.PTP_R2 = PB.ptp_r2;
cfg.MPC_R2 = PB.mpc_r2;
cfg.JOINT = PB.joint;

%% Keep training-side fields expected by existing scripts
cfg.ALPHA_BUDGET = PB.alpha_budget;
cfg.TRAIN_TRF    = PB.train_trf;
% This cfg feeds online simulation, while the trainer reads PB.train_trf directly.
if isfield(PB,'inference') && isfield(PB.inference,'executionEnvironment')
    cfg.TRAIN_TRF.executionEnvironment = PB.inference.executionEnvironment;
end
if isfield(PB,'training') && isstruct(PB.training)
    cfg.TRAINING = PB.training;
end
if isfield(PB,'theta_train') && isstruct(PB.theta_train)
    cfg.THETA_TRAIN = PB.theta_train;
end

cfg.LEARNED_MODELS = PB.learned_models;
if isfield(PB,'paper_eval') && isstruct(PB.paper_eval)
    cfg.PAPER_EVAL = PB.paper_eval;
end
cfg.PREFLIGHT = PB.preflight;
cfg.PAPER = PB.paper;
cfg.SMOKE = PB.smoke;
end

function robot = resolve_robot(PB,robot_params)
robot = [];
if isfield(PB,'robot') && isfield(PB.robot,'object') && ~isempty(PB.robot.object)
    verified = isfield(PB.robot,'object_params_verified') && ...
        ~isempty(PB.robot.object_params_verified) && ...
        logical(PB.robot.object_params_verified(1));

    robot = PB.robot.object;
    return;
end
if isfield(PB,'robot') && isfield(PB.robot,'builder') && ~isempty(PB.robot.builder)
    if ~isempty(robot_params)
        robot = feval(PB.robot.builder,robot_params);
    else
        robot = feval(PB.robot.builder);
    end
    return;
end
try
    if evalin('base','exist(''robot'',''var'')')
        robot = evalin('base','robot');
        return;
    end
catch
end
error('PB.robot 未提供 object 或 builder，且工作区亦无 robot。');
end

function params = resolve_robot_params(PB)
params = [];
if ~isfield(PB,'robot') || ~isstruct(PB.robot)
    return;
end

has_overrides = isfield(PB.robot,'param_overrides') && ...
    isstruct(PB.robot.param_overrides);
has_params = isfield(PB.robot,'params') && ~isempty(PB.robot.params);
if has_params

end

if has_overrides
    params_expected = spinn3d_roarm_m2s_nominal_params(PB.robot.param_overrides);
    if has_params && ~isequaln(PB.robot.params,params_expected)
        error('spinn3d_params_to_cfg:StaleRobotDerivedParams', ...
            ['PB.robot.params no longer matches PB.robot.param_overrides. ', ...
             'Do not edit derived fields directly; edit param_overrides and ', ...
             'regenerate the parameter block.']);
    end
    params = params_expected;
elseif has_params
    params = PB.robot.params;
end
end

function [qmin,qmax] = robot_joint_limits_local(robot,nJ)
qmin = -inf(1,nJ);
qmax = inf(1,nJ);
jj = 0;
for ib = 1:numel(robot.Bodies)
    body = robot.Bodies{ib};
    if strcmpi(char(body.Joint.Type),'fixed')
        continue;
    end
    jj = jj + 1;
    if jj > nJ
        break;
    end
    p = double(body.Joint.PositionLimits(:).');
    if numel(p)==2 && all(isfinite(p))
        qmin(jj) = min(p);
        qmax(jj) = max(p);
    end
end
end
