function PB = spinn3d_params_block()
% ===== Central Param Block =====
% Revision-oriented parameter block.
%
% Naming convention used in the revised code:
%   - PB.time.Ts                : plant / servo simulation step [s].
%   - PB.policy.dt              : high-level governor / NN decision period [s].
%   - PB.power.P_GRID_BUDGET    : active DC-bus grid-input power budget used by
%                                 teacher generation, governors, and evaluation [W].
%   - PB.power.P_SUPPLY_MAX     : hardware supply rating V_nom * I_rating [W].
%   - PB.power.P_TOTAL_MAX      : legacy alias kept only for backward compatibility.
%
% The old manuscript/code mixed the names Pmax/Pbudget. In this code, 12 V and
% 5 A define a 60 W hardware rating, whereas P_GRID_BUDGET is the imposed
% controller/evaluation budget (11 W by default). Do not interpret 5 A as the
% active control budget unless P_GRID_BUDGET is explicitly set to 60 W.
%
% Angle unit for q0/qf is degree.

%% Robot
PB.robot.object  = [];                    % optional rigidBodyTree
PB.robot.builder = @spinn3d_build_robot;  % builder handle

%% Trajectory（demo/online 基准位形与名义限速）
PB.traj.q0_deg = [-45;  50; -45;  -20];
PB.traj.qf_deg = [  20; -35; -110;  50];
% Default to a Cartesian position path followed by null-space regularized IK.
% This keeps the representative end-effector trajectory interpretable while
% preserving the paper's fixed-path/time-scaling formulation q=q_g(s).  The
% joint-space S-curve generator remains available with PB.traj.mode='joint_scurve'.
PB.traj.mode   = 'cartesian_ik_scurve';
PB.traj.shape  = 'quintic_smootherstep';
% Cartesian reference path + null-space IK keeps the end-effector trajectory
% visually/geometrically meaningful while the timing is still governed by
% alpha(t).  Use 'joint_scurve' only for pure joint-space path tests.
PB.traj.cart_lift_z      = 0.06;
PB.traj.ik_null_weight   = 0.12;
PB.traj.ik_damping       = 8e-3;
PB.traj.ik_max_iter      = 15;
PB.traj.ik_tol_m         = 1e-4;
PB.traj.ik_max_step_rad  = 0.08;
PB.traj.vmax   = 60;                      % [rad/s], path-generator nominal value
PB.traj.amax   = 120;                     % [rad/s^2]

%% Sampling / Time
PB.sample.dt = 0.01;                      % geometric sampling / offline prescan step
PB.time.Ts   = 1e-3;                      % plant + inner-loop simulation step

%% Policy / inference cadence
% The simulation and inner CTC are still integrated at 1 ms. The expensive
% high-level operation (NN inference / teacher search / planning decision) is
% updated at PB.policy.dt and held by zero-order hold between policy updates.
% This is the cadence that should be reported for policy inference, while 1 ms
% should be reported as the servo/digital-twin integration step.
PB.policy.dt   = 0.01;                    % 10 ms default; use 0.02 for 20 ms tests
PB.policy.hold = 'zoh';
PB.policy.update_every_n = max(1, round(PB.policy.dt / PB.time.Ts));

%% DC-bus power budget and hardware rating
PB.power.V_BUS_NOMINAL = 12.0;            % nominal DC-bus voltage [V]
PB.power.I_SUPPLY_RATING = 5.0;           % hardware supply current rating [A]
PB.power.P_SUPPLY_MAX = PB.power.V_BUS_NOMINAL * PB.power.I_SUPPLY_RATING; % 60 W

PB.power.P_GRID_BUDGET = 13;        % 6.5-s balanced retrain target: imposed sustained grid-input budget [W]; still far below the 60 W hardware rating
PB.power.P_GRID_PEAK_MAX = PB.power.P_SUPPLY_MAX; % raw safety/rating cap [W]

% Legacy alias. Existing functions/scripts may still read P_TOTAL_MAX; new
% code should use P_GRID_BUDGET. Keep both equal unless intentionally testing
% old scripts.
PB.power.P_TOTAL_MAX = PB.power.P_GRID_BUDGET;

%% Alpha / governor
PB.alpha.alpha_min   = 0.08;      % paper-balanced floor; avoids excessively slow floor labels once P_GRID_BUDGET is feasible
PB.alpha.itmax       = 12;

%% Limits（CAPS）
% Use values consistent with the manuscript table unless deliberately testing a
% heavier digital twin. The 60 W hardware rating is now handled by PB.power, not
% by unrealistically large torque limits.
PB.caps.tau_max    = [2.94; 5.88; 2.94; 2.94];
PB.caps.qd_max     = 4.19*ones(4,1);       % 40 rpm no-load speed upper bound [rad/s]
PB.caps.qdd_max    = 12.0*ones(4,1);
PB.caps.P_axis_max = inf(4,1);
PB.caps.countRegen = false;

%% Controller gains
PB.control.Kp = 40;
PB.control.Kd = 8;

%% Stop（demo/online）
PB.stop.ee_tol      = 0.03;               % [m]
PB.stop.dq_tol_rad  = 0.05;               % [rad/s]
PB.stop.stable_time = 0.10;               % [s]

%% Friction（训练/在线统一口径）
PB.fric.B       = [0.05 0.05 0.03 0.02];  % viscous [N*m*s/rad]
PB.fric.Fc      = [0.4  0.3  0.2  0.1 ];  % smoothed Coulomb [N*m]
PB.fric.vel_eps = 1e-3;

%% Measurement / torque noise（demo/online）
PB.noise.sigma_q  = 1e-4;                 % [rad]
PB.noise.sigma_dq = 1e-4;                 % [rad/s]
PB.noise.tau_std  = 1e-4;                 % [N*m]

%% Paths（使用 char + fullfile，避免 "…" 与 + ）
PB.paths.root  = 'data_alpha';
PB.paths.model_alpha = fullfile(PB.paths.root, 'model_alpha_gov_trf.mat');
PB.paths.model_alpha_mlp = fullfile(PB.paths.root, 'model_alpha_gov_mlp.mat');
PB.paths.dataset_alpha_gov_seq_master = fullfile(PB.paths.root, 'spinn3d_alpha_gov_seq_dataset_master.mat');

%% ==== Train Settings: Transformer look-ahead ====
PB.train_trf.enable = true;

% K=81 with H=0.8 gives geometric preview spacing close to 0.01 when Tf=1.
% Existing trained models with K=100 remain loadable; retraining is recommended
% after changing K/policy_dt.
PB.train_trf.K = 81;
PB.train_trf.H = 1;
PB.train_trf.policy_dt = PB.policy.dt;

PB.train_trf.epochs = 80;        % 6.5-s balanced retrain: give the surrogate more time to match Feasible-RG labels
PB.train_trf.batch  = 512;
PB.train_trf.lr     = 8e-4;

PB.train_trf.lambda_over  = 220;   % 6.5-s balanced retrain: still safety-biased, but less under-speeding than 350
PB.train_trf.lambda_under = 120;   % 6.5-s balanced retrain: penalize systematic under-use so TRF follows Feasible-RG speed better
PB.train_trf.lambda_pinn   = 0.0;   % disabled until PINN matches the revised electrical bus model
PB.train_trf.pinn_token_decay = 0.35;
PB.train_trf.lambda_speed  = 0.0;   % do not reward large alpha beyond the max-feasible teacher
PB.train_trf.lambda_end    = 0.1;   % 6.5-s balanced retrain: reduce end-phase over-slowing; stop condition still enforces settling
PB.train_trf.end_phase     = 0.85;
PB.train_trf.end_width     = 0.05;

PB.train_trf.d_model  = 192;
PB.train_trf.n_heads  = 6;
PB.train_trf.ffn_dim  = 512;
PB.train_trf.n_layers = 3;
PB.train_trf.dropout  = 0.03;
PB.train_trf.executionEnvironment = 'auto';

%% Payload（demo 固定值 + 训练采样范围）
PB.payload.mode = 'overwrite_last';
PB.payload.demo.mass    = 0.40;                  % [kg]
PB.payload.demo.com     = [0 0 0.05];            % [m]
PB.payload.demo.inertia = [0.0008 0.0008 0.0006 0 0 0];
PB.payload.demo.about   = 'com';
PB.payload.demo.mode    = 'replace';
PB.payload.demo.mount.xyz     = [0 0 0];
PB.payload.demo.mount.rpy_deg = [0 0 0];

PB.payload.range.mass     = [0.20 0.50];
PB.payload.range.com      = [-0.03 0.03; -0.03 0.03; 0.00 0.10];
PB.payload.range.box_dims = [0.03 0.06; 0.03 0.06; 0.03 0.08];

%% ===== Alpha 数据集预算 =====
PB.alpha_budget.n_traj      = 360;  % 6.5-s balanced retrain: more trajectories to reduce TRF/MLP surrogate bias
PB.alpha_budget.chunk_traj  = 20;
PB.alpha_budget.parallel_dataset = true;  % use parfor over trajectories when a pool is active
% Avoid training on budgets below unavoidable auxiliary/frictional draw at
% alpha_min.  A narrow range around the active budget improves budget tracking and keeps
% sensitivity tests separate from the main model training.
PB.alpha_budget.pbud_mode   = 'sample';
p_range = 0.30; % 6.5-s balanced retrain: train tightly around the 13 W main budget; sensitivity sweeps are separate
PB.alpha_budget.pbud_range  = [PB.power.P_GRID_BUDGET-p_range, PB.power.P_GRID_BUDGET+p_range];
PB.alpha_budget.pbud_range  = max(PB.alpha_budget.pbud_range, 1e-6);

% Backward-compatible aliases used by older scripts.
PB.alpha_budget.pmax_mode  = PB.alpha_budget.pbud_mode;
PB.alpha_budget.pmax_range = PB.alpha_budget.pbud_range;

% --- 用 demo 轨迹自动生成采样范围 ---
q0_deg = PB.traj.q0_deg(:);
qf_deg = PB.traj.qf_deg(:);
margin_q0   = 10;
margin_qref = 10;
PB.alpha_budget.q0_range_deg   = [q0_deg - margin_q0,   q0_deg + margin_q0  ];
PB.alpha_budget.qref_range_deg = [qf_deg - margin_qref, qf_deg + margin_qref];

PB.alpha_budget.dls_lambda  = 5e-3;
PB.alpha_budget.dls_w_ori   = 0.02;
PB.alpha_budget.bez_lift_z  = 0.04;
PB.alpha_budget.bez_gamma   = 1/3;
PB.alpha_budget.ns_min      = 200;

%% Governor / alpha regulator（严格必填）
PB.alpha_reg.beta     = 0.18;    % modest smoothing to avoid alpha-dot induced power spikes
PB.alpha_reg.a_dot_up = 12.0;    % still fast, but less aggressive than fast13
PB.alpha_reg.a_dot_dn = 45.0;    % avoid slamming alpha to the floor and exciting CTC transients
PB.alpha_reg.kinematic_cap_before_filter = true;

%% Budget-utilization feedback around the learned command
% This deterministic scalar PRG post-processor makes TRF/MLP use the available
% active input-power budget.  From v5 onward we use a single academic/evaluation
% LPF for the bus-power signal, not separate "fast" and "slow" operating
% states.  The filtered signal is
%     Pbar[k] = lambda Pbar[k-1] + (1-lambda) Pgrid[k],
%     lambda = exp(-Ts/tau_power).
% The default tau_power=0.05 s is a standard causal first-order low-pass
% filter, suitable for evaluating and feeding back the overall run.
PB.budget_ctrl.enable       = true;
PB.budget_ctrl.apply_to_nn   = true;     % applies to TRF and MLP modes
PB.budget_ctrl.apply_to_feasible = true;  % Feasible-RG uses the same low-cost budget post-processor
PB.budget_ctrl.apply_to_profile  = false; % keep PTP/DP baselines unmodified unless explicitly enabled
PB.budget_ctrl.apply_to_mpc      = true;  % MPC/RG must also pass through the same deterministic budget cap
PB.budget_ctrl.tau_power    = 0.05;     % [s] single first-order LPF for filtered bus power
PB.budget_ctrl.tau_slow     = PB.budget_ctrl.tau_power; % compatibility alias only
PB.budget_ctrl.tau_fast     = PB.budget_ctrl.tau_power; % compatibility alias only
PB.budget_ctrl.target_ratio = 0.960;     % 6.5-s balanced: use more budget headroom after retraining the surrogate
PB.budget_ctrl.soft_high_ratio = 0.995;  % 6.5-s balanced: trim just below the 5% evaluation band
PB.budget_ctrl.fast_high_ratio = 0.995;  % same LPF; kept only for old code paths
PB.budget_ctrl.recover_low_ratio = 0.85;% 6.5-s balanced recovery threshold
PB.budget_ctrl.gamma        = 2.0;      % empirical P ~ alpha^gamma scaling in the local trim law
PB.budget_ctrl.cut_gain     = 1.2;      % moderate trim; predictive grid handles non-monotone cases
PB.budget_ctrl.boost_gain   = 1.15;
PB.budget_ctrl.max_boost_per_policy = 0.014; % 6.5-s balanced: moderate recovery; avoid rebound spikes
PB.budget_ctrl.max_cut_per_policy   = 0.22;  % avoid collapsing alpha for long intervals
PB.budget_ctrl.enable_boost = true;      % important: otherwise NN stays at a very low command after early trim
PB.budget_ctrl.init_lp_ratio = 0.90;    % do not start artificially close to the red line
PB.budget_ctrl.servo_rate_feedback = true;  % re-run budget trim every servo step, not only every policy update
PB.budget_ctrl.raw_high_ratio = 1.30;       % 6.5-s balanced raw-demand early warning
PB.budget_ctrl.hard_high_ratio = 1.040;      % 6.5-s balanced hard LPF trim below the 5% tolerance

% Predictive one-step hard cap for budget compliance.  This caps the command
% using the same CTC + electrical bus model before the LPF budget metric crosses
% the red line.  Keep this enabled for the revised paper runs; switch it off
% only for ablations that intentionally test the learned command without a
% safety governor.
PB.budget_ctrl.predictive_cap_enable = true;
PB.budget_ctrl.predictive_lpf_ratio  = 1.025;  % 6.5-s balanced one-step LPF guard
PB.budget_ctrl.predictive_raw_ratio  = 1.30;  % 6.5-s balanced raw burst guard
PB.budget_ctrl.predictive_iters      = 14;
PB.budget_ctrl.predictive_grid_N     = 21;    % grid scan avoids false monotonic assumptions
PB.budget_ctrl.floor_fallback_mode   = 'min_power_grid'; % if floor is infeasible, choose minimum-predicted-power alpha, not always alpha_min
PB.budget_ctrl.floor_fallback_raw_weight = 0.50;
PB.budget_ctrl.floor_fallback_move_weight = 0.03;
PB.budget_ctrl.floor_fallback_speed_weight = 0.01;
PB.budget_ctrl.predictive_use_dq_next = true;

% TRF feed-forward commands are mildly scaled; the LPF budget servo can still recover speed. Previous
% validation MAE.  This calibrated shrinkage is applied only to Transformer
% feed-forward commands; the LPF budget servo can still boost when power is
% genuinely under-used.  MLP/Feasible are left unscaled.
PB.budget_ctrl.trf_command_scale = 0.96;  % 6.5-s balanced retrain target
PB.budget_ctrl.mlp_command_scale = 0.93;
PB.budget_ctrl.feasible_command_scale = 0.98;

%% BUS model defaults
PB.bus.P_grid_budget   = PB.power.P_GRID_BUDGET;
PB.bus.P_total_max     = PB.bus.P_grid_budget;       % legacy alias
PB.bus.P_grid_peak_max = PB.power.P_GRID_PEAK_MAX;
PB.bus.V_nom           = PB.power.V_BUS_NOMINAL;
PB.bus.I_rating        = PB.power.I_SUPPLY_RATING;
PB.bus.P_supply_rating = PB.power.P_SUPPLY_MAX;
PB.bus.I_budget        = PB.bus.P_grid_budget / PB.bus.V_nom;

% Electrical common-bus model used by revised telemetry/evaluation.
% Parameters are representative defaults and should be calibrated/validated
% against measured Vdc/Igrid telemetry for the final manuscript.
% Tuned to the RoArm-M2-S / ST3215 class in a source-bounded way.
% The active budget is PB.power.P_GRID_BUDGET, whereas the 12 V/5 A adapter gives a
% 60 W hardware cap.  The compact smart-servo stack has non-negligible
% no-load/drive electronics current; adding this term makes the uncontrolled
% baseline draw about 20--30 W in the representative trial, as expected for a
% 5-servo 12 V arm, without allowing delivered source power above 60 W.
PB.bus.eta_share   = 0.60;       % compact smart servos: limited useful DC-bus regen sharing
PB.bus.eta_mot     = 0.75;       % effective electrical-to-output efficiency (swept in sensitivity)
PB.bus.eta_regen   = 0.35;       % conservative regenerative availability
PB.bus.R_batt      = 0.10;       % [ohm], adapter/wiring/internal source resistance
PB.bus.Cdc         = 4700e-6;    % [F], representative local bus capacitance
PB.bus.P_brk_peak  = 20.0;
PB.bus.P_dump_max  = PB.bus.P_brk_peak;  % legacy alias
PB.bus.V_min       = 10.5;
PB.bus.V_max       = 13.2;
PB.bus.V_chop      = 13.0;
% IMPORTANT: tau in the simulation is the *output-shaft* joint torque.
% Therefore Kt and R_phase below are output-equivalent parameters.  Do not
% use raw motor-side torque constants together with gearbox-output torque,
% otherwise the inferred current tau/Kt becomes much too large and the
% electrical loss term R*I^2 can dominate the bus power even when mechanical
% joint power tau*dq is small.
%
% The default Kt is tied to the torque-limit / supply-rating convention in
% Table 2: each joint reaches its listed output torque limit at about the
% 5-A hardware current rating.  The effective resistance is chosen as a
% compact supply-side/output-equivalent loss parameter; it should be replaced
% by calibrated values when Vdc/Igrid telemetry is available.
PB.bus.P_inv0      = 1.2;               % [W], shared controller/drive electronics allowance
PB.bus.k_inv       = 0.006;             % [W/A^2], wiring/driver loss proportional to output-equivalent current^2
PB.bus.Kt          = (PB.caps.tau_max(:).' ./ max(PB.power.I_SUPPLY_RATING,eps)); % [N*m/A], output-equivalent
PB.bus.R_phase     = [0.12 0.18 0.12 0.10]; % [ohm], output-equivalent copper/loss fit

% Smart-servo auxiliary/no-load current model.  ST3215 public specifications
% list about 200 mA no-load current and about 30 mA standby current per servo;
% RoArm-M2-S uses five servos with a dual-drive shoulder.  We add this
% supply-side term during motion instead of treating Pgrid as purely tau*dq.
PB.bus.enable_servo_aux_loss = true;
PB.bus.n_servo_axis          = [1 2 1 1];
PB.bus.I_servo_standby_A     = 0.03;      % [A/servo]
PB.bus.I_servo_no_load_A     = 0.20;      % [A/servo] at no-load rated speed
PB.bus.omega_no_load_servo_rad_s = (pi/3) / 0.222; % 0.222 s / 60 deg = 4.72 rad/s
PB.bus.servo_aux_speed_exp   = 0.50;      % sqrt speed scaling gives realistic mid-speed draw
PB.bus.P_control_board_W     = 0.6;       % [W], ESP32/control board/shared electronics

% Physical source/current limiter.  Delivered Pgrid is bounded by the 12 V/5 A
% hardware supply; the pre-limit electrical demand is logged as Pgrid_request.
PB.bus.enable_supply_current_limit = true;
PB.bus.I_source_max    = PB.power.I_SUPPLY_RATING;
PB.bus.P_source_max_W  = PB.power.P_SUPPLY_MAX;

% Stateful regenerative-energy/bus telemetry used by revised evaluation.
% Do not pre-charge an artificial large energy buffer; otherwise the early
% grid draw is made too low.  The physical DC-link effect is represented by Cdc/Vdc.
PB.bus.enable_state    = false;
PB.bus.Ebuf_min_J      = 0.0;
PB.bus.Ebuf_max_J      = 0.0;
PB.bus.Ebuf_init_J     = 0.0;
PB.bus.Ereuse_init_J   = 0.0;
PB.bus.Pbuf_chg_max_W  = 0.0;
PB.bus.Pbuf_dis_max_W  = 0.0;
PB.bus.eta_buf_chg     = 0.95;
PB.bus.eta_buf_dis     = 0.95;
PB.bus.Tamb_C          = 25.0;
PB.bus.Tbrk_init_C     = 25.0;
PB.bus.Tbrk_max_C      = 120.0;
PB.bus.Rth_brk_C_per_W = 8.0;
PB.bus.Cth_brk_J_per_C = 40.0;

% Supply-side evaluation filter. This is not the alpha-governor filter.
% v5 uses one causal first-order LPF for all scalar budget metrics.
PB.eval.power_eval_mode   = 'lpf';
PB.eval.power_tau_s       = PB.budget_ctrl.tau_power;
PB.eval.power_eval_tau_s  = PB.eval.power_tau_s; % compatibility alias
PB.eval.power_slow_tau_s  = PB.eval.power_tau_s; % compatibility alias only
PB.eval.power_fast_tau_s  = PB.eval.power_tau_s; % compatibility alias only
PB.eval.power_lpf_fc_hz   = NaN;        % legacy cutoff-frequency option only
PB.eval.Pbudget_margin_frac = 0.05;     % rho
PB.eval.rho = PB.eval.Pbudget_margin_frac;

%% Debug / optional diagnostics
PB.debug.log_afeas = false;  % keep false for pure TRF latency/fairness
end
