function ctl = spinn3d_controller_ctcgov(robot, traj, gains, limits, opts)
% Computed-torque tracking with progress-rate regulation.
% Policy commands are held between updates and passed through the shared
% physical-admission layer before advancing the reference path.

try, robot.DataFormat='row'; end
nJ = local_num_joints(robot);

Ts  = double(opts.Ts);
Kp  = vec_row(reqf(gains,'Kp'), nJ);
Kd  = vec_row(reqf(gains,'Kd'), nJ);

% ===== NN α / feasible α =====

nn = opts.nnAlpha;

nn.enable = logical(nn.enable);

nn.mode = lower(strtrim(char(nn.mode)));

nn.alpha_floor = double(nn.alpha_floor);
nn.alpha_max = double(nn.alpha_max);
nn.itmax = round(double(nn.itmax));

nn.feasibility_use_alpha_dot = ...
    logical(nn.feasibility_use_alpha_dot);

% ===== High-level policy / inference cadence =====
% Only the high-level decision a_des is refreshed at policy_dt.
% The plant, CTC, BUS integration, torque clamp, governor smoothing,
% and actual safety guard still run every Ts.

policy = opts.policy;
policy_contract = spinn3d_control_schedule(Ts,policy,[],[]);
policy_dt = policy_contract.policy_dt;
policy_stride = policy_contract.policy_stride;
policy_hold_mode = policy_contract.policy_hold;

if nn.enable && strcmp(nn.mode,'nn')

    

    validate_alpha_inference_cadence_local(nn.model.preproc,policy_dt);

    signed_gains=nn.model.preproc.control_fingerprint;

    

    execSpec = getfield_def(nn,'executionEnvironment', ...
               getfield_def(nn,'ExecutionEnvironment', ...
               getfield_def(nn.model.preproc,'inferenceExecutionEnvironment', ...
               getfield_def(nn.model.preproc,'executionEnvironment','auto'))));
    inferVerbose = logical(getfield_def(nn,'gpuVerbose',false));
    [nn.useGPU, nn.executionEnvironment] = spinn3d_gpu_execution_env(execSpec, inferVerbose, 'alpha-ctrl');
elseif nn.enable
    % feasible / planned / actual_feasible / mpc_grid: no dlnetwork required
    if strcmp(nn.mode,'planned')

        
    end
    if strcmp(nn.mode,'mpc_grid')

    end
end

req_fields = {'A_DOT_UP','A_DOT_DN','BETA'};
areg = struct();
for i = 1:numel(req_fields)
    k = req_fields{i};

    areg.(k) = double(opts.alphaReg.(k));
end

if isfield(opts.alphaReg,'N_BISECT')
    areg.N_BISECT = double(opts.alphaReg.N_BISECT);
end
if isfield(opts.alphaReg,'GAMMA')
    areg.GAMMA = double(opts.alphaReg.GAMMA);
end

bus  = spinn3d_bus_defaults(opts.bus);
fric = opts.fric;
caps = normalize_caps(limits, nJ);
[caps.q_min, caps.q_max] = robot_joint_limits_local(robot, nJ);
Pcap = double(limits.P_total_max);
if isfield(opts,'executionAudit'), caps.execution_audit=opts.executionAudit; end

% power-off NoGuard control. The only search/objective switch is power_enabled.
use_state_teacher = nn.enable && strcmp(nn.mode,'feasible');
use_noguard_teacher = use_state_teacher && isinf(Pcap) && Pcap>0;
if use_state_teacher

    

    teacher_clock=opts.alphaStateTeacher;

end

% Optional online soft-Pavg governor.  This layer is deliberately separate
% from the 1-kHz actual guard: it may reshape a planned 100-Hz command for
% performance, but it never changes physical pass/fail, enters safe-hold, or
% authorizes a plant step.  Formal Joint callers pass the complete central
% cfg.SOFT_POWER contract plus the paired NoGuard arrival-time reference.
softPower = soft_power_contract_local(opts,Pcap,policy_dt,nn.mode);

% Matched physical initialization: use the source power required to hold the
% payload-injected robot at q0.  Starting the causal filters from zero grants
% every method an artificial P_avg_window_s-long power credit.
[q_hold0, ~, ~] = traj.eval(0);
power_init = spinn3d_static_hold_power_init( ...
    robot,row(q_hold0),fric,bus,Pcap,Ts);
if isfield(opts,'initial_bus_measurement') && ...
        isstruct(opts.initial_bus_measurement) && ...
        strcmp(bus.model_scope,'dynamic_capacitor_common_dc_v1')
    Z=opts.initial_bus_measurement;

    nominal_dc=spinn3d_dc_state_for_model(Z.bus_state,bus);
    bus=spinn3d_bus_with_state(bus,nominal_dc,Ts);
    power_init.bus_state0=nominal_dc;
    power_init.Pgrid_static_W=double(Z.Pgrid_static_W);
    measured_pavg=double(getfield_def(Z,'Pgrid_avg_initial_W',Z.Pgrid_static_W));

    power_init.Pgrid_lp0_W=measured_pavg;
    power_init.Pgrid_avg0_W=measured_pavg;
    power_init.initial_dc_measurement_used=true;
end
tau_hold_ratio = max(abs(row(power_init.tau_static_Nm)) ./ ...
    max(abs(row(caps.tau_max)),eps));
power_init.tau_limit_ratio = tau_hold_ratio;
power_init.static_hold_feasible = logical( ...
    power_init.static_source_feasible && isfinite(tau_hold_ratio) && ...
    tau_hold_ratio <= 1 + 1e-9);
if ~power_init.static_hold_feasible
    error('spinn3d_controller_ctcgov:InitialStaticHoldInfeasible', ...
        ['The q0 static hold is infeasible before motion: Pgrid=%.6g W, ', ...
         'Pavg KPI target=%.6g W, peak cap=%.6g W, tau ratio=%.6g, motor_ok=%d.'], ...
        power_init.Pgrid_static_W,power_init.Pavg_cap_W, ...
        power_init.Pgrid_peak_cap_W,tau_hold_ratio, ...
        double(power_init.motor_envelope_initial_ok));
end

use_bus_state = isfield(bus,'enable_state') && ~isempty(bus.enable_state) && logical(bus.enable_state);
st_bus0 = power_init.bus_state0;
alpha_nominal = min(1.0,nn.alpha_max);
if nn.enable
    alpha_initial = nn.alpha_floor;
else
    % An explicitly disabled alpha controller means unscaled path execution.
    % The independently configured actual guard may still intervene, and the
    % explicit method ceiling remains authoritative if it is below nominal.
    alpha_initial = alpha_nominal;
end
state = struct('a_prev',      alpha_initial, ...
               'Pgrid_lp',    power_init.Pgrid_lp0_W, ...   % previous observed source-power LP
               'Pgrid_avg_lp', power_init.Pgrid_avg0_W, ... % causal filtered-power KPI state
               'gate_active', false, ...        
               'bus',         st_bus0, ...
               'use_bus_state', use_bus_state, ...
               'power_init',  power_init, ...
               'policy_dt', policy_dt, ...
               'policy_stride', policy_stride, ...
               'policy_hold_mode', policy_hold_mode, ...
               'policy_next_t', 0.0, ...
               'policy_hold_valid', false, ...
               'policy_a0_hold', alpha_initial, ...
               'policy_a_star_hold', NaN, ...
               'policy_a_star_gate_hold', NaN, ...
               'policy_rho_hold', NaN, ...
               'policy_mode_hold', 'init', ...
               'policy_gate_active_hold', false, ...
               'policy_gate_ratio_hold', NaN, ...
               'soft_power_diag_hold', soft_power_diag_default_local( ...
                    alpha_initial,softPower), ...
               'mpc_diag_hold',struct('active',false), ...
               'noguard_policy_diag_hold',struct(), ...
               'recovery_latched', false, ...
               'recovery_new_entry_pending', false, ...
               'recovery_release_trial_pending', false, ...
               'recovery_hold_steps', 0, ...
               'recovery_clear_steps', 0, ...
               'recovery_entry_reason', 'none', ...
               'recovery_entered_last_observe', false, ...
               'recovery_released_last_observe', false, ...
               'post_noise_feedback_valid', false, ...
               'policy_alpha_ceiling', nn.alpha_max, ...
               'last_step_hold_ok', true, ...
               'last_step_policy_tick', false, ...
               'last_step_t', NaN);

gate = nn.gate;

% ===== Optional actual-step overlay guard =====

actualGuard = opts.actualGuard;

runtime_contract = spinn3d_control_schedule( ...
    Ts,policy,gate,actualGuard,nn.mode);
if use_noguard_teacher

end
if double(actualGuard.alpha_floor) > double(actualGuard.search.alpha_max)
    error('spinn3d:ActualGuardSearchValue', ...
        'opts.actualGuard.alpha_floor must not exceed search.alpha_max.');
end

% alpha_feasible receives a method-local copy so a 1.0 baseline cannot

caps.alpha_feasible.alpha_max = nn.alpha_max;

position_hold_grid = double(actualGuard.safe_hold.position_gain_scale_grid(:).');
damping_hold_grid = double(actualGuard.safe_hold.damping_gain_scale_grid(:).');
safe_hold_refine_iterations = ...
    double(actualGuard.safe_hold.gain_refine_iterations);

if nn.enable && strcmp(nn.mode,'planned')
    if isfield(nn.plan,'rho') && isfield(nn.plan,'a_des_seq')
        nn.plan.rho       = row(double(nn.plan.rho));
        nn.plan.a_des_seq = row(double(nn.plan.a_des_seq));
    elseif isfield(nn.plan,'rho_knots') && isfield(nn.plan,'a_knots')
        nn.plan.rho       = row(double(nn.plan.rho_knots));
        nn.plan.a_des_seq = row(double(nn.plan.a_knots));
    else
        error('planned 模式需要 nn.plan.rho + nn.plan.a_des_seq，或 nn.plan.rho_knots + nn.plan.a_knots');
    end

    if numel(nn.plan.rho) < 2
        nn.plan.rho       = [0 1];
        nn.plan.a_des_seq = [nn.plan.a_des_seq(1) nn.plan.a_des_seq(1)];
    end
    nn.plan.rho = min(1.0, max(0.0, nn.plan.rho));
    [nn.plan.rho, iu] = unique(nn.plan.rho, 'stable');
    nn.plan.a_des_seq = nn.plan.a_des_seq(iu);
    if nn.plan.rho(1) > 0
        nn.plan.rho       = [0, nn.plan.rho];
        nn.plan.a_des_seq = [nn.plan.a_des_seq(1), nn.plan.a_des_seq];
    end
    if nn.plan.rho(end) < 1
        nn.plan.rho       = [nn.plan.rho, 1];
        nn.plan.a_des_seq = [nn.plan.a_des_seq, nn.plan.a_des_seq(end)];
    end
    nn.plan.a_des_seq = min(nn.alpha_max, ...
        max(nn.alpha_floor, nn.plan.a_des_seq));
end

    function [tau_cmd, info] = step(t, q_meas, dq_meas, q_ref, dq_ref, ddq_ref, robot_in, s) %#ok<INUSD>

        step_wall_clock = tic;
        bus=spinn3d_bus_with_state(bus,state.bus,Ts);
        a_prev_step = state.a_prev;
        network_forward_s=NaN;

        a0      = 0;
        a       = 0;
        a_slew  = 0;
        a_filt  = 0;
        adot    = 0;
        guard   = struct();
        actual_guard = struct();
        soft_power_diag = state.soft_power_diag_hold;
        mpc_diag = struct('active',false);
        noguard_diag = state.noguard_policy_diag_hold;
        noguard_policy_tick = false;
        dq_cmd  = row(dq_ref);
        ddq_cmd = row(ddq_ref);
        mode_str = '';

gate_hit    = false;
gate_ratio  = NaN;
a_star_gate = NaN;
Pgrid_est   = NaN;

        q_meas = row(q_meas); dq_meas = row(dq_meas);
        q_ref  = row(q_ref);  dq_ref  = row(dq_ref);  ddq_ref = row(ddq_ref);

        % High-level policy tick: refresh a_des only every policy_dt.
        % Between ticks, reuse the previous policy output (zero-order hold).
        tol_policy = max(1e-12, 1e-9 * max(1.0, abs(t)));
        if state.policy_dt <= Ts * (1 + 1e-9)
            policy_tick = true;
        else
            policy_tick = (~state.policy_hold_valid) || (t + tol_policy >= state.policy_next_t);
        end
        policy_hold = (~policy_tick) && state.policy_hold_valid;
        if policy_tick
            state.policy_alpha_ceiling = nn.alpha_max;
            if ~isfinite(state.policy_next_t) || state.policy_next_t < t - tol_policy
                state.policy_next_t = t;
            end
            while state.policy_next_t <= t + tol_policy
                state.policy_next_t = state.policy_next_t + state.policy_dt;
            end
        end

        % reference-side BUS guard. The explicit bus state is kept here for
        % logging/observation and for the optional actual_feasible mode only.
        caps_feas = caps;
        caps_feas.eta_share  = getfield_def(bus,'eta_share', getfield_def(caps,'eta_share',1.0));
        caps_feas.P_brk_peak = getfield_def(bus,'P_brk_peak', ...
                                  getfield_def(bus,'P_dump_max', getfield_def(caps,'P_brk_peak',inf)));
        caps_feas.bus = bus;
        caps_feas.bus_Ts = Ts;
        if state.use_bus_state, caps_feas.bus_state=state.bus; end
        caps_feas.Pgrid_avg_prev = state.Pgrid_avg_lp;
        % alpha_feasible is refreshed at the 100-Hz policy tick, but its
        % post-governor cap describes the NEXT plant sample.  Therefore its
        % alpha-dot horizon is Ts, not policy_dt.  This reference-side cap is
        % converted back into the desired-alpha domain below.  The actual-step
        % guard still checks every measured-state plant sample.
        caps_feas.alpha_prev = state.a_prev;
        caps_feas.alpha_Ts = Ts;
        caps_feas.use_alpha_dot = nn.feasibility_use_alpha_dot;
        caps_feas_actual = caps_feas;
        if state.use_bus_state
            caps_feas_actual.bus_state = state.bus;
        end

        if nn.enable && state.recovery_latched
            % Recovery owns zero progress / restart admission. Do not ask a
            % nominal policy search to jump from zero to its training floor
            % before the measured-state recovery latch has released.
            a0=clamp(state.policy_a0_hold,nn.alpha_floor,nn.alpha_max);
            [a,a_slew,a_filt,adot,guard]=alpha_guard_backtrack( ...
                a0,state.a_prev,Ts,q_ref,dq_ref,ddq_ref,robot, ...
                caps_feas,Pcap,fric,areg,bus,nn.alpha_max);
            mode_str='recovery_policy_hold';
        elseif ~nn.enable
            % Central method switch: execute the path without alpha scaling.
            % Safety recovery remains governed independently by actualGuard.
            a0 = alpha_nominal;
            a = alpha_nominal;
            a_slew = alpha_nominal;
            a_filt = alpha_nominal;
            adot = 0.0;
            mode_str = 'alpha_controller_disabled';
            state.policy_a0_hold = alpha_nominal;
            state.policy_a_star_hold = alpha_nominal;
            state.policy_rho_hold = min(1.0,s/max(traj.Tf,1e-12));
            state.policy_mode_hold = mode_str;
            state.policy_hold_valid = true;
        elseif strcmp(nn.mode,'planned')
            if policy_hold
                a0=state.policy_a0_hold;
                mode_str='planned_hold';
            else
                rho_plan=min(1,s/max(traj.Tf,1e-12));
                a0=clamp(plan_lookup(nn.plan,rho_plan),nn.alpha_floor,nn.alpha_max);
                state.policy_a0_hold=a0;
                state.policy_rho_hold=rho_plan;
                state.policy_mode_hold='planned';
                state.policy_hold_valid=true;
                mode_str='planned';
            end
            [a,a_slew,a_filt,adot,guard]=alpha_guard_backtrack( ...
                a0,state.a_prev,Ts,q_ref,dq_ref,ddq_ref,robot, ...
                caps_feas,Pcap,fric,areg,bus,nn.alpha_max);
            guard.direct_desired_replay=true;

        elseif strcmp(nn.mode,'feasible')
            % Current measured q/dq, causal DC state and current path only.
            % No elapsed time, future measured state or NG arrival anchor.
            if policy_hold
                a0 = clamp(double(state.policy_a0_hold),0,nn.alpha_max);
                a_star = state.policy_a_star_hold;
                mode_str = 'state_teacher_hold';
            else
                C=opts.alphaStateTeacher;
                C.caps=caps_feas;
                C.Kp=Kp; C.Kd=Kd;
                S=struct('q',q_meas,'dq',dq_meas,'s',s, ...
                    'a_prev',state.a_prev,'bus',state.bus,'Pavg',state.Pgrid_avg_lp);
                [a0,teacher_diag]=spinn3d_alpha_teacher_state('select',robot,traj,S,C);
                % Configured ceiling is telemetry, not admission of this value.
                a_star=nn.alpha_max;
                soft_power_diag=teacher_diag;
                state.soft_power_diag_hold=teacher_diag;
                if use_noguard_teacher
                    noguard_diag=teacher_diag;
                    noguard_policy_tick=true;
                    state.noguard_policy_diag_hold=teacher_diag;
                end
                state.policy_a0_hold=a0;
                state.policy_a_star_hold=a_star;
                state.policy_rho_hold=NaN;
                state.policy_mode_hold='state_teacher';
                state.policy_hold_valid=true;
                mode_str='state_teacher';
            end
            % Selection never grants execution. The same existing governor
            % and independent measured-state guard admit each 1-ms step.
            [a,a_slew,a_filt,adot,guard]=alpha_guard_backtrack( ...
                a0,state.a_prev,Ts,q_ref,dq_ref,ddq_ref,robot, ...
                caps_feas,Pcap,fric,areg,bus,nn.alpha_max);
            guard.a_star=a_star;

elseif strcmp(nn.mode,'mpc_grid')

    if policy_hold
        a0 = clamp(double(state.policy_a0_hold),nn.alpha_floor,nn.alpha_max);
        mode_str = 'mpc_grid_hold';
        mpc_diag = getfield_def(state,'mpc_diag_hold',struct('active',true));
    else
        mpc_state = struct('s',double(s),'a_prev',double(state.a_prev), ...
            'Pgrid_avg',double(state.Pgrid_avg_lp),'bus_state',state.bus, ...
            'q_meas',q_meas,'dq_meas',dq_meas);
        [a0,mpc_diag] = spinn3d_mpc_rg_grid_step_r2( ...
            mpc_state,traj,robot,caps,bus,fric,areg,policy_dt,nn.mpc);
        a0 = clamp(double(a0),nn.alpha_floor,nn.alpha_max);
        state.policy_a0_hold = a0;
        state.policy_a_star_hold = nn.alpha_max;
        state.policy_rho_hold = min(1.0,s/max(traj.Tf,1e-12));
        state.policy_mode_hold = 'mpc_grid';
        state.policy_hold_valid = true;
        state.mpc_diag_hold = mpc_diag;
        mode_str = 'mpc_grid';
    end
    [a, a_slew, a_filt, adot, guard] = alpha_guard_backtrack( ...
        a0, state.a_prev, Ts, q_ref, dq_ref, ddq_ref, robot, ...
        caps_feas, Pcap, fric, areg, bus, nn.alpha_max);
    guard.mpc = mpc_diag;

        elseif strcmp(nn.mode,'actual_feasible')

            
            a0 = nn.alpha_max;
            a = nn.alpha_max;
            a_slew = a;
            a_filt = a;
            adot = (a - state.a_prev)/max(Ts,1e-12);
            guard.direct_actual = true;
            mode_str = 'actual_feasible';
        else
            % ===== NN mode: NN/gating decision at policy_dt; ZOH between ticks =====
            if policy_hold
                a0 = clamp(double(state.policy_a0_hold), ...
                    nn.alpha_floor, nn.alpha_max);
                a_star_gate = state.policy_a_star_gate_hold;
                gate_hit = state.policy_gate_active_hold;
                gate_ratio = state.policy_gate_ratio_hold;
                if gate_hit && ...
                        strcmpi(strtrim(char(gate.fallback)),'feasible_cap') && ...
                        ~isfinite(a_star_gate)
                    error('spinn3d_controller_ctcgov:InvalidFeasibleHold', ...
                        'The NN-gate policy hold contains an invalid feasible-alpha cap.');
                end
                mode_str = state.policy_mode_hold;
                if isempty(mode_str) || strcmp(mode_str,'init')
                    mode_str = 'nn_hold';
                else
                    mode_str = [mode_str '_hold'];
                end
            else

if gate.enable && isfinite(Pcap) && Pcap>0
    gate_ratio = state.Pgrid_lp / max(Pcap, 1e-12);
    if state.gate_active
        if gate_ratio > gate.off_ratio
            state.gate_active = false;
        end
    else
        if gate_ratio < gate.on_ratio
            state.gate_active = true;
        end
    end
    gate_hit = state.gate_active;
else
    state.gate_active = false;
end

if gate_hit && gate.skip_forward
    if strcmpi(strtrim(char(gate.fallback)),'feasible_cap')
        [a_star_gate,a_star_gate_info] = alpha_feasible( ...
            q_ref, dq_ref, ddq_ref, nn.alpha_max, robot, ...
            caps_feas, Pcap, ...
            nn.alpha_floor, nn.itmax, fric);
        a_star_gate = require_feasible_alpha_local( ...
            a_star_gate,a_star_gate_info,'NN feasible-cap gate');
        a_star_gate = clamp(a_star_gate, nn.alpha_floor, nn.alpha_max);

        % Hold the policy request. Its physical first-step cap is refreshed
        % below at every Ts, rather than holding a 1-ms inverse for 10 ms.
        a0 = nn.alpha_max;
        mode_str = 'nn_gate_feasible';
    else
        a0 = clamp(double(gate.a_nom), nn.alpha_floor, nn.alpha_max);
        mode_str = 'nn_gate_nom';
    end
else

            

            % intentionally rejected during controller construction.

            kb    = logical(nn.model.preproc.keepBaseIdx(:).');

            Klook = round(double(nn.model.preproc.K));

                ds = double(nn.model.preproc.ds);

                % Cache actual-state M/h once at this measured policy tick.
                % Every token still computes errors relative to its own reference.
                observed=struct('q',q_meas,'dq',dq_meas,'fric',fric);
                [~,observed]=spinn3d_alpha_observed_state_features( ...
                    q_ref,dq_ref,state.a_prev,robot,observed);

                s_norm0 = min(1.0, s / max(traj.Tf, 1e-9));
                x1 = feature_vector_local(nn, q_ref, dq_ref, ddq_ref, robot, Pcap, caps, state.a_prev, areg, Ts, s_norm0, bus, state, observed);
                x1 = row(double(x1));

                Dtot  = numel(x1);
                Dtail = 6;
                Dbase = Dtot - Dtail;

                Xfull = zeros(Klook, Dtot);
                Xfull(1,:) = x1;
                for ti = 2:Klook
                    s_i = min(traj.Tf, s + (ti-1)*ds);
                    [qi, dqgi, ddqgi] = traj.eval(s_i);
                    s_norm_i = min(1.0, s_i / max(traj.Tf, 1e-9));
                    xi = feature_vector_local(nn, row(qi), row(dqgi), row(ddqgi), robot, Pcap, caps, state.a_prev, areg, Ts, s_norm_i, bus, state, observed);
                    Xfull(ti,:) = row(double(xi));
                end

                Xbase = Xfull(:,1:Dbase);
                Xtail = Xfull(:,Dbase+1:end);
                Xc    = [Xbase(:,kb), Xtail];
                Din   = size(Xc,2);

                mu  = row(double(nn.model.preproc.mu));
                sig = row(double(nn.model.preproc.sig));

                Z = (Xc - mu) ./ max(sig, 1e-12);   % K×Din

                Xmini = single(reshape(Z.', [Din, 1, Klook]));
                if getfield_def(nn,'useGPU',false)
                    Xmini = gpuArray(Xmini);
                end
                dlX = dlarray(Xmini, 'CBT');
                network_wall_clock=tic;
                a0_raw = double(gather(extractdata( ...
                    predict(nn.model.net, dlX))));
                network_forward_s=toc(network_wall_clock);
                if ~isreal(a0_raw) || ~isscalar(a0_raw) || ~isfinite(a0_raw)
                    error('spinn3d:AlphaModelInferenceFailed', ...
                        'Alpha network returned a non-finite/non-scalar command.');
                end
                a0 = clamp(a0_raw, nn.alpha_floor, nn.alpha_max);

end % gate.skip_forward

                if isempty(mode_str)
                    mode_str = 'nn';
                end
                state.policy_a0_hold = a0;
                state.policy_a_star_gate_hold = a_star_gate;
                state.policy_a_star_hold = a_star_gate;
                state.policy_rho_hold = NaN;
                state.policy_mode_hold = mode_str;
                state.policy_gate_active_hold = gate_hit;
                state.policy_gate_ratio_hold = gate_ratio;
                state.policy_hold_valid = true;
            end

            gate_step_request=a0;
            if gate_hit && gate.skip_forward && ...
                    strcmpi(strtrim(char(gate.fallback)),'feasible_cap')
                if policy_hold
                    [a_star_gate,gate_step_info]=alpha_feasible( ...
                        q_ref,dq_ref,ddq_ref,nn.alpha_max,robot,caps_feas, ...
                        Pcap,nn.alpha_floor,nn.itmax,fric);
                    a_star_gate=require_feasible_alpha_local( ...
                        a_star_gate,gate_step_info,'NN gate plant-step cap');
                end
                gate_cap_des=alpha_des_teacher(a_star_gate,state.a_prev, ...
                    areg.BETA,areg.A_DOT_UP,areg.A_DOT_DN,Ts,nn.alpha_max);
                gate_step_request=min(a0,gate_cap_des);
            end

            [a, a_slew, a_filt, adot, guard] = alpha_guard_backtrack( ...
                gate_step_request, state.a_prev, Ts, q_ref, dq_ref, ddq_ref, robot, ...
                caps, Pcap, fric, areg, bus, nn.alpha_max);
guard.gate_step_request=gate_step_request;
% a0 remains the policy-ZOH request; a_star_gate is a plant-step cap.
if gate_hit && strcmpi(strtrim(char(gate.fallback)),'feasible_cap') && ...
        isfinite(a_star_gate)
    guard.a_star_gate = a_star_gate;
end

guard.gate_active = gate_hit;
guard.gate_ratio  = gate_ratio;

        end

        % The execution overlay has exactly one owner: the centrally mapped
        % method guard.  In particular, selecting the descriptive
        % `actual_feasible` alpha mode must not secretly turn the overlay on
        % when PB.method_guards.<method>.enable/apply_to_modes disables it.
        apply_actual_guard = runtime_contract.actual_guard_applies;
        recovery_latched_before = logical(state.recovery_latched);
        release_eligible = recovery_latched_before && ...
            state.recovery_hold_steps >= double( ...
                actualGuard.safe_hold.minimum_hold_plant_steps) && ...
            state.recovery_clear_steps >= double( ...
                actualGuard.safe_hold.release_confirm_plant_steps);
        release_trial = release_eligible && ...
            (~logical(actualGuard.safe_hold.release_on_policy_tick_only) || ...
             policy_tick);
        if apply_actual_guard
            a_nom_pre_actual = a;
            amin_actual = double(actualGuard.alpha_floor);
            bus_guard = bus;
            bus_guard.Pgrid_avg_prev = state.Pgrid_avg_lp;
            actualGuard_step = actualGuard;
            if recovery_latched_before && ~release_trial
                % Recompute the measured-state braking/hold command every
                % millisecond, but retain zero path progress for the complete
                % latched episode.
                actualGuard_step.force_latched_hold = true;
                actualGuard_step.runtime_alpha_upper = 0.0;
                a_target_actual = 0.0;
            elseif release_trial
                % A release is only a trial until observe() receives the

                actualGuard_step.force_latched_hold = false;
                actualGuard_step.runtime_alpha_upper = double( ...
                    actualGuard.safe_hold.restart_alpha_max);
                a_target_actual = min(a,double( ...
                    actualGuard.safe_hold.restart_alpha_max));
            else
                actualGuard_step.force_latched_hold = false;
                if policy_tick
                    actualGuard_step.runtime_alpha_upper = double( ...
                        actualGuard.search.alpha_max);
                else
                    % ZOH holds desired alpha. Normal slew continues at
                    % 1 kHz; only actual safety reductions latch a ceiling.
                    actualGuard_step.runtime_alpha_upper = ...
                        state.policy_alpha_ceiling;
                end
                a_target_actual = min(a, ...
                    actualGuard_step.runtime_alpha_upper);
            end
            actual_guard = backtrack_actual_feasible_local( ...
                a_target_actual, state.a_prev, q_ref, dq_ref, ddq_ref, q_meas, dq_meas, Kp, Kd, ...
                robot, caps_feas_actual, Pcap, amin_actual, nn.itmax, ...
                fric, Ts, bus_guard, state.bus, state.use_bus_state, ...
                actualGuard_step);

            a       = actual_guard.a;
            adot    = actual_guard.adot;
            dq_cmd  = actual_guard.dq;
            ddq_cmd = actual_guard.ddq;
            Pgrid_est = actual_guard.flow.Pgrid;

            recovery_entered = ~recovery_latched_before && ...
                logical(getfield_def(actual_guard, ...
                    'freeze_on_infeasible',false));
            if recovery_entered && ...
                    logical(actualGuard.safe_hold.latch_enable)
                state.recovery_latched = true;
                state.recovery_new_entry_pending = true;
                state.recovery_release_trial_pending = false;
                state.recovery_hold_steps = 0;
                state.recovery_clear_steps = 0;
                state.recovery_entry_reason = char(getfield_def( ...
                    actual_guard,'trigger_reason','pre_noise_guard'));
            elseif recovery_latched_before
                state.recovery_release_trial_pending = release_trial && ...
                    logical(getfield_def(actual_guard,'ok',false)) && ...
                    ~logical(getfield_def(actual_guard, ...
                        'freeze_on_infeasible',false)) && a>0;
            else
                state.recovery_release_trial_pending = false;
            end
            state.last_step_hold_ok = logical(getfield_def( ...
                actual_guard,'hold_ok',true));
            state.last_step_policy_tick = logical(policy_tick);
            state.last_step_t = double(t);
            state.policy_alpha_ceiling = spinn3d_policy_ceiling( ...
                state.policy_alpha_ceiling,false,a_nom_pre_actual,a,nn.alpha_max);
            actual_guard.recovery_latched_before = ...
                recovery_latched_before;
            actual_guard.recovery_latched_after_step = ...
                logical(state.recovery_latched);
            actual_guard.recovery_entered = logical(recovery_entered);
            actual_guard.recovery_release_eligible = ...
                logical(release_eligible);
            actual_guard.recovery_release_trial = logical(release_trial);
            actual_guard.recovery_hold_steps = state.recovery_hold_steps;
            actual_guard.recovery_clear_steps = state.recovery_clear_steps;
            actual_guard.policy_alpha_ceiling = ...
                state.policy_alpha_ceiling;
            guard.actual = actual_guard;
            guard.a_nom_pre_actual = a_nom_pre_actual;
            if ~strcmp(nn.mode,'actual_feasible')
                guard.actual_overlay = true;
                mode_str = [mode_str '+actual'];
            end
        else
            if recovery_latched_before
                error('spinn3d_controller_ctcgov:LatchedGuardDisabled', ...
                    ['A latched recovery state cannot execute with the ', ...
                     'method actual guard disabled.']);
            end
            dq_cmd  = a    * dq_ref;
            ddq_cmd = a*a  * ddq_ref + adot * dq_ref;
            Pgrid_est = NaN;
        end

        % The actual-alpha guard above is a pre-noise prediction.  It cannot
        % authorize plant integration because the Demo adds torque noise and
        % applies the physical predictive clamp after this function returns.
        % spinn3d_predictive_torque_step is the authoritative final guard and
        % must project or reject the resulting torque/state before the 1-ms
        % commit while preserving one dynamically consistent command.
        if apply_actual_guard

            % consistent torque at q_meas/dq_meas.  Rebuilding it from the
            % original q_ref would undo a measured-state emergency hold and,
            % when track_feedback=false, would execute a different command
            % from the one that was verified.
            tau_cmd = row(actual_guard.tau);
        else
            e  = q_ref  - q_meas;
            ed = dq_cmd - dq_meas;
            v  = ddq_cmd + Kd.*ed + Kp.*e;

            Mq = massMatrix(robot, q_meas);
            c  = velocityProduct(robot, q_meas, dq_meas);
            g  = gravityTorque(robot, q_meas);
            tau_cmd = (Mq * v(:) + c(:) + g(:)).';

            if any(fric.B) || any(fric.Fc)
                tau_cmd = tau_cmd + fric.B.*dq_meas + ...
                    fric.Fc.*tanh(dq_meas./max(fric.vel_eps,1e-9));
            end
        end

        state.a_prev = a;
        flow_pred = getfield_def(actual_guard,'flow',struct());
        st_pred = getfield_def(actual_guard,'st_next', []);
        if isempty(st_pred) && state.use_bus_state
            st_pred = state.bus;
        end
        info = struct('a_max',a,'a0',a0,'guard',guard, ...
              'a_prev_step',a_prev_step, ...
              'actual_guard_enabled',logical(apply_actual_guard), ...
              'noguard_policy',noguard_diag, ...
              'noguard_policy_tick',noguard_policy_tick, ...
              'a_slew',a_slew,'a_filt',a_filt,'adot',adot,'mode',mode_str, ...
              'Pgrid_est',Pgrid_est,'Pgrid_lp',state.Pgrid_lp, 'Pgrid_avg_lp',state.Pgrid_avg_lp, ...
              'Pgrid_static_init_W',power_init.Pgrid_static_W, ...
              'Pgrid_avg_init_W',power_init.Pgrid_avg0_W, ...
              'power_init_scheme',power_init.scheme, ...
              'policy_tick',policy_tick,'policy_hold',policy_hold,'policy_dt',state.policy_dt, ...
              'Pdump_est',getfield_def(flow_pred,'Pdump',NaN), ...
              'Pspill_est',getfield_def(flow_pred,'Pspill',NaN), ...
              'gate_active',gate_hit,'gate_ratio',gate_ratio, ...
              'soft_power',soft_power_diag, ...
              'mpc',mpc_diag, ...
              'bus_state_pred',st_pred, ...
              'final_step_safety_pending',true, ...
              'controller_wall_s',toc(step_wall_clock), ...
              'network_forward_s',network_forward_s);
    end

    function [obs, st_out] = observe(tau_exec, dq_exec, a_exec, step_audit, measured)
        tau_exec = row(tau_exec);
        dq_exec  = row(dq_exec);
        if nargin >= 3

            
            % authoritative post-noise execution layer may retain less of the
            % command, so commit the actually executed progress rate here
            % before the next 1-ms controller sample.
            state.policy_alpha_ceiling = spinn3d_policy_ceiling( ...
                state.policy_alpha_ceiling,false,state.a_prev,double(a_exec),nn.alpha_max);
            state.a_prev = double(a_exec);
        end

        if nargin < 4, step_audit = []; end
        [post_valid,post_modified,release_margin_ok] = ...
            post_noise_recovery_audit_local(step_audit, ...
                double(actualGuard.safe_hold.release_ratio_max));
        state.post_noise_feedback_valid = logical(post_valid);
        state.recovery_entered_last_observe = false;
        state.recovery_released_last_observe = false;
        release_trial_observed = logical( ...
            state.recovery_release_trial_pending);

        if state.recovery_new_entry_pending
            % The command returned by step() was already a measured-state
            % safe hold, so this committed sample is the first latch sample.
            state.recovery_latched = true;
            state.recovery_hold_steps = 1;
            if post_valid && state.last_step_hold_ok && ~post_modified
                state.recovery_clear_steps = 1;
            else
                state.recovery_clear_steps = 0;
            end
            state.recovery_new_entry_pending = false;
            state.recovery_entered_last_observe = true;
        elseif ~state.recovery_latched && post_modified
            % A pre-noise command may pass while the authoritative final
            % projector still has to alter it. Enter recovery for the next
            % sample; this modified sample is not counted as a hold sample.
            state.recovery_latched = true;
            state.recovery_hold_steps = 0;
            state.recovery_clear_steps = 0;
            state.recovery_entry_reason = 'post_noise_projection';
            state.recovery_entered_last_observe = true;
        elseif state.recovery_latched
            state.recovery_hold_steps = state.recovery_hold_steps + 1;
            if state.recovery_release_trial_pending
                % Two-phase release commit: step() only proposed the restart;
                % the latch clears here after the executed command and final
                % plant state are known.
                if post_valid && ~post_modified && ...
                        release_margin_ok && double(a_exec)>0
                    state.recovery_latched = false;
                    state.recovery_released_last_observe = true;
                    state.recovery_hold_steps = 0;
                    state.recovery_clear_steps = 0;
                    state.recovery_entry_reason = 'none';
                    state.policy_alpha_ceiling = double(a_exec);
                else
                    state.recovery_clear_steps = 0;
                end
            elseif post_valid && state.last_step_hold_ok && ~post_modified
                state.recovery_clear_steps = ...
                    state.recovery_clear_steps + 1;
            else
                state.recovery_clear_steps = 0;
            end
        end
        state.recovery_release_trial_pending = false;

        if nargin>=5 && isstruct(measured) && isfield(measured,'flow')
            % Plant observation has already committed exactly one interval.
            % Do not replay or advance the electrical state a second time.
            flow=measured.flow;
            st_out=measured.bus_state;

            if state.use_bus_state
                % Returned observation retains actual capacitor energy for
                % evidence; only the private predictor uses nominal C.
                state.bus=spinn3d_dc_state_for_model(st_out,bus);
            end
            if isfield(measured,'Pgrid_avg_initial_W') && ...
                    isfinite(measured.Pgrid_avg_initial_W)
                state.Pgrid_avg_lp=double(measured.Pgrid_avg_initial_W);
                state.Pgrid_lp=double(measured.Pgrid_avg_initial_W);
            end
        elseif state.use_bus_state
            [flow,st_out]=spinn3d_bus_step(tau_exec,dq_exec,bus,state.bus,Ts);

            state.bus=st_out;
        else
            bus_nostate=bus; bus_nostate.enable_state=false;
            [flow,st_out]=spinn3d_bus_step(tau_exec,dq_exec,bus_nostate,[],Ts);
        end
        Pgrid_obs = flow.Pgrid;
        if gate.enable && isfinite(Pcap) && Pcap>0
            tau_lp = max(gate.lp_tau_s, 1e-6);
            betaP  = exp(-Ts/tau_lp);
            state.Pgrid_lp = betaP*state.Pgrid_lp + (1-betaP)*Pgrid_obs;
        else
            state.Pgrid_lp = Pgrid_obs;
        end

        bus_avg = bus;
        bus_avg.Pgrid_avg_prev = state.Pgrid_avg_lp;
        [state.Pgrid_avg_lp, ~, Pavg_cap_obs, Ppeak_cap_obs] = spinn3d_bus_avg_guard(Pgrid_obs, Pcap, bus_avg, Ts);

        obs = struct('flow',flow,'bus_state',st_out,'Pgrid_lp',state.Pgrid_lp, ...
                     'Pgrid_avg_lp',state.Pgrid_avg_lp, 'Pavg_cap',Pavg_cap_obs, ...
                     'Pgrid_peak_cap',Ppeak_cap_obs,'a_prev_committed',state.a_prev, ...
                     'recovery_latched',logical(state.recovery_latched), ...
                     'recovery_entered', ...
                        logical(state.recovery_entered_last_observe), ...
                     'recovery_released', ...
                        logical(state.recovery_released_last_observe), ...
                     'recovery_release_trial',release_trial_observed, ...
                     'recovery_entry_reason',state.recovery_entry_reason, ...
                     'recovery_hold_steps',state.recovery_hold_steps, ...
                     'recovery_clear_steps',state.recovery_clear_steps, ...
                     'post_noise_feedback_valid',logical(post_valid), ...
                     'post_noise_command_modified',logical(post_modified), ...
                     'recovery_release_margin_ok', ...
                        logical(release_margin_ok), ...
                     'policy_alpha_ceiling',state.policy_alpha_ceiling);
    end

    function st = get_state()
        st = state;
    end

execution_contract = struct( ...
    'post_noise_post_clamp_motion_recheck_required',true, ...
    'state_commit_only_after_recheck',true, ...
    'soft_power_performance_only',true, ...
    'soft_power_hard_enforced',false, ...
    'soft_power_schema',softPower.schema, ...
    'plant_step_s',Ts);
ctl = struct('step',@step, 'observe',@observe, 'get_state',@get_state, ...
             'Kp',Kp,'Kd',Kd,'limits',caps,'nnAlpha',nn,'Ts',Ts,'alphaReg',areg, ...
             'bus',bus,'power_init',power_init,'softPower',softPower, ...
             'execution_contract',execution_contract);
end

function [valid,modified,margin_ok] = ...
        post_noise_recovery_audit_local(A,ratio_max)
% Validate only the authoritative output of spinn3d_predictive_torque_step.
% Missing feedback never releases an existing latch. It also does not invent
% a post-noise trigger for compatibility callers that still use observe(3).
valid = false;
modified = false;
margin_ok = false;
if ~isstruct(A) || ~isscalar(A)
    return;
end
required = {'tau_ratio','motor_ratio','Paxis_ratio', ...
    'Pgrid_mid_W','Pgrid_peak_W','Pdump_mid_W','Pdump_peak_W', ...
    'execution_progress_ratio','execution_command_modified', ...
    'motion_projection_applied','joint_limit_viability_pass', ...
    'final_state_verified_before_commit','final_motion'};
if ~all(isfield(A,required)) || ~isstruct(A.final_motion) || ...
        ~isscalar(A.final_motion)
    return;
end
M = A.final_motion;
if ~all(isfield(M,{'pass','qd_ratio','qdd_ratio', ...
        'joint_limit_viability_pass'}))
    return;
end
ratios = double([A.tau_ratio,A.motor_ratio,A.Paxis_ratio, ...
    M.qd_ratio,M.qdd_ratio, ...
    cap_ratio_recovery_local(A.Pgrid_mid_W,A.Pgrid_peak_W), ...
    cap_ratio_recovery_local(A.Pdump_mid_W,A.Pdump_peak_W)]);
progress = double(A.execution_progress_ratio);
valid = all(isfinite(ratios)) && isscalar(progress) && ...
    isfinite(progress) && logical(A.final_state_verified_before_commit) && ...
    logical(M.pass) && logical(A.joint_limit_viability_pass) && ...
    logical(M.joint_limit_viability_pass);
if ~valid
    return;
end
modified = logical(A.execution_command_modified) || ...
    logical(A.motion_projection_applied) || progress < 1.0-1e-12;
margin_ok = all(ratios<=double(ratio_max)+1e-12);
end

function ratio = cap_ratio_recovery_local(value,cap)
value=double(value); cap=double(cap);
if ~isscalar(value) || ~isscalar(cap) || ~isfinite(value) || ...
        isnan(cap) || cap<0
    ratio=inf;
elseif isinf(cap)
    ratio=0;
elseif cap==0
    if value<=1e-12, ratio=0; else, ratio=inf; end
else
    ratio=max(value,0)/cap;
end
end

function n = local_num_joints(robot)
    n = numel(homeConfiguration(robot));
end
function caps = normalize_caps(lim, nJ)
    caps = struct();

    % Preserve the central path-domain search contract when numeric joint
    % limits are normalized; alpha_feasible rejects a missing/private copy.
    caps.alpha_feasible = lim.alpha_feasible;
    caps.pavg_soft_target_select = logical(getfield_def(lim,'pavg_soft_target_select',false));
    if isfield(lim,'execution_audit'), caps.execution_audit=lim.execution_audit; end
    fn = {'tau_max','qd_max','qdd_max','qdd_cmd_max_actual','P_axis_max'};
    for i=1:numel(fn)
        if isfield(lim,fn{i}) && ~isempty(lim.(fn{i}))
            v = lim.(fn{i})(:).'; if isscalar(v), v = repmat(v,1,nJ); end
            caps.(fn{i}) = double(v(1:nJ));
        else
            caps.(fn{i}) = inf(1,nJ);
        end
    end
    if isfield(lim,'countRegen'), caps.countRegen = logical(lim.countRegen); end
end
function [qmin,qmax] = robot_joint_limits_local(robot,nJ)
    qmin = -inf(1,nJ);
    qmax =  inf(1,nJ);
    jj = 0;
    for ib = 1:numel(robot.Bodies)
        body = robot.Bodies{ib};
        if ~strcmpi(body.Joint.Type,'fixed')
            jj = jj + 1;
            if jj <= nJ && numel(body.Joint.PositionLimits)==2
                qmin(jj) = double(body.Joint.PositionLimits(1));
                qmax(jj) = double(body.Joint.PositionLimits(2));
            end
        end
    end
end
function v = vec_row(x, n), x=double(x); if isscalar(x), v=repmat(x,1,n); else, v=x(:).'; end, v=v(1:n); end
function v = reqf(S, k),  v = S.(k); end
function x = getfield_def(S, k, d), if isstruct(S)&&isfield(S,k)&&~isempty(S.(k)), x=S.(k); else, x=d; end, end
function r = row(v), r = v(:)'; end
function y = clamp(x, lo, hi), y = min(hi, max(lo, x)); end
function a = require_feasible_alpha_local(a,info,context)
if ~isstruct(info) || ~isscalar(info) || ...
        ~valid_binary_field_local(info,'any_feasible') || ...
        ~valid_binary_field_local(info,'no_feasible')
    error('spinn3d_controller_ctcgov:InvalidFeasibilityInfo', ...
        'alpha_feasible returned malformed metadata during %s.',context);
end
if ~logical(info.any_feasible) || logical(info.no_feasible)
    error('spinn3d_controller_ctcgov:NoFeasibleAlpha', ...
        ['No feasible alpha exists during %s.  The diagnostic alpha_min ', ...
         'floor will not be clamped or executed as a command.'],context);
end
if ~isnumeric(a) || ~isreal(a) || ~isscalar(a) || ~isfinite(double(a))
    error('spinn3d_controller_ctcgov:InvalidFeasibleAlpha', ...
        ['alpha_feasible returned an invalid command despite feasible ', ...
         'metadata during %s.'],context);
end
a = double(a);
end
function tf = valid_binary_field_local(S,field)
v = [];
if isstruct(S) && isfield(S,field)
    v = S.(field);
end
tf = (islogical(v) || isnumeric(v)) && isscalar(v) && ...
     isreal(v) && isfinite(double(v)) && any(double(v)==[0 1]);
end
function validate_alpha_inference_cadence_local(P,expected_policy_dt)
required = {'K','ds','policy_dt'};
if ~isstruct(P) || ~isscalar(P) || ~all(isfield(P,required))
    error('spinn3d:AlphaModelCadenceMissing', ...
        ['Alpha inference requires preproc.K, preproc.ds and ', ...
         'preproc.policy_dt; a plant-Ts fallback is forbidden.']);
end
K = double(P.K);
ds = double(P.ds);
saved_dt = double(P.policy_dt);
tol = 64*eps(max([1,abs(expected_policy_dt),abs(ds),abs(saved_dt)]));
if ~(isnumeric(P.K) && isreal(P.K) && isscalar(K) && ...
        isfinite(K) && K>1 && K==round(K) && ...
        isnumeric(P.ds) && isreal(P.ds) && isscalar(ds) && ...
        isfinite(ds) && ds>0 && ...
        isnumeric(P.policy_dt) && isreal(P.policy_dt) && ...
        isscalar(saved_dt) && isfinite(saved_dt) && saved_dt>0 && ...
        abs(ds-expected_policy_dt)<=tol && ...
        abs(saved_dt-expected_policy_dt)<=tol)
    error('spinn3d:AlphaModelCadenceMismatch', ...
        ['Alpha inference ds and saved policy_dt must both equal the ', ...
         'central policy cadence %.9g s (saved ds %.9g s, policy_dt %.9g s).'], ...
        expected_policy_dt,ds,saved_dt);
end
end
function a0 = plan_lookup(plan, rho)
    rho = min(1.0, max(0.0, double(rho)));
    xr  = row(plan.rho);
    ya  = row(plan.a_des_seq);
    if numel(xr) < 2
        a0 = ya(1);
        return;
    end
    a0 = interp1(xr, ya, min(max(rho,xr(1)),xr(end)), 'previous');
end

function tf = model_uses_bus_features_local(nn)
tf = false;
try
    if isfield(nn,'model') && isstruct(nn.model) && isfield(nn.model,'preproc') && isstruct(nn.model.preproc)
        P = nn.model.preproc;
        if isfield(P,'use_bus_state_features') && ~isempty(P.use_bus_state_features)
            tf = logical(P.use_bus_state_features);
            return;
        end
        if isfield(P,'bus_feature_dim') && ~isempty(P.bus_feature_dim)
            tf = double(P.bus_feature_dim) > 0;
        end
    end
catch
    tf = false;
end
end

function x = feature_vector_local(nn, q, dq, ddq, robot, Pcap, caps, a_prev, areg, Ts, s_norm, bus, state, observed)
if model_uses_bus_features_local(nn)
    st_feat = [];
    pavg_feat = 0.0;
    if isstruct(state)
        if isfield(state,'use_bus_state') && state.use_bus_state && isfield(state,'bus')
            st_feat = state.bus;
        end
        if isfield(state,'Pgrid_avg_lp') && ~isempty(state.Pgrid_avg_lp)
            pavg_feat = state.Pgrid_avg_lp;
        end
    end
    % Every look-ahead token uses this same current measured state.
    % Future tokens describe reference geometry, never future measurements.
    x = nn.featureFcn(q, dq, ddq, robot, nn.payload, Pcap, caps, a_prev, ...
        areg.BETA, areg.A_DOT_UP, areg.A_DOT_DN, Ts, s_norm, bus, st_feat, pavg_feat, observed);
else
    x = nn.featureFcn(q, dq, ddq, robot, nn.payload, Pcap, caps, a_prev, ...
        areg.BETA, areg.A_DOT_UP, areg.A_DOT_DN, Ts, s_norm);
end
end

function caps_feas = build_caps_feas_local(caps, bus, state, Ts)
    caps_feas = caps;
    caps_feas.eta_share  = getfield_def(bus,'eta_share', getfield_def(caps,'eta_share',1.0));
    caps_feas.P_brk_peak = getfield_def(bus,'P_brk_peak', getfield_def(bus,'P_dump_max', getfield_def(caps,'P_brk_peak',inf)));
    if isstruct(state) && isfield(state,'use_bus_state') && state.use_bus_state
        caps_feas.bus = bus;
        caps_feas.bus_Ts = Ts;
        caps_feas.bus_state = state.bus;
    end
end

function G = backtrack_actual_feasible_local(a_nom, a_prev, q_r, dq_r, ddq_r, q_meas, dq_meas, Kp, Kd, robot, caps, Pcap, amin, itmax, fric, Ts, bus, st_bus, use_state, actualGuard)
    % Actual-step feasibility is not monotonic in alpha when the alpha-dot
    % term is active: alpha=0 may be infeasible because it commands a huge
    % deceleration from a_prev.  Therefore do not assume that the floor is
    % feasible.  Search a small candidate set and choose the largest feasible

    % catch up instead of letting s keep advancing into torque saturation.
    search = actualGuard.search;
    % This file-level local function has its own workspace: read the
    % centrally validated refinement count from the propagated guard rather
    % than relying on the constructor workspace above.
    safe_hold_refine_iterations = double( ...
        actualGuard.safe_hold.gain_refine_iterations);
    force_latched_hold = logical(getfield_def( ...
        actualGuard,'force_latched_hold',false));
    runtime_alpha_upper = double(getfield_def(actualGuard, ...
        'runtime_alpha_upper',search.alpha_max));

    alpha_max = min(double(search.alpha_max),runtime_alpha_upper);
    a_nom = clamp(a_nom, 0, alpha_max);

    % For an explicitly enabled recovery guard, the planner alpha floor is a
    % throughput/request preference, not a safety-recovery floor.  In
    % particular, after an emergency hold a_prev is
    % zero and the first qdd-feasible restart can be far below the planner
    % floor (typically 1e-3--1e-2 at a 1-ms servo period).  Clipping the search
    % to amin made that restart impossible and permanently locked s at the
    % same path sample.  Opt-in sub-floor recovery must therefore be allowed
    % to search down to zero; every returned non-hold candidate is still
    % accepted only after all torque/qd/qdd/BUS hard checks below pass.
    requested_floor = clamp(double(actualGuard.requested_alpha_floor), ...
        0,alpha_max);
    if logical(actualGuard.allow_subfloor_recovery)
        recovery_floor = 0.0;
    else

        % alpha>=amin.  WarmStart opts into the zero/sub-floor recovery path.
        recovery_floor = max(0,min(double(amin),a_nom));
    end

    [ok_nom, info_nom] = actual_step_feasible_local(a_nom, a_prev, q_r, dq_r, ddq_r, q_meas, dq_meas, Kp, Kd, robot, caps, Pcap, fric, Ts, bus, st_bus, use_state, actualGuard);
    if ok_nom && ~force_latched_hold
        G = info_nom;
        G.a_nom = a_nom;
        G.a_clip = 0;
        G.backtrack_iters = 0;
        G.floor_infeasible = false;
        G.freeze_on_infeasible = false;
        G = annotate_actual_selection_local(G, info_nom, requested_floor, ...
            recovery_floor, a_prev, 'nominal');
        return;
    end

    best_bad = info_nom;
    best_bad_score = getfield_def(info_nom,'violation',inf);
    n_eval = 0;
    if ~force_latched_hold
    % Feasibility is non-monotone because alpha_dot enters acceleration.
    % When the high-level target drops, every candidate <=a_nom can require
    % an infeasible instantaneous deceleration even though a transition
    % candidate between a_nom and a_prev is safe.  Search the complete
    % recovery-to-transition interval, with candidates on both sides of
    % a_prev, and select the feasible point closest to the governed target.
    da_est = estimate_alpha_step_cap_local(dq_r,ddq_r,caps,Ts,search);
    up_step = double(search.alpha_up_step_max);
    down_step = double(search.alpha_down_step_max);
    search_lower = max(recovery_floor,min(a_nom,a_prev-down_step));
    search_upper = min(alpha_max,max(a_nom,a_prev+up_step));
    [~, info_floor] = actual_step_feasible_local(recovery_floor, a_prev, q_r, dq_r, ddq_r, q_meas, dq_meas, Kp, Kd, robot, caps, Pcap, fric, Ts, bus, st_bus, use_state, actualGuard);
    ngrid = max(double(search.grid_min_points), ...
        min(double(search.grid_max_points), ...
        double(search.grid_points_per_itmax)* ...
        max(double(search.grid_itmax_floor),double(itmax)) + ...
        double(search.grid_point_offset)));
    cand_lin = linspace(search_lower, search_upper, ngrid);
    % Fixed and nominal-scaled small candidates make sub-floor recovery
    % explicit even for very small a_nom.  Do not filter these by amin.
    small_abs = double(search.small_alpha_abs(:).');
    small_rel = search_upper * ...
        2.^(-double(search.small_alpha_rel_exp2(:).'));
    small = [small_abs, small_rel];
    small = small(small >= search_lower & small <= search_upper);
    around_prev = a_prev + da_est * ...
        double(search.around_prev_step_multipliers(:).');
    geom_magnitude = reshape( ...
        10.^double(search.around_prev_geom_exp10(:)),[],1);
    geom_sign = double(search.around_prev_geom_signs(:).');
    geom_prev = a_prev + geom_magnitude.*geom_sign;
    around_prev = [around_prev,geom_prev(:).'];
    around_prev = around_prev(around_prev >= search_lower & around_prev <= search_upper);
    cand = unique([search_lower, small, cand_lin, around_prev, a_nom, ...
                   clamp(a_prev, search_lower, search_upper), search_upper], 'sorted');

    best_feas = [];
    best_bad  = info_floor;
    best_bad_score = getfield_def(info_floor,'violation',inf);
    best_feas_score = inf;
    for ii = 1:numel(cand)
        ai = cand(ii);
        [ok_i, info_i] = actual_step_feasible_local(ai, a_prev, q_r, dq_r, ddq_r, q_meas, dq_meas, Kp, Kd, robot, caps, Pcap, fric, Ts, bus, st_bus, use_state, actualGuard);
        n_eval = n_eval + 1;
        if ok_i
            % Closest to the already-governed nominal command minimizes the
            % extra transition.  Prefer the larger alpha only on a tie.
            sc_feas = abs(info_i.a-a_nom) - ...
                double(search.feasible_tie_alpha_weight)*info_i.a;
            if isempty(best_feas) || sc_feas < best_feas_score
                best_feas = info_i;
                best_feas_score = sc_feas;
            end
        else
            % Prefer lower violation; when tied, prefer smaller alpha so the
            % reference progression is conservative during recovery.
            sc = getfield_def(info_i,'violation',inf) + ...
                double(search.infeasible_alpha_penalty)*info_i.a;
            if sc < best_bad_score
                best_bad_score = sc;
                best_bad = info_i;
            end
        end
    end

    if ~isempty(best_feas)
        G = best_feas;
        G.a_nom = a_nom;
        G.a_clip = max(a_nom - G.a, 0);
        G.backtrack_iters = n_eval;
        G.floor_infeasible = false;
        G.freeze_on_infeasible = false;
        G.restart_bisect = false;
        G = annotate_actual_selection_local(G, info_nom, requested_floor, ...
            recovery_floor, a_prev, 'bidirectional_transition_grid');
        return;
    end
    end

    if logical(actualGuard.freeze_on_infeasible) || force_latched_hold
        % Zero path progress is a recovery state, not permission to retain an

        % stages.  First keep full measured-velocity braking and retain as much
        % strictly positive position catch-up as possible.  A coarse gain grid
        % must not jump directly from a still-infeasible positive gain to zero:

        % reference tracking error, so it can become an absorbing state.  The

        % and the first feasible/infeasible bracket is refined by a centrally
        % declared number of complete audits.  Zero position gain is considered
        % only after every positive candidate failed.  If full damping is itself
        % infeasible, hold the measured position and apply the same positive-
        % first rule to damping.  Every candidate passes the same complete

        ag_hold = actualGuard;
        ag_hold.force_zero_adot = true;
        ag_hold.current_state_safe_hold = true;
        hold_best = [];
        hold_best_score = inf;
        hold_found = false;
        hold_evals = 0;
        position_grid_evals = 0;
        position_refine_evals = 0;
        damping_grid_evals = 0;
        damping_refine_evals = 0;
        positive_position_gain_found = false;
        positive_damping_gain_found = false;
        position_initial_feasible = NaN;
        position_initial_infeasible = NaN;
        position_final_feasible = NaN;
        position_final_infeasible = NaN;
        damping_initial_feasible = NaN;
        damping_initial_infeasible = NaN;
        damping_final_feasible = NaN;
        damping_final_infeasible = NaN;

        position_grid = double( ...
            actualGuard.safe_hold.position_gain_scale_grid(:).');
        position_positive = position_grid(position_grid>0);
        previous_infeasible_gain = NaN;
        for ih = 1:numel(position_positive)
            gain_i = position_positive(ih);
            ag_hold.hold_position_gain_scale = gain_i;
            ag_hold.hold_damping_gain_scale = 1.0;
            [ok_i, info_i] = actual_step_feasible_local(0.0, 0.0, ...
                q_r,dq_r,ddq_r,q_meas,dq_meas,Kp,Kd,robot,caps,Pcap, ...
                fric,Ts,bus,st_bus,use_state,ag_hold);
            hold_evals = hold_evals + 1;
            position_grid_evals = position_grid_evals + 1;
            info_i.fallback_kind = ...
                'frozen_progress_gain_scaled_catchup';
            score_i = getfield_def(info_i,'violation',inf);
            if isempty(hold_best) || score_i < hold_best_score
                hold_best = info_i;
                hold_best_score = score_i;
            end
            if ok_i
                G = info_i;
                hold_found = true;
                positive_position_gain_found = true;
                positive_damping_gain_found = true;
                position_initial_feasible = gain_i;
                position_initial_infeasible = previous_infeasible_gain;
                position_final_feasible = gain_i;
                position_final_infeasible = previous_infeasible_gain;

                % The grid is strictly descending, so the immediately
                % preceding failed point brackets the first feasible point.
                % Feasibility can include coupled actuator/BUS effects; no
                % midpoint is trusted merely because it lies in the bracket.

                if isfinite(previous_infeasible_gain)
                    gain_lo = gain_i;
                    gain_hi = previous_infeasible_gain;
                    for ir = 1:safe_hold_refine_iterations
                        gain_mid = 0.5*(gain_lo+gain_hi);
                        ag_hold.hold_position_gain_scale = gain_mid;
                        ag_hold.hold_damping_gain_scale = 1.0;
                        [ok_mid, info_mid] = actual_step_feasible_local( ...
                            0.0,0.0,q_r,dq_r,ddq_r,q_meas,dq_meas, ...
                            Kp,Kd,robot,caps,Pcap,fric,Ts,bus,st_bus, ...
                            use_state,ag_hold);
                        hold_evals = hold_evals + 1;
                        position_refine_evals = ...
                            position_refine_evals + 1;
                        info_mid.fallback_kind = ...
                            'frozen_progress_gain_scaled_catchup';
                        if ok_mid
                            gain_lo = gain_mid;
                            G = info_mid;
                        else
                            gain_hi = gain_mid;
                        end
                    end
                    position_final_feasible = gain_lo;
                    position_final_infeasible = gain_hi;
                end
                break;
            end
            previous_infeasible_gain = gain_i;
        end

        if ~hold_found
            % The measured-state/full-damping zero-position anchor is tested
            % only after all centrally declared positive position gains.
            ag_hold.hold_position_gain_scale = 0.0;
            ag_hold.hold_damping_gain_scale = 1.0;
            [ok_zero_position, info_zero_position] = ...
                actual_step_feasible_local(0.0,0.0, ...
                    q_r,dq_r,ddq_r,q_meas,dq_meas,Kp,Kd,robot,caps, ...
                    Pcap,fric,Ts,bus,st_bus,use_state,ag_hold);
            hold_evals = hold_evals + 1;
            position_grid_evals = position_grid_evals + 1;
            info_zero_position.fallback_kind = ...
                'measured_state_brake_hold';
            score_i = getfield_def(info_zero_position,'violation',inf);
            if isempty(hold_best) || score_i < hold_best_score
                hold_best = info_zero_position;
                hold_best_score = score_i;
            end
            if ok_zero_position
                G = info_zero_position;
                hold_found = true;
                positive_damping_gain_found = true;
                position_initial_feasible = 0.0;
                position_initial_infeasible = previous_infeasible_gain;
                position_final_feasible = 0.0;
                position_final_infeasible = previous_infeasible_gain;
                % Zero is a physical anchor, not an early stopping value.
                % If the entire positive grid failed, use it as the lower
                % end of one final bracket and actively look for a smaller
                % positive catch-up gain before accepting zero.
                if isfinite(previous_infeasible_gain)
                    gain_lo = 0.0;
                    gain_hi = previous_infeasible_gain;
                    for ir = 1:safe_hold_refine_iterations
                        gain_mid = 0.5*(gain_lo+gain_hi);
                        ag_hold.hold_position_gain_scale = gain_mid;
                        ag_hold.hold_damping_gain_scale = 1.0;
                        [ok_mid, info_mid] = actual_step_feasible_local( ...
                            0.0,0.0,q_r,dq_r,ddq_r,q_meas,dq_meas, ...
                            Kp,Kd,robot,caps,Pcap,fric,Ts,bus,st_bus, ...
                            use_state,ag_hold);
                        hold_evals = hold_evals + 1;
                        position_refine_evals = ...
                            position_refine_evals + 1;
                        info_mid.fallback_kind = ...
                            'frozen_progress_gain_scaled_catchup';
                        if ok_mid
                            gain_lo = gain_mid;
                            G = info_mid;
                            positive_position_gain_found = true;
                        else
                            gain_hi = gain_mid;
                        end
                    end
                    position_final_feasible = gain_lo;
                    position_final_infeasible = gain_hi;
                end
            end
        end

        if ~hold_found
            damping_grid = double(actualGuard.safe_hold.damping_gain_scale_grid(:).');
            % beta=1 at kappa=0 was already tested by the first stage.
            damping_positive = damping_grid( ...
                damping_grid>0 & damping_grid<1);
            previous_infeasible_gain = 1.0;
            for ih = 1:numel(damping_positive)
                gain_i = damping_positive(ih);
                ag_hold.hold_position_gain_scale = 0.0;
                ag_hold.hold_damping_gain_scale = gain_i;
                [ok_i, info_i] = actual_step_feasible_local(0.0,0.0, ...
                    q_r,dq_r,ddq_r,q_meas,dq_meas,Kp,Kd,robot,caps,Pcap, ...
                    fric,Ts,bus,st_bus,use_state,ag_hold);
                hold_evals = hold_evals + 1;
                damping_grid_evals = damping_grid_evals + 1;
                info_i.fallback_kind = 'measured_state_damping_scaled_hold';
                score_i = getfield_def(info_i,'violation',inf);
                if isempty(hold_best) || score_i < hold_best_score
                    hold_best = info_i;
                    hold_best_score = score_i;
                end
                if ok_i
                    G = info_i;
                    hold_found = true;
                    positive_damping_gain_found = true;
                    damping_initial_feasible = gain_i;
                    damping_initial_infeasible = previous_infeasible_gain;
                    damping_final_feasible = gain_i;
                    damping_final_infeasible = previous_infeasible_gain;
                    gain_lo = gain_i;
                    gain_hi = previous_infeasible_gain;
                    for ir = 1:safe_hold_refine_iterations
                        gain_mid = 0.5*(gain_lo+gain_hi);
                        ag_hold.hold_position_gain_scale = 0.0;
                        ag_hold.hold_damping_gain_scale = gain_mid;
                        [ok_mid, info_mid] = actual_step_feasible_local( ...
                            0.0,0.0,q_r,dq_r,ddq_r,q_meas,dq_meas, ...
                            Kp,Kd,robot,caps,Pcap,fric,Ts,bus,st_bus, ...
                            use_state,ag_hold);
                        hold_evals = hold_evals + 1;
                        damping_refine_evals = damping_refine_evals + 1;
                        info_mid.fallback_kind = ...
                            'measured_state_damping_scaled_hold';
                        if ok_mid
                            gain_lo = gain_mid;
                            G = info_mid;
                        else
                            gain_hi = gain_mid;
                        end
                    end
                    damping_final_feasible = gain_lo;
                    damping_final_infeasible = gain_hi;
                    break;
                end
                previous_infeasible_gain = gain_i;
            end
        end
        if ~hold_found
            % Zero damping is the final anchor and is never considered while
            % any positive damping candidate remains untested.
            ag_hold.hold_position_gain_scale = 0.0;
            ag_hold.hold_damping_gain_scale = 0.0;
            [ok_zero_damping, info_zero_damping] = ...
                actual_step_feasible_local(0.0,0.0, ...
                    q_r,dq_r,ddq_r,q_meas,dq_meas,Kp,Kd,robot,caps, ...
                    Pcap,fric,Ts,bus,st_bus,use_state,ag_hold);
            hold_evals = hold_evals + 1;
            damping_grid_evals = damping_grid_evals + 1;
            info_zero_damping.fallback_kind = ...
                'measured_state_damping_scaled_hold';
            score_i = getfield_def(info_zero_damping,'violation',inf);
            if isempty(hold_best) || score_i < hold_best_score
                hold_best = info_zero_damping;
                hold_best_score = score_i;
            end
            if ok_zero_damping
                G = info_zero_damping;
                hold_found = true;
                damping_initial_feasible = 0.0;
                damping_initial_infeasible = previous_infeasible_gain;
                damping_final_feasible = 0.0;
                damping_final_infeasible = previous_infeasible_gain;
                % As for position catch-up, terminal zero is only retained

                if isfinite(previous_infeasible_gain)
                    gain_lo = 0.0;
                    gain_hi = previous_infeasible_gain;
                    for ir = 1:safe_hold_refine_iterations
                        gain_mid = 0.5*(gain_lo+gain_hi);
                        ag_hold.hold_position_gain_scale = 0.0;
                        ag_hold.hold_damping_gain_scale = gain_mid;
                        [ok_mid, info_mid] = actual_step_feasible_local( ...
                            0.0,0.0,q_r,dq_r,ddq_r,q_meas,dq_meas, ...
                            Kp,Kd,robot,caps,Pcap,fric,Ts,bus,st_bus, ...
                            use_state,ag_hold);
                        hold_evals = hold_evals + 1;
                        damping_refine_evals = damping_refine_evals + 1;
                        info_mid.fallback_kind = ...
                            'measured_state_damping_scaled_hold';
                        if ok_mid
                            gain_lo = gain_mid;
                            G = info_mid;
                            positive_damping_gain_found = true;
                        else
                            gain_hi = gain_mid;
                        end
                    end
                    damping_final_feasible = gain_lo;
                    damping_final_infeasible = gain_hi;
                end
            end
        end
        if ~hold_found
            G = hold_best;
            G.fallback_kind = 'measured_state_least_violation';
        end
        G.a_nom = a_nom;
        G.a_clip = a_nom;
        G.backtrack_iters = n_eval + hold_evals;
        G.floor_infeasible = ~force_latched_hold;
        G.freeze_on_infeasible = true;
        selection_mode = 'safe_hold_gain_search';
        if force_latched_hold
            selection_mode = 'safe_hold_latched';
        end
        G = annotate_actual_selection_local(G, info_nom, requested_floor, ...
            recovery_floor, a_prev, selection_mode);
        % Preserve why the non-hold command was rejected separately from the

        % feasible, the least-violation command is only a request to the
        % authoritative post-noise multidimensional projector; demos must not
        % terminate before that final guard runs.
        G.hold_ok = logical(hold_found);
        G.hold_tau = G.tau;
        G.safe_hold_positive_position_gain_found = ...
            logical(positive_position_gain_found);
        G.safe_hold_positive_damping_gain_found = ...
            logical(positive_damping_gain_found);
        G.safe_hold_position_grid_evaluations = position_grid_evals;
        G.safe_hold_position_refine_evaluations = position_refine_evals;
        G.safe_hold_damping_grid_evaluations = damping_grid_evals;
        G.safe_hold_damping_refine_evaluations = damping_refine_evals;
        G.safe_hold_total_evaluations = hold_evals;
        G.safe_hold_position_bracket_initial = ...
            [position_initial_feasible,position_initial_infeasible];
        G.safe_hold_position_bracket_final = ...
            [position_final_feasible,position_final_infeasible];
        G.safe_hold_damping_bracket_initial = ...
            [damping_initial_feasible,damping_initial_infeasible];
        G.safe_hold_damping_bracket_final = ...
            [damping_final_feasible,damping_final_infeasible];
        G.safe_hold_zero_position_gain_selected = ...
            logical(getfield_def(G,'position_gain_scale',0)==0);
        G.safe_hold_zero_damping_gain_selected = ...
            logical(getfield_def(G,'damping_gain_scale',0)==0);
        G.trigger_nominal = compact_actual_trigger_local(info_nom);
        G.trigger_best_nonhold = compact_actual_trigger_local(best_bad);
        G.trigger_reason = dominant_actual_reason_local(best_bad);
        if force_latched_hold
            G.trigger_reason = 'latched_recovery';
        end
        G.hold_reason = dominant_actual_reason_local(G);
        if ~isempty(best_bad)
            G.best_nonhold_violation = getfield_def(best_bad,'violation',NaN);
        end
        return;
    end

    G = best_bad;
    G.a_nom = a_nom;
    G.a_clip = max(a_nom - G.a, 0);
    G.backtrack_iters = n_eval;
    G.floor_infeasible = true;
    G.freeze_on_infeasible = false;
    G = annotate_actual_selection_local(G, info_nom, requested_floor, ...
        recovery_floor, a_prev, 'least_violation_no_freeze');
end

function G = annotate_actual_selection_local(G, info_nom, requested_floor, recovery_floor, a_prev, mode)
    G.requested_alpha_floor = requested_floor;
    G.recovery_alpha_floor = recovery_floor;
    G.below_requested_floor = isfield(G,'a') && isfinite(G.a) && ...
                              (G.a < requested_floor - 1e-12);
    G.recovery_from_freeze = abs(double(a_prev)) <= 1e-12;
    G.actual_search_mode = mode;
    G.nominal_ok = logical(getfield_def(info_nom,'ok',false));
    G.nominal_violation = getfield_def(info_nom,'violation',NaN);
    G.nominal_reason = dominant_actual_reason_local(info_nom);
end

function D = compact_actual_trigger_local(info)
    if ~isstruct(info) || isempty(fieldnames(info))
        D = struct('a',NaN,'violation',NaN,'reason','unknown');
        return;
    end
    flow = getfield_def(info,'flow',struct());
    joint_viability = getfield_def(info,'joint_limit_viability',struct());
    D = struct('a',getfield_def(info,'a',NaN), ...
               'violation',getfield_def(info,'violation',NaN), ...
               'tau_ratio',getfield_def(info,'tau_ratio',NaN), ...
               'qd_ratio',getfield_def(info,'qd_ratio',NaN), ...
               'qdd_ratio',getfield_def(info,'qdd_ratio',NaN), ...
               'qpos_violation_rad',getfield_def(info,'qpos_violation_rad',NaN), ...
               'qpos_planning_excess_rad', ...
                    getfield_def(info,'qpos_planning_excess_rad',NaN), ...
               'qpos_planning_tolerance_rad', ...
                    getfield_def(info,'qpos_planning_tolerance_rad',NaN), ...
               'qpos_planning_numeric_tolerance_rad', ...
                    getfield_def(info, ...
                        'qpos_planning_numeric_tolerance_rad',NaN), ...
               'qpos_audit_tolerance_rad', ...
                    getfield_def(info,'qpos_audit_tolerance_rad',NaN), ...
               'stopping_violation_rad', ...
                    getfield_def(joint_viability,'violation_rad',NaN), ...
               'joint_viability_offending_joint', ...
                    getfield_def(joint_viability,'offending_joint',NaN), ...
               'joint_viability_offending_side', ...
                    getfield_def(joint_viability,'offending_side','unknown'), ...
               'Pax_ratio',getfield_def(info,'Pax_ratio',NaN), ...
               'pgrid_ratio',getfield_def(info,'pgrid_ratio',NaN), ...
               'pavg_ratio',getfield_def(info,'pavg_ratio',NaN), ...
               'pdump_ratio',getfield_def(info,'pdump_ratio',NaN), ...
               'Pgrid_W',getfield_def(flow,'Pgrid',NaN), ...
               'Pspill',getfield_def(flow,'Pspill',NaN), ...
               'Pgrid_avg_prev_W',getfield_def(info,'Pgrid_avg_prev',NaN), ...
               'Pgrid_avg_next_W',getfield_def(info,'Pgrid_avg_next',NaN), ...
               'Pavg_cap_W',getfield_def(info,'Pavg_cap',NaN), ...
               'Pavg_excess_W',getfield_def(info,'Pavg_excess_W',NaN), ...
               'pavg_kpi_pass',logical(getfield_def( ...
                    info,'pavg_kpi_pass',false)), ...
               'Pgrid_peak_cap_W',getfield_def(info,'Pgrid_peak_cap',NaN), ...
               'ok_tau',logical(getfield_def(info,'ok_tau',false)), ...
               'ok_motor_envelope',logical(getfield_def(info,'ok_motor_envelope',false)), ...
               'ok_qd',logical(getfield_def(info,'ok_qd',false)), ...
               'ok_qdd',logical(getfield_def(info,'ok_qdd',false)), ...
               'ok_qpos',logical(getfield_def(info,'ok_qpos',false)), ...
               'ok_joint_limit_viability',logical(getfield_def( ...
                    info,'ok_joint_limit_viability',false)), ...
               'ok_Pax',logical(getfield_def(info,'ok_Pax',false)), ...
               'ok_grid_peak',logical(getfield_def(info,'ok_grid_peak',false)), ...
               'ok_grid_avg',logical(getfield_def(info,'ok_grid_avg',false)), ...
               'ok_spill',logical(getfield_def(info,'ok_spill',false)), ...
               'ok_thermal',logical(getfield_def(info,'ok_thermal',false)), ...
               'ok_buffer',logical(getfield_def(info,'ok_buffer',false)), ...
               'reason',dominant_actual_reason_local(info));
end

function reason = dominant_actual_reason_local(info)
    reason = 'unknown';
    if ~isstruct(info) || isempty(fieldnames(info)), return; end
    checks = {'ok_tau','ok_motor_envelope','ok_qd','ok_qdd','ok_qpos', ...
              'ok_joint_limit_viability','ok_Pax','ok_grid_peak', ...
              'ok_spill','ok_thermal','ok_buffer'};
    names  = {'tau','motor_torque_speed','qd_actual','qdd_actual', ...
              'joint_position_step','joint_limit_braking_viability', ...
              'axis_power','grid_peak', ...
              'spill','brake_thermal','buffer_energy'};
    for ii = 1:numel(checks)
        if isfield(info,checks{ii}) && ~isempty(info.(checks{ii})) && ~logical(info.(checks{ii}))
            reason = names{ii};
            return;
        end
    end
    if logical(getfield_def(info,'ok',false)), reason = 'feasible'; end
end

function [ok, info] = actual_step_feasible_local(a, a_prev, q_r, dq_r, ddq_r, q_meas, dq_meas, Kp, Kd, robot, caps, Pcap, fric, Ts, bus, st_bus, use_state, actualGuard)
    tol = 1e-9;
    qpos_tol = double(actualGuard.position_tolerance_rad);
    qpos_planning_tol = double( ...
        actualGuard.position_planning_tolerance_rad);
    qpos_planning_numeric_tol = double( ...
        actualGuard.joint_limit_viability.numeric_tolerance_rad);

    q_r    = row(q_r);
    dq_r   = row(dq_r);
    ddq_r  = row(ddq_r);
    q_meas = row(q_meas);
    dq_meas= row(dq_meas);

    nJ = numel(q_r);
    tau_lim = ensure_row_len_local(caps.tau_max,    nJ);
    qd_lim  = ensure_row_len_local(caps.qd_max,     nJ);
    qdd_lim = ensure_row_len_local(getfield_def(caps,'qdd_cmd_max_actual', getfield_def(caps,'qdd_max', inf(1,nJ))), nJ);
    Pax_lim = ensure_row_len_local(caps.P_axis_max, nJ);
    q_min   = ensure_row_len_local(getfield_def(caps,'q_min',-inf(1,nJ)), nJ);
    q_max   = ensure_row_len_local(getfield_def(caps,'q_max', inf(1,nJ)), nJ);

    runtime_alpha_upper = double(getfield_def(actualGuard, ...
        'runtime_alpha_upper',actualGuard.search.alpha_max));
    candidate_alpha_max = min(double(actualGuard.search.alpha_max), ...
        runtime_alpha_upper);

    a = clamp(a, 0, candidate_alpha_max);
    dq_cmd = a * dq_r;
    if getfield_def(actualGuard,'force_zero_adot',false)
        adot = 0.0;
        ddq_cmd = zeros(1,nJ);
    else
        adot = (a - a_prev) / max(Ts, 1e-12);
        ddq_cmd = a*a * ddq_r + adot * dq_r;
    end

    current_state_safe_hold = logical(getfield_def( ...
        actualGuard,'current_state_safe_hold',false));
    position_gain_scale = clamp(double(getfield_def( ...
        actualGuard,'hold_position_gain_scale',1.0)),0,1);
    damping_gain_scale = clamp(double(getfield_def( ...
        actualGuard,'hold_damping_gain_scale',1.0)),0,1);
    q_feedback_ref = q_r;
    e = q_r-q_meas;
    ed = dq_cmd-dq_meas;

    
    use_feedback_tau = logical(actualGuard.track_feedback) || ...
        current_state_safe_hold;
    if use_feedback_tau
        if current_state_safe_hold
            q_feedback_ref = q_meas + ...
                position_gain_scale*(q_r-q_meas);
            e = q_feedback_ref-q_meas;
            ed = -damping_gain_scale*dq_meas;
        end
        v  = ddq_cmd + row(Kd).*ed + row(Kp).*e;
        Mq = massMatrix(robot, q_meas);
        c  = velocityProduct(robot, q_meas, dq_meas);
        g  = gravityTorque(robot, q_meas);
        tau = (Mq * v(:) + c(:) + g(:)).';
        tau = tau + row(fric.B).*dq_meas + row(fric.Fc).*tanh(dq_meas./max(fric.vel_eps,1e-9));
    else
        tau = row(inverseDynamics(robot, q_r, dq_cmd, ddq_cmd));
        tau = tau + row(fric.B).*dq_cmd + row(fric.Fc).*tanh(dq_cmd./max(fric.vel_eps,1e-9));
    end

    % Predict the state that this candidate torque actually produces.  A
    % command-side dq/ddq check misses feedback acceleration caused by
    % tracking error and cannot protect the next joint-position sample.
    try
        if use_feedback_tau
            % The feedback command above used this identical measured state.
            % Reuse those exact dynamics; retain the original forward solve.
            Mact = Mq; cact = c; gact = g;
        else
            Mact = massMatrix(robot,q_meas);
            cact = velocityProduct(robot,q_meas,dq_meas);
            gact = gravityTorque(robot,q_meas);
        end
        fact = row(fric.B).*dq_meas + ...
               row(fric.Fc).*tanh(dq_meas./max(fric.vel_eps,1e-9));
        ddq_actual = (Mact \ (tau(:)-cact(:)-gact(:)-fact(:))).';
    catch
        ddq_actual = inf(1,nJ);
    end
    dq_mid_actual = dq_meas + 0.5*Ts*ddq_actual;
    dq_next_actual = dq_meas + Ts*ddq_actual;
    q_next_actual = q_meas + Ts*dq_mid_actual;
    [q_step_min,q_step_max] = quadratic_position_extrema_local( ...
        q_meas,dq_meas,ddq_actual,Ts);
    joint_viability = spinn3d_joint_limit_viability( ...
        q_next_actual,dq_next_actual,q_min,q_max,qdd_lim, ...
        actualGuard.joint_limit_viability,qpos_planning_tol);

    pv = lower(strtrim(char(actualGuard.power_velocity)));
    switch pv
        case {'midpoint','mid','dq_mid'}
            dq_pwr = dq_mid_actual;
        case {'measured','actual'}
            dq_pwr = dq_meas;
        case {'commanded','cmd'}
            dq_pwr = dq_cmd;
        case {'maxabs','max'}
            dq_pwr = dq_cmd;
            for jj = 1:nJ
                if abs(dq_mid_actual(jj)) > abs(dq_pwr(jj)), dq_pwr(jj) = dq_mid_actual(jj); end
                if abs(dq_next_actual(jj)) > abs(dq_pwr(jj)), dq_pwr(jj) = dq_next_actual(jj); end
                if abs(dq_meas(jj)) > abs(dq_pwr(jj)), dq_pwr(jj) = dq_meas(jj); end
            end
        otherwise
            error('spinn3d:ActualGuardPowerVelocity', ...
                'Unsupported actual_guard.power_velocity ''%s''.', ...
                char(actualGuard.power_velocity));
    end

    if use_state
        bus=spinn3d_bus_with_state(bus,st_bus,Ts);
        [flow, st_next] = spinn3d_bus_step(tau, dq_pwr, bus, st_bus, Ts);
    else
        bus_nostate = bus;
        bus_nostate.enable_state = false;
        [flow, st_next] = spinn3d_bus_step(tau, dq_pwr, bus_nostate, [], Ts);
    end
    interval_electric=spinn3d_motor_interval_audit(tau,dq_meas,dq_next_actual,bus);
    Pax=interval_electric.Paxis_motoring_peak_W;
    Pgrid_hard_bound=interval_electric.Pgrid_admission_upper_bound_W;

    [Pgrid_avg_next, ok_avg_guard, Pavg_cap, Pinst_cap] = spinn3d_bus_avg_guard(flow.Pgrid, Pcap, bus, Ts);
    Emin = min(max(double(getfield_def(bus,'Ebuf_min_J',0.0)), 0.0), double(getfield_def(bus,'Ebuf_max_J',0.0)));
    ok_grid_peak = (Pgrid_hard_bound <= Pinst_cap + tol);
    % Filtered average power is a soft performance KPI.  Keep its target
    % status in telemetry, but never use it to reject an otherwise physical

    ok_grid_avg  = logical((~isfinite(Pavg_cap)) || ...
        (Pgrid_avg_next <= Pavg_cap + tol));
    ok_spill     = (flow.Pspill <= tol);
    if use_state && ~strcmp(bus.model_scope,'dynamic_capacitor_common_dc_v1')
        ok_thermal = (st_next.Tbrk <= bus.Tbrk_max_C + tol);
        ok_buffer  = (st_next.Ebuf >= Emin - tol) && ...
                     (st_next.Ebuf <= bus.Ebuf_max_J + tol);
    else
        ok_thermal = true;
        ok_buffer  = true;
    end
    if strcmp(bus.model_scope,'dynamic_capacitor_common_dc_v1')
        ok_thermal=flow.thermal_ok; ok_buffer=flow.voltage_ok;
    end
    ok_power = ok_grid_peak && ok_spill && ok_thermal && ok_buffer && ...
        interval_electric.dynamic_state_pass && ...
        logical(getfield_def(flow,'state_pass',true));

    tau_ratio = max_ratio_local(abs(tau), tau_lim);
    qd_actual_peak = max([abs(dq_meas);abs(dq_mid_actual);abs(dq_next_actual)],[],1);
    qd_ratio  = max_ratio_local(qd_actual_peak, qd_lim);
    qdd_ratio = max_ratio_local(abs(ddq_actual), qdd_lim);
    qd_cmd_ratio = max_ratio_local(abs(dq_cmd),qd_lim);
    qdd_cmd_ratio = max_ratio_local(abs(ddq_cmd),qdd_lim);
    pax_ratio = max_ratio_local(Pax, Pax_lim);
    motor_ratio_vec = zeros(1,nJ);
    if strcmpi(char(getfield_def(bus,'model','legacy_mechanical')),'st3215_pwm')
        motor_ratio_vec=ensure_row_len_local(interval_electric.torque_speed_ratio,nJ);
    end
    motor_ratio = max(motor_ratio_vec);
    pgrid_ratio = Pgrid_hard_bound / max(Pinst_cap, 1e-12);
    pavg_ratio  = Pgrid_avg_next / max(Pavg_cap, 1e-12);
    pdump_cap = getfield_def(bus,'P_brk_peak', getfield_def(bus,'P_dump_max', inf));
    pdump_ratio = flow.Pdump / max(pdump_cap, 1e-12);
    if ~isfinite(pdump_cap), pdump_ratio = 0; end

    ok_tau = all(abs(tau)     <= tau_lim + tol);
    ok_motor_envelope = all(motor_ratio_vec <= 1.0 + tol);
    ok_qd  = all(isfinite(qd_actual_peak)) && all(qd_actual_peak <= qd_lim + tol);
    ok_qdd = all(isfinite(ddq_actual)) && all(abs(ddq_actual) <= qdd_lim + tol);
    % This controller chooses among alpha candidates, so accepting the full

    % Construct controller commands against the physical q limits; retain
    % only the much smaller viability numeric tolerance for arithmetic
    % roundoff.  The authoritative post-noise and hard audits still own
    % qpos_tol and may use it solely when checking an already formed state.
    ok_qpos = all(isfinite([q_step_min,q_step_max])) && ...
              all(q_step_min >= q_min-qpos_planning_numeric_tol) && ...
              all(q_step_max <= q_max+qpos_planning_numeric_tol);
    ok_joint_limit_viability = logical(joint_viability.pass);
    ok_Pax = all(Pax          <= Pax_lim + tol);
    ok = ok_tau && ok_motor_envelope && ok_qd && ok_qdd && ok_qpos && ...
         ok_joint_limit_viability && ok_Pax && ok_power;

    if all(isfinite([q_step_min,q_step_max]))
        qlo_v = q_min-q_step_min;
        qhi_v = q_step_max-q_max;
        qlo_v(~isfinite(q_min)) = -inf;
        qhi_v(~isfinite(q_max)) = -inf;
        qpos_violation = max([0,qlo_v,qhi_v]);
    else
        qpos_violation = inf;
    end
    qpos_planning_excess = max( ...
        qpos_violation-qpos_planning_numeric_tol,0);
    violation = max([tau_ratio-1, motor_ratio-1, qd_ratio-1, qdd_ratio-1, pax_ratio-1, ...
                     qpos_planning_excess, ...
                     double(joint_viability.violation_rad), ...
                     pgrid_ratio-1, ...
                     pdump_ratio-1, max(flow.Pspill,0)]);
    violation = max(0, violation);

    info = struct();
    info.ok = ok;
    info.a = a;
    info.adot = adot;
    info.dq = dq_cmd;
    info.ddq = ddq_cmd;
    info.ddq_actual = ddq_actual;
    info.dq_mid_actual = dq_mid_actual;
    info.dq_next_actual = dq_next_actual;
    info.q_next_actual = q_next_actual;
    info.q_step_min_actual = q_step_min;
    info.q_step_max_actual = q_step_max;
    info.joint_limit_viability = joint_viability;
    info.tau = tau;
    info.Pax = Pax;
    info.motor_torque_speed_ratio = motor_ratio_vec;
    info.flow = flow;
    info.Pgrid_raw = flow.Pgrid;
    info.Pgrid_hard_upper_bound_W=Pgrid_hard_bound;
    info.electrical_interval=interval_electric;
    info.Pgrid_avg_prev = getfield_def(bus,'Pgrid_avg_prev',NaN);
    info.Pgrid_avg_next = Pgrid_avg_next;
    info.Pavg_cap = Pavg_cap;
    info.Pavg_margin_W = Pavg_cap - Pgrid_avg_next;
    info.Pavg_excess_W = max(Pgrid_avg_next-Pavg_cap,0.0);
    info.pavg_kpi_pass = ok_grid_avg;
    info.Pavg_is_hard_constraint = false;
    info.Pgrid_peak_cap = Pinst_cap;
    info.st_next = st_next;
    info.track_feedback = use_feedback_tau;
    info.current_state_safe_hold = current_state_safe_hold;
    info.position_gain_scale = position_gain_scale;
    info.damping_gain_scale = damping_gain_scale;
    info.q_feedback_ref = q_feedback_ref;
    info.power_velocity = pv;
    info.q_err_norm = norm(e);
    info.dq_err_norm = norm(ed);
    info.tau_ratio = tau_ratio;
    info.motor_ratio = motor_ratio;
    info.qd_ratio = qd_ratio;
    info.qdd_ratio = qdd_ratio;
    info.qd_cmd_ratio = qd_cmd_ratio;
    info.qdd_cmd_ratio = qdd_cmd_ratio;
    info.qpos_violation_rad = qpos_violation;
    info.qpos_planning_excess_rad = qpos_planning_excess;
    info.qpos_planning_tolerance_rad = qpos_planning_tol;
    info.qpos_planning_numeric_tolerance_rad = ...
        qpos_planning_numeric_tol;
    info.qpos_audit_tolerance_rad = qpos_tol;
    info.Pax_ratio = pax_ratio;
    info.pgrid_ratio = pgrid_ratio;
    info.pavg_ratio = pavg_ratio;
    info.pdump_ratio = pdump_ratio;
    info.violation = violation;
    info.ok_tau = ok_tau;
    info.ok_motor_envelope = ok_motor_envelope;
    info.ok_qd = ok_qd;
    info.ok_qdd = ok_qdd;
    info.ok_qpos = ok_qpos;
    info.ok_joint_limit_viability = ok_joint_limit_viability;
    info.ok_Pax = ok_Pax;
    info.ok_grid_peak = ok_grid_peak;
    info.ok_grid_avg = ok_grid_avg;
    info.ok_grid_avg_guard_legacy = logical(ok_avg_guard);
    info.ok_spill = ok_spill;
    info.ok_thermal = ok_thermal;
    info.ok_buffer = ok_buffer;
end

function [qmin_step,qmax_step] = quadratic_position_extrema_local( ...
        q0,dq0,ddq,Ts)
% Exact extrema of q(t)=q0+dq0*t+0.5*ddq*t^2 on one ZOH plant step.
q0 = row(q0);
dq0 = row(dq0);
ddq = row(ddq);
q1 = q0 + double(Ts)*dq0 + 0.5*double(Ts)^2*ddq;
qmin_step = min(q0,q1);
qmax_step = max(q0,q1);
for j = 1:numel(q0)
    if isfinite(ddq(j)) && abs(ddq(j))>0
        t_star = -dq0(j)/ddq(j);
        if isfinite(t_star) && t_star>0 && t_star<double(Ts)
            q_star = q0(j) + dq0(j)*t_star + ...
                0.5*ddq(j)*t_star*t_star;
            qmin_step(j) = min(qmin_step(j),q_star);
            qmax_step(j) = max(qmax_step(j),q_star);
        end
    end
end
end

function dq_mid = predict_midpoint_velocity_local(tau, q_meas, dq_meas, robot, fric, Ts)
    % Midpoint velocity used for electromechanical power P=tau*dq.  The
    % torque is the same CTC torque that will be applied at the beginning of
    % this servo sample; dq_mid avoids mixing start-of-step torque with
    % end-of-step velocity in raw-peak and filtered-budget guards.
    tau = row(tau);
    q_meas = row(q_meas);
    dq_meas = row(dq_meas);
    try
        Mq = massMatrix(robot, q_meas);
        c  = velocityProduct(robot, q_meas, dq_meas);
        g  = gravityTorque(robot, q_meas);
        tau_fric = row(fric.B).*dq_meas + row(fric.Fc).*tanh(dq_meas./max(fric.vel_eps,1e-9));
        ddq_act = (Mq \ (tau(:) - c(:) - g(:) - tau_fric(:))).';
        dq_mid = dq_meas + 0.5 * max(double(Ts),1e-12) * ddq_act;
    catch
        dq_mid = dq_meas;
    end
end

function da = estimate_alpha_step_cap_local(dq_r,ddq_r,caps,Ts,search)
    % Conservative alpha increment scale implied by qdd_cmd limit.  This is
    % used only to seed the search grid; actual feasibility is still checked
    % by actual_step_feasible_local.
    dq_r = abs(double(dq_r(:)));
    ddq_r = abs(double(ddq_r(:)));
    nJ = max(numel(dq_r), numel(ddq_r));
    qdd_lim = ensure_row_len_local(getfield_def(caps,'qdd_cmd_max_actual', getfield_def(caps,'qdd_max', inf(1,nJ))), nJ);
    qdd_lim = abs(double(qdd_lim(:)));
    dq_r = dq_r(1:min(numel(dq_r),numel(qdd_lim)));
    qdd_lim = qdd_lim(1:numel(dq_r));
    velocity_epsilon = ...
        double(search.step_estimate_reference_velocity_epsilon);
    time_epsilon = double(search.step_estimate_time_epsilon_s);
    good = isfinite(qdd_lim) & qdd_lim > 0 & ...
        dq_r > velocity_epsilon;
    if any(good)
        da = double(search.step_estimate_scale) * ...
            min(qdd_lim(good).*max(Ts,time_epsilon)./dq_r(good));
    else
        da = double(search.step_estimate_no_velocity);
    end
    if ~isfinite(da) || da <= 0
        da = double(search.step_estimate_invalid);
    end
    da = min(max(da,double(search.step_estimate_min)), ...
        double(search.step_estimate_max));
end

function r = max_ratio_local(num, den)
    num = double(num(:));
    den = double(den(:));
    if isempty(num)
        r = 0;
        return;
    end
    if isscalar(den)
        den = repmat(den, numel(num), 1);
    end
    den = den(1:min(numel(den),numel(num)));
    num = num(1:numel(den));
    finite = isfinite(den) & den > 0;
    if ~any(finite)
        r = 0;
    else
        r = max(num(finite) ./ den(finite));
    end
end

function v = ensure_row_len_local(v, n)
    if isempty(v)
        v = inf(1,n);
        return;
    end
    v = double(v(:).');
    if isscalar(v)
        v = repmat(v,1,n);
    elseif numel(v) < n
        v = [v, repmat(v(end),1,n-numel(v))];
    else
        v = v(1:n);
    end
end

function Pgrid = bus_power_grid(tau, dq, bus)
    % Dynamic previews use the current state and one actual plant interval.
    if isfield(bus,'model_scope') && ...
            strcmp(bus.model_scope,'dynamic_capacitor_common_dc_v1')

        f=spinn3d_bus_step(tau,dq,bus,bus.dc_state,bus.dc_step_s);
        Pgrid=f.Pgrid;
    else
        [~, ~, Pgrid] = spinn3d_bus_power(tau,dq,bus);
    end
end

function m = min_safe(v), m = min(v(:)); if isempty(m), m = 1.0; end, end

function [a, a_slew, a_filt, adot, G] = alpha_guard_backtrack(a0, a_prev, Ts, q, dq_ref, ddq_ref, robot, caps, Pcap, fric, areg, bus, alpha_max) %#ok<INUSD>

    alpha_max = double(alpha_max);
    [a,a_slew,a_filt,adot,G] = spinn3d_alpha_governor_step( ...
        a0,a_prev,areg.BETA,areg.A_DOT_UP,areg.A_DOT_DN,Ts, ...
        dq_ref,ddq_ref,caps,alpha_max);
end

function S = soft_power_contract_local(opts,Pcap,policy_dt,alpha_mode)
% Validate the complete central soft-power contract when it is supplied.
% Callers that do not opt into this performance layer retain the existing
% controller exactly; the Joint demo always supplies cfg.SOFT_POWER.
S = struct('present',false,'enable',false,'enabled_for_method',false, ...
    'method_id','','schema','disabled','target_W',NaN, ...
    'internal_margin_W',0,'internal_target_W',NaN, ...
    'preview_horizon_s',0,'candidate_count',0,'speed_margin_s',0, ...
    'alpha_max',NaN, ...
    'terminal_stable_time_s',0, ...
    'soft_excess_tolerance',0,'apply_methods',{{}}, ...
    'noguard_t_reach_s',NaN,'arrival_deadline_s',NaN, ...
    'path_deadline_s',NaN,'deadline_s',NaN);
if ~isstruct(opts) || ~isfield(opts,'softPower') || isempty(opts.softPower)
    return;
end
X = opts.softPower;
required = {'enable','schema','target_W','internal_margin_W', ...
    'preview_horizon_s','candidate_count','speed_margin_s', ...
    'terminal_stable_time_s','soft_excess_tolerance','apply_methods', ...
    'alpha_max'};

numeric_positive = {'target_W','preview_horizon_s'};
for i = 1:numel(numeric_positive)
    f = numeric_positive{i};

end
numeric_nonnegative = {'internal_margin_W','speed_margin_s', ...
    'terminal_stable_time_s', ...
    'soft_excess_tolerance'};
for i = 1:numel(numeric_nonnegative)
    f = numeric_nonnegative{i};

end

tol_target = max(1e-12,64*eps(max([1,abs(Pcap), ...
    abs(double(X.target_W))])));

ratio_h = double(X.preview_horizon_s)/policy_dt;

methods = X.apply_methods;
if isstring(methods), methods = cellstr(methods(:)); end

methods = cellfun(@(c)lower(strtrim(c)),methods,'UniformOutput',false);

method_id = '';
if isfield(opts,'method_id') && ~isempty(opts.method_id)

    method_id = lower(strtrim(char(opts.method_id)));
end
if logical(X.enable)

end
enabled_for_method = logical(X.enable) && any(strcmp(method_id,methods));
if enabled_for_method

    
end

S.present = true;
S.enable = logical(X.enable);
S.enabled_for_method = logical(enabled_for_method);
S.method_id = method_id;
S.schema = char(X.schema);
S.alpha_max = double(X.alpha_max);
S.target_W = double(X.target_W);
S.internal_margin_W = double(X.internal_margin_W);
S.internal_target_W = S.target_W-S.internal_margin_W;
S.preview_horizon_s = double(X.preview_horizon_s);
S.candidate_count = double(X.candidate_count);
S.speed_margin_s = double(X.speed_margin_s);
S.terminal_stable_time_s = double(X.terminal_stable_time_s);
S.soft_excess_tolerance = double(X.soft_excess_tolerance);
S.apply_methods = methods;
if isfield(X,'noguard_t_reach_s') && ~isempty(X.noguard_t_reach_s)
    S.noguard_t_reach_s = double(X.noguard_t_reach_s);
    S.arrival_deadline_s = ...
        S.noguard_t_reach_s-S.speed_margin_s;
    % Path completion must leave the same consecutive terminal-stability
    % interval used by the full-plant arrival clock.  Without this reserve,
    % a path finishing 10 ms before NoGuard would still arrive roughly
    % 90 ms later after the mandatory 0.10-s stable hold.
    S.path_deadline_s = S.arrival_deadline_s- ...
        S.terminal_stable_time_s;
    S.deadline_s = S.path_deadline_s; % compatibility/telemetry alias
end
end

function D = soft_power_diag_default_local(a,S)
if nargin<1 || isempty(a) || ~isfinite(double(a)), a=NaN; end
if nargin<2 || ~isstruct(S)
    S=struct('enabled_for_method',false,'target_W',NaN, ...
        'internal_target_W',NaN,'terminal_stable_time_s',NaN, ...
        'arrival_deadline_s',NaN,'path_deadline_s',NaN, ...
        'deadline_s',NaN);
end
D = struct( ...
    'schema','elastic_soft_pavg_governor_telemetry_v1', ...
    'enabled',logical(getfield_def(S,'enabled_for_method',false)), ...
    'applied',false,'selection_mode','disabled', ...
    'requested_alpha',double(a),'selected_alpha',double(a), ...
    'speed_floor_alpha',NaN,'speed_floor_unmet_requested',false, ...
    'predicted_Pavg_peak_W',NaN,'predicted_Pavg_final_W',NaN, ...
    'predicted_soft_excess_peak_W',NaN, ...
    'predicted_mean_normalized_squared_excess',NaN, ...
    'predicted_internal_mean_normalized_squared_excess',NaN, ...
    'predicted_path_progress_end',NaN, ...
    'predicted_grid_energy_J',NaN, ...
    'target_W',double(getfield_def(S,'target_W',NaN)), ...
    'internal_target_W',double(getfield_def(S,'internal_target_W',NaN)), ...
    'terminal_stable_time_s',double(getfield_def( ...
        S,'terminal_stable_time_s',NaN)), ...
    'arrival_deadline_s',double(getfield_def( ...
        S,'arrival_deadline_s',NaN)), ...
    'path_deadline_s',double(getfield_def(S,'path_deadline_s',NaN)), ...
    'deadline_s',double(getfield_def(S,'deadline_s',NaN)), ...
    'internal_target_met',false,'speed_feasible',false, ...
    'elastic_flag',false,'preview_physical_ok',false, ...
    'candidate_evaluations',0,'physical_candidate_count',0, ...
    'speed_candidate_count',0);
end

function [a_selected,D] = soft_power_select_local( ...
        a_requested,t,s,q_meas,dq_meas,traj,nn,robot,Kp,Kd,caps, ...
        Pcap,fric,bus,state,areg,Ts,policy_dt,S,actualGuard)
a_requested = clamp(double(a_requested),nn.alpha_floor,nn.alpha_max);
D = soft_power_diag_default_local(a_requested,S);
a_selected = a_requested;
if ~logical(S.enabled_for_method)
    return;
end
D.enabled = true;
if logical(getfield_def(state,'recovery_latched',false))
    D.selection_mode = 'bypass_recovery_latched';
    return;
end
Tf = double(traj.Tf);
if s>=Tf-1e-12
    D.selection_mode = 'bypass_terminal_path';
    D.speed_floor_alpha = 0;
    D.speed_feasible = true;
    return;
end
deadline = double(S.path_deadline_s);
time_left = deadline-double(t);
if time_left<=0
    speed_floor = nn.alpha_max;
else
    speed_floor = clamp((Tf-double(s))/time_left,0,nn.alpha_max);
end
D.speed_floor_alpha = speed_floor;
D.speed_floor_unmet_requested = a_requested+1e-12<speed_floor;

% Fast path: when the uncropped command already meets the conservative
% internal target, it is necessarily the fastest member of a crop-only
% candidate set.  This avoids the full 33-candidate scan over most benign
% policy ticks while preserving exactly the same lexicographic decision.
requested_preview = soft_power_preview_candidate_local( ...
    a_requested,t,s,q_meas,dq_meas,traj,nn,robot,Kp,Kd,caps,Pcap, ...
    fric,bus,state,areg,Ts,policy_dt,S,actualGuard);
if requested_preview.physical_ok && ...
        requested_preview.internal_target_met
    D.applied = false;
    if requested_preview.speed_feasible
        D.selection_mode = 'requested_internal_target_met_fastest';
    else
        D.selection_mode = ...
            'requested_internal_target_met_but_speed_proxy_infeasible';
    end
    D.predicted_Pavg_peak_W = requested_preview.Pavg_peak_W;
    D.predicted_Pavg_final_W = requested_preview.Pavg_final_W;
    D.predicted_soft_excess_peak_W = ...
        requested_preview.target_excess_peak_W;
    D.predicted_mean_normalized_squared_excess = ...
        requested_preview.mean_normalized_squared_excess;
    D.predicted_internal_mean_normalized_squared_excess = ...
        requested_preview.internal_mean_normalized_squared_excess;
    D.predicted_path_progress_end = requested_preview.s_end;
    D.predicted_grid_energy_J = requested_preview.Egrid_J;
    D.internal_target_met = true;
    D.speed_feasible = logical(requested_preview.speed_feasible);
    % There is no faster member in this crop-only overlay.  Preserve the
    % requested command, but expose a speed miss as an elastic performance
    % miss instead of silently labelling the joint objective fully feasible.
    D.elastic_flag = ~logical(requested_preview.speed_feasible);
    D.preview_physical_ok = true;
    D.candidate_evaluations = 1;
    D.speed_candidate_count = double(requested_preview.speed_feasible);
    return;
end

% This is a crop-only overlay: it never accelerates above the learned/CEM
% command.  If that command itself lies below the deadline-derived average
% floor, retain it and expose the miss in telemetry rather than inventing a
% new, unaudited command.
candidate_floor = max(nn.alpha_floor,min(a_requested,speed_floor));
if a_requested<=candidate_floor+double(S.soft_excess_tolerance)
    candidates = a_requested;
else
    candidates = linspace(candidate_floor,a_requested,S.candidate_count);
    a_prev_in = clamp(double(state.a_prev),candidate_floor,a_requested);
    candidates = unique([candidates,a_prev_in,a_requested],'sorted');
end

empty_preview = soft_power_preview_default_local();
P = repmat(empty_preview,numel(candidates),1);
for i = 1:numel(candidates)
    if abs(double(candidates(i))-a_requested)<= ...
            max(1e-12,double(S.soft_excess_tolerance))
        P(i) = requested_preview;
    else
        P(i) = soft_power_preview_candidate_local( ...
            double(candidates(i)),t,s,q_meas,dq_meas,traj,nn,robot, ...
            Kp,Kd,caps,Pcap,fric,bus,state,areg,Ts,policy_dt,S, ...
            actualGuard);
    end
end
D.candidate_evaluations = numel(candidates);
physical_idx = find([P.physical_ok]);
D.physical_candidate_count = numel(physical_idx);
if isempty(physical_idx)
    % Fail open only with respect to the soft objective.  The unchanged

    D.selection_mode = 'soft_preview_no_physical_candidate_hard_guard_owns';
    D.elastic_flag = true;
    return;
end

speed_idx = physical_idx([P(physical_idx).speed_feasible]);
D.speed_candidate_count = numel(speed_idx);
if isempty(speed_idx)
    pool = physical_idx;
    speed_pool_available = false;
else
    pool = speed_idx;
    speed_pool_available = true;
end

internal_ok = [P(pool).internal_target_met];
if any(internal_ok)
    pool2 = pool(internal_ok);
    chosen = soft_power_fastest_candidate_local( ...
        pool2,P,candidates,S.soft_excess_tolerance);
    selection_mode = 'internal_target_met_fastest';
    elastic = false;
else
    values = [P(pool).mean_normalized_squared_excess];
    vmin = min(values);
    near = values<=vmin+double(S.soft_excess_tolerance);
    pool2 = pool(near);
    chosen = soft_power_fastest_candidate_local( ...
        pool2,P,candidates,S.soft_excess_tolerance);
    selection_mode = 'elastic_min_normalized_squared_excess_then_fastest';
    elastic = true;
end
if ~speed_pool_available
    % All proxy rollouts miss the terminal catch-up test.  Preserve the
    % most advanced candidate; never transform this soft miss into hold.
    s_end = [P(pool).s_end];
    max_s = max(s_end);
    tol_s = max(1e-12,double(S.soft_excess_tolerance));
    advanced = pool(s_end>=max_s-tol_s);
    if ~isempty(advanced)
        chosen = soft_power_fastest_candidate_local( ...
            advanced,P,candidates,S.soft_excess_tolerance);
    end
    selection_mode = [selection_mode '+speed_proxy_infeasible'];
end

a_selected = clamp(double(candidates(chosen)),nn.alpha_floor,a_requested);
Q = P(chosen);
D.applied = a_selected<a_requested-1e-12;
D.selection_mode = selection_mode;
D.requested_alpha = a_requested;
D.selected_alpha = a_selected;
D.predicted_Pavg_peak_W = Q.Pavg_peak_W;
D.predicted_Pavg_final_W = Q.Pavg_final_W;
D.predicted_soft_excess_peak_W = Q.target_excess_peak_W;
D.predicted_mean_normalized_squared_excess = ...
    Q.mean_normalized_squared_excess;
D.predicted_internal_mean_normalized_squared_excess = ...
    Q.internal_mean_normalized_squared_excess;
D.predicted_path_progress_end = Q.s_end;
D.predicted_grid_energy_J = Q.Egrid_J;
D.internal_target_met = logical(Q.internal_target_met);
D.speed_feasible = logical(Q.speed_feasible);
D.elastic_flag = logical(elastic);
D.preview_physical_ok = logical(Q.physical_ok);
end

function idx = soft_power_fastest_candidate_local(pool,P,candidates,tol)
s_end = [P(pool).s_end];
best_s = max(s_end);
tol_s = max(1e-12,double(tol));
pool2 = pool(s_end>=best_s-tol_s);
if numel(pool2)>1
    egrid = [P(pool2).Egrid_J];
    emin = min(egrid);
    etol = max(1e-12,double(tol));
    pool2 = pool2(egrid<=emin+etol);
end
if numel(pool2)>1
    [~,j] = max(candidates(pool2));
    idx = pool2(j);
else
    idx = pool2(1);
end
end

function P = soft_power_preview_default_local()
P = struct('physical_ok',false,'failure_reason','not_evaluated', ...
    'speed_feasible',false,'internal_target_met',false, ...
    'Pavg_peak_W',NaN,'Pavg_final_W',NaN, ...
    'target_excess_peak_W',NaN, ...
    'mean_normalized_squared_excess',inf, ...
    'internal_mean_normalized_squared_excess',inf, ...
    's_end',NaN,'Egrid_J',inf,'elapsed_s',0);
end

function P = soft_power_preview_candidate_local( ...
        alpha_cap,t0,s0,q0,dq0,traj,nn,robot,Kp,Kd,caps,Pcap, ...
        fric,bus,state,areg,Ts,policy_dt,S,actualGuard)
% Deterministic measured-state prediction.  The 100-Hz desired command is
% held for one policy interval, but governor slew/filtering, CTC, plant and
% BUS state are propagated at every 1-ms plant substep.  This is essential
% when A_DOT_UP is small: the command cannot be replaced by a_des after one
% sample.  BUS routing always calls spinn3d_bus_step, so signed
% regeneration/direct sharing is preserved and no monotonic alpha-power
% assumption is introduced.
P = soft_power_preview_default_local();
alpha_cap = clamp(double(alpha_cap),nn.alpha_floor,nn.alpha_max);
q = row(double(q0));
dq = row(double(dq0));
s = double(s0);
a_prev = double(state.a_prev);
pavg = double(state.Pgrid_avg_lp);
if logical(state.use_bus_state), st = state.bus; else, st = []; end
remaining = double(S.preview_horizon_s);
elapsed = 0;
pavg_peak = pavg;
target_excess_peak = max(pavg-S.target_W,0);
target_sq_int = 0;
internal_sq_int = 0;
Egrid = 0;
physical_ok = true;
failure_reason = 'feasible';

preview_tol = max(1e-12,1e-9*S.preview_horizon_s);
while remaining>preview_tol
    rho = min(1.0,s/max(double(traj.Tf),1e-12));
    if s>=double(traj.Tf)-1e-12
        a_des = 0.0;
    else
        a_plan_future = clamp(plan_lookup(nn.plan,rho), ...
            nn.alpha_floor,nn.alpha_max);
        a_des = min(alpha_cap,a_plan_future);
    end
    dt_policy = min(policy_dt,remaining);
    policy_remaining = dt_policy;
    while policy_remaining>preview_tol
        dt_step = min([Ts,policy_remaining,remaining]);
        a_next = governor_step(a_des,a_prev,areg.BETA, ...
            areg.A_DOT_UP,areg.A_DOT_DN,dt_step);
        adot_step = (a_next-a_prev)/max(dt_step,1e-12);
        [ok,reason,q,dq,s,st,pavg,flow] = ...
            soft_power_preview_step_local( ...
            q,dq,s,a_prev,a_next,adot_step,dt_step,traj,robot,Kp,Kd, ...
            caps,Pcap,fric,bus,st,pavg,state.use_bus_state,actualGuard);
        elapsed = elapsed+dt_step;
        remaining = max(0,remaining-dt_step);
        policy_remaining = max(0,policy_remaining-dt_step);
        [target_sq_int,internal_sq_int,Egrid,pavg_peak, ...
            target_excess_peak] = soft_power_accumulate_local( ...
            target_sq_int,internal_sq_int,Egrid,pavg_peak, ...
            target_excess_peak,pavg,flow.Pgrid,dt_step,S);
        a_prev = a_next;
        if ~ok
            physical_ok=false;
            failure_reason=reason;
            break;
        end
    end
    if ~physical_ok
        break;
    end
end

deadline_remaining = max(double(S.deadline_s)-(double(t0)+elapsed),0);
speed_feasible = s+nn.alpha_max*deadline_remaining>= ...
    double(traj.Tf)-1e-9;
P.physical_ok = logical(physical_ok);
P.failure_reason = failure_reason;
P.speed_feasible = logical(speed_feasible);
P.internal_target_met = logical(pavg_peak<= ...
    S.internal_target_W+1e-9);
P.Pavg_peak_W = pavg_peak;
P.Pavg_final_W = pavg;
P.target_excess_peak_W = target_excess_peak;
P.mean_normalized_squared_excess = ...
    target_sq_int/max(elapsed,eps);
P.internal_mean_normalized_squared_excess = ...
    internal_sq_int/max(elapsed,eps);
P.s_end = s;
P.Egrid_J = Egrid;
P.elapsed_s = elapsed;
end

function [target_sq,internal_sq,Egrid,pavg_peak,target_peak] = ...
        soft_power_accumulate_local(target_sq,internal_sq,Egrid, ...
        pavg_peak,target_peak,pavg,Pgrid,dt,S)
target_norm = max(pavg-S.target_W,0)/max(S.target_W,eps);
internal_norm = max(pavg-S.internal_target_W,0)/ ...
    max(S.target_W,eps);
target_sq = target_sq+dt*target_norm^2;
internal_sq = internal_sq+dt*internal_norm^2;
Egrid = Egrid+dt*max(double(Pgrid),0);
pavg_peak = max(pavg_peak,pavg);
target_peak = max(target_peak,max(pavg-S.target_W,0));
end

function [ok,reason,q_next,dq_next,s_next,st_next,pavg_next,flow] = ...
        soft_power_preview_step_local(q,dq,s,a_prev,a_next,adot,dt, ...
        traj,robot,Kp,Kd,caps,Pcap,fric,bus,st,pavg,use_state, ...
        actualGuard)
nJ = numel(q);
Tf = double(traj.Tf);
alpha_max = double(actualGuard.search.alpha_max);
alpha_range_ok = isfinite(alpha_max) && alpha_max>0 && ...
    isfinite(double(a_prev)) && isfinite(double(a_next)) && ...
    double(a_prev)>=-1e-12 && double(a_prev)<=alpha_max+1e-12 && ...
    double(a_next)>=-1e-12 && double(a_next)<=alpha_max+1e-12;
[q_r,dq_r,ddq_r] = traj.eval(min(s,Tf));
q_r=row(q_r); dq_r=row(dq_r); ddq_r=row(ddq_r);
if s>=Tf-1e-12
    dq_r=zeros(1,nJ);
    ddq_r=zeros(1,nJ);
end
dq_cmd=a_next*dq_r;
ddq_cmd=a_next*a_next*ddq_r+adot*dq_r;
v=ddq_cmd+row(Kd).*(dq_cmd-dq)+row(Kp).*(q_r-q);
Mq=massMatrix(robot,q);
c=velocityProduct(robot,q,dq);
g=gravityTorque(robot,q);
f=row(fric.B).*dq+row(fric.Fc).* ...
    tanh(dq./max(fric.vel_eps,1e-9));
tau=(Mq*v(:)+c(:)+g(:)).'+f;
ddq_actual=v;
dq_mid=dq+0.5*dt*ddq_actual;
dq_next=dq+dt*ddq_actual;
q_next=q+dt*dq_mid;
[q_step_min,q_step_max]=quadratic_position_extrema_local( ...
    q,dq,ddq_actual,dt);

if use_state
    bus=spinn3d_bus_with_state(bus,st,dt);
    [flow,st_next]=spinn3d_bus_step(tau,dq_mid,bus,st,dt);
else
    bus0=bus; bus0.enable_state=false;
    [flow,st_next]=spinn3d_bus_step(tau,dq_mid,bus0,[],dt);
end
bus_avg=bus;
bus_avg.Pgrid_avg_prev=pavg;
[pavg_next,~,~,Pgrid_peak_cap]= ...
    spinn3d_bus_avg_guard(flow.Pgrid,Pcap,bus_avg,dt);
s_next=min(Tf,s+dt*max(a_next,0));

tau_lim=ensure_row_len_local(caps.tau_max,nJ);
qd_lim=ensure_row_len_local(caps.qd_max,nJ);
qdd_lim=ensure_row_len_local(getfield_def(caps, ...
    'qdd_cmd_max_actual',getfield_def(caps,'qdd_max',inf)),nJ);
Paxis_lim=ensure_row_len_local(caps.P_axis_max,nJ);
q_min=ensure_row_len_local(getfield_def(caps,'q_min',-inf),nJ);
q_max=ensure_row_len_local(getfield_def(caps,'q_max',inf),nJ);
interval_electric=spinn3d_motor_interval_audit(tau,dq,dq_next,bus);
Paxis=interval_electric.Paxis_motoring_peak_W;
motor_ratio=ensure_row_len_local(interval_electric.torque_speed_ratio,nJ);
qd_peak=max([abs(dq);abs(dq_mid);abs(dq_next)],[],1);
joint_viability=spinn3d_joint_limit_viability( ...
    q_next,dq_next,q_min,q_max,qdd_lim, ...
    actualGuard.joint_limit_viability,0.0);
pdump_cap=getfield_def(bus,'P_brk_peak', ...
    getfield_def(bus,'P_dump_max',inf));
finite_ok=all(isfinite([a_prev,a_next,adot,q_next,dq_next,tau,ddq_actual, ...
    Paxis,motor_ratio,pavg_next,flow.Pgrid,flow.Pdump,flow.Pspill]));
tol=1e-9;
checks=[ ...
    finite_ok, ...
    alpha_range_ok, ...
    all(abs(tau)<=tau_lim+tol), ...
    all(motor_ratio<=1+tol), ...
    all(qd_peak<=qd_lim+tol), ...
    all(abs(ddq_actual)<=qdd_lim+tol), ...
    all(q_step_min>=q_min-tol) && all(q_step_max<=q_max+tol), ...
    logical(joint_viability.pass), ...
    all(Paxis<=Paxis_lim+tol), ...
    interval_electric.Pgrid_admission_upper_bound_W<=Pgrid_peak_cap+tol, ...
    (~isfinite(pdump_cap) || flow.Pdump<=pdump_cap+tol), ...
    flow.Pspill<=tol, ...
    logical(getfield_def(flow,'state_pass',true)) && ...
    interval_electric.dynamic_state_pass];
names={'nonfinite','alpha_range','tau','motor_torque_speed','qd','qdd', ...
    'joint_position_step','joint_limit_viability','axis_power', ...
    'grid_peak','brake_peak','spill','dynamic_dc_state'};
if use_state && ~strcmp(bus.model_scope,'dynamic_capacitor_common_dc_v1')
    checks(end+1)=st_next.Tbrk<=bus.Tbrk_max_C+tol; %#ok<AGROW>
    checks(end+1)=st_next.Ebuf>=bus.Ebuf_min_J-tol && ...
        st_next.Ebuf<=bus.Ebuf_max_J+tol; %#ok<AGROW>
    names=[names,{'brake_thermal','buffer_energy'}]; %#ok<AGROW>
end
ok=all(checks);
if ok
    reason='feasible';
else
    ii=find(~checks,1,'first');
    reason=names{ii};
end
end
