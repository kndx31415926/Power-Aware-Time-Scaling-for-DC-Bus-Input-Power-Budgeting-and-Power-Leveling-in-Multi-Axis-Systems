function plan = spinn3d_alpha_rollout_warmstart(robot, traj, cfg, payload, model_in)
%SPINN3D_ALPHA_ROLLOUT_WARMSTART
% Roll out the current alpha model along the nominal trajectory to build a
% warm-start proposal; measured-state execution still requires admission.
% Source-power/EMA propagation is always active. An
% explicit energy/thermal BUS state is propagated only when bus.enable_state
% is true; otherwise the same stateless source model used by dataset generation
% is evaluated at every plant step.

    if nargin < 5 || isempty(model_in)
        M = load_alpha_model(cfg.MODEL_PATH,'','final');
    elseif ischar(model_in) || isstring(model_in)
        M = load_alpha_model(char(model_in),'','final');
    else
        M = model_in;
    end
    limits = cfg.CAPS;
    if ~isfield(limits,'P_total_max') || isempty(limits.P_total_max)
        limits.P_total_max = cfg.P_TOTAL_MAX;
    end
    Ts          = double(cfg.TS);
    alpha_floor = double(cfg.ALPHA_MIN);
    alpha_max   = double(cfg.ALPHA_MAX);
    itmax       = double(cfg.ALPHA_ITMAX);
    areg        = cfg.ALPHA_REG;
    Pbud        = double(limits.P_total_max);
    [budget_in_range, budget_range, budget_meta_present] = ...
        model_budget_coverage_local(M,Pbud);
    if nargin < 4 || isempty(payload)
        payload = infer_payload_from_cfg(cfg);
    end

    bus = get_bus_from_cfg(cfg, Pbud);
    use_bus_state = isfield(bus,'enable_state') && ~isempty(bus.enable_state) && logical(bus.enable_state);
    [q_hold0, ~, ~] = traj.eval(0);
    power_init = spinn3d_static_hold_power_init( ...
        robot,row(q_hold0),cfg.FRIC,bus,Pbud,Ts);
    tau_lim_hold = expand_limit_local(getfield_def(limits,'tau_max',inf), ...
        numel(power_init.tau_static_Nm));
    tau_hold_ratio = max(abs(double(power_init.tau_static_Nm(:).')) ./ ...
        max(abs(tau_lim_hold),eps));
    power_init.tau_limit_ratio = tau_hold_ratio;
    power_init.static_hold_feasible = logical( ...
        power_init.static_source_feasible && isfinite(tau_hold_ratio) && ...
        tau_hold_ratio <= 1 + 1e-9);
    if ~power_init.static_hold_feasible
        error('spinn3d_alpha_rollout_warmstart:InitialStaticHoldInfeasible', ...
            ['The q0 static hold is infeasible before Alpha rollout: ', ...
             'Pgrid=%.6g W, Pavg KPI target=%.6g W, tau ratio=%.6g, motor_ok=%d.'], ...
            power_init.Pgrid_static_W,power_init.Pavg_cap_W,tau_hold_ratio, ...
            double(power_init.motor_envelope_initial_ok));
    end
    st_bus0 = power_init.bus_state0;
    st_bus = st_bus0;

    plan_cfg = cfg.PLAN;
    Nknot = plan_cfg.NKNOT;
    warmstart_use_alpha_dot = require_binary_scalar_local( ...
        plan_cfg,'warmstart_rollout_use_alpha_dot', ...
        'PB.plan.warmstart_rollout_use_alpha_dot');
    use_bus_features = model_uses_bus_features(M);

    execSpec = getfield_def(cfg,'ALPHA_INFERENCE_EXECUTION', ...
               getfield_def(M.preproc,'inferenceExecutionEnvironment', ...
               getfield_def(M.preproc,'executionEnvironment','auto')));
    inferVerbose = logical(getfield_def(cfg,'GPU_VERBOSE',false));
    [useGPUInfer, inferExecEnv] = spinn3d_gpu_execution_env(execSpec, inferVerbose, 'alpha-warmstart');

    Nmax = max(4, ceil(traj.Tf / max(alpha_floor, 1e-6) / Ts) + 4);

    rho_hist    = zeros(Nmax,1);
    a_des_hist  = zeros(Nmax,1);
    a_nn_hist   = zeros(Nmax,1);
    a_post_hist = zeros(Nmax,1);
    a_star_hist = zeros(Nmax,1);
    a_cap_des_hist = zeros(Nmax,1);
    a_target_step_hist = zeros(Nmax,1);
    a_prev_hist = zeros(Nmax,1);
    policy_tick_hist = false(Nmax,1);
    a_use_hist  = zeros(Nmax,1);
    Pgrid_hist  = nan(Nmax,1);
    Pspill_hist = nan(Nmax,1);
    Ereuse_hist = nan(Nmax,1);
    Ebuf_hist   = nan(Nmax,1);
    Tbrk_hist   = nan(Nmax,1);

    caps_feas = limits;
    caps_feas.eta_share  = getfield_def(bus,'eta_share', getfield_def(limits,'eta_share',1.0));
    caps_feas.P_brk_peak = getfield_def(bus,'P_brk_peak', getfield_def(bus,'P_dump_max', getfield_def(limits,'P_brk_peak',inf)));
    caps_feas.bus = bus;
    caps_feas.bus_Ts = Ts;

    Pgrid_avg_lp = power_init.Pgrid_avg0_W;
    policy_dt = get_policy_dt_from_cfg(cfg, Ts);
    next_policy_t = 0.0;
    t = 0.0;
    a_nn_hold = alpha_floor;
    policy_tick = true;
    s      = 0.0;
    a_prev = alpha_floor;
    k      = 0;
    flow   = struct('Pgrid',NaN,'Pspill',NaN,'Preuse_total',NaN);

    while (s < traj.Tf - 1e-12) && (k < Nmax)
        [q_ref, dq_ref, ddq_ref] = traj.eval(s);
        q_ref  = row(q_ref);
        dq_ref = row(dq_ref);
        ddq_ref= row(ddq_ref);
        rho    = min(1.0, s / max(traj.Tf, 1e-12));

        tol_policy = max(1e-12, 1e-9 * max(1.0, abs(t)));
        policy_tick = (t + tol_policy >= next_policy_t);
        if policy_tick
            % Feature construction, trajectory evaluation and signed-shape

            % genuine dlnetwork forward/extract failure is classified as a
            % model inference failure by alpha_network_forward_local.
            a_nn_hold = predict_alpha_once(M, traj, s, q_ref, dq_ref, ...
                ddq_ref, robot, payload, Pbud, limits, a_prev, areg, ...
                Ts, policy_dt, bus, st_bus, Pgrid_avg_lp, use_bus_features, ...
                useGPUInfer, cfg.FRIC);

            a_nn_hold = clamp(double(a_nn_hold), alpha_floor, alpha_max);
            while next_policy_t <= t + tol_policy
                next_policy_t = next_policy_t + policy_dt;
            end
        end
        a_nn = a_nn_hold;

        caps_step = caps_feas;
        if use_bus_state
            caps_step.bus_state = st_bus;
        end
        caps_step.Pgrid_avg_prev = Pgrid_avg_lp;
        caps_step.alpha_prev = a_prev;
        % The network request is refreshed at policy ticks. Every realized
        % transition and its freshly evaluated physical cap use the plant Ts.
        caps_step.alpha_Ts = Ts;
        caps_step.use_alpha_dot = warmstart_use_alpha_dot;
        % The learned command already carries the soft-Pavg objective.  This
        % nominal-reference cap must not include Pavg as an implicit runtime
        % interlock.  It is a warm-start/planning aid; the measured-state
        % 1-ms actual guard remains the authoritative execution safety layer.
        caps_step.pavg_soft_target_select = false;
        % This nominal physical cap is refreshed every plant step. Holding
        % a one-Ts inverse-governor cap across policy_dt would accidentally
        % throttle startup by the policy/plant clock ratio. Only the NN
        % request a_nn_hold is ZOH; no plant or power timestep is enlarged.
        [a_star_step,feas_info] = alpha_feasible(q_ref,dq_ref,ddq_ref, ...
            1.0,robot,caps_step,Pbud,alpha_floor,itmax,cfg.FRIC);
        if ~isstruct(feas_info) || ~isscalar(feas_info) || ...
                ~valid_binary_field_local(feas_info,'any_feasible') || ...
                ~valid_binary_field_local(feas_info,'no_feasible')
            error('spinn3d:AlphaWarmstartInvalidFeasibilityInfo', ...
                ['alpha_feasible returned malformed feasibility ', ...
                 'metadata at t=%.9g s, path=%.9g/%.9g.'], ...
                t,s,traj.Tf);
        end
        if ~logical(feas_info.any_feasible) || ...
                logical(feas_info.no_feasible)
            error('spinn3d:AlphaWarmstartNoFeasibleCandidate', ...
                ['No feasible Alpha command exists at t=%.9g s, ', ...
                 'path=%.9g/%.9g.  The diagnostic alpha_min floor ', ...
                 'will not be executed as if it were feasible.'], ...
                t,s,traj.Tf);
        end
        if ~isnumeric(a_star_step) || ~isreal(a_star_step) || ...
                ~isscalar(a_star_step) || ~isfinite(double(a_star_step))
            error('spinn3d:AlphaWarmstartInvalidFeasibleValue', ...
                ['alpha_feasible returned an invalid command despite ', ...
                 'feasible metadata at t=%.9g s.'],t);
        end
        a_star_step = clamp(double(a_star_step), alpha_floor, alpha_max);
        % alpha_feasible returns the nominally admissible alpha AFTER
        % one Ts governor transition.  It is not a desired-alpha
        % command.  Invert the governor before combining it with the
        % network's desired-alpha output.  At the legal alpha_floor the
        % inverse may saturate, so this reference-side cap alone is not
        % a recursive/measured-state safety proof; the actual guard is.
        a_cap_des_step = alpha_des_teacher(a_star_step,a_prev, ...
            areg.BETA,areg.A_DOT_UP,areg.A_DOT_DN,Ts,alpha_max);
        a_cap_des_step = clamp(double(a_cap_des_step), ...
            alpha_floor,alpha_max);
        a_star = a_star_step;

        % NN inference remains a policy-rate request; governor/adot/BUS
        % advance at Ts.  Cap the target rather than hard-clipping the
        % post-governor command, which would bypass downward slew.
        a_target = min(a_nn_hold,a_cap_des_step);
        a_post = governor_step(a_target, a_prev, areg.BETA, areg.A_DOT_UP, areg.A_DOT_DN, Ts);
        a_post = clamp(double(a_post), alpha_floor, alpha_max);
        a_use  = a_post;
        % The exported plan is a desired-command plan: preserve the NN ZOH
        % request, not the changing plant-step cap. Replaying a cap sample
        % as a 10-ms desired command would reintroduce the startup defect.
        % a_target_step_seq records the nominal cap-adjusted request;
        % a_post_seq/a_use_seq record this rollout's realized transition.
        a_des = a_nn_hold;

        % Power and its causal EMA are dynamic inputs to the learned policy,
        % even when the stock actuator BUS has no explicit buffer/thermal
        % state.  Keep this step identical to the training-data rollout.
        adot    = (a_use - a_prev) / max(Ts, 1e-12);
        dq_start= a_prev * dq_ref;
        dq_cmd  = a_use * dq_ref;
        ddq_cmd = a_use*a_use * ddq_ref + adot * dq_ref;
        tau_cmd = row(inverseDynamics(robot, q_ref, dq_cmd, ddq_cmd));
        tau_cmd = tau_cmd + row(cfg.FRIC.B).*dq_cmd + ...
                  row(cfg.FRIC.Fc).*tanh(dq_cmd./max(cfg.FRIC.vel_eps,1e-9));
        dq_pwr=dq_start+0.5*Ts*ddq_cmd;
        dq_end=dq_start+Ts*ddq_cmd;
        bus_step=spinn3d_bus_with_state(bus,st_bus,Ts);
        if use_bus_state
            [flow,st_next]=spinn3d_bus_step(tau_cmd,dq_pwr,bus_step,st_bus,Ts);
        else
            bus_step.enable_state=false;
            [flow,st_next]=spinn3d_bus_step(tau_cmd,dq_pwr,bus_step,[],Ts);
        end
        if strcmp(char(bus.model_scope),'dynamic_capacitor_common_dc_v1')
            interval=spinn3d_motor_interval_audit(tau_cmd,dq_start,dq_end,bus_step);

        end
        st_bus=st_next;

        bus_avg = bus;
        bus_avg.Pgrid_avg_prev = Pgrid_avg_lp;
        [Pgrid_avg_lp, ~] = spinn3d_bus_avg_guard(flow.Pgrid, Pbud, bus_avg, Ts);

        k = k + 1;
        rho_hist(k)    = rho;
        a_des_hist(k)  = a_des;
        a_nn_hist(k)   = a_nn;
        a_post_hist(k) = a_post;
        a_star_hist(k) = a_star;
        a_cap_des_hist(k) = a_cap_des_step;
        a_target_step_hist(k) = a_target;
        a_prev_hist(k) = a_prev;
        policy_tick_hist(k) = policy_tick;
        a_use_hist(k)  = a_use;
        Pgrid_hist(k) = flow.Pgrid;
        Pspill_hist(k)= flow.Pspill;
        if use_bus_state
            Ereuse_hist(k)= st_bus.Ereuse;
            Ebuf_hist(k)  = st_bus.Ebuf;
            Tbrk_hist(k)  = st_bus.Tbrk;
        end

        a_prev = a_use;
        s      = min(traj.Tf, s + Ts * a_use);
        t      = t + Ts;

        if (Ts * a_use) <= 0
            break;
        end
    end

    % Real completed rollout intervals only. The rho-plan arrays below may
    % add endpoint samples for interpolation; those are not executed steps.
    rollout_telemetry = struct( ...
        'schema','nn_policy_zoh_physical_cap_plant_v1', ...
        't_start_s',(0:k-1)*Ts,'policy_tick',row(policy_tick_hist(1:k)), ...
        'nn_request',row(a_nn_hist(1:k)), ...
        'physical_cap_des',row(a_cap_des_hist(1:k)), ...
        'target_step',row(a_target_step_hist(1:k)), ...
        'alpha_prev',row(a_prev_hist(1:k)), ...
        'alpha_applied',row(a_use_hist(1:k)), ...
        'nominal_cap_dt_s',Ts,'nn_policy_dt_s',policy_dt, ...
        'nominal_cap_evaluations',k, ...
        'nn_forward_evaluations',sum(policy_tick_hist(1:k)));

    if k < 1
        rho_hist    = [0; 1];
        a_des_hist  = alpha_floor * [1; 1];
        a_nn_hist   = a_des_hist;
        a_post_hist = a_des_hist;
        a_star_hist = a_des_hist;
        a_cap_des_hist = a_des_hist;
        a_target_step_hist = a_des_hist;
        a_use_hist  = a_des_hist;
        Pgrid_hist  = nan(2,1);
        Pspill_hist = nan(2,1);
        Ereuse_hist = nan(2,1);
        Ebuf_hist   = nan(2,1);
        Tbrk_hist   = nan(2,1);
    else
        rho_hist    = rho_hist(1:k);
        a_des_hist  = a_des_hist(1:k);
        a_nn_hist   = a_nn_hist(1:k);
        a_post_hist = a_post_hist(1:k);
        a_star_hist = a_star_hist(1:k);
        a_cap_des_hist = a_cap_des_hist(1:k);
        a_target_step_hist = a_target_step_hist(1:k);
        a_use_hist  = a_use_hist(1:k);
        Pgrid_hist  = Pgrid_hist(1:k);
        Pspill_hist = Pspill_hist(1:k);
        Ereuse_hist = Ereuse_hist(1:k);
        Ebuf_hist   = Ebuf_hist(1:k);
        Tbrk_hist   = Tbrk_hist(1:k);

        if rho_hist(1) > 0
            rho_hist    = [0; rho_hist];
            a_des_hist  = [a_des_hist(1); a_des_hist];
            a_nn_hist   = [a_nn_hist(1); a_nn_hist];
            a_post_hist = [a_post_hist(1); a_post_hist];
            a_star_hist = [a_star_hist(1); a_star_hist];
            a_cap_des_hist = [a_cap_des_hist(1); a_cap_des_hist];
            a_target_step_hist = [a_target_step_hist(1); a_target_step_hist];
            a_use_hist  = [a_use_hist(1); a_use_hist];
            Pgrid_hist  = [Pgrid_hist(1); Pgrid_hist];
            Pspill_hist = [Pspill_hist(1); Pspill_hist];
            Ereuse_hist = [Ereuse_hist(1); Ereuse_hist];
            Ebuf_hist   = [Ebuf_hist(1); Ebuf_hist];
            Tbrk_hist   = [Tbrk_hist(1); Tbrk_hist];
        end
        if rho_hist(end) < 1
            rho_hist(end+1,1)    = 1;
            a_des_hist(end+1,1)  = a_des_hist(end);
            a_nn_hist(end+1,1)   = a_nn_hist(end);
            a_post_hist(end+1,1) = a_post_hist(end);
            a_star_hist(end+1,1) = a_star_hist(end);
            a_cap_des_hist(end+1,1) = a_cap_des_hist(end);
            a_target_step_hist(end+1,1) = a_target_step_hist(end);
            a_use_hist(end+1,1)  = a_use_hist(end);
            Pgrid_hist(end+1,1)  = Pgrid_hist(end);
            Pspill_hist(end+1,1) = Pspill_hist(end);
            Ereuse_hist(end+1,1) = Ereuse_hist(end);
            Ebuf_hist(end+1,1)   = Ebuf_hist(end);
            Tbrk_hist(end+1,1)   = Tbrk_hist(end);
        end
    end

    rho_knots = linspace(0,1,Nknot);
    a_knots   = interp1(rho_hist, a_des_hist, rho_knots, 'linear', 'extrap');
    a_knots   = clamp(a_knots, alpha_floor, alpha_max);

    plan = struct();
    plan.rho        = row(rho_hist);
    plan.a_des_seq  = row(a_des_hist);
    plan.a_nn_seq   = row(a_nn_hist);
    plan.a_post_seq = row(a_post_hist);
    plan.a_star_seq = row(a_star_hist);
    plan.a_cap_des_seq = row(a_cap_des_hist);
    plan.a_target_step_seq = row(a_target_step_hist);
    plan.rollout_telemetry = rollout_telemetry;
    plan.command_semantics = 'NN desired request; sampled/held at policy_dt';
    plan.cap_semantics = 'Nominal physical cap recomputed every Ts; not a held command';
    plan.a_use_seq  = row(a_use_hist);
    plan.rho_knots  = row(rho_knots);
    plan.a_knots    = row(a_knots);
    plan.Tf         = traj.Tf;
    plan.Ts         = Ts;
    plan.policy_dt  = policy_dt;
    plan.bus        = bus;
    plan.bus_state0 = st_bus0;
    plan.Pgrid_avg0 = power_init.Pgrid_avg0_W;
    plan.Pgrid_static_init_W = power_init.Pgrid_static_W;
    plan.power_init = power_init;
    plan.use_bus_state_features = use_bus_features;
    plan.bus_state_enabled = use_bus_state;
    plan.alpha_inference_execution = inferExecEnv;
    plan.alpha_inference_useGPU = logical(useGPUInfer);
    plan.model_budget_range_train_W = budget_range;
    plan.model_budget_metadata_present = logical(budget_meta_present);
    plan.model_budget_in_range = logical(budget_in_range);
    plan.Pgrid_seq       = row(Pgrid_hist);
    plan.Pspill_seq      = row(Pspill_hist);
    plan.Pgrid_avg_final = Pgrid_avg_lp;
    plan.flow_last       = flow;
    if use_bus_state
        plan.bus_state_final = st_bus;
        plan.Ereuse_seq = row(Ereuse_hist);
        plan.Ebuf_seq   = row(Ebuf_hist);
        plan.Tbrk_seq   = row(Tbrk_hist);
    end
    if isfield(cfg,'MODEL_PATH'), plan.model_path = cfg.MODEL_PATH; end
end

% =====================================================================
% helpers
% =====================================================================
function a0 = predict_alpha_once(M, traj, s, q_ref, dq_ref, ddq_ref, robot, payload, Pbud, caps, a_prev, areg, Ts, policy_dt, bus, st_bus, Pgrid_avg_lp, use_bus_features, useGPUInfer, fric)
    featFcn = @spinn3d_features_alpha_plus_payload_gov;
    Dtail = 6;
    % This optional warm-start is a nominal planning proposal. Its one
    % explicit nominal current observation is shared by all preview tokens.
    % Deployed controllers and teacher datasets use measured states instead.
    ctx=struct('q',q_ref,'dq',a_prev.*dq_ref,'fric',fric);
    [~,ctx]=spinn3d_alpha_observed_state_features(q_ref,dq_ref,a_prev,robot,ctx);

    Klook = double(M.preproc.K);

    if Klook > 1

        ds = double(M.preproc.ds);

        rho0 = min(1.0, s / max(traj.Tf, 1e-12));
        x1 = feature_local(featFcn, q_ref, dq_ref, ddq_ref, robot, payload, Pbud, caps, a_prev, areg, Ts, rho0, bus, st_bus, Pgrid_avg_lp, use_bus_features, ctx);
        x1 = row(double(x1));

        Dtot  = numel(x1);
        Dbase = Dtot - Dtail;
        kb    = get_keep_base_idx(M, Dbase);

        Xfull = zeros(Klook, Dtot);
        Xfull(1,:) = x1;
        for ti = 2:Klook
            s_i = min(traj.Tf, s + (ti-1)*ds);
            [qi, dqgi, ddqgi] = traj.eval(s_i);
            rho_i = min(1.0, s_i / max(traj.Tf, 1e-12));
            xi = feature_local(featFcn, row(qi), row(dqgi), row(ddqgi), robot, payload, Pbud, caps, a_prev, areg, Ts, rho_i, bus, st_bus, Pgrid_avg_lp, use_bus_features, ctx);
            Xfull(ti,:) = row(double(xi));
        end

        Xbase = Xfull(:,1:Dbase);
        Xtail = Xfull(:,Dbase+1:end);
        Xc    = [Xbase(:,kb), Xtail];
        mu    = row(double(M.preproc.mu));
        sig   = row(double(M.preproc.sig));

        Z = (Xc - mu) ./ max(sig, 1e-12);
        Xmini = single(reshape(Z.', [size(Z,2), 1, size(Z,1)]));
        if useGPUInfer
            Xmini = gpuArray(Xmini);
        end
        dlX = dlarray(Xmini, 'CBT');
        a0 = alpha_network_forward_local(M.net,dlX);
    else
        rho0 = min(1.0, s / max(traj.Tf, 1e-12));
        x_full = feature_local(featFcn, q_ref, dq_ref, ddq_ref, robot, payload, Pbud, caps, a_prev, areg, Ts, rho0, bus, st_bus, Pgrid_avg_lp, use_bus_features, ctx);
        x_full = row(double(x_full));

        Dtot  = numel(x_full);
        Dbase = Dtot - Dtail;
        kb    = get_keep_base_idx(M, Dbase);

        x_base = x_full(1:Dbase);
        x_tail = x_full(Dbase+1:end);
        x      = [x_base(kb), x_tail];

        mu  = row(double(M.preproc.mu));
        sig = row(double(M.preproc.sig));

        z = (x - mu) ./ max(sig, 1e-12);
        Xmini = single(z');
        if useGPUInfer
            Xmini = gpuArray(Xmini);
        end
        dlX = dlarray(Xmini, 'CB');
        a0 = alpha_network_forward_local(M.net,dlX);
    end
end

function a0 = alpha_network_forward_local(net,dlX)
% This is the only scope eligible for the centrally configured model-

% their original identifiers and cannot be mistaken for a bad NN forward.
try
    raw = gather(extractdata(forward(net,dlX)));
catch ME
    wrapped = MException('spinn3d:AlphaModelInferenceFailed', ...
        'Alpha network forward inference failed: %s',ME.message);
    wrapped = addCause(wrapped,ME);
    throw(wrapped);
end
if ~isnumeric(raw) || ~isreal(raw) || ~isscalar(raw) || ...
        ~isfinite(double(raw))
    error('spinn3d:AlphaModelInferenceFailed', ...
        ['Alpha network forward inference returned a non-finite, non-real ', ...
         'or non-scalar command.']);
end
a0 = double(raw);
end

function x = feature_local(featFcn, q, dq, ddq, robot, payload, Pbud, caps, a_prev, areg, Ts, s_norm, bus, st_bus, Pgrid_avg_lp, use_bus_features, ctx)
    if use_bus_features
        x = featFcn(q, dq, ddq, robot, payload, Pbud, caps, a_prev, areg.BETA, areg.A_DOT_UP, areg.A_DOT_DN, Ts, s_norm, bus, st_bus, Pgrid_avg_lp, ctx);
    else
        error('spinn3d:AlphaObservedBusMissing','No-brake observed-state v7 models require BUS features.');
    end
end

function tf = model_uses_bus_features(M)
    tf = false;
    if ~isstruct(M) || ~isfield(M,'preproc') || ~isstruct(M.preproc)
        return;
    end
    if isfield(M.preproc,'use_bus_state_features') && ~isempty(M.preproc.use_bus_state_features)
        tf = logical(M.preproc.use_bus_state_features);
        return;
    end
    if isfield(M.preproc,'bus_feature_dim') && ~isempty(M.preproc.bus_feature_dim)
        tf = double(M.preproc.bus_feature_dim) > 0;
    end
end

function kb = get_keep_base_idx(M, Dbase)
    if isfield(M.preproc,'keepBaseIdx') && ~isempty(M.preproc.keepBaseIdx)
        kb = logical(M.preproc.keepBaseIdx(:).');

    else
        kb = true(1, Dbase);
    end
end

function payload = infer_payload_from_cfg(cfg)
    payload = struct('mass',0,'com',[0 0 0],'inertia',zeros(1,6));
    if isfield(cfg,'PAYLOAD') && isstruct(cfg.PAYLOAD) && isfield(cfg.PAYLOAD,'DEMO') && isstruct(cfg.PAYLOAD.DEMO)
        P = cfg.PAYLOAD.DEMO;
        payload.mass    = getfield_def(P,'mass',0);
        payload.com     = row(getfield_def(P,'com',[0 0 0]));
        payload.inertia = row(getfield_def(P,'inertia',zeros(1,6)));
    end
end

function policy_dt = get_policy_dt_from_cfg(cfg, Ts)

    policy_dt = double(cfg.POLICY.dt);
    ratio = policy_dt/double(Ts);

end

function v = expand_limit_local(v,n)
v = double(v(:).');
if isempty(v), v = inf; end
if isscalar(v)
    v = repmat(v,1,n);
elseif numel(v)<n
    v = [v,repmat(v(end),1,n-numel(v))];
else
    v = v(1:n);
end
end

function [inside,r,present] = model_budget_coverage_local(M,pbud)
r = [NaN NaN];
range_present = isstruct(M) && isfield(M,'preproc') && isstruct(M.preproc) && ...
                isfield(M.preproc,'pbud_range_train') && ...
                isnumeric(M.preproc.pbud_range_train) && ...
                numel(M.preproc.pbud_range_train)==2 && ...
                all(isfinite(double(M.preproc.pbud_range_train(:))));
if range_present
    rr = double(M.preproc.pbud_range_train(:));
    r = [min(rr) max(rr)];
end
anchor_present = isstruct(M) && isfield(M,'preproc') && isstruct(M.preproc) && ...
                 isfield(M.preproc,'deployment_budget_anchor_W') && ...
                 isnumeric(M.preproc.deployment_budget_anchor_W) && ...
                 isscalar(M.preproc.deployment_budget_anchor_W) && ...
                 isfinite(double(M.preproc.deployment_budget_anchor_W)) && ...
                 isfield(M.preproc,'deployment_budget_exact_traj_count') && ...
                 isnumeric(M.preproc.deployment_budget_exact_traj_count) && ...
                 isscalar(M.preproc.deployment_budget_exact_traj_count) && ...
                 isfinite(double(M.preproc.deployment_budget_exact_traj_count));
% The schema token is retained for file-format stability.  Semantically it
% means "fixed within the current experiment trial"; PB.power.P_TOTAL_MAX is
% intentionally allowed to change between independently retrained trials.
current_trial_schema = isstruct(M) && isfield(M,'preproc') && isstruct(M.preproc) && ...
    isfield(M.preproc,'pbud_sampling_schema') && ...
    strcmp(char(M.preproc.pbud_sampling_schema),'fixed_deployment_budget_v2');
declared_exact = isstruct(M) && isfield(M,'preproc') && ...
    exact_range_local(getfield_def(M.preproc,'pbud_range_declared_W',[]),pbud);
observed_exact = isstruct(M) && isfield(M,'preproc') && ...
    exact_range_local(getfield_def(M.preproc,'pbud_range_observed_W',[]),pbud);
train_exact = range_present && exact_range_local(r,pbud);
accepted_count = getfield_def(M.preproc,'accepted_trajectory_count',NaN);
expected_count = NaN;
present_cfg = false;
if isfield(M.preproc,'deployment_budget_exact_traj_count')
    expected_count = double(M.preproc.deployment_budget_exact_traj_count);
end
count_ok = isnumeric(accepted_count) && isscalar(accepted_count) && ...
    isfinite(double(accepted_count)) && double(accepted_count)>0 && ...
    isfinite(expected_count) && expected_count>=double(accepted_count);
present = range_present && anchor_present && current_trial_schema && declared_exact && ...
    observed_exact && train_exact && count_ok;
inside = present && isfinite(pbud) && ...
         abs(pbud-double(M.preproc.deployment_budget_anchor_W)) <= 1e-9 && ...
         double(M.preproc.deployment_budget_exact_traj_count) > 0;
end

function tf = exact_range_local(r,pbud)
tf = isnumeric(r) && numel(r)==2 && all(isfinite(double(r(:)))) && ...
    all(abs(double(r(:))-double(pbud)) <= 1e-9);
end

function value = require_binary_scalar_local(S,field,label)

value = logical(S.(field));
end

function x = getfield_def(S,fn,def)
    if isstruct(S) && isfield(S,fn) && ~isempty(S.(fn))
        x = S.(fn);
    else
        x = def;
    end
end

function v = row(v)
    v = v(:).';
end

function y = clamp(x, lo, hi)
    y = min(hi, max(lo, x));
end

function tf = valid_binary_field_local(S,field)
v = [];
if isstruct(S) && isfield(S,field)
    v = S.(field);
end
tf = (islogical(v) || isnumeric(v)) && isscalar(v) && ...
     isreal(v) && isfinite(double(v)) && any(double(v)==[0 1]);
end
