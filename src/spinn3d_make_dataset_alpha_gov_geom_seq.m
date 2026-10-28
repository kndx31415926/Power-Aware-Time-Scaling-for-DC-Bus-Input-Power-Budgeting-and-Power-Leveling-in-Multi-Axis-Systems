function DS = spinn3d_make_dataset_alpha_gov_geom_seq(robot, TRAJ, caps, opts)
% Generate time-windowed policy inputs and teacher rate commands.
% Each sample retains the associated robot, motor and DC-link state.

    required_opts = {'teacher_config','teacher_max_rollout_duration_s','teacher_heartbeat_walltime_s','stop','ee_name','dt','Ts','policy_dt','alpha_min','alpha_max','itmax','fric', ...
        'max_rollout_steps','teacher_use_alpha_dot', ...
        'teacher_pavg_soft_target_select','teacher_pavg_selection_schema', ...
        'teacher_policy_rollout_admission_enable', ...
        'teacher_policy_rollout_schema','alpha_dataset_label_schema', ...
        'teacher_policy_command_grid_points', ...
        'teacher_local_command_offsets','teacher_refine_command_offsets', ...
        'teacher_successor_command_offsets', ...
        'teacher_transition_refine_iterations', ...
        'teacher_recursive_viability_enable', ...
        'teacher_recursive_horizon_policy_ticks', ...
        'teacher_successor_global_grid_points', ...
        'teacher_tau_reserve_ratio','teacher_qdd_reserve_ratio', ...
        'teacher_internal_margin_W', ...
        'teacher_minimum_time_gain_s','teacher_speed_deadline_enable', ...
        'training_sample_stride_policy_ticks','max_samples_per_traj', ...
        'sample_selection_schema','BETA','A_DOT_UP','A_DOT_DN', ...
        'K','H','payload','bus','bus_state', ...
        'use_bus_state_rollout','use_bus_state_features'};

    

    C = opts.teacher_config;

    
    stop = opts.stop;

    ee_name = scalar_text_local(opts.ee_name,'opts.ee_name');
    heartbeat_s=double(opts.teacher_heartbeat_walltime_s);

    dt   = double(opts.dt);
    Ts   = double(opts.Ts);

    policy_dt = double(opts.policy_dt);
    policy_ratio = policy_dt/Ts;
    alpha_max = double(opts.alpha_max);

    teacher_internal_margin_W = double(opts.teacher_internal_margin_W);
    teacher_minimum_time_gain_s = double(opts.teacher_minimum_time_gain_s);
    teacher_speed_deadline_enable = logical(opts.teacher_speed_deadline_enable);

    

    training_stride = double(opts.training_sample_stride_policy_ticks);

    training_stride = round(training_stride);

    max_samples_per_traj = double(opts.max_samples_per_traj);

    if isfinite(max_samples_per_traj)
        max_samples_per_traj = round(max_samples_per_traj);
    end

    sample_selection_schema = opts.sample_selection_schema;
    if isstring(sample_selection_schema) && isscalar(sample_selection_schema)
        sample_selection_schema = char(sample_selection_schema);
    end

    maxSteps = double(opts.max_rollout_steps);

    maxSteps = round(maxSteps);
    duration_cap=double(opts.teacher_max_rollout_duration_s);

    maxSteps=min(maxSteps,max(1,ceil(duration_cap/Ts)));
    teacher_use_alpha_dot = opts.teacher_use_alpha_dot;

    teacher_use_alpha_dot = logical(teacher_use_alpha_dot);

    teacher_pavg_soft_target_select = ...
        opts.teacher_pavg_soft_target_select;

    teacher_pavg_soft_target_select = ...
        logical(teacher_pavg_soft_target_select);

    teacher_pavg_selection_schema = ...
        opts.teacher_pavg_selection_schema;
    if isstring(teacher_pavg_selection_schema) && ...
            isscalar(teacher_pavg_selection_schema)
        teacher_pavg_selection_schema = ...
            char(teacher_pavg_selection_schema);
    end

    policy_admission_enable = ...
        opts.teacher_policy_rollout_admission_enable;

    policy_admission_schema = scalar_text_local( ...
        opts.teacher_policy_rollout_schema, ...
        'opts.teacher_policy_rollout_schema');
    alpha_dataset_label_schema = scalar_text_local( ...
        opts.alpha_dataset_label_schema, ...
        'opts.alpha_dataset_label_schema');

    
    policy_command_grid_points = double( ...
        opts.teacher_policy_command_grid_points);

    teacher_local_command_offsets = validated_offsets_local( ...
        opts.teacher_local_command_offsets, ...
        'opts.teacher_local_command_offsets');
    teacher_refine_command_offsets = validated_offsets_local( ...
        opts.teacher_refine_command_offsets, ...
        'opts.teacher_refine_command_offsets');
    teacher_successor_command_offsets = validated_offsets_local( ...
        opts.teacher_successor_command_offsets, ...
        'opts.teacher_successor_command_offsets');
    teacher_transition_refine_iterations = double( ...
        opts.teacher_transition_refine_iterations);
    teacher_recursive_viability_enable = logical( ...
        opts.teacher_recursive_viability_enable);
    teacher_recursive_horizon_policy_ticks = double( ...
        opts.teacher_recursive_horizon_policy_ticks);
    teacher_successor_global_grid_points = double( ...
        opts.teacher_successor_global_grid_points);
    teacher_tau_reserve_ratio = double(opts.teacher_tau_reserve_ratio);
    teacher_qdd_reserve_ratio = double(opts.teacher_qdd_reserve_ratio);

    beta = double(opts.BETA);
    up   = double(opts.A_DOT_UP);
    dn   = double(opts.A_DOT_DN);

    fric = opts.fric;

    K  = double(opts.K);

    if K==1
        ds = 0;
    else
        H  = double(opts.H);

        ds = H/(K-1);

    end

    Pbud = double(caps.P_total_max);

    payload = normalize_payload_local(opts.payload);

    bus = opts.bus;

    bus = spinn3d_bus_defaults(bus);
    % Formal dynamic DC-link labels carry voltage, capacitor energy and
    % capacitor voltage at every plant step, alongside the causal power EMA.
    rollout_flag = opts.use_bus_state_rollout;
    feature_flag = opts.use_bus_state_features;

    use_bus_rollout  = logical(rollout_flag);
    use_bus_features = logical(feature_flag);

    if ~isfield(bus,'enable_state') || isempty(bus.enable_state)
        bus.enable_state = false;
    end
    use_explicit_bus_state = use_bus_rollout && logical(bus.enable_state);
    dynamic_bus=strcmp(char(bus.model_scope),'dynamic_capacitor_common_dc_v1');

    st_bus0 = sanitize_bus_state_local(opts.bus_state, bus);
    if use_explicit_bus_state && isempty(st_bus0)
        st_bus0 = spinn3d_bus_state_init(bus);
    end

    % The public injector works on a private rigidBodyTree copy.  Do not
    % mutate-and-restore the shared training robot: an exception anywhere in
    % the rollout would otherwise leak this payload into later trajectories.
    [robot_use, ~] = spinn3d_inject_payload_last_body(robot, payload);

    Tf = double(TRAJ.Tf);
    if ~isfinite(Tf) || Tf <= 0
        error('spinn3d:AlphaDatasetInvalidTrajectoryDuration', ...
            'Alpha dataset trajectory Tf must be finite and positive; got %.9g.', ...
            Tf);
    end

    Tcap_data = Tf / max(double(opts.alpha_min), 1e-6) + policy_dt;
    Npolicy_max = max(2, ceil(Tcap_data / max(policy_dt,1e-12)) + 2);
    % Feature construction is intentionally decimated only at the dataset
    % storage boundary.  Teacher decisions still occur at every policy
    % tick and plant/BUS propagation still occurs at every Ts step.
    Nmax = max(2, ceil(Npolicy_max / training_stride) + 2);

    [q0, dqg0, ddqg0] = TRAJ.eval(0);
    q_initial = row(double(getfield_def(opts,'initial_q',q0)));
    dq_initial = row(double(getfield_def(opts,'initial_dq',double(opts.alpha_min)*row(dqg0))));

    [qf,~,~]=TRAJ.eval(Tf);
    T_goal=getTransform(robot_use,row(qf),ee_name);
    goal_position=T_goal(1:3,4);
    bus_initial=bus;
    explicit_Pavg0=getfield_def(opts,'Pgrid_avg0',NaN);
    has_explicit_Pavg0=isnumeric(explicit_Pavg0) && isreal(explicit_Pavg0) && ...
        isscalar(explicit_Pavg0) && isfinite(explicit_Pavg0);
    if has_explicit_Pavg0

    end
    if dynamic_bus && ~isempty(opts.bus_state)
        % An explicit continuation preserves its voltage and actual EMA
        % history. The shared initializer rejects voltage-only states rather
        % than treating a one-step charging transient as prior filter history.
        initial_cfg=spinn3d_apply_bus_state_to_cfg(struct('BUS',bus),st_bus0);
        bus_initial=initial_cfg.BUS;
        if has_explicit_Pavg0
            bus_initial.initial_Pgrid_avg_W=double(explicit_Pavg0);
        end
    end
    power_init = spinn3d_static_hold_power_init( ...
        robot_use,q_initial,fric,bus_initial,Pbud,Ts);
    tau_lim_hold = double(getfield_def(caps,'tau_max',inf));
    tau_lim_hold = tau_lim_hold(:).';
    if isscalar(tau_lim_hold)
        tau_lim_hold = repmat(tau_lim_hold,1,numel(power_init.tau_static_Nm));
    end
    tau_lim_hold = tau_lim_hold(1:numel(power_init.tau_static_Nm));
    tau_hold_ratio = max(abs(double(power_init.tau_static_Nm(:).')) ./ ...
        max(abs(tau_lim_hold),eps));
    static_physical_source_feasible = ...
        logical(power_init.static_physical_feasible);
    if ~static_physical_source_feasible || ~isfinite(tau_hold_ratio) || ...
            tau_hold_ratio > 1 + 1e-9
        error('spinn3d:AlphaDatasetStaticHoldInfeasible', ...
            ['Reject q0 before label generation on a physical limit: ', ...
             'static Pgrid=%.6g W, Pavg target=%.6g W (KPI only), ', ...
             'peak cap=%.6g W, tau ratio=%.6g, motor_ok=%d, ', ...
             'spill_ok=%d, thermal_ok=%d, buffer_ok=%d.'], ...
            power_init.Pgrid_static_W,power_init.Pavg_cap_W, ...
            power_init.Pgrid_peak_cap_W,tau_hold_ratio, ...
            double(power_init.motor_envelope_initial_ok), ...
            double(power_init.spill_initial_ok), ...
            double(power_init.thermal_initial_ok), ...
            double(power_init.buffer_initial_ok));
    end
    if use_explicit_bus_state && isempty(opts.bus_state)
        st_bus0=power_init.bus_state0;
    end
    power_init.tau_limit_ratio = tau_hold_ratio;
    power_init.static_hold_feasible = true;
    power_init.static_physical_source_feasible = ...
        static_physical_source_feasible;
    power_init.Pavg_is_soft_kpi = true;
    Pgrid_avg_lp=double(power_init.Pgrid_avg0_W);
    if has_explicit_Pavg0
        Pgrid_avg_lp=double(explicit_Pavg0);
    end
    if isfield(opts,'training_initialization_schema')
        % Check the actual state, after any explicit filter override and before
        % the first teacher step. Generic measured continuation has no standard-
        % start flag and retains its separately supplied voltage/EMA history.

    end
    Pgrid_avg_initial = Pgrid_avg_lp;
    Pavg_target_W = double(power_init.Pavg_cap_W);
    Pgrid_avg_peak_W = Pgrid_avg_lp;
    Pavg_excess_peak_W = max(Pgrid_avg_lp-Pavg_target_W,0.0);
    Pavg_excess_integral_J = 0.0;
    Pavg_excess_sq_normalized_integral_s = 0.0;
    Pavg_exceed_duration_s = 0.0;
    Pavg_exceed_plant_step_count = 0;
    teacher_pavg_target_met_policy_tick_count = 0;
    teacher_pavg_soft_fallback_policy_tick_count = 0;
    teacher_selected_Pavg_next_peak_W = Pgrid_avg_lp;
    teacher_policy_seed_accept_count = 0;
    teacher_policy_fallback_select_count = 0;
    teacher_policy_preview_candidate_evaluations = 0;
    teacher_successor_candidate_evaluations = 0;
    teacher_recursive_viability_check_count = 0;
    teacher_recursive_viability_success_count = 0;
    teacher_adaptive_expansion_policy_tick_count = 0;
    teacher_transition_refinement_evaluations = 0;
    teacher_tau_reserve_met_policy_tick_count = 0;
    teacher_qdd_reserve_met_policy_tick_count = 0;
    teacher_policy_hard_admission_plant_step_count = 0;
    % First execute the complete teacher rollout using compact policy-tick
    % snapshots only.  Expensive K-token rigid-body features are materialised
    % after the entire trajectory has passed.  Rejected random candidates
    % therefore no longer pay for thousands of inverse-dynamics feature calls.
    s_record = zeros(Nmax,1);
    a_prev_record = zeros(Nmax,1);
    Pavg_record = zeros(Nmax,1);
    bus_state_record = cell(Nmax,1);
    q_record = zeros(Nmax,numel(q0));
    dq_record = zeros(Nmax,numel(q0));
    Y = zeros(Nmax,1,'single');

    caps_teacher = caps;
    caps_teacher.alpha_feasible.alpha_max = alpha_max;
    caps_teacher.eta_share  = getfield_def(bus,'eta_share', getfield_def(caps,'eta_share',1.0));
    if ~strcmp(char(bus.model_scope),'dynamic_capacitor_common_dc_v1')
        caps_teacher.P_brk_peak = getfield_def(bus,'P_brk_peak', getfield_def(bus,'P_dump_max', getfield_def(caps,'P_brk_peak',inf)));
    end
    if use_bus_rollout
        caps_teacher.bus = bus;
        caps_teacher.bus_Ts = Ts;
    end
    require_policy_admission_caps_local(caps_teacher,numel(q0));

    s       = 0.0;
    t       = 0.0;
    next_policy_t = 0.0;
    a_prev  = double(opts.alpha_min);
    a_des_hold = double(opts.alpha_min);
    st_bus  = st_bus0;
    idx     = 0;
    policy_tick_count = 0;

    stepCnt  = 0;
    S = struct('q',q_initial,'dq',dq_initial,'s',s,'a_prev',a_prev, ...
        'bus',st_bus,'Pavg',Pgrid_avg_lp);
    stable_duration=0; completed=false; endpoint_error=Inf;
    rollout_wall_clock=tic; heartbeat_clock=tic;

    while ~completed && (stepCnt < maxSteps)
        stepCnt = stepCnt + 1;

        tol_policy = max(1e-12, 1e-9 * max(1.0, abs(t)));
        policy_tick = (t + tol_policy >= next_policy_t);

        if policy_tick
            policy_tick_count = policy_tick_count + 1;
            % S contains only observations available at this policy tick.
            % The teacher propagates CTC and plant state; it never resets q/dq
            % to the reference and receives no elapsed or deadline time.
            [a_des_hold,policy_preview] = ...
                spinn3d_alpha_teacher_state('select',robot_use,TRAJ,S,C);
            if ~logical(policy_preview.physical_ok) || ~logical(policy_preview.recursive_viable)
                error('spinn3d:AlphaTeacherNoFeasibleCandidate', ...
                    ['No admissible state-CTC continuation found within the search budget at policy tick %d ', ...
                     '(path=%.9g/%.9g). The candidate is rejected without a fallback label.'], ...
                    policy_tick_count,s,Tf);
            end

            if toc(heartbeat_clock)>=heartbeat_s
                fprintf(['[alpha-gov-data] state teacher | policy tick=%d | ', ...
                    'path=%.4g/%.4g | simulation=%.3f s | wall=%.1f s | ', ...
                    'Pavg=%.3f W | desired/applied Alpha=%.3f/%.3f\n'], ...
                    policy_tick_count,s,Tf,t,toc(rollout_wall_clock), ...
                    Pgrid_avg_lp,a_des_hold,a_prev);
                heartbeat_clock=tic;
            end
            teacher_policy_preview_candidate_evaluations = ...
                teacher_policy_preview_candidate_evaluations + ...
                double(policy_preview.candidate_evaluations);
            teacher_successor_candidate_evaluations = ...
                teacher_successor_candidate_evaluations + ...
                double(policy_preview.successor_candidate_evaluations);
            teacher_recursive_viability_check_count = ...
                teacher_recursive_viability_check_count + 1;
            if logical(policy_preview.recursive_viable)
                teacher_recursive_viability_success_count = ...
                    teacher_recursive_viability_success_count + 1;
            end
            if logical(policy_preview.adaptive_expansion_used)
                teacher_adaptive_expansion_policy_tick_count = ...
                    teacher_adaptive_expansion_policy_tick_count + 1;
            end
            teacher_transition_refinement_evaluations = ...
                teacher_transition_refinement_evaluations + ...
                double(policy_preview.transition_refinement_evaluations);
            if logical(policy_preview.tau_reserve_met)
                teacher_tau_reserve_met_policy_tick_count = ...
                    teacher_tau_reserve_met_policy_tick_count + 1;
            end
            if logical(policy_preview.qdd_reserve_met)
                teacher_qdd_reserve_met_policy_tick_count = ...
                    teacher_qdd_reserve_met_policy_tick_count + 1;
            end
            if logical(policy_preview.seed_selected)
                teacher_policy_seed_accept_count = ...
                    teacher_policy_seed_accept_count + 1;
            else
                teacher_policy_fallback_select_count = ...
                    teacher_policy_fallback_select_count + 1;
            end
            teacher_selected_Pavg_next_peak_W = max( ...
                teacher_selected_Pavg_next_peak_W, ...
                double(policy_preview.Pgrid_avg_peak_W));
            if double(policy_preview.Pavg_excess_peak_W) <= 1e-9
                teacher_pavg_target_met_policy_tick_count = ...
                    teacher_pavg_target_met_policy_tick_count + 1;
            else
                % The selected physical candidate trades progress against
                % soft excess. This does not claim every other candidate
                % exceeded the KPI: a small excess may buy useful progress.
                teacher_pavg_soft_fallback_policy_tick_count = ...
                    teacher_pavg_soft_fallback_policy_tick_count + 1;
            end

            record_policy_tick = ...
                (mod(policy_tick_count - 1, training_stride) == 0);
            if record_policy_tick
                if idx >= Nmax
                    % Grow only compact policy snapshots during the teacher
                    % pass.  K-token features are built after full acceptance.
                    Nmax_new = max(Nmax + 16, ceil(1.25 * Nmax));
                    s_record(Nmax_new,1) = 0;
                    a_prev_record(Nmax_new,1) = 0;
                    Pavg_record(Nmax_new,1) = 0;
                    bus_state_record{Nmax_new,1} = [];
                    Y(Nmax_new,1) = single(0);
                    q_record(Nmax_new,numel(q0))=0;
                    dq_record(Nmax_new,numel(q0))=0;
                    Nmax = Nmax_new;
                end
                idx = idx + 1;
                s_record(idx,1) = double(s);
                a_prev_record(idx,1) = double(a_prev);
                Pavg_record(idx,1) = double(Pgrid_avg_lp);
                bus_state_record{idx,1} = st_bus;
                Y(idx,1) = single(a_des_hold);
                q_record(idx,:)=row(S.q);
                dq_record(idx,:)=row(S.dq);
            end

            while next_policy_t <= t + tol_policy
                next_policy_t = next_policy_t + policy_dt;
            end
        end
        a_des = a_des_hold;
        [S_next,step_info] = spinn3d_alpha_teacher_state( ...
            'step',robot_use,TRAJ,S,a_des,C);
        teacher_policy_hard_admission_plant_step_count = ...
            teacher_policy_hard_admission_plant_step_count + 1;
        if ~logical(step_info.physical_ok)
            error('spinn3d:AlphaTeacherNoFeasibleCandidate', ...
                ['The state-CTC policy failed its 1-ms physical replay: ', ...
                 'tick=%d, step=%d, path=%.9g/%.9g, reason=%s. ', ...
                 'The complete candidate is rejected; no unsafe label is saved.'], ...
                policy_tick_count,stepCnt,s,Tf,char(step_info.failure_reason));
        end
        S=S_next;
        a_post = step_info.a_post;
        st_bus = step_info.st_next;
        Pgrid_avg_lp = step_info.Pgrid_avg_next_W;
        Pavg_excess_step = step_info.Pavg_excess_W;
        Pgrid_avg_peak_W = max(Pgrid_avg_peak_W,Pgrid_avg_lp);
        Pavg_excess_peak_W = max( ...
            Pavg_excess_peak_W,Pavg_excess_step);
        Pavg_excess_integral_J = Pavg_excess_integral_J + ...
            Pavg_excess_step*Ts;
        Pavg_excess_sq_normalized_integral_s = ...
            Pavg_excess_sq_normalized_integral_s + Ts * ...
            (Pavg_excess_step/max(Pavg_target_W,eps))^2;
        if Pavg_excess_step > 1e-9
            Pavg_exceed_duration_s = Pavg_exceed_duration_s + Ts;
            Pavg_exceed_plant_step_count = ...
                Pavg_exceed_plant_step_count + 1;
        end

        a_prev = a_post;
        s      = double(S.s);
        t      = t + Ts;
        T_actual=getTransform(robot_use,row(S.q),ee_name);
        endpoint_error=norm(T_actual(1:3,4)-goal_position);
        terminal_now=s>=Tf-max(1e-12,1e-10*max(1,Tf)) && ...
            endpoint_error<=double(stop.ee_tol) && ...
            norm(row(S.dq))<=double(stop.dq_tol_rad);
        if terminal_now
            stable_duration=stable_duration+Ts;
        else
            stable_duration=0;
        end
        completed=terminal_now && stable_duration+1e-12>=double(stop.stable_time);
    end

    if ~completed
        error('spinn3d:AlphaRolloutTruncatedHard', ...
            ['Alpha rollout did not reach a stable physical endpoint: ', ...
             's=%.9g/Tf=%.9g, steps=%d/%d, EE=%.6g m, max|dq|=%.6g rad/s. ', ...
             'No partial or merely reference-complete trajectory was saved.'], ...
            s,Tf,stepCnt,maxSteps,endpoint_error,max(abs(row(S.dq))));
    end

    if idx < 1
        error('spinn3d:AlphaDatasetEmptyRollout', ...
            'The candidate trajectory produced no alpha training samples.');
    end
    s_record = s_record(1:idx,:);
    a_prev_record = a_prev_record(1:idx,:);
    Pavg_record = Pavg_record(1:idx,:);
    bus_state_record = bus_state_record(1:idx,:);
    Y = Y(1:idx,:);
    q_record=q_record(1:idx,:);
    dq_record=dq_record(1:idx,:);
    if teacher_policy_hard_admission_plant_step_count ~= stepCnt
        error('spinn3d:AlphaDatasetPlantAdmissionCountMismatch', ...
            ['Every actual 1-ms dataset rollout step must pass exactly ', ...
             'one signed actual-state CTC admission check.']);
    end
    if teacher_recursive_viability_check_count ~= policy_tick_count || ...
            teacher_recursive_viability_success_count ~= policy_tick_count
        error('spinn3d:AlphaDatasetRecursiveAuditCountMismatch', ...
            ['Every accepted policy label must pass exactly one signed ', ...
             'successor-viability check.']);
    end

    % Determine the exact feature contract only after teacher acceptance.
    % No inverse-dynamics feature call is paid by a rejected random trajectory.
    s_norm0 = 0;
    observed0=struct('q',q_initial,'dq',dq_initial,'fric',fric);
    x0 = spinn3d_features_alpha_plus_payload_gov(row(q0),row(dqg0), ...
        row(ddqg0),robot_use,payload,Pbud,caps,double(opts.alpha_min), ...
        beta,up,dn,Ts,s_norm0,bus,st_bus0,Pgrid_avg_initial,observed0);
    base0 = spinn3d_features_alpha_plus_payload(row(q0),row(dqg0), ...
        row(ddqg0),robot_use,payload,Pbud,caps);
    Dtot = numel(x0);
    bus_feature_dim = 10;
    state_feature_dim=numel(q0)^2+3*numel(q0);

    % Materialise the expensive sequence features only after the complete
    % teacher rollout has reached a stable actual endpoint.  This is scientifically equivalent to
    % the prior online feature construction because every snapshot preserves
    % the exact pre-command q/dq/s/Alpha/BUS/EMA state used at that policy tick.
    fprintf(['[alpha-gov-data] teacher rollout reached stable actual endpoint | policy rows=%d | ', ...
        'materializing K=%d feature tokens per row after acceptance\n'],idx,K);
    Xseq = zeros(idx,K,Dtot,'single');
    materialization_heartbeat_rows = max(1,ceil(idx/10));
    for ri = 1:idx
        if ri==1 || ri==idx || mod(ri,materialization_heartbeat_rows)==0
            fprintf('[alpha-gov-data] feature materialization %d/%d rows\n',ri,idx);
        end
        s_base = double(s_record(ri));
        a_feature = double(a_prev_record(ri));
        Pavg_feature = double(Pavg_record(ri));
        st_feature = bus_state_record{ri};
        observed=struct('q',q_record(ri,:),'dq',dq_record(ri,:),'fric',fric);
        [q_now,dqg_now,~]=TRAJ.eval(s_base);
        [~,observed]=spinn3d_alpha_observed_state_features( ...
            row(q_now),row(dqg_now),a_feature,robot_use,observed);
        for ti = 1:K
            s_i = min(Tf,s_base+(ti-1)*ds);
            [qi,dqgi,ddqgi] = TRAJ.eval(s_i);
            qi=row(qi); dqgi=row(dqgi); ddqgi=row(ddqgi);
            s_norm_i = min(1.0,s_i/max(Tf,1e-9));
            if use_bus_features
                xi = spinn3d_features_alpha_plus_payload_gov( ...
                    qi,dqgi,ddqgi,robot_use,payload,Pbud,caps, ...
                    a_feature,beta,up,dn,Ts,s_norm_i,bus, ...
                    st_feature,Pavg_feature,observed);
            else
                xb = spinn3d_features_alpha_plus_payload( ...
                    qi,dqgi,ddqgi,robot_use,payload,Pbud,caps);
                xi = double([xb,a_feature,beta,up,dn,Ts,s_norm_i]);
            end
            Xseq(ri,ti,:) = single(xi);
        end
    end
    fprintf('[alpha-gov-data] feature materialization complete | rows=%d | K=%d\n',idx,K);

    raw_recorded_sample_count = idx;
    if ~all_finite_sequence_blocks_local(Xseq,256) || ...
            ~all(isfinite(Y(:)))
        error('spinn3d:AlphaDatasetNonFiniteSample', ...
            'Rejecting candidate because its features or labels contain NaN/Inf.');
    end

    % A finite trajectory-local cap can limit exceptionally slow rollouts.
    % The central +Inf setting deliberately bypasses this block and retains
    % every recorded 10-ms policy tick.
    if isfinite(max_samples_per_traj) && idx > max_samples_per_traj
        keep_idx = round(linspace(1, idx, max_samples_per_traj));
        keep_idx(1) = 1;
        keep_idx(end) = idx;
        keep_idx = unique(keep_idx, 'stable');
        Xseq = Xseq(keep_idx,:,:);
        Y = Y(keep_idx,:);
        s_record=s_record(keep_idx,:); a_prev_record=a_prev_record(keep_idx,:);
        Pavg_record=Pavg_record(keep_idx,:); bus_state_record=bus_state_record(keep_idx,:);
        q_record=q_record(keep_idx,:); dq_record=dq_record(keep_idx,:);
        idx = numel(keep_idx);
    end
    if training_stride == 1 && isinf(max_samples_per_traj) && ...
            raw_recorded_sample_count ~= policy_tick_count
        error('spinn3d:AlphaDatasetPolicyTickLoss', ...
            ['The uncapped stride-1 contract requires one supervised row ', ...
             'for every 10-ms policy tick.']);
    end

    Y = min(single(alpha_max), max(single(opts.alpha_min), Y));

    DS = struct();
    DS.Xseq = Xseq;
    DS.feature_schema='alpha_plus_payload_gov_ctc_nobrake_v7';
    DS.expected_feature_dim=Dtot;
    DS.y    = Y;
    DS.alpha_min = double(opts.alpha_min);
    DS.alpha_max = alpha_max;
    DS.K    = K;
    DS.ds   = ds;
    DS.dt   = dt;
    DS.Ts   = Ts;
    DS.policy_dt = policy_dt;
    DS.teacher_desired_command_dt_s = policy_dt;
    DS.teacher_applied_governor_dt_s = Ts;
    DS.teacher_cadence_schema = ...
        'desired_policy_dt_applied_governor_plant_ts_v1';
    DS.use_bus_state_features = use_bus_features;
    DS.use_bus_state_rollout  = use_bus_rollout;
    DS.use_explicit_bus_state = use_explicit_bus_state;
    DS.bus_feature_dim = bus_feature_dim;
    DS.state_feature_dim = state_feature_dim;
    DS.state_feature_schema='observed_ctc_eq_edq_mass_bias_v1';
    DS.teacher_state_schema='actual_q_dq_ctc_no_deadline_v1';
    DS.state_augmentation_schema=char(getfield_def(opts,'state_augmentation_schema','nominal_initial_state_v1'));
    DS.initial_q=q_initial; DS.initial_dq=dq_initial;
    DS.initial_q_error=q_initial-row(q0);
    DS.initial_dq_error=dq_initial-double(opts.alpha_min)*row(dqg0);
    DS.terminal_reached=completed;
    DS.terminal_ee_error_m=endpoint_error;
    DS.terminal_dq_max_rad_s=max(abs(row(S.dq)));
    DS.terminal_dq_norm_rad_s=norm(row(S.dq));
    DS.terminal_stable_duration_s=stable_duration;
    DS.rollout_elapsed_s=t;
    DS.state_final=S;
    DS.teacher_config=C;

    DS.policy_state=struct('q',q_record,'dq',dq_record,'s',s_record, ...
        'a_prev',a_prev_record,'Pavg',Pavg_record,'bus',{bus_state_record});
    DS.bus = bus;
    DS.bus_state0 = st_bus0;
    DS.power_init = power_init;
    DS.Pgrid_avg_initial = Pgrid_avg_initial;
    DS.Pgrid_avg_final = Pgrid_avg_lp;
    DS.traj_id = ones(size(Y));
    DS.training_sample_stride_policy_ticks = training_stride;
    DS.max_samples_per_traj = max_samples_per_traj;
    DS.sample_selection_schema = sample_selection_schema;
    DS.max_rollout_steps = maxSteps;
    DS.teacher_max_rollout_duration_s=duration_cap;
    DS.teacher_use_alpha_dot = teacher_use_alpha_dot;
    DS.teacher_pavg_soft_target_select = ...
        teacher_pavg_soft_target_select;
    DS.teacher_pavg_selection_schema = ...
        teacher_pavg_selection_schema;
    DS.teacher_policy_rollout_admission_enable = ...
        logical(policy_admission_enable);
    DS.teacher_policy_rollout_schema = policy_admission_schema;
    DS.alpha_dataset_label_schema = alpha_dataset_label_schema;
    DS.teacher_policy_command_grid_points = ...
        policy_command_grid_points;
    DS.teacher_local_command_offsets = teacher_local_command_offsets;
    DS.teacher_refine_command_offsets = teacher_refine_command_offsets;
    DS.teacher_successor_command_offsets = ...
        teacher_successor_command_offsets;
    DS.teacher_transition_refine_iterations = ...
        teacher_transition_refine_iterations;
    DS.teacher_recursive_viability_enable = ...
        teacher_recursive_viability_enable;
    DS.teacher_recursive_horizon_policy_ticks = ...
        teacher_recursive_horizon_policy_ticks;
    DS.teacher_successor_global_grid_points = ...
        teacher_successor_global_grid_points;
    DS.teacher_tau_reserve_ratio = teacher_tau_reserve_ratio;
    DS.teacher_qdd_reserve_ratio = teacher_qdd_reserve_ratio;
    DS.teacher_internal_margin_W = teacher_internal_margin_W;
    DS.teacher_minimum_time_gain_s = teacher_minimum_time_gain_s;
    DS.teacher_speed_deadline_enable = teacher_speed_deadline_enable;
    DS.teacher_policy_seed_accept_count = ...
        teacher_policy_seed_accept_count;
    DS.teacher_policy_fallback_select_count = ...
        teacher_policy_fallback_select_count;
    DS.teacher_policy_preview_candidate_evaluations = ...
        teacher_policy_preview_candidate_evaluations;
    DS.teacher_successor_candidate_evaluations = ...
        teacher_successor_candidate_evaluations;
    DS.teacher_recursive_viability_check_count = ...
        teacher_recursive_viability_check_count;
    DS.teacher_recursive_viability_success_count = ...
        teacher_recursive_viability_success_count;
    DS.teacher_adaptive_expansion_policy_tick_count = ...
        teacher_adaptive_expansion_policy_tick_count;
    DS.teacher_transition_refinement_evaluations = ...
        teacher_transition_refinement_evaluations;
    DS.teacher_tau_reserve_met_policy_tick_count = ...
        teacher_tau_reserve_met_policy_tick_count;
    DS.teacher_qdd_reserve_met_policy_tick_count = ...
        teacher_qdd_reserve_met_policy_tick_count;
    DS.teacher_feature_materialization_schema = ...
        'after_complete_teacher_rollout_accept_v1';
    DS.teacher_policy_hard_admission_plant_step_count = ...
        teacher_policy_hard_admission_plant_step_count;
    DS.raw_plant_step_count = stepCnt;
    DS.pavg_role = 'soft_teacher_target_and_reporting_kpi_v1';
    DS.pavg_hard_rejection_enable = false;
    DS.Pavg_target_W = Pavg_target_W;
    DS.Pgrid_avg_peak_W = Pgrid_avg_peak_W;
    DS.Pavg_excess_peak_W = Pavg_excess_peak_W;
    DS.Pavg_excess_peak_fraction = Pavg_excess_peak_W / ...
        max(Pavg_target_W,eps);
    DS.Pavg_excess_integral_J = Pavg_excess_integral_J;
    DS.Pavg_excess_sq_normalized_integral_s = ...
        Pavg_excess_sq_normalized_integral_s;
    DS.Pavg_excess_sq_normalized_time_mean = ...
        Pavg_excess_sq_normalized_integral_s/max(t,Ts);
    DS.Pavg_exceed_duration_s = Pavg_exceed_duration_s;
    DS.Pavg_exceed_plant_step_count = ...
        Pavg_exceed_plant_step_count;
    DS.teacher_pavg_target_met_policy_tick_count = ...
        teacher_pavg_target_met_policy_tick_count;
    DS.teacher_pavg_soft_fallback_policy_tick_count = ...
        teacher_pavg_soft_fallback_policy_tick_count;
    DS.teacher_selected_Pavg_next_peak_W = ...
        teacher_selected_Pavg_next_peak_W;
    DS.raw_policy_tick_count = policy_tick_count;
    DS.raw_recorded_sample_count = raw_recorded_sample_count;
    DS.kept_sample_count = idx;

end

% =====================================================================
% helper functions
% =====================================================================

function require_policy_admission_caps_local(caps,nJ)
required = {'tau_max','qd_max','qdd_cmd_max_actual','P_axis_max', ...
    'q_min','q_max','execution_audit'};

expand_limit_local(caps.tau_max,nJ,'tau_max');
expand_limit_local(caps.qd_max,nJ,'qd_max');
expand_limit_local(caps.qdd_cmd_max_actual,nJ,'qdd_cmd_max_actual');
expand_limit_local(caps.P_axis_max,nJ,'P_axis_max');
expand_signed_limit_local(caps.q_min,nJ,'q_min');
expand_signed_limit_local(caps.q_max,nJ,'q_max');
A = caps.execution_audit;
required_audit = {'ratio_tolerance','position_planning_tolerance_rad', ...
    'physical_power_abs_tolerance_W','physical_power_rel_tolerance', ...
    'spill_power_tolerance_W','joint_limit_viability'};

end

function v = expand_limit_local(v,nJ,name)

v = row(double(v));
if isscalar(v), v = repmat(v,1,nJ); end

end

function v = expand_signed_limit_local(v,nJ,name)

v = row(double(v));
if isscalar(v), v = repmat(v,1,nJ); end

end

function offsets = validated_offsets_local(value,name)

offsets=double(value(:).');

offsets=unique(offsets,'stable');
end

function out = scalar_text_local(v,name)
if isstring(v) && isscalar(v), v = char(v); end

out = v;
end

function v = row(v)
    v = v(:).';
end

function tf = all_finite_sequence_blocks_local(X,block_rows)
tf = true;
for i0 = 1:block_rows:size(X,1)
    ii = i0:min(i0+block_rows-1,size(X,1));
    xb = X(ii,:,:);
    if ~all(isfinite(xb(:)))
        tf = false;
        return;
    end
end
end

function x = getfield_def(S, fn, def)
    if isstruct(S) && isfield(S,fn) && ~isempty(S.(fn))
        x = S.(fn);
    else
        x = def;
    end
end

function tf = valid_binary_field_local(S,field)
v = [];
if isstruct(S) && isfield(S,field)
    v = S.(field);
end
tf = (islogical(v) || isnumeric(v)) && isscalar(v) && ...
     isreal(v) && isfinite(double(v)) && any(double(v)==[0 1]);
end

function payload = normalize_payload_local(payload_in)
    payload = struct('mass',0,'com',[0 0 0],'inertia',zeros(1,6), ...
                     'about','com','mode','replace');
    if isempty(payload_in)
        return;
    end
    if isstruct(payload_in)
        if isfield(payload_in,'mass') && ~isempty(payload_in.mass), payload.mass = double(payload_in.mass); end
        if isfield(payload_in,'com') && ~isempty(payload_in.com)
            payload.com = row(double(payload_in.com));
            if numel(payload.com) < 3, payload.com = [payload.com, zeros(1,3-numel(payload.com))]; end
            payload.com = payload.com(1:3);
        end
        if isfield(payload_in,'inertia') && ~isempty(payload_in.inertia)
            payload.inertia = row(double(payload_in.inertia));
            if numel(payload.inertia) < 6, payload.inertia = [payload.inertia, zeros(1,6-numel(payload.inertia))]; end
            payload.inertia = payload.inertia(1:6);
        end
        if isfield(payload_in,'about') && ~isempty(payload_in.about), payload.about = char(payload_in.about); end
        if isfield(payload_in,'mode') && ~isempty(payload_in.mode), payload.mode = char(payload_in.mode); end
    else
        v = double(payload_in(:)).';
        if numel(v) < 10, v = [v, zeros(1,10-numel(v))]; end
        payload.mass = v(1);
        payload.com  = v(2:4);
        payload.inertia = v(5:10);
    end
end

function st = sanitize_bus_state_local(st_in, bus)
    if isempty(st_in)
        st=[];
    else
        st=spinn3d_alpha_bus_state(bus,st_in);
    end
end
