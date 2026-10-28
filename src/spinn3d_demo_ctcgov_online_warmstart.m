function out = spinn3d_demo_ctcgov_online_warmstart(cfg)
%SPINN3D_DEMO_CTCGOV_ONLINE_WARMSTART
% Alpha-side warm-start demo:

% 

    if nargin < 1 || isempty(cfg)  
        blk = [];
        if exist('spinn3d_params_block','file')==2 
            blk = spinn3d_params_block();
        elseif evalin('base',"exist(''spinn3d_params_block'',''var'')==1") 
            blk = evalin('base','spinn3d_params_block');  
        end   

        cfg = spinn3d_params_to_cfg(blk, 'demo_alpha');
    end
    [cfg,~] = spinn3d_attach_paired_noguard_reference(cfg);

    req(cfg, {'TS','DT','VMAX','AMAX','Q0_DEG','QF_DEG', ... 
              'P_TOTAL_MAX','ALPHA_MIN','ALPHA_ITMAX','MODEL_PATH'});

    

    

     

    

    

    has_precomputed_solution = ...
        isfield(cfg,'WARMSTART_PRECOMPUTED_SOL') && ...
        ~isempty(cfg.WARMSTART_PRECOMPUTED_SOL);

    stop_stable_required=double(cfg.WARMSTART.stop_stable_time_s);

    allow_subfloor_recovery=logical(cfg.ACTUAL_GUARD.allow_subfloor_recovery);
    actual_recovery_floor=double(cfg.ACTUAL_GUARD.feasible_alpha_floor);

    limits = cfg.CAPS;
    gains  = cfg.CTRL;

    robot = cfg.ROBOT;
    try, robot.DataFormat='row'; end
    nJ = numel(homeConfiguration(robot));

    % Warm-start is a fixed-geometry alpha planner.  In the current demo the
    % BUS power was not the active constraint; the alpha was mostly clipped by
    % path/command acceleration caps before Pgrid_avg could approach the budget.
    % Relax only those kinematic command caps for warmstart, while keeping
    % torque, qd, raw grid peak, dump/spill, buffer and thermal guards active.
    % Filtered average power remains a logged/optimized soft KPI.
    [limits, cfg] = apply_warmstart_kin_relax_local(limits, cfg, nJ);
    policy_dt = get_policy_dt_local(cfg,cfg.TS);
    q0 = deg2rad_local(row(cfg.Q0_DEG));
    qf = deg2rad_local(row(cfg.QF_DEG));
    qf = unwrap_to_near(q0, qf);

    eeName=char(cfg.EE_NAME);

    pay_cfg=cfg.PAYLOAD.DEMO;
    if isstruct(pay_cfg)

    end
    [robot, pay_v10] = inject_payload_last_body(robot, pay_cfg);
    payload = v10_to_struct(pay_v10);

    traj = make_traj_cart_bezier_dls(robot, q0, qf, limits, cfg);
    if has_precomputed_solution

        % Do not load or execute the Alpha network before the branch, otherwise
        % a no-learning PTP baseline would still depend on the learned artifact.
        sol = cfg.WARMSTART_PRECOMPUTED_SOL;
        plan0 = sol.plan;
        fprintf('[warmstart-plan] using the precomputed plan.\n');
    else
        M = load_alpha_model(cfg.MODEL_PATH,'','final');
        plan0 = spinn3d_alpha_rollout_warmstart(robot, traj, cfg, payload, M);
        try
            refine_opts = cfg.PLAN;
            refine_opts.eval_dt = policy_dt;
            sol = spinn3d_alpha_plan_refine(robot, traj, cfg, plan0, refine_opts);
        catch ME
            if ~strcmp(ME.identifier,'spinn3d_alpha_plan_refine:NoNumericalPlan')
                rethrow(ME);
            end
            if ~logical(cfg.WARMSTART.allow_numerical_seed_fallback)
                rethrow(ME);
            end
            % Keep the learned same-method plan available for the authoritative
            % plant rollout if CEM encounters a purely numerical failure.  This

            fair_fallback = spinn3d_fair_time_contract(cfg,NaN,traj.Tf);
            warning('spinn3d:warmstartRefineFallback', ...
                ['Warm-start CEM produced no numerical plan (%s).  Executing its ', ...
                 'learned guarded seed.'],ME.message);
            sol = struct('plan',plan0,'plan0',plan0,'J0',NaN,'Jbest',NaN, ...
                'Tref',fair_fallback.t_reach_noguard_s, ...
                'TnoGuard',fair_fallback.t_reach_noguard_s, ...
                'time_deadline_s',fair_fallback.deadline_s, ...
                'max_delay_vs_noguard_s',fair_fallback.max_delay_vs_noguard_s, ...
                'seed_kind','warmstart-numerical-fallback', ...
                'numerical_feasible',false,'runnable',true, ...
                'proxy_physical_feasible',false,'proxy_time_contract_pass',false, ...
                'feasible',false,'planner_status','learned_guarded_seed_fallback', ...
                'refine_error_identifier',char(ME.identifier), ...
                'refine_error_message',char(ME.message));
        end
    end
    if isfield(sol,'proxy_time_contract_pass') && ~logical(sol.proxy_time_contract_pass)
        warning('spinn3d:warmstartProxyDeadlineMiss', ...
            ['Warm-start surrogate did not predict the configured NoGuard-relative pass.  The ', ...
             'best guarded plan will still receive the authoritative 1-ms plant check.']);
    end

    bus  = get_bus_from_cfg(cfg, limits.P_total_max);
    opts = struct();
    opts.method_id='warmstart';
    opts.executionAudit=cfg.EXECUTION_AUDIT;
    opts.Ts       = cfg.TS;

    
    method_cfg = cfg.METHODS.warmstart;
    opts.policy = cfg.POLICY;
    opts.fric     = cfg.FRIC;
    opts.bus      = bus;
    opts.alphaReg = cfg.ALPHA_REG;
    planned_use_bus_state=logical(cfg.WARMSTART.planned_use_bus_state);
    opts.nnAlpha  = struct( ...
        'enable',      logical(method_cfg.alpha_controller_enable), ...
        'mode',        char(method_cfg.alpha_controller_mode), ...
        'feasibility_use_alpha_dot', ...
                      method_cfg.feasibility_use_alpha_dot, ...
        'plan',        sol.plan, ...
        'alpha_floor', cfg.ALPHA_MIN, ...
        'alpha_max',   double(method_cfg.command_alpha_max), ...
        'planned_use_bus_state', planned_use_bus_state, ...
        'itmax',       cfg.ALPHA_ITMAX);
    opts.nnAlpha.gate=cfg.ALPHA_GATE;
    opts.actualGuard=cfg.METHOD_GUARDS.warmstart;

    plant=spinn3d_plant_controller_models(cfg,robot,bus,q0);

    opts.initial_bus_measurement=struct('bus_state',plant.bus_state, ...
    'Pgrid_static_W',plant.power_init.Pgrid_static_W, ...
    'Pgrid_avg_initial_W',plant.power_init.Pgrid_avg0_W);
ctl = spinn3d_controller_ctcgov(robot, traj, gains, limits, opts);
    power_init=plant.power_init;

    % below ALPHA_MIN.  The actual-step guard is intentionally allowed to do
    % so while recovering, therefore reserve explicit recovery and settling
    % time. Every value below is owned by PB.warmstart.
    Tnom = traj.Tf / max(cfg.ALPHA_MIN,1e-6);
    settle_allowance = cfg.WARMSTART.settle_allowance_s;
    recovery_allowance = max( ...
        double(cfg.WARMSTART.recovery_allowance_min_s), ...
        double(cfg.WARMSTART.recovery_allowance_factor)*Tnom);

    settle_allowance=double(settle_allowance);
    % The paper-wide fairness contract is an acceptance rule, not the

    % method retains a finite measured t_reach and is then marked rejected.
    fair_exec = spinn3d_fair_time_contract(cfg, NaN, traj.Tf);
    [Tcap, horizon_info] = spinn3d_fair_execution_horizon(cfg,traj.Tf,fair_exec.deadline_s);
    diagnostic_requested = Tnom + settle_allowance + recovery_allowance;
    max_time_user = cfg.WARMSTART.max_time_s;
    if isnumeric(max_time_user) && isscalar(max_time_user) && ...
            isfinite(double(max_time_user)) && double(max_time_user) > 0
        diagnostic_requested = min(diagnostic_requested,double(max_time_user));
    end
    diagnostic_requested = min(diagnostic_requested,horizon_info.absolute_cap_s);
    Tcap = max(fair_exec.deadline_s+cfg.TS,diagnostic_requested);
    Tcap = max(cfg.TS,ceil(Tcap/cfg.TS)*cfg.TS);
    horizon_info.Tcap_s = Tcap;
    horizon_info.warmstart_override_applied = isfinite_scalar_positive_local(max_time_user);

    stall_timeout=double(cfg.WARMSTART.stall_timeout_s);

    progress_eps=double(cfg.WARMSTART.progress_eps);

    Nt   = max(1, ceil(Tcap/cfg.TS));
    Tcap = Nt * cfg.TS; % exact simulated horizon; no extra integration step

    B  = row(cfg.FRIC.B);   Fc = row(cfg.FRIC.Fc);   ve = cfg.FRIC.vel_eps;
    noise_state = spinn3d_noise_init(cfg,nJ);

    tau_max = vec_lim(limits,'tau_max', nJ, inf);
    P_axismax = vec_lim(limits,'P_axis_max', nJ, inf);

    t=0; s=0;
    [q_ref0, ~, ~] = traj.eval(0);
    q = row(q_ref0); dq = zeros(1,nJ);

    log.t=zeros(Nt,1); log.s=zeros(Nt,1);
    log.controller_wall_s=nan(Nt,1); log.policy_tick=false(Nt,1);
    log.alpha_des=nan(Nt,1); log.alpha_governor=nan(Nt,1);
    log.controller_alpha_guard_applied=false(Nt,1); log.actual_guard_applied=false(Nt,1);
    log.plant_guard_wall_s=nan(Nt,1); log.step_wall_s=nan(Nt,1);
    log.network_forward_s=nan(Nt,1); log.actuator_saturation=false(Nt,1);
    log.motion_projection=false(Nt,1); log.actual_step_pass=false(Nt,1);
    log.q=zeros(Nt,nJ); log.dq=zeros(Nt,nJ); log.dq_pwr=zeros(Nt,nJ);
    % Replay inputs are sampled before the interval commit. dq_ref and
    % ddq_ref are trajectory derivatives before alpha/time scaling.
    log.q_ref=nan(Nt,nJ); log.dq_ref=nan(Nt,nJ); log.ddq_ref=nan(Nt,nJ);
    log.q_m=nan(Nt,nJ); log.dq_m=nan(Nt,nJ);
    log.q_pre=nan(Nt,nJ); log.dq_pre=nan(Nt,nJ);
    log.t_pre=nan(Nt,1); log.s_pre=nan(Nt,1); log.alpha_prev_step=nan(Nt,1);
    log.tau=zeros(Nt,nJ); log.ddq=zeros(Nt,nJ); log.alpha=zeros(Nt,1);
    log.alpha_cmd=zeros(Nt,1); log.execution_progress_ratio=ones(Nt,1);
    log.a0=zeros(Nt,1); log.afeas=nan(Nt,1); log.aplan=zeros(Nt,1);
    % standardized aliases for fair plotting vs joint demo
    log.a_plan=zeros(Nt,1); log.a_post=zeros(Nt,1); log.a_use=zeros(Nt,1); log.a_feasible=nan(Nt,1);
    log.afeas_valid=false(Nt,1); log.a_feasible_valid=false(Nt,1);
    log.Pi_raw=zeros(Nt,nJ); log.Pgrid=zeros(Nt,1); log.Pdump=zeros(Nt,1);
    log.Pshare=zeros(Nt,1); log.Pbuf_dis=zeros(Nt,1); log.Pbuf_chg=zeros(Nt,1); log.Pspill=zeros(Nt,1);
    log.Pshare_axis=zeros(Nt,1); log.Pshare_potential=zeros(Nt,1);
    log.Pregen_available=zeros(Nt,1); log.Pregen_curtailed=zeros(Nt,1);
    log.Pregen_curtailed_total=zeros(Nt,1); log.Pregen_cap_curtailed=zeros(Nt,1);
    log.Pload_balance_residual=zeros(Nt,1); log.Pregen_route_balance_residual=zeros(Nt,1);
    log.energy_balance_residual_W=zeros(Nt,1);
    log.Preuse_buf=zeros(Nt,1); log.Preuse_total=zeros(Nt,1);
    log.Pregen_bus=zeros(Nt,1); log.Preuse_task_buf=zeros(Nt,1); log.Ploss_total=zeros(Nt,1);
    log.Eregen_task=zeros(Nt,1);
    log.Ebuf=zeros(Nt,1); log.Ereuse=zeros(Nt,1); log.Tbrk=zeros(Nt,1);
    log.Pgrid_avg=zeros(Nt,1);
    log.Pgrid_avg_pre_t0_W=repmat(power_init.Pgrid_avg0_W,Nt,1);

    % whether the active limiter is qdd/tau/qd/BUS instead of filtered power.
    log.a_pre_actual=zeros(Nt,1); log.a_kin=zeros(Nt,1); log.actual_clip=zeros(Nt,1);
    log.actual_qdd_ratio=zeros(Nt,1); log.actual_tau_ratio=zeros(Nt,1); log.actual_qd_ratio=zeros(Nt,1);
    log.actual_pgrid_ratio=zeros(Nt,1); log.actual_pavg_ratio=zeros(Nt,1); log.actual_spill=zeros(Nt,1);
    log.actual_freeze=false(Nt,1); log.actual_floor_infeasible=false(Nt,1);
    log.actual_restart_bisect=false(Nt,1); log.actual_backtrack_iters=nan(Nt,1);
    log.actual_violation=nan(Nt,1); log.actual_subfloor=false(Nt,1);
    log.ee_error_m=nan(Nt,1); log.dq_norm=nan(Nt,1); log.rho=zeros(Nt,1);
    log.stall_elapsed_s=zeros(Nt,1); log.stop_stable_elapsed_s=zeros(Nt,1);
    log.ee=nan(Nt,3);

    ee_start = ee_pos(robot,q0,eeName);
    ee_goal  = ee_pos(robot,qf,eeName);
    t_reach  = NaN;
    termination_reason = 'time_cap_progress_incomplete';
    last_progress_s = 0.0;
    last_progress_t = 0.0;
    final_ee_err = norm(ee_start-ee_goal);
    final_dq_norm = norm(dq);
    stop_stable_elapsed = 0.0;
    terminal_guard = struct('available',false);

    for k=1:Nt
        step_clock=tic;
        [q_r, dq_r, ddq_r] = traj.eval(s);
        q_r  = row(q_r);
        dq_r = row(dq_r);
        ddq_r= row(ddq_r);

        [q_m,dq_m,tau_noise,noise_state] = ...
            spinn3d_noise_step(q,dq,cfg.TS,noise_state);

        controller_clock=tic;
        [tau_cmd, info] = ctl.step(t, q_m, dq_m, q_r, dq_r, ddq_r, robot, s);
        log.controller_wall_s(k)=getfield_def(info,'controller_wall_s',toc(controller_clock));
        log.policy_tick(k)=logical(getfield_def(info,'policy_tick',false));
        log.network_forward_s(k)=getfield_def(info,'network_forward_s',NaN);
        log.alpha_des(k)=info.a0; log.alpha_governor(k)=info.a_filt;
        log.controller_alpha_guard_applied(k)=abs(info.a_max-info.a_filt)>1e-10;
        tau_cmd = row(tau_cmd);

        rho = min(1.0, s / max(traj.Tf, 1e-12));
        a_plan_k = plan_lookup_local(sol.plan, rho, cfg.ALPHA_MIN, ...
            method_cfg.command_alpha_max);
        log.aplan(k)  = a_plan_k;
        log.a_plan(k) = a_plan_k;

        log.a0(k)     = info.a0;
        log.alpha_cmd(k) = info.a_max;
        log.a_post(k) = getfield_def(info,'a_filt', getfield_def(info,'a0', NaN));

        if ~isscalar(info.a_max) || ~isfinite(info.a_max) || any(~isfinite(tau_cmd))
            termination_reason = 'nonfinite_controller_output';
            log.t(k)=t; log.s(k)=s; log.q(k,:)=q; log.dq(k,:)=dq;
            log.tau(k,:)=tau_cmd;
            log.dq_pwr(k,:)=dq;
            log.rho(k)=min(1.0,s/max(traj.Tf,1e-12));
            log.ee(k,:)=tform2trvec_safe(getTransform(robot,q,eeName));
            final_ee_err=norm(log.ee(k,:)-ee_goal);
            final_dq_norm=norm(dq);
            log.ee_error_m(k)=final_ee_err;
            log.dq_norm(k)=final_dq_norm;
            log = truncate_log(log,k);
            break;
        end

        
        % elsewhere in the n-D motion box.  The final projector below is the
        % sole authority that may recover or reject the sample.
        actual_now = struct();
        if isfield(info,'guard') && isstruct(info.guard) && ...
                isfield(info.guard,'actual') && isstruct(info.guard.actual)
            actual_now = info.guard.actual;
        end

        a_feas_k = NaN;
        a_feas_valid_k = false;
        if isfield(info,'guard') && isstruct(info.guard) && isfield(info.guard,'a_star') && ~isempty(info.guard.a_star)
            a_feas_k = double(info.guard.a_star);
            a_feas_valid_k = isscalar(a_feas_k) && isfinite(a_feas_k);
        end
        if ~a_feas_valid_k
            caps_log = limits;
            caps_log.eta_share  = getfield_def(bus,'eta_share', getfield_def(limits,'eta_share',1.0));
            caps_log.P_brk_peak = getfield_def(bus,'P_brk_peak', getfield_def(bus,'P_dump_max', getfield_def(limits,'P_brk_peak',inf)));
            st_now = ctl.get_state();
            caps_log.bus = bus;
            caps_log.bus_Ts = cfg.TS;
            caps_log.Pgrid_avg_prev = getfield_def(st_now,'Pgrid_avg_lp',0.0);
            caps_log.alpha_prev = getfield_def(st_now,'a_prev',cfg.ALPHA_MIN);

            % a no-island result is recorded as NaN/invalid, never alpha_min.
            % BUS observation and the authoritative plant still use cfg.TS.
            caps_log.alpha_Ts = policy_dt;
            caps_log.use_alpha_dot = ...
                method_cfg.diagnostic_feasibility_use_alpha_dot;
            caps_log.alpha_feasible.alpha_max = ...
                double(method_cfg.command_alpha_max);
            if planned_use_bus_state && logical(getfield_def(bus,'enable_state',false))
                caps_log.bus_state = st_now.bus;
            end
            [a_feas_diag,a_feas_info] = alpha_feasible( ...
                q_r, dq_r, ddq_r, method_cfg.command_alpha_max, ...
                robot, caps_log, ...
                limits.P_total_max, cfg.ALPHA_MIN, cfg.ALPHA_ITMAX, cfg.FRIC);
            if ~isstruct(a_feas_info) || ~isscalar(a_feas_info) || ...
                    ~valid_binary_field_local(a_feas_info,'any_feasible') || ...
                    ~valid_binary_field_local(a_feas_info,'no_feasible')
                error('spinn3d_demo_ctcgov_online_warmstart:InvalidFeasibilityInfo', ...
                    'Diagnostic alpha_feasible returned malformed metadata.');
            end
            a_feas_valid_k = logical(a_feas_info.any_feasible) && ...
                ~logical(a_feas_info.no_feasible);
            if a_feas_valid_k
                if ~isnumeric(a_feas_diag) || ~isreal(a_feas_diag) || ...
                        ~isscalar(a_feas_diag) || ~isfinite(double(a_feas_diag))
                    error('spinn3d_demo_ctcgov_online_warmstart:InvalidFeasibleAlpha', ...
                        ['Diagnostic alpha_feasible returned an invalid value ', ...
                         'despite feasible metadata.']);
                end
                a_feas_k = double(a_feas_diag);
            else

                % exists.  NaN plus the validity flag prevents it from looking
                % like a feasible command in plots or saved logs.
                a_feas_k = NaN;
            end
        end
        log.afeas(k)      = a_feas_k;
        log.a_feasible(k) = a_feas_k;
        log.afeas_valid(k) = a_feas_valid_k;
        log.a_feasible_valid(k) = a_feas_valid_k;

        if logical(cfg.WARMSTART.log_guard_diagnostics)
            [log.a_pre_actual(k), log.a_kin(k), log.actual_clip(k), ...
             log.actual_qdd_ratio(k), log.actual_tau_ratio(k), log.actual_qd_ratio(k), ...
             log.actual_pgrid_ratio(k), log.actual_pavg_ratio(k), log.actual_spill(k), ...
             log.actual_freeze(k), log.actual_floor_infeasible(k), log.actual_restart_bisect(k), ...
             log.actual_backtrack_iters(k), log.actual_violation(k)] = ...
                warmstart_guard_diag_local(info, limits, nJ);
        else
            log.a_pre_actual(k)=NaN; log.a_kin(k)=NaN; log.actual_clip(k)=NaN;
            log.actual_qdd_ratio(k)=NaN; log.actual_tau_ratio(k)=NaN; log.actual_qd_ratio(k)=NaN;
            log.actual_pgrid_ratio(k)=NaN; log.actual_pavg_ratio(k)=NaN; log.actual_spill(k)=NaN;
            log.actual_freeze(k)=false; log.actual_floor_infeasible(k)=false;
            log.actual_restart_bisect(k)=false; log.actual_backtrack_iters(k)=NaN;
            log.actual_violation(k)=NaN;
        end
        dq_old = dq;
        plant_guard_clock=tic;
        [tau_m,ddq,dq_mid,step_audit] = spinn3d_plant_step( ...
            plant,q,dq,q_m,dq_m,tau_cmd+tau_noise,tau_max,P_axismax,cfg.TS, ...
            limits,cfg.WORKSPACE,cfg.EXECUTION_AUDIT);
        log.plant_guard_wall_s(k)=toc(plant_guard_clock);
        [~,log] = spinn3d_actual_guard_telemetry(info,log,k,step_audit);
        log.q_ref(k,:)=q_r; log.dq_ref(k,:)=dq_r; log.ddq_ref(k,:)=ddq_r;
        log.q_m(k,:)=q_m; log.dq_m(k,:)=dq_m;
        log.q_pre(k,:)=q; log.dq_pre(k,:)=dq;
        log.t_pre(k)=t; log.s_pre(k)=s;
        if isfield(info,'a_prev_step'), log.alpha_prev_step(k)=info.a_prev_step; end
        log.actual_step_pass(k)=logical(step_audit.final_motion.pass);
        if step_audit.final_motion.pass
            [a_exec,progress_ratio] = ...
                spinn3d_execution_progress_alpha(info.a_max,step_audit, ...
                    method_cfg.command_alpha_max);
        else
            a_exec=0; progress_ratio=0;
        end
        dq  = dq_old + cfg.TS*ddq;
        q   = q + cfg.TS*dq_mid;

        s = min(traj.Tf, s + cfg.TS*a_exec);
        log.alpha(k)=a_exec;
        log.a_use(k)=a_exec;
        log.execution_progress_ratio(k)=progress_ratio;
        log.actual_subfloor(k) = a_exec < cfg.ALPHA_MIN-1e-12;
        sample_end_t = t + cfg.TS;
        if s > last_progress_s + progress_eps
            last_progress_s = s;
            last_progress_t = sample_end_t;
        end

        dq_pwr = dq_mid;
        [measured,plant]=spinn3d_plant_observe(plant,tau_m,dq_pwr,cfg.TS);
        [obs, bus_state_now] = ctl.observe(tau_m,dq_pwr,a_exec,step_audit,measured);
        flow = obs.flow;

        log.t(k)=sample_end_t; log.s(k)=s; log.q(k,:)=q; log.dq(k,:)=dq;
        log.tau(k,:)=tau_m; log.ddq(k,:)=ddq; log.dq_pwr(k,:)=dq_pwr;
        log.Pi_raw(k,:)=tau_m.*dq_pwr; log.Pgrid(k)=flow.Pgrid; log.Pdump(k)=flow.Pdump;
        log.Pshare(k)=flow.Pshare; log.Pbuf_dis(k)=flow.Pbuf_dis; log.Pbuf_chg(k)=flow.Pbuf_chg;
        log.Pshare_axis(k)=flow.Pshare_axis; log.Pshare_potential(k)=flow.Pshare_potential;
        log.Pregen_available(k)=flow.Pregen_available; log.Pregen_curtailed(k)=flow.Pregen_curtailed;
        log.Pregen_curtailed_total(k)=flow.Pregen_curtailed_total; log.Pregen_cap_curtailed(k)=flow.Pregen_cap_curtailed;
        log.Pload_balance_residual(k)=flow.Pload_balance_residual;
        log.Pregen_route_balance_residual(k)=flow.Pregen_route_balance_residual;
        log.energy_balance_residual_W(k)=max(abs([flow.Pload_balance_residual flow.Pregen_route_balance_residual flow.Penergy_balance_residual]));
        log.Pspill(k)=flow.Pspill; log.Preuse_buf(k)=flow.Preuse_buf; log.Preuse_total(k)=flow.Preuse_total;
        log.Pregen_bus(k)=flow.Pregen_bus; log.Preuse_task_buf(k)=flow.Preuse_task_buf; log.Ploss_total(k)=flow.Ploss_total;
        log.Ebuf(k)=bus_state_now.Ebuf; log.Ereuse(k)=bus_state_now.Ereuse; log.Tbrk(k)=bus_state_now.Tbrk;
        log=spinn3d_dynamic_bus_log(log,k,flow,bus_state_now,power_init,bus);
        log.Eregen_task(k)=bus_state_now.Eregen_task;
        log.Pgrid_avg(k)=obs.Pgrid_avg_lp;
        Tee = getTransform(plant.robot, q, eeName); log.ee(k,:) = tform2trvec_safe(Tee);
        log.step_wall_s(k)=toc(step_clock);
        if ~step_audit.final_motion.pass
            termination_reason='execution_hard_constraint_violation';
            log=truncate_log(log,k);
            break;
        end

        ee_err = norm(log.ee(k,:) - ee_goal);
        dq_norm = norm(dq);
        final_ee_err = ee_err;
        final_dq_norm = dq_norm;
        log.ee_error_m(k) = ee_err;
        log.dq_norm(k) = dq_norm;
        log.rho(k) = min(1.0, s/max(traj.Tf,1e-12));
        log.stall_elapsed_s(k) = max(0.0, sample_end_t-last_progress_t);
        stop_inside = (s >= traj.Tf) && (ee_err <= cfg.STOP.NEAR_TOL) && ...
                      (dq_norm <= cfg.STOP.VEL_TOL);
        if stop_inside
            stop_stable_elapsed = stop_stable_elapsed + cfg.TS;
        else
            stop_stable_elapsed = 0.0;
        end
        log.stop_stable_elapsed_s(k) = stop_stable_elapsed;
        if stop_inside && (stop_stable_elapsed + 1e-12 >= stop_stable_required)
            t_reach = sample_end_t;
            termination_reason = 'reached';
            log = truncate_log(log, k);
            break;
        end

        if (s < traj.Tf-1e-12) && isfinite(stall_timeout) && ...
                (log.stall_elapsed_s(k) >= stall_timeout)
            termination_reason = 'stalled_no_progress';
            log = truncate_log(log, k);
            break;
        end

        t = t + cfg.TS;
    end

    if ~isfinite(t_reach) && strncmp(termination_reason,'time_cap',8)
        if s < traj.Tf-1e-12
            termination_reason = 'time_cap_progress_incomplete';
        elseif (final_ee_err > cfg.STOP.NEAR_TOL) && (final_dq_norm > cfg.STOP.VEL_TOL)
            termination_reason = 'time_cap_ee_and_velocity_not_settled';
        elseif final_ee_err > cfg.STOP.NEAR_TOL
            termination_reason = 'time_cap_ee_not_settled';
        elseif final_dq_norm > cfg.STOP.VEL_TOL
            termination_reason = 'time_cap_velocity_not_settled';
        elseif stop_stable_elapsed + 1e-12 < stop_stable_required
            termination_reason = 'time_cap_stability_window_not_met';
        else
            termination_reason = 'time_cap_stop_condition_not_met';
        end
    end

    freeze_samples = sum(log.actual_freeze(:));
    freeze_fraction = freeze_samples / max(numel(log.actual_freeze),1);
    max_freeze_streak_s = cfg.TS * max_true_run_local(log.actual_freeze);
    subfloor_samples = sum(log.actual_subfloor(:));
    elapsed_s = 0.0;
    if ~isempty(log.t), elapsed_s = double(log.t(end)); end
    goal_reached = isfinite(t_reach);
    fair_exec = spinn3d_fair_time_contract(cfg, t_reach, traj.Tf);
    execution_audit=spinn3d_execution_hard_audit( ...
        log,plant.robot,limits,plant.bus,cfg.P_TOTAL_MAX,cfg.EXECUTION_AUDIT, ...
        cfg.WORKSPACE);
    planner_fallback_used = strcmp( ...
        char(getfield_def(sol,'seed_kind','')), ...
        'warmstart-numerical-fallback');
    accepted = goal_reached && fair_exec.pass && execution_audit.pass && ...
        ~planner_fallback_used;
    if goal_reached && ~execution_audit.pass
        termination_reason = 'execution_hard_constraint_violation';
    elseif goal_reached && fair_exec.reference_available && ~fair_exec.pass
        termination_reason = 'arrival_deadline_exceeded';
    end
    term = struct('reason',termination_reason, ...
                  'reached',goal_reached, ...
                  'accepted',accepted, ...
                  'planner_fallback_used',planner_fallback_used, ...
                  'goal_reached',goal_reached, ...
                  'progress_reached',s >= traj.Tf-1e-12, ...
                  'final_s',double(s), ...
                  'rho_end',min(1.0,double(s)/max(traj.Tf,1e-12)), ...
                  'final_ee_error_m',double(final_ee_err), ...
                  'final_dq_norm',double(final_dq_norm), ...
                  'elapsed_s',elapsed_s, ...
                  'time_cap_s',double(Tcap), ...
                  'noguard_t_reach_s',double(fair_exec.t_reach_noguard_s), ...
                  'arrival_deadline_s',double(fair_exec.deadline_s), ...
                  'delay_vs_noguard_s',double(fair_exec.delay_vs_noguard_s), ...
                  'time_contract_available',logical(fair_exec.reference_available), ...
                  'time_contract_pass',logical(fair_exec.pass), ...
                  'nominal_floor_time_s',double(Tnom), ...
                  'settle_allowance_s',double(settle_allowance), ...
                  'recovery_allowance_s',double(recovery_allowance), ...
                  'stall_timeout_s',double(stall_timeout), ...
                  'stop_stable_required_s',double(stop_stable_required), ...
                  'stop_stable_elapsed_s',double(stop_stable_elapsed), ...
                  'actual_guard_recovery_floor',double(actual_recovery_floor), ...
                  'allow_subfloor_recovery',logical(allow_subfloor_recovery), ...
                  'freeze_samples',double(freeze_samples), ...
                  'freeze_fraction',double(freeze_fraction), ...
                  'max_freeze_streak_s',double(max_freeze_streak_s), ...
                  'subfloor_samples',double(subfloor_samples), ...
                  'terminal_guard',terminal_guard);

    summary = summary_from_log_local(log, t_reach, getfield_def(sol,'Tref',NaN), getfield_def(sol,'seed_kind','warmstart'));
    dc_summary=spinn3d_dynamic_bus_summary(log,bus);
    dc_names=fieldnames(dc_summary);
    for dc_i=1:numel(dc_names),summary.(dc_names{dc_i})=dc_summary.(dc_names{dc_i});end
    summary.reached = term.reached;
    summary.goal_reached = term.goal_reached;
    summary.accepted = term.accepted;
    summary.termination_reason = term.reason;
    summary.rho_end = term.rho_end;
    summary.final_ee_error_m = term.final_ee_error_m;
    summary.final_dq_norm = term.final_dq_norm;
    summary.freeze_fraction = term.freeze_fraction;
    summary.max_freeze_streak_s = term.max_freeze_streak_s;
    summary.Pgrid_static_initial_W = power_init.Pgrid_static_W;
    summary.Pgrid_avg_initial_W = power_init.Pgrid_avg0_W;
    summary.power_init_scheme = power_init.scheme;
    out = struct();
    out.actual_guard_telemetry_schema='actual_guard_liveness_v3';
    out.log     = log;
    out.limits  = limits;
    out.gains   = gains;
    out.cfg     = cfg;
    out.t_reach = t_reach;
    out.payload = payload;
    out.plan0   = plan0;
    out.sol     = sol;
    out.summary = summary;
    out.power_init = power_init;
    out.reached = term.reached;
    out.goal_reached = term.goal_reached;
    out.accepted = term.accepted;
    out.time_contract_available = term.time_contract_available;
    out.time_contract_pass = term.time_contract_pass;
    out.termination_reason = term.reason;
    out.termination = term;
    out.terminal_guard = terminal_guard;
    out.fairness = fair_exec;
    out.diagnostic_horizon = horizon_info;
    out.traj_Tf = traj.Tf;
    out.reference = struct('kind','cartesian-bezier-dls', 'Tf',traj.Tf);
    out.execution_audit = execution_audit;

    if ~term.accepted && logical(cfg.WARMSTART.warn_on_incomplete)
        warning('spinn3d:warmstartIncomplete', ...
            ['Warm-start execution was not accepted: %s. ' ...
             'rho=%.6f, EE error=%.6g m, ||dq||=%.6g rad/s, ' ...
             'freeze=%.1f%% (max %.3g s), elapsed=%.3g/%.3g s.'], ...
            term.reason, term.rho_end, term.final_ee_error_m, term.final_dq_norm, ...
            100*term.freeze_fraction, term.max_freeze_streak_s, term.elapsed_s, term.time_cap_s);
    end

    out.perf=spinn3d_execution_perf(log,cfg.TS);
    out.plant_model=plant.meta;
    out.method_id='warmstart';

    if get_disable_plot_local(cfg)
        fig = [];
        return;
    end

    fig = figure('Name','Warm-start planned alpha demo','Visible','on');
    tlo = tiledlayout(fig,3,2,'Padding','compact','TileSpacing','compact');

    ax3 = nexttile(tlo,[2 1]); hold(ax3,'on'); grid(ax3,'on'); axis(ax3,'equal'); view(ax3,135,20);
    [X0,Y0,Z0] = chain_skeleton_xyz(robot, q0, eeName);
    if ~isempty(X0), plot3(ax3, X0, Y0, Z0, '-o','LineWidth',1.0,'DisplayName','Initial'); end
    [Xf,Yf,Zf] = chain_skeleton_xyz(robot, log.q(end,:), eeName);
    if ~isempty(Xf), plot3(ax3, Xf, Yf, Zf, 'm-o','MarkerFaceColor','m','LineWidth',1.0,'DisplayName','Final'); end
    plot3(ax3, log.ee(:,1), log.ee(:,2), log.ee(:,3), 'c-','LineWidth',1.5,'DisplayName','EE traj');
    plot3(ax3, ee_start(1),ee_start(2),ee_start(3),'go','MarkerFaceColor','g','DisplayName','Start');
    plot3(ax3, ee_goal(1), ee_goal(2), ee_goal(3),'co','MarkerFaceColor','c','DisplayName','Target');
    plot3(ax3, log.ee(end,1),log.ee(end,2),log.ee(end,3),'ro','MarkerFaceColor','r','DisplayName','Final EE');
    xlabel(ax3,'X [m]'); ylabel(ax3,'Y [m]'); zlabel(ax3,'Z [m]');
    title(ax3,'EE trajectory and robot skeleton'); legend(ax3,'Location','best');

    axA = nexttile(tlo); hold(axA,'on'); grid(axA,'on');
    plot(axA, log.t, log.a_plan,     'DisplayName','a_{plan}');
    plot(axA, log.t, log.a_post, '--', 'DisplayName','a_{post}');
    plot(axA, log.t, log.a_use,      'DisplayName','a_{use}');
    if isfield(log,'a_kin') && any(isfinite(log.a_kin))
        plot(axA, log.t, log.a_kin, '-.', 'DisplayName','a_{kin cap}');
    end
    plot(axA, log.t, log.a_feasible, ':', 'DisplayName','a_{feasible}');
    xlabel(axA,'t [s]'); ylabel(axA,'alpha'); title(axA,'Planned vs executed alpha');
    ylim(axA,[0,max(1.05,double(method_cfg.command_alpha_max)+0.05)]);
    if numel(log.t) >= 2 && log.t(end) > log.t(1), xlim(axA,[log.t(1), log.t(end)]); end
    legend(axA,'show','Location','best');

    axP = nexttile(tlo); hold(axP,'on'); grid(axP,'on');
    plot(axP, log.t, log.Pgrid, 'DisplayName','P_{grid}');
    if isfield(log,'Pshare') && ~isempty(log.Pshare)
        plot(axP, log.t, log.Pshare,'DisplayName','P_{share}');
    end
    if isfield(log,'Pgrid_avg') && ~isempty(log.Pgrid_avg) && any(isfinite(log.Pgrid_avg))
        plot(axP, log.t, log.Pgrid_avg, 'LineWidth',1.4, 'DisplayName','P_{grid} filtered bus');
    else

        win=max(1,round(double(bus.P_avg_window_s)/cfg.TS));
        plot(axP, log.t, movmean(log.Pgrid,win), 'LineWidth',1.4, 'DisplayName','P_{grid} filtered');
    end
    if isfield(limits,'P_total_max') && isfinite(limits.P_total_max)
        yline(axP, limits.P_total_max, '--r', 'P_{grid,avg,bud}');
    end
    xlabel(axP,'t [s]'); ylabel(axP,'Power [W]'); title(axP,'BUS power');
    if numel(log.t) >= 2 && log.t(end) > log.t(1), xlim(axP,[log.t(1), log.t(end)]); end
    legend(axP,'show','Location','best');
    spinn3d_plot_dynamic_bus(tlo,log,bus);

    % The original demo plotted only the initial/final poses after the
    % simulation.  That made a completed run look like a stationary robot.
    % Replay the recorded physical state (not the reference) as a lightweight
    % skeleton so freezes, restarts and actual motion are directly visible.
    animate_enable = logical(cfg.WARMSTART.animate_enable);
    animation_frame_dt = cfg.WARMSTART.animation_frame_dt_s;
    animation_speed = cfg.WARMSTART.animation_speed;
    animation_frame_dt = max(cfg.TS, ...
        sanitize_positive_scalar_local(animation_frame_dt, ...
        cfg.WARMSTART.animation_frame_dt_s));
    animation_speed = sanitize_positive_scalar_local( ...
        animation_speed,cfg.WARMSTART.animation_speed);
    if animate_enable && ~isempty(log.t) && ~isempty(log.q)
        frame_stride = max(1,round(animation_frame_dt/cfg.TS));
        frame_idx = unique([1:frame_stride:numel(log.t), numel(log.t)]);
        hPlayback = plot3(ax3,X0,Y0,Z0,'b-o','LineWidth',2.0, ...
            'MarkerFaceColor','b','DisplayName','Recorded state');
        title_base = sprintf('Recorded execution (%s)',term.reason);
        for ii = frame_idx
            if ~isgraphics(fig) || ~isgraphics(hPlayback), break; end
            [Xp,Yp,Zp] = chain_skeleton_xyz(robot,log.q(ii,:),eeName);
            if isempty(Xp), continue; end
            set(hPlayback,'XData',Xp,'YData',Yp,'ZData',Zp);
            title(ax3,sprintf('%s, t=%.2f s',title_base,log.t(ii)));
            drawnow;
            if ii ~= frame_idx(end)
                pause(animation_frame_dt/animation_speed);
            end
        end
        if isgraphics(ax3), title(ax3,title_base); end
    end
end

function S = terminal_actual_guard_snapshot_local(A)
% Preserve the actual emergency-hold candidate itself.  Per-sample telemetry
% intentionally reports the best non-hold trigger during recovery; the
% terminal snapshot below separately records why the force-zero hold failed.
S = struct('available',false);
if ~isstruct(A) || isempty(fieldnames(A)), return; end
flow = getfield_def(A,'flow',struct());
S = struct( ...
    'available',true, ...
    'hold_ok',logical(getfield_def(A,'hold_ok',false)), ...
    'hold_reason',char(getfield_def(A,'hold_reason','unknown')), ...
    'trigger_reason',char(getfield_def(A,'trigger_reason','unknown')), ...
    'selection_mode',char(getfield_def(A,'actual_search_mode','unknown')), ...
    'hold_alpha',double(getfield_def(A,'a',NaN)), ...
    'hold_Pgrid_W',double(getfield_def(flow,'Pgrid',NaN)), ...
    'hold_Pspill_W',double(getfield_def(flow,'Pspill',NaN)), ...
    'Pgrid_avg_prev_W',double(getfield_def(A,'Pgrid_avg_prev',NaN)), ...
    'Pgrid_avg_next_W',double(getfield_def(A,'Pgrid_avg_next',NaN)), ...
    'Pavg_cap_W',double(getfield_def(A,'Pavg_cap',NaN)), ...
    'Pavg_margin_W',double(getfield_def(A,'Pavg_margin_W',NaN)), ...
    'Pgrid_peak_cap_W',double(getfield_def(A,'Pgrid_peak_cap',NaN)), ...
    'hold_tau_ratio',double(getfield_def(A,'tau_ratio',NaN)), ...
    'hold_motor_ratio',double(getfield_def(A,'motor_ratio',NaN)), ...
    'hold_qd_ratio',double(getfield_def(A,'qd_ratio',NaN)), ...
    'hold_qdd_ratio',double(getfield_def(A,'qdd_ratio',NaN)), ...
    'hold_axis_power_ratio',double(getfield_def(A,'Pax_ratio',NaN)), ...
    'hold_pgrid_ratio',double(getfield_def(A,'pgrid_ratio',NaN)), ...
    'hold_pavg_ratio',double(getfield_def(A,'pavg_ratio',NaN)), ...
    'hold_ok_tau',logical(getfield_def(A,'ok_tau',false)), ...
    'hold_ok_motor_envelope',logical(getfield_def(A,'ok_motor_envelope',false)), ...
    'hold_ok_qd',logical(getfield_def(A,'ok_qd',false)), ...
    'hold_ok_qdd',logical(getfield_def(A,'ok_qdd',false)), ...
    'hold_ok_axis_power',logical(getfield_def(A,'ok_Pax',false)), ...
    'hold_ok_grid_peak',logical(getfield_def(A,'ok_grid_peak',false)), ...
    'hold_ok_grid_avg',logical(getfield_def(A,'ok_grid_avg',false)), ...
    'hold_ok_spill',logical(getfield_def(A,'ok_spill',false)), ...
    'hold_ok_thermal',logical(getfield_def(A,'ok_thermal',false)), ...
    'hold_ok_buffer',logical(getfield_def(A,'ok_buffer',false)), ...
    'trigger_best_nonhold',getfield_def(A,'trigger_best_nonhold',struct()), ...
    'trigger_nominal',getfield_def(A,'trigger_nominal',struct()));
end

function [limits, cfg] = apply_warmstart_kin_relax_local(limits, cfg, nJ)
% Relax the alpha-governor kinematic caps only for warm-start execution.
% This makes the BUS/torque constraints, rather than a very small qdd cap,
% determine the usable alpha.  It does not disable the actual BUS guard.
if nargin < 3 || isempty(nJ), nJ = numel(row(getfield_def(limits,'tau_max',ones(1,4)))); end
if ~logical(cfg.WARMSTART.relax_kin_limits_enable)
    return;
end

qdd_feas=getfield_def(cfg.WARMSTART,'qdd_max_feas',[]);
if ~isempty(qdd_feas)
    limits.qdd_max = max_limit_local(getfield_def(limits,'qdd_max',inf(1,nJ)), qdd_feas, nJ);
end

qdd_actual=getfield_def(cfg.WARMSTART,'qdd_cmd_max_actual',[]);
if ~isempty(qdd_actual)
    limits.qdd_cmd_max_actual = max_limit_local(getfield_def(limits,'qdd_cmd_max_actual',getfield_def(limits,'qdd_max',inf(1,nJ))), qdd_actual, nJ);
end

qd_exec=getfield_def(cfg.WARMSTART,'qd_max_exec',[]);
if ~isempty(qd_exec)
    limits.qd_max = max_limit_local(getfield_def(limits,'qd_max',inf(1,nJ)), qd_exec, nJ);
end

cfg.CAPS = limits;
end

function v = max_limit_local(v0, v1, n)
v0 = expand_limit_local(v0, n, inf);
v1 = expand_limit_local(v1, n, inf);
v = max(v0, v1);
end

function v = expand_limit_local(v, n, def)
if isempty(v)
    v = def*ones(1,n);
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

function [a_pre, a_kin, clip, qdd_ratio, tau_ratio, qd_ratio, pgrid_ratio, pavg_ratio, spill, ...
          freeze, floor_infeasible, restart_bisect, backtrack_iters, violation] = ...
          warmstart_guard_diag_local(info, limits, nJ)
a_pre = NaN; a_kin = NaN; clip = NaN; qdd_ratio = NaN; tau_ratio = NaN; qd_ratio = NaN;
pgrid_ratio = NaN; pavg_ratio = NaN; spill = NaN;
freeze = false; floor_infeasible = false; restart_bisect = false;
backtrack_iters = NaN; violation = NaN;
if ~(isstruct(info) && isfield(info,'guard') && isstruct(info.guard))
    return;
end
G = info.guard;
a_pre = getfield_def(G,'a_nom_pre_actual',NaN);
a_kin = getfield_def(G,'a_kin',NaN);
if isfield(G,'actual') && isstruct(G.actual)
    A = G.actual;
    clip = getfield_def(A,'a_clip', max(0, a_pre - getfield_def(A,'a',NaN)));
    freeze = logical(getfield_def(A,'freeze_on_infeasible',false));
    floor_infeasible = logical(getfield_def(A,'floor_infeasible',false));
    restart_bisect = logical(getfield_def(A,'restart_bisect',false));
    backtrack_iters = double(getfield_def(A,'backtrack_iters',NaN));
    violation = double(getfield_def(A,'violation',NaN));
    qdd_lim = expand_limit_local(getfield_def(limits,'qdd_cmd_max_actual',getfield_def(limits,'qdd_max',inf(1,nJ))), nJ, inf);
    tau_lim = expand_limit_local(getfield_def(limits,'tau_max',inf(1,nJ)), nJ, inf);
    qd_lim  = expand_limit_local(getfield_def(limits,'qd_max',inf(1,nJ)), nJ, inf);
    if isfield(A,'ddq') && ~isempty(A.ddq), qdd_ratio = max(abs(row(A.ddq))./max(qdd_lim,1e-12)); end
    if isfield(A,'tau') && ~isempty(A.tau), tau_ratio = max(abs(row(A.tau))./max(tau_lim,1e-12)); end
    if isfield(A,'dq')  && ~isempty(A.dq),  qd_ratio  = max(abs(row(A.dq))./max(qd_lim,1e-12)); end
    if isfield(A,'flow') && isstruct(A.flow)
        spill = getfield_def(A.flow,'Pspill',NaN);
        Pg = getfield_def(A.flow,'Pgrid',NaN);
        Ppk = getfield_def(A,'Pgrid_peak_cap',NaN);
        if isfinite(Pg) && isfinite(Ppk) && Ppk > 0, pgrid_ratio = Pg / Ppk; end
    end
    Pavg = getfield_def(A,'Pgrid_avg_next',NaN);
    Pcap = getfield_def(A,'Pavg_cap',NaN);
    if isfinite(Pavg) && isfinite(Pcap) && Pcap > 0, pavg_ratio = Pavg / Pcap; end
    % On an emergency hold A describes the safe zero command.  Use the

    % would misleadingly appear as zero violation after the hold.
    if freeze && isfield(A,'trigger_best_nonhold') && isstruct(A.trigger_best_nonhold)
        D = A.trigger_best_nonhold;
        violation  = double(getfield_def(D,'violation',violation));
        qdd_ratio  = double(getfield_def(D,'qdd_ratio',qdd_ratio));
        tau_ratio  = double(getfield_def(D,'tau_ratio',tau_ratio));
        qd_ratio   = double(getfield_def(D,'qd_ratio',qd_ratio));
        pgrid_ratio= double(getfield_def(D,'pgrid_ratio',pgrid_ratio));
        pavg_ratio = double(getfield_def(D,'pavg_ratio',pavg_ratio));
        spill      = double(getfield_def(D,'Pspill',spill));
    end
end
end

function tf = isfinite_scalar_positive_local(x)
tf = isnumeric(x) && isscalar(x) && isfinite(double(x)) && double(x) > 0;
end

function nmax = max_true_run_local(mask)
mask = logical(mask(:));
nmax = 0;
ncur = 0;
for ii = 1:numel(mask)
    if mask(ii)
        ncur = ncur + 1;
        nmax = max(nmax,ncur);
    else
        ncur = 0;
    end
end
end

function tf = get_disable_plot_local(cfg)

tf = ~logical(cfg.PLOT.enable);
if isfield(cfg,'DISABLE_PLOT') && ~isempty(cfg.DISABLE_PLOT)
    tf = logical(cfg.DISABLE_PLOT);
    return;
end
end

function a = plan_lookup_local(plan, rho, alpha_floor, alpha_max)
rho = min(1.0, max(0.0, double(rho)));
if nargin < 3 || isempty(alpha_floor), alpha_floor = 0; end

if isfield(plan,'rho') && isfield(plan,'a_des_seq') && ~isempty(plan.rho) && ~isempty(plan.a_des_seq)
    xr = row(plan.rho);
    ya = row(plan.a_des_seq);
elseif isfield(plan,'rho_knots') && isfield(plan,'a_knots') && ~isempty(plan.rho_knots) && ~isempty(plan.a_knots)
    xr = row(plan.rho_knots);
    ya = row(plan.a_knots);
else
    error('plan 需要包含 rho+a_des_seq 或 rho_knots+a_knots');
end

if numel(xr) ~= numel(ya)
    error('plan 的 rho 与 alpha 序列长度不一致');
end
if isempty(xr)
    a = alpha_floor;
    return;
end
if numel(xr) == 1
    a = ya(1);
else
    a = interp1(xr, ya, rho, 'linear', 'extrap');
end
a = min(double(alpha_max), max(alpha_floor, double(a)));
end

% ----------------- helpers -----------------
function summary = summary_from_log_local(log, t_reach, Tref, seed_kind)
summary = struct('t_reach',t_reach,'Tref',Tref,'peakPgrid',NaN,'peakPdump',NaN, ...
                 'Egrid',0,'Edump',0,'Eshare',0,'Ereuse_buf',0,'Ereuse_total',0, ...
                 'Eshare_axis',0,'Eoverlap',0,'Eregen_available',0,'Eregen_bus',0, ...
                 'Eregen_curtailed',0,'Eregen_curtailed_total',0, ...
                 'energy_balance_residual_max_W',NaN,'energy_balance_residual_rms_W',NaN, ...
                 'Espill',0,'Ebuf_min',0,'seed_kind',seed_kind);
if isempty(log) || ~isfield(log,'t') || isempty(log.t)
    return;
end
t = log.t(:);
if isfield(log,'Pgrid') && ~isempty(log.Pgrid), summary.peakPgrid = max(log.Pgrid(:)); summary.Egrid = spinn3d_interval_integral(t, log.Pgrid(:)); end
if isfield(log,'Pdump') && ~isempty(log.Pdump), summary.peakPdump = max(log.Pdump(:)); summary.Edump = spinn3d_interval_integral(t, log.Pdump(:)); end
if isfield(log,'Pshare') && ~isempty(log.Pshare), summary.Eshare = spinn3d_interval_integral(t, log.Pshare(:)); end
if isfield(log,'Pshare_axis') && ~isempty(log.Pshare_axis), summary.Eshare_axis = spinn3d_interval_integral(t, log.Pshare_axis(:)); else, summary.Eshare_axis=summary.Eshare; end
if isfield(log,'Pshare_potential') && ~isempty(log.Pshare_potential), summary.Eoverlap = spinn3d_interval_integral(t, log.Pshare_potential(:)); end
if isfield(log,'Pregen_available') && ~isempty(log.Pregen_available), summary.Eregen_available = spinn3d_interval_integral(t, log.Pregen_available(:)); end
if isfield(log,'Pregen_bus') && ~isempty(log.Pregen_bus), summary.Eregen_bus = spinn3d_interval_integral(t, log.Pregen_bus(:)); end
if isfield(log,'Pregen_curtailed') && ~isempty(log.Pregen_curtailed), summary.Eregen_curtailed = spinn3d_interval_integral(t, log.Pregen_curtailed(:)); end
if isfield(log,'Pregen_curtailed_total') && ~isempty(log.Pregen_curtailed_total), summary.Eregen_curtailed_total = spinn3d_interval_integral(t, log.Pregen_curtailed_total(:)); end
if isfield(log,'energy_balance_residual_W') && ~isempty(log.energy_balance_residual_W), summary.energy_balance_residual_max_W=max(abs(log.energy_balance_residual_W(:))); summary.energy_balance_residual_rms_W=sqrt(mean(log.energy_balance_residual_W(:).^2)); end
if isfield(log,'Preuse_buf') && ~isempty(log.Preuse_buf), summary.Ereuse_buf = spinn3d_interval_integral(t, log.Preuse_buf(:)); end
if isfield(log,'Preuse_total') && ~isempty(log.Preuse_total)
    summary.Ereuse_total = spinn3d_interval_integral(t, log.Preuse_total(:));
else
    summary.Ereuse_total = summary.Eshare;
end
if isfield(log,'Pspill') && ~isempty(log.Pspill), summary.Espill = spinn3d_interval_integral(t, log.Pspill(:)); end
if isfield(log,'Ebuf') && ~isempty(log.Ebuf), summary.Ebuf_min = min(log.Ebuf(:)); end
end

function req(S, keys)
for i=1:numel(keys),  end
end
function policy_dt = get_policy_dt_local(cfg, Ts)

policy_dt = double(cfg.POLICY.dt);
ratio = policy_dt/double(Ts);

end
function x = getfield_def(S,fn,def), if isstruct(S)&&isfield(S,fn)&&~isempty(S.(fn)), x=S.(fn); else, x=def; end, end
function tf = valid_binary_field_local(S,field)
v = [];
if isstruct(S) && isfield(S,field), v = S.(field); end
tf = (islogical(v) || isnumeric(v)) && isscalar(v) && ...
     isreal(v) && isfinite(double(v)) && any(double(v)==[0 1]);
end
function x = sanitize_positive_scalar_local(x,def)
if ~(isnumeric(x) || islogical(x)) || ~isscalar(x) || ~isfinite(x) || x <= 0
    x = def;
else
    x = double(x);
end
end
function r = row(v), r=v(:)'; end
function x = deg2rad_local(d), x = pi/180 * row(d); end
function qf = unwrap_to_near(q0, qf)
d = qf - q0; qf = q0 + atan2(sin(d), cos(d));
end
function tau = clamp_tau(tau, tau_max)
tau = row(tau); tmax = row(tau_max); tau = max(-tmax, min(tmax, tau));
end
function v = vec_lim(lim, fn, n, def)
if isfield(lim,fn) && ~isempty(lim.(fn)), v=row(lim.(fn)); else, v=def*ones(1,n); end
if isscalar(v), v = repmat(v,1,n); end
v = v(1:n);
end
function [X,Y,Z] = chain_skeleton_xyz(robot, q, eeName)
X=[];Y=[];Z=[];
try
    b = robot.getBody(eeName); names={};
    while ~strcmp(b.Name, robot.BaseName)
        names{end+1} = b.Name;
        b = robot.getBody(b.Parent);
    end
    names = fliplr(names);
catch
    names = robot.BodyNames;
end
for i=1:numel(names)
    try
        Tb = getTransform(robot, q, names{i});
        p  = tform2trvec(Tb);
        X(end+1)=p(1); Y(end+1)=p(2); Z(end+1)=p(3); %#ok<AGROW>
    catch
    end
end
try
    Tee = getTransform(robot, q, eeName); p = tform2trvec(Tee);
    if isempty(X) || norm([X(end)-p(1),Y(end)-p(2),Z(end)-p(3)])>1e-12
        X(end+1)=p(1); Y(end+1)=p(2); Z(end+1)=p(3);
    end
catch
end
end
function p = ee_pos(robot, q, eeName), p = tform2trvec(getTransform(robot,q,eeName)); end
function p = tform2trvec_safe(T), try, p=tform2trvec(T); catch, p=[NaN NaN NaN]; end, end
function log = truncate_log(log, k)
f = fieldnames(log);
for i=1:numel(f), log.(f{i}) = log.(f{i})(1:k,:); end
end
function S = v10_to_struct(v)
v = row(v); v = [v, zeros(1,10-numel(v))];
S = struct('mass',v(1), 'com',v(2:4), 'inertia',v(5:10));
end
function [robot_out, pay_vec] = inject_payload_last_body(robot_in, pay)
if exist('spinn3d_inject_payload_last_body','file') == 2
    [robot_out,pay_vec] = spinn3d_inject_payload_last_body(robot_in,pay);
    return;
end
    robot_out = robot_in;
    pay_vec   = zeros(1,10);
    if isempty(pay), return; end

    if isnumeric(pay)
        v = double(pay(:)).';
        if numel(v) < 10, v = [v, zeros(1,10-numel(v))]; end
        m  = v(1); com = v(2:4); I6 = v(5:10);
        about = 'com';
        mode  = 'replace';
    elseif isstruct(pay)
        m   = getfield_def(pay,'mass',0);
        com = getfield_def(pay,'com',[0 0 0]);
        I6  = getfield_def(pay,'inertia',[0 0 0 0 0 0]);
        about = lower(getfield_def(pay,'about','com'));
        mode  = lower(getfield_def(pay,'mode','replace'));
    else
        return;
    end
    pay_vec = [double(m), double(com(:)).', double(I6(:)).'];

    try
        b  = robot_out.Bodies{end};
        m0 = double(b.Mass);
        c0 = double(b.CenterOfMass(:)).';
        I0 = I6ToMat(double(b.Inertia(:)).');

        Ic = I6ToMat(double(I6(:)).');
        if strcmpi(about,'com')
            r  = double(com(:)); r = r(:);
            I1 = Ic + double(m) * ((r.'*r)*eye(3) - (r*r.'));
        else
            I1 = Ic;
        end

        switch mode
            case 'add'
                m_new  = m0 + m;
                if m_new <= 0
                    c_new = [0 0 0];
                else
                    c_new = (m0*c0 + m*com(:).') / m_new;
                end
                I_new = I0 + I1;
            otherwise
                m_new  = m;
                c_new  = com(:).';
                I_new  = I1;
        end

        b.Mass         = m_new;
        b.CenterOfMass = c_new;
        b.Inertia      = ImatTo6(I_new).';
    catch
    end
end
function I = I6ToMat(v6)
    v6 = double(v6(:)).';
    Ixx=v6(1); Iyy=v6(2); Izz=v6(3);
    Iyz=v6(4); Ixz=v6(5); Ixy=v6(6);
    I = [ Ixx Ixy Ixz;
          Ixy Iyy Iyz;
          Ixz Iyz Izz ];
end
function v6 = ImatTo6(I)
    v6 = [ I(1,1), I(2,2), I(3,3), I(2,3), I(1,3), I(1,2) ];
end
