function sol = spinn3d_alpha_plan_refine(robot, traj, cfg, plan0, opts)
%SPINN3D_ALPHA_PLAN_REFINE
% Refine the warm-start alpha plan on a fixed geometry trajectory.
%
% Current contract:
%   - rollout cost can reuse explicit plan0.bus_state0
%   - fixed-path refine uses the same unified bus-aware score helper as the
%     joint theta+alpha refine for better objective alignment

    if nargin < 5 || isempty(opts), opts = struct(); end

    req(cfg, {'TS','ALPHA_MIN','ALPHA_MAX','ALPHA_ITMAX'});
    alpha_max = double(cfg.ALPHA_MAX);

    

    plan_cfg = cfg.PLAN;
    required_plan_fields = {'NKNOT','POP','NITER','NELITE','SIGMA0', ...
        'SIGMA_MIN','no_feasible_sigma_decay', ...
        'warmstart_rollout_use_alpha_dot','refine_use_alpha_dot', ...
        'rho_dense_min_points','W', ...
        'SAMPLE_UP_BIAS','parallel_enable', ...
        'parallel_autostart','parallel_workers','parallel_profile', ...
        'parallel_min_pop','parallel_idle_timeout_min','objective'};

    Nknot    = getfield_def(opts, 'NKNOT', plan_cfg.NKNOT);
    POP      = getfield_def(opts, 'POP', plan_cfg.POP);
    NITER    = getfield_def(opts, 'NITER', plan_cfg.NITER);
    NELITE   = getfield_def(opts, 'NELITE', plan_cfg.NELITE);
    SIGMA0   = getfield_def(opts, 'SIGMA0', plan_cfg.SIGMA0);
    SIGMA_MIN= getfield_def(opts, 'SIGMA_MIN', plan_cfg.SIGMA_MIN);

    no_feasible_sigma_decay = double(plan_cfg.no_feasible_sigma_decay);
    refine_use_alpha_dot = require_binary_scalar_local( ...
        plan_cfg,'refine_use_alpha_dot','PB.plan.refine_use_alpha_dot');

    Ndense   = getfield_def(opts, 'RHO_DENSE_N', ...
        max(plan_cfg.rho_dense_min_points, ...
            ceil(traj.Tf/max(cfg.TS,eps))+1));
    % Alpha CEM is a policy-rate search surrogate.  Its selected plan is
    % rechecked later by the authoritative cfg.TS full-plant loop.
    eval_dt_requested = get_policy_dt_local(cfg,double(cfg.TS));
    if isfield(opts,'eval_dt') && ~isempty(opts.eval_dt)
        supplied_dt = double(opts.eval_dt);

    end
    % opts.eval_dt is only an exact contract assertion.  It cannot select a
    % private cadence or force the proxy back to the plant servo period.
    % The desired plan remains policy ZOH, but every candidate replays the
    % same 1-ms DC-link/alpha transition as teacher and execution.
    eval_dt = double(cfg.TS);
    cfg_eval = cfg;
    cfg_eval.TS = eval_dt;

    validate_objective_cfg_local(plan_cfg.objective);
    W = plan_cfg.W;
    if isfield(opts,'W') && isstruct(opts.W)
        W = merge_struct(W, opts.W);
    end

    limits = cfg.CAPS;
    if ~isfield(limits,'P_total_max') || isempty(limits.P_total_max)
        limits.P_total_max = cfg.P_TOTAL_MAX;
    end
    bus   = get_bus_from_cfg(cfg, limits.P_total_max);
    Pcap  = double(limits.P_total_max);
    plan0 = ensure_static_power_init_local(plan0,robot,traj,cfg_eval,bus,Pcap);
    init_check = getfield_def(plan0,'power_init',struct());
    tau_lim_hold = expand_limit_refine_local( ...
        getfield_def(limits,'tau_max',inf),numel(init_check.tau_static_Nm));
    tau_hold_ratio = max(abs(double(init_check.tau_static_Nm(:).')) ./ ...
        max(abs(tau_lim_hold),eps));
    if ~getfield_def(init_check,'static_source_feasible',false) || ...
            ~isfinite(tau_hold_ratio) || tau_hold_ratio>1+1e-9
        error('spinn3d_alpha_plan_refine:InitialStaticHoldInfeasible', ...
            ['The q0 static hold is infeasible before CEM: Pgrid=%.6g W, ', ...
             'Pavg KPI target=%.6g W, tau ratio=%.6g, motor_ok=%d.'], ...
            getfield_def(init_check,'Pgrid_static_W',NaN), ...
            getfield_def(init_check,'Pavg_cap_W',NaN),tau_hold_ratio, ...
            double(getfield_def(init_check,'motor_envelope_initial_ok',false)));
    end
    plan0.power_init.tau_limit_ratio = tau_hold_ratio;
    plan0.power_init.static_hold_feasible = true;

    sample_up_bias  = getfield_def(opts, 'SAMPLE_UP_BIAS', ...
        plan_cfg.SAMPLE_UP_BIAS);

    fair = spinn3d_fair_time_contract(cfg,NaN,NaN);
    if ~isfinite(fair.t_reach_noguard_s)
        error('spinn3d_alpha_plan_refine:MissingNoGuardReference', ...
            'Attach the paired full-plant NoGuard reference before alpha refinement.');
    end
    Tref = max(fair.t_reach_noguard_s,cfg.TS);
    plan_eval = compact_plan0_for_eval(plan0);
    traj_eval = compact_traj_for_eval(traj);
    parcfg = prepare_parallel_cfg(plan_cfg, opts, POP);
    parcfg = ensure_parallel_pool(parcfg);
    useParallel = want_parallel_eval(parcfg);

    [rho_knots0, a_knots0] = canonicalize_knots( ...
        plan0, Nknot, cfg.ALPHA_MIN, alpha_max);
    rho_knots = row(rho_knots0);
    mu        = row(a_knots0);
    sigma     = SIGMA0 * ones(size(mu));

    [seed_hi, has_seed_hi] = build_feasible_seed_from_plan0( ...
        plan0,rho_knots,cfg.ALPHA_MIN,alpha_max);

    [J0, status0, info0] = rollout_cost_safe_local( ...
        mu, rho_knots, robot, traj_eval, ...
        cfg_eval, limits, bus, W, plan_eval, Tref, true, ...
        refine_use_alpha_dot);
    saw_no_feasible_alpha = (status0 == 1);
    bestJ       = J0;
    bestKnots   = mu;
    bestInfo    = info0;

    histJ  = nan(NITER+1,1);
    histJ(1) = J0;

    if has_seed_hi
        [Jseed, status_seed, infoseed] = rollout_cost_safe_local( ...
            seed_hi, rho_knots, robot, ...
            traj_eval, cfg_eval, limits, bus, W, plan_eval, Tref, true, ...
            refine_use_alpha_dot);
        saw_no_feasible_alpha = saw_no_feasible_alpha || (status_seed == 1);
        if Jseed < bestJ
            bestJ     = Jseed;
            bestKnots = seed_hi;
            bestInfo  = infoseed;
        end
        histJ(1) = min(histJ(1), Jseed);
    end

    for it = 1:NITER
        C = zeros(POP, Nknot);
        cidx = 1;
        C(cidx,:) = mu;
        if has_seed_hi && cidx < POP
            cidx = cidx + 1;
            C(cidx,:) = seed_hi;
        end
        if cidx < POP
            cidx = cidx + 1;
            C(cidx,:) = alpha_max*ones(1,Nknot);
        end
        if cidx < POP
            cidx = cidx + 1;
            C(cidx,:) = clamp(0.5*(mu + alpha_max), cfg.ALPHA_MIN, alpha_max);
        end
        if has_seed_hi && cidx < POP
            cidx = cidx + 1;
            C(cidx,:) = clamp(0.5*(seed_hi + alpha_max), cfg.ALPHA_MIN, alpha_max);
        end
        for i = cidx+1:POP
            C(i,:) = mu + sigma .* randn(1, Nknot) + sample_up_bias * sigma;
        end
        C = clamp(C, cfg.ALPHA_MIN, alpha_max);

        J = inf(POP,1);
        reject_status = zeros(POP,1);
        if useParallel
            parfor i = 1:POP
                [J(i),reject_status(i)] = rollout_cost_safe_local( ...
                    C(i,:),rho_knots,robot,traj_eval, ...
                    cfg_eval,limits,bus,W,plan_eval,Tref,false, ...
                    refine_use_alpha_dot);
            end
        else
            for i = 1:POP
                [J(i),reject_status(i)] = rollout_cost_safe_local( ...
                    C(i,:),rho_knots,robot,traj_eval, ...
                    cfg_eval,limits,bus,W,plan_eval,Tref,false, ...
                    refine_use_alpha_dot);
            end
        end
        saw_no_feasible_alpha = saw_no_feasible_alpha || ...
            any(reject_status == 1);

        [Js, ord] = sort(J, 'ascend');
        finite_mask = isfinite(Js);
        if ~any(finite_mask)
            histJ(it+1) = bestJ;
            sigma = max(no_feasible_sigma_decay*sigma,SIGMA_MIN);
            continue;
        end
        Js  = Js(finite_mask);
        ord = ord(finite_mask);

        elite_idx = ord(1:min(NELITE, numel(ord)));
        elite     = C(elite_idx,:);

        mu    = mean(elite, 1);
        sigma = max(std(elite, 0, 1), SIGMA_MIN);
        histJ(it+1) = Js(1);

        if Js(1) < bestJ
            bestJ     = Js(1);
            bestKnots = C(ord(1),:);
        end
    end

    if ~isfinite(bestJ)
        if saw_no_feasible_alpha
            error('spinn3d_alpha_plan_refine:NoFeasibleAlphaPlan', ...
                ['No CEM candidate completed with a feasible alpha command.  ', ...
                 'The diagnostic alpha_min floor was rejected, not executed.']);
        end
        error('spinn3d_alpha_plan_refine:NoNumericalPlan', ...
            ['No numerically valid alpha plan was produced.  Deadline misses ', ...
             'are soft-ranked inside CEM and do not cause this error.']);
    end

    [bestJ,best_status,bestInfo] = rollout_cost_safe_local( ...
        bestKnots,rho_knots,robot, ...
        traj_eval,cfg_eval,limits,bus,W,plan_eval,Tref,true, ...
        refine_use_alpha_dot);
    if best_status == 1
        error('spinn3d_alpha_plan_refine:NoFeasibleAlphaPlan', ...
            ['The selected Alpha plan failed its final feasibility replay; ', ...
             'the diagnostic alpha_min floor was not accepted.']);
    end
    if ~isfinite(bestJ)
        error('spinn3d_alpha_plan_refine:NoNumericalPlan', ...
            ['No numerically valid alpha plan was produced.  Deadline misses ', ...
             'are soft-ranked inside CEM and do not cause this error.']);
    end

    rho_dense   = linspace(0,1,Ndense);
    a_des_dense = interp1(rho_knots, bestKnots, rho_dense, 'pchip', 'extrap');
    a_des_dense = clamp(a_des_dense, cfg.ALPHA_MIN, alpha_max);

    sol = struct();
    sol.J0        = J0;
    sol.Jbest     = bestJ;
    sol.Tref      = Tref;
    sol.TnoGuard  = fair.t_reach_noguard_s;
    sol.time_deadline_s = fair.deadline_s;
    sol.max_delay_vs_noguard_s = fair.max_delay_vs_noguard_s;
    sol.numerical_feasible = logical(getfield_def(bestInfo,'reached',false));
    sol.runnable = sol.numerical_feasible;
    sol.proxy_physical_feasible = ...
        getfield_def(bestInfo,'floor_infeasible_count',inf) == 0;
    sol.proxy_time_contract_pass = logical(getfield_def(bestInfo,'proxy_time_contract_pass',false));

    % the authoritative paper decision, while runnable says whether this
    % same-method plan can be sent to that verifier.
    sol.feasible = sol.runnable && sol.proxy_physical_feasible && ...
                   sol.proxy_time_contract_pass;
    sol.seed_kind = getfield_def(plan0, 'seed_kind', 'warmstart');
    sol.plan0     = plan0;
    sol.rho_knots = rho_knots;
    sol.a_knots   = bestKnots;
    sol.plan      = struct('rho', row(rho_dense), ...
                           'a_des_seq', row(a_des_dense), ...
                           'rho_knots', rho_knots, ...
                           'a_knots', bestKnots, ...
                           'Pgrid_avg0', getfield_def(bestInfo,'Pgrid_avg0',NaN), ...
                           'Pgrid_static_init_W', getfield_def(bestInfo,'Pgrid_static_init_W',NaN), ...
                           'power_init', getfield_def(bestInfo,'power_init',struct()), ...
                           'bus_state0', getfield_def(bestInfo,'bus_state0',[]), ...
                           'bus_state_final', getfield_def(bestInfo,'bus_state_final',[]), ...
                           'score_terms', getfield_def(bestInfo,'score_terms',struct()));
    sol.histJ     = histJ;
    sol.info0     = info0;
    sol.info_best = bestInfo;
    sol.weights   = W;
    sol.Pcap      = Pcap;
    sol.proxy_eval_dt = eval_dt;
    sol.proxy_eval_dt_requested = eval_dt_requested;
    sol.parallel  = struct('requested', parcfg.requested, 'used', useParallel, ...
                           'autostart', parcfg.autostart, ...
                           'workers', parcfg.actual_workers, ...
                           'requested_workers', parcfg.workers, ...
                           'min_pop', parcfg.min_pop, 'reason', parcfg.reason);
end

function [J,info] = rollout_cost(a_knots,rho_knots,robot,traj,cfg, ...
        limits,bus,W,plan0,Tref,need_info,refine_use_alpha_dot)
    if nargin < 11 || isempty(need_info)
        need_info = true;
    end
    keep_trace   = logical(need_info);
    Ts           = double(cfg.TS);
    policy_dt    = get_policy_dt_local(cfg, Ts);
    alpha_floor  = double(cfg.ALPHA_MIN);
    alpha_max    = double(cfg.ALPHA_MAX);
    itmax        = double(cfg.ALPHA_ITMAX);
    Pcap         = double(limits.P_total_max);
    areg         = cfg.ALPHA_REG;
    fric         = cfg.FRIC;
    use_state    = isfield(bus,'enable_state') && ~isempty(bus.enable_state) && logical(bus.enable_state);

    caps_feas = limits;
    caps_feas.eta_share  = getfield_def(bus,'eta_share', getfield_def(limits,'eta_share',1.0));
    caps_feas.P_brk_peak = getfield_def(bus,'P_brk_peak', getfield_def(bus,'P_dump_max', getfield_def(limits,'P_brk_peak',inf)));
    caps_feas.bus = bus;
    caps_feas.bus_Ts = Ts;
    if use_state
        st_bus0 = resolve_bus_state0(plan0, bus);
        st_bus  = st_bus0;
    else
        st_bus0 = [];
        st_bus  = [];
    end

    fair = spinn3d_fair_time_contract(cfg,NaN,NaN);
    objective_cfg = cfg.PLAN.objective;
    % Search horizon and acceptance deadline are deliberately separate.
    % Truncating every surrogate candidate at the configured NoGuard-relative deadline makes a complete
    % CEM generation Inf and removes the gradient toward a faster plan.
    Nmax = max(4, ceil(traj.Tf / max(alpha_floor, 1e-6) / Ts) + 4);
    if keep_trace
        Pgrid = zeros(Nmax,1); Pdump = zeros(Nmax,1); Pspill = zeros(Nmax,1); Preuse_total = zeros(Nmax,1);
        Jerk  = zeros(Nmax,1); Ause  = zeros(Nmax,1); Astar = zeros(Nmax,1); Headroom = zeros(Nmax,1); Rho = zeros(Nmax,1); CapLoss = zeros(Nmax,1);
        Ebuf = zeros(Nmax,1); Tbrk = zeros(Nmax,1); Ereuse = zeros(Nmax,1);
    end

    pcap_safe = max(Pcap, 1e-12);
    pgrid_norm_sum = 0;
    pgrid_norm_sq_sum = 0;
    dpdt_norm_sq_sum = 0;
    jerk_sq_sum = 0;
    cap_sq_sum = 0;
    headroom_sq_sum = 0;
    util_sum = 0;
    util_peak = 0;
    over_budget_acc = 0;
    Pgrid_avg_lp = getfield_def(plan0, 'Pgrid_avg0', NaN);
    if ~isnumeric(Pgrid_avg_lp) || ~isscalar(Pgrid_avg_lp) || ...
            ~isfinite(double(Pgrid_avg_lp)) || Pgrid_avg_lp < 0
        error('spinn3d_alpha_plan_refine:MissingPowerInitialization', ...
            ['Candidate rollout requires the q0 static-hold filtered-power ', ...
             'initial condition; zero fallback is forbidden.']);
    end
    Pgrid_avg_lp = double(Pgrid_avg_lp);
    power_init = getfield_def(plan0,'power_init',struct());
    Pgrid_static_init_W = getfield_def(plan0,'Pgrid_static_init_W',NaN);
    pavg_util_sum = 0;
    pavg_under_acc = 0;
    pavg_over_acc = 0;
    pavg_peak = Pgrid_avg_lp;

    Egrid = 0;
    Edump = 0;
    Ereuse_total = 0;
    Espill = 0;
    peakPgrid = max(double(Pgrid_static_init_W),0);
    if ~isfinite(peakPgrid), peakPgrid = 0; end
    util_peak = min(max(peakPgrid / pcap_safe,0),1);
    peakPdump = 0;
    peakPspill = 0;

    if use_state
        Ebuf_min = st_bus.Ebuf;
        Tbrk_peak = st_bus.Tbrk;
    else
        Ebuf_min = 0;
        Tbrk_peak = getfield_def(bus,'Tamb_C',25);
    end

    s = 0.0;
    a_prev = alpha_floor;
    t = 0.0;
    k = 0;
    ddq_prev = [];
    prev_pgrid = NaN;
    reached = false;
    next_policy_t = 0.0;
    a_des_hold = alpha_floor;
    % Compatibility metric retained for consumers of older solution structs.
    % A no-island result now rejects the whole candidate, so every accepted
    % rollout necessarily leaves this count at zero.
    floor_infeasible_count = 0;

    while (s < traj.Tf - 1e-12) && (k < Nmax)
        [q_ref, dq_ref, ddq_ref] = sample_traj_eval(traj, s);
        q_ref = row(q_ref);
        dq_ref = row(dq_ref);
        ddq_ref = row(ddq_ref);

        rho = min(1.0, s / max(traj.Tf, 1e-12));
        tol_policy = max(1e-12, 1e-9 * max(1.0, abs(t)));
        if t + tol_policy >= next_policy_t
            a_des_hold = interp1(rho_knots, a_knots, rho, 'linear', 'extrap');
            a_des_hold = clamp(double(a_des_hold), alpha_floor, alpha_max);
            while next_policy_t <= t + tol_policy
                next_policy_t = next_policy_t + policy_dt;
            end
        end
        a_des = a_des_hold;

        caps_step = caps_feas;
        if use_state
            caps_step.bus_state = st_bus;
        end
        caps_step.Pgrid_avg_prev = Pgrid_avg_lp;
        caps_step.alpha_prev = a_prev;
        caps_step.alpha_Ts = Ts;
        caps_step.use_alpha_dot = refine_use_alpha_dot;
        [a_star, a_star_info] = alpha_feasible(q_ref, dq_ref, ddq_ref, 1.0, robot, caps_step, Pcap, alpha_floor, itmax, fric);
        if ~isstruct(a_star_info) || ~isscalar(a_star_info) || ...
                ~valid_binary_field_local(a_star_info,'any_feasible') || ...
                ~valid_binary_field_local(a_star_info,'no_feasible')
            error('spinn3d_alpha_plan_refine:InvalidFeasibilityInfo', ...
                ['alpha_feasible returned malformed feasibility metadata at ', ...
                 't=%.9g s, path=%.9g/%.9g.'],t,s,traj.Tf);
        end
        if ~logical(a_star_info.any_feasible) || ...
                logical(a_star_info.no_feasible)
            error('spinn3d_alpha_plan_refine:NoFeasibleAlpha', ...
                ['No feasible alpha exists at t=%.9g s, path=%.9g/%.9g.  ', ...
                 'Rejecting this CEM candidate instead of using alpha_min.'], ...
                t,s,traj.Tf);
        end
        if ~isnumeric(a_star) || ~isreal(a_star) || ...
                ~isscalar(a_star) || ~isfinite(double(a_star))
            error('spinn3d_alpha_plan_refine:InvalidFeasibleAlpha', ...
                ['alpha_feasible returned an invalid command despite ', ...
                 'feasible metadata.']);
        end
        a_star = clamp(double(a_star), alpha_floor, alpha_max);
        % alpha_feasible returns a post-governor alpha, whereas the CEM
        % knots are desired-alpha commands.  Convert the cap to the same
        % command domain before taking the minimum; comparing these two
        % different quantities directly is a discrete-time type error.
        a_cap_des = alpha_des_teacher(a_star,a_prev,areg.BETA, ...
            areg.A_DOT_UP,areg.A_DOT_DN,Ts,alpha_max);
        a_target = min(a_des,a_cap_des);
        [a_post, ~, ~] = governor_step(a_target, a_prev, areg.BETA, areg.A_DOT_UP, areg.A_DOT_DN, Ts);
        a_post = clamp(double(a_post), alpha_floor, alpha_max);
        a_use  = a_post;
        adot   = (a_use - a_prev) / max(Ts, 1e-12);

        dq_cmd  = a_use * dq_ref;
        ddq_cmd = a_use*a_use * ddq_ref + adot * dq_ref;
        tau = row(inverseDynamics(robot, q_ref, dq_cmd, ddq_cmd));
        if any(fric.B) || any(fric.Fc)
            tau = tau + row(fric.B).*dq_cmd + row(fric.Fc).*tanh(dq_cmd./max(fric.vel_eps,1e-9));
        end

        dq_start=a_prev*dq_ref;
        dq_pwr=dq_start+0.5*Ts*ddq_cmd;
        dq_end=dq_start+Ts*ddq_cmd;
        bus_step=spinn3d_bus_with_state(bus,st_bus,Ts);
        if use_state
            [flow, st_next] = spinn3d_bus_step(tau,dq_pwr,bus_step,st_bus,Ts);
        else
            bus_step.enable_state=false;
            [flow, st_next] = spinn3d_bus_step(tau,dq_pwr,bus_step,[],Ts);
        end
        if strcmp(char(bus.model_scope),'dynamic_capacitor_common_dc_v1')
            interval=spinn3d_motor_interval_audit(tau,dq_start,dq_end,bus_step);
            if ~flow.state_pass || ~flow.motor_envelope_ok || ...
                    ~interval.dynamic_state_pass || ~interval.envelope_ok
                error('spinn3d_alpha_plan_refine:NoFeasibleAlpha', ...
                    'Reject CEM candidate: dynamic BUS/motor admission failed at t=%.9g.',t);
            end
        end
        if use_state, st_bus=st_next; end

        k = k + 1;

        Pgrid_k = flow.Pgrid;
        Pdump_k = flow.Pdump;
        Pspill_k = getfield_def(flow,'Pspill',0);
        Preuse_total_k = getfield_def(flow,'Preuse_total',0);
        headroom_k = max(a_star-a_use,0);
        caploss_k = max(a_post-a_star,0);

        Egrid = Egrid + Ts * Pgrid_k;
        Edump = Edump + Ts * Pdump_k;
        Ereuse_total = Ereuse_total + Ts * Preuse_total_k;
        Espill = Espill + Ts * Pspill_k;
        peakPgrid = max(peakPgrid, Pgrid_k);
        peakPdump = max(peakPdump, Pdump_k);
        peakPspill = max(peakPspill, Pspill_k);
        Ebuf_min = min(Ebuf_min, st_next.Ebuf);
        Tbrk_peak = max(Tbrk_peak, st_next.Tbrk);

        util_k = min(max(Pgrid_k / pcap_safe, 0), 1.0);
        util_sum = util_sum + util_k;
        util_peak = max(util_peak, util_k);
        over_budget_acc = over_budget_acc + max(Pgrid_k / pcap_safe - 1.0, 0).^2;

        bus_avg = bus;
        bus_avg.Pgrid_avg_prev = Pgrid_avg_lp;
        [Pgrid_avg_lp, ~, Pavg_cap, ~] = spinn3d_bus_avg_guard(Pgrid_k, Pcap, bus_avg, Ts);
        if isfinite(Pavg_cap) && Pavg_cap > 0
            pavg_norm = Pgrid_avg_lp / max(Pavg_cap, 1e-12);
        else
            pavg_norm = 0;
        end
        pavg_util_sum  = pavg_util_sum + min(max(pavg_norm,0),1.0);
        pavg_under_acc = pavg_under_acc + max(1.0 - pavg_norm, 0)^2 * Ts;
        pavg_over_acc  = pavg_over_acc  + max(pavg_norm - 1.0, 0)^2 * Ts;
        pavg_peak = max(pavg_peak, Pgrid_avg_lp);

        pgrid_norm = Pgrid_k / pcap_safe;
        pgrid_norm_sum = pgrid_norm_sum + pgrid_norm;
        pgrid_norm_sq_sum = pgrid_norm_sq_sum + pgrid_norm^2;

        if isnan(prev_pgrid)
            dpdt_norm = 0;
        else
            dpdt_norm = ((Pgrid_k - prev_pgrid) / max(Ts,1e-12)) / pcap_safe;
        end
        dpdt_norm_sq_sum = dpdt_norm_sq_sum + dpdt_norm^2;
        prev_pgrid = Pgrid_k;

        if isempty(ddq_prev)
            jerk_k = 0;
        else
            jerk_k = norm((ddq_cmd - ddq_prev) / max(Ts,1e-12), 2);
        end
        jerk_sq_sum = jerk_sq_sum + jerk_k^2;
        ddq_prev = ddq_cmd;

        headroom_sq_sum = headroom_sq_sum + headroom_k^2;
        cap_sq_sum = cap_sq_sum + caploss_k^2;

        if keep_trace
            Pgrid(k) = Pgrid_k;
            Pdump(k) = Pdump_k;
            Pspill(k) = Pspill_k;
            Preuse_total(k) = Preuse_total_k;
            Jerk(k) = jerk_k;
            Ause(k) = a_use;
            Astar(k) = a_star;
            Headroom(k) = headroom_k;
            Rho(k) = rho;
            CapLoss(k) = caploss_k;
            Ebuf(k) = st_next.Ebuf;
            Tbrk(k) = st_next.Tbrk;
            Ereuse(k) = st_next.Ereuse;
        end

        a_prev = a_use;
        s = min(traj.Tf, s + Ts*a_use);
        t = t + Ts;
        if (Ts*a_use) <= 0
            break;
        end
        if s >= traj.Tf - 1e-12
            reached = true;
        end
    end

    k_eff = max(k, 1);
    util_mean = util_sum / k_eff;
    pgrid_var_term = max(pgrid_norm_sq_sum / k_eff - (pgrid_norm_sum / k_eff)^2, 0);
    dpdt_term = sqrt(dpdt_norm_sq_sum / k_eff);
    jerk_rms = sqrt(jerk_sq_sum / k_eff);
    jerk_norm = jerk_rms / max(mean(limits.qdd_max(:))/max(Ts,1e-12), 1e-6);
    cap_term = cap_sq_sum / k_eff;
    headroom_term = headroom_sq_sum / k_eff;
    % The over-budget term is based on filtered Pgrid.
    % Raw Pgrid is allowed above P_TOTAL_MAX but limited by P_grid_peak_max.
    over_budget_term = pavg_over_acc / max(t, Ts);

    metrics_raw = struct();
    metrics_raw.t_reach      = t;
    metrics_raw.t_reach_noguard = fair.t_reach_noguard_s;
    metrics_raw.time_deadline_s = fair.deadline_s;
    metrics_raw.delay_vs_noguard_s = t-fair.t_reach_noguard_s;
    metrics_raw.Egrid        = Egrid;
    metrics_raw.Edump        = Edump;
    metrics_raw.Ereuse_total = Ereuse_total;
    metrics_raw.reuse_ratio  = Ereuse_total / max(Ereuse_total + Edump, eps);
    metrics_raw.Espill       = Espill;
    metrics_raw.peakPgrid    = peakPgrid;
    metrics_raw.peakPdump    = peakPdump;
    metrics_raw.peakPspill   = peakPspill;
    metrics_raw.Ebuf_min     = Ebuf_min;
    metrics_raw.Tbrk_peak    = Tbrk_peak;
    metrics_raw.over_budget_term = over_budget_term;
    metrics_raw.util_mean    = util_mean;
    metrics_raw.util_peak    = util_peak;
    metrics_raw.pavg_util_mean  = pavg_util_sum / k_eff;
    metrics_raw.pavg_under_term = pavg_under_acc / max(t, Ts);
    metrics_raw.pavg_over_term  = pavg_over_acc  / max(t, Ts);
    metrics_raw.Pgrid_avg_peak  = pavg_peak;
    metrics_raw.Pgrid_avg_final = Pgrid_avg_lp;
    metrics_raw.pgrid_var_term = pgrid_var_term;
    metrics_raw.dpdt_term      = dpdt_term;
    metrics_raw.jerk_term      = jerk_norm^2;
    metrics_raw.cap_loss_term  = cap_term;
    metrics_raw.headroom_term  = headroom_term;
    metrics_raw.end_error      = max(0, 1 - min(1.0, s / max(traj.Tf,1e-12)))^2;
    metrics_raw.reached        = reached || (s >= traj.Tf - 1e-12);

    deadline_factor = fair.deadline_s / max(Tref,Ts);
    score_cfg = struct('Pcap',Pcap, 'Tref',max(Tref, Ts), ...
                       'bus',bus, 'use_state',use_state, 'max_time_factor',deadline_factor, ...
                       'buf_floor_frac',objective_cfg.buf_floor_frac, ...
                       'incomplete_penalty',objective_cfg.incomplete_penalty);
    [J, score_terms] = spinn3d_bus_aware_score(metrics_raw, W, score_cfg);
    % A deadline miss is a strong but finite search penalty.  The surrogate
    % is not the authoritative full-plant timing measurement.  Invalid
    % numerical values still remain non-finite and are rejected by CEM.
    late_s = max(0,t-fair.deadline_s);
    late_norm = late_s/max(Tref,Ts);
    deadline_penalty = objective_cfg.deadline_penalty * ...
        (late_norm + late_norm^2);
    floor_infeasible_fraction = floor_infeasible_count/max(k_eff,1);
    J = J + deadline_penalty + ...
        objective_cfg.floor_infeasible_penalty*floor_infeasible_fraction;
    if ~isfinite(J) || ~isfinite(metrics_raw.end_error)
        J = inf;
    end
    score_terms.deadline_late_s = late_s;
    score_terms.deadline_penalty = deadline_penalty;
    score_terms.proxy_time_contract_pass = logical(metrics_raw.reached && late_s <= 1e-9);
    score_terms.floor_infeasible_fraction = floor_infeasible_fraction;
    score_terms.objective = J;

    info = struct('cycle_time',t,'grid_peak',peakPgrid,'dump_peak',peakPdump,'spill_peak',peakPspill, ...
                  'dump_energy',Edump,'spill_energy',Espill,'Egrid_total',Egrid, ...
                  'Ereuse_total',Ereuse_total, ...
                  'reuse_ratio',metrics_raw.reuse_ratio, ...
                  'over_budget_term',over_budget_term,'util_mean',util_mean,'util_peak',util_peak, ...
                  'pavg_util_mean',metrics_raw.pavg_util_mean,'pavg_under_term',metrics_raw.pavg_under_term,'pavg_over_term',metrics_raw.pavg_over_term, ...
                  'Pgrid_avg_peak',metrics_raw.Pgrid_avg_peak,'Pgrid_avg_final',metrics_raw.Pgrid_avg_final, ...
                  'Pgrid_avg0',getfield_def(plan0,'Pgrid_avg0',NaN), ...
                  'Pgrid_static_init_W',Pgrid_static_init_W, ...
                  'power_init',power_init, ...
                  'bus_state0',st_bus0,'bus_state_final',st_bus, ...
                  't_reach_noguard',fair.t_reach_noguard_s,'time_deadline_s',fair.deadline_s, ...
                  'delay_vs_noguard_s',t-fair.t_reach_noguard_s, ...
                  'reached',logical(metrics_raw.reached), ...
                  'proxy_time_contract_pass',logical(metrics_raw.reached && late_s <= 1e-9), ...
                  'deadline_late_s',late_s, ...
                  'deadline_penalty',deadline_penalty, ...
                  'floor_infeasible_count',floor_infeasible_count, ...
                  'floor_infeasible_fraction',floor_infeasible_fraction, ...
                  'score_terms',score_terms,'objective',J);

    if ~keep_trace
        return;
    end

    Pgrid = Pgrid(1:k_eff); Pdump = Pdump(1:k_eff); Pspill = Pspill(1:k_eff); Preuse_total = Preuse_total(1:k_eff);
    Jerk = Jerk(1:k_eff); Ause = Ause(1:k_eff); Astar = Astar(1:k_eff); Headroom = Headroom(1:k_eff); Rho = Rho(1:k_eff); CapLoss = CapLoss(1:k_eff); Ebuf = Ebuf(1:k_eff); Tbrk = Tbrk(1:k_eff); Ereuse = Ereuse(1:k_eff);
    dP = [0; diff(Pgrid)] / max(Ts,1e-12);

    info.pgrid_var = var(Pgrid,1);
    info.dpdt_rms = rms_safe(dP);
    info.jerk_rms = rms_safe(Jerk);
    info.headroom_rms = rms_safe(Headroom);
    info.a_mean = mean(Ause);
    info.a_min = min(Ause);
    info.rho_end = Rho(end);
    info.Ebuf_min = min(Ebuf);
    info.Tbrk_peak = max(Tbrk);
    info.Pgrid = Pgrid;
    info.Pdump = Pdump;
    info.Pspill = Pspill;
    info.Preuse_total = Preuse_total;
    info.Ause = Ause;
    info.Astar = Astar;
    info.Headroom = Headroom;
    info.Rho = Rho;
    info.Jerk = Jerk;
    info.CapLoss = CapLoss;
    info.Ebuf = Ebuf;
    info.Ereuse = Ereuse;
    info.Tbrk = Tbrk;
end

function [J,status,info] = rollout_cost_safe_local(varargin)
% Only the exact, physical no-alpha result is a sampled-candidate rejection.
% Configuration/programming exceptions are deliberately rethrown.
status = 0;
info = struct();
try
    [J,info] = rollout_cost(varargin{:});
catch ME
    if ~strcmp(char(ME.identifier), ...
            'spinn3d_alpha_plan_refine:NoFeasibleAlpha')
        rethrow(ME);
    end
    J = inf;
    status = 1;
    info = struct('candidate_rejected',true, ...
        'rejection_id',char(ME.identifier));
    return;
end
if ~isfinite(J)
    status = 2;
end
end

function parcfg = prepare_parallel_cfg(plan_cfg, opts, pop)
parcfg = struct();
parcfg.enable_mode = getfield_def(opts,'parallel_enable', ...
    plan_cfg.parallel_enable);
parcfg.autostart = logical(getfield_def(opts,'parallel_autostart', ...
    plan_cfg.parallel_autostart));
parcfg.workers = getfield_def(opts,'parallel_workers', ...
    plan_cfg.parallel_workers);
parcfg.profile = getfield_def(opts,'parallel_profile', ...
    plan_cfg.parallel_profile);
parcfg.idle_timeout_min = getfield_def(opts,'parallel_idle_timeout_min', ...
    plan_cfg.parallel_idle_timeout_min);
parcfg.min_pop = max(1,round(getfield_def(opts,'parallel_min_pop', ...
    plan_cfg.parallel_min_pop)));
parcfg.requested   = parse_parallel_mode(parcfg.enable_mode);
parcfg.use         = false;
parcfg.pool        = [];
parcfg.actual_workers = 0;
if ~parcfg.requested
    parcfg.reason = 'disabled';
elseif pop < parcfg.min_pop
    parcfg.reason = 'below_min_pop';
else
    parcfg.reason = 'requested';
end
end

function parcfg = ensure_parallel_pool(parcfg)
if ~parcfg.requested || strcmp(parcfg.reason, 'below_min_pop')
    return;
end

pool = gcp('nocreate');
if parcfg.autostart
    % The formal helper replaces only an inspectably idle wrong-size pool;

    pool = spinn3d_prepare_formal_pool(parcfg.profile,parcfg.workers, ...
        parcfg.idle_timeout_min,'alpha-refine');
elseif ~isempty(pool) && ~isempty(parcfg.workers) && ...
        double(pool.NumWorkers) ~= double(parcfg.workers)
    error('spinn3d:AlphaRefineParallelPoolWorkerMismatch', ...
        ['[alpha-refine] the active pool has %d workers but the signed ', ...
         'configuration requires %d. Wrong-size-pool fallback is forbidden.'], ...
        double(pool.NumWorkers),double(parcfg.workers));
end

parcfg.pool = pool;
parcfg.use  = ~isempty(pool);
if parcfg.use
    parcfg.actual_workers = double(pool.NumWorkers);
    if ~isempty(parcfg.workers) && ...
            parcfg.actual_workers ~= double(parcfg.workers)
        error('spinn3d:AlphaRefineParallelPoolWorkerMismatch', ...
            ['[alpha-refine] requested %d workers, but the active pool ', ...
             'reports %d.'],double(parcfg.workers), ...
             parcfg.actual_workers);
    end
    parcfg.reason = 'parallel';
elseif parcfg.autostart
    error('spinn3d:AlphaRefineParallelPoolUnavailable', ...
        ['[alpha-refine] parallel autostart returned no pool. Serial ', ...
         'fallback is forbidden.']);
else
    parcfg.reason = 'no_pool';
    warning_once('warmstart_parallel_nopool', '[alpha-refine] parallel requested but no pool is active; running serial. Start parpool(...) or set cfg.PLAN.parallel_autostart = true.');
end
end

function tf = want_parallel_eval(parcfg)
tf = isstruct(parcfg) && isfield(parcfg, 'use') && logical(parcfg.use);
end

function tf = parse_parallel_mode(mode)
if ischar(mode) || isstring(mode)
    mode = lower(strtrim(char(mode)));
    switch mode
        case {'1','on','true','yes','y','auto','parallel','enabled'}
            tf = true;
        case {'0','off','false','no','n','serial','disabled'}
            tf = false;
        otherwise
            tf = true;
    end
else
    tf = logical(mode);
end
end

function plan = compact_plan0_for_eval(plan0)
plan = struct();
if ~isstruct(plan0) || isempty(plan0)
    return;
end
if isfield(plan0,'Tref') && ~isempty(plan0.Tref)
    plan.Tref = plan0.Tref;
end
copy_fields = {'Pgrid_avg0','Pgrid_static_init_W','power_init'};
for i = 1:numel(copy_fields)
    fn = copy_fields{i};
    if isfield(plan0,fn) && ~isempty(plan0.(fn))
        plan.(fn) = plan0.(fn);
    elseif isfield(plan0,'plan') && isstruct(plan0.plan) && ...
            isfield(plan0.plan,fn) && ~isempty(plan0.plan.(fn))
        plan.(fn) = plan0.plan.(fn);
    end
end
if isfield(plan0,'bus_state0') && isstruct(plan0.bus_state0) && ~isempty(plan0.bus_state0)
    plan.bus_state0 = plan0.bus_state0;
elseif isfield(plan0,'plan') && isstruct(plan0.plan) && isfield(plan0.plan,'bus_state0') && isstruct(plan0.plan.bus_state0) && ~isempty(plan0.plan.bus_state0)
    plan.bus_state0 = plan0.plan.bus_state0;
end
end

function plan0 = ensure_static_power_init_local(plan0,robot,traj,cfg,bus,Pcap)
if ~isstruct(plan0), plan0 = struct(); end
candidate = getfield_def(plan0,'Pgrid_avg0',NaN);
if (~isnumeric(candidate) || ~isscalar(candidate) || ...
        ~isfinite(double(candidate)) || candidate < 0) && ...
        isfield(plan0,'plan') && isstruct(plan0.plan)
    candidate = getfield_def(plan0.plan,'Pgrid_avg0',NaN);
end
[q0,~,~] = sample_traj_eval(traj,0);
init = spinn3d_static_hold_power_init( ...
    robot,row(q0),cfg.FRIC,bus,Pcap,cfg.TS);
if isnumeric(candidate) && isscalar(candidate) && ...
        isfinite(double(candidate)) && candidate >= 0
    % Preserve an explicit carried/paired filter state, but always retain

    plan0.Pgrid_avg0 = double(candidate);
else
    plan0.Pgrid_avg0 = init.Pgrid_avg0_W;
end
plan0.Pgrid_static_init_W = init.Pgrid_static_W;
plan0.power_init = init;
if init.bus_state_enabled
    plan0.bus_state0 = init.bus_state0;
end
end

function v=expand_limit_refine_local(v,n)
v=double(v(:).');
if isempty(v),v=inf;end
if isscalar(v)
    v=repmat(v,1,n);
elseif numel(v)<n
    v=[v,repmat(v(end),1,n-numel(v))];
else
    v=v(1:n);
end
end

function traj2 = compact_traj_for_eval(traj)
traj2 = struct();
if ~isstruct(traj) || isempty(traj)
    return;
end
if isfield(traj,'Tf') && ~isempty(traj.Tf)
    traj2.Tf = traj.Tf;
end
if isfield(traj,'Q') && isfield(traj,'dQ') && isfield(traj,'ddQ') && ...
        ~isempty(traj.Q) && ~isempty(traj.dQ) && ~isempty(traj.ddQ)
    traj2.Q = traj.Q;
    traj2.dQ = traj.dQ;
    traj2.ddQ = traj.ddQ;
    traj2.Ts = max(getfield_def(traj2,'Tf',0) / max(size(traj.Q,1)-1,1), eps);
elseif isfield(traj,'eval') && ~isempty(traj.eval)
    traj2.eval = traj.eval;
end
end

function [q,dq,ddq] = sample_traj_eval(traj, s)
if isstruct(traj) && isfield(traj,'Q') && isfield(traj,'dQ') && isfield(traj,'ddQ') && isfield(traj,'Ts') && ...
        ~isempty(traj.Q) && ~isempty(traj.dQ) && ~isempty(traj.ddQ)
    M = size(traj.Q,1);
    if M <= 1
        q = traj.Q(1,:);
        dq = traj.dQ(1,:);
        ddq = traj.ddQ(1,:);
        return;
    end
    u = max(0, min((M-1), s / max(traj.Ts,1e-12)));
    i = floor(u) + 1;
    i = max(1, min(M-1, i));
    lam = u - (i-1);
    q   = (1-lam)*traj.Q(i,:)   + lam*traj.Q(i+1,:);
    dq  = (1-lam)*traj.dQ(i,:)  + lam*traj.dQ(i+1,:);
    ddq = (1-lam)*traj.ddQ(i,:) + lam*traj.ddQ(i+1,:);
else
    [q,dq,ddq] = traj.eval(s);
end
end

function warning_once(tag, msg, varargin)
persistent seen_tags
if isempty(seen_tags)
    seen_tags = {};
end
if any(strcmp(seen_tags, tag))
    return;
end
seen_tags{end+1} = tag;
warning(msg, varargin{:});
end

function [rho_knots, a_knots] = canonicalize_knots( ...
        plan0, Nknot, alpha_floor, alpha_max)
    if isfield(plan0,'rho_knots') && isfield(plan0,'a_knots')
        rho_knots = row(plan0.rho_knots); a_knots = row(plan0.a_knots);
    elseif isfield(plan0,'rho') && isfield(plan0,'a_des_seq')
        rho_knots = linspace(0,1,Nknot);
        a_knots   = interp1(row(plan0.rho), row(plan0.a_des_seq), rho_knots, 'linear', 'extrap');
    else
        error('plan0 must contain rho_knots/a_knots or rho/a_des_seq.');
    end
    if numel(rho_knots) ~= numel(a_knots), error('rho_knots and a_knots length mismatch.'); end
    if numel(rho_knots) < 2, rho_knots = [0 1]; a_knots = [a_knots(1) a_knots(1)]; end
    rho_knots = min(1.0, max(0.0, rho_knots));
    [rho_knots, iu] = unique(rho_knots, 'stable'); a_knots = a_knots(iu);
    if rho_knots(1) > 0, rho_knots = [0, rho_knots]; a_knots = [a_knots(1), a_knots]; end
    if rho_knots(end) < 1, rho_knots = [rho_knots, 1]; a_knots = [a_knots, a_knots(end)]; end
    a_knots = clamp(a_knots, alpha_floor, alpha_max);
end

function [seed, ok] = build_feasible_seed_from_plan0( ...
        plan0,rho_knots,alpha_floor,alpha_max)
seed = [];
ok = false;

if ~(isstruct(plan0) && isfield(plan0,'rho') && ~isempty(plan0.rho) && ...
        isfield(plan0,'a_des_seq') && ~isempty(plan0.a_des_seq))
    return;
end

rho0 = row(plan0.rho);
a_des0 = row(plan0.a_des_seq);
if numel(rho0) ~= numel(a_des0)
    return;
end

% The immutable warm-start trace already stores the held desired-alpha
% command.  Use it directly.  The former "feasible blend" mixed a_des with
% a_star/a_use post-governor states and biased the seed upward; without a
% per-sample a_prev/governor contract that conversion is undefined.
if any(~isfinite([rho0,a_des0]))
    return;
end

a_seed0 = a_des0;
a_seed0 = clamp(a_seed0, alpha_floor, alpha_max);

seed = interp1(rho0, a_seed0, rho_knots, 'linear', 'extrap');
seed = clamp(seed, alpha_floor, alpha_max);
ok = all(isfinite(seed)) && ~isempty(seed);
end

function st = resolve_bus_state0(plan0, bus)
    cand = [];
    if isstruct(plan0)
        if isfield(plan0,'bus_state0') && isstruct(plan0.bus_state0) && ~isempty(plan0.bus_state0)
            cand = plan0.bus_state0;
        elseif isfield(plan0,'plan') && isstruct(plan0.plan) && isfield(plan0.plan,'bus_state0') && ~isempty(plan0.plan.bus_state0)
            cand = plan0.plan.bus_state0;
        end
    end
    st=spinn3d_alpha_bus_state(bus,cand);
end

function policy_dt = get_policy_dt_local(cfg, Ts)

policy_dt = double(cfg.POLICY.dt);
ratio = policy_dt/double(Ts);

end

function validate_objective_cfg_local(O)
required = {'buf_floor_frac','incomplete_penalty', ...
    'deadline_penalty','floor_infeasible_penalty'};

vals = [O.buf_floor_frac O.incomplete_penalty ...
        O.deadline_penalty O.floor_infeasible_penalty];

end

function value = require_binary_scalar_local(S,field,label)

value = logical(S.(field));
end

function S = merge_struct(S, T), fn = fieldnames(T); for i = 1:numel(fn), S.(fn{i}) = T.(fn{i}); end, end
function y = rms_safe(x), x = double(x(:)); if isempty(x), y = 0; else, y = sqrt(mean(x.^2)); end, end
function req(S, keys), for i=1:numel(keys),  end, end
function x = getfield_def(S,fn,def), if isstruct(S) && isfield(S,fn) && ~isempty(S.(fn)), x = S.(fn); else, x = def; end, end
function r = row(v), r = v(:)'; end
function y = clamp(x, lo, hi), y = min(hi, max(lo, x)); end
function tf = valid_binary_field_local(S,field)
v = [];
if isstruct(S) && isfield(S,field)
    v = S.(field);
end
tf = (islogical(v) || isnumeric(v)) && isscalar(v) && ...
     isreal(v) && isfinite(double(v)) && any(double(v)==[0 1]);
end
