function [a_star, info] = alpha_feasible(q0, dq0, ddq0, a_init, robot, caps, Pbud, amin, itmax, fric) %#ok<INUSD>
% Select a progress-rate command using robot and electrical constraints.
% Candidate costs combine filtered source-power excess and progress delay.

    if nargin ~= 10
        error('alpha_feasible:MissingCentralInputs', ...
            ['Expected ten explicit inputs including Pbud, amin, itmax and ', ...
             'fric; private defaults are forbidden.']);
    end

    q0   = row(q0);
    dq0  = row(dq0);
    ddq0 = row(ddq0);

    nJ = numel(q0);

    
    search = require_alpha_feasible_search(caps);

    tau_lim = ensure_row_len(caps.tau_max,    nJ);
    qd_lim  = ensure_row_len(caps.qd_max,     nJ);
    % Default alpha_feasible is a path-domain feasible cap.
    % The actual alpha-dot/qdd_cmd check is only enabled by explicit opt-in.
    % This prevents high-level teacher / policy decisions from being silently
    % constrained as if alpha jumped within one 1-ms plant step.
    qdd_src = caps.qdd_max;
    if logical(getfield_def(caps,'use_alpha_dot',false)) && ...
       isfield(caps,'qdd_cmd_max_actual') && ~isempty(caps.qdd_cmd_max_actual)
        qdd_src = caps.qdd_cmd_max_actual;
    end
    qdd_lim = ensure_row_len(qdd_src, nJ);
    Pax_lim = ensure_row_len(caps.P_axis_max, nJ);

    Pbud = double(Pbud);

    amin = double(amin);

    itmax = double(itmax);

    [bus, use_state, st0, Ts_bus, Pcap, P_brk_peak, a_prev, Ts_alpha, use_alpha_dot] = setup_bus_mode(caps, Pbud);
    pavg_soft_target_select = getfield_def(caps, ...
        'pavg_soft_target_select', false);

    pavg_soft_target_select = logical(pavg_soft_target_select);

    % ==== non-monotone feasible-island search ====
    % A single [feasible, infeasible] bisection silently loses feasible
    % islands whenever alpha_dot changes the sign/magnitude of acceleration.
    % First identify all islands on a deterministic coarse grid, enriched
    % around alpha_prev, then refine the upper boundary of the highest island.
    lo = amin;
    hi = search.alpha_max;
    n_bis = itmax;
    n_grid = max(search.grid_min_points, ...
        min(search.grid_max_points, ...
            search.grid_points_per_itmax*n_bis + ...
            search.grid_point_offset));
    a_grid = linspace(lo, hi, n_grid);
    if use_alpha_dot
        da = max((hi-lo)/(n_grid-1), search.around_prev_min_step);
        around_prev = a_prev + da * ...
            row(search.around_prev_step_multipliers);
        % Geometric offsets catch the very narrow restart island that can
        % occur at a 1-ms servo period.
        geom_prev = a_prev + ...
            reshape(10.^row(search.around_prev_geom_exp10),[],1) .* ...
            reshape(row(search.around_prev_geom_signs),1,[]);
        around_prev = [around_prev, geom_prev(:).'];
        a_grid = [a_grid, around_prev(:).'];
    end
    a_grid = unique(min(hi,max(lo,double(a_grid))), 'sorted');

    ok_grid = false(size(a_grid));
    info_grid = cell(size(a_grid));
    for ig = 1:numel(a_grid)
        [ok_grid(ig), info_grid{ig}] = eval_candidate(a_grid(ig), ...
            q0, dq0, ddq0, robot, fric, tau_lim, qd_lim, qdd_lim, Pax_lim, ...
            bus, use_state, st0, Ts_bus, Pcap, P_brk_peak, ...
            a_prev, Ts_alpha, use_alpha_dot, search);
    end

    floor_infeasible = ~ok_grid(1);
    % FIND preserves the orientation of a vector input in MATLAB. Force
    % the feasibility mask to a column so the edge masks below always have
    % matching dimensions, regardless of how a_grid/ok_grid was created.
    feasible_idx = find(ok_grid(:));
    if isempty(feasible_idx)
        a_star = lo;
        if nargout > 1
            info = info_grid{1};
            info.a_star = a_star;
            info.floor_infeasible = floor_infeasible;
            info.no_feasible = true;
            info.any_feasible = false;
            info.feasible_island_count = 0;
            info.search_mode = 'coarse_grid_no_island';
            info.selection_mode = 'physical_no_island';
            info.pavg_soft_target_select = pavg_soft_target_select;

            % a feasible command and must never be used as a supervised
            % teacher label.  Dataset generators key off this stable ID and
            % reject the complete candidate trajectory.
            info.failure_identifier = ...
                'spinn3d:AlphaTeacherNoFeasibleCandidate';
            info.coarse_grid_n = numel(a_grid);
            info.search_contract = search;
        end
        return;
    end

    gap_after = diff(feasible_idx) > 1;
    island_starts = feasible_idx([true; gap_after]);
    island_ends   = feasible_idx([gap_after; true]);

    % Optional teacher-label policy: Pavg is a soft target, never a physical
    % feasibility gate. Prefer the smallest normalized-square target excess
    % among all physically feasible coarse-grid candidates. If several
    % candidates have the same cost (including the common zero-excess case),
    % retain research throughput by choosing the largest alpha.
    if pavg_soft_target_select
        pavg_excess_sq_norm_grid = inf(size(a_grid));
        for ii = reshape(feasible_idx,1,[])
            excess_W = double( ...
                getfield_def(info_grid{ii},'Pavg_excess_W',inf));
            target_W = double( ...
                getfield_def(info_grid{ii},'Pavg_cap',NaN));
            if isfinite(excess_W) && isinf(target_W) && target_W>0
                pavg_excess_sq_norm_grid(ii) = 0.0;
            elseif isfinite(excess_W) && isfinite(target_W) && target_W>0
                pavg_excess_sq_norm_grid(ii) = ...
                    (max(excess_W,0)/target_W)^2;
            elseif isfinite(excess_W) && target_W==0 && excess_W<=0
                pavg_excess_sq_norm_grid(ii) = 0.0;
            end
        end
        best_pavg_excess_sq_norm = min( ...
            pavg_excess_sq_norm_grid(feasible_idx));
        tie_tol_sq_norm = max(1e-15, ...
            16*eps(max(1.0,abs(best_pavg_excess_sq_norm))));
        tied_idx = feasible_idx( ...
            pavg_excess_sq_norm_grid(feasible_idx) <= ...
            best_pavg_excess_sq_norm + tie_tol_sq_norm);
        selected_idx = tied_idx(end);
        a_star = a_grid(selected_idx);
        if nargout > 1
            info = info_grid{selected_idx};
            info.a_star = a_star;
            info.floor_infeasible = floor_infeasible;
            info.no_feasible = false;
            info.any_feasible = true;
            info.feasible_island_count = numel(island_starts);
            info.search_mode = 'coarse_grid_pavg_soft_target';
            info.selection_mode = search.soft_selection_schema;
            info.pavg_soft_target_select = true;
            info.pavg_soft_best_excess_sq_normalized = ...
                best_pavg_excess_sq_norm;
            info.pavg_soft_tie_tolerance_sq_normalized = ...
                tie_tol_sq_norm;
            info.pavg_soft_selected_excess_W = double( ...
                getfield_def(info_grid{selected_idx},'Pavg_excess_W',NaN));
            info.coarse_grid_n = numel(a_grid);
            info.search_contract = search;
        end
        return;
    end

    highest_end = island_ends(end);
    a_star = a_grid(highest_end);

    % Refine only the upper edge of the globally highest feasible island.
    % The next coarse point is known infeasible, so bisection is local to
    % one identified island and does not rely on global monotonicity.
    if highest_end < numel(a_grid)
        a_ok = a_grid(highest_end);
        a_bad = a_grid(highest_end+1);
        for it = 1:n_bis
            amid = 0.5*(a_ok+a_bad);
            [ok_mid, ~] = eval_candidate(amid, q0, dq0, ddq0, robot, fric, ...
                tau_lim, qd_lim, qdd_lim, Pax_lim, bus, use_state, st0, ...
                Ts_bus, Pcap, P_brk_peak, a_prev, Ts_alpha, use_alpha_dot, ...
                search);
            if ok_mid
                a_ok = amid;
            else
                a_bad = amid;
            end
            if (a_bad-a_ok) < search.bisection_tolerance
                break;
            end
        end
        a_star = a_ok;
    end

    if nargout > 1
        [~, info] = eval_candidate(a_star, q0, dq0, ddq0, robot, fric, ...
            tau_lim, qd_lim, qdd_lim, Pax_lim, bus, use_state, st0, ...
            Ts_bus, Pcap, P_brk_peak, a_prev, Ts_alpha, use_alpha_dot, ...
            search);
        info.a_star = a_star;
        info.floor_infeasible = floor_infeasible;
        info.no_feasible = false;
        info.any_feasible = true;
        info.feasible_island_count = numel(island_starts);
        info.search_mode = 'coarse_grid_islands_local_refine';
        info.selection_mode = 'highest_physical_alpha';
        info.pavg_soft_target_select = false;
        info.coarse_grid_n = numel(a_grid);
        info.search_contract = search;
    end
end

% ========================= candidate evaluation =========================
function [ok, info] = eval_candidate(a, q0, dq0, ddq0, robot, fric, ...
                                     tau_lim, qd_lim, qdd_lim, Pax_lim, ...
                                     bus, use_state, st0, Ts_bus, Pcap, P_brk_peak, ...
                                     a_prev, Ts_alpha, use_alpha_dot, search)

    tol = search.feasibility_tolerance;

    dq = a * dq0;
    if use_alpha_dot
        adot = (a - a_prev) / ...
            max(Ts_alpha, search.alpha_dot_time_epsilon_s);
        dd = (a * a) * ddq0 + adot * dq0;
    else
        adot = 0.0;
        dd = (a * a) * ddq0;
    end

    tau = row(inverseDynamics(robot, q0, dq, dd));
    tau = tau + row(fric.B) .* dq + row(fric.Fc) .* tanh(dq ./ max(fric.vel_eps, 1e-9));

    if use_state
        bus=spinn3d_bus_with_state(bus,st0,Ts_bus);
        [flow, st_next] = spinn3d_bus_step(tau, dq, bus, st0, Ts_bus);
        Pax = max(flow.Paxis, 0);

        [Pgrid_avg_next, ok_avg_guard, Pavg_cap, Pinst_cap] = spinn3d_bus_avg_guard(flow.Pgrid, Pcap, bus, Ts_bus);
        if ~strcmp(bus.model_scope,'dynamic_capacitor_common_dc_v1')
        ok_power = (flow.Pgrid  <= Pinst_cap + tol) && ...
                   (flow.Pspill <= tol) && ...
                   (st_next.Tbrk <= bus.Tbrk_max_C + tol) && ...
                   (st_next.Ebuf >= min(max(double(getfield_def(bus,'Ebuf_min_J',0.0)),0.0), bus.Ebuf_max_J) - tol) && ...
                   (st_next.Ebuf <= bus.Ebuf_max_J + tol);
        else
            ok_power=flow.Pgrid<=Pinst_cap+tol && flow.Pspill<=tol && ...
                logical(flow.state_pass);
        end
        ok_power=ok_power && logical(getfield_def(flow,'state_pass',true));
        mode = 'state';
    else
        bus_nostate = bus;
        bus_nostate.enable_state = false;
        [flow, st_next] = spinn3d_bus_step(tau, dq, bus_nostate, [], Ts_bus);
        Pax = max(flow.Paxis, 0);
        Pgrid = flow.Pgrid;
        [Pgrid_avg_next, ok_avg_guard, Pavg_cap, Pinst_cap] = spinn3d_bus_avg_guard(Pgrid, Pcap, bus, Ts_bus);
        ok_power = (Pgrid <= Pinst_cap + tol) && ...
                   (flow.Pspill <= tol);
        mode = 'nostate_bus_step';
    end

    motor_ratio = zeros(size(tau));
    ok_motor = true;
    if strcmpi(char(getfield_def(bus,'model','legacy_mechanical')),'st3215_pwm')
        motor_bus=bus;
        if isfield(flow,'Vdc_min'), motor_bus.V_bus_V=max(flow.Vdc_min,eps); end
        mot = spinn3d_motor_power(tau,dq,motor_bus);
        motor_ratio = ensure_row_len(mot.torque_speed_ratio,numel(tau));
        ok_motor = all(motor_ratio <= 1.0 + tol);
    end

    ok =  all(abs(tau) <= tau_lim + tol) && ...
          all(abs(dq)  <= qd_lim  + tol) && ...
          all(abs(dd)  <= qdd_lim + tol) && ...
          all(Pax      <= Pax_lim + tol) && ...
          ok_motor && ...
          ok_power;

    info = struct();
    info.mode      = mode;
    info.used_state = use_state;
    info.a         = a;
    info.q         = q0;
    info.dq        = dq;
    info.ddq       = dd;
    info.a_prev    = a_prev;
    info.adot      = adot;
    info.use_alpha_dot = use_alpha_dot;
    info.tau       = tau;
    info.Pax       = Pax;
    info.motor_torque_speed_ratio = motor_ratio;
    info.ok_motor_envelope = ok_motor;
    info.bus       = bus;
    info.Pavg_cap  = Pavg_cap;
    info.Pgrid_avg_next = Pgrid_avg_next;
    info.Pavg_margin_W = Pavg_cap - Pgrid_avg_next;
    info.Pavg_excess_W = max(Pgrid_avg_next - Pavg_cap,0.0);
    info.pavg_kpi_pass = (~isfinite(Pavg_cap)) || ...
        (Pgrid_avg_next <= Pavg_cap + tol);
    info.ok_grid_avg = info.pavg_kpi_pass;
    info.ok_grid_avg_guard_legacy = logical(ok_avg_guard);
    if isinf(Pavg_cap)
        info.pavg_ratio = 0.0;
    elseif Pavg_cap > 0
        info.pavg_ratio = Pgrid_avg_next / Pavg_cap;
    elseif Pgrid_avg_next <= tol
        info.pavg_ratio = 0.0;
    else
        info.pavg_ratio = inf;
    end
    info.Pavg_is_hard_constraint = false;
    info.Pgrid_peak_cap = Pinst_cap;
    info.st0       = st0;
    info.flow      = flow;
    info.st_next   = st_next;
    info.ok        = ok;
    info.physical_ok = ok;
end

% ========================= bus mode setup =========================
function [bus, use_state, st0, Ts_bus, Pcap, P_brk_peak, a_prev, Ts_alpha, use_alpha_dot] = setup_bus_mode(caps, Pbud)
    if isfield(caps, 'bus') && isstruct(caps.bus)
        bus = caps.bus;
    else
        bus = struct();
    end
    strict_bus = isfield(bus,'strict_config') && ...
        logical(bus.strict_config);

    if strict_bus

        

        bus.Pgrid_avg_prev = double(caps.Pgrid_avg_prev);
    else
        eta_share_caps  = getfield_def(caps, 'eta_share', 1.0);
        P_brk_peak_caps = getfield_def(caps, 'P_brk_peak', inf);
        if ~isfield(bus, 'eta_share') || isempty(bus.eta_share)
            bus.eta_share = eta_share_caps;
        end
        if ~strcmp(getfield_def(bus,'model_scope',''),'dynamic_capacitor_common_dc_v1') && ...
                (~isfield(bus, 'P_brk_peak') || isempty(bus.P_brk_peak))
            bus.P_brk_peak = P_brk_peak_caps;
        end
        if ~isfield(bus, 'P_total_max') || isempty(bus.P_total_max)
            bus.P_total_max = Pbud;
        end
        if ~isfield(bus, 'P_grid_peak_max') || isempty(bus.P_grid_peak_max)
            bus.P_grid_peak_max = getfield_def(caps, ...
                'P_grid_peak_max', []);
        end
        if ~isfield(bus, 'P_avg_window_s') || isempty(bus.P_avg_window_s)
            bus.P_avg_window_s = getfield_def(caps, ...
                'P_avg_window_s', []);
        end
        if ~isfield(bus, 'P_avg_target_frac') || isempty(bus.P_avg_target_frac)
            bus.P_avg_target_frac = getfield_def(caps, ...
                'P_avg_target_frac', []);
        end
        if ~isfield(bus, 'P_avg_hard_enable') || isempty(bus.P_avg_hard_enable)
            bus.P_avg_hard_enable = getfield_def(caps, ...
                'P_avg_hard_enable', []);
        end
        bus.Pgrid_avg_prev = getfield_def(caps, ...
            'Pgrid_avg_prev',getfield_def(bus,'Pgrid_avg_prev',0.0));
    end

    Pcap = double(Pbud);
    bus.P_total_max = Pcap;
    bus = spinn3d_bus_defaults(bus);

    P_brk_peak=getfield_def(bus,'P_brk_peak',0);

    if strict_bus
        Ts_bus = double(caps.bus_Ts);
    else
        Ts_bus = getfield_def(caps, 'bus_Ts', 1e-3);
        if isempty(Ts_bus) || ~isfinite(Ts_bus) || double(Ts_bus) <= 0
            Ts_bus = 1e-3;
        else
            Ts_bus = double(Ts_bus);
        end
    end

    if isfield(bus,'model_scope') && ...
            strcmp(bus.model_scope,'dynamic_capacitor_common_dc_v1')
        if ~isfield(caps,'bus_state') && isfield(bus,'dc_state')
            caps.bus_state=bus.dc_state;
        end

    end
    use_state = bus.enable_state && ...
                isfield(caps, 'bus_state') && isstruct(caps.bus_state);

    if use_state
        st0 = sanitize_bus_state(caps.bus_state, bus);
    else
        st0 = [];
    end

    a_prev_raw = getfield_def(caps, 'alpha_prev', getfield_def(caps, 'a_prev', []));
    % alpha_dot is opt-in.  Passing alpha_prev for feature/state bookkeeping
    % must not change the teacher semantics.
    if strict_bus
        use_alpha_dot = logical(caps.use_alpha_dot);
    else
        use_alpha_dot = logical(getfield_def(caps, ...
            'use_alpha_dot', false));
    end
    a_prev_vec = double(a_prev_raw(:));
    if use_alpha_dot && strict_bus

        a_prev = a_prev_vec(1);
    elseif isempty(a_prev_vec) || ~isfinite(a_prev_vec(1))
        a_prev = 0.0;
        use_alpha_dot = false;
    else
        a_prev = a_prev_vec(1);
    end
    if use_alpha_dot && strict_bus

        Ts_alpha = double(caps.alpha_Ts);
    else
        Ts_alpha = getfield_def(caps,'alpha_Ts', ...
            getfield_def(caps,'Ts',Ts_bus));
    end
    if isempty(Ts_alpha) || ~isfinite(double(Ts_alpha)) || ...
            double(Ts_alpha) <= 0
        Ts_alpha = Ts_bus;
    else
        Ts_alpha = double(Ts_alpha);
    end
end

% Removed from the active path.  No-state candidates now use spinn3d_bus_step
% with bus.enable_state=false so Pdump means handled dump and Pspill remains

% ========================= helpers =========================
function search = require_alpha_feasible_search(caps)

    search = caps.alpha_feasible;
    fields = {'alpha_max','grid_min_points','grid_max_points', ...
        'grid_points_per_itmax','grid_point_offset', ...
        'around_prev_min_step','around_prev_step_multipliers', ...
        'around_prev_geom_signs','around_prev_geom_exp10', ...
        'bisection_tolerance','feasibility_tolerance', ...
        'alpha_dot_time_epsilon_s','soft_selection_schema'};

    search.alpha_max = finite_scalar(search.alpha_max,'alpha_max');

    search.grid_min_points = positive_integer( ...
        search.grid_min_points,'grid_min_points');
    search.grid_max_points = positive_integer( ...
        search.grid_max_points,'grid_max_points');

    search.grid_points_per_itmax = positive_integer( ...
        search.grid_points_per_itmax,'grid_points_per_itmax');
    search.grid_point_offset = nonnegative_integer( ...
        search.grid_point_offset,'grid_point_offset');
    search.around_prev_min_step = positive_scalar( ...
        search.around_prev_min_step,'around_prev_min_step');
    search.around_prev_step_multipliers = finite_vector( ...
        search.around_prev_step_multipliers, ...
        'around_prev_step_multipliers');
    search.around_prev_geom_signs = finite_vector( ...
        search.around_prev_geom_signs,'around_prev_geom_signs');

    search.around_prev_geom_exp10 = finite_vector( ...
        search.around_prev_geom_exp10,'around_prev_geom_exp10');
    search.bisection_tolerance = positive_scalar( ...
        search.bisection_tolerance,'bisection_tolerance');
    if isstring(search.soft_selection_schema) && ...
            isscalar(search.soft_selection_schema)
        search.soft_selection_schema = char(search.soft_selection_schema);
    end

    search.feasibility_tolerance = finite_scalar( ...
        search.feasibility_tolerance,'feasibility_tolerance');

    search.alpha_dot_time_epsilon_s = positive_scalar( ...
        search.alpha_dot_time_epsilon_s,'alpha_dot_time_epsilon_s');
end

function x = positive_integer(x,name)
    x = finite_scalar(x,name);

end

function x = nonnegative_integer(x,name)
    x = finite_scalar(x,name);

end

function x = positive_scalar(x,name)
    x = finite_scalar(x,name);

end

function x = finite_scalar(x,name)

    x = double(x);
end

function x = finite_vector(x,name)

    x = double(x(:).');
end

function st = sanitize_bus_state(st_in, bus)
    if isfield(bus,'model_scope') && ...
            strcmp(bus.model_scope,'dynamic_capacitor_common_dc_v1')
        % Preserve measured voltage and temperature, including violations.
        % The shared dynamic kernel audits them; clipping would hide failure.
        checked=spinn3d_bus_with_state(bus,st_in,1e-3); %#ok<NASGU>
        st=st_in;
        Ederived=0.5*bus.C_dc_F*st.Vdc^2;
        if isfield(st,'Ebuf')

        end
        st.Ebuf=Ederived;
        st.Ereuse=0; st.Eregen_task=0;
        return;
    end
    st = spinn3d_bus_state_init(bus);
    if ~isstruct(st_in), return; end

    if isfield(st_in, 'Ebuf') && ~isempty(st_in.Ebuf)
        Emin = min(max(double(getfield_def(bus,'Ebuf_min_J',0.0)), 0.0), bus.Ebuf_max_J);
        st.Ebuf = min(max(double(st_in.Ebuf), Emin), bus.Ebuf_max_J);
    end
    if isfield(st_in, 'Ereuse') && ~isempty(st_in.Ereuse)
        st.Ereuse = min(max(double(st_in.Ereuse), 0.0), st.Ebuf);
    end
    if isfield(st_in, 'Tbrk') && ~isempty(st_in.Tbrk)
        st.Tbrk = min(max(double(st_in.Tbrk), bus.Tamb_C), bus.Tbrk_max_C);
    end

    st.Ereuse = min(st.Ereuse, st.Ebuf);
end

function y = row(x)
    y = x(:)';
end

function r = ensure_row_len(v, n)
    r = v(:)';

    if isscalar(r)
        r = repmat(r, 1, n);
    end
end

function x = getfield_def(S, fn, def)
    if isstruct(S) && isfield(S, fn) && ~isempty(S.(fn))
        x = S.(fn);
    else
        x = def;
    end
end
