function [a_star, info] = alpha_feasible(q0, dq0, ddq0, a_init, robot, caps, Pbud, amin, itmax, fric) %#ok<INUSD>
%ALPHA_FEASIBLE  Conservative feasible progress-rate teacher.
%
% Revised semantics:
%   Pbud is the active DC-bus input-power budget P_grid_budget [W], not
%   the hardware peak rating Pmax = V_nom * I_rating.  Historical callers may
%   still pass cfg.P_TOTAL_MAX; it is treated as the budget alias.
%
% To avoid relying blindly on global monotonicity, the routine first scans a
% grid over a in [amin,1].  If the feasible set is interval-like, it refines
% the upper boundary by bisection.  If non-interval behavior is detected, it
% falls back conservatively to the largest feasible command before the first
% infeasible transition and reports info.used_fallback=true.

q0 = row(q0); dq0 = row(dq0); ddq0 = row(ddq0);
nJ = numel(q0);
assert(isstruct(caps) && all(isfield(caps, {'tau_max','qd_max','qdd_max','P_axis_max'})), ...
    'alpha_feasible: caps must contain tau_max/qd_max/qdd_max/P_axis_max');
assert(isstruct(fric) && all(isfield(fric, {'B','Fc','vel_eps'})), ...
    'alpha_feasible: fric must contain B/Fc/vel_eps');

if ~exist('Pbud','var') || isempty(Pbud), Pbud = inf; end
if ~exist('amin','var') || isempty(amin), amin = 0; end
if ~exist('itmax','var') || isempty(itmax), itmax = 20; end
amin = max(0, double(amin));
Pbud = double(Pbud);

tau_lim = ensure_row_len(caps.tau_max,    nJ);
qd_lim  = ensure_row_len(caps.qd_max,     nJ);
qdd_lim = ensure_row_len(caps.qdd_max,    nJ);
Pax_lim = ensure_row_len(caps.P_axis_max, nJ);

bus = struct();
if isfield(caps,'bus') && isstruct(caps.bus), bus = caps.bus; end
if ~isfield(bus,'P_grid_budget') || isempty(bus.P_grid_budget), bus.P_grid_budget = Pbud; end
if ~isfield(bus,'P_total_max') || isempty(bus.P_total_max), bus.P_total_max = Pbud; end
if isfield(caps,'eta_share') && ~isempty(caps.eta_share), bus.eta_share = caps.eta_share; end
if isfield(caps,'P_brk_peak') && ~isempty(caps.P_brk_peak), bus.P_brk_peak = caps.P_brk_peak; end
if isfield(caps,'P_grid_peak_max') && ~isempty(caps.P_grid_peak_max), bus.P_grid_peak_max = caps.P_grid_peak_max; end
bus = spinn3d_bus_defaults(bus);

use_alpha_dot = logical(getfield_def(caps,'use_alpha_dot',false));
a_prev = getfield_def(caps,'alpha_prev',getfield_def(caps,'a_prev',NaN));
Ts_alpha = getfield_def(caps,'alpha_Ts',getfield_def(caps,'Ts',1e-3));
if ~isfinite(a_prev), use_alpha_dot = false; a_prev = 0; end

Ngrid = round(getfield_def(caps,'feasible_grid_N',41));
Ngrid = max(11, Ngrid);
agrid = linspace(amin, 1.0, Ngrid);
ok = false(size(agrid));
infos = cell(size(agrid));
for ii = 1:numel(agrid)
    [ok(ii), infos{ii}] = check_candidate(agrid(ii));
end

info = struct();
info.a_grid = agrid;
info.ok_grid = ok;
info.used_grid_scan = true;
info.used_fallback = false;
info.interval_like = true;
info.floor_infeasible = false;
info.n_feasible = sum(ok);
info.n_components = count_components(ok);
info.label_is_feasible = false;
info.a_returned_feasible = false;

if ~any(ok)
    a_star = amin;
    info.floor_infeasible = true;
    info.used_fallback = true;
    info.interval_like = false;
    info.candidate = infos{1};
    info.a_star = a_star;
    info.label_is_feasible = false;
    info.a_returned_feasible = false;
    return;
end

% From here on the returned command is selected from a feasible grid point
% or a bisection refinement of a feasible bracket.  Non-interval behavior may
% still trigger used_fallback=true, but the selected command itself is feasible.
info.label_is_feasible = true;
info.a_returned_feasible = true;

% Detect interval-like prefix after the first feasible point.
first_ok = find(ok, 1, 'first');

% If everything from the first feasible point to a=1 is feasible, no
% bisection is needed.  If there is a gap and a later feasible island, keep
% the diagnostic/fallback path below instead of assuming monotonicity.
if ok(end) && ~any(~ok(first_ok:end))
    a_star = 1.0;
    info.candidate = infos{end};
    info.a_star = a_star;
    return;
end
first_bad_after = find(~ok & (1:numel(ok)) > first_ok, 1, 'first');
if isempty(first_bad_after)
    idx_lo = find(ok, 1, 'last');
else
    if any(ok(first_bad_after:end))
        info.interval_like = false;
        info.used_fallback = true;
    end
    idx_lo = max(first_ok, first_bad_after - 1);
    % Conservative fallback: do not jump to later feasible islands.
    while idx_lo > first_ok && ~ok(idx_lo)
        idx_lo = idx_lo - 1;
    end
end

if idx_lo >= numel(agrid)
    a_star = agrid(idx_lo);
    info.candidate = infos{idx_lo};
    info.a_star = a_star;
    return;
end

if info.used_fallback || ~ok(idx_lo) || ok(idx_lo+1)
    % Non-interval or no clean bracket: choose conservative grid value.
    feasible_before = find(ok(1:idx_lo), 1, 'last');
    if isempty(feasible_before), feasible_before = first_ok; end
    a_star = agrid(feasible_before);
    info.candidate = infos{feasible_before};
    info.a_star = a_star;
    return;
end

% Refine interval boundary by bisection between feasible idx_lo and infeasible idx_lo+1.
lo = agrid(idx_lo);
hi = agrid(idx_lo+1);
last_info = infos{idx_lo};
for it = 1:max(1,itmax)
    mid = 0.5*(lo + hi);
    [ok_mid, mid_info] = check_candidate(mid);
    if ok_mid
        lo = mid;
        last_info = mid_info;
    else
        hi = mid;
    end
    if (hi - lo) < 1e-6, break; end
end

a_star = lo;
info.candidate = last_info;
info.a_star = a_star;

    function [tf, ci] = check_candidate(a)
        dq = a * dq0;
        if use_alpha_dot
            adot = (a - a_prev) / max(double(Ts_alpha), 1e-12);
            dd = (a*a) * ddq0 + adot * dq0;
        else
            adot = 0.0;
            dd = (a*a) * ddq0;  % governor alpha-dot term omitted by design; bounded elsewhere
        end
        tau = row(inverseDynamics(robot, q0, dq, dd));
        tau = tau + row(fric.B).*dq + row(fric.Fc).*tanh(dq./max(fric.vel_eps,1e-9));
        [~,~,~,~,~,flow] = spinn3d_bus_power(tau, dq, bus);
        Pax = max(flow.Paxis, 0);
        Pgrid_cap = min(bus.P_grid_budget, bus.P_grid_peak_max);
        if isfinite(Pbud), Pgrid_cap = min(Pgrid_cap, Pbud); end
        tf = all(abs(tau) <= tau_lim + 1e-9) && ...
             all(abs(dq)  <= qd_lim  + 1e-9) && ...
             all(abs(dd)  <= qdd_lim + 1e-9) && ...
             all(Pax      <= Pax_lim + 1e-9) && ...
             (flow.Pgrid  <= Pgrid_cap + 1e-9) && ...
             (flow.Pdump  <= bus.P_brk_peak + 1e-9) && ...
             (getfield_def(flow,'Psource_shortfall',0) <= 1e-9) && ...
             (flow.Pspill <= 1e-9);
        ci = struct('a',a,'dq',dq,'ddq',dd,'adot',adot,'tau',tau,'Pax',Pax, ...
                    'flow',flow,'Pgrid_cap',Pgrid_cap,'ok',tf, ...
                    'omits_alpha_dot',~use_alpha_dot);
    end
end

% ===== helpers =====
function v = row(v), v = double(v(:)).'; end
function r = ensure_row_len(v, n)
r = double(v(:)).';
assert(numel(r)==n || isscalar(r), 'alpha_feasible: size mismatch');
if isscalar(r), r = repmat(r,1,n); end
r = r(1:n);
end
function x = getfield_def(S,fn,def)
if isstruct(S)&&isfield(S,fn)&&~isempty(S.(fn)), x=S.(fn); else, x=def; end
end
function n = count_components(ok)
ok = logical(ok(:)).';
if isempty(ok), n = 0; return; end
n = sum(ok & [true, ~ok(1:end-1)]);
end
