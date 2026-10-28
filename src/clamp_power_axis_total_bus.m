function tau_out = clamp_power_axis_total_bus(tau, dq, P_axis_max, bus)

%

    tau = row(tau);
    dq  = row(dq);
    n   = numel(tau);

    if nargin < 3 || isempty(P_axis_max), P_axis_max = inf(1,n); end
    P_axis_max = expand_limit(P_axis_max,n,inf);

    if nargin < 4 || isempty(bus), bus = struct(); end
    % The project always ships the authoritative BUS validator.  Never

    % must fail before any torque command can be returned.
    bus = spinn3d_bus_defaults(bus);

    if ~isfield(bus,'P_brk_peak') && isfield(bus,'P_dump_max'), bus.P_brk_peak = bus.P_dump_max; end
    if ~isfield(bus,'P_dump_max') && isfield(bus,'P_brk_peak'), bus.P_dump_max = bus.P_brk_peak; end

    % Use instantaneous peak guard for torque-level clamping.
    % The filtered software budget is handled by planner/controller guards.
    Pcap      = getfield_def(bus, 'P_grid_peak_max', getfield_def(bus, 'P_total_max', inf));

    % ---------- 0) ST3215 torque-speed envelope ----------
    if is_st3215_model(bus)
        mot0 = spinn3d_motor_power(tau,dq,bus);
        tau_lo = row(mot0.tau_envelope_min_Nm);
        tau_hi = row(mot0.tau_envelope_max_Nm);
        if any(~isfinite(tau_lo)) || any(~isfinite(tau_hi)) || ...
                any(tau_lo>tau_hi)
            error('clamp_power_axis_total_bus:EmptyActuatorEnvelope', ...
                ['No finite voltage/current torque interval exists at the ', ...
                 'supplied joint velocity.']);
        end
        tau = min(tau_hi,max(tau_lo,tau));
    end

    Pj = tau .* dq;
    for i = 1:n
        if isfinite(P_axis_max(i)) && (Pj(i) > P_axis_max(i))
            s = P_axis_max(i) / max(Pj(i), 1e-12);   
            tau(i) = tau(i) * s;
        end
    end

    tau = clamp_bus_grid_power(tau, dq, Pcap, bus);

    % A common BUS scale moves torque toward zero.  For a signed bridge above
    % no-load speed, however, the feasible interval can lie wholly in the

    % instead of assuming that any power reduction preserves feasibility.
    if is_st3215_model(bus)
        mot_final = spinn3d_motor_power(tau,dq,bus);
        if ~mot_final.envelope_ok
            error('clamp_power_axis_total_bus:ActuatorEnvelopeLostAfterClamp', ...
                ['Axis/BUS power clamping moved the command outside the ', ...
                 'signed actuator voltage/current envelope.']);
        end
    end

    tau_out = tau;
end

% ===== helpers =====
function tau = clamp_bus_grid_power(tau, dq, Pcap, bus)
    if ~isfinite(Pcap), return; end

    [Pgrid,~,motor_ok] = instantaneous_bus_split(tau,dq,bus);
    if Pgrid <= Pcap + 1e-12 && motor_ok
        return;
    end

    % Use the authoritative BUS router for every actuator model.  The
    % endpoint-current model is nonlinear and regeneration is profile-gated,
    % so the former closed-form splitter could disagree with execution and
    % incorrectly use regenerated power against P_idle_W.
    tau_req = tau;

    % With the explicit paper quadrant mask, a multi-axis regenerative
    % profile need not be globally monotone in one common torque scale:
    % different axes can cross p_i=0 while direct sharing and converter caps
    % become active.  Scan the complete
    % [0,1] interval, retain the largest feasible island and refine its upper
    % boundary without assuming global monotonicity.
    scale_grid = linspace(0.0,1.0,129);
    feasible = false(size(scale_grid));
    for ig = 1:numel(scale_grid)
        [Pg,~,motor_ok] = instantaneous_bus_split( ...
            scale_grid(ig)*tau_req,dq,bus);
        feasible(ig) = Pg <= Pcap + 1e-12 && motor_ok;
    end
    idx = find(feasible,1,'last');
    if isempty(idx)
        % No feasible point exists on the deterministic scale family.
        % Returning an infeasible zero command would violate the hard cap;

        % acceleration/torque direction.
        error('clamp_power_axis_total_bus:NoFeasibleGridScale', ...
            ['No common torque scale satisfies the %.9g-W physical ', ...
             'source cap for the supplied velocity state.'],Pcap);
    end
    lo = scale_grid(idx);
    if idx < numel(scale_grid)
        hi = scale_grid(idx+1);
        for it = 1:32
            mid = 0.5*(lo+hi);
            [Pm,~,motor_ok] = instantaneous_bus_split( ...
                mid*tau_req,dq,bus);
            if Pm <= Pcap + 1e-12 && motor_ok
                lo = mid;
            else
                hi = mid;
            end
        end
    end
    tau = lo*tau_req;
    [Pfinal,~,motor_ok] = instantaneous_bus_split(tau,dq,bus);
    if Pfinal > Pcap + 1e-9 || ~motor_ok
        error('clamp_power_axis_total_bus:GridClampFailed', ...
            ['Final command has source draw %.9g/%.9g W or lies outside ', ...
             'the actuator envelope (motor_ok=%d).'], ...
            Pfinal,Pcap,double(motor_ok));
    end
end

function [Pgrid,Pdump,motor_ok] = instantaneous_bus_split(tau,dq,bus)
    dynamic=isfield(bus,'model_scope') && ...
        strcmp(bus.model_scope,'dynamic_capacitor_common_dc_v1');
    if dynamic

        [flow,~]=spinn3d_bus_step(tau,dq,bus,bus.dc_state,bus.dc_step_s);
    else
        bus.enable_state = false;
        [flow,~] = spinn3d_bus_step(tau,dq,bus,[],1.0);
    end
    Pgrid = flow.Pgrid;
    Pdump = flow.Pdump_req;
    motor_ok = logical(flow.motor_envelope_ok) && ...
        logical(getfield_def(flow,'state_pass',true));
end

function tf = is_st3215_model(bus)
    try
        tf = strcmpi(strtrim(char(getfield_def(bus,'model','legacy'))),'st3215_pwm');
    catch
        tf = false;
    end
end

function v = expand_limit(v,n,def)
    if isempty(v), v = def; end
    v = row(double(v));
    if isscalar(v), v = repmat(v,1,n); end
    if numel(v) < n, v = [v repmat(v(end),1,n-numel(v))]; end
    v = v(1:n);
    v(~isfinite(v)) = def;
end

function y = row(x), y = x(:)'; end
function v = getfield_def(S,fn,def), if isstruct(S)&&isfield(S,fn)&&~isempty(S.(fn)), v=S.(fn); else, v=def; end, end
