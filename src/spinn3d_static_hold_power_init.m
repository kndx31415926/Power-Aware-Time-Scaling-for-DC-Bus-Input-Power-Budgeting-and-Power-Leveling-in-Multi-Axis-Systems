function init = spinn3d_static_hold_power_init(robot, q0, fric, bus, Pcap, Ts)
% Initialize the electrical state and power filter at static payload hold.

if nargin < 3 || isempty(fric), fric = struct(); end
if nargin < 4 || isempty(bus),  bus  = struct(); end
if nargin < 5 || isempty(Pcap), Pcap = inf; end
if nargin < 6 || isempty(Ts),   Ts   = 1e-3; end

try, robot.DataFormat = 'row'; end
nJ = numel(homeConfiguration(robot));
q0 = row_local(q0);

Ts = double(Ts);

bus = spinn3d_bus_defaults(bus);
dq0 = zeros(1,nJ);
ddq0 = zeros(1,nJ);

B  = expand_row_local(getfield_def_local(fric,'B',0.0),nJ,0.0);
Fc = expand_row_local(getfield_def_local(fric,'Fc',0.0),nJ,0.0);
ve = max(double(getfield_def_local(fric,'vel_eps',1e-3)),1e-9);

tau_gravity = row_local(gravityTorque(robot,q0));
tau_friction = B.*dq0 + Fc.*tanh(dq0./ve);
tau_static = tau_gravity + tau_friction;

use_bus_state = logical(getfield_def_local(bus,'enable_state',false));
explicit_dc=strcmp(bus.model_scope,'dynamic_capacitor_common_dc_v1') && ...
    strcmp(getfield_def_local(bus,'dc_initial_state_mode','static_hold_equilibrium'), ...
        'explicit_measured_state');
if explicit_dc

    
    bus_state0=spinn3d_dc_state_for_model(bus.dc_state,bus);
elseif use_bus_state
    bus_init=bus;
    if strcmp(bus.model_scope,'dynamic_capacitor_common_dc_v1')
        % Match the source/load port equation. A 1-ms precharged transient is
        % not a static-hold equilibrium and cannot initialize its power EMA.
        hold_motor=spinn3d_motor_power(tau_static,dq0,bus);
        equilibrium=spinn3d_dc_hold_equilibrium(bus,double(hold_motor.Pmot_axis_dc));

        Vhold=equilibrium.Vdc;
        bus_init.V_init_V=Vhold;
        bus_init.Ebuf_init_J=0.5*bus.C_dc_F*Vhold^2;
    end
    bus_state0 = spinn3d_bus_state_init(bus_init);
else
    bus_state0 = [];
end

% Evaluate one static sample to obtain the source flow. Keep
% bus_state0 unchanged so initialization does not consume buffer energy or
% advance the capacitor state before t=0.
[flow_static, bus_state_static_next] = ...
    spinn3d_bus_step(tau_static,dq0,bus,bus_state0,Ts);
Pgrid_static = double(flow_static.Pgrid);
if strcmp(bus.model_scope,'dynamic_capacitor_common_dc_v1') && ~explicit_dc

end
Pavg_initial=Pgrid_static;
if explicit_dc
    Pavg_initial=double(bus.initial_Pgrid_avg_W);

end

% A causal first-order filter driven by a constant hold is at equilibrium
% when its previous value equals that hold power.
bus_avg = bus;
bus_avg.Pgrid_avg_prev = Pavg_initial;
[Pgrid_avg0, ok_avg0, Pavg_cap, Pgrid_peak_cap, beta_avg] = ...
    spinn3d_bus_avg_guard(Pavg_initial,Pcap,bus_avg,Ts);

init = struct();
init.scheme = 'q0_static_hold_equilibrium_v1';
if strcmp(bus.model_scope,'dynamic_capacitor_common_dc_v1')
    init.scheme='q0_static_hold_dynamic_dc_equilibrium_v2';
end
if explicit_dc
    init.scheme='explicit_dynamic_dc_continuation_v1';
end
init.explicit_dc_continuation=explicit_dc;
init.q0_rad = q0;
init.dq0_rad_s = dq0;
init.ddq0_rad_s2 = ddq0;
init.tau_gravity_Nm = tau_gravity;
init.tau_friction_Nm = tau_friction;
init.tau_static_Nm = tau_static;
init.Pgrid_static_W = Pgrid_static;
init.Pgrid_lp0_W = Pavg_initial;
init.Pgrid_avg0_W = double(Pgrid_avg0);
init.Pavg_cap_W = double(Pavg_cap);
init.Pgrid_peak_cap_W = double(Pgrid_peak_cap);
init.Pavg_soft_target_ok = logical((~isfinite(Pavg_cap)) || ...
    (Pgrid_avg0 <= Pavg_cap + 1e-9));
init.Pavg_soft_excess_W = max(double(Pgrid_avg0)-double(Pavg_cap),0.0);
init.Pavg_margin_W = double(Pavg_cap)-double(Pgrid_avg0);
if isinf(Pavg_cap)
    init.Pavg_soft_ratio = 0.0;
elseif Pavg_cap > 0
    init.Pavg_soft_ratio = double(Pgrid_avg0)/double(Pavg_cap);
elseif Pgrid_avg0 <= 1e-9
    init.Pavg_soft_ratio = 0.0;
else
    init.Pavg_soft_ratio = inf;
end
init.pavg_kpi_pass = init.Pavg_soft_target_ok;
init.Pavg_excess_W = init.Pavg_soft_excess_W;
init.pavg_ratio = init.Pavg_soft_ratio;
init.Pavg_is_hard_constraint = false;
% Compatibility field retained, now with KPI-target semantics independent

init.Pgrid_avg_initial_ok = init.Pavg_soft_target_ok;
init.Pgrid_avg_guard_ok_legacy = logical(ok_avg0);
init.Pgrid_peak_initial_ok = logical( ...
    ~isfinite(Pgrid_peak_cap) || Pgrid_static <= Pgrid_peak_cap + 1e-9);
init.motor_envelope_initial_ok = logical( ...
    getfield_def_local(flow_static,'motor_envelope_ok',true));
init.spill_initial_ok = logical( ...
    getfield_def_local(flow_static,'Pspill',inf) <= 1e-9);
if use_bus_state
    Emin = min(max(double(getfield_def_local(bus, ...
        'Ebuf_min_J',0.0)),0.0),double(bus.Ebuf_max_J));
    init.thermal_initial_ok=true;
    if ~strcmp(bus.model_scope,'dynamic_capacitor_common_dc_v1')
        init.thermal_initial_ok=bus_state_static_next.Tbrk<=bus.Tbrk_max_C+1e-9;
    end
    init.buffer_initial_ok = logical( ...
        bus_state_static_next.Ebuf >= Emin - 1e-9 && ...
        bus_state_static_next.Ebuf <= bus.Ebuf_max_J + 1e-9);
    if strcmp(bus.model_scope,'dynamic_capacitor_common_dc_v1')
        init.thermal_initial_ok=logical(flow_static.thermal_ok);
        init.buffer_initial_ok=logical(flow_static.voltage_ok);
    end
else
    init.thermal_initial_ok = true;
    init.buffer_initial_ok = true;
end
init.dynamic_dc_initial_ok=logical(getfield_def_local(flow_static,'state_pass',true));
init.static_source_feasible = logical( ...
    init.dynamic_dc_initial_ok && init.Pgrid_peak_initial_ok && init.motor_envelope_initial_ok && ...
    init.spill_initial_ok && init.thermal_initial_ok && ...
    init.buffer_initial_ok);
init.static_physical_feasible = init.static_source_feasible;
init.beta_avg = double(beta_avg);
init.plant_dt_s = Ts;
init.bus_state_enabled = use_bus_state;
init.bus_state0 = bus_state0;
init.flow_static = flow_static;
end

function v = expand_row_local(v,n,def)
try, v = double(v(:).'); catch, v = def; end
if isempty(v), v = def; end
if isscalar(v)
    v = repmat(v,1,n);
elseif numel(v) < n
    v = [v repmat(v(end),1,n-numel(v))];
else
    v = v(1:n);
end
v(~isfinite(v)) = def;
end

function v = getfield_def_local(S,fn,def)
if isstruct(S) && isfield(S,fn) && ~isempty(S.(fn))
    v = S.(fn);
else
    v = def;
end
end

function y = row_local(x)
y = double(x(:).');
end
