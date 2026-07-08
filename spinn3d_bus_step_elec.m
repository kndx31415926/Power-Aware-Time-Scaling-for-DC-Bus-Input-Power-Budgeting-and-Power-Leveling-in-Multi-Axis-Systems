function [flow, st_next] = spinn3d_bus_step_elec(tau, dq, bus, st, Ts)
%SPINN3D_BUS_STEP_ELEC  One servo-step electrical DC-bus telemetry update.
%
% This function is intentionally a telemetry/state model, not a new planner.
% It is used to compute Vdc/Igrid/energy/dump/spill/thermal quantities while
% preserving the original fixed-path progress-rate governor architecture.

if nargin < 3 || isempty(bus), bus = struct(); end
if nargin < 5 || isempty(Ts), Ts = 1e-3; end
bus = spinn3d_bus_defaults(bus);
Ts = max(double(Ts), 1e-12);
if nargin < 4 || isempty(st), st = spinn3d_bus_state_init(bus); end

% First compute the instantaneous electrical demand after same-step sharing.
[~, ~, ~, ~, ~, f0] = spinn3d_bus_power(tau, dq, bus);

Ebuf0 = getfield_def(st,'Ebuf',bus.Ebuf_init_J);
Ereuse0 = getfield_def(st,'Ereuse',min(bus.Ereuse_init_J,Ebuf0));
Tbrk0 = getfield_def(st,'Tbrk',bus.Tbrk_init_C);
Vprev = getfield_def(st,'Vdc',bus.V_nom);

Eusable = max(Ebuf0 - bus.Ebuf_min_J, 0.0);
Eroom   = max(bus.Ebuf_max_J - Ebuf0, 0.0);

% Buffer discharge can reduce grid draw.  We keep this explicit so energy
% consumed from source and reused regenerative energy can be reported.
Pbuf_dis = 0.0; Eout = 0.0; Ereuse_buf_out = 0.0;
if bus.enable_state && f0.Pgrid > 0 && Eusable > 0
    Pbuf_dis = min([f0.Pgrid, bus.Pbuf_dis_max_W, Eusable * bus.eta_buf_dis / Ts]);
    Eout = Pbuf_dis / max(bus.eta_buf_dis, eps) * Ts;
    frac_reuse = min(Ereuse0 / max(Ebuf0, eps), 1.0);
    Ereuse_buf_out = Eout * frac_reuse * bus.eta_buf_dis;
end
Pgrid = max(f0.Pgrid - Pbuf_dis, 0.0);

% Regenerative surplus first charges the finite buffer, then goes to braking
% resistor, then spills if neither path can absorb it.
Preg_surplus = f0.Pdump_req;
Pbuf_chg = 0.0; Ein = 0.0;
if bus.enable_state && Preg_surplus > 0 && Eroom > 0
    Pbuf_chg = min([Preg_surplus, bus.Pbuf_chg_max_W, Eroom / max(bus.eta_buf_chg,eps) / Ts]);
    Ein = Pbuf_chg * bus.eta_buf_chg * Ts;
end
Pdump_req2 = max(Preg_surplus - Pbuf_chg, 0.0);
Pdump = min(Pdump_req2, bus.P_brk_peak);
Pspill = max(Pdump_req2 - Pdump, 0.0);

% Brake thermal state: first-order RC model.
Tamb = double(bus.Tamb_C);
Tdot = (Pdump - (Tbrk0 - Tamb) / max(bus.Rth_brk_C_per_W, eps)) / max(bus.Cth_brk_J_per_C, eps);
Tbrk = min(max(Tbrk0 + Ts*Tdot, Tamb), bus.Tbrk_max_C);

% Update buffer/reuse energy bookkeeping.
Ebuf = min(max(Ebuf0 + Ein - Eout, bus.Ebuf_min_J), bus.Ebuf_max_J);
Ereuse = min(max(Ereuse0 + Ein - Ereuse_buf_out, 0.0), Ebuf);

% Source current/voltage after buffer discharge.
[Igrid, Vdc_alg] = source_current_voltage_local(Pgrid, bus);
if bus.Cdc > 0 && isfinite(bus.Cdc) && bus.R_batt > 0
    tau_v = max(bus.R_batt * bus.Cdc, Ts);
    beta_v = exp(-Ts / tau_v);
    Vdc = beta_v * Vprev + (1-beta_v) * Vdc_alg;
else
    Vdc = Vdc_alg;
end
Vdc_raw = Vdc;
if isfinite(bus.V_max), Vdc = min(Vdc, bus.V_max); end
Vdc = max(Vdc, 1e-9);
Ibatt = Pgrid / max(Vdc, 1e-12);

% Pack flow
flow = f0;
flow.Pgrid_raw_after_share = f0.Pgrid;
flow.Pbuf_dis = Pbuf_dis;
flow.Pbuf_chg = Pbuf_chg;
flow.Pgrid = Pgrid;
flow.Pdump_req = Pdump_req2;
flow.Pdump = Pdump;
flow.Pspill = Pspill;
flow.Preuse_share = f0.Pshare;
flow.Preuse_buf = Pbuf_dis;
flow.Preuse_total = f0.Pshare + Pbuf_dis;
flow.Ein = Ein;
flow.Eout = Eout;
flow.Egrid = Pgrid * Ts;
flow.Edump = Pdump * Ts;
flow.Espill = Pspill * Ts;
flow.Ereuse_step = flow.Preuse_total * Ts;
flow.Ereuse_total = getfield_def(st,'Ereuse_total',0) + flow.Ereuse_step;  % cumulative reused energy [J]
flow.Ereuse_cum = flow.Ereuse_total;
flow.Eloss = f0.Ploss * Ts;
flow.Vdc = Vdc;
flow.Vdc_raw = Vdc_raw;
flow.undervoltage = isfinite(bus.V_min) && (Vdc_raw < bus.V_min);
flow.Igrid = Igrid;
flow.Ibatt = Ibatt;
flow.Tbrk = Tbrk;
flow.Ebuf = Ebuf;
flow.Ereuse = Ereuse;

st_next = st;
st_next.Ebuf = Ebuf;
st_next.Ereuse = Ereuse;
st_next.Tbrk = Tbrk;
st_next.Vdc = Vdc;
st_next.Egrid_total  = getfield_def(st,'Egrid_total',0)  + flow.Egrid;
st_next.Edump_total  = getfield_def(st,'Edump_total',0)  + flow.Edump;
st_next.Espill_total = getfield_def(st,'Espill_total',0) + flow.Espill;
st_next.Ereuse_total = flow.Ereuse_total;
st_next.Eloss_total  = getfield_def(st,'Eloss_total',0)  + flow.Eloss;
end

function [I, V] = source_current_voltage_local(P, bus)
P = max(double(P), 0.0);
Voc = max(double(bus.V_nom), 1e-9);
Rb = max(double(bus.R_batt), 0.0);
if Rb > 0 && isfinite(Rb)
    disc = Voc^2 - 4*Rb*P;
    if disc >= 0
        I = (Voc - sqrt(disc)) / (2*Rb);
    else
        I = Voc / (2*Rb);
    end
else
    I = P / Voc;
end
V = Voc - Rb * I;
if isfinite(bus.V_max), V = min(V, bus.V_max); end
V = max(V, 1e-9);
end

function v = getfield_def(S, fn, def)
if isstruct(S) && isfield(S,fn) && ~isempty(S.(fn)), v = S.(fn); else, v = def; end
end
