function [sig, state] = spinn3d_bus_power_elec(tau, dq, bus, state, Ts)
%SPINN3D_BUS_POWER_ELEC  Lightweight DC-bus electrical power model.
%
% This function converts joint mechanical power to supply-side grid draw and
% braking/dump power using explicit drive efficiency, copper/inverter losses,
% source internal resistance, DC-link capacitance, voltage sag, and braking
% limits.  It is still a compact engineering model; parameters should be
% calibrated against measured voltage/current telemetry for validation.
%
% Inputs:
%   tau, dq : joint torque [N*m] and speed [rad/s]
%   bus     : struct from get_bus_from_cfg/spinn3d_bus_defaults
%   state   : optional struct with state.Vdc
%   Ts      : sample time [s]
%
% Output sig includes Pgrid, Pdump, Vdc, Igrid, Ibatt, Ichop, Ploss, Paxis.
    if nargin < 3 || isempty(bus), bus = struct(); end
    if nargin < 5 || isempty(Ts),  Ts  = 1e-3; end
    bus = spinn3d_bus_defaults(bus);
    Ts = max(double(Ts), 1e-12);

    tau = row(tau); dq = row(dq); nJ = numel(tau);
    Kt = ensure_len(bus.Kt, nJ, 0.18);
    Rw = ensure_len(bus.R_phase, nJ, 0.35);

    if nargin < 4 || isempty(state) || ~isstruct(state) || ~isfield(state,'Vdc') || isempty(state.Vdc)
        state = struct('Vdc', bus.V_nom);
    end
    Vprev = double(state.Vdc);
    if ~isfinite(Vprev) || Vprev <= 0, Vprev = bus.V_nom; end
    if isfinite(bus.V_max), Vprev = min(Vprev, bus.V_max); end
    Vprev = max(Vprev, 1e-9);

    Paxis = tau .* dq;
    Pmech_pos = sum(max(Paxis,0));
    Pmech_reg = sum(max(-Paxis,0));

    Iaxis = abs(tau) ./ max(abs(Kt), 1e-9);
    Pcu   = sum(Rw .* (Iaxis.^2));
    Pinv  = bus.P_inv0 + bus.k_inv * sum(Iaxis.^2);
    Paux  = servo_aux_loss_local(dq, bus, nJ);
    Ploss = Pcu + Pinv + Paux;

    Pmot_dc   = Pmech_pos / max(bus.eta_mot, eps) + Ploss;
    Pregen_dc = bus.eta_regen * Pmech_reg;

    Pshare = min(bus.eta_share * Pregen_dc, Pmot_dc);
    Pload_from_source = max(Pmot_dc - Pshare, 0);
    Preg_surplus      = max(Pregen_dc - Pshare, 0);

    % Source internal resistance plus optional adapter current/power limit.
    % Pload_from_source is the requested electrical demand; delivered Pgrid is
    % capped by the physical source when enable_supply_current_limit=true.
    Pgrid_request = max(Pload_from_source, 0);
    [Ibatt, Vsource, limited_by_source] = source_iv_limited_local(Pgrid_request, bus);
    Pgrid = max(Vsource * Ibatt, 0);
    Psource_shortfall = max(Pgrid_request - Pgrid, 0);

    % Braking chopper/dump.  We limit both by configured power and, if a
    % chopper voltage is given, by whether the DC-link tends to over-voltage.
    Pdump_req = Preg_surplus;
    Pdump = min(Pdump_req, bus.P_brk_peak);
    Pspill = max(Pdump_req - Pdump, 0);

    % DC-link state update for diagnostics.
    Iload = Pmot_dc   / max(Vprev, eps);
    Ireg  = Pregen_dc / max(Vprev, eps);
    Ichop = Pdump     / max(Vprev, eps);
    dV = (Ibatt + Ireg - Iload - Ichop) / max(bus.Cdc, eps);
    Vnext_raw = Vprev + Ts*dV;
    Vnext = Vnext_raw;
    if isfinite(bus.V_max), Vnext = min(Vnext, bus.V_max); end
    Vnext = max(Vnext, 1e-9);
    state.Vdc = Vnext;

    sig = struct();
    sig.Paxis       = Paxis;
    sig.Pmech_pos   = Pmech_pos;
    sig.Pmech_regen = Pmech_reg;
    sig.Ppos        = Pmech_pos;      % legacy mechanical positive power
    sig.Pneg        = Pmech_reg;      % legacy mechanical regen power
    sig.Iaxis       = Iaxis;
    sig.Pcu         = Pcu;
    sig.Pinv        = Pinv;
    sig.Paux        = Paux;
    sig.Ploss       = Ploss;
    sig.Pmot_dc     = Pmot_dc;
    sig.Pregen_dc   = Pregen_dc;
    sig.Pshare      = Pshare;
    sig.Preuse_share = Pshare;
    sig.Preuse_buf   = 0.0;
    sig.Preuse_total = Pshare;
    sig.Ereuse_step  = Pshare * Ts;
    sig.Ereuse_total = sig.Ereuse_step;
    sig.Ereuse_cum   = sig.Ereuse_total;
    sig.Pgrid_request = Pgrid_request;
    sig.Pgrid_demand  = Pgrid_request;   % legacy alias
    sig.Psource_limited = logical(limited_by_source);
    sig.Psource_shortfall = Psource_shortfall;
    sig.Pgrid       = Pgrid;
    sig.Pdump_req   = Pdump_req;
    sig.Pdump       = Pdump;
    sig.Pspill      = Pspill;
    sig.Vdc         = Vnext;
    sig.Vdc_raw     = Vnext_raw;
    sig.undervoltage = isfinite(bus.V_min) && (Vnext_raw < bus.V_min);
    sig.Vsource     = Vsource;
    sig.Ibatt       = Ibatt;
    sig.Igrid       = Pgrid / max(Vsource, eps);
    sig.Ichop       = Ichop;
    sig.P_grid_budget   = bus.P_grid_budget;
    sig.P_grid_peak_max = bus.P_grid_peak_max;
end


function Paux = servo_aux_loss_local(dq, bus, nJ)
%SERVO_AUX_LOSS_LOCAL  No-load/electronics current of compact smart servos.
% Public ST3215-class specifications list about 0.20 A no-load current and
% about 0.03 A standby current per servo at 12 V.  RoArm-M2-S has five servos
% with a dual-drive shoulder.  This term raises the delivered bus power to a
% realistic 20--30 W uncontrolled level without using impossible >60 W source
% output.
    if ~isfield(bus,'enable_servo_aux_loss') || ~bus.enable_servo_aux_loss
        Paux = 0.0; return;
    end
    dq = row(dq);
    nserv = ensure_len(bus.n_servo_axis, nJ, 1);
    I_stby = max(double(bus.I_servo_standby_A), 0);
    I_nl   = max(double(bus.I_servo_no_load_A), I_stby);
    w_nl   = max(double(bus.omega_no_load_servo_rad_s), eps);
    expo   = max(double(bus.servo_aux_speed_exp), eps);
    speed_ratio = min(abs(dq) ./ w_nl, 1.0);
    I_per_servo = I_stby + (I_nl - I_stby) .* (speed_ratio .^ expo);
    Paux = bus.V_nom * sum(nserv .* I_per_servo) + double(bus.P_control_board_W);
end

function [I, V, limited] = source_iv_limited_local(Preq, bus)
% Solve V = Voc - Rb*I, P = V*I with optional source current/power cap.
    Preq = max(double(Preq),0);
    Voc = max(double(bus.V_oc), eps);
    Rb  = max(double(bus.R_batt), 0);
    Imax = inf;
    if isfield(bus,'enable_supply_current_limit') && bus.enable_supply_current_limit
        if isfield(bus,'I_source_max') && isfinite(bus.I_source_max) && bus.I_source_max > 0
            Imax = double(bus.I_source_max);
        elseif isfield(bus,'I_rating') && isfinite(bus.I_rating) && bus.I_rating > 0
            Imax = double(bus.I_rating);
        end
    end
    Pcap = inf;
    if isfield(bus,'P_source_max_W') && isfinite(bus.P_source_max_W) && bus.P_source_max_W > 0
        Pcap = double(bus.P_source_max_W);
    end
    if isfield(bus,'P_grid_peak_max') && isfinite(bus.P_grid_peak_max) && bus.P_grid_peak_max > 0
        Pcap = min(Pcap, double(bus.P_grid_peak_max));
    end
    if Preq <= 0
        I = 0; V = Voc; limited = false;
    else
        if Rb > 0
            Pmax_parabola = Voc^2 / max(4*Rb, eps);
            Psolve = min(Preq, 0.999 * Pmax_parabola);
            disc = max(Voc^2 - 4*Rb*Psolve, 0);
            I = (Voc - sqrt(disc)) / max(2*Rb, eps);
        else
            I = Preq / Voc;
        end
        V = Voc - Rb * I;
        limited = false;
        if isfinite(Imax) && I > Imax
            I = Imax; V = Voc - Rb * I; limited = true;
        end
        if isfinite(Pcap) && I*max(V,eps) > Pcap
            I_cap = Pcap / max(Voc,eps);
            if isfinite(Imax), I_cap = min(I_cap, Imax); end
            I = min(I, I_cap); V = Voc - Rb * I; limited = true;
        end
    end
    if isfinite(bus.V_max), V = min(V, bus.V_max); end
    V = max(V, 1e-9);
end

function y = row(x), y = x(:)'; end
function y = ensure_len(v,n,def)
if isempty(v), v = def; end
v = double(v(:)).';
if isscalar(v), y = repmat(v,1,n); else, y = v(1:min(numel(v),n)); end
if numel(y) < n, y = [y, repmat(y(end),1,n-numel(y))]; end
end
