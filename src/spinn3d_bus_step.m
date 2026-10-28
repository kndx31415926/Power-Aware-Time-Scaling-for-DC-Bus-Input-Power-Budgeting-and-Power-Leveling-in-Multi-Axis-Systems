function [flow, st_next] = spinn3d_bus_step(tau, dq, bus, st, Ts)
%SPINN3D_BUS_STEP
% Authoritative power routing, with an explicitly signed physical scope.

% The remainder of this file retains explicitly historical algebraic/storage

%
% Inputs:
%   tau, dq : joint torque / joint speed
%   bus     : BUS config struct (recommended from get_bus_from_cfg / defaults)
%   st      : current state (from spinn3d_bus_state_init)
%   Ts      : sample time [s]
%
% Outputs:
%   flow.Paxis           : per-axis power tau .* dq [W]
%   flow.Ppos / Pneg     : summed mechanical motoring / braking power [W]
%   flow.Pmot_axis_dc    : joint-drive demand eligible for direct sharing [W]
%   flow.Pmot_dc         : joint-drive demand plus auxiliary load [W]
%   flow.Pregen_mechanical_available: braking mechanical input Pminus [W]
%   flow.Pregen_bus      : regenerated power that reaches the common link [W]
%   flow.Ploss_total     : motor/regen/share/buffer conversion loss [W]
%   flow.Pshare_axis     : direct same-step axis-to-axis reuse [W]
%   flow.Pshare_potential: demand-limited overlap before enable/route gate [W]
%   flow.Pregen_curtailed: bus-available regen locally dissipated [W]
%   flow.Pbuf_dis/chg    : buffer discharge / charge power [W]
%   flow.Pgrid           : grid draw after direct share + buffer discharge [W]
%   flow.Pdump           : zero compatibility port in the formal no-brake model
%   flow.Pspill          : regen power that neither buffer nor brake can absorb [W]
%   flow.Preuse_buf      : buffer-to-bus reused power [W]
%   flow.Preuse_total    : total reused power = Pshare + Preuse_buf [W]
%   flow.Ein / Eout      : stored energy in/out of buffer this step [J]
%   flow.Ereuse_buf_out  : reused electrical energy delivered by buffer [J]
%   flow.Ereuse_total    : total reused electrical energy this step [J]
%   st_next.Ebuf / Ereuse / Tbrk

    if nargin < 3 || isempty(bus), bus = struct(); end
    if nargin < 5 || isempty(Ts),  Ts  = 1e-3; end

    bus = spinn3d_bus_defaults(bus);
    if strcmp(bus.model_scope,'dynamic_capacitor_common_dc_v1')
        if nargin<4, st=[]; end
        if (nargin<5 || isempty(Ts)) && isfield(bus,'dc_step_s')
            Ts=bus.dc_step_s;
        end
        [flow,st_next]=spinn3d_dynamic_bus_step(tau,dq,bus,st,Ts);
        return;
    end

    Ts = double(Ts);
    if ~bus.enable_state
        if nargin>=4 && ~isempty(st)

            stored={'Ebuf','Ereuse','Eregen_task'};
            for k=1:numel(stored)
                if isfield(st,stored{k})

                end
            end
        end

        % converted regeneration is reported as local curtailment.
        bus.Ebuf_max_J = 0.0;
        bus.Ebuf_min_J = 0.0;
        bus.Ebuf_init_J = 0.0;
        bus.Ereuse_init_J = 0.0;
    end
    if nargin < 4 || isempty(st)
        if bus.enable_state
            st = spinn3d_bus_state_init(bus);
        else
            % Do not report a fictitious charged buffer inherited from
            % cfg.BUS when the selected architecture has no persistent state.
            st = struct('Ebuf',0.0,'Ereuse',0.0,'Tbrk',double(bus.Tamb_C));
        end
    end

    % Usable-buffer reserve. This keeps the explicit buffer state from
    % being numerically or physically deep-discharged to zero, which is more
    % representative of a small supercap/battery retrofit operated with a
    % minimum voltage/SOC reserve.
    Ebuf_min = min(max(double(getfield_def_local(bus,'Ebuf_min_J',0.0)), 0.0), double(bus.Ebuf_max_J));
    if isstruct(st) && isfield(st,'Ebuf') && ~isempty(st.Ebuf)
        st.Ebuf = min(max(double(st.Ebuf), Ebuf_min), double(bus.Ebuf_max_J));
    end
    if isstruct(st) && isfield(st,'Ereuse') && ~isempty(st.Ereuse)
        st.Ereuse = min(max(double(st.Ereuse), 0.0), st.Ebuf);
    end
    if isstruct(st) && isfield(st,'Tbrk') && ~isempty(st.Tbrk)
        st.Tbrk = min(max(double(st.Tbrk), double(bus.Tamb_C)), double(bus.Tbrk_max_C));
    end
    if ~isfield(st,'Eregen_task') || isempty(st.Eregen_task)
        st.Eregen_task = 0.0;
    end
    st.Eregen_task = min(max(double(st.Eregen_task),0.0),st.Ebuf);

    tau = row(tau);
    dq  = row(dq);

    % One actuator model is shared by telemetry, feasibility, and the hard
    % source clamp. The passive DC kernel decides regeneration from signed
    % terminal power after copper and mechanical losses; negative output
    % mechanical power alone does not imply regeneration.
    mot = spinn3d_motor_power(tau, dq, bus);
    Paxis = mot.Paxis;
    Ppos = mot.Ppos;
    Pneg = mot.Pneg;
    Pmot_electrical = mot.Pmot_electrical;
    Pmot_axis_dc = mot.Pmot_axis_dc;
    Paux = mot.Paux;
    Pmot_dc = mot.Pmot_dc;
    Pregen_converted = mot.Pregen_converted;
    Pregen_bus = mot.Pregen_bus;
    Ploss_mot = mot.Ploss_mot;
    Ploss_regen = mot.Ploss_regen;
    Ploss_share = mot.Ploss_share;

    % Direct cross-axis sharing can offset joint-drive demand only. The
    % board/controller auxiliary load is always supplied by the external

    % and is never counted as regenerative axis-to-axis reuse.
    Pshare_uncapped_potential = min(Pregen_bus,Pmot_axis_dc);
    Pshare_potential = min(Pshare_uncapped_potential, ...
        bus.Pshare_peak_max_W);
    direct_route = strcmp(bus.routing_mode,'demand_limited_direct') && ...
        bus.regen_enable && bus.direct_share_enable;
    if direct_route
        Pshare = Pshare_potential;
    else
        Pshare = 0.0;
    end
    Pdem_axis = max(Pmot_axis_dc - Pshare,0.0);
    Pdem   = Pdem_axis + Paux;
    Preg   = max(Pregen_bus - Pshare, 0.0);

    route_to_state = bus.enable_state && ...
        strcmp(bus.routing_mode,'demand_limited_direct');
    if ~route_to_state
        Pbuf_dis_cap    = 0.0;
        Pbuf_chg_cap    = 0.0;
        Pbuf_dis        = 0.0;
        Pbuf_chg        = 0.0;
        Pgrid           = Pdem;
        % A profile without an identified buffer/brake route does not label
        % unconsumed regenerated power as fictitious dump/spill energy.
        % It is curtailed at the converter/local drive boundary instead.
        Pregen_curtailed = Preg;
        Pdump_req       = 0.0;
        Pdump_thermal_cap = bus.P_brk_peak;
        Pdump_cap       = bus.P_brk_peak;
        Pdump           = 0.0;
        Pspill          = 0.0;

        Ein             = 0.0;
        Eout            = 0.0;
        phi_regen       = 0.0;
        phi_task        = 0.0;
        Ereuse_store_out = 0.0;
        Eregen_task_out = 0.0;
        Preuse_buf      = 0.0;
        Preuse_task_buf = 0.0;
        Ereuse_buf_out  = 0.0;

        st_next = st;
    else
        Pregen_curtailed = 0.0;
        % A single bidirectional buffer converter has one signed port command:
        % it may charge or discharge in one sample, never both.  Residual
        % regeneration is given charge priority (then dump, then spill), which
        % is the documented stateful routing order.  This matters when direct
        % axis sharing has already saturated but Pdem and Preg are both
        % positive, e.g. because Paux remains source-side or Pshare is capped.
        %
        % Compute both physical caps first, then select exactly one direction.
        % Ebuf_min is a reserve/SOC floor; only energy above it is available.
        Eusable = max(st.Ebuf - Ebuf_min, 0.0);
        Pbuf_dis_cap = min(bus.Pbuf_dis_max_W, Eusable * bus.eta_buf_dis / Ts);
        Pbuf_dis_cap = max(Pbuf_dis_cap, 0.0);
        Pbuf_chg_cap = min(bus.Pbuf_chg_max_W, (bus.Ebuf_max_J - st.Ebuf) / max(bus.eta_buf_chg * Ts, 1e-12));
        Pbuf_chg_cap = max(Pbuf_chg_cap, 0.0);

        Pbuf_chg_request = min(Preg, Pbuf_chg_cap);
        if Preg > 0.0
            Pbuf_chg = Pbuf_chg_request;
            Pbuf_dis = 0.0;
        else
            Pbuf_chg = 0.0;
            Pbuf_dis = min(Pdem, Pbuf_dis_cap);
        end
        Pgrid = max(Pdem - Pbuf_dis, 0.0);

        Pdump_req = max(Preg - Pbuf_chg, 0.0);

        % First-order brake thermal model:
        %   Tdot = (Pdump - (T - Tamb)/Rth) / Cth
        % Enforce T_next <= Tbrk_max_C by computing a thermal admissible Pdump cap.
        Pdump_thermal_cap = max(0.0, ...
            ((bus.Tbrk_max_C - st.Tbrk) * bus.Cth_brk_J_per_C / Ts) + ...
            ((st.Tbrk - bus.Tamb_C) / bus.Rth_brk_C_per_W));

        Pdump_cap = min(bus.P_brk_peak, Pdump_thermal_cap);
        Pdump_cap = max(Pdump_cap, 0.0);

        Pdump  = min(Pdump_req, Pdump_cap);
        Pspill = max(Pdump_req - Pdump, 0.0);

        % Buffer stored-energy bookkeeping.
        Ein  = Ts * bus.eta_buf_chg * Pbuf_chg;
        Eout = Ts * Pbuf_dis / max(bus.eta_buf_dis, 1e-12);

        if st.Ebuf > 0
            phi_regen = min(max(st.Ereuse / st.Ebuf, 0.0), 1.0);
            phi_task = min(max(st.Eregen_task / st.Ebuf, 0.0), 1.0);
        else
            phi_regen = 0.0;
            phi_task = 0.0;
        end

        Ereuse_store_out = min(st.Ereuse, phi_regen * Eout);
        Eregen_task_out  = min(st.Eregen_task, phi_task * Eout);
        Preuse_buf       = phi_regen * Pbuf_dis;
        Preuse_task_buf  = phi_task * Pbuf_dis;
        Ereuse_buf_out   = Preuse_buf * Ts;

        st_next = st;
        st_next.Ebuf = min(bus.Ebuf_max_J, max(Ebuf_min, st.Ebuf - Eout + Ein));
        st_next.Ereuse = min(st_next.Ebuf, max(0.0, st.Ereuse - Ereuse_store_out + Ein));
        st_next.Eregen_task = min(st_next.Ebuf, max(0.0, st.Eregen_task - Eregen_task_out + Ein));
        st_next.Tbrk = st.Tbrk + Ts * ...
            (Pdump - (st.Tbrk - bus.Tamb_C) / bus.Rth_brk_C_per_W) / bus.Cth_brk_J_per_C;
        st_next.Tbrk = min(max(st_next.Tbrk, bus.Tamb_C), max(bus.Tbrk_max_C, bus.Tamb_C));
    end

    if Pbuf_dis > 0.0 && Pbuf_chg > 0.0
        error('spinn3d_bus_step:BufferPortDirectionConflict', ...
            ['A single buffer port cannot charge and discharge in the same ', ...
             'sample (Pbuf_dis=%.9g W, Pbuf_chg=%.9g W).'], ...
            Pbuf_dis,Pbuf_chg);
    end

    Preuse_total = Pshare + Preuse_buf;
    Ereuse_total = Preuse_total * Ts;
    Pregen_task_used = Pshare + Preuse_task_buf;

    Ploss_buf_chg = max((1.0 - bus.eta_buf_chg) * Pbuf_chg, 0.0);
    Ploss_buf_dis = max((1.0 / bus.eta_buf_dis - 1.0) * Pbuf_dis, 0.0);
    Pregen_local_dissipation = Pregen_curtailed;
    Plocal_dissipation = Ploss_regen + Ploss_share + ...
        Pregen_local_dissipation;
    Ploss_total = Ploss_mot + Plocal_dissipation + ...
        Ploss_buf_chg + Ploss_buf_dis;

    % Two independent balances prevent the same regenerative watt from
    % being counted simultaneously as share, storage, brake and curtailment.
    Pload_balance_residual = Pmot_dc - ...
        (Pgrid + Pbuf_dis + Pshare);
    Pregen_route_balance_residual = Pregen_bus - ...
        (Pshare + Pbuf_chg + Pdump + Pspill + Pregen_curtailed);
    bal_scale = max([1.0,Pmot_dc,Pregen_bus]);
    if abs(Pload_balance_residual) > 1e-9*bal_scale || ...
            abs(Pregen_route_balance_residual) > 1e-9*bal_scale
        error('spinn3d_bus_step:PowerBalanceViolation', ...
            ['BUS routing balance failed: load residual %.9g W, ', ...
             'regen residual %.9g W.'], ...
            Pload_balance_residual,Pregen_route_balance_residual);
    end

    % Full mechanical/electrical energy identity. The routing-only balances
    % above cannot catch a motor kernel that creates energy upstream.
    % Mechanical braking supplies Pneg, delivered output is Ppos. In a

    % spill is an explicitly unserved sink, never usable power or success.
    Pstorage_rate = (Ein-Eout)/Ts;
    Penergy_balance_residual = Pgrid + Pneg - ...
        (Ppos + Paux + Ploss_total + Pstorage_rate + Pdump + Pspill);
    energy_scale = max([1.0,Pgrid,Ppos,Pneg,Ploss_total]);
    if abs(Penergy_balance_residual)>1e-9*energy_scale || ...
            Ploss_total < -1e-12*energy_scale
        error('spinn3d_bus_step:PhysicalEnergyBalanceViolation', ...
            'Mechanical-to-source energy balance failed by %.9g W.', ...
            Penergy_balance_residual);
    end

    flow = struct();
    flow.model_scope = bus.model_scope;
    flow.motor_power_schema = bus.motor_power_schema;
    flow.auxiliary_topology = bus.auxiliary_topology;
    flow.has_voltage_state = false;
    flow.has_identified_brake_thermal_state = false;
    flow.V_bus_V = bus.V_bus_V;
    flow.V_nominal_V = bus.V_nominal_V;
    flow.Pstorage_rate = Pstorage_rate;
    flow.Penergy_balance_residual = Penergy_balance_residual;
    detailed={'Paxis_terminal_signed','Paxis_dc_signed', ...
        'Paxis_copper_loss','Paxis_mechanical_loss','Paxis_loss_drive', ...
        'P_copper_loss','P_mechanical_loss','Pterminal_balance_residual', ...
        'Paxis_terminal_balance_residual','Pregen_axis_dc_raw', ...
        'Paxis_dc_signed_raw','Ploss_copper','Ploss_mechanical', ...
        'Paxis_loss_copper','Paxis_loss_drag','Pregen_terminal_available_W', ...
        'Pregen_axis_terminal_available_W','I_winding_signed_A', ...
        'V_terminal_signed_V','electrical_balance_residual_W'};
    for k=1:numel(detailed)
        if isfield(mot,detailed{k}), flow.(detailed{k})=mot.(detailed{k}); end
    end
    flow.Paxis            = Paxis;
    flow.Paxis_electrical = mot.Paxis_electrical;
    flow.Ppos             = Ppos;
    flow.Pneg             = Pneg;
    flow.Pmot_axis_dc     = Pmot_axis_dc;
    flow.Pmot_electrical  = Pmot_electrical;
    flow.Pmot_dc          = Pmot_dc;
    flow.Pregen_converted_uncapped = mot.Pregen_converted_uncapped;
    flow.Pregen_mechanical_available = sum( ...
        getfield_def_local(mot,'Pregen_axis_mechanical_available', ...
            max(-Paxis,0.0)));
    flow.Pregen_available = mot.Pregen_available;
    flow.Pregen_converted = Pregen_converted;
    flow.Pregen_cap_curtailed = mot.Pregen_cap_curtailed;
    flow.Pregen_bus       = Pregen_bus;
    flow.Paux             = Paux;
    flow.Ploss_mot        = Ploss_mot;
    flow.Ploss_regen      = Ploss_regen;
    flow.Ploss_share      = Ploss_share;
    flow.Ploss_buf_chg    = Ploss_buf_chg;
    flow.Ploss_buf_dis    = Ploss_buf_dis;
    flow.Ploss_total      = Ploss_total;
    flow.Pshare           = Pshare;
    flow.Pshare_axis      = Pshare;
    flow.Pshare_uncapped_potential = Pshare_uncapped_potential;
    flow.Pshare_potential = Pshare_potential;
    flow.Poverlap         = Pshare_potential;
    flow.Pregen_curtailed = Pregen_curtailed;
    flow.Pregen_curtailed_total = Pregen_curtailed + ...
        mot.Pregen_cap_curtailed;
    flow.Pregen_local_dissipation = Pregen_local_dissipation;
    flow.Plocal_dissipation = Plocal_dissipation;
    flow.Pdem             = Pdem;
    flow.Pdem_axis        = Pdem_axis;
    flow.Preg             = Preg;
    flow.Pbuf_dis_cap     = Pbuf_dis_cap;
    flow.Pbuf_chg_cap     = Pbuf_chg_cap;
    flow.Pbuf_dis         = Pbuf_dis;
    flow.Pbuf_chg         = Pbuf_chg;
    flow.Pgrid            = Pgrid;
    flow.Pdump_req        = Pdump_req;
    flow.Pdump_thermal_cap = Pdump_thermal_cap;
    flow.Pdump_cap        = Pdump_cap;
    flow.Pdump            = Pdump;
    flow.Pspill           = Pspill;

    flow.Ein              = Ein;
    flow.Eout             = Eout;
    flow.phi_regen        = phi_regen;
    flow.phi_task         = phi_task;
    flow.Ereuse_store_out = Ereuse_store_out;
    flow.Eregen_task_out  = Eregen_task_out;
    flow.Preuse_buf       = Preuse_buf;
    flow.Preuse_task_buf  = Preuse_task_buf;
    flow.Preuse_total     = Preuse_total;
    flow.Pregen_task_used = Pregen_task_used;
    flow.Ereuse_buf_out   = Ereuse_buf_out;
    flow.Ereuse_total     = Ereuse_total;
    flow.Eshare_axis      = Pshare * Ts;
    flow.Eoverlap         = Pshare_potential * Ts;
    flow.Eoverlap_uncapped = Pshare_uncapped_potential * Ts;
    flow.Eregen_curtailed = Pregen_curtailed * Ts;
    flow.Eregen_curtailed_total = ...
        (Pregen_curtailed + mot.Pregen_cap_curtailed) * Ts;
    flow.Eregen_bus_in    = Pregen_bus * Ts;
    flow.Pload_balance_residual = Pload_balance_residual;
    flow.Pregen_route_balance_residual = ...
        Pregen_route_balance_residual;
    flow.architecture_id  = bus.architecture_id;
    flow.profile_id       = bus.profile_id;
    flow.routing_mode     = bus.routing_mode;
    flow.direct_share_enable = bus.direct_share_enable;
    flow.motor_model      = mot.model;
    flow.quadrant_power_model = mot.quadrant_power_model;
    flow.Iaxis_source     = mot.Iaxis_source;
    flow.I_source_total_unshared = mot.I_source_total_unshared;
    if isfinite(mot.V_bus_V) && mot.V_bus_V > 0
        flow.I_source_total = Pgrid / mot.V_bus_V;
    else
        flow.I_source_total = NaN;
    end
    flow.motor_duty       = mot.duty;
    flow.torque_speed_ratio = mot.torque_speed_ratio;
    flow.tau_speed_cap    = mot.tau_speed_cap;
    flow.actuator_envelope_model = mot.actuator_envelope_model;
    flow.motor_current_ratio = mot.r_current;
    flow.motor_voltage_ratio = mot.r_voltage;
    flow.motor_voltage_command_ratio_signed = ...
        mot.voltage_command_ratio_signed;
    flow.tau_envelope_min_Nm = mot.tau_envelope_min_Nm;
    flow.tau_envelope_max_Nm = mot.tau_envelope_max_Nm;
    flow.motor_envelope_axis_ok = mot.envelope_axis_ok;
    flow.motor_envelope_ok = mot.envelope_ok;
    flow.Ebuf_min_J       = Ebuf_min;
    if exist('st_next','var') && isstruct(st_next) && isfield(st_next,'Ebuf')
        flow.Ebuf_usable = max(st_next.Ebuf - Ebuf_min, 0.0);
    else
        flow.Ebuf_usable = 0.0;
    end
end

function y = row(x)
    y = x(:)';
end

function v = getfield_def_local(S, fn, def)
    if isstruct(S) && isfield(S, fn) && ~isempty(S.(fn))
        v = S.(fn);
    else
        v = def;
    end
end
