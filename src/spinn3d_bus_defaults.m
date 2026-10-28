function bus = spinn3d_bus_defaults(bus)
% Assemble motor, source and DC-link parameters from the supplied settings.

    if nargin < 1 || isempty(bus), bus = struct(); end
    strict_config = false;
    if isfield(bus,'strict_config')
        sv = bus.strict_config;

        strict_config = logical(sv);
    end
    if strict_config
        validate_strict_bus_local(bus);
    end

    % ----- Actuator electrical model -----
    if ~isfield(bus,'model') || isempty(bus.model), bus.model = 'legacy'; end
    bus.model = normalize_model(bus.model);
    is_st3215 = strcmp(bus.model,'st3215_pwm');

    if ~isfield(bus,'regen_enable') || isempty(bus.regen_enable)

        % servos are not credited with source-side regeneration unless an
        % identified regenerative retrofit is explicitly requested.
        bus.regen_enable = ~is_st3215;
    end
    re = bus.regen_enable;
    bus.regen_enable = logical(re(1));
    if ~isfield(bus,'motor_power_schema') || isempty(bus.motor_power_schema)
        if is_st3215
            bus.motor_power_schema = 'passive_output_dc_pwm_v1';
        else
            bus.motor_power_schema = 'legacy_mechanical_efficiency_v1';
        end
    end
    if ~isfield(bus,'model_scope') || isempty(bus.model_scope)
        if is_st3215
            bus.model_scope = 'regulated_algebraic_common_dc_v1';
        else
            bus.model_scope = 'legacy_storage_diagnostic_v1';
        end
    end
    if ~isfield(bus,'auxiliary_topology') || isempty(bus.auxiliary_topology)
        bus.auxiliary_topology = 'source_side_nonsharing_branch_v1';
    end
    if ~isfield(bus,'quadrant_power_model') || isempty(bus.quadrant_power_model)
        if is_st3215
            bus.quadrant_power_model = 'signed_terminal_power_v1';
        else
            bus.quadrant_power_model = 'legacy_mechanical_efficiency_v1';
        end
    end
    bus.quadrant_power_model = normalize_quadrant_model_local( ...
        bus.quadrant_power_model,is_st3215,bus.regen_enable);
    if ~isfield(bus,'actuator_envelope_model') || isempty(bus.actuator_envelope_model)
        if is_st3215
            bus.actuator_envelope_model = 'passive_dc_current_voltage_speed_v1';
        else
            bus.actuator_envelope_model = 'legacy_no_envelope_v1';
        end
    end
    bus.actuator_envelope_model = normalize_actuator_envelope_local( ...
        bus.actuator_envelope_model,is_st3215,bus.regen_enable);
    if is_st3215
        validate_st3215_profile_coherence_local(bus);
    end

    % Architecture/profile metadata are part of every newly generated

    % identities, rather than an empty value that can be mistaken for a
    % measured hardware configuration.
    if ~isfield(bus,'architecture_id') || isempty(bus.architecture_id)
        if is_st3215 && bus.regen_enable
            bus.architecture_id = ...
                'bidirectional_common_dc_link_retrofit';
        elseif is_st3215
            bus.architecture_id = 'stock_st3215_nonregenerative';
        else
            bus.architecture_id = 'legacy_common_dc_link';
        end
    end
    bus.architecture_id = normalize_identifier_local(bus.architecture_id, ...
        'legacy_common_dc_link');
    if ~isfield(bus,'profile_id') || isempty(bus.profile_id)
        bus.profile_id = bus.architecture_id;
    end
    bus.profile_id = normalize_identifier_local(bus.profile_id, ...
        bus.architecture_id);

    if ~isfield(bus,'direct_share_enable') || isempty(bus.direct_share_enable)

        % regenerative callers. Stock ST3215 remains non-regenerative.
        bus.direct_share_enable = bus.regen_enable;
    end
    dse = bus.direct_share_enable;
    bus.direct_share_enable = logical(dse(1));
    if ~isfield(bus,'routing_mode') || isempty(bus.routing_mode)
        if bus.regen_enable
            bus.routing_mode = 'demand_limited_direct';
        else
            bus.routing_mode = 'local_dissipation';
        end
    end
    bus.routing_mode = normalize_routing_mode_local(bus.routing_mode);
    % Fill omitted capacities consistently BEFORE dynamic validation. A

    % contradictory values are left intact so the validator rejects them.
    missing_regen_capacity = 0.0;
    if bus.regen_enable, missing_regen_capacity = inf; end
    if ~isfield(bus,'Pregen_peak_max_W') || isempty(bus.Pregen_peak_max_W)
        bus.Pregen_peak_max_W = missing_regen_capacity;
    end
    if ~isfield(bus,'Pshare_peak_max_W') || isempty(bus.Pshare_peak_max_W)
        bus.Pshare_peak_max_W = missing_regen_capacity;
    end

    if ~isfield(bus,'V_nominal_V') || isempty(bus.V_nominal_V), bus.V_nominal_V = 12.0; end
    if ~isfield(bus,'motor_friction_smoothing_rad_s') || isempty(bus.motor_friction_smoothing_rad_s)
        bus.motor_friction_smoothing_rad_s = 0.05;
    end
    if ~isfield(bus,'V_bus_V') || isempty(bus.V_bus_V), bus.V_bus_V = 12.0; end
    if ~isfield(bus,'I_supply_max_A') || isempty(bus.I_supply_max_A), bus.I_supply_max_A = 5.0; end
    if ~isfield(bus,'servo_parallel') || isempty(bus.servo_parallel)
        if isfield(bus,'parallel') && ~isempty(bus.parallel)
            bus.servo_parallel = bus.parallel;
        else
            bus.servo_parallel = [1 2 1 1];
        end
    end
    % Keep historical algebraic fixtures explicit; formal dynamic defaults
    % use the published supply endpoint and independent 40-rpm execution cap.
    is_dynamic = strcmp(char(bus.model_scope),'dynamic_capacitor_common_dc_v1');
    default_i0=0.20; default_w0=40*2*pi/60;
    if is_dynamic, default_i0=0.18; default_w0=(pi/3)/0.222; end
    if ~isfield(bus,'I_noload_A') || isempty(bus.I_noload_A), bus.I_noload_A = default_i0; end
    if ~isfield(bus,'I_stall_A') || isempty(bus.I_stall_A), bus.I_stall_A = 2.70; end
    if ~isfield(bus,'tau_stall_single_Nm') || isempty(bus.tau_stall_single_Nm)
        bus.tau_stall_single_Nm = 30.0 * 9.80665 / 100.0;
    end
    if ~isfield(bus,'omega_noload_rad_s') || isempty(bus.omega_noload_rad_s)
        bus.omega_noload_rad_s = default_w0;
    end
    if ~isfield(bus,'motor_speed_contract') || isempty(bus.motor_speed_contract)
        if is_dynamic
            bus.motor_speed_contract='separate_endpoint_and_operating_speed_v2';
        else
            bus.motor_speed_contract='legacy_coupled_endpoint_and_operating_speed_v1';
        end
    end
    if ~isfield(bus,'omega_operating_max_rad_s') || isempty(bus.omega_operating_max_rad_s)
        if is_dynamic
            bus.omega_operating_max_rad_s=40*2*pi/60;
        else
            bus.omega_operating_max_rad_s=bus.omega_noload_rad_s;
        end
    end
    validate_motor_speed_contract_local(bus,is_dynamic);
    if ~isfield(bus,'tau_dead_Nm') || isempty(bus.tau_dead_Nm)
        % Equivalent output-side torque loss used by the ST3215 electrical
        % envelope.  A nonzero output request requires this additional
        % internal motor torque; zero requested torque remains zero.
        bus.tau_dead_Nm = 0.0;
    end

    is_dynamic = strcmp(char(bus.model_scope),'dynamic_capacitor_common_dc_v1');
    if is_dynamic

        
        dynamic_defaults = struct('C_dc_F',0.0047,'R_source_ohm',0.1, ...
            'V_source_V',12.0,'V_min_V',7.0,'V_max_V',12.6, ...
            'brake_model','none','dc_substeps',4, ...
            'dc_newton_iterations',12,'dc_solver_tolerance_W',1e-7, ...
            'V_init_V',12.0);
        names = fieldnames(dynamic_defaults);
        for k=1:numel(names)
            if ~isfield(bus,names{k}) || isempty(bus.(names{k}))
                bus.(names{k}) = dynamic_defaults.(names{k});
            end
        end
    end

    % ----- Source limits and shared-bus fields -----
    if ~isfield(bus,'P_total_max') || isempty(bus.P_total_max), bus.P_total_max = inf; end
    if ~isfield(bus,'P_grid_peak_max') || isempty(bus.P_grid_peak_max)
        if is_dynamic
            bus.P_grid_peak_max = double(bus.V_source_V) * double(bus.I_supply_max_A);
        elseif is_st3215
            bus.P_grid_peak_max = double(bus.V_bus_V) * double(bus.I_supply_max_A);
        else
            bus.P_grid_peak_max = bus.P_total_max;
        end
    end
    if ~isfield(bus,'P_avg_window_s') || isempty(bus.P_avg_window_s), bus.P_avg_window_s = 0.50; end
    if ~isfield(bus,'Pavg_init_mode') || isempty(bus.Pavg_init_mode), bus.Pavg_init_mode = 'static_hold'; end
    if ~isfield(bus,'P_avg_target_frac') || isempty(bus.P_avg_target_frac), bus.P_avg_target_frac = 1.00; end
    if ~isfield(bus,'P_avg_hard_enable') || isempty(bus.P_avg_hard_enable), bus.P_avg_hard_enable = false; end
    if ~isfield(bus,'eta_motoring') || isempty(bus.eta_motoring), bus.eta_motoring = 1.0; end
    if ~isfield(bus,'eta_regen')    || isempty(bus.eta_regen),    bus.eta_regen    = double(bus.regen_enable); end
    if ~isfield(bus,'eta_share')   || isempty(bus.eta_share),   bus.eta_share   = double(bus.regen_enable); end
    if ~isfield(bus,'P_idle_W')    || isempty(bus.P_idle_W)
        if is_st3215, bus.P_idle_W = 2.0; else, bus.P_idle_W = 0.0; end
    end

    if ~is_dynamic && (~isfield(bus,'P_brk_peak') || isempty(bus.P_brk_peak))
        if isfield(bus,'P_dump_max') && ~isempty(bus.P_dump_max)
            bus.P_brk_peak = bus.P_dump_max;
        else
            if bus.regen_enable && ~is_st3215
                bus.P_brk_peak = inf;
            else
                bus.P_brk_peak = 0.0;
            end
        end
    end

    % ----- Optional explicit bus-state model -----
    if ~isfield(bus,'enable_state') || isempty(bus.enable_state), bus.enable_state = is_dynamic; end
    if ~isfield(bus,'Ebuf_max_J')    || isempty(bus.Ebuf_max_J),    bus.Ebuf_max_J    = 0.0; end
    if ~isfield(bus,'Ebuf_min_J')    || isempty(bus.Ebuf_min_J),    bus.Ebuf_min_J    = 0.0; end
    if ~isfield(bus,'Ebuf_init_J')   || isempty(bus.Ebuf_init_J),   bus.Ebuf_init_J   = 0.0; end
    if ~isfield(bus,'Ereuse_init_J') || isempty(bus.Ereuse_init_J), bus.Ereuse_init_J = 0.0; end

    if ~isfield(bus,'Pbuf_chg_max_W') || isempty(bus.Pbuf_chg_max_W)
        if is_st3215, bus.Pbuf_chg_max_W = 0.0; else, bus.Pbuf_chg_max_W = inf; end
    end
    if ~isfield(bus,'Pbuf_dis_max_W') || isempty(bus.Pbuf_dis_max_W)
        if is_st3215, bus.Pbuf_dis_max_W = 0.0; else, bus.Pbuf_dis_max_W = inf; end
    end

    if ~isfield(bus,'eta_buf_chg') || isempty(bus.eta_buf_chg), bus.eta_buf_chg = 1.0; end
    if ~isfield(bus,'eta_buf_dis') || isempty(bus.eta_buf_dis), bus.eta_buf_dis = 1.0; end

    if ~is_dynamic
    if ~isfield(bus,'Tamb_C')          || isempty(bus.Tamb_C),          bus.Tamb_C          = 25.0; end
    if ~isfield(bus,'Tbrk_init_C')     || isempty(bus.Tbrk_init_C),     bus.Tbrk_init_C     = bus.Tamb_C; end
    if ~isfield(bus,'Tbrk_max_C')      || isempty(bus.Tbrk_max_C),      bus.Tbrk_max_C      = 120.0; end
    if ~isfield(bus,'Cth_brk_J_per_C') || isempty(bus.Cth_brk_J_per_C), bus.Cth_brk_J_per_C = 200.0; end
    if ~isfield(bus,'Rth_brk_C_per_W') || isempty(bus.Rth_brk_C_per_W)
        bus.Rth_brk_C_per_W = 1.0;
    end
    end
    if is_dynamic
        % Derived compatibility aliases: changing C or an endpoint updates
        % all capacitor energies. They are never independent buffer controls.
        bus.Ebuf_min_J=0.5*bus.C_dc_F*bus.V_min_V^2;
        bus.Ebuf_max_J=0.5*bus.C_dc_F*bus.V_max_V^2;
        bus.Ebuf_init_J=0.5*bus.C_dc_F*bus.V_init_V^2;
        if ~isfield(bus,'regen_injection_capacity_semantics') || ...
                isempty(bus.regen_injection_capacity_semantics)
            bus.regen_injection_capacity_semantics= ...
                'legacy_Pshare_field_caps_link_injection_v1';
        end
        % Never turn invalid supplied physical parameters into a plausible

        validate_strict_bus_local(bus);
        validate_dynamic_bus_local(bus);
    end

    % ----- Sanitize / clamp -----
    es = bus.enable_state;
    bus.enable_state = logical(es(1));

    if is_dynamic
        bus.V_bus_V = double(bus.V_bus_V); 
    else
        vv = double(bus.V_bus_V); bus.V_bus_V = max(vv(1), eps);
    end
    ii = double(bus.I_supply_max_A); bus.I_supply_max_A = max(ii(1), 0.0);
    bus.servo_parallel = row_finite(bus.servo_parallel, [1 2 1 1]);
    bus.servo_parallel = max(round(bus.servo_parallel), 1.0);
    bus.parallel = bus.servo_parallel; % accepted compatibility alias
    bus.I_noload_A = nonnegative_numeric(bus.I_noload_A, 0.20);
    bus.I_stall_A = nonnegative_numeric(bus.I_stall_A, 2.70);
    bus.tau_stall_single_Nm = positive_numeric(bus.tau_stall_single_Nm, 30.0*9.80665/100.0);
    bus.omega_noload_rad_s = positive_numeric(bus.omega_noload_rad_s,default_w0);
    bus.omega_operating_max_rad_s = positive_numeric( ...
        bus.omega_operating_max_rad_s,40*2*pi/60);
    bus.tau_dead_Nm = nonnegative_numeric(bus.tau_dead_Nm,0.0);
    bus.V_nominal_V = positive_numeric(bus.V_nominal_V,12.0);
    bus.motor_friction_smoothing_rad_s = positive_numeric( ...
        bus.motor_friction_smoothing_rad_s,0.05);
    if is_dynamic
        bus.P_source_rated_W = bus.V_source_V * bus.I_supply_max_A;
    else
        bus.P_source_rated_W = bus.V_bus_V * bus.I_supply_max_A;
    end

    bus.P_total_max = double(bus.P_total_max);
    bus.P_grid_peak_max = double(bus.P_grid_peak_max);
    if isfinite(bus.P_total_max) && isfinite(bus.P_grid_peak_max) && ...
            bus.P_total_max > bus.P_grid_peak_max + 1e-12
        error('spinn3d_bus_defaults:BudgetExceedsPhysicalPeak', ...
            ['Filtered budget %.9g W exceeds the physical source peak ', ...
             '%.9g W. Lower P_total_max or explicitly identify a larger supply.'], ...
            bus.P_total_max,bus.P_grid_peak_max);
    end
    bus.P_avg_window_s = max(double(bus.P_avg_window_s), 1e-6);
    bus.Pavg_init_mode = normalize_identifier_local( ...
        bus.Pavg_init_mode,'static_hold');
    bus.P_avg_target_frac = min(max(double(bus.P_avg_target_frac), 0.0), 1.5);
    bus.P_avg_hard_enable = logical(bus.P_avg_hard_enable);
    bus.eta_motoring = max(min(double(bus.eta_motoring), 1.0), eps);
    bus.eta_regen    = clamp01(double(bus.eta_regen));
    bus.eta_share   = clamp01(double(bus.eta_share));
    pr_cap = double(bus.Pregen_peak_max_W);
    ps_cap = double(bus.Pshare_peak_max_W);
    if isempty(pr_cap) || isnan(pr_cap(1)), pr_cap = 0.0; end
    if isempty(ps_cap) || isnan(ps_cap(1)), ps_cap = 0.0; end
    bus.Pregen_peak_max_W = max(pr_cap(1),0.0);
    bus.Pshare_peak_max_W = max(ps_cap(1),0.0);
    if ~bus.regen_enable
        bus.eta_regen = 0.0;
        bus.eta_share = 0.0;
        bus.direct_share_enable = false;
        bus.Pregen_peak_max_W = 0.0;
        bus.Pshare_peak_max_W = 0.0;
    end
    % Routing identity and the independent enable switch must agree. In

    % were already rejected by validate_strict_bus_local instead.
    if strcmp(bus.routing_mode,'local_dissipation') || ...
            strcmp(bus.routing_mode,'regen_curtailed')
        bus.direct_share_enable = false;
    elseif ~bus.regen_enable
        bus.routing_mode = 'local_dissipation';
    end
    bus.P_idle_W    = max(double(bus.P_idle_W), 0.0);
    if ~is_dynamic, bus.P_brk_peak=max(double(bus.P_brk_peak),0.0); end

    bus.Ebuf_max_J    = max(double(bus.Ebuf_max_J), 0.0);
    bus.Ebuf_min_J    = min(max(double(bus.Ebuf_min_J), 0.0), bus.Ebuf_max_J);
    bus.Ebuf_init_J   = min(max(double(bus.Ebuf_init_J), bus.Ebuf_min_J), bus.Ebuf_max_J);
    bus.Ereuse_init_J = min(max(double(bus.Ereuse_init_J), 0.0), bus.Ebuf_init_J);

    bus.Pbuf_chg_max_W = max(double(bus.Pbuf_chg_max_W), 0.0);
    bus.Pbuf_dis_max_W = max(double(bus.Pbuf_dis_max_W), 0.0);

    bus.eta_buf_chg = max(min(double(bus.eta_buf_chg),1.0), eps);
    bus.eta_buf_dis = max(min(double(bus.eta_buf_dis),1.0), eps);

    if ~is_dynamic
        bus.Tamb_C=double(bus.Tamb_C);
        bus.Tbrk_max_C=max(double(bus.Tbrk_max_C),bus.Tamb_C);
        bus.Tbrk_init_C=min(max(double(bus.Tbrk_init_C),bus.Tamb_C),bus.Tbrk_max_C);
        bus.Cth_brk_J_per_C=max(double(bus.Cth_brk_J_per_C),eps);
        bus.Rth_brk_C_per_W=max(double(bus.Rth_brk_C_per_W),eps);
    end
    validate_model_scope_local(bus);

    if ~is_dynamic, bus.P_dump_max=bus.P_brk_peak; end
    % Internal marker lets the motor helper avoid repeating this sanitation
    % inside every 1-ms BUS step and every nonlinear-clamp bisection trial.
    bus.spinn3d_defaults_applied = true;
end

function y = clamp01(x)
    y = min(max(x, 0.0), 1.0);
end

function s = normalize_model(x)
    try
        s = lower(strtrim(char(x)));
    catch
        s = 'legacy';
    end
    if isempty(s), s = 'legacy'; end
    if any(strcmp(s,{'st3215','roarm','roarm_m2_s','roarm-m2-s'}))
        s = 'st3215_pwm';
    elseif ~strcmp(s,'st3215_pwm')
        s = 'legacy';
    end
end

function s = normalize_identifier_local(x,def)
    try
        s = lower(strtrim(char(x)));
    catch
        s = def;
    end
    if isempty(s), s = def; end
    s = regexprep(s,'[^a-z0-9_\-]','_');
end

function s = normalize_routing_mode_local(x)
    try
        s = lower(strtrim(char(x)));
    catch
        s = '';
    end
    if any(strcmp(s,{'stock','local','nonregenerative'}))
        s = 'local_dissipation';
    elseif any(strcmp(s,{'noshare','no_share','curtailed'}))
        s = 'regen_curtailed';
    elseif any(strcmp(s,{'direct','direct_share','legacy'}))
        s = 'demand_limited_direct';
    end
    if ~any(strcmp(s,{'local_dissipation','regen_curtailed', ...
            'demand_limited_direct'}))
        s = 'local_dissipation';
    end
end

function s = normalize_quadrant_model_local(x,is_st3215,regen_enable) %#ok<INUSD>
    s = lower(strtrim(char(x)));
    if is_st3215
        expected = 'signed_terminal_power_v1';
    else
        expected = 'legacy_mechanical_efficiency_v1';
    end

end

function s = normalize_actuator_envelope_local(x,is_st3215,regen_enable) %#ok<INUSD>
    s = lower(strtrim(char(x)));
    if is_st3215
        expected = 'passive_dc_current_voltage_speed_v1';
    else
        expected = 'legacy_no_envelope_v1';
    end

end

function validate_st3215_profile_coherence_local(bus)

end

function validate_model_scope_local(bus)
    scope=char(bus.model_scope);

    if strcmp(scope,'dynamic_capacitor_common_dc_v1')
        validate_dynamic_bus_local(bus);
    elseif strcmp(scope,'regulated_algebraic_common_dc_v1')

        validate_st3215_profile_coherence_local(bus);

        f={'Ebuf_max_J','Ebuf_min_J','Ebuf_init_J','Ereuse_init_J', ...
            'Pbuf_chg_max_W','Pbuf_dis_max_W','P_brk_peak'};
        for k=1:numel(f)

        end

    elseif strcmp(scope,'legacy_storage_diagnostic_v1')

    else
        error('spinn3d_bus_defaults:UnknownModelScope', ...
            'Unknown BUS model scope: %s.',scope);
    end
end

function v = row_finite(v, def)
    try, v = double(v(:).'); catch, v = def; end
    if isempty(v), v = def; end
    bad = ~isfinite(v);
    if any(bad)
        d = double(def(:).');
        if isscalar(d), d = repmat(d,1,numel(v)); end
        if numel(d) < numel(v), d = [d repmat(d(end),1,numel(v)-numel(d))]; end
        v(bad) = d(find(bad)); %#ok<FNDSB>
    end
end

function v = nonnegative_numeric(v, def)
    try, v = double(v); catch, v = def; end
    v(~isfinite(v)) = def;
    v = max(v,0.0);
end

function v = positive_numeric(v, def)
    v = nonnegative_numeric(v,def);
    v(v<=0) = def;
end

function validate_strict_bus_local(bus)
is_dynamic=isfield(bus,'model_scope') && strcmp(char(bus.model_scope),'dynamic_capacitor_common_dc_v1');
required = {'model','model_scope','auxiliary_topology','motor_power_schema', ...
    'V_nominal_V','motor_friction_smoothing_rad_s', ...
    'quadrant_power_model','actuator_envelope_model', ...
    'architecture_id','profile_id','regen_enable', ...
    'direct_share_enable','routing_mode','Pregen_peak_max_W', ...
    'Pshare_peak_max_W','V_bus_V','I_supply_max_A', ...
    'servo_parallel','I_noload_A','I_stall_A', ...
    'tau_stall_single_Nm','omega_noload_rad_s','tau_dead_Nm', ...
    'P_total_max','P_grid_peak_max','P_avg_window_s','Pavg_init_mode', ...
    'P_avg_target_frac','P_avg_hard_enable','eta_motoring', ...
    'eta_regen','eta_share','P_idle_W','P_brk_peak', ...
    'enable_state','Ebuf_max_J','Ebuf_min_J','Ebuf_init_J', ...
    'Ereuse_init_J','Pbuf_chg_max_W','Pbuf_dis_max_W', ...
    'eta_buf_chg','eta_buf_dis','Tamb_C','Tbrk_init_C', ...
    'Tbrk_max_C','Cth_brk_J_per_C','Rth_brk_C_per_W'};
if is_dynamic
    required=setdiff(required,{'P_brk_peak','Tamb_C','Tbrk_init_C','Tbrk_max_C','Cth_brk_J_per_C','Rth_brk_C_per_W'},'stable');
    required = [required dynamic_fields_local() ...
        {'brake_model','regen_injection_capacity_semantics','motor_speed_contract', ...
        'omega_operating_max_rad_s'}];
end
missing = required(~isfield(bus,required));
if ~isempty(missing)
    error('spinn3d_bus_defaults:MissingCentralBusField', ...
        ['Strict central BUS config is incomplete. Missing: %s. ', ...
         'Define it in spinn3d_params_block.m.'],strjoin(missing,', '));
end

try
    model = lower(strtrim(char(bus.model)));
catch
    model = '';
end

quadrant_model = lower(strtrim(char(bus.quadrant_power_model)));
envelope_model = lower(strtrim(char(bus.actuator_envelope_model)));
validate_logical_scalar_local(bus.regen_enable,'regen_enable');
validate_logical_scalar_local(bus.direct_share_enable,'direct_share_enable');
validate_logical_scalar_local(bus.P_avg_hard_enable,'P_avg_hard_enable');
validate_logical_scalar_local(bus.enable_state,'enable_state');
if strcmp(model,'legacy')

else
    validate_st3215_profile_coherence_local(bus);
end
validate_nonempty_identifier_local(bus.model_scope,'model_scope');
validate_nonempty_identifier_local(bus.auxiliary_topology,'auxiliary_topology');
validate_nonempty_identifier_local(bus.motor_power_schema,'motor_power_schema');
validate_finite_scalar_local(bus.V_nominal_V,'V_nominal_V');
validate_positive_finite_local(bus.V_nominal_V,'V_nominal_V');
validate_positive_finite_local(bus.motor_friction_smoothing_rad_s, ...
    'motor_friction_smoothing_rad_s');

validate_finite_scalar_local(bus.V_bus_V,'V_bus_V');
validate_positive_finite_local(bus.V_bus_V,'V_bus_V');
validate_finite_scalar_local(bus.I_supply_max_A,'I_supply_max_A');
validate_nonnegative_finite_local(bus.I_supply_max_A, ...
    'I_supply_max_A');
sp = double(bus.servo_parallel(:).');

validate_nonnegative_finite_local(bus.I_noload_A,'I_noload_A');
validate_nonnegative_finite_local(bus.I_stall_A,'I_stall_A');
I0 = double(bus.I_noload_A(:));
Ist = double(bus.I_stall_A(:));
nI = max(numel(I0),numel(Ist));
if isscalar(I0), I0 = repmat(I0,nI,1); end
if isscalar(Ist), Ist = repmat(Ist,nI,1); end

validate_positive_finite_local(bus.tau_stall_single_Nm, ...
    'tau_stall_single_Nm');
validate_positive_finite_local(bus.omega_noload_rad_s, ...
    'omega_noload_rad_s');
if isfield(bus,'omega_operating_max_rad_s')
    validate_motor_speed_contract_local(bus, ...
        strcmp(char(bus.model_scope),'dynamic_capacitor_common_dc_v1'));
end
validate_nonnegative_finite_local(bus.tau_dead_Nm,'tau_dead_Nm');
axis_fields={'I_noload_A','I_stall_A','tau_stall_single_Nm', ...
    'omega_noload_rad_s','tau_dead_Nm','motor_friction_smoothing_rad_s'};
if isfield(bus,'omega_operating_max_rad_s')
    axis_fields=[axis_fields {'omega_operating_max_rad_s'}];
end
for k=1:numel(axis_fields)
    n=numel(bus.(axis_fields{k}));

end

validate_nonnegative_allow_inf_scalar_local(bus.P_total_max, ...
    'P_total_max');
validate_nonnegative_allow_inf_scalar_local(bus.P_grid_peak_max, ...
    'P_grid_peak_max');
validate_positive_finite_local(bus.P_avg_window_s, ...
    'P_avg_window_s');
validate_nonempty_identifier_local(bus.Pavg_init_mode,'Pavg_init_mode');
validate_bounded_scalar_local(bus.P_avg_target_frac,0,1.5, ...
    'P_avg_target_frac');
validate_bounded_scalar_local(bus.eta_motoring,eps,1, ...
    'eta_motoring');
validate_bounded_scalar_local(bus.eta_regen,0,1,'eta_regen');
validate_bounded_scalar_local(bus.eta_share,0,1,'eta_share');
validate_nonnegative_allow_inf_scalar_local(bus.Pregen_peak_max_W, ...
    'Pregen_peak_max_W');
validate_nonnegative_allow_inf_scalar_local(bus.Pshare_peak_max_W, ...
    'Pshare_peak_max_W');
validate_nonnegative_finite_scalar_local(bus.P_idle_W,'P_idle_W');
if ~is_dynamic
    validate_nonnegative_allow_inf_scalar_local(bus.P_brk_peak,'P_brk_peak');
end
if ~logical(bus.regen_enable)

    

end

validate_nonempty_identifier_local(bus.architecture_id,'architecture_id');
validate_nonempty_identifier_local(bus.profile_id,'profile_id');
routing_mode = lower(strtrim(char(bus.routing_mode)));

if ~logical(bus.regen_enable)

end
if any(strcmp(routing_mode,{'local_dissipation','regen_curtailed'}))

end
if strcmp(routing_mode,'local_dissipation')

end
if strcmp(routing_mode,'regen_curtailed')

end
if logical(bus.direct_share_enable)

end

energy_fields = {'Ebuf_max_J','Ebuf_min_J','Ebuf_init_J', ...
    'Ereuse_init_J'};
for i = 1:numel(energy_fields)
    validate_nonnegative_finite_scalar_local( ...
        bus.(energy_fields{i}),energy_fields{i});
end

validate_nonnegative_allow_inf_scalar_local(bus.Pbuf_chg_max_W, ...
    'Pbuf_chg_max_W');
validate_nonnegative_allow_inf_scalar_local(bus.Pbuf_dis_max_W, ...
    'Pbuf_dis_max_W');
validate_bounded_scalar_local(bus.eta_buf_chg,eps,1,'eta_buf_chg');
validate_bounded_scalar_local(bus.eta_buf_dis,eps,1,'eta_buf_dis');

if ~is_dynamic
thermal_fields = {'Tamb_C','Tbrk_init_C','Tbrk_max_C'};
for i = 1:numel(thermal_fields)
    validate_finite_scalar_local(bus.(thermal_fields{i}), ...
        thermal_fields{i});
end

validate_positive_finite_local(bus.Cth_brk_J_per_C, ...
    'Cth_brk_J_per_C');
validate_positive_finite_local(bus.Rth_brk_C_per_W, ...
    'Rth_brk_C_per_W');
end
end

function validate_nonempty_identifier_local(v,name)
try
    s = strtrim(char(v));
catch
    s = '';
end

end

function validate_logical_scalar_local(v,name)

end

function validate_finite_scalar_local(v,name)

end

function validate_nonnegative_finite_scalar_local(v,name)
validate_finite_scalar_local(v,name);

end

function validate_nonnegative_finite_local(v,name)

end

function validate_positive_finite_local(v,name)

end

function validate_nonnegative_allow_inf_scalar_local(v,name)

end

function validate_bounded_scalar_local(v,lo,hi,name)
validate_finite_scalar_local(v,name);

end

function names = dynamic_fields_local()
names = {'C_dc_F','R_source_ohm','V_source_V','V_min_V','V_max_V', ...
    'dc_substeps','dc_newton_iterations','dc_solver_tolerance_W','V_init_V'};
end

function validate_dynamic_bus_local(bus)

validate_st3215_profile_coherence_local(bus);

names=dynamic_fields_local();
for k=1:numel(names)
    validate_finite_scalar_local(bus.(names{k}),names{k});
    validate_positive_finite_local(bus.(names{k}),names{k});
end

forbidden={'V_chop_on_V','chopper_gain_W_per_V','R_brake_ohm', ...
    'P_brk_peak','P_dump_max','Tamb_C','Tbrk_init_C','Tbrk_max_C', ...
    'Cth_brk_J_per_C','Rth_brk_C_per_W'};

% Energy aliases are derived from capacitor C and voltage endpoints only;
% the live state is Ecap=0.5*C_dc_F*Vdc^2 with no energy clipping operation.

end

function validate_motor_speed_contract_local(bus,is_dynamic)

validate_positive_finite_local(bus.omega_noload_rad_s,'omega_noload_rad_s');
validate_positive_finite_local(bus.omega_operating_max_rad_s,'omega_operating_max_rad_s');
separate='separate_endpoint_and_operating_speed_v2';
legacy='legacy_coupled_endpoint_and_operating_speed_v1';

contract=char(bus.motor_speed_contract);

if strcmp(contract,legacy)
    w0=double(bus.omega_noload_rad_s(:));
    wm=double(bus.omega_operating_max_rad_s(:));

end
end
