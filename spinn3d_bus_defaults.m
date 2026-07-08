function bus = spinn3d_bus_defaults(bus)
%SPINN3D_BUS_DEFAULTS  Defaults for the revised DC-bus model.
%
% Preferred naming:
%   bus.P_grid_budget   active controller/evaluation grid-input budget [W]
%   bus.P_grid_peak_max instantaneous hardware/rating cap [W]
%   bus.P_hw_rating     nominal supply rating V_nom * I_rating [W]
%   bus.P_total_max     legacy alias synchronized to P_grid_budget
%
% The optional finite-buffer / brake-thermal state is disabled by default so
% old simulations retain the original algebraic common-bus behavior unless
% bus.enable_state is explicitly set to true.

if nargin < 1 || isempty(bus), bus = struct(); end

% Nominal supply / rating
if ~isfield(bus,'V_nom') || isempty(bus.V_nom), bus.V_nom = 12.0; end
if ~isfield(bus,'V_oc')  || isempty(bus.V_oc),  bus.V_oc  = bus.V_nom; end
if ~isfield(bus,'I_rating') || isempty(bus.I_rating), bus.I_rating = inf; end
if ~isfield(bus,'P_hw_rating') || isempty(bus.P_hw_rating)
    if isfield(bus,'P_supply_rating') && ~isempty(bus.P_supply_rating)
        bus.P_hw_rating = bus.P_supply_rating;
    else
        bus.P_hw_rating = bus.V_nom * bus.I_rating;
    end
end

% Active budget.  P_total_max is kept as a legacy alias only.
if ~isfield(bus,'P_grid_budget') || isempty(bus.P_grid_budget)
    if isfield(bus,'P_total_max') && ~isempty(bus.P_total_max)
        bus.P_grid_budget = bus.P_total_max;
    else
        bus.P_grid_budget = inf;
    end
end
bus.P_total_max = bus.P_grid_budget;

% Instantaneous hardware/raw cap.
if ~isfield(bus,'P_grid_peak_max') || isempty(bus.P_grid_peak_max)
    if isfinite(bus.P_hw_rating)
        bus.P_grid_peak_max = bus.P_hw_rating;
    else
        bus.P_grid_peak_max = bus.P_grid_budget;
    end
end
if isfinite(bus.P_grid_budget)
    bus.P_grid_peak_max = max(double(bus.P_grid_peak_max), double(bus.P_grid_budget));
end

% Electrical conversion / source model.
if ~isfield(bus,'model') || isempty(bus.model), bus.model = 'electrical'; end
if ~isfield(bus,'use_electrical_model') || isempty(bus.use_electrical_model)
    bus.use_electrical_model = any(strcmpi(char(bus.model), {'electrical','elec','dc'}));
end
if ~isfield(bus,'eta_share')  || isempty(bus.eta_share),  bus.eta_share  = 0.90; end
if ~isfield(bus,'eta_mot')    || isempty(bus.eta_mot),    bus.eta_mot    = 0.85; end
if ~isfield(bus,'eta_regen')  || isempty(bus.eta_regen),  bus.eta_regen  = 0.70; end
if ~isfield(bus,'R_batt')     || isempty(bus.R_batt),     bus.R_batt     = 0.0; end
if ~isfield(bus,'Cdc')        || isempty(bus.Cdc),        bus.Cdc        = inf; end
if ~isfield(bus,'V_min')      || isempty(bus.V_min),      bus.V_min      = 0.0; end
if ~isfield(bus,'V_max')      || isempty(bus.V_max),      bus.V_max      = inf; end
if ~isfield(bus,'V_chop')     || isempty(bus.V_chop),     bus.V_chop     = bus.V_max; end
if ~isfield(bus,'P_brk_peak') || isempty(bus.P_brk_peak)
    if isfield(bus,'P_dump_max') && ~isempty(bus.P_dump_max)
        bus.P_brk_peak = bus.P_dump_max;
    else
        bus.P_brk_peak = inf;
    end
end
if ~isfield(bus,'Kt')      || isempty(bus.Kt),      bus.Kt      = 0.60; end  % output-equivalent default [N*m/A]
if ~isfield(bus,'R_phase') || isempty(bus.R_phase), bus.R_phase = 0.12; end  % output-equivalent default [ohm]
if ~isfield(bus,'P_inv0')  || isempty(bus.P_inv0),  bus.P_inv0  = 0.0; end
if ~isfield(bus,'k_inv')   || isempty(bus.k_inv),   bus.k_inv   = 0.0; end
% Optional supply-current/no-load model for smart serial bus servos.
if ~isfield(bus,'enable_servo_aux_loss') || isempty(bus.enable_servo_aux_loss), bus.enable_servo_aux_loss = false; end
if ~isfield(bus,'n_servo_axis') || isempty(bus.n_servo_axis), bus.n_servo_axis = ones(1,4); end
if ~isfield(bus,'I_servo_standby_A') || isempty(bus.I_servo_standby_A), bus.I_servo_standby_A = 0.03; end
if ~isfield(bus,'I_servo_no_load_A') || isempty(bus.I_servo_no_load_A), bus.I_servo_no_load_A = 0.20; end
if ~isfield(bus,'omega_no_load_servo_rad_s') || isempty(bus.omega_no_load_servo_rad_s), bus.omega_no_load_servo_rad_s = 4.72; end
if ~isfield(bus,'servo_aux_speed_exp') || isempty(bus.servo_aux_speed_exp), bus.servo_aux_speed_exp = 0.50; end
if ~isfield(bus,'P_control_board_W') || isempty(bus.P_control_board_W), bus.P_control_board_W = 0.0; end
% Optional physical source limiter: active budget and hardware rating are separate.
if ~isfield(bus,'enable_supply_current_limit') || isempty(bus.enable_supply_current_limit), bus.enable_supply_current_limit = false; end
if ~isfield(bus,'I_source_max') || isempty(bus.I_source_max), bus.I_source_max = bus.I_rating; end
if ~isfield(bus,'P_source_max_W') || isempty(bus.P_source_max_W), bus.P_source_max_W = bus.P_hw_rating; end
if ~isfield(bus,'I_budget') || isempty(bus.I_budget)
    bus.I_budget = bus.P_grid_budget / max(bus.V_nom, eps);
end

% Optional finite energy-buffer and brake thermal diagnostics.
if ~isfield(bus,'enable_state') || isempty(bus.enable_state), bus.enable_state = false; end
if ~isfield(bus,'Ebuf_min_J') || isempty(bus.Ebuf_min_J), bus.Ebuf_min_J = 0.0; end
if ~isfield(bus,'Ebuf_max_J') || isempty(bus.Ebuf_max_J), bus.Ebuf_max_J = 0.0; end
if ~isfield(bus,'Ebuf_init_J') || isempty(bus.Ebuf_init_J), bus.Ebuf_init_J = bus.Ebuf_min_J; end
if ~isfield(bus,'Ereuse_init_J') || isempty(bus.Ereuse_init_J), bus.Ereuse_init_J = 0.0; end
if ~isfield(bus,'Pbuf_dis_max_W') || isempty(bus.Pbuf_dis_max_W), bus.Pbuf_dis_max_W = inf; end
if ~isfield(bus,'Pbuf_chg_max_W') || isempty(bus.Pbuf_chg_max_W), bus.Pbuf_chg_max_W = inf; end
if ~isfield(bus,'eta_buf_dis') || isempty(bus.eta_buf_dis), bus.eta_buf_dis = 0.95; end
if ~isfield(bus,'eta_buf_chg') || isempty(bus.eta_buf_chg), bus.eta_buf_chg = 0.95; end
if ~isfield(bus,'Tamb_C') || isempty(bus.Tamb_C), bus.Tamb_C = 25.0; end
if ~isfield(bus,'Tbrk_init_C') || isempty(bus.Tbrk_init_C), bus.Tbrk_init_C = bus.Tamb_C; end
if ~isfield(bus,'Tbrk_max_C') || isempty(bus.Tbrk_max_C), bus.Tbrk_max_C = 120.0; end
if ~isfield(bus,'Rth_brk_C_per_W') || isempty(bus.Rth_brk_C_per_W), bus.Rth_brk_C_per_W = 8.0; end
if ~isfield(bus,'Cth_brk_J_per_C') || isempty(bus.Cth_brk_J_per_C), bus.Cth_brk_J_per_C = 60.0; end

% Normalize scalar fields.
bus.V_nom = double(bus.V_nom);
bus.V_oc  = double(bus.V_oc);
bus.I_rating = double(bus.I_rating);
bus.P_hw_rating = double(bus.P_hw_rating);
bus.P_supply_rating = bus.P_hw_rating;
bus.P_grid_budget = double(bus.P_grid_budget);
bus.P_total_max = double(bus.P_total_max);
bus.P_grid_peak_max = double(bus.P_grid_peak_max);
bus.use_electrical_model = logical(bus.use_electrical_model);
bus.eta_share = min(max(double(bus.eta_share),0),1);
bus.eta_mot   = min(max(double(bus.eta_mot),eps),1);
bus.eta_regen = min(max(double(bus.eta_regen),0),1);
bus.R_batt = max(double(bus.R_batt),0);
bus.Cdc    = max(double(bus.Cdc),eps);
bus.V_min  = double(bus.V_min);
bus.V_max  = double(bus.V_max);
if bus.V_max < bus.V_min, bus.V_max = bus.V_min; end
bus.V_chop = double(bus.V_chop);
bus.P_brk_peak = max(double(bus.P_brk_peak),0);
bus.P_dump_max = bus.P_brk_peak;
bus.P_inv0 = max(double(bus.P_inv0),0);
bus.k_inv  = max(double(bus.k_inv),0);
bus.enable_servo_aux_loss = logical(bus.enable_servo_aux_loss);
bus.n_servo_axis = double(bus.n_servo_axis(:).');
bus.I_servo_standby_A = max(double(bus.I_servo_standby_A),0);
bus.I_servo_no_load_A = max(double(bus.I_servo_no_load_A),bus.I_servo_standby_A);
bus.omega_no_load_servo_rad_s = max(double(bus.omega_no_load_servo_rad_s),eps);
bus.servo_aux_speed_exp = max(double(bus.servo_aux_speed_exp),eps);
bus.P_control_board_W = max(double(bus.P_control_board_W),0);
bus.enable_supply_current_limit = logical(bus.enable_supply_current_limit);
bus.I_source_max = max(double(bus.I_source_max),0);
bus.P_source_max_W = max(double(bus.P_source_max_W),0);

bus.enable_state = logical(bus.enable_state);
bus.Ebuf_min_J = max(double(bus.Ebuf_min_J),0);
bus.Ebuf_max_J = max(double(bus.Ebuf_max_J),bus.Ebuf_min_J);
bus.Ebuf_init_J = min(max(double(bus.Ebuf_init_J),bus.Ebuf_min_J),bus.Ebuf_max_J);
bus.Ereuse_init_J = min(max(double(bus.Ereuse_init_J),0),bus.Ebuf_init_J);
bus.Pbuf_dis_max_W = max(double(bus.Pbuf_dis_max_W),0);
bus.Pbuf_chg_max_W = max(double(bus.Pbuf_chg_max_W),0);
bus.eta_buf_dis = min(max(double(bus.eta_buf_dis),eps),1);
bus.eta_buf_chg = min(max(double(bus.eta_buf_chg),eps),1);
bus.Tamb_C = double(bus.Tamb_C);
bus.Tbrk_init_C = max(double(bus.Tbrk_init_C),bus.Tamb_C);
bus.Tbrk_max_C = max(double(bus.Tbrk_max_C),bus.Tamb_C);
bus.Rth_brk_C_per_W = max(double(bus.Rth_brk_C_per_W),eps);
bus.Cth_brk_J_per_C = max(double(bus.Cth_brk_J_per_C),eps);
end
