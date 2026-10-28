function c = spinn3d_motor_dc_constants(bus,nJ)
%SPINN3D_MOTOR_DC_CONSTANTS Passive, output-equivalent nominal DC fit.
% Constants are fitted at V_nominal_V, NEVER at a perturbed V_bus_V.
% I_noload_A is a SUPPLY-current endpoint at omega_noload_rad_s, the
% declared fitting reference. omega_operating_max_rad_s independently limits
% executable speed. The inferred partial-PWM fit is a nominal equivalent
% retrofit model, not stock-servo winding/firmware calibration.

if nargin<2 || ~isscalar(nJ) || ~isfinite(nJ) || nJ<1 || nJ~=round(nJ)
    error('spinn3d_motor_dc_constants:BadJointCount', ...
        'nJ must be one positive integer.');
end
if nargin<1 || ~isstruct(bus)
    error('spinn3d_motor_dc_constants:BadBus','bus must be a structure.');
end
% Runtime callers may carry a defaults-applied marker. That marker cannot
% authorize a stale dynamic contract with a missing execution-speed limit.
if isfield(bus,'model_scope') && ...
        strcmp(char(bus.model_scope),'dynamic_capacitor_common_dc_v1')

end
c = struct();
c.schema = 'passive_output_dc_pwm_v1';
c.n_parallel = parameter(bus,'servo_parallel',[1 2 1 1],nJ);
c.V_nominal_V = parameter(bus,'V_nominal_V',12,nJ);
c.V_bus_V = parameter(bus,'V_bus_V',12,nJ);
c.I_stall_A = parameter(bus,'I_stall_A',2.7,nJ);
c.I_noload_supply_A = parameter(bus,'I_noload_A',0.18,nJ);
c.tau_stall_single_Nm = parameter(bus,'tau_stall_single_Nm',2.941995,nJ);
c.omega_fit_rad_s = parameter(bus,'omega_noload_rad_s',(pi/3)/0.222,nJ);
c.omega_limit_rad_s = parameter(bus,'omega_operating_max_rad_s', ...
    c.omega_fit_rad_s,nJ);
c.speed_contract = 'legacy_coupled_endpoint_and_operating_speed_v1';
if isfield(bus,'motor_speed_contract')
    c.speed_contract = char(bus.motor_speed_contract);
end
c.omega_smooth_rad_s = parameter(bus,'motor_friction_smoothing_rad_s',0.05,nJ);
tau_c_joint = parameter(bus,'tau_dead_Nm',0,nJ);
positive = [c.n_parallel,c.V_nominal_V,c.V_bus_V,c.I_stall_A, ...
    c.tau_stall_single_Nm,c.omega_fit_rad_s,c.omega_limit_rad_s,c.omega_smooth_rad_s];
if any(positive<=0) || any(c.n_parallel~=round(c.n_parallel)) || ...
        any(c.I_noload_supply_A<0) || ...
        any(c.I_noload_supply_A>=c.I_stall_A) || any(tau_c_joint<0)
    error('spinn3d_motor_dc_constants:NonphysicalParameter', ...
        'Positive nominal constants, integer parallel counts and 0<=I0<Istall are required.');
end

% The blocked full-duty endpoint determines R and output-equivalent K.
% Kt=Ke in SI units is essential for a passive energy balance.
c.K_Nm_A = c.tau_stall_single_Nm./c.I_stall_A;
c.R_ohm = c.V_nominal_V./c.I_stall_A;
emf0 = c.K_Nm_A.*c.omega_fit_rad_s;
P0 = c.V_nominal_V.*c.I_noload_supply_A;
% Stable positive root of R*i0^2 + K*omega0*i0 = Vnom*I0_supply.
c.I_noload_winding_A = 2.*P0 ./ ...
    (sqrt(emf0.^2+4.*c.R_ohm.*P0)+emf0);
c.coulomb_Nm_per_servo = tau_c_joint./c.n_parallel;
drag0 = c.K_Nm_A.*c.I_noload_winding_A;
c.b_Nm_s = (drag0 - c.coulomb_Nm_per_servo.* ...
    tanh(c.omega_fit_rad_s./c.omega_smooth_rad_s))./ ...
    c.omega_fit_rad_s;
if any(c.b_Nm_s<0)
    error('spinn3d_motor_dc_constants:NegativeDragFit', ...
        'tau_dead_Nm exceeds the available no-load drag; fit would be non-passive.');
end
c.no_load_duty = (c.R_ohm.*c.I_noload_winding_A+emf0)./c.V_nominal_V;
if any(c.no_load_duty>1+1e-12)
    error('spinn3d_motor_dc_constants:InfeasibleNoLoadFit', ...
        'Nominal endpoints cannot fit inside the available bridge voltage.');
end
c.parameter_status = 'nominal_passive_retrofit_equivalent_not_stock_calibrated';
end

function v = parameter(S,fn,def,n)
if isfield(S,fn) && ~isempty(S.(fn)), v=S.(fn); else, v=def; end
if ~isnumeric(v) || ~isreal(v)
    error('spinn3d_motor_dc_constants:BadParameter','%s must be real numeric.',fn);
end
v=double(v(:).');
if isscalar(v), v=repmat(v,1,n); end
if numel(v)<n, v=[v repmat(v(end),1,n-numel(v))]; end
v=v(1:n);
if any(~isfinite(v))
    error('spinn3d_motor_dc_constants:BadParameter','%s must be finite.',fn);
end
end
