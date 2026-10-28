function env = spinn3d_motor_envelope(tau,dq,bus,nJ_expected)
% Calculate the motor torque-speed limits at the specified bus voltage.

if nargin<3 || isempty(bus) || ~isstruct(bus), bus=struct(); end
if nargin<4, nJ_expected=[]; end
if ~isfield(bus,'spinn3d_defaults_applied') || ...
        isempty(bus.spinn3d_defaults_applied) || ...
        ~logical(bus.spinn3d_defaults_applied(1))
    bus=spinn3d_bus_defaults(bus);
end
tau=shaped(tau,nJ_expected); dq=shaped(dq,nJ_expected);
if ~isequal(size(tau),size(dq))
    error('spinn3d_motor_envelope:SizeMismatch', ...
        'tau and dq must have identical vector or N-by-nJ shape.');
end
nJ=size(tau,2);
if ~isempty(nJ_expected) && nJ~=nJ_expected
    error('spinn3d_motor_envelope:JointCountMismatch', ...
        'The input does not have the declared nJ_expected columns.');
end
env=struct();
env.actuator_envelope_model=char(field(bus,'actuator_envelope_model', ...
    'legacy_no_envelope_v1'));
if ~strcmpi(char(field(bus,'model','legacy')),'st3215_pwm')
    env.tau_internal_signed_Nm=tau;
    env.r_current=zeros(size(tau)); env.r_voltage=zeros(size(tau));
    env.voltage_command_ratio_signed=zeros(size(tau));
    env.r_tau=zeros(size(tau)); env.r_speed=zeros(size(tau));
    env.torque_speed_ratio=zeros(size(tau)); env.duty=zeros(size(tau));
    env.tau_envelope_min_Nm=-inf(size(tau));
    env.tau_envelope_max_Nm=inf(size(tau));
    env.tau_speed_cap=inf(size(tau));
    env.envelope_axis_ok=true(size(tau)); env.envelope_ok=true;
    return;
end
if ~strcmp(env.actuator_envelope_model,'passive_dc_current_voltage_speed_v1') || ...
        ~strcmp(char(field(bus,'quadrant_power_model','')), ...
        'signed_terminal_power_v1')
    error('spinn3d_motor_envelope:EnvelopeProfileMismatch', ...
        'MathSource97 requires the passive DC envelope and signed-terminal-power profile.');
end
c=spinn3d_motor_dc_constants(bus,nJ);
e=spinn3d_motor_dc_kernel(tau,dq,c);
drag=c.b_Nm_s.*dq+c.coulomb_Nm_per_servo.*tanh(dq./c.omega_smooth_rad_s);
i_lo=max(-c.I_stall_A,(-c.V_bus_V-c.K_Nm_A.*dq)./c.R_ohm);
i_hi=min( c.I_stall_A,( c.V_bus_V-c.K_Nm_A.*dq)./c.R_ohm);
tau_min=c.n_parallel.*(c.K_Nm_A.*i_lo-drag);
tau_max=c.n_parallel.*(c.K_Nm_A.*i_hi-drag);
empty_interval=i_lo>i_hi | e.r_speed>1+1e-12;
tau_min(empty_interval)=NaN; tau_max(empty_interval)=NaN;
env.tau_internal_signed_Nm=e.tau_internal_Nm;
env.I_winding_signed_A=e.I_winding_A;
env.V_terminal_signed_V=e.V_terminal_V;
env.r_current=e.r_current; env.r_voltage=e.r_voltage;
env.voltage_command_ratio_signed=e.V_terminal_V./c.V_bus_V;
env.r_tau=e.r_current; env.r_speed=e.r_speed;
env.torque_speed_ratio=e.envelope_ratio;
env.duty=min(e.r_voltage,1);
env.tau_envelope_min_Nm=tau_min; env.tau_envelope_max_Nm=tau_max;
contains_zero=tau_min<=0 & tau_max>=0;
env.tau_speed_cap=nan(size(tau));
env.tau_speed_cap(contains_zero)=min( ...
    abs(tau_min(contains_zero)),abs(tau_max(contains_zero)));
env.envelope_axis_ok=isfinite(e.envelope_ratio) & ...
    e.envelope_ratio<=1+1e-12 & ~empty_interval;
env.envelope_ok=all(env.envelope_axis_ok,'all');
env.motor_power_schema=c.schema;
end

function y=shaped(x,nJ_expected)
if ~isempty(nJ_expected) && (~isnumeric(nJ_expected) || ...
        ~isreal(nJ_expected) || ~isscalar(nJ_expected) || ...
        ~isfinite(nJ_expected) || nJ_expected<1 || nJ_expected~=round(nJ_expected))
    error('spinn3d_motor_envelope:BadJointCount','nJ_expected must be a positive integer.');
end
y=double(x);
if isvector(y)
    if ~isempty(nJ_expected) && nJ_expected==1, y=y(:); else, y=y(:).'; end
end
if ~isreal(y) || ~ismatrix(y) || isempty(y) || any(~isfinite(y),'all')
    error('spinn3d_motor_envelope:BadShape', ...
        'Inputs must be finite real vectors or N-by-nJ matrices.');
end
end
function v=field(S,fn,def)
if isfield(S,fn) && ~isempty(S.(fn)), v=S.(fn); else, v=def; end
end
