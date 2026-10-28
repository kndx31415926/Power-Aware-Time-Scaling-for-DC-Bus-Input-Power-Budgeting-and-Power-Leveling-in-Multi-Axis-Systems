function mot = spinn3d_motor_power(tau,dq,bus)
%SPINN3D_MOTOR_POWER Passive signed-terminal electrical power and routing.

% Evaluate the passive_output_dc_pwm_v1 motor model.
% The shared numeric/dlarray kernel computes winding current, voltage and
% copper/drag loss. Holding consumes I^2*R. Low-speed braking can consume
% DC power; regeneration is available ONLY for negative terminal power.
% Stock-NR dissipates negative terminal power locally. The hypothetical
% bidirectional profile applies eta_regen, converter cap, then eta_share.
% Neither profile asserts a validated commercial servo drive or thermal
% capability.

if nargin<3 || isempty(bus) || ~isstruct(bus), bus=struct(); end
if ~isfield(bus,'spinn3d_defaults_applied') || ...
        isempty(bus.spinn3d_defaults_applied) || ...
        ~logical(bus.spinn3d_defaults_applied(1))
    bus=spinn3d_bus_defaults(bus);
end
tau=tau(:).'; dq=dq(:).';
if isempty(tau) || numel(tau)~=numel(dq) || ~isreal(tau) || ...
        ~isreal(dq) || any(~isfinite([tau,dq]))
    error('spinn3d_motor_power:BadInput', ...
        'tau and dq must be nonempty finite real vectors of equal length.');
end
nJ=numel(tau);
Paxis=tau.*dq; Ppos=sum(max(Paxis,0)); Pneg=sum(max(-Paxis,0));
model=lower(char(field(bus,'model','legacy')));
regen_enable=logical(field(bus,'regen_enable',~strcmp(model,'st3215_pwm')));
mot=struct('model',model,'Paxis',Paxis,'Ppos',Ppos,'Pneg',Pneg);

if strcmp(model,'st3215_pwm')
    quadrant_model=char(field(bus,'quadrant_power_model',''));
    if ~strcmp(quadrant_model,'signed_terminal_power_v1')
        error('spinn3d_motor_power:QuadrantModelMismatch', ...
            'MathSource97 requires signed_terminal_power_v1; regenerate stale configurations.');
    end
    c=spinn3d_motor_dc_constants(bus,nJ);
    e=spinn3d_motor_dc_kernel(tau,dq,c);
    env=spinn3d_motor_envelope(tau,dq,bus);
    Pelec_axis=max(e.Pterminal_W,0);
    Pterminal_gen_axis=max(-e.Pterminal_W,0);
    Pterminal_gen=sum(Pterminal_gen_axis);
    Pmot_electrical=sum(Pelec_axis);
    Ploss_mot=sum(e.Ploss_W);
    if regen_enable
        Pregen_axis_dc_raw=bus.eta_regen.*Pterminal_gen_axis;
        Pregen_converted_uncapped=sum(Pregen_axis_dc_raw);
        Pregen_available=min(Pregen_converted_uncapped,bus.Pregen_peak_max_W);
        Pregen_cap_curtailed=max(Pregen_converted_uncapped-Pregen_available,0);
        Pregen_bus=bus.eta_share.*Pregen_available;
        Ploss_regen=Pterminal_gen-Pregen_available;
        Ploss_share=Pregen_available-Pregen_bus;
    else
        Pregen_axis_dc_raw=zeros(size(Paxis));
        Pregen_converted_uncapped=0; Pregen_available=0;
        Pregen_cap_curtailed=0; Pregen_bus=0;
        Ploss_regen=Pterminal_gen; Ploss_share=0;
    end
    mot.motor_power_schema=c.schema;
    mot.parameter_status=c.parameter_status;
    mot.Paxis_electrical=Pelec_axis;
    mot.Paxis_dc_signed=e.Pterminal_W;
    mot.Paxis_loss_drive=e.Ploss_W;
    mot.Paxis_loss_copper=e.Pcopper_W;
    mot.Paxis_loss_drag=e.Pdrag_W;
    mot.Pregen_axis_terminal_available_W=Pterminal_gen_axis;
    mot.Pregen_terminal_available_W=Pterminal_gen;
    mot.Pregen_axis_dc_raw=Pregen_axis_dc_raw;
    mot.Pregen_axis_mechanical_available=max(-Paxis,0);
    mot.Pregen_dc_raw=sum(Pregen_axis_dc_raw);
    mot.Iaxis_source=Pelec_axis./c.V_bus_V;
    % Signed, unshared reference current; it is no longer a mechanically
    % masked current. External source current is known only after routing.
    mot.Iaxis_source_unmasked=e.Pterminal_W./c.V_bus_V;
    mot.Paxis_endpoint_reference=e.Pterminal_W;
    mot.Iphase=abs(e.I_winding_A);
    mot.I_winding_signed_A=e.I_winding_A;
    mot.V_terminal_signed_V=e.V_terminal_V;
    mot.I_source_total=sum(mot.Iaxis_source)+bus.P_idle_W/c.V_bus_V(1);
    mot.I_source_total_unshared=mot.I_source_total;
    mot.V_bus_V=c.V_bus_V(1);
    mot.duty=env.duty;
    mot.r_tau=env.r_tau; mot.r_speed=env.r_speed;
    mot.r_current=env.r_current; mot.r_voltage=env.r_voltage;
    mot.voltage_command_ratio_signed=env.voltage_command_ratio_signed;
    mot.torque_speed_ratio=env.torque_speed_ratio;
    mot.tau_dead_Nm=c.n_parallel.*c.coulomb_Nm_per_servo;
    mot.tau_internal_equiv_Nm=abs(e.tau_internal_Nm);
    mot.tau_internal_signed_Nm=e.tau_internal_Nm;
    mot.tau_envelope_min_Nm=env.tau_envelope_min_Nm;
    mot.tau_envelope_max_Nm=env.tau_envelope_max_Nm;
    mot.tau_speed_cap=env.tau_speed_cap;
    mot.envelope_axis_ok=env.envelope_axis_ok;
    mot.envelope_ok=env.envelope_ok;
    mot.actuator_envelope_model=env.actuator_envelope_model;
    mot.quadrant_power_model=quadrant_model;
    mot.electrical_balance_residual_W=sum(e.Pterminal_W-Paxis-e.Ploss_W);
else

    Pmot_electrical=Ppos/bus.eta_motoring;
    Ploss_mot=max(Pmot_electrical-Ppos,0);
    if regen_enable
        Pregen_converted_uncapped=bus.eta_regen*Pneg;
        Pregen_available=min(Pregen_converted_uncapped,bus.Pregen_peak_max_W);
        Pregen_cap_curtailed=max(Pregen_converted_uncapped-Pregen_available,0);
        Pregen_bus=bus.eta_share*Pregen_available;
        Ploss_regen=Pneg-Pregen_available;
        Ploss_share=Pregen_available-Pregen_bus;
    else
        Pregen_converted_uncapped=0; Pregen_available=0;
        Pregen_cap_curtailed=0; Pregen_bus=0;
        Ploss_regen=Pneg; Ploss_share=0;
    end
    mot.Paxis_electrical=max(Paxis,0)/bus.eta_motoring;
    mot.Iaxis_source=nan(1,nJ); mot.Iaxis_source_unmasked=nan(1,nJ);
    mot.Iphase=nan(1,nJ); mot.I_source_total=NaN;
    mot.I_source_total_unshared=NaN; mot.V_bus_V=NaN;
    mot.duty=nan(1,nJ); mot.r_tau=zeros(1,nJ); mot.r_speed=zeros(1,nJ);
    mot.r_current=zeros(1,nJ); mot.r_voltage=zeros(1,nJ);
    mot.voltage_command_ratio_signed=zeros(1,nJ);
    mot.torque_speed_ratio=zeros(1,nJ); mot.tau_speed_cap=inf(1,nJ);
    mot.tau_envelope_min_Nm=-inf(1,nJ); mot.tau_envelope_max_Nm=inf(1,nJ);
    mot.tau_internal_signed_Nm=tau; mot.envelope_axis_ok=true(1,nJ);
    mot.envelope_ok=true; mot.actuator_envelope_model='legacy_no_envelope_v1';
    mot.quadrant_power_model='legacy_mechanical_efficiency_v1';
end
mot.Pmot_electrical=Pmot_electrical;
mot.Pmot_axis_dc=Pmot_electrical;
mot.Paux=bus.P_idle_W;
mot.Pmot_dc=Pmot_electrical+bus.P_idle_W;
mot.Pregen_converted_uncapped=Pregen_converted_uncapped;
mot.Pregen_available=Pregen_available;
mot.Pregen_converted=Pregen_available;
mot.Pregen_cap_curtailed=Pregen_cap_curtailed;
mot.Pregen_bus=Pregen_bus;
% Aggregates partition drive loss and rejected generation exactly once.
mot.Ploss_mot=Ploss_mot; mot.Ploss_regen=Ploss_regen;
mot.Ploss_share=Ploss_share;
mot.architecture_id=bus.architecture_id; mot.profile_id=bus.profile_id;
end

function v=field(S,fn,def)
if isfield(S,fn) && ~isempty(S.(fn)), v=S.(fn); else, v=def; end
end
