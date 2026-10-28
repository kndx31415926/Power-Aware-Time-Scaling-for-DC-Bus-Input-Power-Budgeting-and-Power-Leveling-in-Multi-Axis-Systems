function [flow,st_next] = spinn3d_dynamic_bus_step(tau,dq,bus,st,Ts)
%SPINN3D_DYNAMIC_BUS_STEP Passive drives connected to a finite DC capacitor.
% The auxiliary branch is upstream and nonsharing. Injected regeneration
% enters ONE physical node; direct sharing is an accounting decomposition,
% never a second subtraction from power. Unsunk regeneration raises voltage.
% Profiles with disabled injection posit local drive dissipation explicitly;
% that abstraction is not a calibrated commercial-servo hardware claim.
% The link contains no braking resistor or associated thermal state. There is no optional battery/buffer.
% Ebuf is a derived capacitor-energy compatibility alias. Precharge and
% capacitor discharge are NEVER labelled as current-task regenerative reuse.

if nargin<5 || isempty(Ts), Ts=1e-3; end
if nargin<4 || isempty(st)
    if isfield(bus,'dc_state') && ~isempty(bus.dc_state)
        st=bus.dc_state;
    else
        st=spinn3d_bus_state_init(bus);
    end
end
bus=spinn3d_bus_with_state(bus,st,Ts);
if isfield(st,'Ebuf')
    expected=0.5*bus.C_dc_F*st.Vdc^2;

end
for fn={'Ereuse','Eregen_task'}
    if isfield(st,fn{1})

    end
end
mot=spinn3d_motor_power(tau,dq,bus);
route=spinn3d_dynamic_dc_route(mot.Paxis_dc_signed(:),bus);
Preg_available=route.Pregen_available;
Preg=route.Pregen_injected;
local_rejection=route.Plocal_brake;
K=spinn3d_dc_link_kernel(mot.Pmot_axis_dc,Preg,st.Vdc,0,bus,Ts);
% The requested torque must be voltage-feasible throughout the resolved
% electrical substeps. A solver failure remains explicit even if a finite

env_bus=bus;
env_bus.V_bus_V=max(K.Vdc_min,eps);
env=spinn3d_motor_envelope(tau,dq,env_bus);
Pshare=route.Pshare_direct;
Plocal=route.Pconversion_loss+route.Psharing_loss+local_rejection;
Ploss=mot.Ploss_mot+Plocal+K.Psource_loss;
energy_residual=K.Pgrid+mot.Pneg- ...
    (mot.Ppos+mot.Paux+Ploss+K.Pdump+K.Pstorage_rate);

st_next=st;
st_next.model_scope=bus.model_scope;
st_next.has_voltage_state=true;
st_next.has_brake_thermal_state=false;
st_next.has_identified_brake_thermal_state=false;
st_next.Vdc=K.Vdc_next; st_next.Tbrk=K.Tbrk_next;
st_next.Ebuf=K.Ecap_next;
st_next.Ereuse=0; st_next.Eregen_task=0;

flow=mot;
flow.model_scope=bus.model_scope;
flow.auxiliary_topology=bus.auxiliary_topology;
flow.has_voltage_state=true;
flow.has_brake_thermal_state=false;
flow.has_identified_brake_thermal_state=false;
flow.dc_kernel_schema=K.schema;
flow.brake_model='none';
flow.V_bus_V=K.Vdc_next;
flow.V_source_V=bus.V_source_V;
flow.V_nominal_V=bus.V_nominal_V;
flow.Vdc_initial=st.Vdc; flow.Vdc=K.Vdc_next;
flow.Vdc_next=K.Vdc_next; flow.Tbrk_next=K.Tbrk_next;
flow.Vdc_min=K.Vdc_min; flow.Vdc_max=K.Vdc_max;
flow.Tbrk_initial=0; flow.Tbrk=K.Tbrk_next;
flow.Tbrk_max=K.Tbrk_max;
flow.Ecap_initial=K.Ecap_initial; flow.Ecap=K.Ecap_next;
flow.Ecap_delta=K.Ecap_delta;
flow.Pstorage_rate=K.Pstorage_rate;
flow.Psource_loss=K.Psource_loss;
flow.Pline_loss=K.Pline_loss;
flow.Pcurrent_limit_loss=K.Pcurrent_limit_loss;
flow.Pbus_source=K.Pbus_source;
flow.Pgrid=K.Pgrid;
flow.I_source_total=K.Isource;
flow.I_link_source=K.Ilink;
% Same-state source-droop request; it does not affect delivered power/state.
% Isource_peak_A resolves substeps; I_source_total remains an interval mean.
flow.Psource_request_W=K.Psource_request_W;
flow.Psource_request_peak_W=K.Psource_request_peak_W;
flow.Isource_peak_A=K.Isource_peak_A;
flow.Pdump=K.Pdump;
flow.Pdump_req=K.Pdump_requested;
flow.Pdump_cap=0;
% Resolved electrical-substep evidence, distinct from interval-average Pdump.
brake_keys={'brake_commanded_fraction','brake_active_fraction', ...
    'brake_rated_cap_binding_fraction','brake_resistor_binding_fraction', ...
    'Pdump_peak_W','Pdump_request_peak_W','Pdump_cap_independent_peak_W', ...
    'brake_diagnostic_tolerance_W'};
for j=1:numel(brake_keys),flow.(brake_keys{j})=K.(brake_keys{j});end

flow.Pdump_thermal_cap=0;
flow.Pthermal_cooling=K.Pthermal_cooling;
flow.Pspill=0; % unabsorbed electrical energy changes Vdc, it is not deleted
flow.Ploss_total=Ploss;
flow.Ploss_buf_chg=0; flow.Ploss_buf_dis=0;
flow.Plocal_dissipation=Plocal;
flow.Pregen_curtailed=local_rejection;
flow.Pregen_curtailed_total=local_rejection+mot.Pregen_cap_curtailed;
flow.Pregen_local_dissipation=local_rejection;
flow.Pregen_bus_available=Preg_available;
flow.Pregen_bus=Preg;
flow.Pregen_mechanical_available=mot.Pneg;
flow.Pshare=Pshare; flow.Pshare_axis=Pshare;
flow.Pshare_uncapped_potential=min(Preg_available,mot.Pmot_axis_dc);
flow.Pshare_potential=flow.Pshare_uncapped_potential;
flow.Poverlap=flow.Pshare_potential;
flow.Preuse_buf=0; flow.Preuse_task_buf=0;
flow.Preuse_total=Pshare; flow.Pregen_task_used=Pshare;
flow.Pdem_axis=max(mot.Pmot_axis_dc-Pshare,0);
flow.Pdem=flow.Pdem_axis+mot.Paux;
flow.Preg=max(Preg-Pshare,0);
flow.Pbuf_dis=max(-K.Pstorage_rate,0);
flow.Pbuf_chg=max(K.Pstorage_rate,0);
flow.Pbuf_dis_cap=NaN; flow.Pbuf_chg_cap=NaN;
flow.Ein=max(K.Ecap_delta,0); flow.Eout=max(-K.Ecap_delta,0);
flow.phi_regen=0; flow.phi_task=0;
flow.Ereuse_store_out=0; flow.Eregen_task_out=0;
flow.Ereuse_buf_out=0; flow.Ereuse_total=Pshare*Ts;
flow.Eshare_axis=Pshare*Ts;
flow.Eoverlap=flow.Pshare_potential*Ts;
flow.Eoverlap_uncapped=flow.Pshare_uncapped_potential*Ts;
flow.Eregen_curtailed=local_rejection*Ts;
flow.Eregen_curtailed_total=flow.Pregen_curtailed_total*Ts;
flow.Eregen_bus_in=Preg*Ts;
flow.Ebuf_min_J=0.5*bus.C_dc_F*bus.V_min_V^2;
flow.Ebuf_usable=max(K.Ecap_next-flow.Ebuf_min_J,0);
% Compatibility residuals now mean the dynamic node equation and the
% injection-interface balance, respectively; no fictitious load splitter.
flow.Pload_balance_residual=K.Presidual;
flow.Pregen_route_balance_residual=Preg_available-Preg-local_rejection;
flow.Penergy_balance_residual=energy_residual;
flow.dc_solver_residual_max_W=K.solver_residual_max_W;
flow.dc_solver_ok=logical(K.solver_ok);
flow.dc_voltage_ok=logical(K.voltage_ok);
flow.dc_thermal_ok=logical(K.thermal_ok);
flow.dc_source_ok=logical(K.source_ok);
flow.solver_ok=flow.dc_solver_ok; flow.voltage_ok=flow.dc_voltage_ok;
flow.thermal_ok=flow.dc_thermal_ok; flow.source_ok=flow.dc_source_ok;
flow.dc_state_pass=logical(K.state_pass);
flow.state_pass=flow.dc_state_pass;
flow.dc_substeps=K.n_substeps;
flow.energy_balance_ok=isfinite(energy_residual) && ...
    abs(energy_residual)<=bus.dc_solver_tolerance_W;
flow.state_pass=flow.state_pass && flow.energy_balance_ok;
flow.dc_state_pass=flow.state_pass;
flow.motor_envelope_axis_ok=env.envelope_axis_ok;
flow.motor_envelope_ok=env.envelope_ok;
flow.motor_current_ratio=env.r_current;
flow.motor_voltage_ratio=env.r_voltage;
flow.motor_voltage_command_ratio_signed=env.voltage_command_ratio_signed;
flow.motor_duty=env.duty;
flow.torque_speed_ratio=env.torque_speed_ratio;
flow.tau_speed_cap=env.tau_speed_cap;
flow.tau_envelope_min_Nm=env.tau_envelope_min_Nm;
flow.tau_envelope_max_Nm=env.tau_envelope_max_Nm;
flow.motor_model=mot.model;
flow.actuator_envelope_model=env.actuator_envelope_model;
flow.routing_mode=bus.routing_mode;
flow.direct_share_enable=bus.direct_share_enable;
end
