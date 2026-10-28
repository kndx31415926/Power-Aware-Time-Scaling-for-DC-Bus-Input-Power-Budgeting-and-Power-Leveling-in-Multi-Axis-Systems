function K = spinn3d_dc_link_kernel(Pload_axis,Pregen_injected,V0,T0,bus,Ts)
% Integrate the common DC-link voltage and electrical energy balance.
% The source, capacitance and motor power flows use a common physical state.

if nargin<6 || isempty(Ts), Ts=1e-3; end

C=bus.C_dc_F; R=bus.R_source_ohm; Vs=bus.V_source_V;
Paux=bus.P_idle_W;
Iaux=Paux/Vs;
Icap=max(min(bus.I_supply_max_A,bus.P_grid_peak_max/Vs)-Iaux,0);

% larger plant interval; the schedule depends only on configuration and Ts.
step_ratio=Ts/(1e-3/bus.dc_substeps);
% Differenced timestamps carry a few ulps; do not change the integration
% schedule merely because a logged 1-ms interval is 0.001000000000000001.
nsub=max(bus.dc_substeps,ceil(step_ratio-1e-9*max(1,step_ratio)));
h=Ts/nsub; A=2*C/h;
% T0 is an ignored positional compatibility argument; no thermal state exists.
z=0.*(Pload_axis+Pregen_injected+V0);
v=V0+z;
K=struct();
K.Pgrid=z; K.Pbus_source=z; K.Isource=z; K.Ilink=z;
% Observations only: request is the reverse-blocked source droop before its
% current/power caps, evaluated at the SAME solved substep voltage. It is
% neither net motor demand nor a hypothetical unconstrained-source rollout.
% Source auxiliaries are upstream and included in both observations.
K.Psource_request_W=z; K.Psource_request_peak_W=z; K.Isource_peak_A=z;
K.Pdump=z; K.Pdump_requested=z; K.Psource_loss=z; K.Pline_loss=z;
K.Pcurrent_limit_loss=z; K.Pthermal_cooling=z;
K.Presidual=z; K.solver_residual_max_W=z;
K.Vdc_min=v; K.Vdc_max=v; K.Tbrk_max=z;
K.Vdc_abs_min=v;

K.brake_diagnostic_tolerance_W=1e-8;
K.brake_commanded_fraction=z; K.brake_active_fraction=z;
K.brake_rated_cap_binding_fraction=z; K.brake_resistor_binding_fraction=z;
K.Pdump_peak_W=z; K.Pdump_request_peak_W=z;
K.Pdump_cap_independent_peak_W=z;
for sub=1:nsub
    vold=v;
    % F'(Vm) >= A*(2*Vm-Vold)-Icap. This lower bound keeps
    % the Newton search on the increasing high-voltage root branch.
    floor_vm=max(0.5.*vold+Icap/(2*A)+1e-9,1e-9);
    % F(U)>=0 follows from Ilink<=Icap and Pload,Pdump>=0.
    % This mathematical root bound prevents Newton from jumping to enormous
    % voltages when an infeasible neural request starts near F'=0. It is
    % NOT an operating-voltage clip: genuine overvoltage remains observable,
    % and missing roots still fail the residual/solver_ok admission below.
    root_b=vold+Icap/A;
    upper_vm=max(0.5.*(root_b+sqrt(root_b.^2+ ...
        4.*Pregen_injected./A)),floor_vm);
    guess_v=sqrt(max(vold.^2+2*h/C.*(Pregen_injected-Pload_axis),1e-16));
    vm=min(max(0.5.*(vold+guess_v),floor_vm),upper_vm);
    for it=1:bus.dc_newton_iterations
        [il,dil]=source_port(vm,bus,Icap);
        f=A.*vm.*(vm-vold)-vm.*il-Pregen_injected+Pload_axis;
        df=A.*(2.*vm-vold)-il-vm.*dil;
        vm=min(max(vm-f./max(df,1e-9),floor_vm),upper_vm);
    end
    il=source_port(vm,bus,Icap);
    v=2.*vm-vold;
    pbus=vm.*il;
    pgrid=Vs.*il+Paux;
    psource_request=Vs.*max((Vs-vm)./R,0)+Paux;
    K.Psource_request_W=K.Psource_request_W+psource_request/nsub;
    K.Psource_request_peak_W=max(K.Psource_request_peak_W,psource_request);
    K.Isource_peak_A=max(K.Isource_peak_A,il+Iaux);
    pline=R.*il.^2;
    plimit=max(Vs-vm-R.*il,0).*il;
    pseries=pline+plimit;
    dE=0.5*C.*(v.^2-vold.^2);
    residual=pbus+Pregen_injected-Pload_axis-dE/h;
    K.Pgrid=K.Pgrid+pgrid/nsub;
    K.Pbus_source=K.Pbus_source+pbus/nsub;
    K.Isource=K.Isource+(il+Iaux)/nsub;
    K.Ilink=K.Ilink+il/nsub;
    K.Psource_loss=K.Psource_loss+pseries/nsub;
    K.Pline_loss=K.Pline_loss+pline/nsub;
    K.Pcurrent_limit_loss=K.Pcurrent_limit_loss+plimit/nsub;
    K.Presidual=K.Presidual+residual/nsub;
    K.solver_residual_max_W=max(K.solver_residual_max_W,abs(residual));
    K.Vdc_min=min(K.Vdc_min,v); K.Vdc_max=max(K.Vdc_max,v);
    K.Vdc_abs_min=min(K.Vdc_abs_min,vm);
end
K.Vdc_next=v; K.Tbrk_next=z;
K.Ecap_initial=0.5*C.*V0.^2;
K.Ecap_next=0.5*C.*v.^2;
K.Ecap_delta=K.Ecap_next-K.Ecap_initial;
K.Pstorage_rate=K.Ecap_delta/Ts;
K.Penergy_balance_residual=K.Pgrid+Pregen_injected- ...
    (Pload_axis+Paux+K.Psource_loss+K.Pdump+K.Pstorage_rate);
K.solver_ok=K.solver_residual_max_W<=bus.dc_solver_tolerance_W & ...
    isfinite(v) & v>0;
K.voltage_ok=K.Vdc_min>=bus.V_min_V-1e-10 & ...
    K.Vdc_max<=bus.V_max_V+1e-10;
K.thermal_ok=true; % compatibility only; no thermal constraint
K.source_ok=K.Isource<=bus.I_supply_max_A+1e-10 & ...
    K.Pgrid<=bus.P_grid_peak_max+1e-10 & K.Pgrid>=0;
K.state_pass=K.solver_ok & K.voltage_ok & K.source_ok;
K.n_substeps=nsub;
K.brake_model='none';
K.has_brake_thermal_state=false;
K.schema='implicit_energy_dc_link_no_brake_v2';
end

function [il,dil]=source_port(vm,bus,Icap)
raw_i=(bus.V_source_V-vm)/bus.R_source_ohm;
il=min(max(raw_i,0),Icap);
if nargout>1
    dil=-(raw_i>0 & raw_i<Icap)./bus.R_source_ohm;
end
end
