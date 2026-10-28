function [measured,P] = spinn3d_plant_observe(P,tau,dq_power,Ts)
% Form joint-state observations and source-power measurements for control.

measured=[];
is_dynamic=isfield(P.bus,'model_scope') && ...
    strcmp(P.bus.model_scope,'dynamic_capacitor_common_dc_v1');
if ~P.unknown && ~is_dynamic, return; end
% The plant owns the only physical DC-state commit. Controller observation
% receives this measurement; candidate previews never modify P.bus_state.
[flow,next_state]=spinn3d_bus_step(tau,dq_power,P.bus,P.bus_state,Ts);
if is_dynamic && ~flow.state_pass
    error('spinn3d:PhysicalDCStepRejected', ...
        ['DC interval is physically infeasible (Vmin=%.9g V, Vmax=%.9g V, ', ...
         'residual=%.9g W). No voltage/energy/temperature clipping is used.'], ...
        flow.Vdc_min,flow.Vdc_max,flow.dc_solver_residual_max_W);
end
P.bus_state=next_state;
measured=struct('flow',flow,'bus_state',P.bus_state, ...
    'measurement_kind','ideal_bus_power_measurement');
if P.observed_steps==0
    measured.Pgrid_avg_initial_W=P.power_init.Pgrid_avg0_W;
end
P.observed_steps=P.observed_steps+1;
end
