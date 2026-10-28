function log=spinn3d_dynamic_bus_log(log,k,flow,state,power_init,bus)
%SPINN3D_DYNAMIC_BUS_LOG Physical interval states; missing evidence is NaN.
% Pre-t0 values are repeated so truncation preserves a self-contained log.
if ~strcmp(char(bus.model_scope),'dynamic_capacitor_common_dc_v1'),return;end
names={'Vdc','Vdc_min','Vdc_max','Vdc_pre_t0_V', ...
    'Ecap_pre_t0_J','Ecap_delta_J','Pbus_source','Isource', ...
    'Psource_loss','Pline_loss','Pcurrent_limit_loss','dc_residual_W', ...
    'dc_solver_residual_max_W','dc_solver_ok','dc_state_pass', ...
    'dc_voltage_ok'};
source_keys={'Psource_request_W','Psource_request_peak_W','Isource_peak_A'};
names=[names source_keys];
N=numel(log.t);
for j=1:numel(names)
    if ~isfield(log,names{j}),log.(names{j})=nan(N,1);end
end
s0=power_init.bus_state0;
log.Vdc(k)=state.Vdc;
log.Vdc_pre_t0_V(k)=s0.Vdc;
log.Ecap_pre_t0_J(k)=0.5*bus.C_dc_F*s0.Vdc^2;
log.Vdc_min(k)=flow.Vdc_min;log.Vdc_max(k)=flow.Vdc_max;
log.Ecap_delta_J(k)=flow.Ecap_delta;
keys={'Pbus_source','Psource_loss','Pline_loss','Pcurrent_limit_loss'};
for j=1:numel(keys),log.(keys{j})(k)=flow.(keys{j});end
log.Isource(k)=flow.I_source_total;
% Isource is the interval mean; Isource_peak_A is the maximum of resolved
% substeps. Psource_request is unclipped droop at those same voltages, not
% net motor demand or a free-source counterfactual. Missing means unknown.
for j=1:numel(source_keys)
    key=source_keys{j};
    log.(key)(k)=NaN;
    if isfield(flow,key),log.(key)(k)=double(flow.(key));end
end
log.dc_residual_W(k)=flow.Pload_balance_residual;
log.dc_solver_residual_max_W(k)=flow.dc_solver_residual_max_W;
log.dc_solver_ok(k)=double(flow.solver_ok);
log.dc_state_pass(k)=double(flow.state_pass);
log.dc_voltage_ok(k)=double(flow.voltage_ok);
end
