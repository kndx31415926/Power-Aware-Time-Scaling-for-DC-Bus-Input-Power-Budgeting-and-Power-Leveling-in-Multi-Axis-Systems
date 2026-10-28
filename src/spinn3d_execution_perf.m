function P=spinn3d_execution_perf(L,plant_dt)
%SPINN3D_EXECUTION_PERF Timing of executed software, with explicit boundaries.
% policy_total_ms includes feature construction, model forward, policy search
% and the controller's common guard on policy ticks. plant_guard_ms is the
% separate post-noise execution projector/integrator. Logical block time sums
% all measured 1-ms simulation computations until the next policy tick.
% These host wall clocks include simulation dynamics and logging, exclude
% plotting, file IO, robot communications and actuator hardware response.
P=struct('schema','full_controller_and_execution_wall_clock_v1', ...
    'plant_dt_s',double(plant_dt), ...
    'scope','host simulation; no hardware IO or hard-real-time guarantee');
P.step_total_ms=1000*double(L.step_wall_s(:));
P.controller_step_ms=1000*double(L.controller_wall_s(:));
P.plant_guard_ms=1000*double(L.plant_guard_wall_s(:));
tick=logical(L.policy_tick(:));
P.policy_tick_indices=find(tick);
P.policy_total_ms=P.controller_step_ms(tick);
P.policy_plus_execution_ms=P.controller_step_ms(tick)+P.plant_guard_ms(tick);
P.nonpolicy_controller_ms=P.controller_step_ms(~tick);
P.network_forward_ms=1000*double(L.network_forward_s(tick));
P.network_forward_ms=P.network_forward_ms(isfinite(P.network_forward_ms));
P.logical_policy_block_ms=zeros(numel(P.policy_tick_indices),1);
P.logical_policy_block_duration_ms=zeros(numel(P.policy_tick_indices),1);
for j=1:numel(P.policy_tick_indices)
    first=P.policy_tick_indices(j);
    if j<numel(P.policy_tick_indices),last=P.policy_tick_indices(j+1)-1;
    else,last=numel(P.step_total_ms);end
    P.logical_policy_block_ms(j)=sum(P.step_total_ms(first:last));
    P.logical_policy_block_duration_ms(j)=1000*plant_dt*(last-first+1);
end
P.motion_projection_fraction=mean(double(L.motion_projection));
P.actuator_saturation_fraction=mean(double(L.actuator_saturation));
P.failed_actual_steps=sum(~logical(L.actual_step_pass));
end
