function A = spinn3d_motor_interval_audit(tau,dq0,dq1,bus)
% Evaluate motor physical limits over an executed integration interval.

tau=double(tau(:).'); dq0=double(dq0(:).'); dq1=double(dq1(:).');
if isempty(tau) || ~isequal(size(tau),size(dq0),size(dq1)) || ...
        ~isreal([tau,dq0,dq1]) || any(~isfinite([tau,dq0,dq1]))
    error('spinn3d_motor_interval_audit:BadInput', ...
        'tau, dq0, dq1 must be equal-length finite real vectors.');
end
bus=spinn3d_bus_defaults(bus);
speeds=[dq0;0.5*(dq0+dq1);dq1];
dynamic=isfield(bus,'model_scope') && ...
    strcmp(bus.model_scope,'dynamic_capacitor_common_dc_v1');
Pg=zeros(3,1); Pd=zeros(3,1); axis_draw=zeros(3,numel(tau));
state_ok=true; voltage_min=inf; voltage_max=-inf;
if dynamic

    dt=double(bus.dc_step_s);
else
    dt=1;
end
for j=1:3
    if dynamic
        % Each alternative is a preview from the same measured initial state.
        % Only the midpoint mechanical-power preview is physically committed.
        [f,st_preview]=spinn3d_bus_step(tau,speeds(j,:),bus,bus.dc_state,dt);
        state_ok=state_ok && f.state_pass;
        voltage_min=min(voltage_min,f.Vdc_min);
        voltage_max=max(voltage_max,f.Vdc_max);
    else
        [~,~,~,~,~,f]=spinn3d_bus_power(tau,speeds(j,:),bus);
        st_preview=[];
    end
    Pg(j)=f.Pgrid; Pd(j)=f.Pdump;
    axis_draw(j,:)=f.Paxis_electrical;
    if j==2, flow_mid=f; state_mid=st_preview; end
end
env_bus=bus;
if dynamic, env_bus.V_bus_V=max(voltage_min,eps); end
env=spinn3d_motor_envelope(repmat(tau,3,1),speeds,env_bus,numel(tau));
A=struct();
A.schema='held_torque_affine_speed_interval_electrical_v2';
A.dynamic_state_pass=logical(state_ok);
A.Vdc_min=voltage_min; A.Vdc_max=voltage_max;
if ~dynamic, A.Vdc_min=NaN; A.Vdc_max=NaN; end
A.r_current=max(env.r_current,[],1);
A.r_voltage=max(env.r_voltage,[],1);
A.r_speed=max(env.r_speed,[],1);
A.torque_speed_ratio=max(env.torque_speed_ratio,[],1);
A.envelope_axis_ok=all(env.envelope_axis_ok,1);
A.envelope_ok=all(A.envelope_axis_ok);
A.tau_envelope_min_Nm=max(env.tau_envelope_min_Nm,[],1);
A.tau_envelope_max_Nm=min(env.tau_envelope_max_Nm,[],1);
invalid=any(isnan(env.tau_envelope_min_Nm) | isnan(env.tau_envelope_max_Nm),1);
A.tau_envelope_min_Nm(invalid)=NaN;
A.tau_envelope_max_Nm(invalid)=NaN;
A.Pgrid_samples_W=Pg;
A.Pdump_samples_W=Pd;
A.Pgrid_peak_W=max(Pg);
A.Pdump_peak_W=max(Pd);
A.source_peak_is_sampled_only=true;
A.Pgrid_unshared_upper_bound_W=sum(max(axis_draw([1,3],:),[],1))+flow_mid.Paux;
A.Paxis_motoring_peak_W=max(max(speeds.*tau,0),[],1);
A.Pgrid_mid_W=Pg(2);
A.flow_mid=flow_mid;
A.bus_state_next=state_mid;
A.flow_mid_energy_interval_s=dt;
A.Pgrid_admission_upper_bound_W=A.Pgrid_unshared_upper_bound_W;
if dynamic
    % The physical source equation enforces its cap at every DC substep.
    % Axis demand may exceed source draw while the capacitor discharges.
    A.Pgrid_admission_upper_bound_W=A.Pgrid_peak_W;
end
A.sample_speeds_rad_s=speeds;
end
