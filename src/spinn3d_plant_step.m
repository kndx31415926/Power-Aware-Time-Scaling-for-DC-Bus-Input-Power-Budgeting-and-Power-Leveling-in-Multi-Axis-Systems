function [tau,ddq,dq_mid,A] = spinn3d_plant_step( ...
    P,q,dq,q_meas,dq_meas,tau_request,tau_max,Paxis_max,Ts,limits,workspace,tol)
% Advance the coupled robot and DC-link state over one plant interval.

nominal_dc=spinn3d_dc_state_for_model(P.bus_state,P.nominal_bus);
P.nominal_bus=spinn3d_bus_with_state(P.nominal_bus,nominal_dc,Ts);
P.bus=spinn3d_bus_with_state(P.bus,P.bus_state,Ts);
if ~P.separated_execution
    [tau,ddq,dq_mid,A]=spinn3d_predictive_torque_step( ...
        P.nominal_robot,q,dq,tau_request,tau_max,Paxis_max, ...
        P.nominal_fric,P.nominal_bus,Ts,limits,workspace,tol);
    A.true_plant_motion_oracle_projection=false;
    A.nominal_prediction_only=false;
    A.actuator_saturation_applied=false;
    return;
end
[tau_nom,~,~,A]=spinn3d_predictive_torque_step( ...
    P.nominal_robot,q_meas,dq_meas,tau_request,tau_max,Paxis_max, ...
    P.nominal_fric,P.nominal_bus,Ts,limits,workspace,tol);
q=double(q(:)).'; dq=double(dq(:)).';
n=numel(q); tm=expand_local(tau_max,n); pa=expand_local(Paxis_max,n);
B=expand_local(P.fric.B,n); Fc=expand_local(P.fric.Fc,n);
M=massMatrix(P.robot,q);
h=double(velocityProduct(P.robot,q,dq))+double(gravityTorque(P.robot,q));
h=h(:).'+B.*dq+Fc.*tanh(dq/max(double(P.fric.vel_eps),1e-9));
tau=min(tm,max(-tm,tau_nom)); converged=false;
for it=1:64
    ddq=(M\(tau(:)-h(:))).'; dq_mid=dq+0.5*Ts*ddq;
    dq_next=dq+Ts*ddq;
    interval=spinn3d_motor_interval_audit(tau,dq,dq_next,P.bus);
    lo=max(-tm,interval.tau_envelope_min_Nm);
    hi=min(tm,interval.tau_envelope_max_Nm);
    if any(lo>hi)
        error('spinn3d:UnknownPlantEmptyActuatorEnvelope', ...
            'No held torque meets the actual start/mid/end actuator envelopes.');
    end
    next=min(hi,max(lo,tau));
    % Actual actuator saturation uses the same three interval samples as
    % the nominal physical predictor. Energy telemetry remains midpoint.
    next=clamp_power_axis_total_bus(next,dq,pa,P.bus);
    next=clamp_power_axis_total_bus(next,dq_mid,pa,P.bus);
    next=clamp_power_axis_total_bus(next,dq_next,pa,P.bus);
    next=min(tm,max(-tm,next));
    err=norm(next-tau,inf); tau=next;
    if err<=1e-10*max(1,norm(tau,inf)), converged=true; break; end
end
ddq=(M\(tau(:)-h(:))).'; dq_mid=dq+0.5*Ts*ddq;
interval=spinn3d_motor_interval_audit(tau,dq,dq+Ts*ddq,P.bus);
pg=interval.Pgrid_mid_W; pdump=interval.flow_mid.Pdump;
actuator_ok=converged && interval.envelope_ok && ...
    all(abs(tau)<=tm+1e-9) && ...
    all(interval.Paxis_motoring_peak_W<=pa+1e-9) && ...
    interval.Pgrid_admission_upper_bound_W<=P.bus.P_grid_peak_max+1e-8 && ...
    interval.dynamic_state_pass && ...
    interval.Pdump_peak_W<=dump_cap_local(P.bus)+1e-8;
if ~actuator_ok
    error('spinn3d:UnknownPlantActuatorFailure', ...
        ['Unknown plant has no converged admissible actuator command at ', ...
         'this integration step. No true-model motion recovery is used.']);
end
A.nominal_prediction_only=true;
A.true_plant_motion_oracle_projection=false;
A.nominal_final_motion=A.final_motion;
A.final_motion=motion_audit_local(P.robot,q,dq,ddq,dq_mid,Ts,limits,workspace,tol);
A.final_state_verified_before_commit=logical(A.final_motion.pass);
A.final_qdd=ddq; A.dq_power=dq_mid;
A.joint_limit_viability_pass=A.final_motion.joint_limit_viability_pass;
A.joint_limit_stopping_violation_rad=A.final_motion.joint_limit_stopping_violation_rad;
A.Pgrid_mid_W=pg; A.Pgrid_peak_W=P.bus.P_grid_peak_max;
A.Pdump_mid_W=pdump; A.Pdump_peak_W=dump_cap_local(P.bus);
A.motor_ratio=max(interval.torque_speed_ratio,[],'all');
A.Pgrid_interval_peak_W=interval.Pgrid_peak_W;
A.Pgrid_unshared_upper_bound_W=interval.Pgrid_unshared_upper_bound_W;
A.dynamic_dc_state_pass=interval.dynamic_state_pass;
A.Pgrid_guard_upper_W=interval.Pgrid_admission_upper_bound_W;
A.Pdump_interval_peak_W=interval.Pdump_peak_W;
A.tau_ratio=max(abs(tau)./max(tm,eps));
A.Paxis_ratio=max(interval.Paxis_motoring_peak_W./max(pa,eps));
A.actuator_saturation_applied=norm(tau-tau_nom,inf)>1e-9;
A.execution_command_modified=A.execution_command_modified || A.actuator_saturation_applied;
% Progress correction uses observable torque saturation, not true M or h.
if A.actuator_saturation_applied
    A.execution_progress_ratio=0;
    A.execution_progress_basis='actuator_saturation_pause';
end
A.iterations=A.iterations+it;
end

function v=expand_local(v,n)
v=double(v(:)).'; if isscalar(v),v=repmat(v,1,n);end

end

function F=motion_audit_local(robot,q,dq,ddq,dqm,dt,L,W,T)
% Independent check only: this function cannot change tau or ddq.
n=numel(q); qlo=expand_local(L.q_min,n); qhi=expand_local(L.q_max,n);
alim=expand_local(L.qdd_cmd_max_actual,n); vlim=expand_local(L.qd_max,n);
qn=q+dt*dqm; vn=dq+dt*ddq;
low=min(q,qn); high=max(q,qn); turning=-dq./ddq;
inside=isfinite(turning)&turning>0&turning<dt;
qt=q(inside)+dq(inside).*turning(inside)+0.5*ddq(inside).*turning(inside).^2;
low(inside)=min(low(inside),qt); high(inside)=max(high(inside),qt);
qv=max([0,qlo-low,high-qhi]);
vr=max(max([abs(dq);abs(dqm);abs(vn)],[],1)./max(vlim,eps));
ar=max(abs(ddq)./max(alim,eps));
V=spinn3d_joint_limit_viability(qn,vn,qlo,qhi,alim,T.joint_limit_viability,T.position_tolerance_rad);
G=spinn3d_physical_geometry_check(robot,qn,W);
kin=all(isfinite([qn vn ddq])) && vr<=1+T.ratio_tolerance && ...
    ar<=1+T.ratio_tolerance && qv<=T.position_tolerance_rad && V.pass;
F=struct('pass',logical(kin&&G.pass),'kinematic_pass',logical(kin), ...
    'geometry_pass',logical(G.pass),'qdd_ratio',ar,'qd_ratio',vr, ...
    'qpos_violation_rad',qv,'joint_limit_viability_pass',logical(V.pass), ...
    'joint_limit_stopping_violation_rad',V.violation_rad, ...
    'dq_next',vn,'q_next',qn,'geometry',G);
end

function p=dump_cap_local(bus)
if strcmp(bus.model_scope,'dynamic_capacitor_common_dc_v1'),p=0;else,p=bus.P_brk_peak;end
end
