function [b2,met,ok] = spinn3d_r2_nominal_policy_step(b,a_des,traj,robot,caps,bus,fric,areg,policy_dt,Ts,tol)
%SPINN3D_R2_NOMINAL_POLICY_STEP Nominal-reference screening, every plant step.
% Desired alpha is held over one policy interval. Applied alpha evolves at Ts
% through the SAME governor_step used in execution; alpha_dot is included at
% every substep. Dynamic source admission uses the actual source port and
% voltage/thermal checks; historical algebraic scope uses a conservative
% unshared bound. Each branch carries its own DC state. EMA uses midpoint power. This is a feed-forward reference model, not measured-state
% CTC/plant prediction. Final evidence always comes from common plant replay.

nsub=round(policy_dt/Ts);

b2=b; ok=true;
dynamic=strcmp(bus.model_scope,'dynamic_capacitor_common_dc_v1');
if dynamic

end
Ptarget=double(getf(bus,'P_total_max',inf));
Ppeak=double(getf(bus,'P_grid_peak_max',inf));
betaP=exp(-Ts/max(double(getf(bus,'P_avg_window_s',0.05)),Ts));
met=struct('progress',0,'elapsed_s',0,'soft_integral',0,'raw_integral',0, ...
    'alpha_change_squared',0,'n_plant_evaluations',0,'Pgrid_peak_W',0, ...
    'max_qd_ratio',0,'max_qdd_ratio',0,'max_tau_ratio',0,'reject_reason','');
for k=1:nsub
    ap=b2.a_prev;
    [q,qs,qss]=traj.eval(min(max(b2.s,0),traj.Tf));
    [a,~,~,adot]=spinn3d_alpha_governor_step(a_des,ap,double(areg.BETA), ...
        double(areg.A_DOT_UP),double(areg.A_DOT_DN),Ts,qs,qss,caps, ...
        double(caps.alpha_feasible.alpha_max));
    q=row(q); qd=a*row(qs); qdd=a^2*row(qss)+adot*row(qs);
    tau=row(inverseDynamics(robot,q,qd,qdd));
    tau=tau+row(fric.B).*qd+row(fric.Fc).*tanh(qd./max(double(fric.vel_eps),1e-9));
    qdStart=ap*row(qs);qdEnd=qdStart+Ts*qdd;
    bus_interval=bus;
    if dynamic, bus_interval=spinn3d_bus_with_state(bus,b2.bus_state,Ts); end
    interval=spinn3d_motor_interval_audit(tau,qdStart,qdEnd,bus_interval);
    Pg=interval.Pgrid_mid_W;flow=interval.flow_mid;
    Paxis=interval.Paxis_motoring_peak_W;
    sourceBound=interval.Pgrid_admission_upper_bound_W;
    met.n_plant_evaluations=met.n_plant_evaluations+1;
    rqd=max(max(abs([qd;qdStart;qdEnd])./max(row(caps.qd_max),eps)));
    rqdd=max(abs(qdd)./max(row(caps.qdd_max),eps));
    rtau=max(abs(tau)./max(row(caps.tau_max),eps));
    met.max_qd_ratio=max(met.max_qd_ratio,rqd);
    met.max_qdd_ratio=max(met.max_qdd_ratio,rqdd);
    met.max_tau_ratio=max(met.max_tau_ratio,rtau);
    finiteOK=all(isfinite([q qd qdStart qdEnd qdd tau Pg Paxis sourceBound b2.Pavg]));
    hardOK=finiteOK&&rqd<=1+tol&&rqdd<=1+tol&&rtau<=1+tol&& ...
        all(max(row(Paxis),0)<=row(caps.P_axis_max)+tol)&& ...
        logical(interval.dynamic_state_pass)&&logical(interval.envelope_ok)&&(~isfinite(Ppeak)||sourceBound<=Ppeak+tol)&& ...
        double(getf(flow,'Pspill',0))<=tol;
    if ~hardOK
        ok=false; met.reject_reason='nominal_substep_hard_constraint'; return;
    end
    if dynamic, b2.bus_state=interval.bus_state_next; end
    b2.Pavg=betaP*b2.Pavg+(1-betaP)*Pg;
    excess=0; if isfinite(Ptarget)&&Ptarget>0,excess=max(b2.Pavg/Ptarget-1,0);end
    raw=0; if isfinite(Ppeak)&&Ppeak>0,raw=max(Pg/Ppeak-1,0);end
    ds=min(max(traj.Tf-b2.s,0),Ts*max(a,0));
    b2.s=b2.s+ds; b2.a_prev=a;
    met.progress=met.progress+ds;
    met.elapsed_s=met.elapsed_s+Ts;
    met.soft_integral=met.soft_integral+Ts*excess^2;
    met.raw_integral=met.raw_integral+Ts*raw^2;
    met.alpha_change_squared=met.alpha_change_squared+(a-ap)^2;
    met.Pgrid_peak_W=max(met.Pgrid_peak_W,interval.Pgrid_peak_W);
end
end
function y=row(x),y=double(x(:)).';end
function v=getf(S,f,d),if isstruct(S)&&isfield(S,f)&&~isempty(S.(f)),v=S.(f);else,v=d;end,end
