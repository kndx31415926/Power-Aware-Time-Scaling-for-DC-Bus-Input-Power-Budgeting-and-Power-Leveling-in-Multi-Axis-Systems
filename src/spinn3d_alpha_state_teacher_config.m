function C=spinn3d_alpha_state_teacher_config(P,caps,fric,bus,Kp,Kd,Pbud)
% Construct teacher settings for measured-state rate-command selection.

if isfield(P,'alpha_budget')
    B=P.alpha_budget; dt=P.time.Ts; policy=P.policy;
    A=struct('BETA',P.alpha_reg.beta,'A_DOT_UP',P.alpha_reg.a_dot_up, ...
        'A_DOT_DN',P.alpha_reg.a_dot_dn);
    amin=P.alpha.alpha_min; amax=P.alpha.alpha_max;
    workspace=P.workspace; stop=P.stop;
else
    B=P.ALPHA_BUDGET; dt=P.TS; policy=P.POLICY; A=P.ALPHA_REG;
    amin=P.ALPHA_MIN; amax=P.ALPHA_MAX; workspace=P.WORKSPACE;
    stop=struct('ee_tol',P.STOP.EE_TOL,'dq_tol_rad',P.STOP.DQ_TOL_RAD, ...
        'stable_time',P.STOP.STABLE_TIME);
end

C=B.state_teacher;
required={'schema','power_enabled','horizon_policy_ticks','grid_points', ...
    'local_offsets','transition_refine_iterations','weights', ...
    'tracking_q_scale_rad','tracking_dq_scale_rad_s','terminal_window_s', ...
    'geometry_every_step','progress_velocity_floor_rad_s'};

C.Ts=double(dt); C.policy_dt=double(policy.dt);
C.policy_steps=round(C.policy_dt/C.Ts);
C.beta=double(A.BETA); C.up=double(A.A_DOT_UP); C.dn=double(A.A_DOT_DN);
C.alpha_min=double(amin); C.alpha_max=min(double(amax), ...
    double(caps.alpha_feasible.alpha_max));
C.caps=caps; C.fric=fric; C.bus=spinn3d_bus_defaults(bus);
C.Kp=double(Kp(:).'); C.Kd=double(Kd(:).'); C.Pbud=double(Pbud);
C.workspace=workspace; C.stop=stop;
C.tau_reserve_ratio=double(B.teacher_tau_reserve_ratio);
C.qdd_reserve_ratio=double(B.teacher_qdd_reserve_ratio);
C.refine_offsets=double(B.teacher_refine_command_offsets(:).');

end
