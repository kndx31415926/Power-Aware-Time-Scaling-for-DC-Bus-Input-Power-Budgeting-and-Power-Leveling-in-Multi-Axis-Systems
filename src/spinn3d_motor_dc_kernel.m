function e = spinn3d_motor_dc_kernel(tau,dq,c)
% Evaluate the motor electrical model from torque and angular velocity.

drag = c.b_Nm_s.*dq + c.coulomb_Nm_per_servo.* ...
    tanh(dq./c.omega_smooth_rad_s);
e = struct();
e.I_winding_A = (tau./c.n_parallel+drag)./c.K_Nm_A;
e.V_terminal_V = c.R_ohm.*e.I_winding_A+c.K_Nm_A.*dq;
e.Pmechanical_W = tau.*dq;
e.Pcopper_W = c.n_parallel.*c.R_ohm.*e.I_winding_A.^2;
e.Pdrag_W = c.n_parallel.*drag.*dq;
e.Ploss_W = e.Pcopper_W+e.Pdrag_W;
e.Pterminal_W = c.n_parallel.*e.V_terminal_V.*e.I_winding_A;
e.tau_internal_Nm = c.n_parallel.*c.K_Nm_A.*e.I_winding_A;
e.r_current = abs(e.I_winding_A)./c.I_stall_A;
e.r_voltage = abs(e.V_terminal_V)./c.V_bus_V;
e.r_speed = abs(dq)./c.omega_limit_rad_s;
e.envelope_ratio = max(max(e.r_current,e.r_voltage),e.r_speed);
end
