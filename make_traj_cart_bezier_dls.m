function TRAJ = make_traj_cart_bezier_dls(robot, q0, qf, limits, cfg) %#ok<INUSD>
%MAKE_TRAJ_CART_BEZIER_DLS  Compatibility wrapper for the revised geometric path.
%
% Earlier versions returned a time-domain DLS trajectory with TRAJ.Tf=(M-1)*Ts.
% That caused an important mismatch: the TRF/teacher was trained on geometric
% derivatives dq/ds and d2q/ds2, whereas online rollouts used nominal time
% derivatives dq/dt and d2q/dt2.  The revised paper formulation is a genuine
% time-scaling method, so all online controllers now use the same normalized
% geometric path as the dataset generator:
%
%       q = q_g(s),    s in [0,1],    dq_cmd = alpha*q_s,
%       ddq_cmd = alpha^2*q_ss + alpha_dot*q_s.
%
% The fifth input (limits) is retained for old call sites but is not needed by
% the geometric path generator.

    if nargin < 5 || isempty(cfg), cfg = struct(); end
    TRAJ = make_traj_cart_bezier_dls_geom(robot, q0, qf, cfg);
    TRAJ.kind = 'cart-bezier-dls-geom-wrapper';
end
