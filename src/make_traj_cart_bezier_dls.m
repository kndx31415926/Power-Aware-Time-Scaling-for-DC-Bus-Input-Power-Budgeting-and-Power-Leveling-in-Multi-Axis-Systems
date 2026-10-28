function TRAJ = make_traj_cart_bezier_dls(robot, q0, qf, limits, cfg)
% Construct a fixed Cartesian path and its joint-space representation.
% Damped least-squares inverse kinematics maps the path to the robot joints;
% the resulting path derivatives support subsequent time scaling.

    try, robot.DataFormat = 'row'; end
    q0 = row(q0); qf = row(qf);
    ee = robot.BodyNames{end};
    tb = trajectory_builder_contract_local(cfg);
    Ts = required_positive_scalar_local(cfg,'TS','cfg.TS');

    % Geometry parameters
    h  = required_finite_scalar_local(cfg,'BEZ_Z_CLEAR','cfg.BEZ_Z_CLEAR');
    d  = required_finite_scalar_local(cfg,'BEZ_LAT','cfg.BEZ_LAT');
    k1 = required_finite_scalar_local(cfg,'BEZ_K1','cfg.BEZ_K1');
    k2 = required_finite_scalar_local(cfg,'BEZ_K2','cfg.BEZ_K2');

    % IK / smoothing defaults.  IK iterations are numerical solver steps,
    % not physical time steps, so they must not be clipped by qd_max*Ts.
    % Physical qd/qdd limits are enforced later by the time-scaling/controller.
    lam0 = tb.dls_lambda;
    wori = tb.dls_w_ori;
    qstep_deg = tb.dls_qstep_deg;
    pos_tol   = tb.dls_pos_tol_m;
    innerMax  = tb.dls_inner_max;
    lineMax   = tb.dls_linesearch_max;
    postureGain = tb.dls_posture_gain;

    % ---- start/end EE positions ----
    T0 = getTransform(robot, q0, ee);
    Tf = getTransform(robot, qf, ee);
    p0 = T0(1:3,4);
    pf = Tf(1:3,4);

    % A yaw + planar-pitch arm has two Cartesian IK representations:
    % (q1,rho) and (q1+pi,-rho), where rho is the signed horizontal reach
    % in the yaw-body +X direction.  A continuous exact-endpoint joint path
    % cannot change the sign of rho without a dedicated self-motion through
    % the yaw axis.  This generator does not contain such a singular bridge.
    % Detect the topology before running hundreds of IK/spline solves.  The
    % former seed-2 case is rho0>0, rhof<0; independent IK therefore jumped
    % J1 by about pi and every smoother left the Cartesian path by 0.134 m.
    branch_check = spinn3d_endpoint_yaw_branch_check(robot,q0,qf,ee);
    rho0 = branch_check.rho0_m;
    rhof = branch_check.rhof_m;
    yaw_guard_available = branch_check.available;
    if yaw_guard_available && ~branch_check.compatible
        error('make_traj_cart_bezier_dls:BranchTopology', ...
            ['Endpoint IK yaw branches are incompatible with this trajectory ', ...
             'generator (signed reach rho0=%.6f m, rhof=%.6f m). ', ...
             'Resample q0/qf or choose a non-singular target; do not loosen ', ...
             'the Cartesian path tolerance.'],rho0,rhof);
    end
    v  = pf - p0;

    % Lateral direction: horizontal normal to line-of-flight.
    u_lat = cross([0;0;1], v);
    if norm(u_lat) < tb.lateral_degenerate_norm_m
        u_lat = [0;1;0];
    end
    u_lat = u_lat / max(norm(u_lat),tb.lateral_normalization_floor_m);

    P0 = p0;
    P1 = p0 + k1*v + d*u_lat + h*[0;0;1];
    P2 = pf - k2*v + d*u_lat + h*[0;0;1];
    P3 = pf;

    % ---- Centrally selected arc-length spacing ----
    % The active contract uses cosine-eased geometric samples; the separate
    % quintic time law later enforces exact rest-to-rest endpoint kinematics.
    % The Cartesian hard tolerance is 15 mm.  A 1.5-mm IK grid is already
    % ten times finer than that tolerance and avoids solving thousands of
    % nearly identical IK problems for every CEM candidate.
    ws_cfg = required_struct_local(cfg,'WORKSPACE','cfg.WORKSPACE');
    step_mm = tb.ik_path_step_mm;
    PP = sample_bezier_cubic(P0,P1,P2,P3,tb);
    M  = size(PP,2);

    % ---- DLS IK (position + mild orientation regularizer) ----
    Q = zeros(M, numel(q0));
    Q(1,:) = q0;
    [qmin,qmax] = robot_joint_limits(robot,numel(q0));
    ik_err = zeros(M,1);

    for k = 2:M
        ptgt = PP(:,k);
        uk = (k-1)/max(M-1,1);
        hk = 10*uk^3-15*uk^4+6*uk^5;
        qpost = q0 + hk*(qf-q0);
        opt = struct('lambda',lam0,'wori',wori,'qstep_deg',qstep_deg, ...
            'pos_tol',pos_tol,'inner_max',innerMax,'line_max',lineMax, ...
            'posture_gain',postureGain, ...
            'retry_without_posture',tb.dls_retry_without_posture, ...
            'line_initial_scale',tb.linesearch_initial_scale, ...
            'line_shrink',tb.linesearch_shrink_factor, ...
            'line_min_abs_improvement', ...
                tb.linesearch_min_absolute_improvement_m, ...
            'line_min_rel_improvement', ...
                tb.linesearch_min_relative_improvement);
        [q,ik_err(k)] = project_one_point(robot,ee,Q(k-1,:),ptgt,qpost, ...
            qmin,qmax,opt);
        Q(k,:) = q;
    end

    % ---- Explicit centrally selected filtering ----
    % A selected toolbox algorithm is never replaced based on installation.
    [Q,filter_meta] = apply_initial_filter_local(Q,tb);

    % Smooth endpoint correction.  Position-only IK may converge to a
    % different redundant wrist solution, and both smoothing filters alter
    % endpoints.  A quintic correction enforces the requested joint states
    % without introducing endpoint velocity/acceleration discontinuities.
    u = linspace(0,1,M).';
    h5 = 10*u.^3 - 15*u.^4 + 6*u.^5;
    Q = Q + (1-h5)*(q0-Q(1,:)) + h5*(qf-Q(end,:));
    Q(1,:) = q0;
    Q(end,:) = qf;

    % Smoothing and endpoint correction both perturb Cartesian position.
    % Treat the corrected curve as a seed, then project every interior
    % sample back onto the requested Bézier path.  This preserves q0/qf and
    % avoids hiding an infeasible solver result behind a loose tolerance.
    proj_opt = struct( ...
        'lambda',tb.reprojection_lambda, ...
        'wori',tb.reprojection_w_ori, ...
        'qstep_deg',tb.reprojection_qstep_deg, ...
        'pos_tol',tb.reprojection_pos_tol_m, ...
        'inner_max',tb.reprojection_inner_max, ...
        'line_max',tb.reprojection_linesearch_max, ...
        'posture_gain',tb.reprojection_posture_gain, ...
        'retry_without_posture',tb.dls_retry_without_posture, ...
        'line_initial_scale',tb.linesearch_initial_scale, ...
        'line_shrink',tb.linesearch_shrink_factor, ...
        'line_min_abs_improvement', ...
            tb.linesearch_min_absolute_improvement_m, ...
        'line_min_rel_improvement', ...
            tb.linesearch_min_relative_improvement);
    reproj_err = zeros(M,1);
    for k = 2:M-1
        % Reproject each already-smoothed sample independently.  Chaining
        % Q(k-1,:) here makes one constrained/singular IK miss propagate to

        % away from the Cartesian path regardless of spline knot density.
        qseed = Q(k,:);
        [Q(k,:),reproj_err(k)] = project_one_point(robot,ee,qseed, ...
            PP(:,k),qseed,qmin,qmax,proj_opt);
    end
    Q(1,:) = q0;
    Q(end,:) = qf;
    dQraw = atan2(sin(diff(Q,1,1)),cos(diff(Q,1,1)));
    [raw_jump_max,raw_jump_linear_index] = max(abs(dQraw),[],'all','linear');
    [raw_jump_sample,raw_jump_joint] = ind2sub(size(dQraw),raw_jump_linear_index);
    raw_jump_limit = deg2rad(tb.raw_branch_jump_limit_deg);
    if logical(getdef(ws_cfg,'verbose_geometry',false))
        fprintf(['[traj-reprojection] mode=independent-per-point | max error=%.3f mm ', ...
                 '| max adjacent jump=%.3f deg (sample %d->%d, J%d)\n'], ...
            1e3*max(reproj_err),rad2deg(raw_jump_max),raw_jump_sample, ...
            raw_jump_sample+1,raw_jump_joint);
    end
    if raw_jump_max > raw_jump_limit
        error('make_traj_cart_bezier_dls:IKBranchDiscontinuity', ...
            ['Independent IK changed branch between samples %d and %d at J%d ', ...
             '(wrapped jump %.3f deg > %.3f deg). A joint-space smoother ', ...
             'cannot repair this without leaving the Cartesian path.'], ...
            raw_jump_sample,raw_jump_sample+1,raw_jump_joint, ...
            rad2deg(raw_jump_max),rad2deg(raw_jump_limit));
    end

    % Independent per-point IK projection is intentionally followed by a
    % geometric smoothing/decimation stage.  Differentiating the projected
    % samples directly at Ts=1 ms turns sub-millimetre IK tolerance jitter
    % into unrealistically large qdd spikes.  An adaptively sampled C2 cubic
    % spline removes that high-frequency component without acceleration jumps,
    % while Cartesian, work-plane and joint checks remain hard constraints.
    [pp_path,Qgeom,geom_validation,geom_meta] = ...
        build_smooth_geometric_path_local( ...
            robot,Q,cfg,tb,PP.',q0,qf,step_mm,qmin,qmax);

    % Analytic q(u(t)) time parameterization.  The quintic progress law has
    % zero velocity and acceleration at both endpoints.  Total duration is
    % enlarged from the actual path derivatives until qd/qdd are below the
    % configured limits; limits are never relaxed.
    [Ttime,Qtime,dQtime,ddQtime,time_meta,pp_d,pp_dd] = ...
        time_parameterize_path_local( ...
            pp_path,Ts,limits,tb,q0,qf,qmin,qmax);
    cfg_timecheck = cfg;
    cfg_timecheck.CAPS = limits; % validate the exact limits used for timing
    validation = spinn3d_validate_trajectory(robot,Qtime,cfg_timecheck,[],dQtime,ddQtime);
    % The dense geometric check and the time-sampled kinematic check describe

    % validation record so existing report code does not see a misleading NaN.
    validation.path_error_max_m = geom_validation.path_error_max_m;
    validation.geometry_min_z_m = geom_validation.min_z_m;
    validation.min_z_m = min(validation.min_z_m,geom_validation.min_z_m);

    TRAJ.Tf   = Ttime;
    TRAJ.Ts   = Ts;
    TRAJ.time = (0:size(Qtime,1)-1).' * Ts;
    TRAJ.eval = @(t) eval_time_scaled_path_local(t,pp_path,pp_d,pp_dd,Ttime,q0,qf);
    TRAJ.Q    = Qtime;
    TRAJ.dQ   = dQtime;
    TRAJ.ddQ  = ddQtime;
    TRAJ.meta = struct('trajectory_contract_id',tb.contract_id, ...
                       'theta', [h d k1 k2], 'ctrl_pts', [P0 P1 P2 P3], ...
                       'trajectory_builder',tb, ...
                       'filter_mode',filter_meta.mode, ...
                       'filter_sgolay_applied',filter_meta.sgolay_applied, ...
                       'validation',validation, ...
                       'geometry_validation',geom_validation, ...
                       'geometry_samples',size(Qgeom,1), ...
                       'geometry_knots',geom_meta.knots, ...
                       'geometry_smoothing_half_window',geom_meta.smoothing_half_window, ...
                       'geometry_validation_step_mm',geom_meta.validation_step_mm, ...
                       'geometry_curve_type',geom_meta.curve_type, ...
                       'geometry_control_clip_max_rad',geom_meta.control_clip_max_rad, ...
                       'geometry_node_fit_error_max_rad',geom_meta.node_fit_error_max_rad, ...
                       'geometry_joint_limit_pre_violation_rad',geom_meta.joint_limit_pre_violation_rad, ...
                       'geometry_joint_limit_post_violation_rad',geom_meta.joint_limit_post_violation_rad, ...
                       'geometry_raw_fallback_used',geom_meta.raw_fallback_used, ...
                       'ik_path_step_mm',step_mm, ...
                       'time_parameterization',time_meta, ...
                       'ik_error_max_m',max(ik_err), ...
                       'reprojection_error_max_m',max(reproj_err), ...
                       'raw_reprojection_path_error_max_m',max(reproj_err), ...
                       'reprojection_mode',tb.reprojection_mode, ...
                       'raw_adjacent_joint_jump_max_rad',raw_jump_max, ...
                       'raw_adjacent_joint_jump_sample',raw_jump_sample, ...
                       'raw_adjacent_joint_jump_joint',raw_jump_joint, ...
                       'yaw_signed_reach_start_m',rho0, ...
                       'yaw_signed_reach_end_m',rhof, ...
                       'q0_error_rad',max(abs(Qtime(1,:)-q0)), ...
                       'qf_error_rad',max(abs(Qtime(end,:)-qf)));
end

function [q,best_err] = project_one_point(robot,ee,qseed,ptgt,qpost,qmin,qmax,opt)
% Robust damped least-squares projection with monotone backtracking.
% qpost only acts in the position-task null space.
q = min(max(row(qseed),qmin),qmax);
qpost = row(qpost);
best_q = q;
best_err = position_error(robot,ee,q,ptgt);
nJ = numel(q);

for it = 1:opt.inner_max
    Tnow = getTransform(robot,q,ee);
    dp = ptgt-Tnow(1:3,4);
    err0 = norm(dp);
    if err0 <= opt.pos_tol, break; end

    J = geometricJacobian(robot,q,ee);
    Jlin = J(4:6,:);
    lam = opt.lambda;
    Jpinv = Jlin.'/(Jlin*Jlin.'+(lam^2)*eye(3));

    if opt.wori > 0
        Jaug = [Jlin; opt.wori*J(1:3,:)];
        daug = [dp; zeros(3,1)];
        dqtask = (Jaug.'*((Jaug*Jaug.'+(lam^2)*eye(6))\daug)).';
    else
        dqtask = (Jpinv*dp).';
    end
    Nnull = eye(nJ)-Jpinv*Jlin;
    dq = dqtask + opt.posture_gain*(qpost-q)*Nnull;
    dq = cap_step(dq,deg2rad(opt.qstep_deg));

    [accepted,qn,enew] = backtrack_step( ...
        robot,ee,q,dq,ptgt,err0,qmin,qmax,opt);
    if ~accepted && opt.posture_gain > 0 && opt.retry_without_posture
        % Near a singularity the posture term may oppose path correction.
        % Retry the same iteration using only the primary position task.
        dq = cap_step((Jpinv*dp).',deg2rad(opt.qstep_deg));
        [accepted,qn,enew] = backtrack_step( ...
            robot,ee,q,dq,ptgt,err0,qmin,qmax,opt);
    end
    if ~accepted, break; end

    q = qn;
    if enew < best_err
        best_err = enew;
        best_q = q;
    end
end
q = best_q;
end

function [ok,qn,enew] = backtrack_step( ...
        robot,ee,q,dq,ptgt,err0,qmin,qmax,opt)
ok = false; qn = q; enew = err0;
scale = opt.line_initial_scale;
for ls = 1:opt.line_max
    qcand = unwrap_to_near(q,q+scale*dq);
    qcand = min(max(qcand,qmin),qmax);
    ecand = position_error(robot,ee,qcand,ptgt);
    min_improvement = max(opt.line_min_abs_improvement, ...
        opt.line_min_rel_improvement*err0);
    if ecand <= opt.pos_tol || ecand < err0-min_improvement
        ok = true; qn = qcand; enew = ecand; return;
    end
    scale = opt.line_shrink*scale;
end
end

function e = position_error(robot,ee,q,ptgt)
T = getTransform(robot,q,ee);
e = norm(ptgt-T(1:3,4));
end

function dq = cap_step(dq,cap)
if ~isfinite(cap) || cap <= 0, return; end
dq = dq/max(1,max(abs(dq))/cap);
end

function [qmin,qmax] = robot_joint_limits(robot,nJ)
qmin = -inf(1,nJ); qmax = inf(1,nJ); jj = 0;
for ib = 1:numel(robot.Bodies)
    body = robot.Bodies{ib};
    if ~strcmpi(body.Joint.Type,'fixed')
        jj = jj+1;
        if jj <= nJ && numel(body.Joint.PositionLimits) == 2
            qmin(jj) = body.Joint.PositionLimits(1);
            qmax(jj) = body.Joint.PositionLimits(2);
        end
    end
end
end

function [pp,Qcheck,report,meta] = build_smooth_geometric_path_local( ...
        robot,Qraw,cfg,tb,target_xyz,q0,qf,ik_step_mm,qmin,qmax)
% Reduce independent-IK sample jitter without weakening Cartesian checks.
M = size(Qraw,1);
ug = linspace(0,1,M);
ws_cfg = required_struct_local(cfg,'WORKSPACE','cfg.WORKSPACE');
max_knots = tb.max_geometry_knots;
max_knots = max(4,min(M,round(max_knots)));
candidate_knots = unique(min(max_knots, ...
    [tb.geometry_candidate_knot_grid max_knots]),'stable');
candidate_knots = candidate_knots(candidate_knots >= 4);

% The coarser grid accelerates IK, not physical validation.  Re-sample both
% the analytic joint spline and the requested Cartesian curve at <=0.5 mm
% before applying the existing hard validator.
validation_step_mm = tb.cartesian_validation_step_mm;
validation_factor = max(1,ceil(double(ik_step_mm)/double(validation_step_mm)));
Mcheck = max(M,validation_factor*(M-1)+1);
ug_check = linspace(0,1,Mcheck);
target_check = interp1(ug(:),target_xyz,ug_check(:), ...
    tb.target_resample_interp_method);
verbose_geometry = logical(getdef(ws_cfg,'verbose_geometry',false));

last_error = [];
for ic = 1:numel(candidate_knots)
    nk = candidate_knots(ic);
    idx = unique(round(linspace(1,M,nk)));
    nk = numel(idx);

    % Every candidate is genuinely smoothed.  The previous last candidate
    % used every raw IK sample with halfwin=0 and was selected
    % unconditionally; that retained IK branch jitter and produced a
    % 6262-second trajectory in the reported seed-2 task.
    halfwin = max(tb.geometry_smoothing_halfwindow_min, ...
        round(tb.geometry_smoothing_halfwindow_scale*M/max(nk-1,1)));
    Qwork = Qraw;
    switch tb.geometry_filter_mode
        case 'movmean'
            for ipass = 1:tb.geometry_smoothing_passes
                Qwork = movmean(Qwork,[halfwin halfwin],1);
            end
        otherwise
            error('make_traj_cart_bezier_dls:BadBuilderEnum', ...
                'Unsupported trajectory_builder.geometry_filter_mode: %s.', ...
                tb.geometry_filter_mode);
    end
    Qwork(1,:) = q0;
    Qwork(end,:) = qf;

    % An open-uniform cubic B-spline does not pass through its controls.
    % Its controls correspond to Greville abscissae, not to a uniform
    % linspace grid.  The previous code fed uniform samples as controls;
    % this parameter mismatch displaced the curve by 0.1--0.6 rad and kept
    % Cartesian error near 20 mm even as the knot count increased.  Sample
    % the smoothed joint path at the correct abscissae.  Linear sampling is
    % convex, so in-limit Qwork produces in-limit controls.
    unodes = open_uniform_cubic_greville_local(nk);
    Qnodes = interp1(ug(:),Qwork,unodes(:), ...
        tb.geometry_control_interp_method);
    Qnodes(1,:) = q0;
    Qnodes(end,:) = qf;
    % An interpolating not-a-knot cubic is C2 but can overshoot a joint limit
    % even when every node is valid.  Build a clamped-end uniform cubic
    % B-spline instead.  It remains C2, and its convex-hull property keeps the
    % complete analytic curve inside the limits of its valid control points.
    [pp_try,Q_try,limit_meta] = joint_limit_safe_c2_bspline_local( ...
        Qnodes,ug_check,q0,qf,qmin,qmax,tb);

    try
        report_try = spinn3d_validate_trajectory(robot,Q_try,cfg,target_check);
        % Knot counts are ordered from smooth/coarse to dense.  Return the
        % first candidate that passes the configured *hard* 15-mm Cartesian,
        % work-plane and joint checks.  There is no hidden 0.75*tolerance
        % preference and no raw-path escape hatch.
        pp = pp_try;
        Qcheck = Q_try;
        report = report_try;
        meta = struct('knots',nk,'smoothing_half_window',halfwin, ...
                      'candidate_index',ic, ...
                      'candidate_count',numel(candidate_knots), ...
                      'validation_step_mm',double(ik_step_mm)/validation_factor, ...
                      'curve_type',limit_meta.curve_type, ...
                      'control_clip_max_rad',limit_meta.control_clip_max_rad, ...
                      'node_fit_error_max_rad',limit_meta.node_fit_error_max_rad, ...
                      'joint_limit_pre_violation_rad',limit_meta.pre_violation_rad, ...
                      'joint_limit_post_violation_rad',limit_meta.post_violation_rad, ...
                      'raw_fallback_used',false);
        if verbose_geometry
            fprintf(['[traj-geometry] ACCEPT knots=%d halfwin=%d fit=%.4g rad ', ...
                     'path=%.3f mm\n'],nk,halfwin,limit_meta.node_fit_error_max_rad, ...
                    1e3*report_try.path_error_max_m);
        end
        return;
    catch ME
        last_error = ME;
        if verbose_geometry
            fprintf(2,['[traj-geometry] reject knots=%d halfwin=%d fit=%.4g rad ', ...
                       '| %s: %s\n'],nk,halfwin,limit_meta.node_fit_error_max_rad, ...
                       ME.identifier,ME.message);
        end
        recoverable = any(strcmp(ME.identifier,{ ...
            'spinn3d_validate_trajectory:PathError', ...
            'spinn3d_validate_trajectory:Workplane', ...
            'spinn3d_validate_trajectory:JointLimit'}));
        if ~recoverable, rethrow(ME); end
    end
end

if ~isempty(last_error), rethrow(last_error); end
error('make_traj_cart_bezier_dls:GeometrySmoothing', ...
    ['Unable to construct a valid smoothed geometric path. The raw ', ...
     'per-point IK fallback is disabled because it can create pathological ', ...
     'curvature and multi-million-sample trajectories.']);
end

function [pp,Qeval,meta] = joint_limit_safe_c2_bspline_local( ...
        Qnodes,ueval,q0,qf,qmin,qmax,tb)
% Construct a base-MATLAB piecewise-polynomial representation of an
% open-uniform clamped cubic B-spline.  Clamped end knots give exact endpoint
% positions.  Its nonnegative basis weights sum to one, so every segment
% lies in the convex hull of in-limit control points.  Endpoint rest is
% supplied later by the quintic time law, not by altering this curve.
ueval = double(ueval(:).');
q0 = row(q0); qf = row(qf);
qmin = row(qmin); qmax = row(qmax);
nJ = numel(q0);
if size(Qnodes,2) ~= nJ || size(Qnodes,1) < 2
    error('make_traj_cart_bezier_dls:BadSplineNodes', ...
        'C2 B-spline requires at least two finite joint-space nodes.');
end
if any(~isfinite(Qnodes),'all')
    error('make_traj_cart_bezier_dls:BadSplineNodes', ...
        'C2 B-spline nodes contain NaN or Inf.');
end

node_violation = max([max(qmin-Qnodes,[],'all'), ...
                      max(Qnodes-qmax,[],'all'),0]);
if node_violation > tb.geometry_joint_limit_tolerance_rad
    error('make_traj_cart_bezier_dls:SplineNodeJointLimit', ...
        'B-spline control nodes exceed a joint limit by %.6g rad.',node_violation);
end
% Do not clip controls or evaluated samples.  Any violation remains visible
% to this check and to spinn3d_validate_trajectory.
Qnodes(1,:) = q0;
Qnodes(end,:) = qf;

% sampled at the Greville locations by convex linear interpolation, it must
% be zero (up to roundoff); no post-hoc trajectory clipping is used.
pre_violation = node_violation;

% Use the standard open-uniform clamped knot vector.  The previous repeated-
% endpoint uniform construction had N+1 spans for N controls and shifted the
% curve by roughly one node.  A collocation inverse for that shifted basis was
% ill-conditioned (the reported fit error was about 4 rad).  The clamped
% B-spline has N-3 physical spans, exact endpoints, aligned Greville locations,
% and preserves the convex-hull joint-limit guarantee without any inverse.
[pp,bs_meta] = open_uniform_cubic_bspline_pp_local(Qnodes);
nseg = bs_meta.segments;
Qends = pp_rows_local(pp,[0 1]);
endpoint_error = max(abs([Qends(1,:)-q0,Qends(2,:)-qf]));
if endpoint_error > tb.geometry_endpoint_tolerance_rad
    error('make_traj_cart_bezier_dls:BSplineEndpoint', ...
        'C2 B-spline endpoint construction error is %.6g rad.',endpoint_error);
end
Qeval = pp_rows_local(pp,ueval);
Qeval(1,:) = q0;
Qeval(end,:) = qf;
post_violation = max([max(qmin-Qeval,[],'all'), ...
                      max(Qeval-qmax,[],'all'),0]);
if post_violation > tb.geometry_joint_limit_tolerance_rad
    error('make_traj_cart_bezier_dls:BSplineJointLimit', ...
        'Convex C2 B-spline exceeds a joint limit by %.6g rad.',post_violation);
end

meta = struct('curve_type','open-uniform-clamped-cubic-bspline-greville-convex-c2', ...
              'pre_violation_rad',pre_violation, ...
              'post_violation_rad',post_violation, ...
              'control_clip_max_rad',0, ...
              'node_fit_error_max_rad',bs_meta.node_fit_error_max_rad, ...
              'active_control_count',0, ...
              'endpoint_error_rad',endpoint_error,'segments',nseg);
end

function [pp,meta] = open_uniform_cubic_bspline_pp_local(P)
% Convert an open-uniform clamped cubic B-spline to base-MATLAB pp form.
% The curve itself is evaluated by de Boor; four values on each nonzero knot
% span recover that span's exact cubic coefficients (up to roundoff).
nctrl = size(P,1);
nJ = size(P,2);
p = 3;
if nctrl < p+1
    error('make_traj_cart_bezier_dls:BSplineControls', ...
        'Open cubic B-spline requires at least four control points.');
end
nseg = nctrl-p;
interior = (1:nseg-1)/nseg;
knots = [zeros(1,p+1),interior,ones(1,p+1)];
breaks = linspace(0,1,nseg+1);
tfit = [0; 1/3; 2/3; 1];
V = [tfit.^3,tfit.^2,tfit,ones(4,1)];
coefs = zeros(nseg*nJ,4);
for iseg = 1:nseg
    a = breaks(iseg);
    b = breaks(iseg+1);
    h = b-a;
    Y = zeros(4,nJ);
    for k = 1:4
        uk = a+h*tfit(k);
        Y(k,:) = deboor_cubic_local(P,knots,uk);
    end
    ct = V\Y;
    rows = (iseg-1)*nJ + (1:nJ);
    coefs(rows,:) = [ct(1,:).'/h^3,ct(2,:).'/h^2, ...
                     ct(3,:).'/h,ct(4,:).'];
end
pp = mkpp(breaks,coefs,nJ);
u_nodes = open_uniform_cubic_greville_local(nctrl);
Qnodes_fit = pp_rows_local(pp,u_nodes);
meta = struct('segments',nseg, ...
              'node_fit_error_max_rad',max(abs(Qnodes_fit-P),[],'all'));
end

function u = open_uniform_cubic_greville_local(nctrl)
% Greville abscissae for the degree-three open-uniform knot vector used by
% open_uniform_cubic_bspline_pp_local.  Control i represents the curve near
% mean(U(i+1:i+3)); using linspace controls instead creates a parameter
% shift, especially near the clamped endpoints.
p = 3;
if ~isscalar(nctrl) || nctrl < p+1 || nctrl ~= round(nctrl)
    error('make_traj_cart_bezier_dls:BSplineControls', ...
        'Open cubic B-spline requires at least four controls.');
end
nseg = nctrl-p;
interior = (1:nseg-1)/nseg;
knots = [zeros(1,p+1),interior,ones(1,p+1)];
u = zeros(1,nctrl);
for i = 1:nctrl
    u(i) = mean(knots(i+1:i+p));
end
u(1) = 0;
u(end) = 1;
end

function q = deboor_cubic_local(P,U,u)
% de Boor evaluation for a degree-three open knot vector.
p = 3;
nctrl = size(P,1);
if u >= U(end)
    q = P(end,:);
    return;
end
kspan = find(U <= u,1,'last');
kspan = min(nctrl,max(p+1,kspan));
d = P(kspan-p:kspan,:);
for r = 1:p
    for j = p:-1:r
        ictrl = kspan-p+j;
        den = U(ictrl+p-r+1)-U(ictrl);
        if den <= 0
            alpha = 0;
        else
            alpha = (u-U(ictrl))/den;
        end
        d(j+1,:) = (1-alpha)*d(j,:) + alpha*d(j+1,:);
    end
end
q = d(p+1,:);
end

function [Tf,Q,Qd,Qdd,meta,pp_d,pp_dd] = ...
        time_parameterize_path_local(pp,Ts,limits,tb,q0,qf,qmin,qmax)
% Globally time-scale an analytic geometric path.  qd scales as 1/T and
% qdd as 1/T^2, so the duration computed below is a hard kinematic bound.
nJ = numel(q0);
qd_lim = positive_limit_local(getdef(limits,'qd_max',[]),nJ,'qd_max');
qdd_lim = positive_limit_local(getdef(limits,'qdd_max',[]),nJ,'qdd_max');

margin = tb.kinematic_time_margin;

pp_d = pp_derivative_local(pp);
pp_dd = pp_derivative_local(pp_d);
[breaks,~,pieces] = unmkpp(pp);
nprobe = max(tb.time_probe_min_count, ...
    tb.time_probe_per_spline_piece*pieces+1);
rbase = linspace(0,1,nprobe).';
ubase = 10*rbase.^3 - 15*rbase.^4 + 6*rbase.^5;

[ubase_unique,iu] = unique(ubase);
rbase_unique = rbase(iu);
break_col = min(1,max(0,breaks(:)));
rbreak = interp1(ubase_unique,rbase_unique,break_col, ...
    tb.time_probe_inverse_interp_method,'extrap');
rbreak(break_col <= 0) = 0;
rbreak(break_col >= 1) = 1;
dr = tb.time_probe_knot_half_step_factor/max(nprobe-1,1);
rprobe = unique([rbase; rbreak; max(0,rbreak-dr); min(1,rbreak+dr)]);
[~,qd_unit,qdd_unit] = eval_time_scaled_path_local(rprobe,pp,pp_d,pp_dd,1.0,q0,qf);
qd_ratio_unit = limit_ratio_local(qd_unit,qd_lim);
qdd_ratio_unit = limit_ratio_local(qdd_unit,qdd_lim);

Tneed = max([Ts, qd_ratio_unit/max(margin,eps), ...
             sqrt(qdd_ratio_unit/max(margin,eps))]);
Tf = max(Ts,ceil((tb.time_initial_safety_factor*Tneed)/Ts)*Ts);

% Derive a conservative ceiling from the physical joint ranges and the
% same qd/qdd limits used for time scaling.  A single pick-and-place spline
% that needs longer than moving every joint through its complete range one
% after another is not a slow valid task; it is an ill-conditioned IK path.
% Apply this before allocating any Nt-by-nJ arrays.
[duration_limit,max_samples,duration_contract] = ...
    trajectory_allocation_contract_local(tb,Ts,q0,qf,qmin,qmax, ...
                                         qd_lim,qdd_lim,margin);
assert_trajectory_allocation_local(Tf,Ts,duration_limit,max_samples);

% Verify the actual uniform samples and enlarge again if roundoff or a

for it = 1:tb.time_refine_max_iterations
    assert_trajectory_allocation_local(Tf,Ts,duration_limit,max_samples);
    Nt = max(2,round(Tf/Ts)+1);
    Tf = (Nt-1)*Ts;
    tgrid = (0:Nt-1).' * Ts;
    [Q,Qd,Qdd] = eval_time_scaled_path_local(tgrid,pp,pp_d,pp_dd,Tf,q0,qf);
    r_qd = limit_ratio_local(Qd,qd_lim);
    r_qdd = limit_ratio_local(Qdd,qdd_lim);
    scale = max([1,r_qd/max(margin,eps),sqrt(r_qdd/max(margin,eps))]);
    if scale <= 1 + tb.time_refine_ratio_tolerance, break; end
    Tf = ceil((tb.time_refine_safety_factor*scale*Tf)/Ts)*Ts;
end

% Always evaluate once more at the final Tf.  In the rare case that the
% eighth iteration enlarges Tf, the arrays from that iteration belong to
% the previous duration and must not be returned as though they matched Tf.
assert_trajectory_allocation_local(Tf,Ts,duration_limit,max_samples);
Nt = max(2,round(Tf/Ts)+1);
Tf = (Nt-1)*Ts;
tgrid = (0:Nt-1).' * Ts;
[Q,Qd,Qdd] = eval_time_scaled_path_local(tgrid,pp,pp_d,pp_dd,Tf,q0,qf);
final_qd_ratio = limit_ratio_local(Qd,qd_lim);
final_qdd_ratio = limit_ratio_local(Qdd,qdd_lim);
if final_qd_ratio > 1+tb.time_limit_tolerance || ...
        final_qdd_ratio > 1+tb.time_limit_tolerance
    error('make_traj_cart_bezier_dls:TimeScaling', ...
        'Uniform trajectory violates qd/qdd limits after time scaling (%.6f, %.6f).', ...
        final_qd_ratio,final_qdd_ratio);
end

% Exact endpoint-rest contract, also removing harmless floating residue.
Q(1,:) = q0; Q(end,:) = qf;
Qd(1,:) = 0; Qd(end,:) = 0;
Qdd(1,:) = 0; Qdd(end,:) = 0;

% Dense continuous-time verification, not only the stored Ts samples.
[~,qd_dense,qdd_dense] = eval_time_scaled_path_local(Tf*rprobe,pp,pp_d,pp_dd,Tf,q0,qf);
dense_qd_ratio = limit_ratio_local(qd_dense,qd_lim);
dense_qdd_ratio = limit_ratio_local(qdd_dense,qdd_lim);
if dense_qd_ratio > 1+tb.time_limit_tolerance || ...
        dense_qdd_ratio > 1+tb.time_limit_tolerance
    error('make_traj_cart_bezier_dls:TimeScaling', ...
        'Continuous trajectory violates qd/qdd limits after time scaling (%.6f, %.6f).', ...
        dense_qd_ratio,dense_qdd_ratio);
end

meta = struct('method','c2-cubic-spline-plus-quintic-time-scaling', ...
              'Tf_s',Tf,'samples',size(Q,1),'limit_margin',margin, ...
              'duration_limit_s',duration_limit, ...
              'max_stored_samples',max_samples, ...
              'duration_contract',duration_contract, ...
              'qd_ratio_max',limit_ratio_local(Qd,qd_lim), ...
              'qdd_ratio_max',limit_ratio_local(Qdd,qdd_lim), ...
              'qd_ratio_dense_max',dense_qd_ratio, ...
              'qdd_ratio_dense_max',dense_qdd_ratio, ...
              'endpoint_rest',true);
end

function [max_Tf,max_samples,contract] = trajectory_allocation_contract_local( ...
        tb,Ts,q0,qf,qmin,qmax,qd_lim,qdd_lim,margin)
% Quintic progress constants: max h'=15/8 and max |h''|=10/sqrt(3).
hpmax = 15/8;
hppmax = 10/sqrt(3);

span = double(qmax(:).'-qmin(:).');
fallback_span = max(abs(row(qf)-row(q0)), ...
    tb.allocation_fallback_span_min_rad);
bad = ~isfinite(span) | span <= 0;
span(bad) = fallback_span(bad);

Tvel = hpmax*span ./ max(margin*qd_lim,eps);
Tacc = sqrt(hppmax*span ./ max(margin*qdd_lim,eps));
full_range_sequential_s = ...
    tb.allocation_full_range_safety_factor*sum(max(Tvel,Tacc));
full_range_sequential_s = max(Ts,ceil(full_range_sequential_s/Ts)*Ts);

absolute_max_s = tb.max_trajectory_duration_s;
max_Tf = min(double(absolute_max_s),full_range_sequential_s);
max_Tf = max(Ts,floor(max_Tf/Ts)*Ts);

max_samples = tb.max_trajectory_samples;

contract = struct('kind','full-joint-range-sequential-quintic-v1', ...
    'quintic_hdot_max',hpmax,'quintic_hddot_max',hppmax, ...
    'joint_span_rad',span, ...
    'full_range_sequential_s',full_range_sequential_s, ...
    'absolute_max_s',double(absolute_max_s));
end

function assert_trajectory_allocation_local(Tf,Ts,max_Tf,max_samples)
Ns = max(2,round(Tf/Ts)+1);
if ~isfinite(Tf) || Tf > max_Tf+10*eps(max_Tf) || Ns > max_samples
    error('make_traj_cart_bezier_dls:PathologicalDuration', ...
        ['Smoothed path requires Tf=%.6g s and %d stored samples at ', ...
         'Ts=%.6g s; the physical allocation contract permits at most ', ...
         '%.6g s and %d samples. The task is rejected before allocation; ', ...
         'this normally indicates an IK branch change or residual jitter.'], ...
        Tf,Ns,Ts,max_Tf,max_samples);
end
end

function [q,dq,ddq] = eval_time_scaled_path_local(t,pp,pp_d,pp_dd,Tf,q0,qf)
% Analytic composition q(u(t)); q, qd and qdd are mutually consistent.
tcol = double(t(:));
r = min(1,max(0,tcol/max(Tf,eps)));
u = 10*r.^3 - 15*r.^4 + 6*r.^5;
udot = (30*r.^2 - 60*r.^3 + 30*r.^4)/max(Tf,eps);
uddot = (60*r - 180*r.^2 + 120*r.^3)/max(Tf,eps)^2;

q = pp_rows_local(pp,u.');
qu = pp_rows_local(pp_d,u.');
quu = pp_rows_local(pp_dd,u.');
dq = bsxfun(@times,qu,udot);
ddq = bsxfun(@times,quu,udot.^2) + bsxfun(@times,qu,uddot);

at0 = (r <= 0);
at1 = (r >= 1);
if any(at0)
    q(at0,:) = repmat(row(q0),nnz(at0),1);
    dq(at0,:) = 0; ddq(at0,:) = 0;
end
if any(at1)
    q(at1,:) = repmat(row(qf),nnz(at1),1);
    dq(at1,:) = 0; ddq(at1,:) = 0;
end
end

function V = pp_rows_local(pp,x)
% ppval returns dimension-by-sample for a vector-valued piecewise polynomial.
x = double(x(:).');
Vraw = ppval(pp,x);
V = reshape(Vraw,[],numel(x)).';
end

function ppd = pp_derivative_local(pp)
[breaks,coefs,~,order,dim] = unmkpp(pp);
if order <= 1
    coefs_d = zeros(size(coefs,1),1);
else
    coefs_d = bsxfun(@times,coefs(:,1:order-1),order-1:-1:1);
end
ppd = mkpp(breaks,coefs_d,dim);
end

function lim = positive_limit_local(lim,nJ,name)
if isempty(lim)
    error('make_traj_cart_bezier_dls:MissingLimit','limits.%s is required.',name);
end
lim = double(lim(:).');
if isscalar(lim), lim = repmat(lim,1,nJ); end
if numel(lim) ~= nJ || any(isnan(lim)) || any(lim <= 0)
    error('make_traj_cart_bezier_dls:BadLimit', ...
        'limits.%s must be a positive scalar or %d-element vector.',name,nJ);
end
end

function r = limit_ratio_local(X,lim)
finite = isfinite(lim);
if any(finite)
    R = bsxfun(@rdivide,abs(X(:,finite)),lim(finite));
    r = max(R,[],'all');
else
    r = 0;
end
if isempty(r), r = 0; end
end

function [Q,meta] = apply_initial_filter_local(Q,tb)
% Apply exactly the centrally selected filter chain.  In particular, a
% missing Signal Processing Toolbox never changes the algorithm silently.
mode = tb.filter_mode;
uses_movmean = any(strcmp(mode,{'movmean','movmean_then_sgolay'}));
uses_sgolay = any(strcmp(mode,{'sgolay','movmean_then_sgolay'}));
if uses_movmean && ~function_available_local('movmean')
    error('make_traj_cart_bezier_dls:MissingSelectedFilter', ...
        ['trajectory_builder.filter_mode=''%s'' requires movmean, ', ...
         'but movmean is unavailable in this MATLAB installation.'],mode);
end
if uses_sgolay && ~function_available_local('sgolayfilt')
    error('make_traj_cart_bezier_dls:MissingSelectedFilter', ...
        ['trajectory_builder.filter_mode=''%s'' requires sgolayfilt ', ...
         '(Signal Processing Toolbox), but it is unavailable. Change ', ...
         'PB.trajectory_builder.filter_mode explicitly or install the ', ...
         'selected dependency.'],mode);
end

movmean_applied = false;
if uses_movmean && size(Q,1) >= tb.movmean_min_samples
    for ipass = 1:tb.movmean_passes
        Q = movmean(Q, ...
            [tb.movmean_half_window tb.movmean_half_window],1);
    end
    movmean_applied = tb.movmean_passes > 0;
end

sgolay_applied = false;
if uses_sgolay
    if size(Q,1) >= tb.sgolay_min_samples
        for j = 1:size(Q,2)
            Q(:,j) = sgolayfilt(Q(:,j), ...
                tb.sgolay_order,tb.sgolay_frame_length);
        end
        sgolay_applied = true;
    elseif strcmp(tb.sgolay_short_path_action,'error')
        error('make_traj_cart_bezier_dls:SelectedFilterPathTooShort', ...
            ['The selected Savitzky-Golay filter requires at least %d ', ...
             'path samples, but the path contains %d.'], ...
            tb.sgolay_min_samples,size(Q,1));
    end
end
meta = struct('mode',mode, ...
    'movmean_applied',movmean_applied, ...
    'sgolay_applied',sgolay_applied);
end

function tf = function_available_local(name)
file_kind = exist(name,'file');
tf = any(file_kind==[2 3 6]) || exist(name,'builtin')==5;
end

function tb = trajectory_builder_contract_local(cfg)
if ~isstruct(cfg) || ~isfield(cfg,'TRAJECTORY_BUILDER') || ...
        ~isstruct(cfg.TRAJECTORY_BUILDER) || ...
        isempty(cfg.TRAJECTORY_BUILDER)
    error('make_traj_cart_bezier_dls:MissingBuilderContract', ...
        ['cfg.TRAJECTORY_BUILDER must be mapped directly from ', ...
         'PB.trajectory_builder.']);
end
tb = cfg.TRAJECTORY_BUILDER;
required = { ...
    'contract_id','ik_path_step_mm','bezier_min_step_mm', ...
    'bezier_arclength_probe_count','bezier_min_samples', ...
    'bezier_spacing_mode','bezier_arc_interp_method', ...
    'lateral_degenerate_norm_m','lateral_normalization_floor_m', ...
    'dls_lambda','dls_w_ori','dls_qstep_deg','dls_pos_tol_m', ...
    'dls_inner_max','dls_linesearch_max','dls_posture_gain', ...
    'dls_posture_progress_mode','dls_retry_without_posture', ...
    'linesearch_initial_scale','linesearch_shrink_factor', ...
    'linesearch_min_absolute_improvement_m', ...
    'linesearch_min_relative_improvement', ...
    'filter_mode','movmean_half_window','movmean_passes', ...
    'movmean_min_samples','sgolay_order','sgolay_frame_length', ...
    'sgolay_min_samples','sgolay_short_path_action', ...
    'endpoint_correction_mode', ...
    'reprojection_mode','reprojection_lambda','reprojection_w_ori', ...
    'reprojection_qstep_deg','reprojection_pos_tol_m', ...
    'reprojection_inner_max','reprojection_linesearch_max', ...
    'reprojection_posture_gain','raw_branch_jump_limit_deg', ...
    'geometry_curve_mode','max_geometry_knots', ...
    'geometry_candidate_knot_grid','cartesian_validation_step_mm', ...
    'target_resample_interp_method', ...
    'geometry_smoothing_halfwindow_scale', ...
    'geometry_smoothing_halfwindow_min', ...
    'geometry_smoothing_passes','geometry_filter_mode', ...
    'geometry_control_interp_method', ...
    'geometry_joint_limit_tolerance_rad', ...
    'geometry_endpoint_tolerance_rad','time_progress_mode', ...
    'kinematic_time_margin', ...
    'time_probe_min_count','time_probe_per_spline_piece', ...
    'time_probe_knot_half_step_factor', ...
    'time_probe_inverse_interp_method','time_initial_safety_factor', ...
    'time_refine_max_iterations','time_refine_ratio_tolerance', ...
    'time_refine_safety_factor','time_limit_tolerance', ...
    'allocation_full_range_safety_factor', ...
    'allocation_fallback_span_min_rad', ...
    'max_trajectory_duration_s','max_trajectory_samples'};
missing = required(~isfield(tb,required));
if ~isempty(missing)
    error('make_traj_cart_bezier_dls:MissingBuilderContract', ...
        'PB.trajectory_builder is missing required field(s): %s.', ...
        strjoin(missing,', '));
end

tb.contract_id = required_text_local(tb,'contract_id');

positive = { ...
    'ik_path_step_mm','bezier_min_step_mm', ...
    'lateral_degenerate_norm_m','lateral_normalization_floor_m', ...
    'dls_lambda','dls_qstep_deg','dls_pos_tol_m', ...
    'linesearch_initial_scale','linesearch_shrink_factor', ...
    'reprojection_lambda','reprojection_qstep_deg', ...
    'reprojection_pos_tol_m','raw_branch_jump_limit_deg', ...
    'cartesian_validation_step_mm','kinematic_time_margin', ...
    'time_initial_safety_factor','time_refine_safety_factor', ...
    'allocation_full_range_safety_factor', ...
    'allocation_fallback_span_min_rad','max_trajectory_duration_s'};
for i = 1:numel(positive)
    required_positive_scalar_local(tb,positive{i}, ...
        ['PB.trajectory_builder.' positive{i}]);
end
nonnegative = { ...
    'dls_w_ori','dls_posture_gain', ...
    'linesearch_min_absolute_improvement_m', ...
    'linesearch_min_relative_improvement','reprojection_w_ori', ...
    'reprojection_posture_gain', ...
    'geometry_smoothing_halfwindow_scale', ...
    'geometry_joint_limit_tolerance_rad', ...
    'geometry_endpoint_tolerance_rad', ...
    'time_probe_knot_half_step_factor', ...
    'time_refine_ratio_tolerance','time_limit_tolerance'};
for i = 1:numel(nonnegative)
    value = required_finite_scalar_local(tb,nonnegative{i}, ...
        ['PB.trajectory_builder.' nonnegative{i}]);
    if value < 0
        error('make_traj_cart_bezier_dls:BadBuilderValue', ...
            'PB.trajectory_builder.%s must be nonnegative.', ...
            nonnegative{i});
    end
end
positive_integer = { ...
    'bezier_arclength_probe_count','bezier_min_samples', ...
    'dls_inner_max','dls_linesearch_max','movmean_min_samples', ...
    'sgolay_frame_length','sgolay_min_samples','reprojection_inner_max', ...
    'reprojection_linesearch_max','max_geometry_knots', ...
    'time_probe_min_count','time_refine_max_iterations', ...
    'max_trajectory_samples'};
for i = 1:numel(positive_integer)
    required_integer_local(tb,positive_integer{i},1);
end
nonnegative_integer = { ...
    'movmean_half_window','movmean_passes','sgolay_order', ...
    'geometry_smoothing_halfwindow_min', ...
    'geometry_smoothing_passes','time_probe_per_spline_piece'};
for i = 1:numel(nonnegative_integer)
    required_integer_local(tb,nonnegative_integer{i},0);
end

tb.bezier_spacing_mode = required_enum_local(tb, ...
    'bezier_spacing_mode',{'cosine_ease','uniform'});
tb.bezier_arc_interp_method = required_enum_local(tb, ...
    'bezier_arc_interp_method',{'linear','pchip'});
tb.filter_mode = required_enum_local(tb,'filter_mode', ...
    {'none','movmean','sgolay','movmean_then_sgolay'});
tb.dls_posture_progress_mode = required_enum_local(tb, ...
    'dls_posture_progress_mode',{'quintic'});
tb.sgolay_short_path_action = required_enum_local(tb, ...
    'sgolay_short_path_action',{'skip_sgolay','error'});
tb.endpoint_correction_mode = required_enum_local(tb, ...
    'endpoint_correction_mode',{'quintic'});
tb.reprojection_mode = required_enum_local(tb,'reprojection_mode', ...
    {'independent_per_point'});
tb.geometry_curve_mode = required_enum_local(tb,'geometry_curve_mode', ...
    {'open_uniform_clamped_cubic_bspline'});
tb.target_resample_interp_method = required_enum_local(tb, ...
    'target_resample_interp_method',{'linear','pchip'});
tb.geometry_filter_mode = required_enum_local(tb, ...
    'geometry_filter_mode',{'movmean'});
tb.geometry_control_interp_method = required_enum_local(tb, ...
    'geometry_control_interp_method',{'linear'});
tb.time_progress_mode = required_enum_local(tb, ...
    'time_progress_mode',{'quintic'});
tb.time_probe_inverse_interp_method = required_enum_local(tb, ...
    'time_probe_inverse_interp_method',{'linear','pchip'});

if ~islogical(tb.dls_retry_without_posture) || ...
        ~isscalar(tb.dls_retry_without_posture)
    error('make_traj_cart_bezier_dls:BadBuilderValue', ...
        ['PB.trajectory_builder.dls_retry_without_posture must be a ', ...
         'logical scalar.']);
end

if tb.ik_path_step_mm < tb.bezier_min_step_mm
    error('make_traj_cart_bezier_dls:BadBuilderValue', ...
        ['PB.trajectory_builder.ik_path_step_mm must be at least ', ...
         'bezier_min_step_mm; the builder will not clamp it privately.']);
end
if tb.bezier_arclength_probe_count < 2 || tb.bezier_min_samples < 2
    error('make_traj_cart_bezier_dls:BadBuilderValue', ...
        'Bezier probe and output grids must each contain at least two points.');
end
if tb.linesearch_shrink_factor <= 0 || ...
        tb.linesearch_shrink_factor >= 1
    error('make_traj_cart_bezier_dls:BadBuilderValue', ...
        'PB.trajectory_builder.linesearch_shrink_factor must lie in (0,1).');
end
if tb.sgolay_frame_length <= tb.sgolay_order || ...
        mod(tb.sgolay_frame_length,2) ~= 1 || ...
        tb.sgolay_frame_length ~= round(tb.sgolay_frame_length)
    error('make_traj_cart_bezier_dls:BadBuilderValue', ...
        ['PB.trajectory_builder.sgolay_frame_length must be an odd ', ...
         'integer greater than sgolay_order.']);
end
if tb.sgolay_min_samples < tb.sgolay_frame_length
    error('make_traj_cart_bezier_dls:BadBuilderValue', ...
        ['PB.trajectory_builder.sgolay_min_samples must be at least ', ...
         'sgolay_frame_length.']);
end
if tb.raw_branch_jump_limit_deg > 180
    error('make_traj_cart_bezier_dls:BadBuilderValue', ...
        'PB.trajectory_builder.raw_branch_jump_limit_deg must not exceed 180.');
end
if tb.max_geometry_knots < 4
    error('make_traj_cart_bezier_dls:BadBuilderValue', ...
        'PB.trajectory_builder.max_geometry_knots must be at least four.');
end
grid_raw = tb.geometry_candidate_knot_grid;
if ~isnumeric(grid_raw) || ~isreal(grid_raw)
    error('make_traj_cart_bezier_dls:BadBuilderValue', ...
        ['PB.trajectory_builder.geometry_candidate_knot_grid must be a ', ...
         'real numeric vector.']);
end
grid = double(grid_raw(:).');
if isempty(grid) || any(~isfinite(grid)) || any(grid < 4) || ...
        any(grid ~= round(grid)) || any(diff(grid) <= 0)
    error('make_traj_cart_bezier_dls:BadBuilderValue', ...
        ['PB.trajectory_builder.geometry_candidate_knot_grid must be a ', ...
         'strictly increasing vector of integers >= 4.']);
end
tb.geometry_candidate_knot_grid = grid;
if tb.kinematic_time_margin >= 1
    error('make_traj_cart_bezier_dls:BadBuilderValue', ...
        'PB.trajectory_builder.kinematic_time_margin must lie in (0,1).');
end
if tb.time_initial_safety_factor < 1 || ...
        tb.time_refine_safety_factor < 1 || ...
        tb.allocation_full_range_safety_factor < 1
    error('make_traj_cart_bezier_dls:BadBuilderValue', ...
        'Trajectory timing/allocation safety factors must be at least one.');
end
if tb.time_probe_min_count < 2 || tb.max_trajectory_samples < 2
    error('make_traj_cart_bezier_dls:BadBuilderValue', ...
        'Time probe and stored-sample limits must each be at least two.');
end
if ~function_available_local('movmean')
    error('make_traj_cart_bezier_dls:MissingSelectedFilter', ...
        ['PB.trajectory_builder.geometry_filter_mode=''movmean'' ', ...
         'requires movmean, but it is unavailable.']);
end
end

function value = required_finite_scalar_local(S,field,label)
if ~isstruct(S) || ~isfield(S,field) || isempty(S.(field))
    error('make_traj_cart_bezier_dls:MissingBuilderContract', ...
        '%s is required.',label);
end
raw = S.(field);
if ~isnumeric(raw) || ~isreal(raw) || ~isscalar(raw)
    error('make_traj_cart_bezier_dls:BadBuilderValue', ...
        '%s must be a real numeric scalar.',label);
end
value = double(raw);
if ~isfinite(value)
    error('make_traj_cart_bezier_dls:BadBuilderValue', ...
        '%s must be a finite scalar.',label);
end
end

function value = required_positive_scalar_local(S,field,label)
value = required_finite_scalar_local(S,field,label);
if value <= 0
    error('make_traj_cart_bezier_dls:BadBuilderValue', ...
        '%s must be positive.',label);
end
end

function value = required_integer_local(S,field,minimum)
label = ['PB.trajectory_builder.' field];
value = required_finite_scalar_local(S,field,label);
if value < minimum || value ~= round(value)
    error('make_traj_cart_bezier_dls:BadBuilderValue', ...
        '%s must be an integer >= %d.',label,minimum);
end
end

function value = required_enum_local(S,field,allowed)
if ~isstruct(S) || ~isfield(S,field) || isempty(S.(field)) || ...
        ~(ischar(S.(field)) || ...
          (isstring(S.(field)) && isscalar(S.(field))))
    error('make_traj_cart_bezier_dls:MissingBuilderContract', ...
        'PB.trajectory_builder.%s must be a nonempty enum.',field);
end
value = lower(strtrim(char(S.(field))));
if ~any(strcmp(value,allowed))
    error('make_traj_cart_bezier_dls:BadBuilderEnum', ...
        'PB.trajectory_builder.%s must be one of: %s.', ...
        field,strjoin(allowed,', '));
end
end

function value = required_text_local(S,field)
if ~isstruct(S) || ~isfield(S,field) || isempty(S.(field)) || ...
        ~(ischar(S.(field)) || ...
          (isstring(S.(field)) && isscalar(S.(field))))
    error('make_traj_cart_bezier_dls:MissingBuilderContract', ...
        'PB.trajectory_builder.%s must be nonempty text.',field);
end
value = strtrim(char(S.(field)));
if isempty(value)
    error('make_traj_cart_bezier_dls:MissingBuilderContract', ...
        'PB.trajectory_builder.%s must be nonempty text.',field);
end
end

function value = required_struct_local(S,field,label)
if ~isstruct(S) || ~isfield(S,field) || ~isstruct(S.(field))
    error('make_traj_cart_bezier_dls:MissingBuilderContract', ...
        '%s must be a struct.',label);
end
value = S.(field);
end

function P = sample_bezier_cubic(P0,P1,P2,P3,tb)
    step = tb.ik_path_step_mm/1000;
    tt = linspace(0,1,tb.bezier_arclength_probe_count);
    B0 = (1-tt).^3; B1 = 3*(1-tt).^2.*tt; B2 = 3*(1-tt).*tt.^2; B3 = tt.^3;
    C = P0*B0 + P1*B1 + P2*B2 + P3*B3;
    ds = vecnorm(diff(C,1,2));
    s = [0, cumsum(ds)];
    L = s(end);
    N = max(tb.bezier_min_samples,ceil(L/step));
    u = linspace(0,1,N);
    switch tb.bezier_spacing_mode
        case 'cosine_ease'
            u_e = 0.5-0.5*cos(pi*u);
        case 'uniform'
            u_e = u;
        otherwise
            error('make_traj_cart_bezier_dls:BadBuilderEnum', ...
                'Unsupported trajectory_builder.bezier_spacing_mode: %s.', ...
                tb.bezier_spacing_mode);
    end
    t_eq = interp1(s./max(L,eps),tt,u_e, ...
        tb.bezier_arc_interp_method,'extrap');
    B0 = (1-t_eq).^3; B1 = 3*(1-t_eq).^2.*t_eq; B2 = 3*(1-t_eq).*t_eq.^2; B3 = t_eq.^3;
    P = P0*B0 + P1*B1 + P2*B2 + P3*B3;
end

function qn = unwrap_to_near(qprev, qcur)
    d = row(qcur) - row(qprev);
    qn = row(qprev) + atan2(sin(d), cos(d));
end

function v = row(v)
    v = v(:)';
end

function val = getdef(S, f, def)
    if isfield(S,f) && ~isempty(S.(f))
        val = S.(f);
    else
        val = def;
    end
end
