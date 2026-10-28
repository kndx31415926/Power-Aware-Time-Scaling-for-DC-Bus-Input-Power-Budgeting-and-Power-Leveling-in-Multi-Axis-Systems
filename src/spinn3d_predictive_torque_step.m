function [tau_applied, ddq, dq_mid, audit] = spinn3d_predictive_torque_step( ...
        robot,q,dq,tau_cmd,tau_max,P_axis_max,fric,bus,Ts, ...
        motion_limits,workspace,hard_tolerances)
%SPINN3D_PREDICTIVE_TORQUE_STEP Consistent torque/power clamp for one plant step.
%
% The source-power envelope depends on velocity.  Clamping at the velocity
% from the beginning of a sample and then reporting power at midpoint
% velocity can violate the physical source peak during acceleration.  This
% helper uses midpoint power for energy, but audits current, voltage, speed
% and conservative source draw across the full linear-velocity step. The
% executed torque and plant integration use the same interval. The final
% command is not returned to the caller until the
% noise-perturbed, power-clamped torque has also passed qdd, start/mid/next
% qd, swept next-q, successor joint-stop braking reserve, work-plane,
% payload-box and self-collision checks.  A request that can be made safe is
% projected in acceleration space around the gravity-support torque; an
% unrecoverable step is rejected before commit.

if nargin~=12
    error('spinn3d_predictive_torque_step:MissingFinalSafetyContract', ...
        ['Expected the complete central motion limits, WORKSPACE geometry ', ...
         'and EXECUTION_AUDIT tolerances after Ts.']);
end

q = row_local(q);
dq = row_local(dq);
tau_req = row_local(tau_cmd);
nJ = numel(tau_req);
if numel(q)~=nJ || numel(dq)~=nJ
    error('spinn3d_predictive_torque_step:BadStateDimensions', ...
        'q, dq and tau_cmd must contain the same number of joints.');
end
if ~isreal(q) || ~isreal(dq) || any(~isfinite([q,dq]))
    error('spinn3d_predictive_torque_step:BadState', ...
        'q and dq must be finite real plant-state rows.');
end
if ~isreal(tau_req) || any(~isfinite(tau_req))
    error('spinn3d_predictive_torque_step:BadTorqueRequest', ...
        'The post-noise torque request must be finite and real.');
end
tau_lim = expand_local(tau_max,nJ,inf);
Pax_lim = expand_local(P_axis_max,nJ,inf);
Ts = double(Ts);

bus = spinn3d_bus_defaults(bus);
if isfield(bus,'dc_state'), bus=spinn3d_bus_with_state(bus,bus.dc_state,Ts); end
recovery_cfg = recovery_contract_local(hard_tolerances);

B = expand_local(field_local(fric,'B',0),nJ,0);
Fc = expand_local(field_local(fric,'Fc',0),nJ,0);
ve = max(double(field_local(fric,'vel_eps',1e-3)),1e-9);

Mq = massMatrix(robot,q);
c = row_local(velocityProduct(robot,q,dq));
g = row_local(gravityTorque(robot,q));
tau_fric = B.*dq + Fc.*tanh(dq./ve);
h = c + g + tau_fric;

tau_applied = min(tau_lim,max(-tau_lim,tau_req));
max_iter = 32;
iterations = 0;
converged = false;
metrics = struct();
fast_clamp_error = '';
for it = 1:max_iter
    iterations = it;
    % Predict with the currently retained torque, then clamp that torque at
    % its own midpoint velocity.  Starting each pass from the previous
    % result makes torque magnitude non-increasing and avoids a two-point
    % oscillation between incompatible velocity envelopes.
    ddq = (Mq \ (tau_applied(:)-h(:))).';
    dq_mid = dq + 0.5*Ts*ddq;
    tau_next = min(tau_lim,max(-tau_lim,tau_applied));
    try
        tau_next = clamp_power_axis_total_bus(tau_next,dq_mid,Pax_lim,bus);
    catch MEclamp
        if recoverable_clamp_error_local(MEclamp)
            fast_clamp_error=MEclamp.identifier; break;
        end
        rethrow(MEclamp);
    end
    dtau = norm(tau_next-tau_applied,inf);
    tau_applied = tau_next;

    % Recompute the state after the clamp and validate every hard actuator
    % constraint across the same interval used by the plant integrator.
    ddq = (Mq \ (tau_applied(:)-h(:))).';
    dq_mid = dq + 0.5*Ts*ddq;
    metrics = constraint_metrics_local(tau_applied,dq_mid,tau_lim,Pax_lim,bus,dq,2*dq_mid-dq);
    try
        tau_verify = clamp_power_axis_total_bus(tau_applied,dq_mid,Pax_lim,bus);
    catch MEclamp
        if recoverable_clamp_error_local(MEclamp)
            fast_clamp_error=MEclamp.identifier; break;
        end
        rethrow(MEclamp);
    end
    idempotent = norm(tau_verify-tau_applied,inf) <= ...
        1e-10*max(1,norm(tau_applied,inf));
    if metrics.pass && idempotent && dtau <= ...
            1e-10*max(1,norm(tau_applied,inf))
        converged = true;
        break;
    end
    tau_applied = tau_verify;
end

% because the iteration count expired.
ddq = (Mq \ (tau_applied(:)-h(:))).';
dq_mid = dq + 0.5*Ts*ddq;
metrics = constraint_metrics_local(tau_applied,dq_mid,tau_lim,Pax_lim,bus,dq,2*dq_mid-dq);
motion = final_motion_metrics_local( ...
    robot,q,dq,ddq,dq_mid,Ts,motion_limits,workspace,hard_tolerances);
fast_motion = motion;
fast_actuator_ok = logical(converged && metrics.pass);

% The ordinary torque/power clamp is retained as the fast path.  Its ST3215
% branch scales absolute torque toward zero, however, and zero torque is not
% zero acceleration under gravity.  If that fast command is not completely
% safe, recover in acceleration space around a gravity-supporting anchor.
% Every returned recovery torque is rebuilt as tau=h+M*qdd and is checked at
% its own midpoint velocity; it is never modified by a zero-centred clamp
% after the motion projection.
projection = projection_diagnostic_default_local(fast_motion);
if ~(fast_actuator_ok && motion.pass)
    [candidate,projection] = recover_final_command_local( ...
        robot,Mq,h,tau_req,q,dq,Ts,tau_lim,Pax_lim,bus, ...
        motion_limits,workspace,hard_tolerances,fast_motion,recovery_cfg);
    if candidate.found
        tau_applied = candidate.tau;
        ddq = candidate.ddq;
        dq_mid = candidate.dq_mid;
        metrics = candidate.metrics;
        motion = candidate.motion;
        converged = true;
    elseif projection.bounds_feasible && ~fast_actuator_ok && ...
            ~projection.any_actuator_feasible
        error('spinn3d_predictive_torque_step:NoConsistentMidpointCommand', ...
            ['No actuator-consistent command was found after %d fast-path ', ...
             'iterations and %d acceleration-domain trials: tau ratio=%.9g, ', ...
             'axis-power ratio=%.9g, motor ratio=%.9g, Pgrid=%.9g/%.9g W, ', ...
             'Pdump=%.9g/%.9g W, requested qdd ratio=%.9g; exact ', ...
             'torque-box tested=%d feasible=%d novel=%d, ', ...
             'any actuator-feasible=%d.'], ...
            iterations,projection.trials,metrics.tau_ratio, ...
            metrics.Paxis_ratio,metrics.motor_ratio,metrics.Pgrid_W, ...
            metrics.Pgrid_cap_W,metrics.Pdump_W,metrics.Pdump_cap_W, ...
            projection.requested_qdd_ratio, ...
            projection.exact_torque_box_tested, ...
            projection.exact_torque_box_feasible, ...
            projection.exact_torque_box_novel_candidate, ...
            projection.any_actuator_feasible);
    else
        motion = projection.last_motion;
        geometry_reason = geometry_failure_reason_local(motion.geometry);
        [diag_joint,diag_side,raw_lo,raw_hi,viable_lo,viable_hi, ...
            current_raw_shortage,full_endpoint_raw_shortage, ...
            roundoff_only_endpoint,position_planning_tol, ...
            position_audit_tol] = ...
            projection_viability_diagnostic_local(projection,motion);
        error('spinn3d_predictive_torque_step:FinalAppliedTorqueUnsafe', ...
            ['No acceleration-domain recovery candidate passed the complete ', ...
             'post-noise execution contract after %d trials: requested qdd ', ...
             'ratio=%.9g, last qdd ratio=%.9g, qd ratio=%.9g, q violation=', ...
             '%.9g rad (J%d %s), stopping violation=%.9g rad (J%d %s), ', ...
             'diagnostic viable-qdd J%d %s raw=[%.9g,%.9g], ', ...
             'tight=[%.9g,%.9g], current raw shortage=%.12g rad, ', ...
             'full-brake endpoint raw shortage=%.12g rad, ', ...
             'roundoff-only endpoint=%d, position planning/audit ', ...
             'tolerance=%.12g/%.12g rad, exact torque-box tested=%d feasible=%d ', ...
             'novel=%d, ', ...
             'any actuator-feasible=%d; ', ...
             'workplane ', ...
             'margin=%.9g m, signed self-clearance-to-required-margin=', ...
             '%.9g m; %s.'], ...
            projection.trials,projection.requested_qdd_ratio, ...
            motion.qdd_ratio,motion.qd_ratio,motion.qpos_violation_rad, ...
            motion.qpos_violation_joint,char(motion.qpos_violation_side), ...
            motion.joint_limit_stopping_violation_rad, ...
            motion.joint_limit_viability_joint, ...
            char(motion.joint_limit_viability_side), ...
            diag_joint,char(diag_side),raw_lo,raw_hi,viable_lo,viable_hi, ...
            current_raw_shortage,full_endpoint_raw_shortage, ...
            roundoff_only_endpoint,position_planning_tol, ...
            position_audit_tol, ...
            projection.exact_torque_box_tested, ...
            projection.exact_torque_box_feasible, ...
            projection.exact_torque_box_novel_candidate, ...
            projection.any_actuator_feasible, ...
            motion.geometry.workplane_margin_m, ...
            motion.geometry.min_self_collision_margin_m,geometry_reason);
    end
end

% Authoritative post-clamp/post-projection motion and geometry check.  The
% controller's actual guard is pre-noise; only this location can authorize
% the torque that is submitted to the 1-ms plant integrator.
if ~motion.pass
    geometry_reason = geometry_failure_reason_local(motion.geometry);
    error('spinn3d_predictive_torque_step:FinalAppliedTorqueUnsafe', ...
        ['Final post-noise/post-clamp torque was rejected before state ', ...
         'commit: qdd ratio=%.9g, qd ratio=%.9g, q violation=%.9g rad ', ...
         '(J%d %s), stopping violation=%.9g rad (J%d %s), workplane ', ...
         'margin=%.9g m, ', ...
         'signed self-clearance-to-required-margin=%.9g m; %s.'], ...
        motion.qdd_ratio,motion.qd_ratio,motion.qpos_violation_rad, ...
        motion.qpos_violation_joint,char(motion.qpos_violation_side), ...
        motion.joint_limit_stopping_violation_rad, ...
        motion.joint_limit_viability_joint, ...
        char(motion.joint_limit_viability_side), ...
        motion.geometry.workplane_margin_m, ...
        motion.geometry.min_self_collision_margin_m,geometry_reason);
end

ddq_requested = (Mq \ (tau_req(:)-h(:))).';
[a_lo_progress,a_hi_progress,bounds_progress,qdd_lim_progress, ...
        viability_bounds_progress] = ...
    acceleration_bounds_local(q,dq,Ts,motion_limits,hard_tolerances);
if ~bounds_progress
    error('spinn3d_predictive_torque_step:FinalAppliedTorqueUnsafe', ...
        'The final command cannot be committed because the acceleration box is empty.');
end
[execution_progress_ratio,command_modified,progress_basis] = ...
    spinn3d_execution_progress_ratio(ddq_requested,ddq, ...
        a_lo_progress,a_hi_progress,qdd_lim_progress, ...
        hard_tolerances.ratio_tolerance);

audit = struct('iterations',iterations,'converged',logical(converged), ...
    'Pgrid_mid_W',metrics.Pgrid_mid_W,'Pgrid_peak_W',metrics.Pgrid_cap_W, ...
    'Pgrid_guard_upper_W',metrics.Pgrid_W, ...
    'dynamic_dc_state_pass',metrics.dynamic_dc_state_pass, ...
    'fast_clamp_error_id',fast_clamp_error, ...
    'Pdump_mid_W',metrics.Pdump_W,'Pdump_peak_W',metrics.Pdump_cap_W, ...
    'Paxis_ratio',metrics.Paxis_ratio,'motor_ratio',metrics.motor_ratio, ...
    'tau_ratio',metrics.tau_ratio,'dq_power',dq_mid, ...
    'fast_actuator_converged',fast_actuator_ok, ...
    'motion_projection_applied',logical(projection.used), ...
    'motion_projection_lambda',projection.lambda, ...
    'motion_projection_trials',projection.trials, ...
    'recovery_search_mode',projection.search_mode, ...
    'recovery_anchor_kind',projection.anchor_kind, ...
    'recovery_any_actuator_feasible', ...
        projection.any_actuator_feasible, ...
    'recovery_exact_torque_box_tested', ...
        projection.exact_torque_box_tested, ...
    'recovery_exact_torque_box_feasible', ...
        projection.exact_torque_box_feasible, ...
    'recovery_exact_torque_box_novel_candidate', ...
        projection.exact_torque_box_novel_candidate, ...
    'execution_progress_ratio',execution_progress_ratio, ...
    'execution_command_modified',logical(command_modified), ...
    'execution_progress_basis',progress_basis, ...
    'requested_qdd',ddq_requested,'final_qdd',ddq, ...
    'requested_qdd_ratio',projection.requested_qdd_ratio, ...
    'pre_projection_qdd_ratio',fast_motion.qdd_ratio, ...
    'pre_projection_joint_limit_viability_pass', ...
        fast_motion.joint_limit_viability_pass, ...
    'pre_projection_stopping_violation_rad', ...
        fast_motion.joint_limit_stopping_violation_rad, ...
    'pre_projection_viability_joint', ...
        fast_motion.joint_limit_viability_joint, ...
    'pre_projection_viability_side', ...
        fast_motion.joint_limit_viability_side, ...
    'joint_limit_viability_pass',motion.joint_limit_viability_pass, ...
    'joint_limit_stopping_violation_rad', ...
        motion.joint_limit_stopping_violation_rad, ...
    'joint_limit_viability_joint',motion.joint_limit_viability_joint, ...
    'joint_limit_viability_side',motion.joint_limit_viability_side, ...
    'joint_position_violation_rad',motion.qpos_violation_rad, ...
    'joint_position_violation_joint',motion.qpos_violation_joint, ...
    'joint_position_violation_side',motion.qpos_violation_side, ...
    'acceleration_lower_bound',a_lo_progress, ...
    'acceleration_upper_bound',a_hi_progress, ...
    'successor_viability_acceleration_bounds', ...
        viability_bounds_progress, ...
    'final_motion',motion,'final_state_verified_before_commit',true);
end

function [joint,side,raw_lo,raw_hi,viable_lo,viable_hi, ...
        current_raw_shortage,full_endpoint_raw_shortage, ...
        roundoff_only_endpoint,position_planning_tol, ...
        position_audit_tol] = ...
        projection_viability_diagnostic_local(D,motion)
V = field_local(D,'successor_viability_acceleration_bounds',struct());
infeasible_joint = double(field_local(V,'infeasible_joint',0));
infeasible_side = char(field_local(V,'infeasible_side','none'));
if ~logical(field_local(V,'pass',true)) && ...
        isfinite(infeasible_joint) && infeasible_joint>=1 && ...
        infeasible_joint==round(infeasible_joint)

    % can be a different side and its shortage is not evidence about the
    % saturated endpoint that made the box empty.
    joint = infeasible_joint;
    side = infeasible_side;
else
    % A pure one-step position failure has no braking-viability offender.

    
    joint = double(field_local(motion,'qpos_violation_joint',0));
    side = char(field_local(motion,'qpos_violation_side','none'));
    if ~(isfinite(joint) && joint>=1 && joint==round(joint))
        joint = double(field_local( ...
            motion,'joint_limit_viability_joint',0));
        side = char(field_local( ...
            motion,'joint_limit_viability_side','none'));
        if ~(isfinite(joint) && joint>=1 && joint==round(joint))
            joint = infeasible_joint;
            side = infeasible_side;
        end
    end
end
raw_lower = double(field_local(V,'incoming_lower_bound',nan));
raw_upper = double(field_local(V,'incoming_upper_bound',nan));
tight_lower = double(field_local(V,'lower_bound',field_local(D,'a_lo',nan)));
tight_upper = double(field_local(V,'upper_bound',field_local(D,'a_hi',nan)));
current_lower = double(field_local(V, ...
    'current_lower_raw_shortage_rad',nan));
current_upper = double(field_local(V, ...
    'current_upper_raw_shortage_rad',nan));
full_lower = double(field_local(V, ...
    'lower_full_endpoint_raw_shortage_rad',nan));
full_upper = double(field_local(V, ...
    'upper_full_endpoint_raw_shortage_rad',nan));
roundoff_lower = logical(field_local(V, ...
    'lower_roundoff_only_saturated_endpoint',false));
roundoff_upper = logical(field_local(V, ...
    'upper_roundoff_only_saturated_endpoint',false));
position_planning_tol = double(field_local( ...
    V,'position_planning_tolerance_rad',nan));
position_audit_tol = double(field_local( ...
    V,'position_audit_tolerance_rad',nan));
if joint>=1 && joint<=numel(raw_lower) && ...
        joint<=numel(raw_upper) && joint<=numel(tight_lower) && ...
        joint<=numel(tight_upper)
    raw_lo = raw_lower(joint);
    raw_hi = raw_upper(joint);
    viable_lo = tight_lower(joint);
    viable_hi = tight_upper(joint);
    if strcmp(side,'lower') && joint<=numel(current_lower) && ...
            joint<=numel(full_lower) && joint<=numel(roundoff_lower)
        current_raw_shortage = current_lower(joint);
        full_endpoint_raw_shortage = full_lower(joint);
        roundoff_only_endpoint = roundoff_lower(joint);
    elseif strcmp(side,'upper') && joint<=numel(current_upper) && ...
            joint<=numel(full_upper) && joint<=numel(roundoff_upper)
        current_raw_shortage = current_upper(joint);
        full_endpoint_raw_shortage = full_upper(joint);
        roundoff_only_endpoint = roundoff_upper(joint);
    else
        current_raw_shortage = NaN;
        full_endpoint_raw_shortage = NaN;
        roundoff_only_endpoint = false;
    end
else
    raw_lo = NaN;
    raw_hi = NaN;
    viable_lo = NaN;
    viable_hi = NaN;
    current_raw_shortage = NaN;
    full_endpoint_raw_shortage = NaN;
    roundoff_only_endpoint = false;
end
end

function M = final_motion_metrics_local( ...
        robot,q,dq,ddq,dq_mid,Ts,limits,workspace,tol_cfg)
required_limits = {'qd_max','qdd_cmd_max_actual','q_min','q_max'};

required_tol = {'ratio_tolerance','position_planning_tolerance_rad', ...
    'position_tolerance_rad', ...
    'joint_limit_viability'};

ratio_tol = nonnegative_scalar_local( ...
    tol_cfg.ratio_tolerance,'ratio_tolerance');
q_tol = nonnegative_scalar_local( ...
    tol_cfg.position_tolerance_rad,'position_tolerance_rad');
q_planning_tol = nonnegative_scalar_local( ...
    tol_cfg.position_planning_tolerance_rad, ...
    'position_planning_tolerance_rad');

nJ = numel(q);
qd_lim = expand_local(limits.qd_max,nJ,inf);
qdd_lim = expand_local(limits.qdd_cmd_max_actual,nJ,inf);
qlo = expand_signed_limit_local(limits.q_min,nJ,-inf);
qhi = expand_signed_limit_local(limits.q_max,nJ,inf);

dq_next = dq + Ts*ddq;
q_next = q + Ts*dq_mid;
qd_peak = max([abs(dq);abs(dq_mid);abs(dq_next)],[],1);
qd_ratio = max_ratio_local(qd_peak,qd_lim);
qdd_ratio = max_ratio_local(abs(ddq),qdd_lim);

% The midpoint integrator commits q_next, but a constant acceleration can
% have an interior position extremum when velocity reverses inside the 1-ms

q_swept_min = min(q,q_next);
q_swept_max = max(q,q_next);
turn_time = nan(1,nJ);
nonzero_acceleration = isfinite(ddq) & abs(ddq)>0;
turn_time(nonzero_acceleration) = ...
    -dq(nonzero_acceleration)./ddq(nonzero_acceleration);
interior_turn = turn_time>0 & turn_time<Ts;
if any(interior_turn)
    q_turn = q(interior_turn) + ...
        dq(interior_turn).*turn_time(interior_turn) + ...
        0.5*ddq(interior_turn).*turn_time(interior_turn).^2;
    q_swept_min(interior_turn) = min( ...
        q_swept_min(interior_turn),q_turn);
    q_swept_max(interior_turn) = max( ...
        q_swept_max(interior_turn),q_turn);
end
lo_v = qlo-q_swept_min;
hi_v = q_swept_max-qhi;
lo_v(~isfinite(qlo)) = -inf;
hi_v(~isfinite(qhi)) = -inf;
[q_violation,q_violation_index] = max([0,lo_v,hi_v]);
if q_violation_index==1
    q_violation_joint = 0;
    q_violation_side = 'none';
elseif q_violation_index<=1+nJ
    q_violation_joint = q_violation_index-1;
    q_violation_side = 'lower';
else
    q_violation_joint = q_violation_index-1-nJ;
    q_violation_side = 'upper';
end

% One-step position feasibility is not a controlled-invariance guarantee.
% Require the committed successor to retain enough joint-position distance
% to stop each outward velocity under the unchanged qdd limit.
viability = spinn3d_joint_limit_viability( ...
    q_next,dq_next,qlo,qhi,qdd_lim, ...
    tol_cfg.joint_limit_viability,q_tol);
finite_state = all(isfinite([ddq,dq_mid,dq_next,q_next]));
kin_pass = finite_state && qd_ratio<=1+ratio_tol && ...
    qdd_ratio<=1+ratio_tol && q_violation<=q_tol && viability.pass;
geometry = spinn3d_physical_geometry_check(robot,q_next,workspace);
M = struct('pass',logical(kin_pass && geometry.pass), ...
    'kinematic_pass',logical(kin_pass), ...
    'geometry_pass',logical(geometry.pass), ...
    'qdd_ratio',qdd_ratio,'qd_ratio',qd_ratio, ...
    'qpos_violation_rad',q_violation, ...
    'qpos_planning_tolerance_rad',q_planning_tol, ...
    'qpos_audit_tolerance_rad',q_tol, ...
    'qpos_violation_joint',q_violation_joint, ...
    'qpos_violation_side',q_violation_side, ...
    'qpos_swept_min',q_swept_min,'qpos_swept_max',q_swept_max, ...
    'joint_limit_viability_pass',logical(viability.pass), ...
    'joint_limit_stopping_violation_rad',viability.violation_rad, ...
    'joint_limit_viability_joint',viability.offending_joint, ...
    'joint_limit_viability_side',viability.offending_side, ...
    'joint_limit_viability',viability, ...
    'dq_next',dq_next,'q_next',q_next,'geometry',geometry);
end

function D = projection_diagnostic_default_local(fast_motion)
D = struct('attempted',false,'used',false,'lambda',NaN,'trials',0, ...
    'requested_qdd_ratio',NaN,'any_actuator_feasible',false, ...
    'bounds_feasible',false,'a_lo',NaN,'a_hi',NaN, ...
    'successor_viability_acceleration_bounds',struct(), ...
    'exact_torque_box_tested',false, ...
    'exact_torque_box_feasible',false, ...
    'exact_torque_box_novel_candidate',false, ...
    'exact_torque_box_report',struct(), ...
    'search_mode','fast_path','anchor_kind','none', ...
    'last_motion',fast_motion);
end

function [C,D] = recover_final_command_local( ...
        robot,Mq,h,tau_req,q,dq,Ts,tau_lim,Pax_lim,bus, ...
        limits,workspace,tol_cfg,fast_motion,R)
% Search only acceleration-consistent torques.  The motion box is convex,
% but actuator/power feasibility need not intersect the old single line from
% zero acceleration to the request.  Build physical n-D unloading anchors,
% locate a completely safe anchor, then restore the requested command along
% a line and individual coordinates.  Every retained point is independently

C = empty_candidate_local();
D = projection_diagnostic_default_local(fast_motion);
D.attempted = true;
% Report the centrally owned recovery schema instead of duplicating a
% private mode literal that can drift from PB.execution_audit.recovery.
D.search_mode = char(R.schema);

[a_lo,a_hi,bounds_ok,qdd_lim,viability_bounds] = ...
    acceleration_bounds_local( ...
    q,dq,Ts,limits,tol_cfg);
D.a_lo = a_lo;
D.a_hi = a_hi;
D.bounds_feasible = logical(bounds_ok);
D.successor_viability_acceleration_bounds = viability_bounds;

ddq_req = (Mq \ (tau_req(:)-h(:))).';
D.requested_qdd_ratio = max_ratio_local(abs(ddq_req),qdd_lim);
if ~bounds_ok || any(~isfinite(ddq_req))
    return;
end

% Validate that the central budget can actually execute every configured
% phase for this joint count.  Quietly truncating the late pattern/restore
% phases would reintroduce a deterministic false-rejection region.
anchor_count_max = 7;
required_trials = 1 + anchor_count_max + ...
    (anchor_count_max-1)*(R.bridge_grid_points-2) + ...
    2*numel(ddq_req)*R.pattern_search_iterations + ...
    (R.line_grid_points-2) + R.line_refine_iterations + ...
    numel(ddq_req)*(1+R.coordinate_refine_iterations);

a_target = min(a_hi,max(a_lo,ddq_req));
a_fast = min(a_hi,max(a_lo, ...
    (fast_motion.dq_next-dq)/max(Ts,1e-12)));
a_hold = min(a_hi,max(a_lo,zeros(size(ddq_req))));
a_free = min(a_hi,max(a_lo,(Mq \ (-h(:))).'));
a_brake = min(a_hi,max(a_lo,-dq/max(Ts,1e-12)));
tau_box = min(tau_lim,max(-tau_lim,tau_req));
a_tau_box = min(a_hi,max(a_lo,(Mq \ (tau_box(:)-h(:))).'));
tau_scale = abs(tau_lim);
bad_scale = ~isfinite(tau_scale) | tau_scale<=0;
fallback_scale = max([abs(h(:)),abs(tau_req(:)),ones(numel(h),1)],[],2).';
tau_scale(bad_scale) = fallback_scale(bad_scale);
[a_tau_min,~] = spinn3d_box_torque_min_anchor( ...
    Mq,h,a_lo,a_hi, ...
    [a_fast;a_hold;a_target;a_free;a_brake;a_tau_box], ...
    tau_scale,R.torque_min_sweeps);

% Target first: clipping the raw request into the motion box may already be

[trial,actuator_ok,motion_try,score_try] = evaluate_counted_local( ...
    a_target,robot,Mq,h,q,dq,Ts,tau_lim,Pax_lim,bus,limits, ...
    workspace,tol_cfg,D.trials,R.max_trials);
D.trials = D.trials + 1;
D.any_actuator_feasible = D.any_actuator_feasible || actuator_ok;
if ~isempty(motion_try), D.last_motion = motion_try; end
if trial.found
    C = trial;
    D.used = true;
    D.lambda = 1.0;
    D.anchor_kind = 'box_projected_target';
    return;
end

anchor_rows = [a_fast;a_hold;a_free;a_brake;a_tau_box;a_tau_min];
anchor_kinds = {'viability_projected_fast_command', ...
    'zero_acceleration','zero_torque_unload', ...
    'velocity_brake','torque_box','weighted_torque_minimum'};
[anchor_rows,anchor_kinds] = unique_rows_stable_local( ...
    anchor_rows,anchor_kinds);
safe = empty_candidate_local();
safe_a = [];
safe_kind = 'none';
safe_distance = inf;
best_score = score_try;
best_a = a_target;

for ia = 1:size(anchor_rows,1)
    if D.trials>=R.max_trials, break; end
    a_try = anchor_rows(ia,:);
    [trial,actuator_ok,motion_try,score_try] = evaluate_counted_local( ...
        a_try,robot,Mq,h,q,dq,Ts,tau_lim,Pax_lim,bus,limits, ...
        workspace,tol_cfg,D.trials,R.max_trials);
    D.trials = D.trials + 1;
    D.any_actuator_feasible = D.any_actuator_feasible || actuator_ok;
    if ~isempty(motion_try), D.last_motion = motion_try; end
    if score_try < best_score
        best_score = score_try;
        best_a = a_try;
    end
    if trial.found
        d = normalized_distance_local(a_try,a_target,a_lo,a_hi);
        if d < safe_distance
            safe = trial;
            safe_a = a_try;
            safe_kind = anchor_kinds{ia};
            safe_distance = d;
        end
    end
end

% 1820 tiny active-set solves for this four-axis arm).  Delay it until the
% target and all inexpensive physical anchors have failed so ordinary
% projected samples do not pay that cost.  A returned vertex is still only

if ~safe.found
    known_rows = [a_target;anchor_rows];
    [a_tau_feasible,tau_feasible_report] = ...
        spinn3d_box_torque_feasible_anchor( ...
            Mq,h,a_lo,a_hi,tau_lim,a_target,known_rows);
    D.exact_torque_box_tested = true;
    D.exact_torque_box_feasible = ...
        logical(tau_feasible_report.feasible);
    D.exact_torque_box_novel_candidate = ...
        logical(tau_feasible_report.novel_candidate);
    D.exact_torque_box_report = tau_feasible_report;
    if tau_feasible_report.feasible && ...
            tau_feasible_report.novel_candidate && ...
            D.trials<R.max_trials
        anchor_rows = [anchor_rows;a_tau_feasible]; %#ok<AGROW>
        anchor_kinds{end+1} = 'exact_torque_box_active_set';
        [trial,actuator_ok,motion_try,score_try] = ...
            evaluate_counted_local( ...
                a_tau_feasible,robot,Mq,h,q,dq,Ts,tau_lim,Pax_lim, ...
                bus,limits,workspace,tol_cfg,D.trials,R.max_trials);
        D.trials = D.trials + 1;
        D.any_actuator_feasible = ...
            D.any_actuator_feasible || actuator_ok;
        if ~isempty(motion_try), D.last_motion = motion_try; end
        if score_try < best_score
            best_score = score_try;
            best_a = a_tau_feasible;
        end
        if trial.found
            safe = trial;
            safe_a = a_tau_feasible;
            safe_kind = 'exact_torque_box_active_set';
        end
    end
end

% If no endpoint is safe, bridge the zero-acceleration point to each physical
% unloading anchor.  This finds feasible interior points that the old
% zero-to-request line could never visit.
if ~safe.found
    bridge_lambda = linspace(0,1,R.bridge_grid_points);
    bridge_lambda = bridge_lambda(2:end-1);
    for ia = 1:size(anchor_rows,1)
        % unique_rows_stable_local may retain a_fast and remove an equal
        % a_hold row.  Skip by value, not by a positional convention, so
        % the critical a_hold-to-a_fast viable-box bridge is never lost.
        if all(abs(anchor_rows(ia,:)-a_hold)<=1e-12)
            continue;
        end
        for il = 1:numel(bridge_lambda)
            if D.trials>=R.max_trials, break; end
            a_try = a_hold + bridge_lambda(il)*(anchor_rows(ia,:)-a_hold);
            [trial,actuator_ok,motion_try,score_try] = evaluate_counted_local( ...
                a_try,robot,Mq,h,q,dq,Ts,tau_lim,Pax_lim,bus,limits, ...
                workspace,tol_cfg,D.trials,R.max_trials);
            D.trials = D.trials + 1;
            D.any_actuator_feasible = D.any_actuator_feasible || actuator_ok;
            if ~isempty(motion_try), D.last_motion = motion_try; end
            if score_try < best_score
                best_score = score_try;
                best_a = a_try;
            end
            if trial.found
                safe = trial;
                safe_a = a_try;
                safe_kind = ['bridge_to_' anchor_kinds{ia}];
                break;
            end
        end
        if safe.found || D.trials>=R.max_trials, break; end
    end
end

% is used only when all physical anchors/bridges failed and is bounded by the
% same central trial budget.
if ~safe.found
    a_pattern = best_a;
    step = 0.5*(a_hi-a_lo);
    for ip = 1:R.pattern_search_iterations
        improved = false;
        for j = 1:numel(a_pattern)
            for sg = [-1 1]
                if D.trials>=R.max_trials, break; end
                a_try = a_pattern;
                a_try(j) = min(a_hi(j),max(a_lo(j), ...
                    a_try(j)+sg*step(j)));
                [trial,actuator_ok,motion_try,score_try] = ...
                    evaluate_counted_local(a_try,robot,Mq,h,q,dq,Ts, ...
                        tau_lim,Pax_lim,bus,limits,workspace,tol_cfg, ...
                        D.trials,R.max_trials);
                D.trials = D.trials + 1;
                D.any_actuator_feasible = ...
                    D.any_actuator_feasible || actuator_ok;
                if ~isempty(motion_try), D.last_motion = motion_try; end
                if trial.found
                    safe = trial;
                    safe_a = a_try;
                    safe_kind = 'multidimensional_pattern';
                    break;
                end
                if score_try < best_score
                    best_score = score_try;
                    a_pattern = a_try;
                    improved = true;
                end
            end
            if safe.found || D.trials>=R.max_trials, break; end
        end
        if safe.found || D.trials>=R.max_trials, break; end
        if ~improved, step = 0.5*step; end
    end
end
if ~safe.found
    return;
end

% Restore as much of the requested acceleration as possible along a verified
% safe-anchor line.  Retain the last safe point at every refinement.
C = safe;
a_best = safe_a;
lambda_best = 0.0;
previous_higher = 1.0;
lambda_grid = linspace(1,0,R.line_grid_points);
for il = 2:numel(lambda_grid)-1
    if D.trials>=R.max_trials, break; end
    lambda = lambda_grid(il);
    a_try = safe_a + lambda*(a_target-safe_a);
    [trial,actuator_ok,motion_try,~] = evaluate_counted_local( ...
        a_try,robot,Mq,h,q,dq,Ts,tau_lim,Pax_lim,bus,limits, ...
        workspace,tol_cfg,D.trials,R.max_trials);
    D.trials = D.trials + 1;
    D.any_actuator_feasible = D.any_actuator_feasible || actuator_ok;
    if ~isempty(motion_try), D.last_motion = motion_try; end
    if ~trial.found
        previous_higher = lambda;
        continue;
    end
    C = trial;
    a_best = a_try;
    lambda_best = lambda;
    lo = lambda;
    hi = previous_higher;
    for ir = 1:R.line_refine_iterations
        if D.trials>=R.max_trials, break; end
        lambda_refine = 0.5*(lo+hi);
        a_try = safe_a + lambda_refine*(a_target-safe_a);
        [trial_refine,actuator_ok,motion_refine,~] = ...
            evaluate_counted_local(a_try,robot,Mq,h,q,dq,Ts, ...
                tau_lim,Pax_lim,bus,limits,workspace,tol_cfg, ...
                D.trials,R.max_trials);
        D.trials = D.trials + 1;
        D.any_actuator_feasible = ...
            D.any_actuator_feasible || actuator_ok;
        if ~isempty(motion_refine), D.last_motion = motion_refine; end
        if trial_refine.found
            C = trial_refine;
            a_best = a_try;
            lambda_best = lambda_refine;
            lo = lambda_refine;
        else
            hi = lambda_refine;
        end
    end
    break;
end

% A line still couples all joints.  Starting from the verified point above,
% recover each requested coordinate independently, in largest-gap order.
coordinate_changed = false;
gap_scale = max(a_hi-a_lo,1e-9);
[~,coordinate_order] = sort(abs(a_target-a_best)./gap_scale,'descend');
for jo = 1:numel(coordinate_order)
    j = coordinate_order(jo);
    if abs(a_target(j)-a_best(j))<=1e-12 || D.trials>=R.max_trials
        continue;
    end
    safe_value = a_best(j);
    unsafe_value = a_target(j);
    a_try = a_best;
    a_try(j) = unsafe_value;
    [trial,actuator_ok,motion_try,~] = evaluate_counted_local( ...
        a_try,robot,Mq,h,q,dq,Ts,tau_lim,Pax_lim,bus,limits, ...
        workspace,tol_cfg,D.trials,R.max_trials);
    D.trials = D.trials + 1;
    D.any_actuator_feasible = D.any_actuator_feasible || actuator_ok;
    if ~isempty(motion_try), D.last_motion = motion_try; end
    if trial.found
        C = trial;
        a_best = a_try;
        coordinate_changed = true;
        continue;
    end
    for ir = 1:R.coordinate_refine_iterations
        if D.trials>=R.max_trials, break; end
        mid_value = 0.5*(safe_value+unsafe_value);
        a_try = a_best;
        a_try(j) = mid_value;
        [trial_refine,actuator_ok,motion_refine,~] = ...
            evaluate_counted_local(a_try,robot,Mq,h,q,dq,Ts, ...
                tau_lim,Pax_lim,bus,limits,workspace,tol_cfg, ...
                D.trials,R.max_trials);
        D.trials = D.trials + 1;
        D.any_actuator_feasible = ...
            D.any_actuator_feasible || actuator_ok;
        if ~isempty(motion_refine), D.last_motion = motion_refine; end
        if trial_refine.found
            C = trial_refine;
            a_best = a_try;
            safe_value = mid_value;
            coordinate_changed = true;
        else
            unsafe_value = mid_value;
        end
    end
end

D.used = true;
D.anchor_kind = safe_kind;
if coordinate_changed
    D.lambda = NaN;
else
    D.lambda = lambda_best;
end
end

function [C,actuator_ok,motion,score] = evaluate_counted_local( ...
        a_try,robot,Mq,h,q,dq,Ts,tau_lim,Pax_lim,bus,limits, ...
        workspace,tol_cfg,trial_count,max_trials)

[C,actuator_ok,motion,score] = evaluate_recovery_candidate_local( ...
    a_try,robot,Mq,h,q,dq,Ts,tau_lim,Pax_lim,bus,limits, ...
    workspace,tol_cfg);
end

function [C,actuator_ok,motion,score] = evaluate_recovery_candidate_local( ...
        a_try,robot,Mq,h,q,dq,Ts,tau_lim,Pax_lim,bus,limits, ...
        workspace,tol_cfg)
C = empty_candidate_local();
actuator_ok = false;
motion = [];
tau_try = (Mq*a_try(:) + h(:)).';
ddq_try = (Mq \ (tau_try(:)-h(:))).';
dq_mid_try = dq + 0.5*Ts*ddq_try;
metrics_try = constraint_metrics_local( ...
    tau_try,dq_mid_try,tau_lim,Pax_lim,bus,dq,2*dq_mid_try-dq);
score = actuator_score_local(metrics_try);
% Motion/geometry scoring is independent of whether this torque currently
% passes the actuator envelope.  Evaluate it for every candidate so the

% discontinuity exactly at the actuator-feasibility boundary.
motion = final_motion_metrics_local( ...
    robot,q,dq,ddq_try,dq_mid_try,Ts,limits,workspace,tol_cfg);
score = max(score,motion_score_local(motion,tol_cfg));
if ~metrics_try.pass
    return;
end
try
    tau_verify = clamp_power_axis_total_bus(tau_try,dq_mid_try,Pax_lim,bus);
catch MEclamp
    if recoverable_clamp_error_local(MEclamp)
        score=max(score,1+1e-6); return;
    end
    rethrow(MEclamp);
end
idempotent = norm(tau_verify-tau_try,inf) <= ...
    1e-10*max(1,norm(tau_try,inf));
if ~idempotent
    score = max(score,1+norm(tau_verify-tau_try,inf)/ ...
        max(1,norm(tau_try,inf)));
    return;
end
actuator_ok = true;
if ~motion.pass
    return;
end
C = struct('found',true,'tau',tau_try,'ddq',ddq_try, ...
    'dq_mid',dq_mid_try,'metrics',metrics_try,'motion',motion,'a',a_try);
score = min(score,1.0);
end

function C = empty_candidate_local()
C = struct('found',false,'tau',[],'ddq',[],'dq_mid',[], ...
    'metrics',struct(),'motion',struct(),'a',[]);
end

function [rows,kinds] = unique_rows_stable_local(rows,kinds)
[rows,ia] = unique(double(rows),'rows','stable');
kinds = kinds(ia);
end

function d = normalized_distance_local(a,b,a_lo,a_hi)
scale = max(double(a_hi-a_lo),1e-9);
d = norm((double(a)-double(b))./scale,2);
end

function score = actuator_score_local(M)
ratios = [double(M.tau_ratio),double(M.Paxis_ratio), ...
    double(M.motor_ratio),cap_ratio_value_local(M.Pgrid_W,M.Pgrid_cap_W), ...
    cap_ratio_value_local(M.Pdump_W,M.Pdump_cap_W)];
if any(~isfinite(ratios))
    score = inf;
else
    score = max(ratios);
end
end

function r = cap_ratio_value_local(v,cap)
v = max(0,double(v));
cap = double(cap);
if isfinite(cap)
    if cap>0
        r = v/cap;
    elseif v<=1e-12
        r = 0;
    else
        r = inf;
    end
else
    r = 0;
end
end

function score = motion_score_local(M,tol_cfg)
q_tol = max(double(tol_cfg.position_tolerance_rad),1e-9);
score = max([double(M.qdd_ratio),double(M.qd_ratio), ...
    1+max(0,double(M.qpos_violation_rad))/q_tol, ...
    1+max(0,double(M.joint_limit_stopping_violation_rad))/q_tol]);
if ~logical(M.geometry_pass)
    margins = [double(field_local(M.geometry,'workplane_margin_m',NaN)), ...
        double(field_local(M.geometry,'min_self_collision_margin_m',NaN))];
    margins = margins(isfinite(margins));
    if isempty(margins)
        score = inf;
    else
        score = max(score,1+max(0,-min(margins))/1e-3);
    end
end
if ~isfinite(score), score=inf; end
end

function R = recovery_contract_local(tol_cfg)

R = tol_cfg.recovery;
required = {'schema','max_trials','torque_min_sweeps', ...
    'line_grid_points','line_refine_iterations','bridge_grid_points', ...
    'pattern_search_iterations','coordinate_refine_iterations'};

numeric_fields = required(2:end);
for i = 1:numel(numeric_fields)
    name = numeric_fields{i};
    value = double(R.(name));

    R.(name) = round(value);
end

end

function [a_lo,a_hi,feasible,qdd_lim,viability_bounds] = ...
        acceleration_bounds_local( ...
        q,dq,Ts,limits,tol_cfg)
nJ = numel(q);
qd_lim = expand_local(limits.qd_max,nJ,inf);
qdd_lim = expand_local(limits.qdd_cmd_max_actual,nJ,inf);
qlo = expand_signed_limit_local(limits.q_min,nJ,-inf);
qhi = expand_signed_limit_local(limits.q_max,nJ,inf);

ratio_tol = nonnegative_scalar_local( ...
    tol_cfg.ratio_tolerance,'ratio_tolerance');
q_tol = nonnegative_scalar_local( ...
    tol_cfg.position_tolerance_rad,'position_tolerance_rad');
q_planning_tol = nonnegative_scalar_local( ...
    tol_cfg.position_planning_tolerance_rad, ...
    'position_planning_tolerance_rad');

qlo_audit = qlo-q_tol;
qhi_audit = qhi+q_tol;

% The incoming state was already committed by the previous independently

% used below to construct any successor command boundary.
incoming_ok = all(isfinite([q,dq])) && ...
    all(abs(dq)<=qd_lim.*(1+ratio_tol)) && ...
    all(q>=qlo_audit) && all(q<=qhi_audit);

a_lo = -qdd_lim;
a_hi = qdd_lim;
% Midpoint velocity: dq_mid=dq+0.5*Ts*a.
a_lo = max(a_lo,2*(-qd_lim-dq)/Ts);
a_hi = min(a_hi,2*( qd_lim-dq)/Ts);
% End-of-step velocity: dq_next=dq+Ts*a.
a_lo = max(a_lo,(-qd_lim-dq)/Ts);
a_hi = min(a_hi,( qd_lim-dq)/Ts);
% Midpoint position integration: q_next=q+Ts*dq+0.5*Ts^2*a.
% Construct this box against the physical q limits.  Using qlo-q_tol and

% source of the seed-3 1.00000093e-8 rad false rejection.
a_lo = max(a_lo,2*(qlo-q-Ts*dq)/(Ts*Ts));
a_hi = min(a_hi,2*(qhi-q-Ts*dq)/(Ts*Ts));
if any(isnan([a_lo,a_hi]))
    error('spinn3d_predictive_torque_step:BadFinalMotionLimits', ...
        'Central motion limits produced a NaN acceleration interval.');
end
if all(a_lo<=a_hi)
    [a_lo,a_hi,viability_bounds] = ...
        spinn3d_joint_limit_viable_acceleration_bounds( ...
            q,dq,Ts,qlo,qhi,qdd_lim,a_lo,a_hi, ...
            tol_cfg.joint_limit_viability,q_tol);
else
    viability_bounds = struct( ...
        'schema','successor_viable_acceleration_box_v3', ...
        'enabled',logical(tol_cfg.joint_limit_viability.enable), ...
        'pass',false,'incoming_lower_bound',a_lo, ...
        'incoming_upper_bound',a_hi,'lower_bound',a_lo, ...
        'upper_bound',a_hi, ...
        'current_lower_raw_shortage_rad',nan(1,nJ), ...
        'current_upper_raw_shortage_rad',nan(1,nJ), ...
        'lower_full_endpoint_raw_shortage_rad',nan(1,nJ), ...
        'upper_full_endpoint_raw_shortage_rad',nan(1,nJ), ...
        'lower_roundoff_only_saturated_endpoint',false(1,nJ), ...
        'upper_roundoff_only_saturated_endpoint',false(1,nJ), ...
        'roundoff_only_saturated_endpoint',false(1,nJ), ...
        'position_planning_tolerance_rad',q_planning_tol, ...
        'position_audit_tolerance_rad',q_tol, ...
        'infeasible_joint',0, ...
        'infeasible_side','preexisting_motion_box', ...
        'numeric_tolerance_rad',double( ...
            tol_cfg.joint_limit_viability.numeric_tolerance_rad));
end
feasible = logical(incoming_ok && viability_bounds.pass && ...
    all(a_lo<=a_hi));
end

function M = constraint_metrics_local(tau,dq_mid,tau_lim,Pax_lim,bus,dq0,dq1)
I=spinn3d_motor_interval_audit(tau,dq0,dq1,bus);
Paxis=I.Paxis_motoring_peak_W;
Pgrid=I.Pgrid_admission_upper_bound_W;
Pdump=I.Pdump_peak_W;
mot=I;
Pgrid_cap = double(field_local(bus,'P_grid_peak_max',inf));
Pdump_cap = double(field_local(bus,'P_brk_peak', ...
    field_local(bus,'P_dump_max',inf)));

M = struct();
M.tau_ratio = max_ratio_local(abs(tau),abs(tau_lim));
M.Paxis_ratio = max_ratio_local(Paxis,Pax_lim);
M.motor_ratio = max(double(mot.torque_speed_ratio),[],'all');
M.Pgrid_W = double(Pgrid);
M.Pgrid_mid_W = double(I.Pgrid_mid_W);
M.Pgrid_cap_W = Pgrid_cap;
M.Pdump_W = double(Pdump);
M.Pdump_cap_W = Pdump_cap;

tol_tau = 1e-10;
tol_axis = cap_tol_local(Pax_lim);
tol_grid = cap_tol_local(Pgrid_cap);
tol_dump = cap_tol_local(Pdump_cap);
M.dynamic_dc_state_pass=I.dynamic_state_pass;
M.pass = I.dynamic_state_pass && all(abs(tau)<=tau_lim+tol_tau) && ...
    all(Paxis<=Pax_lim+tol_axis) && M.motor_ratio<=1+1e-9 && ...
    (~isfinite(Pgrid_cap) || Pgrid<=Pgrid_cap+tol_grid) && ...
    (~isfinite(Pdump_cap) || Pdump<=Pdump_cap+tol_dump);
end

function r = max_ratio_local(num,lim)
num = double(num(:));
lim = double(lim(:));
if isscalar(lim), lim=repmat(lim,numel(num),1); end
lim = lim(1:min(numel(lim),numel(num)));
num = num(1:numel(lim));
rvec = zeros(size(num));
finite = isfinite(lim);
rvec(finite) = num(finite)./max(abs(lim(finite)),eps);
r = max(rvec,[],'all');
if isempty(r), r=0; end
end

function tol = cap_tol_local(cap)
v = double(cap(:));
v = v(isfinite(v));
if isempty(v), tol=0; else, tol=max(1e-8,1e-9*max(1,max(abs(v)))); end
end

function v = expand_local(v,n,def)
try, v = double(v(:).'); catch, v = def; end
if isempty(v), v = def; end
if isscalar(v), v = repmat(v,1,n); end
if numel(v)<n, v = [v repmat(v(end),1,n-numel(v))]; end
v = v(1:n);
v(~isfinite(v)) = def;
end

function v = expand_signed_limit_local(v,n,def)
try, v = double(v(:).'); catch, v = def; end
if isempty(v), v = def; end
if isscalar(v), v = repmat(v,1,n); end
if numel(v)<n, v = [v repmat(v(end),1,n-numel(v))]; end
v = v(1:n);
v(isnan(v)) = def;
end

function v = field_local(S,k,def)
if isstruct(S) && isfield(S,k) && ~isempty(S.(k)), v=S.(k); else, v=def; end
end

function y = row_local(x)
y = double(x(:).');
end

function v = nonnegative_scalar_local(v,name)
v = double(v);

end

function reason = geometry_failure_reason_local(G)
if ~logical(G.workplane_pass)
    reason = sprintf('lowest primitive %s at sample %d', ...
        char(G.min_z_primitive),round(double(G.min_z_sample_index)));
elseif ~logical(G.self_collision_pass)
    names = G.min_self_collision_pair_names;
    if iscell(names) && numel(names)>=2
        reason = sprintf('collision pair %s versus %s at sample %d', ...
            char(names{1}),char(names{2}), ...
            round(double(G.min_self_collision_sample_index)));
    else
        reason = 'unresolved conservative self-collision pair';
    end
else
    reason = 'motion-limit violation with geometry still valid';
end
end

function tf=recoverable_clamp_error_local(ME)
% Only physical trial infeasibility may enter recovery. Contract/programming
% errors remain fatal; this does not turn an unsafe candidate into a pass.
ids={'clamp_power_axis_total_bus:EmptyActuatorEnvelope', ...
 'clamp_power_axis_total_bus:ActuatorEnvelopeLostAfterClamp', ...
 'clamp_power_axis_total_bus:NoFeasibleGridScale', ...
 'clamp_power_axis_total_bus:GridClampFailed'};
tf=any(strcmp(ME.identifier,ids));
end
