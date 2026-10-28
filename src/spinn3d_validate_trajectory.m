function report = spinn3d_validate_trajectory(robot, Q, cfg, target_xyz, Qd, Qdd)
% Evaluate path geometry, joint limits, speed and acceleration feasibility.

if nargin < 4, target_xyz = []; end
if nargin < 5, Qd = []; end
if nargin < 6, Qdd = []; end

if xor(isempty(Qd),isempty(Qdd))
    error('spinn3d_validate_trajectory:IncompleteKinematics', ...
        'Qd and Qdd must either both be supplied or both be empty.');
end

ws = struct();
if isstruct(cfg) && isfield(cfg,'WORKSPACE') && isstruct(cfg.WORKSPACE)
    ws = cfg.WORKSPACE;
end
stride = max(1,round(getdef(ws,'check_stride',10)));
endpoint_tol = getdef(ws,'endpoint_joint_tol',1e-10);
plane_z = getdef(ws,'workplane_z_m',-inf);
clearance = max(0,double(getdef(ws,'clearance_m',0)));
path_tol = max(0,double(getdef(ws,'max_path_error_m',inf)));
kin_tol = max(0,double(getdef(ws,'kinematic_limit_tol',1e-8)));
rest_tol_qd = max(0,double(getdef(ws,'endpoint_qd_tol_rad_s',1e-10)));
rest_tol_qdd = max(0,double(getdef(ws,'endpoint_qdd_tol_rad_s2',1e-9)));

nJ = size(Q,2);
qmin = -inf(1,nJ); qmax = inf(1,nJ); jj = 0;
for ib = 1:numel(robot.Bodies)
    body = robot.Bodies{ib};
    if ~strcmpi(body.Joint.Type,'fixed')
        jj = jj + 1;
        if jj <= nJ && numel(body.Joint.PositionLimits) == 2
            qmin(jj) = body.Joint.PositionLimits(1);
            qmax(jj) = body.Joint.PositionLimits(2);
        end
    end
end
joint_violation = max([max(qmin-Q,[],'all'), max(Q-qmax,[],'all'), 0]);
if joint_violation > endpoint_tol
    error('spinn3d_validate_trajectory:JointLimit', ...
        'Trajectory exceeds a joint position limit by %.6g rad.',joint_violation);
end

idx = unique([1:stride:size(Q,1), size(Q,1)]);
body_names = robot.BodyNames;
ee = body_names{end};
ee_xyz = nan(size(Q,1),3);
geometry = spinn3d_physical_geometry_check(robot,Q(idx,:),ws);
min_z = geometry.min_geometry_z_m;
if ~geometry.workplane_pass
    error('spinn3d_validate_trajectory:Workplane', ...
        ['Conservative robot/payload geometry crosses the configured work ', ...
         'plane: min z=%.6f m (%s), required z>=%.6f m. Adjust the path ', ...
         'or measured mounting geometry; q0/qf are not modified automatically.'], ...
         min_z,geometry.min_z_primitive,geometry.required_min_z_m);
end
if ~geometry.self_collision_pass
    pair_names = geometry.min_self_collision_pair_names;
    error('spinn3d_validate_trajectory:SelfCollision', ...
        ['Conservative self-collision clearance failed: %s vs %s at ', ...
         'checked sample %d, signed clearance-to-required-margin %.6g m. ', ...
         'payload_obb_bounding_sphere denotes the conservative bound ', ...
         'around the injected payload box.'], ...
         pair_names{1},pair_names{2}, ...
         geometry.min_self_collision_sample_index, ...
         geometry.min_self_collision_margin_m);
end

path_error_max = NaN;
if ~isempty(target_xyz)

    for k = 1:size(Q,1)
        T = getTransform(robot,Q(k,:),ee);
        ee_xyz(k,:) = T(1:3,4).';
    end
    path_error_max = max(vecnorm(ee_xyz-target_xyz,2,2));
    if path_error_max > path_tol
        error('spinn3d_validate_trajectory:PathError', ...
            'Post-smoothed Cartesian path error %.6f m exceeds %.6f m.', ...
            path_error_max,path_tol);
    end
end

% Kinematic limits are checked only when derivatives are explicitly
% supplied.  This preserves compatibility for geometry-only callers while
% making every time-parameterized trajectory call a hard qd/qdd check.
kin_checked = ~isempty(Qd);
max_abs_qd = nan(1,nJ);
max_abs_qdd = nan(1,nJ);
qd_ratio_max = NaN;
qdd_ratio_max = NaN;
qd_violation = NaN;
qdd_violation = NaN;
endpoint_qd_max = NaN;
endpoint_qdd_max = NaN;
require_zero_endpoint_qdd = true;
if isstruct(cfg) && isfield(cfg,'WORKSPACE') && isstruct(cfg.WORKSPACE)
    require_zero_endpoint_qdd = logical(getdef(cfg.WORKSPACE, ...
        'require_zero_endpoint_acceleration',true));
end
if kin_checked

    

    
    qd_lim = expand_limit(getdef(cfg.CAPS,'qd_max',[]),nJ,'qd_max');
    qdd_lim = expand_limit(getdef(cfg.CAPS,'qdd_max',[]),nJ,'qdd_max');

    max_abs_qd = max(abs(Qd),[],1);
    max_abs_qdd = max(abs(Qdd),[],1);
    qd_violation = max([max_abs_qd-qd_lim,0]);
    qdd_violation = max([max_abs_qdd-qdd_lim,0]);
    qd_ratio_max = finite_ratio_max(max_abs_qd,qd_lim);
    qdd_ratio_max = finite_ratio_max(max_abs_qdd,qdd_lim);
    if qd_violation > kin_tol
        error('spinn3d_validate_trajectory:VelocityLimit', ...
            'Time-parameterized trajectory exceeds qd_max by %.6g rad/s (max ratio %.6f).', ...
            qd_violation,qd_ratio_max);
    end
    if qdd_violation > kin_tol
        error('spinn3d_validate_trajectory:AccelerationLimit', ...
            'Time-parameterized trajectory exceeds qdd_max by %.6g rad/s^2 (max ratio %.6f).', ...
            qdd_violation,qdd_ratio_max);
    end

    endpoint_qd_max = max(abs([Qd(1,:),Qd(end,:)]));
    endpoint_qdd_max = max(abs([Qdd(1,:),Qdd(end,:)]));
    if endpoint_qd_max > rest_tol_qd
        error('spinn3d_validate_trajectory:EndpointVelocity', ...
            'Endpoint velocity is not zero: max |qd|=%.6g rad/s.',endpoint_qd_max);
    end
    if require_zero_endpoint_qdd && endpoint_qdd_max > rest_tol_qdd
        error('spinn3d_validate_trajectory:EndpointAcceleration', ...
            'Endpoint acceleration is not zero: max |qdd|=%.6g rad/s^2.',endpoint_qdd_max);
    end
end

report = struct('valid',true,'min_z_m',min_z,'workplane_z_m',plane_z, ...
    'clearance_m',clearance,'joint_limit_violation_rad',joint_violation, ...
    'path_error_max_m',path_error_max,'samples_checked',numel(idx), ...
    'kinematics_checked',kin_checked,'max_abs_qd_rad_s',max_abs_qd, ...
    'max_abs_qdd_rad_s2',max_abs_qdd,'qd_ratio_max',qd_ratio_max, ...
    'qdd_ratio_max',qdd_ratio_max,'qd_violation_rad_s',qd_violation, ...
    'qdd_violation_rad_s2',qdd_violation, ...
    'endpoint_qd_max_rad_s',endpoint_qd_max, ...
    'endpoint_qdd_max_rad_s2',endpoint_qdd_max, ...
    'zero_endpoint_acceleration_required',require_zero_endpoint_qdd, ...
    'geometry_schema',geometry.geometry_schema, ...
    'geometry_workplane_pass',geometry.workplane_pass, ...
    'geometry_self_collision_pass',geometry.self_collision_pass, ...
    'workplane_margin_m',geometry.workplane_margin_m, ...
    'min_self_collision_margin_m', ...
        geometry.min_self_collision_margin_m, ...
    'payload_box_checked',geometry.payload_box_checked);
end

function v = getdef(S,fn,def)
if isstruct(S) && isfield(S,fn) && ~isempty(S.(fn)), v=S.(fn); else, v=def; end
end

function v = expand_limit(v,n,name)
if isempty(v)
    error('spinn3d_validate_trajectory:MissingLimit','cfg.CAPS.%s is required.',name);
end
v = double(v(:).');
if isscalar(v), v = repmat(v,1,n); end
if numel(v) ~= n || any(isnan(v)) || any(v <= 0)
    error('spinn3d_validate_trajectory:BadLimit', ...
        'cfg.CAPS.%s must be a positive scalar or a %d-element vector.',name,n);
end
end

function r = finite_ratio_max(x,lim)
finite = isfinite(lim);
if any(finite)
    r = max(x(finite)./lim(finite));
else
    r = 0;
end
end
