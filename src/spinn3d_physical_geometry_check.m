function R = spinn3d_physical_geometry_check(robot,Q,workspace)
% Evaluate workspace, workplane and robot geometry constraints.

persistent geometry_cache
cache_hit = ~isempty(geometry_cache) && ...
    isequaln(workspace,geometry_cache.workspace) && ...
    isequal(robot.BodyNames,geometry_cache.body_names);
if cache_hit
    current_payload = getBody(robot,geometry_cache.payload_name);
    cache_hit = isequal(double(current_payload.Mass),geometry_cache.payload_mass) && ...
        isequal(double(current_payload.CenterOfMass(:).'),geometry_cache.payload_com);
end
if ~cache_hit
    required = {'geometry_schema','capsule_body_names', ...
        'capsule_start_local_m','capsule_end_local_m','capsule_radius_m', ...
        'workplane_enable','workplane_z_m','clearance_m', ...
        'self_collision_enable','self_collision_pair_indices', ...
        'self_collision_margin_m','payload_body_name', ...
        'payload_box_dims_max_m','payload_check_requires_positive_mass', ...
        'payload_collision_capsule_indices'};

    

    names = cellstr(string(workspace.capsule_body_names));
    P0 = double(workspace.capsule_start_local_m);
    P1 = double(workspace.capsule_end_local_m);
    rad = double(workspace.capsule_radius_m(:));
    nCaps = numel(names);

    

    plane_enable = logical_scalar_local(workspace.workplane_enable, ...
        'workplane_enable');
    self_enable = logical_scalar_local(workspace.self_collision_enable, ...
        'self_collision_enable');
    plane_z = finite_scalar_local(workspace.workplane_z_m,'workplane_z_m');
    clearance = nonnegative_scalar_local(workspace.clearance_m,'clearance_m');
    self_margin = nonnegative_scalar_local( ...
        workspace.self_collision_margin_m,'self_collision_margin_m');
    required_z = plane_z + clearance;

    pairs = double(workspace.self_collision_pair_indices);
    if self_enable

    else
        pairs = zeros(0,2);
    end

    payload_name = char(string(workspace.payload_body_name));

    payload_dims = double(workspace.payload_box_dims_max_m(:).');

    payload_body = getBody(robot,payload_name);
    payload_mass = double(payload_body.Mass);

    payload_required = logical_scalar_local( ...
        workspace.payload_check_requires_positive_mass, ...
        'payload_check_requires_positive_mass');
    payload_checked = ~payload_required || payload_mass>0;
    payload_com = double(payload_body.CenterOfMass(:).');

    payload_corners_local = zeros(0,3);
    payload_radius = 0.5*norm(payload_dims);
    payload_collision_indices = double( ...
        workspace.payload_collision_capsule_indices(:).');

    if payload_checked
        h = 0.5*payload_dims;
        [sx,sy,sz] = ndgrid([-1 1],[-1 1],[-1 1]);
        payload_corners_local = payload_com + ...
            [sx(:)*h(1),sy(:)*h(2),sz(:)*h(3)];
    end

    geometry_cache = struct();
    geometry_cache.workspace = workspace;
    geometry_cache.body_names = robot.BodyNames;
    geometry_cache.names = names;
    geometry_cache.P0 = P0;
    geometry_cache.P1 = P1;
    geometry_cache.rad = rad;
    geometry_cache.nCaps = nCaps;
    geometry_cache.plane_enable = plane_enable;
    geometry_cache.self_enable = self_enable;
    geometry_cache.required_z = required_z;
    geometry_cache.pairs = pairs;
    geometry_cache.payload_name = payload_name;
    geometry_cache.payload_mass = payload_mass;
    geometry_cache.payload_com = payload_com;
    geometry_cache.payload_checked = payload_checked;
    geometry_cache.payload_corners_local = payload_corners_local;
    geometry_cache.payload_radius = payload_radius;
    geometry_cache.payload_collision_indices = payload_collision_indices;
    geometry_cache.self_margin = self_margin;
else
    names = geometry_cache.names;
    P0 = geometry_cache.P0;
    P1 = geometry_cache.P1;
    rad = geometry_cache.rad;
    nCaps = geometry_cache.nCaps;
    plane_enable = geometry_cache.plane_enable;
    self_enable = geometry_cache.self_enable;
    required_z = geometry_cache.required_z;
    pairs = geometry_cache.pairs;
    payload_name = geometry_cache.payload_name;
    payload_mass = geometry_cache.payload_mass;
    payload_com = geometry_cache.payload_com;
    payload_checked = geometry_cache.payload_checked;
    payload_corners_local = geometry_cache.payload_corners_local;
    payload_radius = geometry_cache.payload_radius;
    payload_collision_indices = geometry_cache.payload_collision_indices;
    self_margin = geometry_cache.self_margin;
end

min_z = inf;
min_z_sample = NaN;
min_z_primitive = '';
min_self_margin = inf;
min_self_sample = NaN;
min_self_pair = [NaN NaN];
min_self_pair_names = {};
for k = 1:size(Q,1)
    qk = double(Q(k,:));
    A = zeros(nCaps,3);
    B = zeros(nCaps,3);
    for ic = 1:nCaps
        T = getTransform(robot,qk,names{ic});
        A(ic,:) = transform_point_local(T,P0(ic,:));
        B(ic,:) = transform_point_local(T,P1(ic,:));
        zc = min(A(ic,3),B(ic,3)) - rad(ic);
        if zc < min_z
            min_z = zc;
            min_z_sample = k;
            min_z_primitive = ['capsule:' names{ic}];
        end
    end

    if payload_checked
        Tp = getTransform(robot,qk,payload_name);
        corners = transform_points_local(Tp,payload_corners_local);
        payload_center_world = transform_point_local(Tp,payload_com);
        [zp,ip] = min(corners(:,3));
        if zp < min_z
            min_z = zp;
            min_z_sample = k;
            min_z_primitive = sprintf('payload_corner:%d',ip);
        end
        if self_enable
            for ic = payload_collision_indices
                d = point_segment_distance_local( ...
                    payload_center_world,A(ic,:),B(ic,:));
                margin = d-payload_radius-rad(ic)-self_margin;
                if margin < min_self_margin
                    min_self_margin = margin;
                    min_self_sample = k;
                    min_self_pair = [ic 0];
                    min_self_pair_names = { ...
                        names{ic},'payload_obb_bounding_sphere'};
                end
            end
        end
    end

    for ipair = 1:size(pairs,1)
        ia = pairs(ipair,1);
        ib = pairs(ipair,2);
        d = segment_distance_local(A(ia,:),B(ia,:),A(ib,:),B(ib,:));
        margin = d - rad(ia) - rad(ib) - self_margin;
        if margin < min_self_margin
            min_self_margin = margin;
            min_self_sample = k;
            min_self_pair = [ia ib];
            min_self_pair_names = {names{ia},names{ib}};
        end
    end
end

if ~self_enable
    min_self_margin = inf;
end
tol = 1e-12;
workplane_pass = ~plane_enable || min_z >= required_z-tol;
self_pass = ~self_enable || min_self_margin >= -tol;
R = struct( ...
    'pass',logical(workplane_pass && self_pass), ...
    'workplane_pass',logical(workplane_pass), ...
    'self_collision_pass',logical(self_pass), ...
    'geometry_schema',char(workspace.geometry_schema), ...
    'min_geometry_z_m',min_z, ...
    'required_min_z_m',required_z, ...
    'workplane_margin_m',min_z-required_z, ...
    'min_z_sample_index',min_z_sample, ...
    'min_z_primitive',min_z_primitive, ...
    'min_self_collision_margin_m',min_self_margin, ...
    'min_self_collision_sample_index',min_self_sample, ...
    'min_self_collision_pair_indices',min_self_pair, ...
    'payload_box_checked',logical(payload_checked), ...
    'payload_mass_kg',payload_mass, ...
    'payload_com_tool_m',payload_com, ...
    'samples_checked',size(Q,1));
R.min_self_collision_pair_names = min_self_pair_names;
end

function p = transform_point_local(T,p0)
p = (double(T(1:3,1:3))*double(p0(:)) + double(T(1:3,4))).';
end

function P = transform_points_local(T,P0)
P0 = double(P0);

Rmat = double(T(1:3,1:3));
tcol = double(T(1:3,4));
% P0 is N-by-3: transpose it exactly once to 3-by-N, transform every
% column, then transpose the result once back to N-by-3.
P = bsxfun(@plus,Rmat*P0.',tcol).';
end

function d = segment_distance_local(p1,q1,p2,q2)
% Closest distance between two finite 3-D line segments, including
% degenerate zero-length capsules.
u = double(q1(:)-p1(:));
v = double(q2(:)-p2(:));
w = double(p1(:)-p2(:));
a = dot(u,u);
b = dot(u,v);
c = dot(v,v);
d0 = dot(u,w);
e = dot(v,w);
D = a*c-b*b;
small = 1e-15;
if a<=small
    d = point_segment_distance_local(p1,p2,q2);
    return;
elseif c<=small
    d = point_segment_distance_local(p2,p1,q1);
    return;
end
sD = D;
tD = D;
if D<small
    sN = 0; sD = 1;
    tN = e; tD = c;
else
    sN = b*e-c*d0;
    tN = a*e-b*d0;
    if sN<0
        sN=0; tN=e; tD=c;
    elseif sN>sD
        sN=sD; tN=e+b; tD=c;
    end
end
if tN<0
    tN=0;
    if -d0<0
        sN=0; sD=1;
    elseif -d0>a
        sN=sD;
    else
        sN=-d0; sD=a;
    end
elseif tN>tD
    tN=tD;
    if (-d0+b)<0
        sN=0; sD=1;
    elseif (-d0+b)>a
        sN=sD;
    else
        sN=-d0+b; sD=a;
    end
end
sc = 0;
tc = 0;
if abs(sN)>small, sc=sN/sD; end
if abs(tN)>small, tc=tN/tD; end
dp = w + sc*u - tc*v;
d = norm(dp);
end

function d = point_segment_distance_local(p,a,b)
p = double(p(:));
a = double(a(:));
b = double(b(:));
u = b-a;
den = dot(u,u);
if den<=1e-15
    d = norm(p-a);
    return;
end
t = min(1,max(0,dot(p-a,u)/den));
d = norm(p-(a+t*u));
end

function v = finite_scalar_local(v,name)
v = double(v);

end

function v = nonnegative_scalar_local(v,name)
v = finite_scalar_local(v,name);

end

function v = logical_scalar_local(v,name)

v = logical(v);
end
