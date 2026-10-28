function [robot, kin] = spinn3d_robot(params)
%SPINN3D_ROBOT Reduced 4R stock-geometry RoArm-M2-S model.
%
% Joint convention retained by the existing controller:
%   q1 : base yaw about +Z
%   q2-q4 : shoulder/elbow/wrist pitch about +Y
%
% Rigid-body convention required by Robotics System Toolbox:
%   CenterOfMass is expressed in the body frame, while Inertia is the
%   inertia tensor about the body-frame origin.  Source URDF tensors are
%   about each COM, so every tensor is PSD-sanitized at the COM and shifted
%   to the body origin before addBody is called.
%
% Permanent stock gripper hardware is part of link_wrist. The 256-mm long
% workpiece is NOT a link extension; it is injected into the final massless
% `tool` body at runtime, so mode='replace' changes only the grasped object.

if nargin < 1 || isempty(params)
    params = spinn3d_roarm_m2s_nominal_params();
else

    if ~(isfield(params,'derived_complete') && ...
            ~isempty(params.derived_complete) && logical(params.derived_complete(1)))
        % Raw caller overrides are merged and all dependent fields regenerated.
        params = spinn3d_roarm_m2s_nominal_params(params);
    end
end

L = require_row(params.L,3,'params.L');
m = require_row(params.mass,3,'params.mass');
C = double(params.com_xyz);
I6C = double(params.I6_com);

base_joint_xyz = require_row(params.base_joint_xyz,3,'params.base_joint_xyz');
jposlim = double(params.jposlim);

robot = rigidBodyTree('DataFormat','row','MaxNumBodies',6);
robot.Gravity = require_row(params.gravity,3,'params.gravity');

% ---------------------------------------------------------------------
% Fixed official base_link.  This body is deliberately outside the yaw
% subtree so its mass is not accelerated by q1.
base_fixed = rigidBody('base_fixed');
jf = rigidBodyJoint('fix_base','fixed');
setFixedTransform(jf,eye(4));
base_fixed.Joint = jf;
[mf,cf,If_o] = inertial_from_com( ...
    params.fixed_base_mass, params.fixed_base_com_xyz, ...
    params.fixed_base_I6_com, 'base_fixed');
base_fixed.Mass = mf;
base_fixed.CenterOfMass = cf;
base_fixed.Inertia = If_o;
try
    Tvis = trvec2tform([0 0 0.5*double(params.base_height)]);
    addVisual(base_fixed,'Cylinder', ...
        [double(params.base_radius),double(params.base_height)],Tvis);
catch
end
addBody(robot,base_fixed,robot.BaseName);

% Moving official link1: base-yaw assembly.
b1 = rigidBody('link_baseYaw');
j1 = rigidBodyJoint('joint_baseYaw','revolute');
j1.JointAxis = [0 0 1];
j1.PositionLimits = jposlim(1,:);
setFixedTransform(j1,trvec2tform(base_joint_xyz));
b1.Joint = j1;
[m1,c1,I1o] = inertial_from_com( ...
    params.base_mass,params.base_com_xyz,params.base_I6_com,'link_baseYaw');
b1.Mass = m1;
b1.CenterOfMass = c1;
b1.Inertia = I1o;
try
    addVisual(b1,'Cylinder',[0.035 0.045],trvec2tform([0 0 -0.0225]));
catch
end
addBody(robot,b1,'base_fixed');

% Official link2 mapped to the reduced shoulder-link frame.
b2 = rigidBody('link_shoulder');
j2 = rigidBodyJoint('joint_shoulder','revolute');
j2.JointAxis = [0 1 0];
j2.PositionLimits = jposlim(2,:);
setFixedTransform(j2,eye(4));
b2.Joint = j2;
[m2,c2,I2o] = inertial_from_com(m(1),C(1,:),I6C(1,:),'link_shoulder');
b2.Mass = m2;
b2.CenterOfMass = c2;
b2.Inertia = I2o;
try
    addVisual(b2,'Cylinder',[double(params.link_radius),L(1)],link_visual_T(L(1)));
catch
end
addBody(robot,b2,'link_baseYaw');

% Official link3 mapped to the reduced elbow-link frame.
b3 = rigidBody('link_elbow');
j3 = rigidBodyJoint('joint_elbow','revolute');
j3.JointAxis = [0 1 0];
j3.PositionLimits = jposlim(3,:);
setFixedTransform(j3,trvec2tform([L(1) 0 0]));
b3.Joint = j3;
[m3,c3,I3o] = inertial_from_com(m(2),C(2,:),I6C(2,:),'link_elbow');
b3.Mass = m3;
b3.CenterOfMass = c3;
b3.Inertia = I3o;
try
    addVisual(b3,'Cylinder',[double(params.link_radius),L(2)],link_visual_T(L(2)));
catch
end
addBody(robot,b3,'link_shoulder');

% Permanent wrist body: official stock gripper_link CAD body only. The
% grasped long workpiece is carried by the separate massless tool body.
b4 = rigidBody('link_wrist');
j4 = rigidBodyJoint('joint_wrist','revolute');
j4.JointAxis = [0 1 0];
j4.PositionLimits = jposlim(4,:);
setFixedTransform(j4,trvec2tform([L(2) 0 0]));
b4.Joint = j4;
[m4,c4,I4o] = inertial_from_com(m(3),C(3,:),I6C(3,:),'link_wrist');
b4.Mass = m4;
b4.CenterOfMass = c4;
b4.Inertia = I4o;
try
    addVisual(b4,'Cylinder', ...
        [double(params.link_radius),L(3)],link_visual_T(L(3)));
catch
end
addBody(robot,b4,'link_elbow');

% Massless fixed payload carrier.  If a construction-time payload is
% explicitly supplied, compute and assign it before addBody.
tool = rigidBody('tool');
jt = rigidBodyJoint('fix_tool','fixed');
setFixedTransform(jt,trvec2tform([L(3) 0 0]));
tool.Joint = jt;
[mt,ct,Ito] = payload_inertial(params);
tool.Mass = mt;
tool.CenterOfMass = ct;
tool.Inertia = Ito;
try
    addVisual(tool,'Sphere',0.012,eye(4));
catch
end
addBody(robot,tool,'link_wrist');

kin = struct();
kin.profile_id = params.profile_id;
kin.calibration_status = params.calibration_status;
kin.model_name = params.model_name;
kin.topology_assumption = params.topology_assumption;
kin.inertial_source = params.inertial_source;
kin.L = L;
kin.base_z = base_joint_xyz(3);
kin.base_joint_xyz = base_joint_xyz;
kin.jposlim = jposlim;
kin.mass = m;
kin.com_xyz = C;
kin.I6_com = I6C;
kin.com_ratio = C(:,1).' ./ L;
kin.link_radius = params.link_radius;
kin.base_mass = params.base_mass;
kin.base_com_xyz = params.base_com_xyz;
kin.fixed_base_mass = params.fixed_base_mass;
kin.extension = params.extension; % deprecated non-robot geometry alias
if isfield(params,'workpiece_reference'), kin.workpiece_reference = params.workpiece_reference; end
kin.stock_mass_kg = params.stock_mass_kg;
kin.stock_moving_mass_kg = params.stock_moving_mass_kg;
kin.extension_mass_kg = params.extension_mass_kg;
kin.permanent_end_mass_kg = params.permanent_end_mass_kg;
kin.nominal_moving_mass_kg = params.nominal_moving_mass_kg;
kin.nominal_model_mass_kg = params.nominal_model_mass_kg;
kin.mass_scope_note = params.mass_scope_note;
kin.hardware_map = params.hardware_map;
end

% =====================================================================
function [m,c,I6_O] = inertial_from_com(m,c,I6_C,label)
m = double(m);
c = require_row(c,3,[label ' COM']);
I6_C = require_row(I6_C,6,[label ' I6_com']);

IC = sanitize_psd(I6_to_mat(I6_C),1e-12,label);
IO = IC + m*parallel_axis(c);
IO = sanitize_psd(IO,1e-12,[label ' origin inertia']);
I6_O = mat_to_I6(IO);
end

function [m,c,I6_O] = payload_inertial(params)
m = 0;
c = [0 0 0];
I6_O = zeros(1,6);
if ~isfield(params,'payload') || isempty(params.payload) || ...
        ~isstruct(params.payload) || ~isfield(params.payload,'mass') || ...
        isempty(params.payload.mass) || double(params.payload.mass)<=0
    return;
end

P = params.payload;
m = double(P.mass);
if isfield(P,'com') && ~isempty(P.com)
    c = require_row(P.com,3,'params.payload.com');
end

about = 'com';
if isfield(P,'about') && ~isempty(P.about), about = lower(char(P.about)); end

if isfield(P,'I6_com') && ~isempty(P.I6_com)
    I6 = require_row(P.I6_com,6,'params.payload.I6_com');
    about = 'com';
elseif isfield(P,'inertia') && ~isempty(P.inertia)
    I6 = require_row(P.inertia,6,'params.payload.inertia');
else
    I6 = payload_shape_I6_com(P,m);
    about = 'com';
end

if strcmpi(about,'origin')
    IO = sanitize_psd(I6_to_mat(I6),1e-12,'payload origin inertia');
    I6_O = mat_to_I6(IO);
else
    [m,c,I6_O] = inertial_from_com(m,c,I6,'payload');
end
end

function I6 = payload_shape_I6_com(P,m)
shape = 'box';
if isfield(P,'shape') && ~isempty(P.shape), shape = lower(char(P.shape)); end
dims = struct();
if isfield(P,'dims') && isstruct(P.dims), dims = P.dims; end
switch shape
    case 'box'
        a = getdef(dims,'a',0.06); b = getdef(dims,'b',0.04); c = getdef(dims,'c',0.03);
        I6 = [(m/12)*(b^2+c^2),(m/12)*(a^2+c^2),(m/12)*(a^2+b^2),0,0,0];
    case 'cyl_x'
        r = getdef(dims,'r',0.015); h = getdef(dims,'h',0.06);
        I6 = [0.5*m*r^2,(m/12)*(3*r^2+h^2),(m/12)*(3*r^2+h^2),0,0,0];
    case 'cyl_z'
        r = getdef(dims,'r',0.015); h = getdef(dims,'h',0.06);
        I6 = [(m/12)*(3*r^2+h^2),(m/12)*(3*r^2+h^2),0.5*m*r^2,0,0,0];
    case 'sphere'
        r = getdef(dims,'r',0.03); I = (2/5)*m*r^2;
        I6 = [I I I 0 0 0];
    otherwise
        error('Unknown params.payload.shape = %s',shape);
end
end

function T = link_visual_T(L)
% MATLAB cylinders are Z-aligned.  Translate their centre after rotating
% the cylinder axis onto +X.
T = trvec2tform([0.5*L 0 0]) * troty(pi/2);
end

function S = parallel_axis(r)
r = double(r(:));
S = (r.'*r)*eye(3) - r*r.';
end

function I = I6_to_mat(v)
v = double(v(:)).';
I = [v(1),v(6),v(5); v(6),v(2),v(4); v(5),v(4),v(3)];
end

function v = mat_to_I6(I)
I = 0.5*(I+I.');
v = [I(1,1),I(2,2),I(3,3),I(2,3),I(1,3),I(1,2)];
end

function I = sanitize_psd(I,floor_eig,label)
I = double(I);

I = 0.5*(I+I.');
[V,D] = eig(I);
d = real(diag(D));
if any(d < -1e-10)
    warning('spinn3d_robot:InertiaPSD', ...
        '%s had a negative principal inertia (min %.3g); projected to PSD.', ...
        label,min(d));
end
d = max(d,floor_eig);
I = real(V*diag(d)*V.');
I = 0.5*(I+I.');
end

function r = require_row(v,n,label)
r = double(v(:)).';

end

function v = getdef(S,f,default)
if isstruct(S) && isfield(S,f) && ~isempty(S.(f)), v=double(S.(f)); else, v=default; end
end

function T = troty(a)
c=cos(a); s=sin(a);
T=eye(4); T(1:3,1:3)=[c 0 s; 0 1 0; -s 0 c];
end
