function p = spinn3d_roarm_m2s_nominal_params(overrides)
%SPINN3D_ROARM_M2S_NOMINAL_PARAMS Nominal stock-geometry RoArm-M2-S model.
%
% The stock rigid-body values below are mapped from Waveshare's public
% RoArm-M2-S URDF.  The 50-mm object used in the paper study is NOT a robot
% link: the gripper remains at the stock wrist location and the elongated
% workpiece is injected at runtime through the massless `tool` body.
%
% IMPORTANT MODEL BOUNDARY
%   * fixed_base_* is the non-moving pedestal and must not be lumped into
%     the base-yaw inertia.
%   * mass(3) contains only the permanent stock gripper-link CAD body.
%   * the fixed `tool` body remains massless. Runtime payload injection
%     represents the grasped long workpiece only and uses mode='replace'.
%   * workpiece_reference below is geometry metadata for the paper task; it
%     is deliberately excluded from robot link length/mass/inertia.
%   * the public URDF masses sum to about 0.6355 kg, below the published
%     0.826-kg complete-arm mass. This nominal profile proportionally scales
%     stock URDF spatial inertias to match the published total; measured
%     assembly masses are still required for an identified digital twin.
%   * ST3215 internal position/velocity loops are not identified here. The
%     plant is an uncalibrated equivalent output-torque abstraction; its
%     simulated tau_cmd must not be sent directly to the real serial servo.

if nargin < 1 || isempty(overrides)
    overrides = struct();
end

if isfield(overrides,'derived_complete') && ~isempty(overrides.derived_complete) && ...
        logical(overrides.derived_complete(1))
    error('spinn3d_roarm_m2s_nominal_params:DerivedStructAsOverride', ...
        ['A fully derived robot-parameter struct cannot be reused as raw ', ...
         'overrides. Edit PB.robot.param_overrides instead.']);
end

p = struct();
p.profile_id = 'roarm_m2s_stock_urdf_long_workpiece_v2';
p.calibration_status = ['Stock total mass matched to published 0.826 kg by ', ...
    'proportional URDF scaling; grasped workpiece is a runtime payload; ', ...
    'component measurements pending'];
p.model_name = 'stock-geometry-roarm-m2-s-with-runtime-long-workpiece';
p.topology_assumption = 'RoArm firmware EEMode=1: servo 15 is wrist pitch, not gripper opening';
p.inertial_source = 'Waveshare roarm_description.urdf; long workpiece injected as runtime payload';
p.actuator_model_boundary = [ ...
    'Uncalibrated equivalent output-torque abstraction; real ST3215 ', ...
    'internal position/velocity loops are outside this model and tau_cmd ', ...
    'is not a directly deployable hardware command'];
p.gravity = [0 0 -9.81];

% Reduced 4R geometry.  The small stock link offsets are collapsed into
% equivalent centre-to-centre lengths so the existing +Z yaw / +Y pitch
% controller convention is retained.
p.base_joint_xyz = [0.0100000008759151 0 0.123059270461044];
p.base_z = p.base_joint_xyz(3);
p.stock_upper_length_m = hypot(0.236815132922094, 0.0300023995170449);
p.stock_forearm_length_m = hypot(0.002906, 0.21599);
p.stock_wrist_length_m = hypot(0.06785, 0.00598);

% Position limits intentionally preserve the project's existing coordinate
% convention.  Replace them with measured zero-offset/hard-stop limits for
% the exact EEMode=1 hardware before experimental deployment.
p.jposlim = deg2rad([-180 180; -90 90; -150 150; -180 180]);

% Non-moving official URDF base_link.  It is represented as a fixed body so
% it contributes to total centre-of-mass reporting but not joint dynamics.
p.fixed_base_mass = 0.326392759093976;
p.fixed_base_com_xyz = [ ...
    0.00373847392098165, -1.52445893424664e-10, 0.0264265327292804];
p.fixed_base_I6_com = [ ...
    0.000224202818963409, 0.000286616921839535, 0.000349957607258242, ...
   -3.10964171549742e-13, -2.99224798427881e-05, -1.96773409840189e-13];

% Moving yaw body: official URDF link1, expressed at the yaw-joint frame.
p.base_mass = 0.0928887601998397;
p.base_com_xyz = [ ...
   -3.11559626639357e-12, 9.70240468164796e-06, -0.0146667419724465];
p.base_I6_com = [ ...
    6.09210980255232e-05, 2.53493566630784e-05, 4.88050835919342e-05, ...
    2.16442830142331e-08, 4.24841739939559e-15, -2.79207542479444e-18];

% Official link2/link3 values rotated into the reduced body frames:
% +X follows the outgoing link and +Y is the pitch-joint axis.
p.mass = [0.0895817175833772, 0.123009044986454, NaN];
p.com_xyz = [ ...
     0.121061842048527,   -9.50311342843365e-05,  0.015270810710803552; ...
     0.11439903962751329, -0.000177274968057332,  0.00230204053639010; ...
     NaN,          NaN,                    NaN];
p.I6_com = [ ...
    3.0752423446815196e-05, 3.3305683323084600e-04, 3.4588646683634980e-04, ...
    6.2793719903529110e-09,-3.7939237146567876e-05,-4.7291083151741930e-09; ...
    1.4649045313469763e-05, 1.0400929771010600e-04, 1.0409855312808143e-04, ...
   -1.5270070805557302e-07,-1.0972184507425480e-06, 1.6642456178711043e-07; ...
    NaN, NaN, NaN, NaN, NaN, NaN];

% Official gripper_link CAD body, rotated into the reduced wrist frame.
p.permanent_end.mass = 0.00365697683069397;
p.permanent_end.com_xyz = [ ...
    0.0288857415118809, -0.000779764534254559, 0.000776842039187049];
p.permanent_end.I6_com = [ ...
    1.15139272656909e-06, 1.47179863096272e-06, 2.33034171341163e-06, ...
   -4.13922575054970e-13, -1.31250385773157e-07, -2.41954534039415e-13];

% Reference geometry for the grasped long workpiece used by the paper task.
% This is NOT a robot extension and is never added to p.L or p.mass.
p.workpiece_reference.description = '50-mm end-grasped long workpiece';
p.workpiece_reference.length_m = 0.050;
p.workpiece_reference.width_m = 0.020;
p.workpiece_reference.height_m = 0.020;
p.workpiece_reference.com_from_tool_m = [0.025 0 0];
p.workpiece_reference.grasp_mode = 'one_end_along_tool_x';
p.workpiece_reference.included_in_robot_dynamics = false;

% explicitly as non-robot hardware and must not be used to extend kinematics.
p.extension = struct('material','workpiece_reference_only', ...
    'length_m',p.workpiece_reference.length_m, ...
    'outer_diameter_m',p.workpiece_reference.width_m, ...
    'inner_diameter_m',0, ...
    'density_kg_m3',NaN, ...
    'start_x_m',0, ...
    'mass_kg',0, ...
    'included_in_robot_dynamics',false);

% Visual-only dimensions; none of these are used to fabricate inertias.
p.base_radius = 0.055;
p.base_height = 0.060;
p.link_radius = 0.010;

% Explicit hardware/model coordinate interface.  Identity values are only a
% placeholder for simulation and are deliberately marked uncalibrated.
p.hardware_map.sign = [1 1 1 1];
p.hardware_map.zero_offset_rad = [0 0 0 0];
p.hardware_map.calibrated = false;
p.hardware_map.convention = 'q_model = sign .* q_hw + zero_offset_rad';

% Merge caller changes exactly once while all mass/inertia entries are still
% raw source inputs.  Every dependent value below is then regenerated.
p = merge_struct_recursive(p, overrides);

% takes precedence when both forms are supplied.
if isfield(overrides,'base_z') && ~isempty(overrides.base_z) && ...
        (~isfield(overrides,'base_joint_xyz') || isempty(overrides.base_joint_xyz))
    p.base_joint_xyz(3) = double(overrides.base_z);
end
p.base_z = double(p.base_joint_xyz(3));

% The public URDF inertial entries sum to only about 0.635529 kg whereas the
% manufacturer publishes 0.826 +/- 0.015 kg for the complete RoArm-M2-S.
% Missing servo/cable/controller/fastener mass cannot be located uniquely
% without weighing assemblies.  As a traceable nominal assumption, scale
% every stock URDF mass and COM tensor by the same factor.  COM positions are
% unchanged.  Replace these fields with measured assemblies for a true twin.
if ~isfield(overrides,'raw_urdf_mass_kg') || isempty(overrides.raw_urdf_mass_kg)
    p.raw_urdf_mass_kg = double(p.fixed_base_mass) + double(p.base_mass) + ...
        sum(double(p.mass(1:2))) + double(p.permanent_end.mass);
end
if ~isfield(p,'published_stock_mass_kg') || isempty(p.published_stock_mass_kg)
    p.published_stock_mass_kg = 0.826;
end
p.urdf_mass_scale = double(p.published_stock_mass_kg) / ...
    double(p.raw_urdf_mass_kg);

p.fixed_base_mass = p.fixed_base_mass * p.urdf_mass_scale;
p.fixed_base_I6_com = p.fixed_base_I6_com * p.urdf_mass_scale;
p.base_mass = p.base_mass * p.urdf_mass_scale;
p.base_I6_com = p.base_I6_com * p.urdf_mass_scale;
p.mass(1:2) = p.mass(1:2) * p.urdf_mass_scale;
p.I6_com(1:2,:) = p.I6_com(1:2,:) * p.urdf_mass_scale;
p.permanent_end.mass = p.permanent_end.mass * p.urdf_mass_scale;
p.permanent_end.I6_com = p.permanent_end.I6_com * p.urdf_mass_scale;

if ~isfield(overrides,'L') || isempty(overrides.L)
    p.L = [p.stock_upper_length_m, p.stock_forearm_length_m, ...
           p.stock_wrist_length_m];
else
    p.L = double(overrides.L(:)).';
end

% The moving wrist contains only the permanent stock gripper-link body.
% A grasped workpiece is injected later into the massless tool body.
has_m3 = isfield(overrides,'mass') && numel(overrides.mass) >= 3 && ...
         isfinite(overrides.mass(3));
has_c3 = isfield(overrides,'com_xyz') && size(overrides.com_xyz,1) >= 3 && ...
         size(overrides.com_xyz,2) >= 3 && all(isfinite(overrides.com_xyz(3,1:3)));
has_I3 = isfield(overrides,'I6_com') && size(overrides.I6_com,1) >= 3 && ...
         size(overrides.I6_com,2) >= 6 && all(isfinite(overrides.I6_com(3,1:6)));
if ~(has_m3 && has_c3 && has_I3)
    p.mass(3) = p.permanent_end.mass;
    p.com_xyz(3,:) = p.permanent_end.com_xyz;
    p.I6_com(3,:) = p.permanent_end.I6_com;
end

p.com_ratio = p.com_xyz(:,1).' ./ max(p.L, eps); 
p.nominal_urdf_mass_kg = p.raw_urdf_mass_kg; 
p.stock_mass_kg = p.published_stock_mass_kg;
p.stock_moving_mass_kg = p.base_mass + sum(p.mass);
p.extension_mass_kg = 0; 
p.permanent_end_mass_kg = p.permanent_end.mass;
p.nominal_moving_mass_kg = p.base_mass + sum(p.mass);
p.nominal_model_mass_kg = p.fixed_base_mass + p.base_mass + sum(p.mass);
p.mass_scope_note = ['Published stock total mass with proportional URDF spatial ', ...
    'distribution. The 50-mm long workpiece is excluded here and injected ', ...
    'through the runtime payload body.'];
sgn = double(p.hardware_map.sign(:).');
zoff = double(p.hardware_map.zero_offset_rad(:).');

p.hardware_map.sign = sgn;
p.hardware_map.zero_offset_rad = zoff;
p.hardware_map.calibrated = logical(p.hardware_map.calibrated(1));
p.derived_complete = true;
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

function I = sanitize_psd(I, floor_eig)
I = 0.5*(double(I)+double(I).');
[V,D] = eig(I);
d = real(diag(D));
d(~isfinite(d)) = floor_eig;
d = max(d, floor_eig);
I = real(V*diag(d)*V.');
I = 0.5*(I+I.');
end

function out = merge_struct_recursive(base, over)
out = base;
if isempty(over), return; end
fn = fieldnames(over);
for k = 1:numel(fn)
    f = fn{k};
    if isfield(out,f) && isstruct(out.(f)) && isstruct(over.(f))
        out.(f) = merge_struct_recursive(out.(f), over.(f));
    else
        out.(f) = over.(f);
    end
end
end
