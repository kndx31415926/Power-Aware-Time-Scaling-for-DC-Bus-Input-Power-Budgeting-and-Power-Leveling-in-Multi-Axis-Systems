function [robot,kin] = spinn3d_build_robot(params)
%SPINN3D_BUILD_ROBOT Build the nominal or caller-overridden RoArm model.
% The optional params input is intentionally supported because
% spinn3d_params_to_cfg forwards PB.robot.params to this builder.
if nargin < 1 || isempty(params)
    params = spinn3d_roarm_m2s_nominal_params();
end
[robot,kin] = spinn3d_robot(params);
try, robot.DataFormat = 'row'; end
end
