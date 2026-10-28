function check = spinn3d_endpoint_yaw_branch_check(robot,q0,qf,ee)
% Evaluate the endpoint branch of the continuous yaw trajectory.

if nargin < 4 || isempty(ee)
    ee = robot.BodyNames{end};
end

check = struct('schema','signed_yaw_reach_same_branch_v1', ...
    'available',false,'compatible',true,'rho0_m',NaN,'rhof_m',NaN, ...
    'branch_eps_m',1e-4,'yaw_body','', ...
    'reason','yaw-branch guard unavailable');

yaw_body = '';
for ib = 1:numel(robot.Bodies)
    body = robot.Bodies{ib};
    if strcmpi(char(body.Joint.Type),'fixed'), continue; end
    axis = row_local(body.Joint.JointAxis);
    if numel(axis)==3 && abs(axis(3))>=0.9 && norm(axis(1:2))<=0.1
        yaw_body = body.Name;
        break;
    end
end
if isempty(yaw_body)
    return;
end

rho0 = signed_reach_local(robot,q0,yaw_body,ee);
rhof = signed_reach_local(robot,qf,yaw_body,ee);
available = isfinite(rho0) && isfinite(rhof);
eps_branch = check.branch_eps_m;
compatible = ~available || (abs(rho0)>eps_branch && ...
    abs(rhof)>eps_branch && sign(rho0)==sign(rhof));

check.available = logical(available);
check.compatible = logical(compatible);
check.rho0_m = double(rho0);
check.rhof_m = double(rhof);
check.yaw_body = char(yaw_body);
if ~available
    check.reason = 'signed yaw reach is unavailable';
elseif abs(rho0)<=eps_branch || abs(rhof)<=eps_branch
    check.reason = 'endpoint lies on the yaw-axis singularity';
elseif sign(rho0)~=sign(rhof)
    check.reason = 'endpoint signed reaches use different yaw branches';
else
    check.reason = 'compatible';
end
end

function rho = signed_reach_local(robot,q,yaw_body,ee)
Ty = getTransform(robot,row_local(q),yaw_body);
Te = getTransform(robot,row_local(q),ee);
dxy = Te(1:2,4)-Ty(1:2,4);
xdir = Ty(1:2,1);
rho = dot(dxy,xdir);
end

function y = row_local(x)
y = double(x(:).');
end
