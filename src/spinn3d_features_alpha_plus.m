function x = spinn3d_features_alpha_plus(q0, dq0, ddq0, varargin)

%   A) (q0,dq0,ddq0,  Pbud, robot, kin,  caps, payload4)
%   B) (q0,dq0,ddq0,  robot, payload4, Pbud, caps)

Pbud = []; robot = []; kin = []; caps = struct(); payload4 = [];
need = @(n) (numel(varargin)>=n && ~isempty(varargin{n}));

if ~isempty(varargin)
    v1 = varargin{1};
    if is_rbt(v1) || (isobject(v1) && ismethod(v1,'inverseDynamics'))

        robot    = v1;
        if need(2), payload4 = varargin{2}; end
        if need(3), Pbud     = varargin{3}; end
        if need(4), caps     = varargin{4}; end
    else

        Pbud = v1;
        if need(2), robot    = varargin{2}; end
        if need(3), kin      = varargin{3}; end %#ok<NASGU>
        if need(4), caps     = varargin{4}; end
        if need(5), payload4 = varargin{5}; end
    end
end

fmt = getDataFormatSafe(robot);   
[qf,dqf,ddqf] = castToFmt(fmt, q0, dq0, ddq0);
asRow = @(v) reshape(v,1,[]);     
asCol = @(v) reshape(v,[],1);

tau_ff0 = inverseDynamics(robot, qf, dqf, ddqf);    
tau_row = asRow(tau_ff0);
dq_row  = asRow(dqf);
% Exact coefficient of alpha_dot in
% M(q)*(alpha^2*ddq_ref + alpha_dot*dq_ref).  Keeping this feature makes
% the training PINN torque residual match the plant-rate governor model.
M0 = massMatrix(robot, qf);
tau_adot_row = asRow(M0 * asCol(dq_row));
p_axis0 = max(tau_row .* dq_row, 0);
Pff0    = sum(p_axis0);

n = numel(asRow(qf));
z = zeros(1,n); if strcmp(fmt,'column'), z = asCol(z); end
gq0 = inverseDynamics(robot, qf, z, z);
g_row = asRow(gq0);

if isempty(payload4), payload4 = getPayload4OrZero(robot); end
limvec = double(Pbud);   
if isstruct(caps)
    if isfield(caps,'tau_max'),    limvec = [limvec, caps.tau_max(:)'];    end
    if isfield(caps,'P_axis_max'), limvec = [limvec, caps.P_axis_max(:)']; end
end

vx = zeros(1,3);
try
    ee = getEndEffectorNameSafe(robot);
    J  = geometricJacobian(robot, qf, ee);   % 6×n
    Jv = J(4:6,:);
    vx = asRow(Jv*asCol(asRow(dqf)));
catch

    vx = [norm(asRow(dqf)), 0, 0];
end

q_row  = asRow(qf);
sinq   = sin(q_row);  cosq = cos(q_row);  sgn_dq = sign(dq_row);
norms  = [norm(dq_row), norm(vx), norm(tau_row)];

dist_to_jlim = zeros(1, 2*numel(q_row));
try
    [lo,hi] = getJointLimitsRST(robot);
    if ~isempty(lo)
        lo = asRow(lo);
        hi = asRow(hi);
        finite_lim = isfinite(lo) & isfinite(hi) & isfinite(hi-lo) & ((hi-lo) > 1e-9);
        d_lo = zeros(1, numel(q_row));
        d_hi = zeros(1, numel(q_row));
        if any(finite_lim)
            span = hi(finite_lim) - lo(finite_lim);
            d_lo(finite_lim) = (q_row(finite_lim) - lo(finite_lim)) ./ span;
            d_hi(finite_lim) = (hi(finite_lim) - q_row(finite_lim)) ./ span;
        end
        dist_to_jlim = [d_lo, d_hi];
    end
catch
end

x = double([ ...
     q_row, dq_row, asRow(ddqf), ...
     tau_row, tau_adot_row, p_axis0, Pff0, ...
     g_row, ...
     payload4, ...
     limvec, ...
     vx, norms, ...
     sinq, cosq, sgn_dq, ...
     dist_to_jlim ...
]);
x(~isfinite(x)) = 0;
end

function tf = is_rbt(x)
tf = isa(x,'rigidBodyTree') || (isobject(x) && ismethod(x,'inverseDynamics'));
end
function fmt = getDataFormatSafe(robot)
try, fmt = robot.DataFormat; if ~ischar(fmt), fmt='row'; end
catch, fmt = 'row'; end
end
function [qf,dqf,ddqf] = castToFmt(fmt,q,dq,ddq)
if strcmp(fmt,'column')
    qf = q(:);  dqf = dq(:);  ddqf = ddq(:);
else
    qf = q(:)'; dqf = dq(:)'; ddqf = ddq(:)';
end
end
function payload4 = getPayload4OrZero(robot)
payload4 = [0 0 0 0];
try
    if isfield(robot,'tool')
        if isfield(robot.tool,'mass'), payload4(1)=robot.tool.mass; end
        if isfield(robot.tool,'com') && numel(robot.tool.com)>=3
            payload4(2:4) = robot.tool.com(1:3);
        end
    end
catch, end
end
function ee = getEndEffectorNameSafe(robot)
try, ee = robot.BodyNames{end}; catch, ee = robot.BodyNames{end}; end
end
function [lo,hi] = getJointLimitsRST(robot)
try
    names = robot.BodyNames;
    lo=[]; hi=[];
    for i=1:numel(names)
        b = robot.getBody(names{i});
        if ~strcmp(b.Joint.Type,'fixed')
            lim = b.Joint.PositionLimits; if isempty(lim), lim=[-inf,inf]; end
            lo(end+1)=lim(1); hi(end+1)=lim(2); %#ok<AGROW>
        end
    end
catch
    lo=[]; hi=[];
end
end
