function TRAJ = make_traj_cartesian_ik_geom(robot, q0, qf, cfg)
%MAKE_TRAJ_CARTESIAN_IK_GEOM  Cartesian-position geometric path + null-space IK.
%
% The end-effector reference is a smooth Cartesian curve between the start and
% target poses.  A damped least-squares IK tracker follows this Cartesian curve
% while a null-space regularization term keeps the joints close to a smooth
% q0->qf nominal path.  The output is still a geometric joint path q_g(s), so
% the outer-loop time-scaling formulation is unchanged.

    if nargin < 4 || isempty(cfg), cfg = struct(); end
    try, robot.DataFormat = 'row'; end

    q0 = row(q0);
    qf = unwrap_to_near(q0, row(qf));
    nJ = numel(q0);

    DT     = getdef(cfg,'DT',0.01);
    NS_MIN = getdef(cfg,'NS_MIN',200);
    Ns = max(round(1/DT)+1, NS_MIN);
    s = linspace(0,1,Ns).';

    eeName = getdef(cfg,'EE_NAME','');
    if isempty(eeName), eeName = robot.BodyNames{end}; end
    baseName = getdef(cfg,'BASE_NAME','');

    p0 = fk_pos(robot, q0, eeName, baseName);
    pf = fk_pos(robot, qf, eeName, baseName);

    lift_z = getdef(cfg,'CART_LIFT_Z', getdef(cfg,'BEZ_LIFT_Z',0.06));
    lam    = getdef(cfg,'IK_DAMPING', getdef(cfg,'DLS_LAMBDA',8e-3));
    w_nom  = getdef(cfg,'IK_NULL_WEIGHT',0.12);
    itmax  = max(1, round(getdef(cfg,'IK_MAX_ITER',15)));
    tol    = getdef(cfg,'IK_TOL_M',1e-4);
    maxStep = getdef(cfg,'IK_MAX_STEP_RAD',0.08);

    % Smooth scalar timing for the geometric curve; this is not the execution
    % time law, only the curve parameterization used to tabulate q_g(s).
    [h, ~, ~] = smootherstep_all(s);
    arch = 4*h.*(1-h);  % zero at endpoints, maximum at mid path
    P = repmat(p0(:).',Ns,1) + h*(pf(:).'-p0(:).') + arch*[0 0 lift_z];

    Qnom = repmat(q0,Ns,1) + h*(qf-q0);
    Q = zeros(Ns,nJ);
    Q(1,:) = q0;

    q = q0;
    for k = 2:Ns
        pdes = P(k,:).';
        qnom = Qnom(k,:);
        % Continue from previous solution, but never allow the redundant DOF to
        % drift too far from the endpoint-consistent nominal joint path.
        for it = 1:itmax
            pcur = fk_pos(robot, q, eeName, baseName);
            epos = pdes - pcur(:);
            if norm(epos) <= tol, break; end

            J = geometricJacobian(robot, q, eeName);
            Jlin = J(4:6,:);  % Robotics System Toolbox convention: [omega; v]

            % Solve min ||Jlin*dq-epos||^2 + w_nom^2||q+dq-qnom||^2 + lam^2||dq||^2.
            A = [Jlin; w_nom*eye(nJ)];
            b = [epos; w_nom*(qnom(:)-q(:))];
            dq = (A.'*A + (lam^2)*eye(nJ)) \ (A.'*b);
            dq = row(dq);

            nd = norm(dq);
            if nd > maxStep
                dq = dq * (maxStep / max(nd,eps));
            end
            q = unwrap_to_near(q, q + dq);
        end
        % Softly keep the redundant solution close to Qnom without sacrificing
        % the Cartesian path too much.
        Q(k,:) = q;
    end

    % Terminal correction.  Check the natural IK endpoint BEFORE forcing qf.
    % The previous implementation assigned Q(end,:)=qf before computing eend,
    % which made the terminal-blend test always pass and could leave a hard last-
    % sample jump that pollutes finite-difference velocity/acceleration.
    q_end_ik = Q(end,:);
    eend_cart  = norm(fk_pos(robot, q_end_ik, eeName, baseName) - pf(:));
    eend_joint = norm(qf - q_end_ik);
    if eend_cart > 5e-4 || eend_joint > 1e-3
        w = terminal_blend_weight(s,0.82,1.00);
        Q = Q + w*(qf - q_end_ik);
    end

    % Exact endpoints are still enforced after the smooth terminal correction.
    Q(1,:) = q0;
    Q(end,:) = qf;

    [QDg,QDDg] = finite_geom_derivatives(Q,s);
    QDg(1,:) = 0; QDg(end,:) = 0;
    QDDg(1,:) = 0; QDDg(end,:) = 0;

    TRAJ.s    = s;
    TRAJ.Q    = Q;
    TRAJ.QDg  = QDg;
    TRAJ.QDDg = QDDg;
    TRAJ.eval = @(x) interp_geom(TRAJ, clamp01(x));
    TRAJ.Tf   = 1.0;
    TRAJ.kind = 'cartesian-ik-scurve-geom';
    TRAJ.Pcart = P;
end

function [QDg,QDDg] = finite_geom_derivatives(Q,s)
    [Ns,nJ] = size(Q);
    QDg = zeros(Ns,nJ); QDDg = zeros(Ns,nJ);
    if Ns < 3, return; end
    ds = s(2)-s(1);
    QDg(2:Ns-1,:)  = (Q(3:Ns,:) - Q(1:Ns-2,:))/(2*ds);
    QDg(1,:)       = (Q(2,:) - Q(1,:))/ds;
    QDg(Ns,:)      = (Q(Ns,:) - Q(Ns-1,:))/ds;
    QDDg(2:Ns-1,:) = (Q(3:Ns,:) - 2*Q(2:Ns-1,:) + Q(1:Ns-2,:))/(ds^2);
    QDDg(1,:)      = QDDg(2,:);
    QDDg(Ns,:)     = QDDg(Ns-1,:);
end

function [q, qdg, qddg] = interp_geom(TR, x)
    q    = interp1(TR.s, TR.Q,    x, 'linear', 'extrap');
    qdg  = interp1(TR.s, TR.QDg,  x, 'linear', 'extrap');
    qddg = interp1(TR.s, TR.QDDg, x, 'linear', 'extrap');
end

function [h, hp, hpp] = smootherstep_all(x)
    x = clamp01(x(:));
    h   = x.^3 .* (10 - 15*x + 6*x.^2);
    hp  = 30*x.^2 - 60*x.^3 + 30*x.^4;
    hpp = 60*x - 180*x.^2 + 120*x.^3;
end

function w = terminal_blend_weight(s,s0,s1)
    u = (s(:)-s0)/max(s1-s0,eps);
    u = clamp01(u);
    w = u.^3 .* (10 - 15*u + 6*u.^2);
end

function p = fk_pos(robot,q,eeName,baseName)
    if ~isempty(baseName)
        T = getTransform(robot, row(q), eeName, baseName);
    else
        T = getTransform(robot, row(q), eeName);
    end
    p = T(1:3,4);
end

function x = clamp01(x), x = max(0,min(1,x)); end
function r = row(v), r = double(v(:)).'; end
function qn = unwrap_to_near(qprev, qcur)
    d  = row(qcur) - row(qprev);
    qn = row(qprev) + atan2(sin(d), cos(d));
end
function val = getdef(S, f, def)
    if isstruct(S) && isfield(S,f) && ~isempty(S.(f)), val=S.(f); else, val=def; end
end
