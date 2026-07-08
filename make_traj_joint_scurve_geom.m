function TRAJ = make_traj_joint_scurve_geom(~, q0, qf, cfg)
%MAKE_TRAJ_JOINT_SCURVE_GEOM  Endpoint-exact joint-space geometric path.
%
% The path is q_g(s) = q0 + h(s) (qf-q0), s in [0,1], with smootherstep
% h(s)=10s^3-15s^4+6s^5.  It is a geometric path, not a time trajectory;
% the outer-loop progress rate alpha=ds/dt determines the execution time.
%
% Advantages for the revised paper/demo:
%   - q_g(0)=q0 and q_g(1)=qf exactly;
%   - q_s and q_ss are zero at the endpoints, so the CTC does not request
%     residual velocity/acceleration after s reaches the terminal point;
%   - no DLS-IK drift or Cartesian-loop artifact appears in the representative
%     end-effector plot;
%   - training, teacher generation, and online rollout all use the same q_s/q_ss.

    if nargin < 4 || isempty(cfg), cfg = struct(); end
    q0 = row(q0);
    qf = unwrap_to_near(q0, row(qf));
    nJ = numel(q0);

    DT     = getdef(cfg,'DT',0.01);
    NS_MIN = getdef(cfg,'NS_MIN',200);
    Ns = max(round(1/DT)+1, NS_MIN);
    s = linspace(0,1,Ns).';

    [h, hp, hpp] = smootherstep_all(s);
    dq = qf - q0;

    Q    = repmat(q0, Ns, 1) + h   * dq;
    QDg  = hp  * dq;
    QDDg = hpp * dq;

    Q(1,:) = q0;       Q(end,:) = qf;
    QDg(1,:) = 0;      QDg(end,:) = 0;
    QDDg(1,:) = 0;     QDDg(end,:) = 0;

    TRAJ.s    = s;
    TRAJ.Q    = Q;
    TRAJ.QDg  = QDg;
    TRAJ.QDDg = QDDg;
    TRAJ.eval = @(x) interp_geom(TRAJ, clamp01(x));
    TRAJ.Tf   = 1.0;
    TRAJ.kind = 'joint-scurve-geom';
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

function x = clamp01(x), x = max(0,min(1,x)); end
function r = row(v), r = double(v(:)).'; end
function qn = unwrap_to_near(qprev, qcur)
    d  = row(qcur) - row(qprev);
    qn = row(qprev) + atan2(sin(d), cos(d));
end
function val = getdef(S, f, def)
    if isstruct(S) && isfield(S,f) && ~isempty(S.(f)), val=S.(f); else, val=def; end
end
