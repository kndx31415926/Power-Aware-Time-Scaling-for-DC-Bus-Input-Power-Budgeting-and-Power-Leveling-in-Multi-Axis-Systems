function [a_des, info] = spinn3d_mpc_rg_grid_step(state, traj, robot, caps, bus, fric, areg, policy_dt, opts)
%SPINN3D_MPC_RG_GRID_STEP  Budget-aware receding-horizon RG grid baseline.
%
% This version is intentionally conservative.  It predicts the SAME filtered
% grid-input metric used in the paper, namely the first-order LPF of Pgrid, and
% rejects candidate progress-rate sequences that would exceed the active
% sustained budget.  This prevents the MPC/RG baseline from behaving like a
% budget-agnostic speed maximizer that only respects the 60 W hardware cap.

if nargin < 9 || isempty(opts), opts = struct(); end
policy_dt = max(double(policy_dt), 1e-6);

alpha_floor = getfield_def(opts,'alpha_floor', getfield_def(state,'alpha_floor',0.06));
alpha_floor = max(0.0, min(1.0, double(alpha_floor)));
Na    = max(3, round(getfield_def(opts,'Na',13)));
H     = max(1, round(getfield_def(opts,'H',5)));
beamN = max(1, round(getfield_def(opts,'beamN',15)));
allow_soft = logical(getfield_def(opts,'allow_soft_violation',false));

Pbud  = getfield_def(bus,'P_grid_budget', getfield_def(caps,'P_grid_budget', getfield_def(caps,'P_total_max',inf)));
Ppeak = getfield_def(bus,'P_grid_peak_max', getfield_def(caps,'P_grid_peak_max',inf));
Pbrk  = getfield_def(bus,'P_brk_peak', inf);
Vmin  = getfield_def(bus,'V_min', -inf);

% The paper/evaluation LPF.  If not supplied, use the same 0.05 s default as
% spinn3d_get_power_lpf_tau / PB.budget_ctrl.tau_power.
tau_lpf = getfield_def(opts,'tau_power', getfield_def(opts,'tau_lpf', getfield_def(opts,'power_tau_s',0.05)));
tau_lpf = max(double(tau_lpf), 1e-6);
betaP = exp(-policy_dt / tau_lpf);

% Enforce the active budget on the LPF signal.  Raw Pgrid is only guarded as an
% optional short-time diagnostic, because the manuscript metric is the LPF power.
budget_lpf_ratio = getfield_def(opts,'budget_lpf_ratio',0.98);
budget_lpf_ratio = max(0.05, double(budget_lpf_ratio));
raw_guard_ratio = getfield_def(opts,'raw_guard_ratio',inf);
raw_guard_ratio = double(raw_guard_ratio);
if ~isfinite(raw_guard_ratio) || raw_guard_ratio <= 0, raw_guard_ratio = inf; end
peak_guard_ratio = getfield_def(opts,'peak_guard_ratio',1.00);
peak_guard_ratio = max(0.05,double(peak_guard_ratio));

cand = getfield_def(opts,'candidate_grid',[]);
if isempty(cand)
    cand = linspace(alpha_floor, 1.0, Na);
else
    cand = double(cand(:).');
    cand = unique(min(1,max(alpha_floor,cand)),'stable');
end
if isempty(cand), cand = alpha_floor; end

W = getfield_def(opts,'W',struct());
W.progress = getfield_def(W,'progress',1.0);
W.budget   = getfield_def(W,'budget',2.0e4);
W.peak     = getfield_def(W,'peak',5.0e3);
W.dump     = getfield_def(W,'dump',200.0);
W.ramp     = getfield_def(W,'ramp',0.05);
W.slew     = getfield_def(W,'slew',0.02);
W.vdc      = getfield_def(W,'vdc',200.0);
W.track    = getfield_def(W,'track',0.20);

a_prev0 = min(1,max(alpha_floor,double(getfield_def(state,'a_prev',alpha_floor))));
Pbar0   = getfield_def(state,'Pgrid_lp',0);
if ~isfinite(Pbar0)
    Pbar0 = 0;
end
if isfinite(Pbud) && Pbud > 0
    Pbar0 = max(0,double(Pbar0));
else
    Pbar0 = 0;
end

beam = struct('cost',0,'s',double(state.s),'a_prev',a_prev0, ...
              'Pbar',Pbar0,'Praw_prev',Pbar0,'seq',[]);
beam = beam(:);

n_eval = 0;
n_reject_budget = 0;
n_reject_raw = 0;
n_reject_kin = 0;
for h = 1:H
    newbeam = repmat(beam(1),0,1);
    for b = 1:numel(beam)
        for ci = 1:numel(cand)
            a_cmd = cand(ci);
            [b2, met, ok, reason] = rollout_one(beam(b), a_cmd, traj, robot, caps, bus, fric, areg, policy_dt, alpha_floor, betaP, Pbud, Ppeak, Pbrk, Vmin, budget_lpf_ratio, raw_guard_ratio, peak_guard_ratio);
            n_eval = n_eval + 1;
            if ~ok
                switch reason
                    case 'budget', n_reject_budget = n_reject_budget + 1;
                    case 'raw',    n_reject_raw    = n_reject_raw + 1;
                    otherwise,     n_reject_kin    = n_reject_kin + 1;
                end
            end
            cadd = stage_cost(met, Pbud, Ppeak, Pbrk, Vmin, W, budget_lpf_ratio, raw_guard_ratio, peak_guard_ratio);
            if ok || allow_soft
                nb = b2;
                nb.cost = beam(b).cost + cadd;
                nb.seq = [beam(b).seq, a_cmd]; %#ok<AGROW>
                newbeam(end+1,1) = nb; %#ok<AGROW>
            end
        end
    end
    if isempty(newbeam)
        break;
    end
    [~,idx] = sort([newbeam.cost]);
    idx = idx(1:min(numel(idx),beamN));
    beam = newbeam(idx);
end

if isempty(beam)
    a_des = alpha_floor;
    info = struct('failed',true,'fallback','alpha_floor','n_candidates',numel(cand), ...
                  'H',H,'beamN',beamN,'n_eval',n_eval, ...
                  'n_reject_budget',n_reject_budget,'n_reject_raw',n_reject_raw,'n_reject_kin',n_reject_kin, ...
                  'budget_lpf_ratio',budget_lpf_ratio,'raw_guard_ratio',raw_guard_ratio, ...
                  'tau_lpf',tau_lpf,'allow_soft_violation',allow_soft);
    return;
end
[~,best] = min([beam.cost]);
bestBeam = beam(best);
if isempty(bestBeam.seq)
    a_des = alpha_floor;
else
    a_des = bestBeam.seq(1);
end
info = struct('failed',false,'cost',bestBeam.cost,'seq',bestBeam.seq, ...
              'n_candidates',numel(cand),'H',H,'beamN',beamN,'n_eval',n_eval, ...
              'n_reject_budget',n_reject_budget,'n_reject_raw',n_reject_raw,'n_reject_kin',n_reject_kin, ...
              'budget_lpf_ratio',budget_lpf_ratio,'raw_guard_ratio',raw_guard_ratio, ...
              'tau_lpf',tau_lpf,'allow_soft_violation',allow_soft, ...
              'Pbar_pred_end',bestBeam.Pbar);
end

function [b2, met, ok, reason] = rollout_one(b, a_des, traj, robot, caps, bus, fric, areg, dt, alpha_floor, betaP, Pbud, Ppeak, Pbrk, Vmin, budget_lpf_ratio, raw_guard_ratio, peak_guard_ratio)
a_des = min(1,max(alpha_floor,a_des));

% Same governor map, evaluated at policy-period resolution for the MPC preview.
a_slew = min(a_des, b.a_prev + getfield_def(areg,'A_DOT_UP',inf)*dt);
a_slew = max(a_slew, b.a_prev - getfield_def(areg,'A_DOT_DN',inf)*dt);
a = getfield_def(areg,'BETA',0)*b.a_prev + (1-getfield_def(areg,'BETA',0))*a_slew;
a = min(1,max(alpha_floor,a));
adot = (a - b.a_prev) / max(dt,1e-9);

s = min(traj.Tf, b.s);
[q, qs, qss] = traj.eval(s);
q = row(q); qs = row(qs); qss = row(qss);
qd = a * qs;
qdd = a*a*qss + adot*qs;

% Kinematic constraints.
vel_viol = max(max(abs(qd) ./ max(row(caps.qd_max),1e-12) - 1),0);
acc_viol = max(max(abs(qdd) ./ max(row(caps.qdd_max),1e-12) - 1),0);

try
    Mq = massMatrix(robot, q);
    c  = velocityProduct(robot, q, qd);
    g  = gravityTorque(robot, q);
    tau = (Mq*qdd(:) + c(:) + g(:)).';
catch
    tau = zeros(size(q));
end
if any(row(fric.B)) || any(row(fric.Fc))
    tau = tau + row(fric.B).*qd + row(fric.Fc).*tanh(qd./max(getfield_def(fric,'vel_eps',1e-3),1e-9));
end

tau_viol = max(max(abs(tau) ./ max(row(caps.tau_max),1e-12) - 1),0);
Paxis = tau.*qd;
Paxis_viol = max(max(max(Paxis,0)./max(row(caps.P_axis_max),1e-12) - 1),0);

try
    [~,~,Pgrid,Pdump,~,flow] = spinn3d_bus_power(tau, qd, bus);
catch
    Pgrid = max(sum(max(Paxis,0)),0);
    Pdump = 0;
    flow = struct('Pgrid',Pgrid,'Pdump',Pdump,'Pspill',0,'Vdc',getfield_def(bus,'V_nom',12));
end
Pbar_next = betaP * b.Pbar + (1-betaP) * Pgrid;
ramp = abs(Pgrid - b.Praw_prev) / max(dt,1e-9);
snext = min(traj.Tf, s + dt*a);

met = struct();
met.Pgrid = Pgrid;
met.Pbar = Pbar_next;
met.Pdump = Pdump;
met.Vdc = getfield_def(flow,'Vdc',NaN);
met.ramp = ramp;
met.progress = snext - s;
met.vel_viol = vel_viol;
met.acc_viol = acc_viol;
met.tau_viol = tau_viol;
met.Paxis_viol = Paxis_viol;
met.Pspill = getfield_def(flow,'Pspill',0);
met.Psource_shortfall = getfield_def(flow,'Psource_shortfall',0);
met.slew = abs(a - b.a_prev);
met.a = a;

kin_ok = (vel_viol<=0) && (acc_viol<=0) && (tau_viol<=0) && (Paxis_viol<=0);
budget_ok = true;
raw_ok = true;
peak_ok = true;
dump_ok = true;
source_ok = (met.Psource_shortfall <= 1e-9);
spill_ok  = (met.Pspill <= 1e-9);
if isfinite(Pbud) && Pbud > 0
    budget_ok = (Pbar_next <= budget_lpf_ratio * Pbud + 1e-9);
    raw_ok = (Pgrid <= raw_guard_ratio * Pbud + 1e-9);
end
if isfinite(Ppeak) && Ppeak > 0
    peak_ok = (Pgrid <= peak_guard_ratio * Ppeak + 1e-9);
end
if isfinite(Pbrk) && Pbrk > 0
    dump_ok = (Pdump <= Pbrk + 1e-9);
end
ok = kin_ok && budget_ok && raw_ok && peak_ok && dump_ok && source_ok && spill_ok;
reason = 'ok';
if ~kin_ok || ~dump_ok || ~source_ok || ~spill_ok
    reason = 'kin';
elseif ~budget_ok
    reason = 'budget';
elseif ~raw_ok || ~peak_ok
    reason = 'raw';
end

b2 = b;
b2.s = snext;
b2.a_prev = a;
b2.Pbar = Pbar_next;
b2.Praw_prev = Pgrid;
end

function J = stage_cost(met, Pbud, Ppeak, Pbrk, Vmin, W, budget_lpf_ratio, raw_guard_ratio, peak_guard_ratio)
if ~isfinite(Pbud) || Pbud<=0, Pbud = inf; end
if ~isfinite(Ppeak) || Ppeak<=0, Ppeak = inf; end
if ~isfinite(Pbrk) || Pbrk<=0, Pbrk = inf; end
rb = max(met.Pbar/max(budget_lpf_ratio*Pbud,eps)-1,0);
rr = 0;
if isfinite(raw_guard_ratio)
    rr = max(met.Pgrid/max(raw_guard_ratio*Pbud,eps)-1,0);
end
rp = max(met.Pgrid/max(peak_guard_ratio*Ppeak,eps)-1,0);
rd = max(met.Pdump/max(Pbrk,eps)-1,0);
rs = max(met.Psource_shortfall/max(min_noninf(Pbud,Ppeak,1),eps),0);
rspill = max(met.Pspill/max(min_noninf(Pbrk,Pbud,1),eps),0);
rv = max((Vmin - met.Vdc)/max(abs(Vmin),eps),0);
% The progress reward is active only after hard feasibility.  Budget and hard
% constraints dominate so that MPC/RG does not choose a 60 W launch simply to
% finish fast.
J = -W.progress*met.progress + W.budget*rb^2 + W.peak*(rp^2 + rr^2) + W.dump*rd^2 + ...
    W.ramp*(met.ramp/max(Pbud,eps))^2 + W.vdc*rv^2 + W.slew*met.slew^2 + ...
    W.peak*rs^2 + W.dump*rspill^2 + ...
    1000*(met.vel_viol^2 + met.acc_viol^2 + met.tau_viol^2 + met.Paxis_viol^2);
% Mild term encourages use of available budget without exceeding it.
if isfinite(Pbud) && Pbud > 0
    targetP = 0.92 * budget_lpf_ratio * Pbud;
    J = J + W.track*((met.Pbar - targetP)/max(Pbud,eps))^2;
end
end

function r = row(x), r = double(x(:)).'; end
function y = min_noninf(varargin)
vals = cellfun(@double,varargin); vals = vals(isfinite(vals) & vals>0);
if isempty(vals), y = 1; else, y = min(vals); end
end
function x = getfield_def(S,fn,def)
if isstruct(S) && isfield(S,fn) && ~isempty(S.(fn)), x = S.(fn); else, x = def; end
end
