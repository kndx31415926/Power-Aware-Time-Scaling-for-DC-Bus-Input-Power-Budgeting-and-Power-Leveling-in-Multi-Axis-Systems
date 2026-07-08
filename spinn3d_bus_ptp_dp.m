function profile = spinn3d_bus_ptp_dp(traj, robot, limits, cfg, opts)
%SPINN3D_BUS_PTP_DP  Grid-based power-constrained path time-parameterization baseline.
%
% The method solves a simple dynamic-programming problem over path progress
% and progress rate.  It is intentionally implemented as a transparent fair
% baseline rather than as an optimized TOPP package: it has the same path,
% dynamics, actuator limits, electrical-bus budget, and braking constraints as
% the learned PRG surrogates, but performs offline planning followed by online table
% lookup.

if nargin < 5 || isempty(opts), opts = struct(); end
Ts = getfield_def(cfg,'POLICY_DT', getfield_def(cfg,'TS',1e-3)); %#ok<NASGU>
alpha_min = getfield_def(cfg,'ALPHA_MIN',0.15);
Ns = max(21, round(getfield_def(opts,'Ns',101)));
Na = max(11, round(getfield_def(opts,'Na',41)));
alpha_grid = getfield_def(opts,'alpha_grid',[]);
if isempty(alpha_grid)
    alpha_grid = linspace(alpha_min,1,Na);
else
    alpha_grid = unique(min(1,max(alpha_min,double(alpha_grid(:).'))),'stable');
    Na = numel(alpha_grid);
end

W = getfield_def(opts,'W',struct());
W.time   = getfield_def(W,'time',1.0);
W.ramp   = getfield_def(W,'ramp',0.01);
W.slew   = getfield_def(W,'slew',0.01);
W.soft   = getfield_def(W,'soft',1e4);
allow_soft = logical(getfield_def(opts,'allow_soft_violation',false));

sgrid = linspace(0,traj.Tf,Ns);
J = inf(Ns,Na);
pre = zeros(Ns,Na);
J(1,1) = 0;  % start from the minimum feasible rate

bus = get_bus_from_cfg(cfg, getfield_def(limits,'P_grid_budget',getfield_def(limits,'P_total_max',inf)));
fric = getfield_def(cfg,'FRIC',struct('B',0,'Fc',0,'vel_eps',1e-3));

tStart = tic;
for i = 1:Ns-1
    ds = sgrid(i+1) - sgrid(i);
    for jp = 1:Na
        if ~isfinite(J(i,jp)), continue; end
        ap = alpha_grid(jp);
        for jn = 1:Na
            an = alpha_grid(jn);
            amid = 0.5*(ap+an);
            dt = ds/max(amid,1e-6);
            adot = (an-ap)/max(dt,1e-9);
            [ok, met] = check_transition(traj, sgrid(i), amid, adot, robot, limits, bus, fric);
            soft = met.soft_violation;
            if ok || allow_soft
                cost = W.time*dt + W.ramp*(met.ramp_norm^2)*dt + W.slew*(an-ap)^2 + W.soft*soft^2;
                if J(i,jp) + cost < J(i+1,jn)
                    J(i+1,jn) = J(i,jp) + cost;
                    pre(i+1,jn) = jp;
                end
            end
        end
    end
end

[bestCost,j] = min(J(end,:));
used_fallback = false;
if ~isfinite(bestCost)
    % Conservative fallback: constant minimum-rate profile.
    aprof = alpha_min*ones(Ns,1);
    used_fallback = true;
else
    aprof = zeros(Ns,1);
    for i = Ns:-1:1
        aprof(i) = alpha_grid(j);
        if i > 1
            jprev = pre(i,j);
            if jprev <= 0, jprev = 1; used_fallback = true; end
            j = jprev;
        end
    end
end

profile = struct();
profile.method = 'Bus-PTP-DP';
profile.s = sgrid(:);
profile.a = aprof(:);
profile.Ns = Ns;
profile.Na = Na;
profile.solve_time_s = toc(tStart);
profile.best_cost = bestCost;
profile.used_fallback = used_fallback;
profile.alpha_min = alpha_min;
end

function [ok, met] = check_transition(traj, s, a, adot, robot, limits, bus, fric)
[q, qs, qss] = traj.eval(s); q=row(q); qs=row(qs); qss=row(qss);
qd = a*qs;
qdd = a*a*qss + adot*qs;
try
    Mq = massMatrix(robot,q);
    c = velocityProduct(robot,q,qd);
    g = gravityTorque(robot,q);
    tau = (Mq*qdd(:)+c(:)+g(:)).';
catch
    tau = zeros(size(q));
end
if isstruct(fric)
    tau = tau + row(getfield_def(fric,'B',zeros(size(q)))).*qd + ...
          row(getfield_def(fric,'Fc',zeros(size(q)))).*tanh(qd./max(getfield_def(fric,'vel_eps',1e-3),1e-9));
end

qdv  = max(abs(qd)./max(row(getfield_def(limits,'qd_max',inf(size(q)))),1e-12)-1,0);
qddv = max(abs(qdd)./max(row(getfield_def(limits,'qdd_max',inf(size(q)))),1e-12)-1,0);
tauv = max(abs(tau)./max(row(getfield_def(limits,'tau_max',inf(size(q)))),1e-12)-1,0);
Paxis = tau.*qd;
Pav  = max(max(Paxis,0)./max(row(getfield_def(limits,'P_axis_max',inf(size(q)))),1e-12)-1,0);
try
    st = spinn3d_bus_state_init(bus);
    [flow,~] = spinn3d_bus_step_elec(tau, qd, bus, st, 1e-2);
catch
    [~,~,Pgrid,Pdump] = spinn3d_bus_power(tau, qd, bus);
    flow = struct('Pgrid',Pgrid,'Pdump',Pdump,'Vdc',getfield_def(bus,'V_nom',12));
end
Pbud = getfield_def(bus,'P_grid_budget',getfield_def(limits,'P_grid_budget',getfield_def(limits,'P_total_max',inf)));
Ppeak = getfield_def(bus,'P_grid_peak_max',getfield_def(limits,'P_grid_peak_max',inf));
Pbrk = getfield_def(bus,'P_brk_peak',inf);
rb = max(flow.Pgrid/max(Pbud,eps)-1,0);
rp = max(flow.Pgrid/max(Ppeak,eps)-1,0);
rd = max(flow.Pdump/max(Pbrk,eps)-1,0);
Psrc_short = getfield_def(flow,'Psource_shortfall',0);
Pspill = getfield_def(flow,'Pspill',0);
source_ok = Psrc_short <= 1e-9;
spill_ok  = Pspill <= 1e-9;
rs = max(Psrc_short/max(min_noninf(Pbud,Ppeak,1),eps),0);
rspill = max(Pspill/max(min_noninf(Pbrk,Pbud,1),eps),0);
kin = max([qdv(:); qddv(:); tauv(:); Pav(:); rb; rp; rd; rs; rspill]);
met.soft_violation = kin;
met.ramp_norm = 0;
met.Psource_shortfall = Psrc_short;
met.Pspill = Pspill;
ok = (kin <= 0) && source_ok && spill_ok;
end

function r = row(x), r = double(x(:)).'; end
function y = min_noninf(varargin)
vals = cellfun(@double,varargin); vals = vals(isfinite(vals) & vals>0);
if isempty(vals), y = 1; else, y = min(vals); end
end
function x = getfield_def(S,fn,def)
if isstruct(S) && isfield(S,fn) && ~isempty(S.(fn)), x = S.(fn); else, x = def; end
end
