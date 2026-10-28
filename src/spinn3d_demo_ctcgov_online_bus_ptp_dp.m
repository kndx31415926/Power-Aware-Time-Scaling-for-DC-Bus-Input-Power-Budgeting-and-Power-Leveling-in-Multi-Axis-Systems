function [out,fig] = spinn3d_demo_ctcgov_online_bus_ptp_dp(cfg)
%SPINN3D_DEMO_CTCGOV_ONLINE_BUS_PTP_DP Approximate nominal beam-plan + authoritative replay.
if nargin<1||isempty(cfg), PB=spinn3d_params_block(); cfg=spinn3d_params_to_cfg(PB,'demo_alpha'); end

limits=cfg.CAPS; robot=cfg.ROBOT; try,robot.DataFormat='row';end
q0=deg2rad_local(cfg.Q0_DEG); qf=unwrap_to_near(q0,deg2rad_local(cfg.QF_DEG));
[robot2,~]=spinn3d_inject_payload_last_body(robot,cfg.PAYLOAD.DEMO);
traj=make_traj_cart_bezier_dls(robot2,q0,qf,limits,cfg);
t0=tic; solptp=spinn3d_bus_ptp_dp_r2(robot2,traj,cfg,limits); solve_s=toc(t0);
sol=struct('plan',solptp.plan,'seed_kind','bus-ptp-bounded-beam-r97', ...
    'planner_status','nominal_hard_screened_approximate_plan','proxy_time_contract_pass',true, ...
    'ptp',solptp);

cfg.DIAGNOSTICS.allow_precomputed_solution=true;
cfg.WARMSTART_PRECOMPUTED_SOL=sol;

% method guard/cap so no warm-start-specific method setting leaks in.
cfg.METHODS.warmstart=cfg.METHODS.bus_ptp_dp;
cfg.METHOD_GUARDS.warmstart=cfg.METHOD_GUARDS.bus_ptp_dp;
out=spinn3d_demo_ctcgov_online_warmstart(cfg);
fig=[];
out.method='bus_ptp_dp'; out.planner_runtime_s=solve_s; out.ptp=solptp;
if isfield(out,'summary')&&isstruct(out.summary), out.summary.planner_runtime_s=solve_s; end
end
function q=deg2rad_local(d), q=pi/180*double(d(:)).'; end
function qf=unwrap_to_near(q0,qf), d=qf-q0; qf=q0+atan2(sin(d),cos(d)); end
