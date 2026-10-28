function [out, fig] = spinn3d_demo_ctcgov_online_nonn(cfg)

if nargin < 1 || isempty(cfg)
    blk = [];
    if exist('spinn3d_params_block','file')==2
        blk = spinn3d_params_block();
    elseif evalin('base',"exist(''spinn3d_params_block'',''var'')==1")
        blk = evalin('base','spinn3d_params_block');
    end

    cfg = spinn3d_params_to_cfg(blk, 'demo_alpha');
end
[cfg,~] = spinn3d_attach_paired_noguard_reference(cfg);

req(cfg, {'TS','DT','Q0_DEG','QF_DEG','P_TOTAL_MAX','ALPHA_MIN','ALPHA_ITMAX'});

limits = pick_struct(cfg, {'CAPS','LIMITS'});
gains  = pick_struct(cfg, {'CTRL','GAINS'});
req(limits, {'tau_max','qd_max','qdd_max','P_axis_max'});
req(gains,  {'Kp','Kd'});

robot = cfg.ROBOT; try, robot.DataFormat='row'; end
nJ    = numel(homeConfiguration(robot));
q0    = deg2rad_local(row(cfg.Q0_DEG));
qf    = unwrap_to_near(q0, deg2rad_local(row(cfg.QF_DEG)));

eeName = char(cfg.EE_NAME);

pay_cfg = cfg.PAYLOAD.DEMO;

if isstruct(pay_cfg)

end
[robot, ~] = inject_payload_last_body(robot, pay_cfg);

traj  = make_traj_cart_bezier_dls(robot, q0, qf, limits, cfg);

areg = struct('BETA',[], 'A_DOT_UP',[], 'A_DOT_DN',[]);
if isfield(cfg,'ALPHA_REG')
    fns = fieldnames(areg);
    for i=1:numel(fns)
        if isfield(cfg.ALPHA_REG,fns{i})
            areg.(fns{i}) = cfg.ALPHA_REG.(fns{i});
        end
    end
end

bus = get_bus_from_cfg(cfg, limits.P_total_max);  
plant=spinn3d_plant_controller_models(cfg,robot,bus,q0);
power_init=plant.power_init;

opts = struct();
opts.method_id='nonn';
opts.executionAudit=cfg.EXECUTION_AUDIT;
opts.Ts      = cfg.TS;

method_cfg = cfg.METHODS.nonn;
opts.policy = cfg.POLICY;
opts.fric    = cfg.FRIC;
opts.bus     = bus;
opts.alphaReg= areg;     
opts.nnAlpha = struct( ...
    'enable',      logical(method_cfg.alpha_controller_enable), ...
    'mode',        char(method_cfg.alpha_controller_mode), ...
    'feasibility_use_alpha_dot', ...
                  method_cfg.feasibility_use_alpha_dot, ...
    'alpha_floor', cfg.ALPHA_MIN, ...
    'alpha_max',   double(method_cfg.command_alpha_max), ...
    'itmax',       cfg.ALPHA_ITMAX ...
    );
opts.nnAlpha.gate=cfg.ALPHA_GATE;
teacher_caps=cfg.CAPS; teacher_caps.execution_audit=cfg.EXECUTION_AUDIT;
opts.alphaStateTeacher=spinn3d_alpha_state_teacher_config( ...
    cfg,teacher_caps,cfg.FRIC,bus,gains.Kp,gains.Kd,cfg.P_TOTAL_MAX);
opts.alphaStateTeacher.power_enabled=true;

% NONN is an online feasible governor.  The final actual-step guard must

% the reference step unsafe; otherwise the reference keeps advancing while the
% torque clamp holds the plant back, which creates looped/chaotic EE traces.
% A buffer-free BUS profile still needs the
% actual-step torque, kinematic and source-power safety checks.

opts.actualGuard=cfg.METHOD_GUARDS.nonn;
opts.initial_bus_measurement=struct('bus_state',plant.bus_state, ...
    'Pgrid_static_W',plant.power_init.Pgrid_static_W, ...
    'Pgrid_avg_initial_W',plant.power_init.Pgrid_avg0_W);
ctl = spinn3d_controller_ctcgov(robot, traj, gains, limits, opts);

fair_plan = spinn3d_fair_time_contract(cfg,NaN,traj.Tf);
[Tcap, horizon_info] = spinn3d_fair_execution_horizon(cfg,traj.Tf,fair_plan.deadline_s);
Nt   = max(1, ceil(Tcap/cfg.TS));
Tcap = Nt * cfg.TS;

B  = row(cfg.FRIC.B);   Fc = row(cfg.FRIC.Fc);   ve = cfg.FRIC.vel_eps;
tau_max   = vec_lim(limits,'tau_max',    nJ, inf);
P_axismax = vec_lim(limits,'P_axis_max', nJ, inf);

t=0; s=0; [q_ref0, dq_ref0, ddq_ref0] = traj.eval(0); %#ok<ASGLU>
q = row(q_ref0); dq = zeros(1,nJ);
noise_state = spinn3d_noise_init(cfg,nJ);

log.t=zeros(Nt,1); log.s=zeros(Nt,1);
log.controller_wall_s=nan(Nt,1); log.policy_tick=false(Nt,1);
log.alpha_des=nan(Nt,1); log.alpha_governor=nan(Nt,1);
log.controller_alpha_guard_applied=false(Nt,1); log.actual_guard_applied=false(Nt,1);
log.plant_guard_wall_s=nan(Nt,1); log.step_wall_s=nan(Nt,1);
log.network_forward_s=nan(Nt,1); log.actuator_saturation=false(Nt,1);
log.motion_projection=false(Nt,1); log.actual_step_pass=false(Nt,1);
log.q=zeros(Nt,nJ); log.dq=zeros(Nt,nJ); log.dq_pwr=zeros(Nt,nJ);
% Replay inputs are sampled before the interval commit. dq_ref and
% ddq_ref are trajectory derivatives before alpha/time scaling.
log.q_ref=nan(Nt,nJ); log.dq_ref=nan(Nt,nJ); log.ddq_ref=nan(Nt,nJ);
log.q_m=nan(Nt,nJ); log.dq_m=nan(Nt,nJ);
log.q_pre=nan(Nt,nJ); log.dq_pre=nan(Nt,nJ);
log.t_pre=nan(Nt,1); log.s_pre=nan(Nt,1); log.alpha_prev_step=nan(Nt,1);
log.tau=zeros(Nt,nJ); log.ddq=zeros(Nt,nJ); log.alpha=zeros(Nt,1);
log.alpha_cmd=zeros(Nt,1); log.execution_progress_ratio=ones(Nt,1);
log.Pi_raw=zeros(Nt,nJ); log.Ptot_pos=zeros(Nt,1);  
log.Pgrid=zeros(Nt,1); log.Pdump=zeros(Nt,1);       % BUS:
log.Pshare=zeros(Nt,1); log.Pbuf_dis=zeros(Nt,1); log.Pbuf_chg=zeros(Nt,1); log.Pspill=zeros(Nt,1);
log.Pshare_axis=zeros(Nt,1); log.Pshare_potential=zeros(Nt,1);
log.Pregen_available=zeros(Nt,1); log.Pregen_curtailed=zeros(Nt,1);
log.Pregen_curtailed_total=zeros(Nt,1); log.Pregen_cap_curtailed=zeros(Nt,1);
log.Pload_balance_residual=zeros(Nt,1); log.Pregen_route_balance_residual=zeros(Nt,1);
log.energy_balance_residual_W=zeros(Nt,1);
log.Preuse_buf=zeros(Nt,1); log.Preuse_total=zeros(Nt,1);
log.Pregen_bus=zeros(Nt,1); log.Preuse_task_buf=zeros(Nt,1); log.Ploss_total=zeros(Nt,1);
log.Eregen_task=zeros(Nt,1);
log.Ebuf=zeros(Nt,1); log.Ereuse=zeros(Nt,1); log.Tbrk=zeros(Nt,1);
log.Pgrid_avg=zeros(Nt,1);
log.Pgrid_avg_pre_t0_W=repmat(power_init.Pgrid_avg0_W,Nt,1);
log.q_track_err=zeros(Nt,1); log.actual_clip=zeros(Nt,1); log.actual_tau_ratio=zeros(Nt,1);
log.actual_qd_ratio=zeros(Nt,1); log.actual_qdd_ratio=zeros(Nt,1);
log.actual_pgrid_ratio=zeros(Nt,1); log.actual_pavg_ratio=zeros(Nt,1);
log.actual_floor_infeasible=false(Nt,1); log.actual_restart_bisect=false(Nt,1);
log.ee=nan(Nt,3);

ee_start= ee_pos(robot,q0,eeName);
ee_goal = ee_pos(robot,qf,eeName);

t_reach=NaN;
stop_stable_required=double(cfg.STOP.STABLE_TIME);

stop_stable_elapsed=0;

for k=1:Nt
    step_clock=tic;
    [q_r, dq_r, ddq_r] = traj.eval(s); q_r=row(q_r); dq_r=row(dq_r); ddq_r=row(ddq_r);

    [q_m,dq_m,tau_noise,noise_state] = ...
        spinn3d_noise_step(q,dq,cfg.TS,noise_state);
    controller_clock=tic;
    [tau_cmd, info] = ctl.step(t, q_m, dq_m, q_r, dq_r, ddq_r, robot, s);
    log.controller_wall_s(k)=getfield_def(info,'controller_wall_s',toc(controller_clock));
    log.policy_tick(k)=logical(getfield_def(info,'policy_tick',false));
    log.network_forward_s(k)=getfield_def(info,'network_forward_s',NaN);
    log.alpha_des(k)=info.a0; log.alpha_governor(k)=info.a_filt;
    log.controller_alpha_guard_applied(k)=abs(info.a_max-info.a_filt)>1e-10;
    tau_cmd = row(tau_cmd);

    dq_old = dq;
    plant_guard_clock=tic;
    [tau_m,ddq,dq_mid,step_audit] = spinn3d_plant_step( ...
        plant,q,dq,q_m,dq_m,tau_cmd+tau_noise,tau_max,P_axismax,cfg.TS, ...
        limits,cfg.WORKSPACE,cfg.EXECUTION_AUDIT);
    log.plant_guard_wall_s(k)=toc(plant_guard_clock);
    [~,log] = spinn3d_actual_guard_telemetry(info,log,k,step_audit);
    log.q_ref(k,:)=q_r; log.dq_ref(k,:)=dq_r; log.ddq_ref(k,:)=ddq_r;
    log.q_m(k,:)=q_m; log.dq_m(k,:)=dq_m;
    log.q_pre(k,:)=q; log.dq_pre(k,:)=dq;
    log.t_pre(k)=t; log.s_pre(k)=s;
    if isfield(info,'a_prev_step'), log.alpha_prev_step(k)=info.a_prev_step; end
    log.actual_step_pass(k)=logical(step_audit.final_motion.pass);
    if step_audit.final_motion.pass
        [a_exec,progress_ratio] = ...
            spinn3d_execution_progress_alpha(info.a_max,step_audit, ...
                method_cfg.command_alpha_max);
    else
        a_exec=0; progress_ratio=0;
    end
    dq  = dq_old + cfg.TS*ddq;
    q   = q + cfg.TS*dq_mid;
    sample_end_t=t+cfg.TS;

    % Reference progress must match the command retained by the final
    % post-noise projection, not the pre-projection controller request.
    s = min(traj.Tf, s + cfg.TS*a_exec);

    dq_pwr = dq_mid;
    Pi_raw = tau_m .* dq_pwr;
    [measured,plant]=spinn3d_plant_observe(plant,tau_m,dq_pwr,cfg.TS);
    [obs, bus_state_now] = ctl.observe(tau_m,dq_pwr,a_exec,step_audit,measured);
    flow = obs.flow;

    log.t(k)=sample_end_t; log.s(k)=s; log.q(k,:)=q; log.dq(k,:)=dq;
    log.tau(k,:)=tau_m; log.ddq(k,:)=ddq; log.dq_pwr(k,:)=dq_pwr;
    log.alpha(k)=a_exec; log.alpha_cmd(k)=info.a_max;
    log.execution_progress_ratio(k)=progress_ratio; log.Pi_raw(k,:)=Pi_raw;
    log.Ptot_pos(k)=flow.Ppos; log.Pgrid(k)=flow.Pgrid; log.Pdump(k)=flow.Pdump;
    log.Pshare(k)=flow.Pshare; log.Pbuf_dis(k)=flow.Pbuf_dis; log.Pbuf_chg(k)=flow.Pbuf_chg;
    log.Pshare_axis(k)=flow.Pshare_axis; log.Pshare_potential(k)=flow.Pshare_potential;
    log.Pregen_available(k)=flow.Pregen_available; log.Pregen_curtailed(k)=flow.Pregen_curtailed;
    log.Pregen_curtailed_total(k)=flow.Pregen_curtailed_total; log.Pregen_cap_curtailed(k)=flow.Pregen_cap_curtailed;
    log.Pload_balance_residual(k)=flow.Pload_balance_residual;
    log.Pregen_route_balance_residual(k)=flow.Pregen_route_balance_residual;
    log.energy_balance_residual_W(k)=max(abs([flow.Pload_balance_residual flow.Pregen_route_balance_residual flow.Penergy_balance_residual]));
    log.Pspill(k)=flow.Pspill; log.Preuse_buf(k)=flow.Preuse_buf; log.Preuse_total(k)=flow.Preuse_total;
    log.Pregen_bus(k)=flow.Pregen_bus; log.Preuse_task_buf(k)=flow.Preuse_task_buf; log.Ploss_total(k)=flow.Ploss_total;
    log.Ebuf(k)=bus_state_now.Ebuf; log.Ereuse(k)=bus_state_now.Ereuse; log.Tbrk(k)=bus_state_now.Tbrk;
    log=spinn3d_dynamic_bus_log(log,k,flow,bus_state_now,power_init,bus);
    log.Eregen_task(k)=bus_state_now.Eregen_task;
    log.Pgrid_avg(k)=obs.Pgrid_avg_lp;
    log.q_track_err(k)=norm(q - q_r);
    if isfield(info,'guard') && isstruct(info.guard) && isfield(info.guard,'actual') && isstruct(info.guard.actual)
        act = info.guard.actual;
        log.actual_clip(k) = getfield_def(act,'a_clip',NaN);
        log.actual_tau_ratio(k) = getfield_def(act,'tau_ratio',NaN);
        log.actual_qd_ratio(k) = getfield_def(act,'qd_ratio',NaN);
        log.actual_qdd_ratio(k) = getfield_def(act,'qdd_ratio',NaN);
        log.actual_pgrid_ratio(k) = getfield_def(act,'pgrid_ratio',NaN);
        log.actual_pavg_ratio(k) = getfield_def(act,'pavg_ratio',NaN);
        log.actual_floor_infeasible(k) = logical(getfield_def(act,'floor_infeasible',false));
        log.actual_restart_bisect(k) = logical(getfield_def(act,'restart_bisect',false));
    else
        log.actual_clip(k)=NaN; log.actual_tau_ratio(k)=NaN; log.actual_qd_ratio(k)=NaN; log.actual_qdd_ratio(k)=NaN;
        log.actual_pgrid_ratio(k)=NaN; log.actual_pavg_ratio(k)=NaN;
        log.actual_floor_infeasible(k)=false; log.actual_restart_bisect(k)=false;
    end
    Tee = getTransform(plant.robot, q, eeName); log.ee(k,:) = tform2trvec_safe(Tee);
    log.step_wall_s(k)=toc(step_clock);
    if ~step_audit.final_motion.pass
        termination_reason='execution_hard_constraint_violation';
        log=truncate_log(log,k);
        break;
    end

    ee_err = norm(log.ee(k,:) - ee_goal);
    stop_inside=(s>=traj.Tf)&&(ee_err<=cfg.STOP.NEAR_TOL)&&(norm(dq)<=cfg.STOP.VEL_TOL);
    if stop_inside, stop_stable_elapsed=stop_stable_elapsed+cfg.TS;
    else, stop_stable_elapsed=0; end
    if stop_inside && stop_stable_elapsed+1e-12>=stop_stable_required
        t_reach = sample_end_t;
        log = truncate_log(log, k);
        break;
    end

    t = t + cfg.TS;
end

summary = summary_from_log_local(log, t_reach, NaN, 'nonn');
dc_summary=spinn3d_dynamic_bus_summary(log,bus);
dc_names=fieldnames(dc_summary);
for dc_i=1:numel(dc_names),summary.(dc_names{dc_i})=dc_summary.(dc_names{dc_i});end
summary.Pgrid_avg_initial_W=power_init.Pgrid_avg0_W;
goal_reached = isfinite(t_reach);
fair = spinn3d_fair_time_contract(cfg,t_reach,traj.Tf);
execution_audit=spinn3d_execution_hard_audit( ...
    log,plant.robot,limits,plant.bus,cfg.P_TOTAL_MAX,cfg.EXECUTION_AUDIT, ...
    cfg.WORKSPACE);
accepted = goal_reached && fair.pass && execution_audit.pass;
if ~execution_audit.pass
    termination_reason = 'execution_hard_constraint_violation';
elseif ~goal_reached
    termination_reason = 'time_cap_not_reached';
elseif fair.reference_available && ~fair.pass
    termination_reason = 'arrival_deadline_exceeded';
else
    termination_reason = 'reached';
end
reference = struct('kind','cartesian-bezier-dls','Tf',double(traj.Tf));
out = struct('log',log,'limits',limits,'gains',gains,'cfg',cfg, ...
    't_reach',t_reach,'reached',goal_reached,'goal_reached',goal_reached, ...
    'accepted',accepted,'time_contract_pass',logical(fair.pass), ...
    'time_contract_available',logical(fair.reference_available), ...
    'termination_reason',termination_reason,'traj_Tf',double(traj.Tf), ...
    'reference',reference,'fairness',fair,'diagnostic_horizon',horizon_info, ...
    'summary',summary,'execution_audit',execution_audit);
out.actual_guard_telemetry_schema='actual_guard_liveness_v3';

out.perf=spinn3d_execution_perf(log,cfg.TS);
out.plant_model=plant.meta;
out.method_id='nonn';

if get_disable_plot_local(cfg)
    fig = [];
    return;
end

fig = figure('Name','CTC+Gov (online nonn, BUS power)','Visible','on');
tlo = tiledlayout(fig,3,2,'Padding','compact','TileSpacing','compact');

ax3 = nexttile(tlo,[2 1]); hold(ax3,'on'); grid(ax3,'on'); axis(ax3,'equal'); view(ax3,135,20);
[X0,Y0,Z0] = chain_skeleton_xyz(robot, q0, eeName);
if ~isempty(X0), plot3(ax3, X0, Y0, Z0, '-o','LineWidth',1.0,'DisplayName','Initial'); end
try
    show(robot, log.q(end,:), 'Parent', ax3, 'PreservePlot', true, 'Frames','off','Visuals','on');
catch
end
[Xf,Yf,Zf] = chain_skeleton_xyz(robot, log.q(end,:), eeName);
if ~isempty(Xf), plot3(ax3, Xf, Yf, Zf, 'm-o','MarkerFaceColor','m','LineWidth',1.0,'DisplayName','Final'); end
plot3(ax3, log.ee(:,1), log.ee(:,2), log.ee(:,3), 'c-','LineWidth',1.5,'DisplayName','EE traj');
plot3(ax3, ee_start(1),ee_start(2),ee_start(3),'go','MarkerFaceColor','g','DisplayName','Start');
plot3(ax3, ee_goal(1), ee_goal(2), ee_goal(3),'co','MarkerFaceColor','c','DisplayName','Target');
plot3(ax3, log.ee(end,1),log.ee(end,2),log.ee(end,3),'ro','MarkerFaceColor','r','DisplayName','Final EE');
xlabel(ax3,'X [m]'); ylabel(ax3,'Y [m]'); zlabel(ax3,'Z [m]');
title(ax3,'EE Trajectory + Robot Skeleton (Initial/Final; nonn)'); legend(ax3,'Location','best');

axU = nexttile(tlo); hold(axU,'on'); grid(axU,'on');
for j=1:nJ, plot(axU, log.t, log.Pi_raw(:,j), 'DisplayName', sprintf('J%d',j)); end
xlabel(axU,'t [s]'); ylabel(axU,'Joint Power P_j [W]');
title(axU,'Per-Joint Power'); legend(axU,'show','Location','best'); xlim(axU,[log.t(1),log.t(end)]);

axD = nexttile(tlo); hold(axD,'on'); grid(axD,'on');
plot(axD, log.t, log.Pgrid, 'DisplayName','P_{grid}');
if isfield(log,'Pgrid_avg') && ~isempty(log.Pgrid_avg) && any(isfinite(log.Pgrid_avg))
    plot(axD, log.t, log.Pgrid_avg, 'LineWidth',1.6, 'DisplayName','P_{grid} filtered bus');
else

    win = max(1,round(double(bus.P_avg_window_s)/cfg.TS));
    plot(axD, log.t, movmean(log.Pgrid,win), 'LineWidth',1.6, 'DisplayName','P_{grid} filtered');
end
ymax=max(log.Pgrid)*1.2; if ~isfinite(ymax)||ymax<=0, ymax=1; end
ylim(axD,[0,ymax]); xlim(axD,[log.t(1), log.t(end)]);
if limits.P_total_max <= ymax, yline(axD, limits.P_total_max, '--r','P_{grid,avg,bud}'); end
xlabel(axD,'t [s]'); ylabel(axD,'Power [W]'); title(axD,'Source Power'); legend(axD,'show','Location','best');

spinn3d_plot_dynamic_bus(tlo,log,bus);
drawnow;

end

%% ==================== helpers ====================

function tf = get_disable_plot_local(cfg)

tf = ~logical(cfg.PLOT.enable);
if isfield(cfg,'DISABLE_PLOT') && ~isempty(cfg.DISABLE_PLOT)
    tf = logical(cfg.DISABLE_PLOT);
    return;
end
end

function summary = summary_from_log_local(log, t_reach, Tref, seed_kind)
summary = struct('t_reach',t_reach,'Tref',Tref,'peakPgrid',NaN,'peakPdump',NaN, ...
                 'Egrid',0,'Edump',0,'Eshare',0,'Ereuse_buf',0,'Ereuse_total',0, ...
                 'Eshare_axis',0,'Eoverlap',0,'Eregen_available',0,'Eregen_bus',0, ...
                 'Eregen_curtailed',0,'Eregen_curtailed_total',0, ...
                 'energy_balance_residual_max_W',NaN,'energy_balance_residual_rms_W',NaN, ...
                 'Espill',0,'Ebuf_min',0,'seed_kind',seed_kind);
if isempty(log) || ~isfield(log,'t') || isempty(log.t)
    return;
end
t = log.t(:);
if isfield(log,'Pgrid') && ~isempty(log.Pgrid), summary.peakPgrid = max(log.Pgrid(:)); summary.Egrid = spinn3d_interval_integral(t, log.Pgrid(:)); end
if isfield(log,'Pdump') && ~isempty(log.Pdump), summary.peakPdump = max(log.Pdump(:)); summary.Edump = spinn3d_interval_integral(t, log.Pdump(:)); end
if isfield(log,'Pshare') && ~isempty(log.Pshare), summary.Eshare = spinn3d_interval_integral(t, log.Pshare(:)); end
if isfield(log,'Pshare_axis') && ~isempty(log.Pshare_axis), summary.Eshare_axis = spinn3d_interval_integral(t, log.Pshare_axis(:)); else, summary.Eshare_axis=summary.Eshare; end
if isfield(log,'Pshare_potential') && ~isempty(log.Pshare_potential), summary.Eoverlap = spinn3d_interval_integral(t, log.Pshare_potential(:)); end
if isfield(log,'Pregen_available') && ~isempty(log.Pregen_available), summary.Eregen_available = spinn3d_interval_integral(t, log.Pregen_available(:)); end
if isfield(log,'Pregen_bus') && ~isempty(log.Pregen_bus), summary.Eregen_bus = spinn3d_interval_integral(t, log.Pregen_bus(:)); end
if isfield(log,'Pregen_curtailed') && ~isempty(log.Pregen_curtailed), summary.Eregen_curtailed = spinn3d_interval_integral(t, log.Pregen_curtailed(:)); end
if isfield(log,'Pregen_curtailed_total') && ~isempty(log.Pregen_curtailed_total), summary.Eregen_curtailed_total = spinn3d_interval_integral(t, log.Pregen_curtailed_total(:)); end
if isfield(log,'energy_balance_residual_W') && ~isempty(log.energy_balance_residual_W), summary.energy_balance_residual_max_W=max(abs(log.energy_balance_residual_W(:))); summary.energy_balance_residual_rms_W=sqrt(mean(log.energy_balance_residual_W(:).^2)); end
if isfield(log,'Preuse_buf') && ~isempty(log.Preuse_buf), summary.Ereuse_buf = spinn3d_interval_integral(t, log.Preuse_buf(:)); end
if isfield(log,'Preuse_total') && ~isempty(log.Preuse_total)
    summary.Ereuse_total = spinn3d_interval_integral(t, log.Preuse_total(:));
else
    summary.Ereuse_total = summary.Eshare;
end
if isfield(log,'Pspill') && ~isempty(log.Pspill), summary.Espill = spinn3d_interval_integral(t, log.Pspill(:)); end
if isfield(log,'Ebuf') && ~isempty(log.Ebuf), summary.Ebuf_min = min(log.Ebuf(:)); end
end

function req(S, names), if ischar(names), names={names}; end, for i=1:numel(names),  end, end
function def = getfield_def(S, field, def), if isfield(S,field)&&~isempty(S.(field)), def=S.(field); end, end
function S = pick_struct(cfg, keys), S=[]; for i=1:numel(keys), k=keys{i}; if isfield(cfg,k)&&~isempty(cfg.(k))&&isstruct(cfg.(k)), S=cfg.(k); return; end, end, error('cfg 缺少结构体：%s', strjoin(keys,'/')); end
function v = row(v), v=v(:)'; end
function y = deg2rad_local(x), y=(pi/180).*x; end
function p = tform2trvec_safe(T), try, p=tform2trvec(T); catch, p=T(1:3,4).'; end, end
function log = truncate_log(log,k), fn=fieldnames(log); for i=1:numel(fn), v=log.(fn{i}); if size(v,1)>=k, log.(fn{i})=v(1:k,:); end, end, end
function p=ee_pos(robot,qrow,eeName), T=getTransform(robot,qrow,eeName); p=tform2trvec_safe(T); end

function [X,Y,Z] = chain_skeleton_xyz(robot, qrow, eeName)
    try, robot.DataFormat='row'; end
    X=[];Y=[];Z=[];
    try
        names = {};
        b = getBody(robot, eeName);
        while ~strcmp(b.Name, robot.BaseName)
            names{end+1} = b.Name; %#ok<AGROW>
            b = getBody(robot, b.Parent);
        end
        names = fliplr(names);
    catch
        names = robot.BodyNames;
    end
    if isempty(names), return; end
    for i=1:numel(names)
        b = getBody(robot, names{i});
        if strcmp(b.Joint.Type,'fixed'), continue; end
        Tb = getTransform(robot, qrow, names{i});
        p  = tform2trvec_safe(Tb);
        X(end+1)=p(1); Y(end+1)=p(2); Z(end+1)=p(3); %#ok<AGROW>
    end
    Tee = getTransform(robot, qrow, eeName);
    pee = tform2trvec_safe(Tee);
    X(end+1)=pee(1); Y(end+1)=pee(2); Z(end+1)=pee(3);
end

function v = vec_lim(S, field, n, def)
    if ~isfield(S,field) || isempty(S.(field)), v = def*ones(1,n); return; end
    v = S.(field); if isscalar(v), v = repmat(v,1,n); end
    v(~isfinite(v)) = def; v = v(:)'; 
end
function tau = clamp_tau(tau, tau_max)
    for j=1:numel(tau)
        if isfinite(tau_max(j))
            tau(j) = max(-tau_max(j), min(tau_max(j), tau(j)));
        end
    end
end

function tau = clamp_power_axis_total_bus_local(tau, vel, P_axis_max, bus)
    % Deprecated local duplicate.  Keep only as a compatibility shim; the
    % global helper uses per-axis power-ratio scaling and the instantaneous
    % raw guard bus.P_grid_peak_max rather than the filtered budget.
    tau = clamp_power_axis_total_bus(tau, vel, P_axis_max, bus);
end

function bus = get_bus_from_cfg_local(cfg, P_total_max)
    bus = struct('eta_share',1.0,'P_brk_peak',inf,'P_total_max',P_total_max,'P_grid_peak_max',inf);
    if isfield(cfg,'BUS') && isstruct(cfg.BUS)
        if isfield(cfg.BUS,'eta_share') && ~isempty(cfg.BUS.eta_share), bus.eta_share = cfg.BUS.eta_share; end
        if isfield(cfg.BUS,'P_brk_peak') && ~isempty(cfg.BUS.P_brk_peak), bus.P_brk_peak = cfg.BUS.P_brk_peak; end
        if isfield(cfg.BUS,'P_grid_peak_max') && ~isempty(cfg.BUS.P_grid_peak_max), bus.P_grid_peak_max = cfg.BUS.P_grid_peak_max; end
    end
end
function qn = unwrap_to_near(qprev, qcur)
    d  = row(qcur) - row(qprev);
    qn = row(qprev) + atan2(sin(d), cos(d));
end
function [robot_out, pay_vec] = inject_payload_last_body(robot_in, pay)
if exist('spinn3d_inject_payload_last_body','file') == 2
    [robot_out,pay_vec] = spinn3d_inject_payload_last_body(robot_in,pay);
    return;
end

%

%
% about:

% mode:

    robot_out = robot_in;
    pay_vec   = zeros(1,10);
    if isempty(pay), return; end

    if isnumeric(pay)
        v = double(pay(:)).';
        if numel(v) < 10, v = [v, zeros(1,10-numel(v))]; end
        m  = v(1); com = v(2:4); I6 = v(5:10);
        about = 'com';    
        mode  = 'replace';
    elseif isstruct(pay)
        m   = getfield_def(pay,'mass',0);
        com = getfield_def(pay,'com',[0 0 0]);
        I6  = getfield_def(pay,'inertia',[0 0 0 0 0 0]);
        about = lower(getfield_def(pay,'about','com'));
        mode  = lower(getfield_def(pay,'mode','replace'));
    else
        return;
    end
    pay_vec = [double(m), double(com(:)).', double(I6(:)).'];

    try
        b  = robot_out.Bodies{end};
        m0 = double(b.Mass);
        c0 = double(b.CenterOfMass(:)).';
        I0 = I6ToMat(double(b.Inertia(:)).');

        Ic = I6ToMat(double(I6(:)).');
        if strcmpi(about,'com')
            r  = double(com(:)); r = r(:);
            I1 = Ic + double(m) * ((r.'*r)*eye(3) - (r*r.'));
        else
            I1 = Ic;
        end

        switch mode
            case 'add'
                m_new  = m0 + m;
                if m_new <= 0
                    c_new = [0 0 0];
                else
                    c_new = (m0*c0 + m*com(:).') / m_new;
                end
                I_new = I0 + I1;
            otherwise % 'replace'
                m_new  = m;
                c_new  = com(:).';
                I_new  = I1;
        end

        b.Mass         = m_new;
        b.CenterOfMass = c_new;
        b.Inertia      = ImatTo6(I_new).';
    catch

    end
end

function I = I6ToMat(v6)

    v6 = double(v6(:)).';
    Ixx=v6(1); Iyy=v6(2); Izz=v6(3);
    Iyz=v6(4); Ixz=v6(5); Ixy=v6(6);
    I = [ Ixx Ixy Ixz;
          Ixy Iyy Iyz;
          Ixz Iyz Izz ];
end

function v6 = ImatTo6(I)

    v6 = [ I(1,1), I(2,2), I(3,3), I(2,3), I(1,3), I(1,2) ];
end
