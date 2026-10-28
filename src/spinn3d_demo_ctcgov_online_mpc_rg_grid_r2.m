function [out, fig] = spinn3d_demo_ctcgov_online_mpc_rg_grid_r2(cfg)

if nargin < 1 || isempty(cfg)
    blk = [];
    if exist('spinn3d_params_block','file')==2
        blk = spinn3d_params_block();
    elseif evalin('base',"exist(''spinn3d_params_block'',''var'')==1")
        blk = evalin('base','spinn3d_params_block');
    end

    cfg = spinn3d_params_to_cfg(blk, 'demo_alpha');
end

req(cfg, {'TS','DT','Q0_DEG','QF_DEG','P_TOTAL_MAX','ALPHA_MIN','ALPHA_ITMAX'});

method_cfg = cfg.METHODS.mpc_rg_grid;

limits = pick_struct(cfg, {'CAPS','LIMITS'});
gains  = pick_struct(cfg, {'CTRL','GAINS'});
req(limits, {'tau_max','qd_max','qdd_max','P_axis_max'});
req(gains,  {'Kp','Kd'});

Pbud_plot = limits.P_total_max;  

% Disable only the experimental filtered-power budget.  The ST3215
% torque-speed envelope, per-axis physical maximum and 12-V/5-A source cap
% are hardware limits and remain active in every comparison method.
if ~logical(method_cfg.filtered_power_guard_enable)
    limits.P_total_max = method_cfg.software_power_limit_W;
end

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

RELAX_ALPHA_REG = logical(method_cfg.relax_alpha_reg_enable);
if RELAX_ALPHA_REG
    areg.BETA     = method_cfg.relaxed_alpha_beta;
    areg.A_DOT_UP = method_cfg.relaxed_alpha_dot_up;
    areg.A_DOT_DN = method_cfg.relaxed_alpha_dot_down;
end

% Two BUS structs are used here:
%   bus      : controller BUS with the filtered software budget disabled.
%   bus_tel  : identical physical source model used for telemetry/hard clamp.
bus_tel = get_bus_from_cfg(cfg, Pbud_plot);
bus_tel = spinn3d_bus_defaults(bus_tel);
% Initialize telemetry at the physical q0 static hold.  This remains
% stateless when cfg.BUS.enable_state=false; it does not turn on a buffer.
plant=spinn3d_plant_controller_models(cfg,robot,bus_tel,q0);
power_init=plant.power_init;
if logical(getfield_def(bus_tel,'enable_state',false))
    st_bus_tel = power_init.bus_state0;
else
    st_bus_tel = spinn3d_bus_state_init(bus_tel);
end
Pgrid_avg_tel = power_init.Pgrid_avg0_W;

bus = bus_tel;
% Keep Pbud_plot only as the red reference line.  Do not make the physical
% supply infinite: "MPC-RG-grid" means no learned/filtered governor, not an
% impossible motor or PSU.
if ~logical(method_cfg.filtered_power_guard_enable)
    bus.P_total_max = method_cfg.software_power_limit_W;
    bus.P_avg_hard_enable = logical(method_cfg.average_power_guard_enable);
end
% The nominal-reference search uses the same physical DC state as the plant.
% Its soft objective cannot select a different electrical state model.

opts = struct();
opts.method_id='mpc_rg_grid';
opts.executionAudit=cfg.EXECUTION_AUDIT;
opts.Ts      = cfg.TS;

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
    'itmax',       cfg.ALPHA_ITMAX, ...
    'mpc',         cfg.MPC_R2 ...
    );
opts.nnAlpha.gate=cfg.ALPHA_GATE;

opts.actualGuard = cfg.METHOD_GUARDS.mpc_rg_grid;
opts.initial_bus_measurement=struct('bus_state',plant.bus_state, ...
    'Pgrid_static_W',plant.power_init.Pgrid_static_W, ...
    'Pgrid_avg_initial_W',plant.power_init.Pgrid_avg0_W);
ctl = spinn3d_controller_ctcgov(robot, traj, gains, limits, opts);

% Its acceptance role remains the paired timing baseline.
fair_plan = spinn3d_fair_time_contract(cfg,NaN,traj.Tf);
[Tcap, horizon_info] = spinn3d_fair_execution_horizon( ...
    cfg,traj.Tf,fair_plan.deadline_s);
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
log.mpc_failed=nan(Nt,1); log.mpc_completed_horizon_steps=nan(Nt,1);
log.mpc_n_eval=nan(Nt,1); log.mpc_n_hard_reject=nan(Nt,1);
log.mpc_solve_wall_s=nan(Nt,1);
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
% Actual-guard liveness telemetry is observational only.  It records the
% governed command before the overlay, the selected recovery command and the
% reason/gain metadata already returned by the controller.  None of these
% channels feeds back into the controller or plant.
log.alpha_policy_target=nan(Nt,1); log.alpha_pre_actual=nan(Nt,1);
log.actual_guard_applied=false(Nt,1);
log.actual_guard_safe_hold=false(Nt,1);
log.actual_guard_floor_infeasible=false(Nt,1);
log.actual_guard_hold_ok=true(Nt,1);
log.actual_guard_below_requested_floor=false(Nt,1);
log.actual_guard_position_gain_scale=nan(Nt,1);
log.actual_guard_damping_gain_scale=nan(Nt,1);
log.actual_guard_q_error_norm=nan(Nt,1);
log.actual_guard_dq_error_norm=nan(Nt,1);
log.actual_guard_reference_q_error_norm=nan(Nt,1);
log.actual_guard_positive_position_gain_found=false(Nt,1);
log.actual_guard_positive_damping_gain_found=false(Nt,1);
log.actual_guard_zero_position_gain_selected=false(Nt,1);
log.actual_guard_zero_damping_gain_selected=false(Nt,1);
log.actual_guard_position_grid_evaluations=zeros(Nt,1);
log.actual_guard_position_refine_evaluations=zeros(Nt,1);
log.actual_guard_damping_grid_evaluations=zeros(Nt,1);
log.actual_guard_damping_refine_evaluations=zeros(Nt,1);
log.actual_guard_total_evaluations=zeros(Nt,1);
log.actual_guard_position_bracket_initial=nan(Nt,2);
log.actual_guard_position_bracket_final=nan(Nt,2);
log.actual_guard_damping_bracket_initial=nan(Nt,2);
log.actual_guard_damping_bracket_final=nan(Nt,2);
log.actual_guard_search_mode=repmat({'none'},Nt,1);
log.actual_guard_fallback_kind=repmat({'none'},Nt,1);
log.actual_guard_trigger_reason=repmat({'none'},Nt,1);
log.actual_guard_hold_reason=repmat({'none'},Nt,1);
log.recovery_latched=false(Nt,1);
log.recovery_entered=false(Nt,1);
log.recovery_released=false(Nt,1);
log.recovery_release_trial=false(Nt,1);
log.recovery_hold_steps=zeros(Nt,1);
log.recovery_clear_steps=zeros(Nt,1);
log.post_noise_feedback_valid=false(Nt,1);
log.policy_alpha_ceiling=nan(Nt,1);
log.policy_tick=false(Nt,1);
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
log.ee=nan(Nt,3);

ee_start= ee_pos(robot,q0,eeName);
ee_goal = ee_pos(robot,qf,eeName);

t_reach=NaN;
stop_stable_required=double(cfg.STOP.STABLE_TIME);

stop_stable_elapsed = 0;

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
    if log.policy_tick(k) && isfield(info,'mpc') && isstruct(info.mpc)
        log.mpc_failed(k)=double(getfield_def(info.mpc,'failed',NaN));
        log.mpc_completed_horizon_steps(k)=double(getfield_def(info.mpc,'completed_horizon_steps',NaN));
        log.mpc_n_eval(k)=double(getfield_def(info.mpc,'n_eval',NaN));
        log.mpc_n_hard_reject(k)=double(getfield_def(info.mpc,'n_hard_reject',NaN));
        log.mpc_solve_wall_s(k)=double(getfield_def(info.mpc,'solve_wall_s',NaN));
    end
    tau_cmd = row(tau_cmd);

    % Predictive execution clamp: source power and plant integration use the
    % same midpoint velocity, so the physical 12-V/5-A peak is authoritative.
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
    sample_end_t = t + cfg.TS;

    s = min(traj.Tf, s + cfg.TS*a_exec);

    dq_pwr = dq_mid;
    Pi_raw = tau_m .* dq_pwr;
    [measured,plant]=spinn3d_plant_observe(plant,tau_m,dq_pwr,cfg.TS);
    [obs, bus_state_now] = ctl.observe(tau_m,dq_pwr,a_exec,step_audit,measured);
    flow = obs.flow;

    % Only the filtered experimental budget is disabled above.  The shared
    % physical motion/actuator guard remains active; compute the separate
    % physical BUS telemetry for comparable Ebuf/Tbrk reporting without
    % reintroducing the filtered budget.
    if ~strcmp(bus.model_scope,'dynamic_capacitor_common_dc_v1') && ~plant.unknown && exist('bus_tel','var') && isstruct(bus_tel) && logical(getfield_def(bus_tel,'enable_state',false))
        [flow_tel, st_bus_tel] = spinn3d_bus_step(tau_m, dq_pwr, bus_tel, st_bus_tel, cfg.TS);
        bus_avg_tel = bus_tel;
        bus_avg_tel.Pgrid_avg_prev = Pgrid_avg_tel;
        [Pgrid_avg_tel, ~, ~, ~] = spinn3d_bus_avg_guard(flow_tel.Pgrid, Pbud_plot, bus_avg_tel, cfg.TS);
        flow = flow_tel;
        bus_state_now = st_bus_tel;
        obs.Pgrid_avg_lp = Pgrid_avg_tel;
    end

    log.t(k)=sample_end_t; log.s(k)=s; log.q(k,:)=q; log.dq(k,:)=dq;
    log.tau(k,:)=tau_m; log.ddq(k,:)=ddq; log.dq_pwr(k,:)=dq_pwr;
    log.alpha(k)=a_exec; log.alpha_cmd(k)=info.a_max;
    log.actual_guard_reference_q_error_norm(k)=norm(row(q_r)-row(q_m));
    log.recovery_latched(k)=logical(obs.recovery_latched);
    log.recovery_entered(k)=logical(obs.recovery_entered);
    log.recovery_released(k)=logical(obs.recovery_released);
    log.recovery_release_trial(k)=logical(obs.recovery_release_trial);
    log.recovery_hold_steps(k)=double(obs.recovery_hold_steps);
    log.recovery_clear_steps(k)=double(obs.recovery_clear_steps);
    log.post_noise_feedback_valid(k)= ...
        logical(obs.post_noise_feedback_valid);
    log.policy_alpha_ceiling(k)=double(obs.policy_alpha_ceiling);
    log.policy_tick(k)=logical(info.policy_tick);
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
    Tee = getTransform(plant.robot, q, eeName); log.ee(k,:) = tform2trvec_safe(Tee);
    log.step_wall_s(k)=toc(step_clock);
    if ~step_audit.final_motion.pass
        termination_reason='execution_hard_constraint_violation';
        log=truncate_log(log,k);
        break;
    end

    ee_err = norm(log.ee(k,:) - ee_goal);
    stop_inside = (s >= traj.Tf) && (ee_err <= cfg.STOP.NEAR_TOL) && ...
                  (norm(dq) <= cfg.STOP.VEL_TOL);
    if stop_inside
        stop_stable_elapsed = stop_stable_elapsed + cfg.TS;
    else
        stop_stable_elapsed = 0;
    end
    if stop_inside && stop_stable_elapsed + 1e-12 >= stop_stable_required
        t_reach = sample_end_t;
        log = truncate_log(log, k);
        break;
    end

    t = t + cfg.TS;
end

summary = summary_from_log_local(log, t_reach, NaN, 'mpc_rg_grid');
dc_summary=spinn3d_dynamic_bus_summary(log,bus);
dc_names=fieldnames(dc_summary);
for dc_i=1:numel(dc_names),summary.(dc_names{dc_i})=dc_summary.(dc_names{dc_i});end
summary.Pgrid_static_initial_W = power_init.Pgrid_static_W;
summary.Pgrid_avg_initial_W = power_init.Pgrid_avg0_W;
summary.power_init_scheme = power_init.scheme;
goal_reached = isfinite(t_reach);
execution_audit = spinn3d_execution_hard_audit( ...
    log,plant.robot,limits,plant.bus,Pbud_plot,cfg.EXECUTION_AUDIT,cfg.WORKSPACE);
accepted = goal_reached && execution_audit.pass;
if ~execution_audit.pass
    termination_reason = 'execution_hard_constraint_violation';
elseif ~goal_reached
    termination_reason = 'time_cap_not_reached';
else
    termination_reason = 'reached';
end
reference = struct('kind','cartesian-bezier-dls','Tf',double(traj.Tf));
out = struct('log',log,'limits',limits,'gains',gains,'cfg',cfg, ...
    'method_id','mpc_rg_grid', ...
    'actual_guard_telemetry_schema','actual_guard_liveness_v3', ...
    'power_init',power_init, ...
    't_reach',t_reach,'reached',goal_reached,'goal_reached',goal_reached, ...
    'accepted',accepted,'time_contract_pass',goal_reached, ...
    'termination_reason',termination_reason,'traj_Tf',double(traj.Tf), ...
    'reference',reference,'diagnostic_horizon',horizon_info, ...
    'Pbud_plot',Pbud_plot,'summary',summary, ...
    'execution_audit',execution_audit);

out.perf=spinn3d_execution_perf(log,cfg.TS);
out.plant_model=plant.meta;
out.method_id='mpc_rg_grid';
out.summary.mpc_failed_policy_ticks=sum(log.mpc_failed==1);
out.summary.mpc_total_policy_ticks=sum(log.policy_tick);
out.summary.mpc_unreported_policy_ticks=sum(log.policy_tick & ~isfinite(log.mpc_failed));

if get_disable_plot_local(cfg)
    fig = [];
    return;
end

fig = figure('Name', ...
    'CTC+Gov (online nonn; filtered budget off)','Visible','on');
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
title(ax3, ...
    'EE Trajectory + Robot Skeleton (Initial/Final; MPC-RG-grid)');
legend(ax3,'Location','best');

axU = nexttile(tlo); hold(axU,'on'); grid(axU,'on');
for j=1:nJ, plot(axU, log.t, log.Pi_raw(:,j), 'DisplayName', sprintf('J%d',j)); end
xlabel(axU,'t [s]'); ylabel(axU,'Joint Power P_j [W]');
title(axU,'Per-Joint Power'); legend(axU,'show','Location','best'); xlim(axU,[log.t(1),log.t(end)]);

axD = nexttile(tlo); hold(axD,'on'); grid(axD,'on');
plot(axD, log.t, log.Pgrid, 'DisplayName','P_{grid}');
if isfield(log,'Pgrid_avg') && ~isempty(log.Pgrid_avg) && any(isfinite(log.Pgrid_avg))
    plot(axD, log.t, log.Pgrid_avg, 'LineWidth',1.6, 'DisplayName','P_{grid} filtered bus');
else

    win=max(1,round(double(bus.P_avg_window_s)/cfg.TS));
    plot(axD, log.t, movmean(log.Pgrid,win), 'LineWidth',1.6, 'DisplayName','P_{grid} filtered');
end
ymax=max(log.Pgrid)*1.2; if ~isfinite(ymax)||ymax<=0, ymax=1; end
% Keep the disabled configured filtered-power reference visible even when this
% nominal MPC-RG-grid trajectory never approaches it.  Otherwise the autoscaled
% plot can hide the line and make unused headroom impossible to diagnose.
if isfinite(Pbud_plot)
    ymax=max(ymax,1.05*Pbud_plot);
end
ylim(axD,[0,ymax]); xlim(axD,[log.t(1), log.t(end)]);
if isfinite(Pbud_plot) && Pbud_plot <= ymax, yline(axD, Pbud_plot, '--r','P_{budget} (disabled)'); end
xlabel(axD,'t [s]'); ylabel(axD,'Power [W]');
if isfinite(Pbud_plot) && Pbud_plot>0 && any(isfinite(log.Pgrid_avg))
    pavg_util=100*max(log.Pgrid_avg,[],'omitnan')/Pbud_plot;
    title(axD,sprintf('Bus Power (MPC-RG-grid; filtered peak %.1f%% of %.1f W)', ...
        pavg_util,Pbud_plot));
else
    title(axD,'Bus Power (MPC-RG-grid; filtered budget disabled)');
end
legend(axD,'show','Location','best');

spinn3d_plot_dynamic_bus(tlo,log,bus);
drawnow;

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

%% ==================== helpers ====================

function tf = get_disable_plot_local(cfg)

tf = ~logical(cfg.PLOT.enable);
if isfield(cfg,'DISABLE_PLOT') && ~isempty(cfg.DISABLE_PLOT)
    tf = logical(cfg.DISABLE_PLOT);
    return;
end
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
