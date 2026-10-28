function A = spinn3d_execution_hard_audit( ...
    log,robot,limits,bus,Pavg_budget_W,audit,workspace)
% Evaluate trajectory completion and joint, actuator and DC-link limits.

if nargin ~= 7
    error('spinn3d:MissingCentralExecutionAudit', ...
        ['Expected log, robot, limits, bus, Pavg_budget_W and the complete ', ...
         'cfg.EXECUTION_AUDIT/cfg.WORKSPACE contracts.']);
end
required_audit = {'ratio_tolerance', ...
    'position_planning_tolerance_rad','position_tolerance_rad', ...
    'filtered_power_tolerance_W','physical_power_abs_tolerance_W', ...
    'physical_power_rel_tolerance','spill_power_tolerance_W', ...
    'telemetry_match_tolerance_W','energy_balance_tolerance_W', ...
    'joint_limit_viability'};
if ~isstruct(audit) || ~all(isfield(audit,required_audit))
    error('spinn3d:MissingCentralExecutionAudit', ...
        ['cfg.EXECUTION_AUDIT is incomplete. Configure all audit ', ...
         'tolerances in spinn3d_params_block.m.']);
end
planning_qpos_tol = nonnegative_scalar_local( ...
    audit.position_planning_tolerance_rad, ...
    'position_planning_tolerance_rad');
if planning_qpos_tol~=0
    error('spinn3d:NonphysicalPositionPlanningTolerance', ...
        ['cfg.EXECUTION_AUDIT.position_planning_tolerance_rad must be ', ...
         'exactly zero; position_tolerance_rad is final-audit-only.']);
end
ratio_tol = nonnegative_scalar_local( ...
    audit.ratio_tolerance,'ratio_tolerance');
qpos_tol = nonnegative_scalar_local( ...
    audit.position_tolerance_rad,'position_tolerance_rad');
filtered_power_tol = nonnegative_scalar_local( ...
    audit.filtered_power_tolerance_W,'filtered_power_tolerance_W');
physical_power_abs_tol = nonnegative_scalar_local( ...
    audit.physical_power_abs_tolerance_W, ...
    'physical_power_abs_tolerance_W');
physical_power_rel_tol = nonnegative_scalar_local( ...
    audit.physical_power_rel_tolerance, ...
    'physical_power_rel_tolerance');
spill_power_tol = nonnegative_scalar_local( ...
    audit.spill_power_tolerance_W,'spill_power_tolerance_W');
telemetry_tol = nonnegative_scalar_local( ...
    audit.telemetry_match_tolerance_W,'telemetry_match_tolerance_W');
energy_balance_tol = nonnegative_scalar_local( ...
    audit.energy_balance_tolerance_W,'energy_balance_tolerance_W');

nJ = size(log.q,2);

qd_lim = expand_local(field_local(limits,'qd_max',inf),nJ,inf);
qdd_lim = expand_local(field_local(limits,'qdd_cmd_max_actual', ...
    field_local(limits,'qdd_max',inf)),nJ,inf);
tau_lim = expand_local(field_local(limits,'tau_max',inf),nJ,inf);
Pax_lim = expand_local(field_local(limits,'P_axis_max',inf),nJ,inf);

q = double(log.q);
dq = double(log.dq);
tau = double(log.tau);
nS = size(q,1);
if isfield(log,'ddq') && ~isempty(log.ddq) && ...
        isequal(size(log.ddq),size(dq))
    ddq = double(log.ddq);
elseif size(dq,1) >= 2
    dt = diff(double(log.t(:)));
    ddq = diff(dq,1,1)./max(dt,eps);
else
    ddq = zeros(1,nJ);
end
if ~isequal(size(ddq),size(q))
    error('spinn3d_execution_hard_audit:BadAccelerationLog', ...
        ['Executed log.ddq must have the same N-by-nJ shape as log.q/log.dq ', ...
         'for the independent within-step joint-position audit.']);
end

% Reconstruct every constant-acceleration interval from its logged step-end
% state.  Endpoint-only checks miss a reversal extremum inside the 1-ms ZOH
% interval, precisely when an emergency braking command changes velocity
% sign before the next sample.
t_motion = required_log_vector_local(log,'t',nS);
dt_motion = [t_motion(1);diff(t_motion)];
if any(dt_motion<=0)
    error('spinn3d_execution_hard_audit:BadTimebase', ...
        'Executed log.t must contain strictly increasing positive step-end times.');
end
dq_step_start = dq-dt_motion.*ddq;
q_step_start = q-dt_motion.*(dq-0.5*dt_motion.*ddq);
[q_step_min,q_step_max] = quadratic_step_extrema_local( ...
    q_step_start,dq_step_start,ddq,dt_motion,q);

if isfield(log,'dq_pwr') && ~isempty(log.dq_pwr) && ...
        isequal(size(log.dq_pwr),size(dq))
    dq_pwr = double(log.dq_pwr);
elseif isequal(size(ddq),size(dq)) && isfield(log,'t') && ...
        numel(log.t)==size(dq,1)
    dt = [double(log.t(1));diff(double(log.t(:)))];
    dq_pwr = dq - 0.5*dt.*ddq;
else
    dq_pwr = dq;
end

[qlo,qhi] = joint_limits_local(robot,nJ);
q_violation = max([0; qlo(:)-min(q_step_min,[],1).'; ...
                      max(q_step_max,[],1).'-qhi(:)]);
joint_limit_viability = spinn3d_joint_limit_viability( ...
    q,dq,qlo,qhi,qdd_lim,audit.joint_limit_viability,qpos_tol);
qd_ratio = max(abs(dq)./max(abs(qd_lim),eps),[],'all');
qdd_ratio = max(abs(ddq)./max(abs(qdd_lim),eps),[],'all');
tau_ratio = max(abs(tau)./max(abs(tau_lim),eps),[],'all');
Paxis = max(max(tau.*dq_step_start,tau.*dq),0.0);
Paxis_ratio = max(Paxis./max(Pax_lim,eps),[],'all');
geometry = spinn3d_physical_geometry_check(robot,q,workspace);

bus = spinn3d_bus_defaults(bus);
dynamic_dc=strcmp(bus.model_scope,'dynamic_capacitor_common_dc_v1');
nS=size(tau,1);Pgrid_guard_upper=zeros(nS,1);motor_ratio=0;
if ~dynamic_dc
    motor_ratio=motor_ratio_local(tau,dq_pwr,bus,nJ);
    for ki=1:nS
        interval=spinn3d_motor_interval_audit(tau(ki,:),dq_step_start(ki,:),dq(ki,:),bus);
        motor_ratio=max(motor_ratio,max(interval.torque_speed_ratio));
        Pgrid_guard_upper(ki)=interval.Pgrid_unshared_upper_bound_W;
    end
end
Ppeak = double(field_local(bus,'P_grid_peak_max',inf));

% every buffer-free BUS step from executed torque and midpoint velocity.
% This prevents a missing/contaminated log.Pgrid from making the post-run

% profiles retain logged telemetry because their carried initial energy is
% not part of this function's public input.
nS = size(tau,1);
Pgrid_logged = required_log_vector_local(log,'Pgrid',nS);
Pspill_logged = required_log_vector_local(log,'Pspill',nS);
innovation_names = {'Pshare','Pshare_axis','Pshare_potential', ...
    'Pregen_available','Pregen_bus','Pregen_cap_curtailed', ...
    'Pregen_curtailed','Pregen_curtailed_total','Preuse_total', ...
    'Ploss_total'};
innovation_logged = zeros(nS,numel(innovation_names));
for j = 1:numel(innovation_names)
    innovation_logged(:,j) = required_log_vector_local( ...
        log,innovation_names{j},nS);
end
balance_logged = required_log_vector_local( ...
    log,'energy_balance_residual_W',nS);
Pavg_logged = [];
Pavg_initial_logged = [];
Pavg_audit = [];
Pavg_telemetry_error_max = NaN;
Pavg_telemetry_pass = true;
Pavg_telemetry_available = true;
Pavg_replay_available = false;
Pavg_replay_authoritative = false;
Pavg_replay_initial_source = 'not_requested';
Pavg_initial_W = NaN;
if ~isempty(Pavg_budget_W) && isfinite(double(Pavg_budget_W))
    [Pavg_logged,pavg_log_ok] = optional_log_vector_local( ...
        log,'Pgrid_avg',nS);
    [Pavg_initial_logged,pavg_initial_log_ok] = ...
        optional_log_vector_local(log,'Pgrid_avg_pre_t0_W',nS);
    Pavg_telemetry_available = logical( ...
        pavg_log_ok && pavg_initial_log_ok);
    Pavg_telemetry_pass = false;
end
Pgrid_audit = Pgrid_logged;
Pspill_audit = Pspill_logged;
telemetry_error_max = NaN;
innovation_telemetry_error_max = NaN;
energy_balance_residual_max = NaN;
telemetry_pass = true;
innovation_telemetry_pass = true;
energy_balance_pass = true;
dc_audit=struct('available',false,'pass',true,'voltage_pass',true, ...
    'thermal_pass',true,'solver_pass',true,'source_pass',true, ...
    'Vdc_min_V',NaN,'Vdc_max_V',NaN,'initial_state_error',NaN, ...
    'state_telemetry_error',NaN,'solver_residual_max_W',NaN);
if ~logical(bus.enable_state)
    Pgrid_calc = zeros(nS,1);
    Pspill_calc = zeros(nS,1);
    innovation_calc = zeros(nS,numel(innovation_names));
    balance_calc = zeros(nS,1);
    for k = 1:nS
        [flow_k,~] = spinn3d_bus_step( ...
            tau(k,:),dq_pwr(k,:),bus,[],1.0);
        Pgrid_calc(k) = flow_k.Pgrid;
        Pspill_calc(k) = flow_k.Pspill;
        innovation_calc(k,:) = [flow_k.Pshare, ...
            flow_k.Pshare_axis,flow_k.Pshare_potential, ...
            flow_k.Pregen_available,flow_k.Pregen_bus, ...
            flow_k.Pregen_cap_curtailed,flow_k.Pregen_curtailed, ...
            flow_k.Pregen_curtailed_total,flow_k.Preuse_total, ...
            flow_k.Ploss_total];
        balance_calc(k) = max(abs([ ...
            flow_k.Pload_balance_residual, ...
            flow_k.Pregen_route_balance_residual, ...
            flow_k.Penergy_balance_residual]));
    end
    innovation_telemetry_error_max = max(abs( ...
        innovation_logged-innovation_calc),[],'all');
    telemetry_error_max = max(abs([ ...
        Pgrid_logged-Pgrid_calc;Pspill_logged-Pspill_calc; ...
        innovation_logged(:)-innovation_calc(:); ...
        balance_logged-balance_calc]));
    energy_balance_residual_max = max([balance_calc;balance_logged]);
    innovation_telemetry_pass = ...
        innovation_telemetry_error_max <= telemetry_tol;
    telemetry_pass = telemetry_error_max <= telemetry_tol;
    energy_balance_pass = energy_balance_residual_max <= ...
        energy_balance_tol;
    Pgrid_audit = Pgrid_calc;
    Pspill_audit = Pspill_calc;
end

if dynamic_dc
    % Reconstruct the common static-hold boundary using the ACTUAL robot and

    % retrospective validation, not an oracle available to the controller.
    fric0=struct('B',zeros(1,nJ),'Fc',zeros(1,nJ),'vel_eps',1e-3);
    Hdc=spinn3d_static_hold_power_init(robot,q_step_start(1,:),fric0,bus, ...
        max(1,double(field_local(bus,'P_soft_reference_W',13))),dt_motion(1));
    sdc=Hdc.bus_state0;
    v0log=required_log_vector_local(log,'Vdc_pre_t0_V',nS);
    e0log=required_log_vector_local(log,'Ecap_pre_t0_J',nS);
    v_log=required_log_vector_local(log,'Vdc',nS);
    energy_log=required_log_vector_local(log,'Ebuf',nS);
    dE_log=required_log_vector_local(log,'Ecap_delta_J',nS);
    vmin_log=required_log_vector_local(log,'Vdc_min',nS);
    vmax_log=required_log_vector_local(log,'Vdc_max',nS);
    solver_log=required_log_vector_local(log,'dc_solver_residual_max_W',nS);
    flags={'dc_solver_ok','dc_state_pass','dc_voltage_ok'};
    flaglog=zeros(nS,numel(flags));
    for j=1:numel(flags),flaglog(:,j)=required_log_vector_local(log,flags{j},nS);end
    dc_audit.available=true;
    dc_audit.initial_state_error=max(abs([v0log-sdc.Vdc; ...
        e0log-0.5*bus.C_dc_F*sdc.Vdc^2]));
    dyn_names={'Pbus_source','Isource','Psource_loss','Pline_loss','Pcurrent_limit_loss','Pdump','dc_residual_W'};
    dyn_flow={'Pbus_source','I_source_total','Psource_loss','Pline_loss','Pcurrent_limit_loss','Pdump','Pload_balance_residual'};
    dyn_log=zeros(nS,numel(dyn_names));dyn_calc=dyn_log;
    for j=1:numel(dyn_names),dyn_log(:,j)=required_log_vector_local(log,dyn_names{j},nS);end

    % them at the same independently reconstructed electrical substeps.
    source_diag_names={'Psource_request_W','Psource_request_peak_W','Isource_peak_A'};
    source_diag_calc=nan(nS,numel(source_diag_names));
    Pgrid_calc=zeros(nS,1);Pspill_calc=zeros(nS,1);
    innovation_calc=zeros(nS,numel(innovation_names));balance_calc=zeros(nS,1);
    state_calc=zeros(nS,6);flagcalc=flaglog;voltok=true;solvok=true;sourceok=true;
    for k=1:nS
        [flow_k,snext]=spinn3d_bus_step(tau(k,:),dq_pwr(k,:),bus,sdc,dt_motion(k));
        Pgrid_calc(k)=flow_k.Pgrid;Pspill_calc(k)=flow_k.Pspill;
        for j=1:numel(innovation_names),innovation_calc(k,j)=flow_k.(innovation_names{j});end
        for j=1:numel(dyn_names),dyn_calc(k,j)=flow_k.(dyn_flow{j});end
        for j=1:numel(source_diag_names)
            source_diag_calc(k,j)=flow_k.(source_diag_names{j});
        end
        balance_calc(k)=max(abs([flow_k.Pload_balance_residual, ...
            flow_k.Pregen_route_balance_residual,flow_k.Penergy_balance_residual]));
        state_calc(k,:)=[snext.Vdc,snext.Ebuf,flow_k.Ecap_delta, ...
            flow_k.Vdc_min,flow_k.Vdc_max,flow_k.dc_solver_residual_max_W];
        flagcalc(k,:)=[flow_k.solver_ok,flow_k.state_pass,flow_k.voltage_ok];
        voltok=voltok && flow_k.voltage_ok;
        solvok=solvok && flow_k.solver_ok;sourceok=sourceok && flow_k.source_ok;
        % Worst resolved link voltage combined with each mechanical speed
        % endpoint bounds the passive current/voltage envelope for this step.
        benv=bus;benv.V_bus_V=max(flow_k.Vdc_min,eps);
        env=spinn3d_motor_envelope(repmat(tau(k,:),2,1), ...
            [dq_step_start(k,:);dq(k,:)],benv,nJ);
        motor_ratio=max(motor_ratio,max(env.torque_speed_ratio,[],'all'));
        Pgrid_guard_upper(k)=flow_k.Pgrid;
        sdc=snext;
    end
    state_logged=[v_log,energy_log,dE_log,vmin_log,vmax_log,solver_log];
    dc_audit.state_telemetry_error=max(abs(state_logged-state_calc),[],'all');
    dc_audit.Vdc_min_V=min(state_calc(:,4));dc_audit.Vdc_max_V=max(state_calc(:,5));
    dc_audit.solver_residual_max_W=max(state_calc(:,6));
    dc_audit.voltage_pass=voltok;dc_audit.thermal_pass=true;
    dc_audit.brake_model='none';
    dc_audit.solver_pass=solvok;dc_audit.source_pass=sourceok;
    dc_audit.source_diagnostic_schema='same_state_unclipped_source_droop_v1';
    dc_audit.source_diagnostic_available=true;
    source_diag_logged=nan(size(source_diag_calc));
    for j=1:numel(source_diag_names)
        key=source_diag_names{j};
        if isfield(log,key)&&isnumeric(log.(key))&&numel(log.(key))==nS&& ...
                all(isfinite(log.(key)(:)))
            source_diag_logged(:,j)=double(log.(key)(:));
        else
            dc_audit.source_diagnostic_available=false;
        end
        dc_audit.([key '_replay'])=source_diag_calc(:,j);
    end
    dc_audit.source_diagnostic_error_max=NaN;
    if dc_audit.source_diagnostic_available
        dc_audit.source_diagnostic_error_max=max(abs(source_diag_logged-source_diag_calc),[],'all');
    end
    dc_audit.source_diagnostic_pass=dc_audit.source_diagnostic_available&& ...
        dc_audit.source_diagnostic_error_max<=telemetry_tol;
    dc_audit.pass=voltok && solvok && sourceok && ...
        dc_audit.initial_state_error<=1e-8 && dc_audit.state_telemetry_error<=1e-8 && ...
        isequal(flaglog,double(flagcalc));
    innovation_telemetry_error_max=max(abs(innovation_logged-innovation_calc),[],'all');
    telemetry_error_max=max(abs([Pgrid_logged-Pgrid_calc;Pspill_logged-Pspill_calc; ...
        innovation_logged(:)-innovation_calc(:);dyn_log(:)-dyn_calc(:);balance_logged-balance_calc]));
    energy_balance_residual_max=max([balance_calc;abs(balance_logged)]);
    innovation_telemetry_pass=innovation_telemetry_error_max<=telemetry_tol;
    telemetry_pass=telemetry_error_max<=telemetry_tol && dc_audit.pass;
    energy_balance_pass=energy_balance_residual_max<=energy_balance_tol;
    Pgrid_audit=Pgrid_calc;Pspill_audit=Pspill_calc;
end

% Independently replay the causal EMA from the executed/replayed source
% power.  The logged filtered trace is evidence only and cannot be the
% acceptance signal.  Every demo signs the common pre-t0 static-hold state.
if ~isempty(Pavg_budget_W) && isfinite(double(Pavg_budget_W))
    t_audit=required_log_vector_local(log,'t',nS);
    dt_step=[t_audit(1);diff(t_audit)];
    if t_audit(1)<=0 || any(dt_step<=0)
        error('spinn3d_execution_hard_audit:BadTimebase', ...
            'Executed log.t must contain strictly increasing step-end times.');
    end
    % Reconstruct the signed pre-t0 static-hold EMA independently from the
    % executed initial configuration.  Pavg log fields are optional KPI
    % evidence: absent/corrupt evidence lowers only Pavg data quality and
    % can never turn an otherwise physical execution into a method failure.
    fric0=struct('B',zeros(1,nJ),'Fc',zeros(1,nJ), ...
        'vel_eps',1e-3);
    try
        H0=spinn3d_static_hold_power_init(robot,q_step_start(1,:), ...
            fric0,bus,double(Pavg_budget_W),dt_step(1));
        p0=double(H0.Pgrid_avg0_W);
    catch
        p0=NaN;
    end
    if isfinite(p0) && p0>=0
        Pavg_initial_W=p0;
        Pavg_replay_available=true;
        if ~logical(bus.enable_state) || dynamic_dc
            % Source power was independently replayed above as well, so the
            % complete EMA input and initial condition are authoritative.
            Pavg_replay_authoritative=true;
            Pavg_replay_initial_source= ...
                'independent_static_hold_reconstruction_v1';
        else
            % A stateful BUS requires its carried pre-t0 electrical state to

            
            Pavg_replay_initial_source= ...
                ['independent_static_hold_plus_logged_stateful_', ...
                 'power_non_authoritative'];
        end
    elseif pavg_initial_log_ok

        % trace is available, but do not call that replay authoritative.
        Pavg_initial_W=Pavg_initial_logged(1);
        Pavg_replay_available=true;
        Pavg_replay_initial_source='logged_initial_fallback_non_authoritative';
    else
        Pavg_replay_initial_source='unavailable';
    end
    if Pavg_replay_available
        Pavg_audit=zeros(nS,1);prev=Pavg_initial_W;
        tau_avg=double(bus.P_avg_window_s);
        for k=1:nS
            beta=exp(-dt_step(k)/tau_avg);
            prev=beta*prev+(1-beta)*Pgrid_audit(k);
            Pavg_audit(k)=prev;
        end
    end
    if Pavg_telemetry_available && Pavg_replay_available
        p0_error=max(abs(Pavg_initial_logged-Pavg_initial_W));
        Pavg_telemetry_error_max=max([p0_error; ...
            abs(Pavg_logged-Pavg_audit)]);
        Pavg_telemetry_pass = ...
            Pavg_telemetry_error_max<=telemetry_tol;
    end

    % independently so missing/corrupt KPI evidence is visible, but do not
    % fold this result into the hard BUS telemetry pass used by A.pass.
end

Pgrid_peak = max(Pgrid_audit,[],'all');
physical_power_pass = ~isfinite(Ppeak) || ...
    max(Pgrid_peak,max(Pgrid_guard_upper)) <= Ppeak + max(physical_power_abs_tol, ...
                              physical_power_rel_tol*Ppeak);
Pspill_peak = max(Pspill_audit,[],'all');
spill_pass = Pspill_peak <= spill_power_tol;

filtered_power_pass = true;
Pavg_peak = NaN;
Pavg_budget = NaN;
Pavg_excess_peak = NaN;
Pavg_excess_duration = NaN;
Pavg_excess_fraction = NaN;
Pavg_excess_integral = NaN;
Pavg_excess_sq_norm_integral = NaN;
Pavg_excess_sq_norm_mean = NaN;
Pavg_evaluation_duration = NaN;
Pavg_budget_margin = NaN;
Pavg_excess_percent = NaN;
if ~isempty(Pavg_budget_W) && isfinite(double(Pavg_budget_W)) && ...
        Pavg_replay_available
    Pavg_budget = double(Pavg_budget_W)* ...
        double(field_local(bus,'P_avg_target_frac',1.0));
    Pavg_peak = max(Pavg_audit,[],'all');
    filtered_power_pass = ...
        Pavg_peak <= Pavg_budget + filtered_power_tol;
    Pavg_excess = max(Pavg_audit-Pavg_budget,0.0);
    Pavg_excess_peak = max(Pavg_excess,[],'all');
    Pavg_budget_margin = Pavg_budget-Pavg_peak;
    Pavg_excess_percent = 100*Pavg_excess_peak/max(Pavg_budget,eps);
    Pavg_excess_duration = sum(dt_step.*double(Pavg_audit>Pavg_budget));
    Pavg_excess_fraction = Pavg_excess_duration/max(sum(dt_step),eps);
    % The logged samples are step-end values for a causal 1-ms ZOH update;
    % include the first [0,t(1)] interval exactly as the duration metric does.
    Pavg_excess_integral = sum(dt_step.*Pavg_excess);
    % Primary soft-power objective.  Normalising before squaring makes the
    % metric comparable across configured budget scan points; dividing its
    % integral by the observed duration prevents a faster method from
    % appearing better merely because it generated fewer samples.
    Pavg_evaluation_duration = sum(dt_step);
    Pavg_excess_sq_norm = ...
        (Pavg_excess./max(Pavg_budget,eps)).^2;
    Pavg_excess_sq_norm_integral = ...
        sum(dt_step.*Pavg_excess_sq_norm);
    Pavg_excess_sq_norm_mean = Pavg_excess_sq_norm_integral / ...
        max(Pavg_evaluation_duration,eps);
end

axis_power_pass = Paxis_ratio <= 1+ratio_tol;
motor_envelope_pass = motor_ratio <= 1+ratio_tol;
nonpower_pass = q_violation <= qpos_tol && ...
    joint_limit_viability.pass && ...
    qd_ratio <= 1+ratio_tol && ...
    qdd_ratio <= 1+ratio_tol && tau_ratio <= 1+ratio_tol && ...
    axis_power_pass && motor_envelope_pass && geometry.pass;
A = struct('pass',logical(nonpower_pass && physical_power_pass && ...
    spill_pass && telemetry_pass && ...
    energy_balance_pass), ...
    'nonpower_pass',logical(nonpower_pass), ...
    'physical_power_pass',logical(physical_power_pass), ...
    'filtered_power_pass',logical(filtered_power_pass), ...
    'filtered_power_hard_enforced',false, ...
    'Pgrid_avg_hard_enforced',false, ...
    'Pavg_hard_enforced',false, ...
    'spill_pass',logical(spill_pass), ...
    'telemetry_pass',logical(telemetry_pass), ...
    'Pavg_telemetry_pass',logical(Pavg_telemetry_pass), ...
    'Pavg_telemetry_available',logical(Pavg_telemetry_available), ...
    'Pavg_replay_available',logical(Pavg_replay_available), ...
    'Pavg_replay_authoritative',logical(Pavg_replay_authoritative), ...
    'Pavg_replay_schema', ...
        'execution_hard_audit_pavg_replay_v2_normalized_square', ...
    'Pavg_replay_initial_source',Pavg_replay_initial_source, ...
    'Pgrid_avg_initial_replay_W',Pavg_initial_W, ...
    'innovation_telemetry_pass',logical(innovation_telemetry_pass), ...
    'energy_balance_pass',logical(energy_balance_pass), ...
    'axis_power_pass',logical(axis_power_pass), ...
    'motor_envelope_pass',logical(motor_envelope_pass), ...
    'joint_limit_viability_pass',logical(joint_limit_viability.pass), ...
    'geometry_pass',logical(geometry.pass), ...
    'workplane_pass',logical(geometry.workplane_pass), ...
    'self_collision_pass',logical(geometry.self_collision_pass), ...
    'qpos_violation_rad',q_violation,'qd_ratio',qd_ratio, ...
    'joint_limit_viability_violation_rad', ...
        joint_limit_viability.violation_rad, ...
    'qdd_ratio',qdd_ratio,'tau_ratio',tau_ratio, ...
    'Paxis_ratio',Paxis_ratio,'motor_envelope_ratio',motor_ratio, ...
    'Pgrid_peak_W',Pgrid_peak,'Pgrid_physical_cap_W',Ppeak, ...
    'Pgrid_interval_guard_upper_peak_W',max(Pgrid_guard_upper), ...
    'electrical_interval_schema','held_torque_affine_speed_passive_upper_bound_v97', ...
    'Pgrid_avg_peak_W',Pavg_peak, ...
    'Pgrid_avg_replay_W',Pavg_audit, ...
    'Pgrid_avg_budget_W',Pavg_budget, ...
    'Pgrid_avg_excess_peak_W',Pavg_excess_peak, ...
    'Pgrid_avg_budget_margin_W',Pavg_budget_margin, ...
    'Pgrid_avg_excess_peak_percent',Pavg_excess_percent, ...
    'Pgrid_avg_excess_duration_s',Pavg_excess_duration, ...
    'Pgrid_avg_excess_time_fraction',Pavg_excess_fraction, ...
    'Pgrid_avg_excess_integral_J',Pavg_excess_integral, ...
    'Pgrid_avg_excess_sq_norm_integral_s', ...
        Pavg_excess_sq_norm_integral, ...
    'Pgrid_avg_excess_sq_norm_mean',Pavg_excess_sq_norm_mean, ...
    'Pgrid_avg_evaluation_duration_s',Pavg_evaluation_duration, ...
    'Pavg_excess_peak_W',Pavg_excess_peak, ...
    'Pavg_excess_peak_pct',Pavg_excess_percent, ...
    'Pavg_excess_duration_s',Pavg_excess_duration, ...
    'Pavg_excess_integral_J',Pavg_excess_integral, ...
    'Pavg_excess_sq_norm_integral_s', ...
        Pavg_excess_sq_norm_integral, ...
    'Pavg_excess_sq_norm_mean',Pavg_excess_sq_norm_mean, ...
    'R_soft_integral_s',Pavg_excess_sq_norm_integral, ...
    'R_soft',Pavg_excess_sq_norm_mean, ...
    'Pspill_peak_W',Pspill_peak, ...
    'telemetry_error_max_W',telemetry_error_max, ...
    'Pgrid_avg_telemetry_error_max_W',Pavg_telemetry_error_max, ...
    'innovation_telemetry_error_max_W', ...
        innovation_telemetry_error_max, ...
    'energy_balance_residual_max_W',energy_balance_residual_max, ...
    'workplane_margin_m',geometry.workplane_margin_m, ...
    'min_self_collision_margin_m', ...
        geometry.min_self_collision_margin_m, ...
    'geometry',geometry, ...
    'joint_limit_viability',joint_limit_viability, ...
    'audit_tolerances',audit);
A.dynamic_dc=dc_audit;
A.dc_voltage_pass=dc_audit.voltage_pass;A.dc_thermal_pass=dc_audit.thermal_pass;
A.dc_solver_pass=dc_audit.solver_pass;
if dynamic_dc
    A.electrical_interval_schema='dynamic_dc_implicit_substeps_mechanical_endpoint_envelope_v1';
    A.dc_energy=spinn3d_dynamic_bus_summary(log,bus);
end
end

function [qmin_step,qmax_step] = quadratic_step_extrema_local( ...
        q0,dq0,ddq,dt,q1)
qmin_step = min(q0,q1);
qmax_step = max(q0,q1);
for k = 1:size(q0,1)
    for j = 1:size(q0,2)
        if isfinite(ddq(k,j)) && abs(ddq(k,j))>0
            t_star = -dq0(k,j)/ddq(k,j);
            if isfinite(t_star) && t_star>0 && t_star<dt(k)
                q_star = q0(k,j)+dq0(k,j)*t_star+ ...
                    0.5*ddq(k,j)*t_star*t_star;
                qmin_step(k,j) = min(qmin_step(k,j),q_star);
                qmax_step(k,j) = max(qmax_step(k,j),q_star);
            end
        end
    end
end
end

function v = required_log_vector_local(log,name,n)
if ~isstruct(log) || ~isfield(log,name) || isempty(log.(name))
    error('spinn3d_execution_hard_audit:MissingTelemetry', ...
        'Required executed BUS telemetry log.%s is missing.',name);
end
v = double(log.(name));
if ~isreal(v) || ~isvector(v) || numel(v)~=n || any(~isfinite(v(:)))
    error('spinn3d_execution_hard_audit:BadTelemetry', ...
        'log.%s must contain %d finite real samples.',name,n);
end
v = v(:);
end

function [v,ok] = optional_log_vector_local(log,name,n)
v = [];
ok = false;
if ~isstruct(log) || ~isfield(log,name) || isempty(log.(name))
    return;
end
try
    x = double(log.(name));
catch
    return;
end
if ~isreal(x) || ~isvector(x) || numel(x)~=n || ...
        any(~isfinite(x(:)))
    return;
end
v = x(:);
ok = true;
end

function r = motor_ratio_local(tau,dq,bus,nJ)
if ~strcmpi(char(field_local(bus,'model','legacy')),'st3215_pwm')
    r=0;
    return;
end
if ~isequal(size(tau),size(dq)) || size(tau,2)~=nJ
    error('spinn3d_execution_hard_audit:BadMotorTelemetryShape', ...
        'Executed tau and midpoint dq must both be N-by-nJ arrays.');
end
env = spinn3d_motor_envelope(tau,dq,bus,nJ);
r=max(env.torque_speed_ratio,[],'all');
end

function [lo,hi] = joint_limits_local(robot,nJ)
lo=-inf(1,nJ); hi=inf(1,nJ); k=0;
for i=1:numel(robot.Bodies)
    J=robot.Bodies{i}.Joint;
    if strcmpi(char(J.Type),'fixed'), continue; end
    k=k+1;
    if k>nJ, break; end
    p=double(J.PositionLimits(:).');
    if numel(p)==2 && all(isfinite(p))
        lo(k)=min(p); hi(k)=max(p);
    end
end
end

function v=expand_local(v,n,def)
v=double(v(:).');
if isempty(v),v=def;end
if isscalar(v),v=repmat(v,1,n);end
if numel(v)<n,v=[v repmat(v(end),1,n-numel(v))];end
v=v(1:n);
end

function v=field_local(S,k,def)
if isstruct(S)&&isfield(S,k)&&~isempty(S.(k)),v=S.(k);else,v=def;end
end

function v=nonnegative_scalar_local(v,name)
v=double(v);
if ~isreal(v) || ~isscalar(v) || ~isfinite(v) || v<0
    error('spinn3d:InvalidCentralExecutionAudit', ...
        'cfg.EXECUTION_AUDIT.%s must be one finite nonnegative scalar.', ...
        name);
end
end
