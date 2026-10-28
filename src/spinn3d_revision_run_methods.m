function [T,Outputs,Executions]=spinn3d_revision_run_methods(cfg,evalSeed,callbacks,labels,executionOpts)
% Run the compared methods on one task with common random seeds.
% RG-0 supplies the paired arrival-time and soft-power reference.
if nargin<3||isempty(callbacks)
    callbacks={@spinn3d_demo_ctcgov_online_nonn_nolimit,@spinn3d_demo_ctcgov_online_nonn,@spinn3d_demo_ctcgov_online_nn};
end
if nargin<4||isempty(labels),labels={'NoGuard','Feasible-RG','Alpha-PRG'};end
if nargin<5,executionOpts=struct();end
caller_rng=rng;restore_rng=onCleanup(@()rng(caller_rng)); %#ok<NASGU>
Outputs=cell(numel(callbacks),1);rows=cell(numel(callbacks),1);
Executions=cell(numel(callbacks),1);
basecfg=cfg;basecfg.FAIRNESS.revision_independent_failure_eval=true;
basecfg.FAIRNESS.noguard_reference_available=false;
basecfg.FAIRNESS.t_reach_noguard_s=NaN;
basecfg.FAIRNESS.reference_source='paired-noguard-unavailable';
basecfg.FAIRNESS.noguard_reference_Tf_s=NaN;
basecfg.FAIRNESS.noguard_soft_reference_available=false;
basecfg.FAIRNESS.noguard_soft_excess_sq_mean=NaN;
basecfg.FAIRNESS.noguard_soft_excess_sq_integral_s=NaN;
basecfg.FAIRNESS.noguard_soft_reference_source='unavailable';
basecfg.FAIRNESS.noguard_soft_budget_W=NaN;
for j=1:numel(callbacks)
    r=blank_local(labels{j},evalSeed);
    if isfield(basecfg,'BUS') && isstruct(basecfg.BUS) && isfield(basecfg.BUS,'model_scope')
        r.bus_model_scope=string(basecfg.BUS.model_scope);
        r.manuscript_metrics_requested=double(strcmp(char(basecfg.BUS.model_scope),'dynamic_capacitor_common_dc_v1'));
    end
    attempt_clock=tic; callback_clock=[]; callback_returned=false; metric_clock=[];
    execution=[];
    r.attempt_started=1;
    try
        callback_clock=tic;
        [out,execution]=spinn3d_revision_execute(basecfg,labels{j}, ...
            evalSeed,callbacks{j},executionOpts);
        r.demo_wall_s=execution.callback_wall_s;
        callback_returned=execution.callback_returned;
        if ~callback_returned
            error(execution.error.identifier,'%s',execution.error.message);
        end
        if isstruct(out)&&isscalar(out),out.revision_demo_wall_s=r.demo_wall_s;end
        Outputs{j}=out;

        metric_clock=tic;
        M=spinn3d_revision_metrics_from_out(out,basecfg.P_TOTAL_MAX);
        M.manuscript_metrics_requested=max(r.manuscript_metrics_requested, ...
            M.manuscript_metrics_requested);
        f=fieldnames(M);for k=1:numel(f),r.(f{k})=M.(f{k});end
        if r.bus_model_scope=="unavailable" && isfield(basecfg,'BUS') && ...
                isstruct(basecfg.BUS) && isfield(basecfg.BUS,'model_scope')
            r.bus_model_scope=string(basecfg.BUS.model_scope);
        end
        r.completed=1;r.exception=0;
        if j==1
            if M.success==1&&isfinite(M.t_reach)&&M.t_reach>0
                basecfg=attach_local(basecfg,out,M);
            end
        end
    catch ME
        if ~strcmp(ME.identifier,'spinn3d_bus_ptp_dp_r2:NoPath'),rethrow(ME);end
        r.completed=0;r.exception=1;r.success=0;r.accepted=0;
        if isempty(execution) && ~callback_returned && ~isempty(callback_clock)
            r.demo_wall_s=toc(callback_clock);
        end
        r.error_id=string(ME.identifier);r.error_message=string(regexprep(ME.message,'[\r\n]+',' '));
    end
    if ~isempty(metric_clock),r.metric_wall_s=toc(metric_clock);end
    r.attempt_wall_s=toc(attempt_clock);
    if r.exception==1,r.exception_wall_s=r.attempt_wall_s;end
    if ~isempty(execution),Executions{j}=execution;end
    rows{j}=struct2table(r);
end
T=vertcat(rows{:});
end
function r=blank_local(label,seed)
r=struct('method',string(label),'eval_seed',double(seed),'completed',0,'exception',1, ...
    'attempt_started',0,'attempt_wall_s',NaN,'demo_wall_s',NaN, ...
    'exception_wall_s',NaN,'metric_wall_s',NaN, ...
    'success',0,'physical_success',0,'reached',0,'accepted',0,'hard_pass',0, ...
    'time_contract_available',0,'time_contract_pass',0,'t_reach',NaN,'duration_s',NaN, ...
    'R_soft',NaN,'R_soft_integral_s',NaN,'Pavg_peak_W',NaN,'Pgrid_peak_W',NaN,'Egrid_J',NaN, ...
    'tolerance_W',0,'exceed_duration_s',NaN,'exceed_fraction',NaN,'exceed_energy_J',NaN, ...
    'max_qd_ratio',NaN,'max_qdd_ratio',NaN,'max_tau_ratio',NaN, ...
    'error_id',string(''),'error_message',string(''));
% Keep one common table layout for trajectories and failed PTP searches.
DC=spinn3d_dynamic_bus_summary(struct());fields=fieldnames(DC);
for k=1:numel(fields),r.(fields{k})=DC.(fields{k});end
r.bus_model_scope="unavailable";
Paper=spinn3d_revision_manuscript_fields();fields=fieldnames(Paper);
for k=1:numel(fields),r.(fields{k})=Paper.(fields{k});end
end
function cfg=attach_local(cfg,out,M)
F=cfg.FAIRNESS;F.noguard_reference_available=true;F.t_reach_noguard_s=M.t_reach;
F.reference_source='paired-full-plant';

A=out.execution_audit;
if isfield(A,'Pavg_replay_authoritative')&&logical(A.Pavg_replay_authoritative)
    F.noguard_soft_reference_available=true;
    F.noguard_soft_excess_sq_mean=A.Pgrid_avg_excess_sq_norm_mean;
    F.noguard_soft_excess_sq_integral_s=A.Pgrid_avg_excess_sq_norm_integral_s;
    F.noguard_soft_budget_W=A.Pgrid_avg_budget_W;
    F.noguard_soft_reference_source=A.Pavg_replay_schema;
    F.reference_source='paired-full-plant-with-authoritative-soft-power';
else
    F.noguard_soft_reference_available=false;
end
cfg.FAIRNESS=F;
end
