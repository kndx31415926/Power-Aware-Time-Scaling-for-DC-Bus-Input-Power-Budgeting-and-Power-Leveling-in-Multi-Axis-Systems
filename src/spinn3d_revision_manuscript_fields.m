function M=spinn3d_revision_manuscript_fields(out,Pbudget,opts)
%SPINN3D_REVISION_MANUSCRIPT_FIELDS Stable supplement success/failure schema.
% All outputs are scalar numeric/logical/string values suitable for tables.
if nargin<1,out=struct();end
if nargin<2,Pbudget=[];end
if nargin<3||isempty(opts)
    PB=spinn3d_params_block();opts=PB.paper_eval;
end
R=spinn3d_paper_metric_registry();R=R(strcmp({R.group},'manuscript'));
M=struct();for k=1:numel(R),M.(R(k).name)=NaN;end
M.manuscript_metrics_requested=0;
M.manuscript_window_start_s=NaN;M.manuscript_window_end_s=NaN;
M.manuscript_window_intervals=0;M.manuscript_window_boundary_valid=0;
M.manuscript_margin_frac=double(opts.manuscript_margin_frac);
M.manuscript_budget_W=NaN;M.manuscript_threshold_W=NaN;
M.manuscript_filter_window_s=NaN;
M.manuscript_filter_source="unavailable";M.manuscript_filter_scope="unavailable";
M.manuscript_quality_available=0;M.manuscript_quality_complete=0;
M.manuscript_quality_reason="no_method_output";
M.manuscript_metric_available_json="{}";M.manuscript_metric_reason_json="{}";
M.manuscript_metadata_json="{}";
M.manuscript_metric_schema=string(opts.manuscript_metric_schema);
if ~isnumeric(Pbudget)||~isscalar(Pbudget)||~isfinite(Pbudget)||Pbudget<=0,return;end
if ~isstruct(out)||~isfield(out,'log'),return;end
[values,Q]=spinn3d_manuscript_metrics(out,Pbudget,opts);
fields=fieldnames(values);for k=1:numel(fields),M.(fields{k})=values.(fields{k});end

A=struct();if isfield(out,'execution_audit'),A=out.execution_audit;end
M.manuscript_metrics_requested=double(isstruct(A)&&isfield(A,'Pavg_replay_schema'));
if isfield(out,'cfg')&&isstruct(out.cfg)&&isfield(out.cfg,'BUS')&& ...
        isstruct(out.cfg.BUS)&&isfield(out.cfg.BUS,'model_scope')
    M.manuscript_metrics_requested=max(M.manuscript_metrics_requested, ...
        double(strcmp(char(out.cfg.BUS.model_scope),'dynamic_capacitor_common_dc_v1')));
end
M.manuscript_window_start_s=Q.window_start_s;M.manuscript_window_end_s=Q.window_end_s;
M.manuscript_window_intervals=Q.window_intervals;M.manuscript_window_boundary_valid=double(Q.window_boundary_valid);
M.manuscript_budget_W=Q.Pbud_W;M.manuscript_threshold_W=Q.excess_threshold_W;
M.manuscript_filter_window_s=Q.filter_window_s;M.manuscript_filter_source=string(Q.filter_source);
M.manuscript_filter_scope=string(Q.filter_scope);
M.manuscript_quality_available=double(Q.available);M.manuscript_quality_complete=double(Q.complete);
M.manuscript_quality_reason=string(Q.reason);
M.manuscript_metric_available_json=string(jsonencode(Q.metric_available));
M.manuscript_metric_reason_json=string(jsonencode(Q.metric_reason));
M.manuscript_metadata_json=string(jsonencode(Q));
end
