function M=spinn3d_revision_filter_metrics(out,Ptarget,window_s,tolerance_W,reanalyse)
% Recalculate causal filtered-power metrics on a saved physical trajectory.

if nargin<5,reanalyse=true;end

original_out=out;
if reanalyse
    t=double(out.log.t(:));P=double(out.log.Pgrid(:));dt=[t(1);diff(t)];

    A=out.execution_audit;

    prev=double(A.Pgrid_avg_initial_replay_W);Pav=zeros(size(P));
    for k=1:numel(t),b=exp(-dt(k)/window_s);prev=b*prev+(1-b)*P(k);Pav(k)=prev;end
    out.log.Pgrid_avg=Pav;out.execution_audit.Pavg_replay_authoritative=false;
end
M=spinn3d_revision_metrics_from_out(out,Ptarget,tolerance_W);
PB=spinn3d_params_block();report=PB.paper_eval;
report.manuscript_margin_frac=tolerance_W/Ptarget;
if reanalyse,report.filter_reanalysis_window_s=window_s;end
Paper=spinn3d_revision_manuscript_fields(original_out,Ptarget,report);
fields=fieldnames(Paper);for k=1:numel(fields),M.(fields{k})=Paper.(fields{k});end
end
