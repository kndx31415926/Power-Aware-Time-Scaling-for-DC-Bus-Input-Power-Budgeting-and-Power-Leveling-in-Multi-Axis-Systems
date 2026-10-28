function M=spinn3d_revision_metrics_from_out(out,Ptarget,tolerance_W)
% Calculate motion, source-power and energy measures from a trajectory.

if nargin<3,tolerance_W=0;end

t=double(out.log.t(:));P=double(out.log.Pgrid(:));dt=[t(1);diff(t)];

A=getf(out,'execution_audit',struct());
if logical(getf(A,'Pavg_replay_authoritative',false))&&isfield(A,'Pgrid_avg_replay_W')
    Pav=double(A.Pgrid_avg_replay_W(:));
else

    Pav=double(out.log.Pgrid_avg(:));
end

M=struct();M.t_reach=double(getf(out,'t_reach',NaN));M.reached=double(getf(out,'reached',false));
M.accepted=double(getf(out,'accepted',false));M.hard_pass=double(getf(A,'pass',false));
F=getf(out,'fairness',struct());M.time_contract_available=double(getf(F,'reference_available',true));
M.time_contract_pass=double(getf(out,'time_contract_pass',getf(F,'pass',M.accepted)));
M.physical_success=double(M.reached==1&&M.hard_pass==1);
M.success=double(M.physical_success==1&&M.accepted==1);
M.duration_s=sum(dt);M.Pgrid_peak_W=max(P);M.Pavg_peak_W=max(Pav);
e=max(Pav/Ptarget-1,0);M.R_soft=sum(dt.*e.^2)/M.duration_s;
M.R_soft_integral_s=sum(dt.*e.^2);M.Egrid_J=sum(dt.*P);
DC=spinn3d_dynamic_bus_summary(out.log);dc_fields=fieldnames(DC);
for k=1:numel(dc_fields),M.(dc_fields{k})=DC.(dc_fields{k});end
M.bus_model_scope="unavailable";
if isfield(out,'cfg') && isstruct(out.cfg) && isfield(out.cfg,'BUS') && isstruct(out.cfg.BUS)
    M.bus_model_scope=string(getf(out.cfg.BUS,'model_scope','unavailable'));
end
M.tolerance_W=double(tolerance_W);M.exceed_duration_s=sum(dt.*double(Pav>Ptarget+tolerance_W));
M.exceed_fraction=M.exceed_duration_s/M.duration_s;
M.exceed_energy_J=sum(dt.*max(Pav-Ptarget-tolerance_W,0));
M.max_qd_ratio=double(getf(A,'qd_ratio',getf(A,'max_qd_ratio',NaN)));M.max_qdd_ratio=double(getf(A,'qdd_ratio',getf(A,'max_qdd_ratio',NaN)));
M.max_tau_ratio=double(getf(A,'tau_ratio',getf(A,'max_tau_ratio',NaN)));
Paper=spinn3d_revision_manuscript_fields(out,Ptarget);
fields=fieldnames(Paper);for k=1:numel(fields),M.(fields{k})=Paper.(fields{k});end
end
function v=getf(S,f,d),if isstruct(S)&&isfield(S,f)&&~isempty(S.(f)),v=S.(f);else,v=d;end,end
