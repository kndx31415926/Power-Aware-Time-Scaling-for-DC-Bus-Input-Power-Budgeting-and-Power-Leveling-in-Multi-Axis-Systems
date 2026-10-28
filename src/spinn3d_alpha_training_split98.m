function [Itr,Iva,S]=spinn3d_alpha_training_split98(DS,validation_fraction,seed)
%SPINN3D_ALPHA_TRAINING_SPLIT98 Leakage-free whole-trajectory split.
% New equilibrium corpora split independently inside all 4 load x 3 template
% strata. Historical corpora retain their original split for inspection.

N=numel(DS.y);
rng(seed,'twister');

tid = double(DS.traj_id(:));

uTid = unique(tid, 'stable');

new_split=isfield(DS,'training_split_schema') && strcmp( ...
    char(DS.training_split_schema),'trajectory_stratified_payload_template_v1');
if new_split
    [trTid,vaTid,strata]=stratified_local(DS,uTid,validation_fraction);
else
    uTid = uTid(randperm(numel(uTid)));
    nTrT = max(1, round((1-validation_fraction)*numel(uTid)));
    if nTrT >= numel(uTid), nTrT = numel(uTid)-1; end
    trTid = uTid(1:nTrT);
    vaTid = uTid(nTrT+1:end);
end
Itr = find(ismember(tid, trTid)).';
Iva = find(ismember(tid, vaTid)).';
S = struct('type','trajectory', ...
    'nTrainTraj',numel(trTid),'nValTraj',numel(vaTid), ...
    'train_trajectory_ids',trTid(:).','validation_trajectory_ids',vaTid(:).', ...
    'seed',seed);
if new_split
    S.schema=char(DS.training_split_schema);
    S.strata=strata;
end
end

function [trTid,vaTid,strata]=stratified_local(DS,uTid,fraction)

metaIds=double([DS.meta.traj_id]);

stratumIds=zeros(size(metaIds));
for k=1:numel(metaIds)
    M=DS.meta(k); T=M.task_meta;

    b=double(T.payload_group); t=double(T.template_index);
    bounds=double(T.mass_bounds_kg(:).'); mass=double(M.payload.mass);

    stratumIds(k)=double(T.stratum_index);
end
trTid=zeros(0,1); vaTid=zeros(0,1);
strata=repmat(struct('stratum_index',0,'payload_group',0,'template_index',0, ...
    'nTraj',0,'nTrainTraj',0,'nValTraj',0),12,1);
counts=accumarray(stratumIds(:),1,[12 1]);

for s=1:12
    ids=metaIds(stratumIds==s); ids=ids(randperm(numel(ids)));
    nv=max(1,round(fraction*numel(ids))); nv=min(nv,numel(ids)-1);
    vaTid=[vaTid;ids(1:nv).']; %#ok<AGROW>
    trTid=[trTid;ids(nv+1:end).']; %#ok<AGROW>
    strata(s)=struct('stratum_index',s,'payload_group',1+floor((s-1)/3), ...
        'template_index',1+mod(s-1,3),'nTraj',numel(ids), ...
        'nTrainTraj',numel(ids)-nv,'nValTraj',nv);
end
end
