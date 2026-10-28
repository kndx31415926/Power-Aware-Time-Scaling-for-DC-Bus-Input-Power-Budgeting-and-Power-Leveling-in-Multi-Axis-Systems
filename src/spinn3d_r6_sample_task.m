function [cfg,meta]=spinn3d_r6_sample_task(PB,seed,index,role)
% Deterministic predeclared family; final outcomes never choose the family.
% Restore caller RNG so this draw cannot perturb controller/noise streams.

old=rng; cleanup=onCleanup(@() rng(old)); %#ok<NASGU>
rng(double(seed),'twister');
bank=PB.revision.task_domain.templates;
training_role=strcmpi(char(role),'training');
if training_role
    stratum=spinn3d_training_stratum(PB,index);
    j=stratum.template_index;
else
    j=1+mod(double(index)-1,numel(bank));
end
T=bank(j);
cfg=spinn3d_params_to_cfg(PB,'demo_alpha');
cfg.Q0_DEG=double(T.q0_deg(:))+(-1+2*rand(4,1))*double(PB.training.q0_jitter_deg);
cfg.QF_DEG=double(T.qf_deg(:))+(-1+2*rand(4,1))*double(PB.training.qf_jitter_deg);
pr=PB.payload.range;
if training_role, mass_bounds=stratum.mass_bounds_kg; else, mass_bounds=pr.mass; end
m=mass_bounds(1)+diff(mass_bounds)*rand;
com=pr.com(:,1).'+diff(pr.com,1,2).'.*rand(1,3);
d=pr.box_dims(:,1).'+diff(pr.box_dims,1,2).'.*rand(1,3);
I=(m/12)*[d(2)^2+d(3)^2,d(1)^2+d(3)^2,d(1)^2+d(2)^2,0,0,0];
cfg.PAYLOAD.DEMO=struct('mass',m,'com',com,'inertia',I,'about','com','mode','replace');
cfg.DISABLE_PLOT=true;
meta=struct('task_schema',PB.revision.task_domain.schema, ...
    'template_index',j,'template_name',char(T.name),'role',char(role), ...
    'seed',double(seed),'index',double(index),'q0_deg',cfg.Q0_DEG, ...
    'qf_deg',cfg.QF_DEG);
if training_role
    meta.sampling_schema=stratum.schema;
    meta.stratum_index=stratum.stratum_index;
    meta.payload_group=stratum.payload_group;
    meta.mass_bounds_kg=stratum.mass_bounds_kg;
end
end
