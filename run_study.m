function T=run_study(study,task_ids)
% RUN_STUDY Reproduce one study, or selected task IDs within a study.
% Studies: main, ablation, robustness, parameter, filter, inference.
% The main run includes both supervised controls, sharing the other methods
% with the learning ablation. Every method writes its own trajectory MAT.
if nargin<1,study='main';end
if nargin<2,task_ids=[];end
root=fileparts(mfilename('fullpath'));addpath(fullfile(root,'src'));
study=lower(char(study));
if strcmp(study,'ablation'),study='main';end
if strcmp(study,'filter'),T=reviewer_filter(root,task_ids);return;end
if strcmp(study,'inference'),T=reviewer_inference(root);return;end
S=load(fullfile(root,'inputs','tasks.mat'),'bank');bank=S.bank;
if strcmp(study,'main'),entries=bank.ptp;else,entries=bank.(study);end
if isempty(task_ids),task_ids=1:numel(entries);end
C=reviewer_conditions(study);output_root=fullfile(root,'results',study);
if ~isfolder(output_root),mkdir(output_root);end
if ~strcmp(study,'main'),writetable(struct2table(C),fullfile(output_root,'conditions.csv'));end
[named,labels]=spinn3d_revision_prg_methods();
if strcmp(study,'main')
    named=[named,{@spinn3d_demo_ctcgov_online_nominal_alpha1, ...
        @spinn3d_demo_ctcgov_online_mpc_rg_grid_r2,@spinn3d_demo_ctcgov_online_bus_ptp_dp, ...
        @trf_supervised,@mlp_supervised}];
    labels=[labels,{'Baseline','Nominal-RG-grid','Bus-PTP-beam','TRF-supervised','MLP-supervised'}];
end
for task_id=task_ids(:).'
    entry=entries{task_id};
    for case_id=1:numel(C)
        folder=fullfile(output_root,sprintf('task_%03d',task_id));
        if ~strcmp(study,'main'),folder=fullfile(folder,sprintf('case_%02d',case_id));end
        if ~isfolder(folder),mkdir(folder);end
        summary_file=fullfile(folder,'summary.mat');
        if isfile(summary_file),continue;end
        cfg=reviewer_configuration(root,entry,study,case_id);
        callbacks=cell(size(named));
        for j=1:numel(named)
            file=fullfile(folder,[matlab.lang.makeValidName(labels{j}) '.mat']);
            callbacks{j}=saved_callback(named{j},file,labels{j},entry.seed);
        end
        [T,Outputs]=spinn3d_revision_run_methods(cfg,entry.seed,callbacks,labels, ...
            struct('enabled',false,'close_figs',true)); %#ok<ASGLU>
        timing=intersect(T.Properties.VariableNames, ...
            {'attempt_wall_s','demo_wall_s','exception_wall_s','metric_wall_s'});
        T(:,timing)=[];
        T.study=repmat(string(study),height(T),1);
        T.task_id=repmat(task_id,height(T),1);T.case_id=repmat(case_id,height(T),1);
        T.task_seed=repmat(entry.task_seed,height(T),1);
        T.payload_mass_kg=repmat(cfg.PAYLOAD.DEMO.mass,height(T),1);
        T.case_name=repmat(string(C(case_id).name),height(T),1);
        if strcmp(study,'main')
            manifest=readtable(fullfile(root,'inputs','final_test_manifest.csv'),'TextType','string');
            row=manifest(manifest.task_id==task_id,:);
            T.payload_group=repmat(row.payload_group,height(T),1);
            T.template_name=repmat(row.template_name,height(T),1);
        end
        temporary=[tempname(folder) '.mat'];save(temporary,'T');movefile(temporary,summary_file);
        writetable(T,fullfile(folder,'summary.csv'));
        fprintf('%s: task %d/%d, condition %d/%d completed.\n',study,task_id,numel(entries),case_id,numel(C));
        clear Outputs;
    end
end
files=dir(fullfile(output_root,'**','summary.mat'));T=table();
for k=1:numel(files)
    Q=load(fullfile(files(k).folder,files(k).name),'T');T=[T;Q.T]; %#ok<AGROW>
end
T=sortrows(T,{'task_id','case_id','method'});
save(fullfile(output_root,'trials.mat'),'T');writetable(T,fullfile(output_root,'trials.csv'));
end

function callback=saved_callback(named,file,label,seed)
callback=@(cfg)run_saved(named,cfg,file,label,seed);
end

function out=run_saved(named,cfg,file,label,seed)
if isfile(file)
    Q=load(file,'Result');Result=Q.Result;
else
    Result=struct('method',label,'seed',seed,'out',[], ...
        'error',struct('identifier','','message',''));
    try
        Result.out=named(cfg);
    catch ME
        % A bounded PTP search with no admissible path is an experimental outcome.
        if ~strcmp(ME.identifier,'spinn3d_bus_ptp_dp_r2:NoPath'),rethrow(ME);end
        Result.error=struct('identifier',ME.identifier,'message',ME.message);
    end
    temporary=[tempname(fileparts(file)) '.mat'];
    save(temporary,'Result','-v7.3');movefile(temporary,file);
end
if ~isempty(Result.error.message)
    id=Result.error.identifier;if isempty(id),id='simulation:MethodError';end
    error(id,'%s',Result.error.message);
end
out=Result.out;
end

function out=trf_supervised(cfg)
cfg.MODEL_PATH_ALPHA=cfg.MODEL_PATH_ALPHA_SUPERVISED;cfg.MODEL_PATH=cfg.MODEL_PATH_ALPHA_SUPERVISED;
cfg.ALPHA_EXPECTED_MODEL_ROLE='supervised';cfg.ALPHA_EXPECTED_ARCHITECTURE='transformer';
if isfield(cfg,'MODEL_SHA256'),cfg=rmfield(cfg,'MODEL_SHA256');end
out=spinn3d_demo_ctcgov_online_nn(cfg);
end

function out=mlp_supervised(cfg)
cfg.MODEL_PATH_ALPHA_MLP=cfg.MODEL_PATH_ALPHA_MLP_SUPERVISED;
cfg.MODEL_PATH_ALPHA=cfg.MODEL_PATH_ALPHA_MLP_SUPERVISED;cfg.MODEL_PATH=cfg.MODEL_PATH_ALPHA_MLP_SUPERVISED;
cfg.ALPHA_EXPECTED_MODEL_ROLE='supervised';cfg.ALPHA_EXPECTED_ARCHITECTURE='mlp';
if isfield(cfg,'MODEL_SHA256'),cfg=rmfield(cfg,'MODEL_SHA256');end
out=spinn3d_demo_ctcgov_online_mlp(cfg);
end
