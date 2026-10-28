function T=reviewer_filter(root,task_ids)
% Recalculate power metrics on the saved main-comparison trajectories.
% The initial holding power and original source-power trace are retained.
if nargin<2,task_ids=[];end
main_root=fullfile(root,'results','main');outdir=fullfile(root,'results','filter');
if ~isfolder(outdir),mkdir(outdir);end
if isempty(task_ids)
    summaries=dir(fullfile(main_root,'task_*','summary.mat'));task_ids=zeros(1,numel(summaries));
    for k=1:numel(summaries)
        Q=load(fullfile(summaries(k).folder,summaries(k).name),'T');task_ids(k)=Q.T.task_id(1);
    end
end
windows=[.025 .05 .1 .2 .5];tolerances=[0 .025 .05 .1];
labels={'NoGuard','Feasible-RG','TRF-PRG','MLP-PRG','Baseline','Nominal-RG-grid','Bus-PTP-beam'};
for task_id=task_ids(:).'
    target=fullfile(outdir,sprintf('task_%03d.mat',task_id));
    if isfile(target),continue;end
    folder=fullfile(main_root,sprintf('task_%03d',task_id));
    S=load(fullfile(folder,'summary.mat'),'T');T=table();
    for j=1:numel(labels)
        row=S.T(string(S.T.method)==string(labels{j}),:);
        Q=load(fullfile(folder,[matlab.lang.makeValidName(labels{j}) '.mat']),'Result');
        out=Q.Result.out;
        for window_s=windows
            for tolerance_frac=tolerances
                W=row;W.metric_valid=0;
                if ~isempty(out)
                    M=spinn3d_revision_filter_metrics(out,10.5,window_s,10.5*tolerance_frac,true);
                    fields=fieldnames(M);
                    for fi=1:numel(fields),W.(fields{fi})(1)=M.(fields{fi});end
                    W.metric_valid=1;
                end
                W.study="filter";W.experiment="fixed_logs";
                W.window_s=window_s;W.feedback_window_s=.05;
                W.tolerance_frac=tolerance_frac;W.tolerance_W=10.5*tolerance_frac;
                T=[T;W]; %#ok<AGROW>
            end
        end
    end
    temporary=[tempname(outdir) '.mat'];save(temporary,'T');movefile(temporary,target);
    fprintf('Filter metrics: task %d complete.\n',task_id);
end
files=dir(fullfile(outdir,'task_*.mat'));T=table();
for k=1:numel(files)
    Q=load(fullfile(files(k).folder,files(k).name),'T');T=[T;Q.T]; %#ok<AGROW>
end
save(fullfile(outdir,'trials.mat'),'T');writetable(T,fullfile(outdir,'trials.csv'));
end
