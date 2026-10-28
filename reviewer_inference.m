function T=reviewer_inference(root)
% Batch-one CPU network inference on the common 1000 validation windows.
% Feature preparation precedes timing; twenty calls warm up each network.
outdir=fullfile(root,'results','inference');
if ~isfolder(outdir),mkdir(outdir);end
summary_file=fullfile(outdir,'summary.mat');
if isfile(summary_file),S=load(summary_file,'T');T=S.T;return;end
S=load(fullfile(root,'inputs','validation_windows.mat'));
if isfield(S,'Xseq'),Xseq=S.Xseq;else,Xseq=S.V.Xseq;end
names={'trf_phy.mat','mlp_phy.mat'};labels={'TRF-PRG','MLP-PRG'};
architecture={'transformer','mlp'};T=table();Samples=table();
for j=1:2
    M=load_alpha_model(fullfile(root,'models','r8_eq',names{j}),'','final',architecture{j});
    M.net=dlupdate(@gather,M.net);
    inputs=spinn3d_r6_runtime_model_inputs(Xseq,M.preproc);
    y=predict(M.net,inputs{1});gather(extractdata(y));
    for k=1:20,y=predict(M.net,inputs{k});gather(extractdata(y));end
    milliseconds=zeros(numel(inputs),1);
    for k=1:numel(inputs)
        timer=tic;y=predict(M.net,inputs{k});gather(extractdata(y));
        milliseconds(k)=1e3*toc(timer);
    end
    row=table(string(labels{j}),numel(inputs),mean(milliseconds),median(milliseconds), ...
        prctile(milliseconds,95),prctile(milliseconds,99),max(milliseconds), ...
        'VariableNames',{'method','N','mean_ms','median_ms','p95_ms','p99_ms','max_ms'});
    T=[T;row]; %#ok<AGROW>
    sample=table(repmat(string(labels{j}),numel(inputs),1),(1:numel(inputs)).',milliseconds, ...
        'VariableNames',{'method','window_id','forward_ms'});
    Samples=[Samples;sample]; %#ok<AGROW>
end
temporary=[tempname(outdir) '.mat'];save(temporary,'T','Samples');movefile(temporary,summary_file);
writetable(T,fullfile(outdir,'summary.csv'));
writetable(Samples,fullfile(outdir,'samples.csv'));
end
