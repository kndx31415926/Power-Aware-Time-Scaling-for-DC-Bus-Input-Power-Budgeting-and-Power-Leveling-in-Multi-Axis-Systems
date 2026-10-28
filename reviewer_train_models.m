function reviewer_train_models()
% Generate 300 teacher trajectories and train both architectures and losses.
root=fileparts(mfilename('fullpath'));
addpath(fullfile(root,'src'));
previous=getenv('POWER_TIMING_TRAIN_ROOT');
restore=onCleanup(@()restore_local(previous));
setenv('POWER_TIMING_TRAIN_ROOT',fullfile(root,'results','training'));
clear spinn3d_params_block;
architectures={'transformer','mlp','transformer','mlp'};
variants={'physics','physics','supervised','supervised'};
PB=spinn3d_params_block();
paths={PB.paths.model_alpha,PB.paths.model_alpha_mlp, ...
    PB.paths.model_alpha_supervised,PB.paths.model_alpha_mlp_supervised};
for k=1:4
    if isfile(paths{k}),continue;end
    options=struct('stage','final','architecture',architectures{k}, ...
        'variant',variants{k},'existing_corpus_only',k>1);
    spinn3d_run_alpha_gov_budget_trf(options);
end
end
function restore_local(value)
setenv('POWER_TIMING_TRAIN_ROOT',value);
clear spinn3d_params_block;
end
