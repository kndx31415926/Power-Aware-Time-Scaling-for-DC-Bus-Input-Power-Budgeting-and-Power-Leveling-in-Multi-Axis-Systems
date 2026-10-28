function PB=spinn3d_params_block()
% SPINN3D_PARAMS_BLOCK Parameters used for the reported simulations.
% Numerical values are stored in inputs/parameters.mat. Training writes
% its generated data and models beneath results/training.
root=fileparts(fileparts(mfilename('fullpath')));
S=load(fullfile(root,'inputs','parameters.mat'),'PB');PB=S.PB;
target=root;
training_root=getenv('POWER_TIMING_TRAIN_ROOT');
if ~isempty(training_root),target=training_root;end
PB=rebind(PB,target);
PB.paths.project_root=target;
end
function x=rebind(x,root)
if isstruct(x)
    names=fieldnames(x);
    for k=1:numel(x)
        for j=1:numel(names),x(k).(names{j})=rebind(x(k).(names{j}),root);end
    end
elseif iscell(x)
    for k=1:numel(x),x{k}=rebind(x{k},root);end
elseif ischar(x)
    for drive={'C','D','E'}
        for suffix={'A','B'}
            prefix=[drive{1} ':\MathSource' suffix{1}];
            x=strrep(x,prefix,root);x=strrep(x,strrep(prefix,'\','/'),strrep(root,'\','/'));
        end
    end
elseif isstring(x)
    for k=1:numel(x),x(k)=string(rebind(char(x(k)),root));end
elseif isa(x,'function_handle')
    x=str2func(func2str(x));
end
end
