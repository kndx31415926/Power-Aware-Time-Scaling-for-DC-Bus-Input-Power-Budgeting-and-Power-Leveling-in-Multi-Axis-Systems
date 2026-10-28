function M=load_alpha_model(model_path,varargin)
% LOAD_ALPHA_MODEL Read a trained policy and its feature normalization.
% The supplied MAT files contain model.net and model.preproc. The same
% feature mask and normalization are applied in training and inference.
persistent paths models
if isempty(paths),paths={};models={};end
k=find(strcmp(paths,char(model_path)),1);
if ~isempty(k),M=models{k};return;end
S=load(model_path,'model');M=S.model;
M.preproc.mu=double(M.preproc.mu(:)).';
M.preproc.sig=double(M.preproc.sig(:)).';
if isfield(M.preproc,'keepBaseIdx') && ~isempty(M.preproc.keepBaseIdx)
    M.preproc.keepBaseIdx=logical(M.preproc.keepBaseIdx(:)).';
elseif isfield(M.preproc,'keepBaseIdx')
    M.preproc=rmfield(M.preproc,'keepBaseIdx');
end
paths{end+1}=char(model_path);models{end+1}=M;
end
