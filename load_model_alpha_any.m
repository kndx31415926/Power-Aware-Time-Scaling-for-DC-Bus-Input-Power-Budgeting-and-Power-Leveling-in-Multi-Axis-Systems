function M = load_model_alpha_any(model_path)
%LOAD_MODEL_ALPHA_ANY Load TRF or MLP alpha-governor model.
assert(exist(model_path,'file')==2, 'Model file not found: %s', model_path);
S = load(model_path);
if isfield(S,'model')
    M = S.model;
elseif isfield(S,'M')
    M = S.M;
else
    error('No variable named model or M in %s', model_path);
end
assert(isfield(M,'net') && isfield(M,'preproc'), 'model must contain net and preproc');
if ~isfield(M,'type')
    M.type = getfield_def(M.preproc,'kind','trf_lookahead');
end
end
function x = getfield_def(S,fn,def)
if isstruct(S) && isfield(S,fn) && ~isempty(S.(fn)), x = S.(fn); else, x = def; end
end
