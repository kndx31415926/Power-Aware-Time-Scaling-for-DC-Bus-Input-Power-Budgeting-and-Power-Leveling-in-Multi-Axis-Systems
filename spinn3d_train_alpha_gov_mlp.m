function model = spinn3d_train_alpha_gov_mlp(dataset_mat, opts)
%SPINN3D_TRAIN_ALPHA_GOV_MLP  Compact non-attention MLP-PRG learned surrogate.
%
% This trains an MLP on the SAME governor-consistent teacher labels and
% look-ahead token sequences used by the Transformer.  The K-token sequence is
% flattened after the same keep-column and z-score preprocessing.  The saved
% model is compatible with spinn3d_controller_ctcgov through model.type='mlp_flat'.

if nargin < 1 || isempty(dataset_mat)
    PB = spinn3d_params_block();
    dataset_mat = fullfile(PB.paths.root,'spinn3d_alpha_gov_seq_dataset_master.mat');
end
if nargin < 2 || isempty(opts), opts = struct(); end
opts = default_opts(opts);
S = load(dataset_mat);
assert(isfield(S,'Xseq') && isfield(S,'y'), 'dataset must contain Xseq and y');
Xseq = double(S.Xseq);
y = double(S.y(:));
[N,K,Dtot] = size(Xseq);
assert(numel(y)==N, 'y length mismatch');
alpha_min = get_dataset_alpha_min(S, opts);

Dtail = 6;
baseDim = Dtot - Dtail;
assert(baseDim>0, 'bad token dimension');
Xbase_flat_all = reshape(Xseq(:,:,1:baseDim), [], baseDim);
v = var(Xbase_flat_all,0,1);
keepBaseIdx = logical((v > 1e-12) & isfinite(v));
Xbase_keep = Xseq(:,:,keepBaseIdx);
Xtail = Xseq(:,:,baseDim+1:end);
Xc = cat(3, Xbase_keep, Xtail);
Din = size(Xc,3);
Xflat = zeros(N,K*Din);
for i = 1:N
    Xi = squeeze(Xc(i,:,:)); % K x Din
    Xflat(i,:) = reshape(Xi.',1,[]);
end

[Itr,Iva,split_info] = split_indices(S,N,opts);
mu = mean(Xflat(Itr,:),1,'omitnan');
sig = std(Xflat(Itr,:),0,1,'omitnan');
sig(~isfinite(sig) | sig<1e-12) = 1;
Z = (Xflat - mu) ./ sig;
yn = (y - alpha_min) ./ max(1-alpha_min,eps);
yn = min(1,max(0,yn));

layers = [
    featureInputLayer(size(Z,2),'Name','input','Normalization','none')
    fullyConnectedLayer(opts.Hidden1,'Name','fc1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(opts.Hidden2,'Name','fc2')
    reluLayer('Name','relu2')
    fullyConnectedLayer(opts.Hidden3,'Name','fc3')
    reluLayer('Name','relu3')
    fullyConnectedLayer(1,'Name','fc_out')
    sigmoidLayer('Name','sigmoid')
    regressionLayer('Name','regression')];

trainOpts = trainingOptions('adam', ...
    'MaxEpochs',opts.MaxEpochs, ...
    'MiniBatchSize',opts.MiniBatchSize, ...
    'InitialLearnRate',opts.InitialLearnRate, ...
    'Shuffle','every-epoch', ...
    'ValidationData',{single(Z(Iva,:)), single(yn(Iva))}, ...
    'Verbose',opts.Verbose, ...
    'Plots','none');

net = trainNetwork(single(Z(Itr,:)), single(yn(Itr)), layers, trainOpts);
ypn = predict(net, single(Z(Iva,:)));
yp = alpha_min + (1-alpha_min)*double(ypn(:));
yv = y(Iva);

model = struct();
model.type = 'mlp_flat';
model.net = net;
model.preproc = struct();
model.preproc.mu = mu;
model.preproc.sig = sig;
model.preproc.keepBaseIdx = keepBaseIdx;
model.preproc.K = K;
model.preproc.ds = get_dataset_ds(S, opts, K);
model.preproc.kind = 'mlp_flat_lookahead';
model.preproc.flatten = true;
model.preproc.alpha_min = alpha_min;
model.preproc.target_normalized = true;
model.preproc.Ts_servo = get_dataset_Ts_servo(S, opts);
model.preproc.policy_dt = get_dataset_policy_dt(S, opts, model.preproc.Ts_servo);

model.train_info = struct('N_samples',N,'K',K,'Dtot',Dtot,'Din',Din,'Dflat',size(Z,2), ...
    'n_train',numel(Itr),'n_val',numel(Iva),'split',split_info, ...
    'MaxEpochs',opts.MaxEpochs,'MiniBatchSize',opts.MiniBatchSize, ...
    'InitialLearnRate',opts.InitialLearnRate,'Hidden',[opts.Hidden1 opts.Hidden2 opts.Hidden3]);
model.val_metrics = struct('MAE',mean(abs(yp-yv),'omitnan'), ...
    'RMSE',sqrt(mean((yp-yv).^2,'omitnan')), 'MaxAE',max(abs(yp-yv)), 'N',numel(yv));

if ~isempty(opts.save_model)
    save_model = opts.save_model; %#ok<NASGU>
    ensure_dir(fileparts(opts.save_model));
    save(opts.save_model,'model','-v7.3');
    if opts.Verbose, fprintf('[MLP-PRG] saved model: %s\n', opts.save_model); end
end
end


function a = get_dataset_alpha_min(S, opts)
    a = getfield_def(S,'alpha_min',[]);
    if isempty(a) || ~isfinite(double(a))
        if isfield(S,'PB') && isstruct(S.PB) && isfield(S.PB,'alpha') && isfield(S.PB.alpha,'alpha_min') && ~isempty(S.PB.alpha.alpha_min)
            a = double(S.PB.alpha.alpha_min);
        else
            a = getfield_def(opts,'alpha_min',0.15);
        end
    end
    a = double(a);
end

function ds = get_dataset_ds(S, opts, K)
    ds = getfield_def(S,'ds',[]);
    if isempty(ds) || ~isfinite(double(ds))
        ds = [];
        if isfield(S,'PB') && isstruct(S.PB) && isfield(S.PB,'train_trf') && isfield(S.PB.train_trf,'H')
            if K > 1
                ds = double(S.PB.train_trf.H) / (K - 1);
            else
                ds = 0;
            end
        end
        if isempty(ds), ds = getfield_def(opts,'ds',0); end
    end
    ds = double(ds);
end

function Ts = get_dataset_Ts_servo(S, opts)
    Ts = getfield_def(S,'Ts_servo',[]);
    if isempty(Ts) || ~isfinite(double(Ts))
        if isfield(S,'PB') && isstruct(S.PB) && isfield(S.PB,'time') && isfield(S.PB.time,'Ts') && ~isempty(S.PB.time.Ts)
            Ts = double(S.PB.time.Ts);
        else
            Ts = getfield_def(opts,'Ts_servo',NaN);
        end
    end
    Ts = double(Ts);
end

function dt = get_dataset_policy_dt(S, opts, Ts)
    dt = getfield_def(S,'policy_dt',[]);
    if isempty(dt) || ~isfinite(double(dt))
        dt = [];
        if isfield(S,'PB') && isstruct(S.PB)
            if isfield(S.PB,'policy') && isfield(S.PB.policy,'dt') && ~isempty(S.PB.policy.dt)
                dt = double(S.PB.policy.dt);
            elseif isfield(S.PB,'policy') && isfield(S.PB.policy,'N_hold') && ~isempty(S.PB.policy.N_hold) && isfinite(Ts)
                dt = double(S.PB.policy.N_hold) * double(Ts);
            end
        end
        if isempty(dt), dt = getfield_def(opts,'policy_dt',NaN); end
    end
    dt = double(dt);
end

function opts = default_opts(opts)
def = struct('MaxEpochs',80,'MiniBatchSize',512,'InitialLearnRate',1e-3, ...
    'Hidden1',512,'Hidden2',256,'Hidden3',64,'Verbose',true, ...
    'save_model','','alpha_min',0.15,'train_fraction',0.80,'seed',0);
if isfield(opts,'SaveModel') && ~isfield(opts,'save_model'), opts.save_model = opts.SaveModel; end
f = fieldnames(def);
for i=1:numel(f)
    if ~isfield(opts,f{i}) || isempty(opts.(f{i})), opts.(f{i}) = def.(f{i}); end
end
if isempty(opts.save_model)
    try
        PB = spinn3d_params_block();
        opts.save_model = fullfile(PB.paths.root,'model_alpha_gov_mlp.mat');
    catch
        opts.save_model = fullfile(pwd,'data_alpha','model_alpha_gov_mlp.mat');
    end
end
end

function [Itr,Iva,split_info] = split_indices(S,N,opts)
rng(opts.seed);
split_info = struct('strategy','sample_level','train_fraction',opts.train_fraction);
if isfield(S,'traj_id') && ~isempty(S.traj_id) && numel(S.traj_id)==N
    tids = unique(double(S.traj_id(:)));
    tids = tids(randperm(numel(tids)));
    nTr = max(1,round(opts.train_fraction*numel(tids)));
    trT = tids(1:nTr);
    vaT = tids(nTr+1:end);
    Itr = find(ismember(double(S.traj_id(:)),trT));
    Iva = find(ismember(double(S.traj_id(:)),vaT));
    split_info.strategy = 'trajectory_level';
    split_info.n_train_traj = numel(trT);
    split_info.n_val_traj = numel(vaT);
    if isempty(Iva)
        idx = randperm(N); nTrS = max(1,round(0.90*N));
        Itr = idx(1:nTrS); Iva = idx(nTrS+1:end);
        split_info.strategy = 'sample_level_fallback_empty_val';
    end
else
    idx = randperm(N); nTrS = max(1,round(0.90*N));
    Itr = idx(1:nTrS); Iva = idx(nTrS+1:end);
end
end

function x = getfield_def(S,fn,def)
if isstruct(S) && isfield(S,fn) && ~isempty(S.(fn)), x = S.(fn); else, x = def; end
end
function ensure_dir(d), if ~isempty(d) && exist(d,'dir')~=7, mkdir(d); end, end
