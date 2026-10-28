function model = spinn3d_train_alpha_gov_trf(DS, out_path, opts)
% Train a progress-rate policy using trajectory-level data splits.
% The objective combines teacher supervision and the selected physical loss.
% Feature normalization is estimated from training trajectories.

	if nargin < 2 || isempty(out_path)
        error('spinn3d_train_alpha_gov_trf:MissingCentralModelPath', ...
            'The versioned Alpha model path must come from PB.paths.');
	end

	if nargin < 3 || isempty(opts)
        error('spinn3d_train_alpha_gov_trf:MissingCentralTrainOptions', ...
            ['Training options must be assembled from PB.train_trf by ', ...
             'spinn3d_run_alpha_gov_budget_trf.']);
	end
    required_opts = {'MaxEpochs','MiniBatchSize','InitialLearnRate', ...
        'LambdaOver','LambdaUnder','LambdaPINN','LambdaSpeed', ...
        'LambdaPavgSq','SoftPavgLossSchema', ...
        'LambdaEnd','EndPhase','EndWidth','PinnTokenDecay', ...
        'ValidationFraction','SplitSeed','DModel','NumHeads','FFNDim', ...
        'NumLayers','Dropout','HeadWidth','Activation','PINNWeights', ...
        'CAPS','FRIC','Verbose','ExecutionEnvironment', ...
        'SequenceCacheBudgetGiB','ModelRole','Architecture', ...
        'MLPHiddenWidths','MLPActivation','InitializationSeed','BatchOrderSeed'};

    opts.ValidationFraction = double(opts.ValidationFraction);

    opts.SplitSeed = double(opts.SplitSeed);

    cache_budget_gib = opts.SequenceCacheBudgetGiB;

    opts.SequenceCacheBudgetGiB = double(cache_budget_gib);

    opts.ModelRole = char(opts.ModelRole);

    opts.Architecture=lower(char(opts.Architecture));

    for seedName={'InitializationSeed','BatchOrderSeed'}
        seedValue=opts.(seedName{1});

    end
    opts.Activation = lower(char(opts.Activation));

    
	opts.SoftPavgLossSchema = char(opts.SoftPavgLossSchema);

    
	opts.Verbose = logical(opts.Verbose);
    initial_model_path='';
    fine_tune_signature=struct();
	% ---- GPU / CPU execution ----
	[useGPU, execEnv] = spinn3d_gpu_execution_env(opts, opts.Verbose, ['alpha-',opts.Architecture]);

	% ---- basic validation ----

	

    
    [alpha_min,alpha_max] = resolve_alpha_bounds_local(DS);
    alpha_output_schema = 'physical_alpha_scaled_sigmoid_v1';
    learningContract=spinn3d_alpha_learning_contract();
    % Authenticate the three independently persisted schema copies before
    % opening sequence shards or starting any expensive training work.
    alpha_training_schema=char(DS.training_schema);
    centralPB=spinn3d_params_block();

    if strcmp(opts.ModelRole,'alpha_only_supervised_for_ablation')

    else

    end

    % holds as many single-precision shards as the central client-process
    % budget permits; arbitrary shuffled mini-batch rows are indexed only in
    % ordinary RAM.  The cleanup guard releases all shards on every exit path.
    alpha_shard_cache_local('reset');
    alpha_shard_cache_local('configure',opts.SequenceCacheBudgetGiB);
    alpha_cache_guard = onCleanup(@() alpha_shard_cache_local('reset')); %#ok<NASGU>
    source = alpha_sequence_source_local(DS);

    y    = single(DS.y(:));
    if strcmp(source.mode,'chunked')
        corpus_bytes = sum(source.shard_bytes);
        largest_shard_bytes = max(source.shard_bytes);
        budget_bytes = opts.SequenceCacheBudgetGiB*(1024^3);

        if corpus_bytes<=budget_bytes
            cache_mode = 'full-corpus';
        else
            cache_mode = 'LRU';
        end
        if opts.Verbose
            fprintf(['[alpha-trf] sequence cache: budget=%.3f GiB | ', ...
                'corpus=%.3f GiB | shards=%d | mode=%s\n'], ...
                opts.SequenceCacheBudgetGiB,corpus_bytes/(1024^3), ...
                numel(source.manifest),cache_mode);
        end
    end
    assert_finite_xseq_blocked_local(source, 512);
    if strcmp(source.mode,'chunked')
        cache_status = alpha_shard_cache_local('status');

        if corpus_bytes<=budget_bytes

        end
        if opts.Verbose
            fprintf(['[alpha-trf] sequence cache warmed: %d shard(s), ', ...
                '%.3f GiB resident in client CPU RAM.\n'], ...
                cache_status.entry_count,cache_status.total_bytes/(1024^3));
        end
    end
    alpha_tol = single(1e-6*max(1,abs(alpha_max)));

    y = min(max(y,single(alpha_min)),single(alpha_max));

    N = source.N;
    K = source.K;
    Dtot = source.D;

    nJ_schema = infer_nJ_from_caps(opts.CAPS);
    use_bus_schema = logical(getfield_def(DS,'use_bus_state_features',false));
    bus_dim_schema = double(getfield_def(DS,'bus_feature_dim',0));
    if ~use_bus_schema, bus_dim_schema=0; end

    state_layout=spinn3d_alpha_state_feature_layout(nJ_schema);
    expected_Dtot = state_layout.total_dim;

    
    for i0=1:512:N
        block=read_alpha_rows_local(source,i0:min(i0+511,N));

    end

    Dtail = 6;
    baseDim = Dtot - Dtail;

    % ========= train/val split =========

    % adjacent windows from one rollout across train/validation.
    [Itr,Iva,splitInfo]=spinn3d_alpha_training_split98( ...
        DS,opts.ValidationFraction,opts.SplitSeed);

    % ========= training-only feature selection and z-score =========
    % Compute moments in bounded row blocks.  Validation trajectories never
    % participate in feature selection or normalisation.  Only the compact
    % column index and its statistics are retained; selected/normalised
    % tensors are formed for one mini-batch at a time below.
    if isempty(initial_model_path)
        [sumAll, sumSqAll, nTokenTrain] = ...
            feature_moments_blocked_local(source, Itr, 256);
        muAll = sumAll ./ max(nTokenTrain,1);
        if nTokenTrain > 1
            vAll = (sumSqAll - (sumAll.^2)./nTokenTrain) ./ (nTokenTrain-1);
        else
            vAll = zeros(1,Dtot);
        end
        % Guard tiny negative round-off from sum/sum-of-squares arithmetic.
        vAll = max(vAll,0);
        keepBaseIdx = logical(((vAll(1:baseDim) > 1e-12) & ...
                               isfinite(vAll(1:baseDim))).');
        inputIdx = [find(keepBaseIdx).' (baseDim+1):Dtot];
        mu  = muAll(inputIdx);
        sig = sqrt(vAll(inputIdx));
        sig(~isfinite(sig) | sig<1e-12) = 1;
        net = [];

    end
    Din = numel(inputIdx);

    muSingle  = reshape(single(mu),  1,1,Din);
    sigSingle = reshape(single(sig), 1,1,Din);

    % Matched input graph, architecture-specific parameterization. The
    % initialization RNG is independent of the trajectory split and batches.
    if isempty(net)
        rng(opts.InitializationSeed,'twister');
        net=spinn3d_build_alpha_network(Din,K,opts,alpha_min,alpha_max);
    end

    if useGPU
        net=dlupdate(@gpuArray,net);
    end

    % One supervised a_des label is produced at each high-level policy tick
    % and then held for policy_dt, but the governor begins executing that
    % command on the next plant sample.  Consequently all governor-response
    % and alpha-dot loss terms use the signed plant Ts from the token tail;
    % policy_dt remains the 100-Hz row/inference cadence only.

    policy_dt_loss = double(DS.policy_dt);

    pinn = build_pinn_cfg(opts, K, policy_dt_loss, DS.PB.control);
    pinn.plant_Ts=double(DS.data_signature.Ts_s);
    pinn.power_preview_steps=round(policy_dt_loss/pinn.plant_Ts);

    pinn.power_preview_duration_s=pinn.power_preview_steps*pinn.plant_Ts;
    pinn = pinn_to_device(pinn, useGPU);

    maxEpochs = opts.MaxEpochs;
    mbs       = opts.MiniBatchSize;
    initialLR = opts.InitialLearnRate;
    optimizer = learningContract.optimizer;
    lamO      = opts.LambdaOver;
    lamU      = opts.LambdaUnder;

    lamPINN   = opts.LambdaPINN;
    lamPAVG   = opts.LambdaPavgSq;
    lamSPD    = opts.LambdaSpeed;
    lamEND    = opts.LambdaEnd;

    trailingAvg = [];
    trailingAvgSq = [];
    iterGlobal = 0;
    bestValLoss = inf;
    bestEpoch = 0;
    bestNet = net;
    trainHistory = nan(maxEpochs,1);
    valHistory = nan(maxEpochs,1);
    componentNames={'mse','teacher_logit','governor','progress','physics','pavg','endpoint'};
    trainComponents=nan(maxEpochs,numel(componentNames));
    valComponents=nan(maxEpochs,numel(componentNames));
    gradientNormHistory=nan(maxEpochs,1);
    appliedGradientNormHistory=nan(maxEpochs,1);
    clippedBatchFractionHistory=nan(maxEpochs,1);
    learningRateRangeHistory=nan(maxEpochs,2);
    epochWallHistory=nan(maxEpochs,1);
    numericalSchema=learningContract.schema;
    logitLayer=alpha_logit_layer_local(net);
    runEpochs=maxEpochs;
    diagBase=struct('component_names',{componentNames},'logit_layer',logitLayer, ...
        'learning_contract',learningContract);

    if opts.Verbose
        fprintf('[alpha-train] N=%d, K=%d, Dtot=%d, Din=%d | train=%d, val=%d\n', N, K, Dtot, Din, numel(Itr), numel(Iva));
        fprintf('[alpha-train] epochs=%d, batch=%d, lr=%.3g | lamPINN=%.3g, lamSPD=%.3g, lamEND=%.3g | exec=%s\n', ...
            maxEpochs, mbs, initialLR, lamPINN, lamSPD, lamEND, execEnv);
        fprintf('[alpha-optimizer] %s | global norm <= %.4g | warmup=%d epoch | decay every %d epochs\n', ...
            optimizer.schema,optimizer.max_gradient_norm,optimizer.warmup_epochs,optimizer.decay_period_epochs);
    end

    nJ = infer_nJ_from_caps(opts.CAPS);
    idxF = decode_feat_indices(nJ,use_bus_schema,bus_dim_schema);

    for epoch = 1:runEpochs
        epochClock=tic;
        % Separate per-epoch batch RNG removes architecture-size/dropout
        % consumption as a source of different training-row permutations.
        rng(opts.BatchOrderSeed+epoch-1,'twister');
        if useGPU, gpurng(opts.BatchOrderSeed+epoch-1); end
        % Shuffle shard order and rows within each shard.  This preserves
        % the signed stochastic training order while the byte-bounded cache
        % services arbitrary rows from ordinary client RAM.
        Itr = shuffle_training_indices_local(Itr,source);

        nIter = ceil(numel(Itr)/mbs);
        loss_epoch = 0;
        component_acc=zeros(1,numel(componentNames));
        nSeen=0; maxGradientNorm=0; maxAppliedGradientNorm=0; clippedBatches=0;
        epochLrMin=inf; epochLrMax=0;

        for it = 1:nIter
            I = Itr((it-1)*mbs+1 : min(it*mbs, numel(Itr)));
            B = numel(I);

            % --- inputs (CBT), selected and normalised on demand ---
            Xbatch = read_alpha_rows_local(source,I);      % B×K×Dtot, single
            XrawMini = Xbatch(:,:,inputIdx);               % B×K×Din, single
            Zmini = bsxfun(@rdivide, ...
                bsxfun(@minus, XrawMini, muSingle), sigSingle);
            Zmini = permute(Zmini, [3 1 2]);               % Din×B×K
            Ymini = y(I).';                                % 1×B, single
            if useGPU
                Zmini = gpuArray(Zmini);
                Ymini = gpuArray(Ymini);
            end
            dlX = dlarray(Zmini, 'CBT');
            dly = dlarray(Ymini, 'CB');

            Xtail1 = reshape(Xbatch(:,1,baseDim+(1:Dtail)), [B,Dtail]);
            Ap = Xtail1(:,1).';
            Be = Xtail1(:,2).';
            Up = Xtail1(:,3).';
            Dn = Xtail1(:,4).';
            TsPlant = Xtail1(:,5).';

            Ts1 = TsPlant;
            S0 = Xtail1(:,6).';
            if useGPU
                Ap = gpuArray(Ap); Be = gpuArray(Be); Up = gpuArray(Up);
                Dn = gpuArray(Dn); Ts1 = gpuArray(Ts1);
                TsPlant = gpuArray(TsPlant); S0 = gpuArray(S0);
            end
            dlAp = dlarray(Ap, 'CB');
            dlBe = dlarray(Be, 'CB');
            dlUp = dlarray(Up, 'CB');
            dlDn = dlarray(Dn, 'CB');
            dlTs = dlarray(Ts1, 'CB');
            dlTsPlant = dlarray(TsPlant, 'CB');
            dlS0 = dlarray(S0, 'CB');

            DQG   = permute(Xbatch(:,:,idxF.dq),    [3 1 2]);   % nJ×B×K
            DDQG  = permute(Xbatch(:,:,idxF.ddq),   [3 1 2]);
            TAUFF = permute(Xbatch(:,:,idxF.tau_ff),[3 1 2]);
            TAUADOT = permute(Xbatch(:,:,idxF.tau_adot),[3 1 2]);
            GQ    = permute(Xbatch(:,:,idxF.g),     [3 1 2]);
            PbudB = reshape(Xbatch(:,1,idxF.Pbud), [1, B]);
            Pavg0NormB = reshape(Xbatch(:,1,idxF.Pavg_norm), [1, B]);
            Vdc0B = reshape(Xbatch(:,1,idxF.Vdc_norm),[1,B])*single(pinn.bus.V_source_V);
            if useGPU
                DQG = gpuArray(DQG); DDQG = gpuArray(DDQG);
                TAUFF = gpuArray(TAUFF); TAUADOT = gpuArray(TAUADOT);
                GQ = gpuArray(GQ); PbudB = gpuArray(PbudB);
                Pavg0NormB = gpuArray(Pavg0NormB);
                Vdc0B=gpuArray(Vdc0B);
            end
            dlDQG   = dlarray(DQG,   'CBT');
            dlDDQG  = dlarray(DDQG,  'CBT');
            dlTAUFF = dlarray(TAUFF, 'CBT');
            dlTAUADOT = dlarray(TAUADOT, 'CBT');
            dlG     = dlarray(GQ,    'CBT');
            dlPbud  = dlarray(PbudB, 'CB');
            dlPavg0Norm = dlarray(Pavg0NormB, 'CB');
            dlVdc0=dlarray(Vdc0B,'CB');
            dlSTATE=observed_state_batch_local(Xbatch,idxF,useGPU);

            % --- forward + loss ---
            [L,grad,parts]=dlfeval(@loss_batch,net,dlX,dly,dlAp,dlBe,dlUp,dlDn,dlTs,dlTsPlant, ...
                lamO,lamU,pinn,dlDQG,dlDDQG,dlTAUFF,dlTAUADOT,dlG,dlPbud,dlPavg0Norm,dlVdc0,dlSTATE, ...
                lamPINN,lamPAVG,lamSPD,alpha_min,alpha_max,lamEND,dlS0,opts.EndPhase,opts.EndWidth, ...
                logitLayer,learningContract.teacher_logit_weight);

            % Clip the full parameter gradient once, preserving its direction.
            % Record both the raw norm and the norm actually passed to Adam.
            [grad,clipStats]=spinn3d_alpha_clip_gradients(grad,optimizer.max_gradient_norm);
            maxAppliedGradientNorm=max(maxAppliedGradientNorm,clipStats.applied_norm);
            clippedBatches=clippedBatches+double(clipStats.clipped);
            lr=spinn3d_alpha_learning_rate(initialLR,epoch,it,nIter,optimizer);
            epochLrMin=min(epochLrMin,lr); epochLrMax=max(epochLrMax,lr);
            % Keep the last verified state until the candidate network AND
            % both Adam moments have passed the finite gate.
            nextIter=iterGlobal+1;
            [nextNet,nextAvg,nextAvgSq]=adamupdate(net,grad, ...
                trailingAvg,trailingAvgSq,nextIter,lr);
            net=nextNet; trailingAvg=nextAvg; trailingAvgSq=nextAvgSq;
            iterGlobal=nextIter;
            lossValue=scalar_double_local(L);
            c=component_vector_local(parts,componentNames);
            loss_epoch=loss_epoch+B*lossValue;
            component_acc=component_acc+B*c;
            nSeen=nSeen+B;
            maxGradientNorm=max(maxGradientNorm,clipStats.raw_norm);
        end

        loss_epoch=loss_epoch/max(nSeen,1);
        trainComponents(epoch,:)=component_acc/max(nSeen,1);
        gradientNormHistory(epoch)=maxGradientNorm;
        appliedGradientNormHistory(epoch)=maxAppliedGradientNorm;
        clippedBatchFractionHistory(epoch)=clippedBatches/nIter;
        learningRateRangeHistory(epoch,:)=[epochLrMin epochLrMax];

        % --- val ---
        if ~isempty(Iva)
            [Lva,valComponents(epoch,:),nValBatches] = validate_epoch(net, source, y, Iva, mbs, idxF, ...
                inputIdx, muSingle, sigSingle, baseDim, Dtail, ...
                policy_dt_loss, lamO, lamU, pinn, lamPINN, lamPAVG, lamSPD, ...
                alpha_min, alpha_max, lamEND, opts.EndPhase, opts.EndWidth, useGPU, ...
                diagBase);
        else
            error('spinn3d:AlphaTrainingValidationMissing', ...
                'Training requires held-out validation rows; no model was saved.');
        end
        trainHistory(epoch) = loss_epoch;
        valHistory(epoch) = Lva;
        selectionLoss = Lva;
        if isfinite(selectionLoss) && selectionLoss < bestValLoss
            bestValLoss = selectionLoss;
            bestEpoch = epoch;
            bestNet = net;
        end

        if opts.Verbose
            fprintf('[alpha-train] epoch %3d/%3d | L=%.4g | Lval=%.4g\n', epoch, maxEpochs, loss_epoch, Lva);
            fprintf(['[alpha-components] train: mse=%.4g teacher_logit=%.4g gov=%.4g progress=%.4g ', ...
                'physics=%.4g pavg=%.4g end=%.4g | max gradL2=%.4g\n'], ...
                trainComponents(epoch,:),gradientNormHistory(epoch));
            fprintf(['[alpha-components] validation: mse=%.4g teacher_logit=%.4g gov=%.4g ', ...
                'progress=%.4g physics=%.4g pavg=%.4g end=%.4g\n'], ...
                valComponents(epoch,:));
        end
        epochWallHistory(epoch)=toc(epochClock);
    end

    % ========= save =========
    if ~(bestEpoch>0 && isfinite(bestValLoss))
        error('spinn3d:AlphaTrainingNoFiniteValidationModel', ...
            ['No epoch produced a finite validation loss.  No model was ', ...
             'saved; inspect the data, optimiser settings and hardware.']);
    end
    net = bestNet;
    if useGPU
        net = dlupdate(@gather, net);
    end

    model = struct();
    model.net = net;
    model.preproc = struct();
    model.preproc.mu = mu;
    model.preproc.sig = sig;
    model.preproc.keepBaseIdx = keepBaseIdx;
    required_dataset_meta = {'K','ds','policy_dt', ...
        'use_bus_state_features','use_bus_state_rollout', ...
        'bus_feature_dim','state_feature_dim','state_feature_schema', ...
        'training_sample_stride_policy_ticks', ...
        'max_samples_per_traj','sample_selection_schema', ...
        'accepted_trajectory_count','rejected_candidate_draw_count', ...
        'training_schema','data_signature','power_fingerprint','PB'};

    model.preproc.K = double(DS.K);
    model.preproc.ds = double(DS.ds);
    model.preproc.policy_dt = double(DS.policy_dt);
    model.preproc.use_bus_state_features = ...
        logical(DS.use_bus_state_features);
    model.preproc.use_bus_state_rollout = ...
        logical(DS.use_bus_state_rollout);
    model.preproc.bus_feature_dim = double(DS.bus_feature_dim);
    model.preproc.feature_schema = 'alpha_plus_payload_gov_ctc_nobrake_v7';
    model.preproc.bus_feature_schema=state_layout.bus_schema;
    model.preproc.bus_feature_names=state_layout.bus_names;
    model.preproc.state_feature_dim=double(DS.state_feature_dim);
    model.preproc.state_feature_schema=char(DS.state_feature_schema);
    model.preproc.control_fingerprint=DS.data_signature.control;
    model.preproc.state_teacher_fingerprint=DS.data_signature.state_teacher;
    model.preproc.state_augmentation_fingerprint=DS.data_signature.state_augmentation;
    model.preproc.base_feature_dim = 14*nJ+12;
    model.preproc.expected_feature_dim = Dtot;
    model.preproc.alpha_min = alpha_min;
    model.preproc.alpha_max = alpha_max;
    model.preproc.alpha_output_schema = alpha_output_schema;
    model.preproc.training_sample_stride_policy_ticks = ...
        double(DS.training_sample_stride_policy_ticks);
    model.preproc.max_samples_per_traj = ...
        double(DS.max_samples_per_traj);
    model.preproc.sample_selection_schema = ...
        char(DS.sample_selection_schema);
    model.preproc.dataset_storage_precision = source.storage_precision;
    [pbud_observed,pbud_declared,pbud_anchor,pbud_anchor_count,pbud_schema] = ...
        infer_pbud_metadata_local(DS);

    model.preproc.pbud_range_train = pbud_observed; 
    model.preproc.pbud_range_observed_W = pbud_observed;
    model.preproc.pbud_range_declared_W = pbud_declared;
    % Historical field names retain "deployment" for file-format stability.
    % They identify the budget selected for this single experiment/training
    % trial; PB.power.P_TOTAL_MAX may intentionally differ in another trial.
    model.preproc.deployment_budget_anchor_W = pbud_anchor;
    model.preproc.deployment_budget_exact_traj_count = pbud_anchor_count;
    model.preproc.pbud_sampling_schema = pbud_schema;
    model.preproc.accepted_trajectory_count = ...
        double(DS.accepted_trajectory_count);
    model.preproc.rejected_candidate_draw_count = ...
        double(DS.rejected_candidate_draw_count);

    physics_id = char(DS.PB.physics.profile_id);
    model.preproc.physics_profile_id = physics_id;
    if strcmp(opts.Architecture,'mlp')
        model.preproc.kind='mlp_lookahead';
    else
        model.preproc.kind='trf_lookahead';
    end

    model.preproc.architecture=opts.Architecture;
    model.preproc.architecture_schema='alpha_matched_tokens_dual_architecture_v1';
    model.preproc.executionEnvironment = execEnv;
    model.preproc.trained_on_gpu = logical(useGPU);

    % retains the requested policy (normally "auto") so a GPU-trained model
    % can safely fall back to CPU on another machine.
    model.preproc.inferenceExecutionEnvironment = 'auto';
    model.preproc.splitInfo = splitInfo;

    model.preproc.best_epoch = bestEpoch;
    model.preproc.best_validation_loss = bestValLoss;
    model.preproc.learning_contract=learningContract;

    model.preproc.training_numerics_schema=numericalSchema;

    model.preproc.loss_component_names=componentNames;
    model.preproc.train_loss_component_history=trainComponents;
    model.preproc.validation_loss_component_history=valComponents;
    model.preproc.max_gradient_norm_history=gradientNormHistory;
    model.preproc.applied_gradient_norm_history=appliedGradientNormHistory;
    model.preproc.clipped_batch_fraction_history=clippedBatchFractionHistory;
    model.preproc.learning_rate_range_history=learningRateRangeHistory;
    model.preproc.epoch_wall_s=epochWallHistory;
    model.preproc.train_loss_history = trainHistory;
    model.preproc.validation_loss_history = valHistory;
    model.preproc.physics_regularizer_domain='causal_first_observed_ctc_plant_step_dynamic_dc_v3';
    model.preproc.teacher_runtime_boundary='observed_model_ctc_teacher_runtime_v2';
    model.preproc.power_preview_domain='zoh_policy_local_frozen_Mh_second_order_path_v1';
    model.preproc.power_preview_duration_s=pinn.power_preview_duration_s;
    model.preproc.power_preview_steps=pinn.power_preview_steps;
    model.preproc.soft_pavg_loss_schema = opts.SoftPavgLossSchema;
    model.preproc.lambda_pavg_sq = double(opts.LambdaPavgSq);
    model.preproc.lambda_speed = double(opts.LambdaSpeed);
    model.preproc.lambda_teacher_progress_shortfall = ...
        double(opts.LambdaSpeed);
    model.preproc.data_signature = DS.data_signature;
    model.preproc.training_schema = alpha_training_schema;
    model.preproc.training_signature = struct( ...
        'training_schema',alpha_training_schema, ...
        'TrainingNumericsSchema',numericalSchema, ...
        'LearningContract',learningContract, ...
        'Architecture',opts.Architecture, ...
        'ArchitectureSchema',model.preproc.architecture_schema, ...
        'ValidationInferenceMode','predict', ...
        'CTCControl',model.preproc.control_fingerprint, ...
        'StateTeacher',model.preproc.state_teacher_fingerprint, ...
        'StateAugmentation',model.preproc.state_augmentation_fingerprint, ...
        'StateFeatureSchema',model.preproc.state_feature_schema, ...
        'PhysicsRegularizerDomain',model.preproc.physics_regularizer_domain, ...
        'PowerPreviewDomain',model.preproc.power_preview_domain, ...
        'PowerPreviewDuration_s',model.preproc.power_preview_duration_s, ...
        'PowerPreviewSteps',model.preproc.power_preview_steps, ...
        'MLPHiddenWidths',opts.MLPHiddenWidths,'MLPActivation',opts.MLPActivation, ...
        'InputTokenCount',K,'InputFeatureCount',Din, ...
        'InitializationSeed',opts.InitializationSeed, ...
        'BatchOrderSeed',opts.BatchOrderSeed, ...
        'data_signature',model.preproc.data_signature, ...
        'MaxEpochs',opts.MaxEpochs,'MiniBatchSize',opts.MiniBatchSize, ...
        'InitialLearnRate',opts.InitialLearnRate, ...
        'ValidationFraction',opts.ValidationFraction, ...
        'SplitSeed',opts.SplitSeed,'DModel',opts.DModel, ...
        'NumHeads',opts.NumHeads,'FFNDim',opts.FFNDim, ...
        'NumLayers',opts.NumLayers,'Dropout',opts.Dropout, ...
        'Activation',opts.Activation, ...
        'LambdaOver',opts.LambdaOver,'LambdaUnder',opts.LambdaUnder, ...
        'LambdaPINN',opts.LambdaPINN, ...
        'LambdaPavgSq',opts.LambdaPavgSq, ...
        'SoftPavgLossSchema',opts.SoftPavgLossSchema, ...
        'LambdaSpeed',opts.LambdaSpeed, ...
        'AlphaMin',alpha_min,'AlphaMax',alpha_max, ...
        'AlphaOutputSchema',alpha_output_schema, ...
        'FineTuneSignature',fine_tune_signature, ...
        'ModelRole',opts.ModelRole, ...
        'LambdaEnd',opts.LambdaEnd,'EndPhase',opts.EndPhase, ...
        'EndWidth',opts.EndWidth,'PinnTokenDecay',opts.PinnTokenDecay, ...
        'HeadWidth',opts.HeadWidth,'PINNWeights',opts.PINNWeights);
    model.preproc.fine_tune_signature = fine_tune_signature;

    model.preproc.model_role = opts.ModelRole;

    if isfield(DS,'PB') && isstruct(DS.PB)
        model.preproc.plant_Ts_s = double(DS.PB.time.Ts);
        model.preproc.lookahead_H_s = double(DS.PB.train_trf.H);
        model.preproc.robot_params_fingerprint = DS.PB.robot.params;
        model.preproc.caps_fingerprint = DS.PB.caps;
        model.preproc.friction_fingerprint = DS.PB.fric;
        model.preproc.alpha_reg_fingerprint = DS.PB.alpha_reg;
        model.preproc.motor_fingerprint = getfield_def(DS.PB,'motor',struct());
        if isfield(DS.PB,'payload') && isstruct(DS.PB.payload) && ...
                isfield(DS.PB.payload,'range')
            model.preproc.payload_range_train = DS.PB.payload.range;
        end
        model.preproc.power_fingerprint = power_fingerprint_local(DS.PB);
    end

    if ~isempty(out_path)
        model_path = char(out_path);
        if exist(model_path,'file') == 2
            error('spinn3d_train_alpha_gov_trf:ModelExists', ...
                ['Refusing to overwrite the existing alpha model ''%s''. ' ...
                 'Delete or archive it manually before intentionally retraining.'], ...
                model_path);
        end
        atomic_save_model_local(model_path,model);
        if opts.Verbose
            fprintf('[alpha-train] saved model: %s\n', model_path);
        end
    end
end

function [alpha_min,alpha_max] = resolve_alpha_bounds_local(DS)

alpha_min = double(DS.alpha_min);
alpha_min_pb = double(DS.PB.alpha.alpha_min);
alpha_max = double(DS.PB.alpha.alpha_max);

if isfield(DS,'alpha_max') && ~isempty(DS.alpha_max)

end
end

function tf=same_nonnegative_integer_local(a,b)
tf=isnumeric(a) && isnumeric(b) && isscalar(a) && isscalar(b) && ...
   isfinite(double(a)) && isfinite(double(b)) && ...
   double(a)>=0 && double(b)>=0 && ...
   double(a)==fix(double(a)) && double(b)==fix(double(b)) && ...
   double(a)==double(b);
end

function atomic_save_model_local(model_path,model)
parent=fileparts(model_path);
if isempty(parent), parent=pwd; end
tmp=[tempname(parent) '.mat'];
save(tmp,'model','-v7.3');
if exist(model_path,'file')==2
    error('spinn3d_train_alpha_gov_trf:ModelRace', ...
        'Model appeared while training; temporary model retained at %s.',tmp);
end
[ok,msg]=movefile(tmp,model_path);
if ~ok
    error('spinn3d_train_alpha_gov_trf:CommitFailed', ...
        'Could not commit model (%s); temporary model retained at %s.',msg,tmp);
end
end

function fp = power_fingerprint_local(PB)

fp=spinn3d_power_fingerprint(PB);
end

function [L,grad,parts] = loss_batch(net, dlX, dly_t, dlAp, dlBe, dlUp, dlDn, dlTs, dlTsPlant, ...
                                lamO, lamU, pinn, dlDQG, dlDDQG, dlTAUFF, dlTAUADOT, dlG, dlPbud, dlPavg0Norm, dlVdc0, dlSTATE, ...
                                lamPINN, lamPAVG, lamSPD, alphaMin, alphaMax, ...
                                lamEND, dlS0, endPhase, endWidth,logitLayer,logitWeight)
    [dly_hat,logits]=forward(net,dlX,'Outputs',{'alpha_physical',logitLayer});
    [L,parts]=loss_terms_local(dly_hat,logits,logitWeight,dly_t,dlAp,dlBe,dlUp,dlDn, ...
        dlTs,dlTsPlant,lamO,lamU,pinn,dlDQG,dlDDQG,dlTAUFF, ...
        dlTAUADOT,dlG,dlPbud,dlPavg0Norm,dlVdc0,dlSTATE, ...
        lamPINN,lamPAVG,lamSPD,alphaMin,alphaMax,lamEND,dlS0,endPhase,endWidth);
    if isfield(parts,'power_preview'),parts=rmfield(parts,'power_preview');end
    grad=dlgradient(L,net.Learnables);
end

function [L,parts]=loss_terms_local(dly_hat,logits,logitWeight,dly_t,dlAp,dlBe,dlUp,dlDn, ...
        dlTs,dlTsPlant,lamO,lamU,pinn,dlDQG,dlDDQG,dlTAUFF, ...
        dlTAUADOT,dlG,dlPbud,dlPavg0Norm,dlVdc0,dlSTATE, ...
        lamPINN,lamPAVG,lamSPD,alphaMin,alphaMax,lamEND,dlS0,endPhase,endWidth)
    a_star=nominal_governor_dl_local(dly_t,dlAp,dlBe,dlUp,dlDn, ...
        dlTs,dlDQG,dlDDQG,pinn,alphaMax);
    a_post_h=nominal_governor_dl_local(dly_hat,dlAp,dlBe,dlUp,dlDn, ...
        dlTs,dlDQG,dlDDQG,pinn,alphaMax);
    parts=struct();
    parts.mse=mean((dly_hat-dly_t).^2,'all');
    [logitLoss,logitParts]=spinn3d_alpha_logit_teacher_loss(logits,dly_t,alphaMin,alphaMax);
    parts.teacher_logit=single(logitWeight).*logitLoss;
    parts.teacher_entropy=logitParts.teacher_entropy;
    parts.logits=logits;
    parts.governor=lamO*mean(max(a_post_h-a_star,0).^2,'all')+ ...
        lamU*mean(max(a_star-a_post_h,0).^2,'all');
    zero=parts.mse*single(0);
    parts.progress=zero; parts.physics=zero; parts.pavg=zero; parts.endpoint=zero;
    if lamSPD>0
        parts.progress=spinn3d_soft_power_speed_loss(single(0),single(1), ...
            a_post_h,a_star,alphaMin,alphaMax,0,lamSPD);
    end
    if (lamPINN>0 || lamPAVG>0) && pinn.enabled
        physTs=max(dlTsPlant,single(1e-6));
        [Lphys,PavgTrace,PavgTargetTrace,DC]= ...
            spinn3d_alpha_policy_power_loss(dly_hat,dlAp,dlBe,dlUp,dlDn,physTs, ...
                dlDQG,dlDDQG,dlTAUFF,dlTAUADOT,dlG,dlPbud, ...
                dlPavg0Norm,pinn,dlVdc0,dlSTATE,alphaMax);
        a_exec_h=DC.first_applied_alpha;
        adot_exec_h=DC.first_applied_alpha_dot;
        parts.physics=lamPINN*Lphys;
        parts.pavg=spinn3d_soft_power_speed_loss(PavgTrace, ...
            PavgTargetTrace,a_post_h,a_star,alphaMin,alphaMax,lamPAVG,0);
        if isfield(DC,'loss_components'), parts.physics_terms=DC.loss_components; end
        parts.applied_alpha=a_exec_h;
        parts.applied_alpha_dot=adot_exec_h;
        parts.power_preview=struct('Pgrid_W',DC.policy_grid_trace, ...
            'Pavg_W',PavgTrace,'target_W',PavgTargetTrace, ...
            'applied_alpha',DC.policy_alpha_trace);
    end
    if lamEND>0 && isfield(pinn,'qd_lim') && isfield(pinn,'nJ')
        parts.endpoint=lamEND*end_slowdown_loss(a_post_h, ...
            (a_post_h-dlAp)./dlTs,dlTs,dlDQG,dlDDQG,dlSTATE,dlS0,pinn,endPhase,endWidth);
    end
    L=parts.mse+parts.teacher_logit+parts.governor+parts.progress+parts.physics+parts.pavg+parts.endpoint;
    parts.desired_alpha=dly_hat;
    parts.teacher_applied_alpha=a_star;
end

function [Lva,components,nIter] = validate_epoch(net, source, y, Iva, mbs, idxF, ...
        inputIdx, muSingle, sigSingle, baseDim, Dtail, ...
        policy_dt_loss, lamO, lamU, pinn, lamPINN, lamPAVG, lamSPD, ...
        alphaMin, alphaMax, lamEND, endPhase, endWidth, useGPU, ...
        diagBase)
    nIter = ceil(numel(Iva)/mbs);
    acc = 0; nSeen=0;
    component_acc=zeros(1,numel(diagBase.component_names));
    for it=1:nIter
        I = Iva((it-1)*mbs+1 : min(it*mbs, numel(Iva)));

        B = numel(I);
        Xbatch = read_alpha_rows_local(source,I);
        XrawMini = Xbatch(:,:,inputIdx);
        Zmini = bsxfun(@rdivide, ...
            bsxfun(@minus, XrawMini, muSingle), sigSingle);
        Zmini = permute(Zmini, [3 1 2]);
        Ymini = y(I).';
        if useGPU
            Zmini = gpuArray(Zmini);
            Ymini = gpuArray(Ymini);
        end
        dlX = dlarray(Zmini, 'CBT');
        dly = dlarray(Ymini, 'CB');

        Xtail1 = reshape(Xbatch(:,1,baseDim+(1:Dtail)), [B,Dtail]);
        Ap = Xtail1(:,1).';
        Be = Xtail1(:,2).';
        Up = Xtail1(:,3).';
        Dn = Xtail1(:,4).';
        TsPlant = Xtail1(:,5).';

        Ts1 = TsPlant;
        S0 = Xtail1(:,6).';
        if useGPU
            Ap = gpuArray(Ap); Be = gpuArray(Be); Up = gpuArray(Up);
            Dn = gpuArray(Dn); Ts1 = gpuArray(Ts1);
            TsPlant = gpuArray(TsPlant); S0 = gpuArray(S0);
        end
        dlAp = dlarray(Ap, 'CB');
        dlBe = dlarray(Be, 'CB');
        dlUp = dlarray(Up, 'CB');
        dlDn = dlarray(Dn, 'CB');
        dlTs = dlarray(Ts1, 'CB');
        dlTsPlant = dlarray(TsPlant, 'CB');
        dlS0 = dlarray(S0, 'CB');

        DQG   = permute(Xbatch(:,:,idxF.dq),    [3 1 2]);
        DDQG  = permute(Xbatch(:,:,idxF.ddq),   [3 1 2]);
        TAUFF = permute(Xbatch(:,:,idxF.tau_ff),[3 1 2]);
        TAUADOT = permute(Xbatch(:,:,idxF.tau_adot),[3 1 2]);
        GQ    = permute(Xbatch(:,:,idxF.g),     [3 1 2]);
        PbudB = reshape(Xbatch(:,1,idxF.Pbud), [1, B]);
        Pavg0NormB = reshape(Xbatch(:,1,idxF.Pavg_norm), [1, B]);
        Vdc0B = reshape(Xbatch(:,1,idxF.Vdc_norm),[1,B])*single(pinn.bus.V_source_V);
        if useGPU
            DQG = gpuArray(DQG); DDQG = gpuArray(DDQG);
            TAUFF = gpuArray(TAUFF); TAUADOT = gpuArray(TAUADOT);
            GQ = gpuArray(GQ); PbudB = gpuArray(PbudB);
            Pavg0NormB = gpuArray(Pavg0NormB);
            Vdc0B=gpuArray(Vdc0B);
        end
        dlDQG   = dlarray(DQG,   'CBT');
        dlDDQG  = dlarray(DDQG,  'CBT');
        dlTAUFF = dlarray(TAUFF, 'CBT');
        dlTAUADOT = dlarray(TAUADOT, 'CBT');
        dlG     = dlarray(GQ,    'CBT');
        dlPbud  = dlarray(PbudB, 'CB');
        dlPavg0Norm = dlarray(Pavg0NormB, 'CB');
        dlVdc0=dlarray(Vdc0B,'CB');
            dlSTATE=observed_state_batch_local(Xbatch,idxF,useGPU);

        % Validation uses inference mode: dropout must be disabled when
        % selecting best_epoch, identically to actual deployed prediction.
        [dly_hat,logits]=predict(net,dlX,'Outputs',{'alpha_physical',diagBase.logit_layer});
        [L,parts]=loss_terms_local(dly_hat,logits,diagBase.learning_contract.teacher_logit_weight,dly,dlAp,dlBe,dlUp,dlDn, ...
            dlTs,dlTsPlant,lamO,lamU,pinn,dlDQG,dlDDQG,dlTAUFF,dlTAUADOT,dlG,dlPbud,dlPavg0Norm,dlVdc0,dlSTATE, ...
            lamPINN,lamPAVG,lamSPD,alphaMin,alphaMax,lamEND,dlS0,endPhase,endWidth);
        acc=acc+B*scalar_double_local(L);
        component_acc=component_acc+B*component_vector_local(parts,diagBase.component_names);
        nSeen=nSeen+B;
    end
    Lva=acc/max(nSeen,1);
    components=component_acc/max(nSeen,1);

end

% ===================== network =====================
% ===================== PINN =====================
function pinn = build_pinn_cfg(opts, K, policy_dt, control)
    pinn = struct();
    pinn.enabled = false;

    if ~isstruct(opts.CAPS) || isempty(fieldnames(opts.CAPS))
        return;
    end

    caps = opts.CAPS;
    nJ = infer_nJ_from_caps(caps);
    pinn.nJ = nJ;

    pinn.ctc_Kp=expand_axis_param_local(control.Kp,nJ,NaN);
    pinn.ctc_Kd=expand_axis_param_local(control.Kd,nJ,NaN);

    fric = opts.FRIC;
    pinn.fricB    = reshape(double(getfield_def(fric,'B',0)), [], 1);
    pinn.fricFc   = reshape(double(getfield_def(fric,'Fc',0)), [], 1);
    pinn.vel_eps  = double(getfield_def(fric,'vel_eps',1e-3));

    pinn.tau_lim  = reshape(double(getfield_def(caps,'tau_max',inf(nJ,1))), [], 1);
    pinn.qd_lim   = reshape(double(getfield_def(caps,'qd_max',inf(nJ,1))),  [], 1);
    pinn.qdd_lim  = reshape(double(getfield_def(caps,'qdd_max',inf(nJ,1))), [], 1);
    pinn.Pax_lim  = reshape(double(getfield_def(caps,'P_axis_max',inf(nJ,1))), [], 1);

    bus_pinn=getfield_def(caps,'bus',struct());
    bus_pinn=spinn3d_bus_defaults(bus_pinn);
    pinn.eta_motoring=double(bus_pinn.eta_motoring);
    pinn.eta_regen=double(bus_pinn.eta_regen);
    pinn.eta_share=double(bus_pinn.eta_share);
    pinn.P_idle_W=double(bus_pinn.P_idle_W);
    pinn.Pavg_target_frac = double(bus_pinn.P_avg_target_frac);
    pavg_tau_s = double(bus_pinn.P_avg_window_s);

    pinn.Pavg_tau_s=pavg_tau_s;
    pinn.physics_domain='causal_first_observed_ctc_plant_step_dynamic_dc_v3';
    pinn.bus=bus_pinn;
    pinn.dynamic_dc=strcmp(char(bus_pinn.model_scope),'dynamic_capacitor_common_dc_v1');

    pinn.dc_constants=spinn3d_motor_dc_constants(bus_pinn,nJ);
    if ~pinn.dynamic_dc,pinn.P_brk_peak=double(bus_pinn.P_brk_peak);end
    pinn.regen_enable = logical(bus_pinn.regen_enable);
    pinn.direct_share_enable = logical(bus_pinn.direct_share_enable);
    pinn.routing_mode = char(bus_pinn.routing_mode);
    pinn.quadrant_power_model = char(bus_pinn.quadrant_power_model);
    pinn.actuator_envelope_model = char( ...
        bus_pinn.actuator_envelope_model);
    pinn.demand_limited_direct = strcmpi(strtrim(pinn.routing_mode),'demand_limited_direct');
    pinn.Pregen_peak_max_W=double(bus_pinn.Pregen_peak_max_W);
    pinn.Pshare_peak_max_W=double(bus_pinn.Pshare_peak_max_W);
    pinn.bus_model_st3215=strcmpi(char(bus_pinn.model),'st3215_pwm');

    pinn.V_bus_V = double(getfield_def(bus_pinn,'V_bus_V',12.0));
    pinn.I_noload_A = expand_axis_param_local( ...
        getfield_def(bus_pinn,'I_noload_A',0.20),nJ,0.20);
    pinn.I_stall_A = expand_axis_param_local( ...
        getfield_def(bus_pinn,'I_stall_A',2.70),nJ,2.70);
    pinn.tau_stall_single_Nm = expand_axis_param_local( ...
        getfield_def(bus_pinn,'tau_stall_single_Nm',30*0.0980665), ...
        nJ,30*0.0980665);
    pinn.omega_noload_rad_s = expand_axis_param_local( ...
        getfield_def(bus_pinn,'omega_noload_rad_s',40*2*pi/60), ...
        nJ,40*2*pi/60);
    pinn.servo_parallel = expand_axis_param_local( ...
        getfield_def(bus_pinn,'servo_parallel',[1 2 1 1]),nJ,1.0);
    pinn.tau_dead_Nm = max(expand_axis_param_local( ...
        getfield_def(bus_pinn,'tau_dead_Nm',0.0),nJ,0.0),0.0);

    % P_TOTAL_MAX is the filtered-power target. The physical Pgrid residual
    % still uses only the raw peak cap; the distinct soft objective below
    % propagates the causal EMA and penalises normalized-square target excess.
    % Keeping these roles separate allows finite soft excess without
    % misclassifying it as a physical violation.
    pinn.Pgrid_peak_cap = double(getfield_def(caps,'P_grid_peak_max', ...
                              getfield_def(caps,'Pgrid_peak_max', inf)));
    if ~isfinite(pinn.Pgrid_peak_cap) || pinn.Pgrid_peak_cap <= 0
        pinn.Pgrid_peak_cap = inf;
    end

    % Geometry tokens are encoder inputs, not samples at policy_dt.
    % The hard physical residual uses the first interval; the independent
    % soft-power objective unfolds one complete held-command policy interval.
    pinn.wtoken=ones(1,1,1);
    pinn.first_token_mask=ones(1,1,1);

    required_weights = {'tau','qd','qdd','Pax','Pgrid'};

    pinn.w = opts.PINNWeights;

    pinn.enabled = true;
end

function pinn = pinn_to_device(pinn, useGPU)
    if ~useGPU || ~isstruct(pinn) || ~isfield(pinn,'enabled') || ~pinn.enabled
        return;
    end
    cf=fieldnames(pinn.dc_constants);
    for ic=1:numel(cf)
        if isnumeric(pinn.dc_constants.(cf{ic}))
            pinn.dc_constants.(cf{ic})=gpuArray(single(pinn.dc_constants.(cf{ic})));
        end
    end
    % Keep the original CPU fields for scalar control-flow checks such as
    % all(isfinite(pinn.qd_lim)). Store GPU copies in *_dev fields for math.
    fns = {'ctc_Kp','ctc_Kd','fricB','fricFc','tau_lim','qd_lim','qdd_lim','Pax_lim', ...
           'wtoken','first_token_mask','servo_parallel','tau_dead_Nm', ...
           'I_noload_A','I_stall_A','tau_stall_single_Nm', ...
           'omega_noload_rad_s'};
    for i = 1:numel(fns)
        f = fns{i};
        if isfield(pinn,f) && ~isempty(pinn.(f))
            pinn.([f '_dev']) = gpuArray(single(pinn.(f)));
        end
    end
end

function v = pinn_compute_field(pinn, f)
    fd = [f '_dev'];
    if isfield(pinn, fd) && ~isempty(pinn.(fd))
        v = pinn.(fd);
    else
        v = single(pinn.(f));
    end
end

function Lend = end_slowdown_loss(alpha,adot,dt,dlDQG,dlDDQG,dlSTATE,dlS0,pinn,endPhase,endWidth)

% alpha: 1×B
% dlDQG: nJ×B×K

    nJ = pinn.nJ;
    kp=reshape(pinn_compute_field(pinn,'ctc_Kp'),[],1);
    kd=reshape(pinn_compute_field(pinn,'ctc_Kd'),[],1);
    [~,~,~,dq0]=spinn3d_alpha_observed_ctc_kernel(alpha,adot,dt, ...
        dlDQG(:,:,1),dlDDQG(:,:,1),dlSTATE,kp,kd);
    % Penalize predicted actual end velocity, including CTC correction;
    % a zero geometric derivative alone is not evidence that the arm stopped.

    qd_lim_raw = pinn_compute_field(pinn, 'qd_lim');
    qd_lim = reshape(qd_lim_raw(1:nJ), [], 1);
    v = mean((dq0./max(qd_lim,1e-6)).^2, 1);    % 1×B

    % w_end(s) = sigmoid((s - endPhase)/endWidth)
    z = (dlS0 - endPhase) ./ max(endWidth,1e-6);
    w = 1 ./ (1 + exp(-z));

    Lend = mean(w .* v, 'all');
end

% ===================== indices / utils =====================
function nJ = infer_nJ_from_caps(caps)
    if isstruct(caps) && isfield(caps,'tau_max') && ~isempty(caps.tau_max)
        nJ = numel(caps.tau_max);
    elseif isstruct(caps) && isfield(caps,'qd_max') && ~isempty(caps.qd_max)
        nJ = numel(caps.qd_max);
    else
        nJ = 4; 
    end
end

function v = expand_axis_param_local(v,nJ,def)
% Match spinn3d_motor_power's scalar/short-vector expansion for PINN mirrors.
try
    v = double(v(:));
catch
    v = double(def(:));
end
if isempty(v), v = double(def(:)); end
v(~isfinite(v)) = double(def(1));
if isscalar(v)
    v = repmat(v,nJ,1);
elseif numel(v)<nJ
    v = [v;repmat(v(end),nJ-numel(v),1)];
else
    v = v(1:nJ);
end
end

function idx = decode_feat_indices(nJ,use_bus_features,bus_feature_dim)

% DS.Xseq = [alpha_plus, payload10, BUS10, observed eq/edq/M/h, tail6]
    idx = struct();
    idx.dq     = (nJ+1):(2*nJ);
    idx.ddq    = (2*nJ+1):(3*nJ);
    idx.tau_ff = (3*nJ+1):(4*nJ);
    idx.tau_adot = (4*nJ+1):(5*nJ);
    idx.g      = (6*nJ+2):(7*nJ+1);
    idx.Pbud   = (7*nJ+6);

    state_layout=spinn3d_alpha_state_feature_layout(nJ);
    idx.Pavg_norm=state_layout.Pavg_norm;
    idx.Vdc_norm=state_layout.Vdc_norm;
    idx.state=state_layout.state;
end

function x = getfield_def(S,fn,def)
    if isstruct(S) && isfield(S,fn) && ~isempty(S.(fn))
        x = S.(fn);
    else
        x = def;
end
end

function assert_finite_xseq_blocked_local(source, blockRows)
% Validate immutable shards without materialising an N-by-K-by-D tensor.
N = source.N;
for i0 = 1:blockRows:N
    ii = i0:min(i0+blockRows-1,N);
    xb = read_alpha_rows_local(source,ii);
    if ~all(isfinite(xb(:)))
        error('spinn3d_train_alpha_gov_trf:NonfiniteDataset', ...
            ['Alpha dataset Xseq contains NaN/Inf in sample rows %d:%d; ' ...
             'the checkpoint is corrupted.'], ii(1), ii(end));
    end
end
end

function [sumX,sumSqX,nToken] = feature_moments_blocked_local(source, sampleIdx, blockRows)
% Accumulate training-only token moments using bounded temporary blocks.
D = source.D;
K = source.K;
sumX = zeros(1,D,'double');
sumSqX = zeros(1,D,'double');
nToken = 0;
for i0 = 1:blockRows:numel(sampleIdx)
    jj = i0:min(i0+blockRows-1,numel(sampleIdx));
    xb = read_alpha_rows_local(source,sampleIdx(jj));
    xr = reshape(xb,[],D);
    % Compute stable statistics in a bounded double-precision workspace.
    xd = double(xr);
    sumX = sumX + sum(xd,1);
    sumSqX = sumSqX + sum(xd.^2,1);
    nToken = nToken + numel(jj)*K;
end
end

function source = alpha_sequence_source_local(DS)

% Support chunked MAT-file and in-memory sequence data.
if isfield(DS,'sequence_storage') && ...
        strcmp(char(DS.sequence_storage),'chunked_matfile_v1')

    
    source = struct();
    source.mode = 'chunked';
    source.manifest = DS.chunk_manifest(:);
    source.N = double(numel(DS.y));
    source.K = double(DS.K);
    source.D = double(DS.expected_feature_dim);
    source.shard_bytes = zeros(numel(source.manifest),1);
    source.storage_precision = char( ...
        getfield_def(DS,'storage_precision','single'));

    expected_first = 1;
    manifest_files = cell(numel(source.manifest),1);
    for ic = 1:numel(source.manifest)
        C = source.manifest(ic);
        fields = {'file','first_sample','last_sample','n_samples'};

        first = double(C.first_sample);
        last = double(C.last_sample);
        ns = double(C.n_samples);

        fn = char(C.file);
        manifest_files{ic} = fn;

        W = whos('-file',fn,'Xseq_c');

        source.shard_bytes(ic) = double(W.bytes);

        expected_first = last+1;
    end
    manifest_keys = cellfun(@(p) lower(char(p)),manifest_files, ...
        'UniformOutput',false);

    
elseif isfield(DS,'Xseq') && ~isempty(DS.Xseq)
    X = single(DS.Xseq);
    dims = size(X);

    source = struct('mode','memory','Xseq',X,'N',double(dims(1)), ...
        'K',double(dims(2)),'D',double(dims(3)), ...
        'storage_precision','single');
else
    error('spinn3d:AlphaSequenceSourceMissing', ...
        ['Alpha dataset has neither chunked_matfile_v1 storage nor a ', ...
         'bounded in-memory Xseq source.']);
end
end

function X = read_alpha_rows_local(source,sampleIdx)
sampleIdx = double(sampleIdx(:));

if strcmp(source.mode,'memory')
    X = source.Xseq(sampleIdx,:,:);
    return;
end

X = zeros(numel(sampleIdx),source.K,source.D,'single');
shard_id = zeros(numel(sampleIdx),1);
for ic = 1:numel(source.manifest)
    C = source.manifest(ic);
    take = sampleIdx>=double(C.first_sample) & ...
           sampleIdx<=double(C.last_sample);
    if ~any(take), continue; end

    shard_id(take) = ic;
end

% Follow first occurrence in the requested batch; the LRU retains every
% shard that fits the central byte ceiling without changing output order.
shard_order = unique(shard_id,'stable');
for io = 1:numel(shard_order)
    ic = shard_order(io);
    C = source.manifest(ic);
    take = shard_id==ic;
    out_pos = find(take);
    local_rows = sampleIdx(take)-double(C.first_sample)+1;
    xb = alpha_shard_cache_local('rows',char(C.file),local_rows, ...
        double(C.n_samples),source.K,source.D);
    X(out_pos,:,:) = xb;
end
end

function X = alpha_shard_cache_local(action,varargin)
% Byte-bounded client-process LRU for immutable single Xseq_c shards.  Disk
% access is always a whole-variable sequential load; arbitrary/duplicate row
% indices are applied only to ordinary in-memory arrays, never to a MatFile.
persistent cache_files cache_shapes cache_data cache_bytes cache_last_used
persistent cache_tick budget_bytes

if isempty(cache_files), cache_files = cell(0,1); end
if isempty(cache_shapes), cache_shapes = zeros(0,3); end
if isempty(cache_data), cache_data = cell(0,1); end
if isempty(cache_bytes), cache_bytes = zeros(0,1); end
if isempty(cache_last_used), cache_last_used = zeros(0,1); end
if isempty(cache_tick), cache_tick = 0; end
if isempty(budget_bytes), budget_bytes = 0; end

action = lower(char(action));
switch action
    case 'reset'
        % Dropping the cell array releases every cached shard on normal or
        % exceptional trainer exit; no MAT-file handle remains open.
        cache_data = cell(0,1);
        cache_files = cell(0,1);
        cache_shapes = zeros(0,3);
        cache_bytes = zeros(0,1);
        cache_last_used = zeros(0,1);
        cache_tick = 0;
        budget_bytes = 0;
        X = zeros(0,0,0,'single');
        return;

    case 'configure'

        budget_gib = varargin{1};

        budget_bytes = double(budget_gib)*(1024^3);
        X = zeros(0,0,0,'single');
        return;

    case 'status'

        X = struct('entry_count',numel(cache_files), ...
            'total_bytes',sum(cache_bytes), ...
            'budget_bytes',budget_bytes);
        return;

    case 'rows'

    otherwise
        error('spinn3d:AlphaChunkCacheBadRequest', ...
            'Unknown Alpha shard cache action: %s',action);
end

fn = char(varargin{1});
local_rows = double(varargin{2}(:));
n_rows = double(varargin{3});
K = double(varargin{4});
D = double(varargin{5});
expected_shape = [n_rows,K,D];

shape_match = false(numel(cache_files),1);
if ~isempty(cache_shapes)
    shape_match = all(bsxfun(@eq,cache_shapes,expected_shape),2);
end
hit = find(strcmp(cache_files,fn) & shape_match,1,'first');
if isempty(hit)
    % Never retain two identities for one immutable path.  A shape change is
    % treated as a cache miss and the stale entry is released before I/O.
    stale = find(strcmp(cache_files,fn));
    for is = numel(stale):-1:1
        victim = stale(is);
        cache_data{victim} = zeros(0,0,0,'single');
        cache_data(victim) = [];
        cache_files(victim) = [];
        cache_shapes(victim,:) = [];
        cache_bytes(victim) = [];
        cache_last_used(victim) = [];
    end

    needed_bytes = 4*prod(expected_shape);

    while sum(cache_bytes)+needed_bytes>budget_bytes

        [~,victim] = min(cache_last_used);
        cache_data{victim} = zeros(0,0,0,'single');
        cache_data(victim) = [];
        cache_files(victim) = [];
        cache_shapes(victim,:) = [];
        cache_bytes(victim) = [];
        cache_last_used(victim) = [];
    end

    S = load(fn,'Xseq_c');

    candidate = S.Xseq_c;
    clear S
    candidate_info = whos('candidate');

    cache_tick = cache_tick+1;
    cache_files{end+1,1} = fn;
    cache_shapes(end+1,:) = expected_shape;
    cache_data{end+1,1} = candidate;
    cache_bytes(end+1,1) = needed_bytes;
    cache_last_used(end+1,1) = cache_tick;
    hit = numel(cache_files);

else
    cache_tick = cache_tick+1;
    cache_last_used(hit) = cache_tick;
end

X = cache_data{hit}(local_rows,1:K,1:D);
end

function out = shuffle_training_indices_local(indices,source)
indices = double(indices(:).');
if strcmp(source.mode,'memory')
    out = indices(randperm(numel(indices)));
    return;
end
groups = cell(numel(source.manifest),1);
for ic = 1:numel(source.manifest)
    C = source.manifest(ic);
    g = indices(indices>=double(C.first_sample) & ...
                indices<=double(C.last_sample));
    if ~isempty(g), g = g(randperm(numel(g))); end
    groups{ic} = g;
end
order = randperm(numel(groups));
out = [groups{order}];

end

function value=scalar_double_local(value)
if isa(value,'dlarray'), value=extractdata(value); end
value=double(gather(value));

end

function values=component_vector_local(parts,names)
values=zeros(1,numel(names));
for k=1:numel(names), values(k)=scalar_double_local(parts.(names{k})); end
end

function name=alpha_logit_layer_local(net)
names={net.Layers.Name};
if ismember('head_out',names), name='head_out';
elseif ismember('mlp_out',names), name='mlp_out';
else
    error('spinn3d:AlphaLogitLayerMissing','The signed Alpha output head is missing.');
end
end

function [observed,declared,anchor,anchor_count,schema] = infer_pbud_metadata_local(DS)
vals = [];
declared = [NaN NaN];
anchor = NaN;
anchor_count = 0;
schema = 'legacy-unspecified';
if isstruct(DS) && isfield(DS,'meta') && isstruct(DS.meta)
    for ii = 1:numel(DS.meta)
        if isfield(DS.meta(ii),'pbud') && isnumeric(DS.meta(ii).pbud) && ...
                isscalar(DS.meta(ii).pbud) && isfinite(double(DS.meta(ii).pbud))
            vals(end+1) = double(DS.meta(ii).pbud); %#ok<AGROW>
        end
    end
end
if isempty(vals)
    observed = [NaN NaN];
else
    observed = [min(vals) max(vals)];
end
if isstruct(DS) && isfield(DS,'PB') && isstruct(DS.PB)
    if isfield(DS.PB,'alpha_budget') && isstruct(DS.PB.alpha_budget) && ...
            isfield(DS.PB.alpha_budget,'pbud_range')
        rr = double(DS.PB.alpha_budget.pbud_range(:));
        rr = rr(isfinite(rr));
        if ~isempty(rr), declared = [min(rr) max(rr)]; end
    end
    if isfield(DS.PB,'power') && isstruct(DS.PB.power) && ...
            isfield(DS.PB.power,'P_TOTAL_MAX') && ...
            isscalar(DS.PB.power.P_TOTAL_MAX) && isfinite(double(DS.PB.power.P_TOTAL_MAX))
        anchor = double(DS.PB.power.P_TOTAL_MAX);
    end
end
if isstruct(DS) && isfield(DS,'pbud_deployment_anchor_W') && ...
        isscalar(DS.pbud_deployment_anchor_W) && isfinite(double(DS.pbud_deployment_anchor_W))
    anchor = double(DS.pbud_deployment_anchor_W);
end
if isfinite(anchor) && ~isempty(vals)
    anchor_count = sum(abs(vals-anchor) <= 1e-9);
end
if isstruct(DS) && isfield(DS,'pbud_deployment_exact_traj_count') && ...
        isscalar(DS.pbud_deployment_exact_traj_count) && ...
        isfinite(double(DS.pbud_deployment_exact_traj_count))
    anchor_count = double(DS.pbud_deployment_exact_traj_count);
end
if isstruct(DS) && isfield(DS,'pbud_sampling_schema') && ~isempty(DS.pbud_sampling_schema)
    schema = char(DS.pbud_sampling_schema);
end
end

function a=nominal_governor_dl_local(a_des,ap,beta,up,dn,Ts,dqg,ddqg,pinn,amax)
a=spinn3d_alpha_governor_dl(a_des,ap,beta,up,dn,Ts,dqg(:,:,1),ddqg(:,:,1),pinn,amax);
end

function dlSTATE=observed_state_batch_local(Xbatch,idxF,useGPU)
% State is one causal observation. Future geometry columns are not a rollout.
B=size(Xbatch,1);
state=reshape(Xbatch(:,1,idxF.state),[B,numel(idxF.state)]).';
if useGPU, state=gpuArray(state); end
dlSTATE=dlarray(state,'CB');
end
