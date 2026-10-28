function net = spinn3d_build_alpha_network(Din,K,opts,alphaMin,alphaMax)
% Build the sequence-to-rate Transformer or MLP architecture.

architecture=lower(char(opts.Architecture));

if strcmp(architecture,'transformer')
    net=build_trf_encoder(Din,opts.DModel,opts.NumHeads,opts.FFNDim, ...
        opts.NumLayers,opts.Dropout,opts.HeadWidth,opts.Activation,alphaMin,alphaMax);
    return;
end

widths=double(opts.MLPHiddenWidths(:).');

activation=lower(char(opts.MLPActivation));

lg=layerGraph();
lg=addLayers(lg,sequenceInputLayer(Din,Normalization="none",Name="in"));
lg=addLayers(lg,functionLayer(@(X) spinn3d_alpha_flatten_tokens(X,Din,K), ...
    Name="mlp_flatten_tokens",Formattable=true));
lg=connectLayers(lg,"in","mlp_flatten_tokens");
previous="mlp_flatten_tokens";
for i=1:numel(widths)
    fcName=sprintf('mlp_fc_%d',i); actName=sprintf('mlp_act_%d',i);
    lg=addLayers(lg,fullyConnectedLayer(widths(i),Name=fcName));
    if strcmp(activation,'gelu')
        lg=addLayers(lg,geluLayer(Name=actName));
    else
        lg=addLayers(lg,reluLayer(Name=actName));
    end
    lg=connectLayers(lg,previous,fcName);
    lg=connectLayers(lg,fcName,actName);
    previous=actName;
end
head=[fullyConnectedLayer(1,Name="mlp_out")
    sigmoidLayer(Name="sig")
    functionLayer(@(X) single(alphaMin)+single(alphaMax-alphaMin).*X, ...
        Name="alpha_physical",Formattable=true)];
lg=addLayers(lg,head);
lg=connectLayers(lg,previous,"mlp_out");
% Sequence input alone cannot infer the fixed flattened length. Initialize
% with the actual central K, rather than accepting an inferred one-token FC.
net=dlnetwork(lg,Initialize=false);
net=initialize(net,dlarray(zeros(Din,1,K,'single'),'CBT'));
end

function net = build_trf_encoder(Din, dModel, numHeads, ffnDim, ...
        numLayers, dropP, headWidth, activation,alphaMin,alphaMax)

    numKeyChannels = dModel;  

    lg = layerGraph();

    lg = addLayers(lg, sequenceInputLayer(Din, Normalization="none", Name="in"));
    lg = addLayers(lg, fullyConnectedLayer(dModel, Name="embed"));
    lg = addLayers(lg, reluLayer(Name="embed_relu"));
    lg = connectLayers(lg, "in", "embed");
    lg = connectLayers(lg, "embed", "embed_relu");

    prev = "embed_relu";

    for l = 1:numLayers
        saName    = sprintf("sa_%d", l);
        addSaName = sprintf("add_sa_%d", l);
        lnSaName  = sprintf("ln_sa_%d", l);

        fc1Name   = sprintf("ffn1_%d", l);
        actName   = sprintf("ffn_act_%d", l);
        fc2Name   = sprintf("ffn2_%d", l);
        addFName  = sprintf("add_ffn_%d", l);
        lnFName   = sprintf("ln_ffn_%d", l);

        sa = selfAttentionLayer(numHeads, numKeyChannels, Name=saName, DropoutProbability=dropP);
        lg = addLayers(lg, sa);
        lg = addLayers(lg, additionLayer(2, Name=addSaName));
        lg = addLayers(lg, layerNormalizationLayer(Name=lnSaName));

        lg = addLayers(lg, fullyConnectedLayer(ffnDim, Name=fc1Name));
        switch activation
            case 'gelu'

                lg = addLayers(lg, geluLayer(Name=actName));
            case 'relu'
                lg = addLayers(lg, reluLayer(Name=actName));
            otherwise
                error('spinn3d_train_alpha_gov_trf:InvalidActivation', ...
                    'Unsupported central activation ''%s''.',activation);
        end
        lg = addLayers(lg, dropoutLayer(dropP, Name=sprintf("drop_%d",l)));
        lg = addLayers(lg, fullyConnectedLayer(dModel, Name=fc2Name));
        lg = addLayers(lg, additionLayer(2, Name=addFName));
        lg = addLayers(lg, layerNormalizationLayer(Name=lnFName));

        % connections: prev -> sa -> addSa/in1 ; prev -> addSa/in2
        lg = connectLayers(lg, prev, saName);
        lg = connectLayers(lg, saName, addSaName + "/in1");
        lg = connectLayers(lg, prev, addSaName + "/in2");
        lg = connectLayers(lg, addSaName, lnSaName);

        % ffn: lnSa -> fc1 -> act -> dropout -> fc2 -> addF/in1 ; lnSa -> addF/in2
        lg = connectLayers(lg, lnSaName, fc1Name);
        lg = connectLayers(lg, fc1Name, actName);
        lg = connectLayers(lg, actName, sprintf("drop_%d",l));
        lg = connectLayers(lg, sprintf("drop_%d",l), fc2Name);
        lg = connectLayers(lg, fc2Name, addFName + "/in1");
        lg = connectLayers(lg, lnSaName, addFName + "/in2");
        lg = connectLayers(lg, addFName, lnFName);

        prev = lnFName;
    end

    if exist('indexing1dLayer','file')==2
        pick = indexing1dLayer("first", Name="pick_first");
        lg = addLayers(lg, pick);
        lg = connectLayers(lg, prev, "pick_first");
        prev2 = "pick_first";
    else
        pick = functionLayer(@(X) X(:,:,1), Name="pick_first", Formattable=true);
        lg = addLayers(lg, pick);
        lg = connectLayers(lg, prev, "pick_first");
        prev2 = "pick_first";
    end

    head = [
        fullyConnectedLayer(headWidth, Name="head_fc1")
        reluLayer(Name="head_relu")
        fullyConnectedLayer(1, Name="head_out")
        sigmoidLayer(Name="sig")
        functionLayer(@(X) single(alphaMin) + ...
            single(alphaMax-alphaMin).*X, ...
            Name="alpha_physical", Formattable=true)
    ];

    lg = addLayers(lg, head);
    lg = connectLayers(lg, prev2, "head_fc1");

    net = dlnetwork(lg);
end

