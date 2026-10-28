function learningRate=spinn3d_alpha_learning_rate(initialLR,epoch,batch,nBatches,optimizer)
%SPINN3D_ALPHA_LEARNING_RATE Deterministic linear warmup and epoch-step decay.
% Required optimizer fields: warmup_epochs, warmup_start_factor,
% decay_period_epochs, decay_factor, min_lr_factor. With warmup_epochs=1,
% the first and last batches of epoch 1 use start_factor and 1 respectively.
% A one-step warmup uses its endpoint (factor 1). Decay is anchored to epoch
% numbering: a period of 10 first decays at epoch 11, then 21, 31, ... .
fields={'warmup_epochs','warmup_start_factor','decay_period_epochs', ...
    'decay_factor','min_lr_factor'};

w=optimizer.warmup_epochs; start=optimizer.warmup_start_factor;
period=optimizer.decay_period_epochs; decay=optimizer.decay_factor; floorFactor=optimizer.min_lr_factor;

epoch=double(epoch);batch=double(batch);nBatches=double(nBatches);
w=double(w);start=double(start);period=double(period);
decay=double(decay);floorFactor=double(floorFactor);
if w>0 && epoch<=w
    steps=w*nBatches; step=(epoch-1)*nBatches+batch;
    if steps==1, factor=1;
    else, factor=start+(1-start)*(step-1)/(steps-1); end
else
    factor=max(floorFactor,decay^floor((epoch-1)/period));
end
learningRate=double(initialLR)*factor;
end

function tf=positive_scalar_local(value)
tf=isnumeric(value)&&isreal(value)&&isscalar(value)&&isfinite(value)&&value>0;
end
function tf=integer_local(value,minimum)
tf=isnumeric(value)&&isreal(value)&&isscalar(value)&&isfinite(value)&& ...
    value>=minimum&&value==fix(value);
end
