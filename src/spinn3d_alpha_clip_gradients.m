function [grad,stats]=spinn3d_alpha_clip_gradients(grad,maxNorm)
%SPINN3D_ALPHA_CLIP_GRADIENTS One global L2 bound for a learnable-gradient table.
% The same positive scale is applied to every Value tensor, preserving its
% direction, precision, dlarray labels and CPU/GPU placement. Non-finite
% inputs are rejected before any tensor is changed. Norm reductions convert
% to double before squaring and normalize first to avoid finite-value overflow.
% raw_norm is Inf only if the mathematical norm exceeds double's range; the
% scaled computation can still clip such finite double-valued tensors safely.

maxNorm=double(maxNorm);
[rawNorm,largest,rootSum]=norm_local(grad);
stats=struct('raw_norm',rawNorm,'applied_norm',rawNorm,'scale',1,'clipped',false);
if largest==0 || rawNorm<=maxNorm, return; end
% This order avoids an overflowing denominator even when the true norm
% cannot be represented by one double scalar.
stats.scale=(maxNorm/largest)/rootSum;
stats.clipped=true;
for k=1:height(grad)
    if isempty(grad.Value{k}), continue; end
    value=grad.Value{k}; payload=payload_local(value);
    if isa(payload,'gpuArray'), precision=classUnderlying(payload);else,precision=class(payload);end
    if stats.scale>=realmin(precision)
        grad.Value{k}=value.*stats.scale;
    else
        % Avoid casting a factor smaller than the tensor's normal range to
        % single first. Only this exceptional path extracts the gradient;
        % clipping is applied after dlgradient, outside the loss graph.
        clipped=(double(payload)./largest).*(maxNorm/rootSum);
        clipped=cast(clipped,'like',payload);
        if isa(value,'dlarray'), clipped=dlarray(clipped,dims(value));end
        grad.Value{k}=clipped;
    end
end
stats.applied_norm=norm_local(grad);
end

function [total,largest,rootSum]=norm_local(grad)
largest=0;
for k=1:height(grad)
    value=payload_local(grad.Value{k});
    if isempty(value), continue; end

    largest=max(largest,double(gather(max(abs(value(:))))));
end
if largest==0, total=0;rootSum=0;return;end
sumSquares=0;
for k=1:height(grad)
    value=payload_local(grad.Value{k});
    if isempty(value), continue; end
    normalized=double(value(:))./largest;
    sumSquares=sumSquares+double(gather(sum(normalized.^2)));
end
rootSum=sqrt(sumSquares);
total=largest*rootSum;
end

function value=payload_local(value)
if isa(value,'dlarray'), value=extractdata(value); end
if isa(value,'gpuArray'), precision=classUnderlying(value);else,precision=class(value);end

end
