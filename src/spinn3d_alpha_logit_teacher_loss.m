function [L,parts] = spinn3d_alpha_logit_teacher_loss( ...
        logits,alpha_teacher,alpha_min,alpha_max)
%SPINN3D_ALPHA_LOGIT_TEACHER_LOSS Stable soft-target teacher supervision.
% The physical command remains alpha_min + span*sigmoid(logits). This loss
% acts on that SAME graph's pre-sigmoid logits, without detaching or changing
% the forward command. For normalized fixed teacher target y in [0,1],
% d(BCE)/d(logits) = sigmoid(logits)-y (before the mean reduction). Thus an
% incorrect saturated command retains a teacher correction, whereas a
% correct endpoint is allowed to have zero gradient. Endpoint labels are
% valid; neither labels nor logits are clipped.
%
% L is Bernoulli cross entropy, not a probability of a physical event. It
% has the fixed teacher entropy as an irreducible baseline. The optional KL

% not max-clamped or used to create a flat training region near the optimum.

% Form BOTH endpoints in the label precision before taking their span.
% In single precision, single(amax)-single(amin) need not equal
% single(amax-amin). The latter can map a valid upper endpoint above one.
zero_label=alpha_teacher.*0;
lo=zero_label+alpha_min;
hi=zero_label+alpha_max;
target=(alpha_teacher-lo)./(hi-lo);

% Labels are fixed data. This validation never extracts/detaches logits.
payload=target;
if isa(payload,'dlarray'), payload=extractdata(payload); end
valid=all(isfinite(payload) & payload>=0 & payload<=1,'all');
if isa(valid,'gpuArray'), valid=gather(valid); end

% All exponent arguments are nonpositive. Keeping the TWO exponential
% branches also gives the exact .5-y derivative at z=0 regardless of the
% subgradient chosen by max at its tie. No log(sigmoid(z)) is evaluated.
m=max(logits,0);
per_element=(m-target.*logits)+log(exp(-m)+exp(logits-m));
L=mean(per_element,'all');

if nargout>1
    if isa(payload,'gpuArray'), dtype=classUnderlying(payload); else, dtype=class(payload); end
    tiny=realmin(dtype);
    entropy=mean(-target.*log(max(target,tiny)) ...
        -(1-target).*log(max(1-target,tiny)),'all');
    parts=struct('bernoulli_cross_entropy',L,'teacher_entropy',entropy, ...
        'kl_diagnostic',L-entropy, ...
        'schema','alpha_teacher_stable_logit_bce_v1');
end
end
