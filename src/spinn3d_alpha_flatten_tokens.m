function Y=spinn3d_alpha_flatten_tokens(X,expectedC,expectedK)
%SPINN3D_ALPHA_FLATTEN_TOKENS Differentiable ordered C x K -> C*K mapping.
% Input/output formats are explicit, including in MATLAB R2026a and newer.
% No token is pooled, discarded, re-normalized or mixed across observations.

c=finddim(X,'C'); b=finddim(X,'B'); t=finddim(X,'T');
nC=size(X,c); nB=size(X,b); nT=size(X,t);

raw=permute(stripdims(X),[c t b]);
Y=dlarray(reshape(raw,nC*nT,nB),'CB');
end
