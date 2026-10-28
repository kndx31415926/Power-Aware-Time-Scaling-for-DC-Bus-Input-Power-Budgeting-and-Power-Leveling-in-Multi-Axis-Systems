function inputs=spinn3d_r6_runtime_model_inputs(Xseq,P)

% Original saved corpus windows remain unchanged. Feature preparation is
% outside the separately measured batch-one network forward boundary.

K=double(P.K);D=double(P.expected_feature_dim);Dtail=6;
kb=logical(P.keepBaseIdx(:).');idx=[find(kb),D-Dtail+1:D];
mu=double(P.mu(:).');sig=double(P.sig(:).');Din=numel(mu);

inputs=cell(size(Xseq,1),1);
for k=1:numel(inputs)
    X=reshape(double(Xseq(k,:,idx)),K,Din);
    Z=(X-mu)./max(sig,1e-12);
    inputs{k}=dlarray(single(reshape(Z.',Din,1,K)),'CBT');
end
end
