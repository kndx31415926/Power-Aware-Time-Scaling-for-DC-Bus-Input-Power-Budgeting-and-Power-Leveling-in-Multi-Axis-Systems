function [x,state] = spinn3d_alpha_observed_state_features(qref,dqg,a_prev,robot,state)
% One observation is reused for every geometric look-ahead token. Errors
% are relative to that token; M/h always refer to the current observation.
% M and h may be precomputed once from the controller's observed model.
n=numel(qref);

q=double(state.q(:)); dq=double(state.dq(:));

haveM=isfield(state,'M') && ~isempty(state.M);
haveH=isfield(state,'h') && ~isempty(state.h);

if ~haveM

    qf=q.'; dqf=dq.';
    if strcmpi(char(robot.DataFormat),'column'), qf=q; dqf=dq; end
    state.M=double(massMatrix(robot,qf));
    c=velocityProduct(robot,qf,dqf); g=gravityTorque(robot,qf);
    B=expand_local(state.fric.B,n); Fc=expand_local(state.fric.Fc,n);
    f=B.*dq+Fc.*tanh(dq./max(double(state.fric.vel_eps),1e-9));
    state.h=double(c(:))+double(g(:))+f;
end
M=double(state.M); h=double(state.h(:));

eq=q-double(qref(:)); edq=dq-double(a_prev).*double(dqg(:));
x=[eq.',edq.',M(:).',h.'];
end

function v=expand_local(v,n)
v=double(v(:));
if isscalar(v), v=repmat(v,n,1); end

end
