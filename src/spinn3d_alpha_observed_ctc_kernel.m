function [tau,dq_start,dq_mid,dq_next,ddq_actual] = ...
    spinn3d_alpha_observed_ctc_kernel(alpha,alpha_dot,dt,dqg,ddqg,state,Kp,Kd)
% Numeric/dlarray kernel, n-by-batch state and n^2-by-batch column-major M.
% state=[eq;edq;M(:);h]. Current q/dq are observations, never future labels.
n=size(dqg,1);

eq=state(1:n,:); edq=state(n+(1:n),:);
Mf=state(2*n+(1:n*n),:); h=state(2*n+n*n+(1:n),:);
aprev=alpha-alpha_dot.*dt;
dq_start=aprev.*dqg+edq;
ddq_actual=alpha.^2.*ddqg+alpha_dot.*dqg + ...
    Kd.*(alpha.*dqg-dq_start)-Kp.*eq;
% Explicit small-joint product avoids format/device conversion and keeps AD.
rows=cell(n,1);
for i=1:n
    rows{i}=sum(Mf(i:n:n*n,:).*ddq_actual,1)+h(i,:);
end
tau=cat(1,rows{:});
dq_mid=dq_start+0.5.*dt.*ddq_actual;
dq_next=dq_start+dt.*ddq_actual;
end
