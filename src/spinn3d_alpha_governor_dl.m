function a=spinn3d_alpha_governor_dl(a_des,ap,beta,up,dn,Ts,dqg,ddqg,pinn,amax)
% Numeric/dlarray implementation of the common plant-rate governor.
% Saturated requests retain their true zero derivative; no straight-through
% approximation is used. Geometry is the current locally predicted geometry.
ql=reshape(field_local(pinn,'qd_lim'),[],1);
al=reshape(field_local(pinn,'qdd_lim'),[],1);
akin=min(single(amax),min(min(ql./max(abs(dqg),single(1e-12)),[],1), ...
    min(sqrt(al./max(abs(ddqg),single(1e-12))),[],1)));
target=min(max(a_des,0),akin);
slew=min(ap+up.*Ts,max(ap-dn.*Ts,target));
a=beta.*ap+(1-beta).*slew;
end
function v=field_local(pinn,f)
fd=[f '_dev'];
if isfield(pinn,fd) && ~isempty(pinn.(fd)),v=pinn.(fd);else,v=single(pinn.(f));end
end
