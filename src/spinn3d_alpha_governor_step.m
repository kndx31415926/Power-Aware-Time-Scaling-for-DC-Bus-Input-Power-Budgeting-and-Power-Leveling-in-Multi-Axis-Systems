function [a,a_slew,a_filt,adot,G] = spinn3d_alpha_governor_step( ...
        a_des,a_prev,beta,up,dn,Ts,dqg,ddqg,caps,alpha_max)
% Exact common plant-rate governor: ZOH applies to a_des, never a_post.

a_des=min(alpha_max,max(0,a_des));
a_kin=min([double(alpha_max), ...
    min(double(caps.qd_max(:))./max(abs(double(dqg(:))),1e-12)), ...
    min(sqrt(double(caps.qdd_max(:))./max(abs(double(ddqg(:))),1e-12)))]);
a_target=min(a_des,a_kin);
[a,a_slew,a_filt]=governor_step(a_target,a_prev,beta,up,dn,Ts);
adot=(a-a_prev)/Ts;
G=struct('a0',a_des,'a_target',a_target,'a_slew',a_slew, ...
    'a_filt',a_filt,'a_kin',a_kin);
end
