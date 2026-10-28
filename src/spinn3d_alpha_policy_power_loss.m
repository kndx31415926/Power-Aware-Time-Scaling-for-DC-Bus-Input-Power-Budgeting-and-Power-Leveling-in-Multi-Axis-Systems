function [Lphys,PavgTrace,PavgTargetTrace,DC]=spinn3d_alpha_policy_power_loss( ...
        command,ap,beta,up,dn,plantTs,dlDQG,dlDDQG,dlTAUFF,dlTAUADOT,dlG, ...
        dlPbud,dlPavg0Norm,pinn,dlVdc0,dlSTATE,alphaMax)
% A held request is executed at plant rate for one complete policy interval.
% The soft-power loss uses
% every causal EMA update in the interval, with advancing alpha, velocity,
% tracking error and DC voltage. M and total bias h stay at their observed
% values; the local reference uses constant q_ss. This is explicitly a local
% model approximation, not a complete nonlinear robot/guard rollout. Later
% geometry tokens are not future measured states and are not replayed as time.

N=pinn.power_preview_steps;n=pinn.nJ;
dt=reshape(plantTs,1,[]);a_prev=ap;
dqg=cb_local(dlDQG(:,:,1));ddqg=cb_local(dlDDQG(:,:,1));state=cb_local(dlSTATE);
pavg=reshape(dlPavg0Norm.*dlPbud,1,[],1);V=dlVdc0;
kp=reshape(field_local(pinn,'ctc_Kp'),[],1);
kd=reshape(field_local(pinn,'ctc_Kd'),[],1);
ema=exp(-dt./single(pinn.Pavg_tau_s));
powers=cell(1,N);alphas=cell(1,N);voltages=cell(1,N);grids=cell(1,N);
for k=1:N
    a=spinn3d_alpha_governor_dl(command,a_prev,beta,up,dn,dt,dqg,ddqg,pinn,alphaMax);
    adot=(a-a_prev)./dt;
    if k==1
        [Lphys,pavg,target,DC]=spinn3d_alpha_first_step_loss(a,adot,dt, ...
            dlDQG,dlDDQG,dlTAUFF,dlTAUADOT,dlG,dlPbud,dlPavg0Norm,pinn,V,state);
        ddq=cb_local(DC.ctc_ddq_actual);dq_start=cb_local(DC.ctc_dq_start);
        dq_next=cb_local(DC.ctc_dq_next);pavg=cb_local(pavg);target=cb_local(target);
        V=cb_local(DC.Vdc_next);Pgrid=cb_local(DC.Pgrid);
        DC.first_applied_alpha=a;DC.first_applied_alpha_dot=adot;
    else
        [tau,dq_start,dq_mid,dq_next,ddq]=spinn3d_alpha_observed_ctc_kernel( ...
            a,adot,dt,dqg,ddqg,state,kp,kd);
        dc=pinn.dc_constants;
        names=fieldnames(dc);
        for j=1:numel(names)
            key=names{j};value=dc.(key);
            if isnumeric(value),dc.(key)=reshape(single(value),[],1,1)+tau(:,1,1).*single(0);end
        end
        electrical=spinn3d_motor_dc_kernel(tau,dq_mid,dc);
        route=spinn3d_dynamic_dc_route(electrical.Pterminal_W,pinn.bus);
        next=spinn3d_dc_link_kernel(route.Pload_axis,route.Pregen_injected,V,0,pinn.bus,pinn.plant_Ts);
        V=cb_local(next.Vdc_next);Pgrid=cb_local(next.Pgrid);
        pavg=ema.*pavg+(single(1)-ema).*Pgrid;
    end
    powers{k}=pavg;alphas{k}=a;voltages{k}=V;grids{k}=Pgrid;
    ds=a.*dt;
    dqg_next=dqg+ddqg.*ds;
    % eq=q_actual-q_reference; edq=dq_actual-a_previous*q_s.
    eq_next=state(1:n,:)+dt.*dq_start+single(.5).*dt.^2.*ddq ...
        -dqg.*ds-single(.5).*ddqg.*ds.^2;
    edq_next=dq_next-a.*dqg_next;
    state=cat(1,eq_next,edq_next,state(2*n+1:end,:));
    dqg=dqg_next;a_prev=a;
end
PavgTrace=cat(3,powers{:});PavgTargetTrace=target+single(0).*PavgTrace;
DC.policy_alpha_trace=cat(3,alphas{:});DC.policy_vdc_trace=cat(3,voltages{:});
DC.policy_grid_trace=cat(3,grids{:});DC.policy_final_state=state;
DC.power_preview_steps=N;
DC.power_preview_domain='zoh_policy_local_frozen_Mh_second_order_path_v1';
end
function v=field_local(pinn,f)
fd=[f '_dev'];
if isfield(pinn,fd) && ~isempty(pinn.(fd)),v=pinn.(fd);else,v=single(pinn.(f));end
end

function x=cb_local(x)
% A plant state is C-by-B. Remove the input token's singleton T label while
% preserving its AD graph before concatenating mechanical state blocks.
if isa(x,'dlarray')
    x=dlarray(reshape(stripdims(x),size(x,1),[]),'CB');
else
    x=reshape(x,size(x,1),[]);
end
end
