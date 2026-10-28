function sol = spinn3d_bus_ptp_dp_r2(robot,traj,cfg,limits)
%SPINN3D_BUS_PTP_DP_R2 Offline bounded-beam fixed-path command search.
% A label retains (progress, applied alpha, filtered source power, elapsed
% time, entire desired-command history). Labels are NOT collapsed by (s,a).
% Global beam pruning is approximate and carries no global-optimality claim.
% Commands change only on policy boundaries and each edge executes the same
% plant-rate scalar governor used online. Inverse-dynamics predictions are
% nominal; measured-state CTC, noise and physical-guard feedback are assessed
% only by the subsequent common 1-ms plant replay.

O=cfg.PTP_R2;req={'Ns','Na','alpha_min','alpha_max','soft_power_weight', ...
    'slew_weight','progress_time_weight','beamN','max_plan_time_s','hard_tolerance'};

Ns=max(5,round(double(O.Ns)));Na=max(3,round(double(O.Na)));Bn=max(1,round(double(O.beamN)));
amin=double(O.alpha_min);amax=double(O.alpha_max);aGrid=linspace(amin,amax,Na);
sTargets=linspace(0,traj.Tf,Ns);
bus=get_bus_from_cfg(cfg,limits.P_total_max);bus.plant_Ts_s=cfg.TS;
Ptarget=double(bus.P_total_max);[qInit,~,~]=traj.eval(0);
init=spinn3d_static_hold_power_init(robot,row(qInit),cfg.FRIC,bus,Ptarget,cfg.TS);
dt=double(cfg.POLICY.dt);maxTime=double(O.max_plan_time_s);
b0=struct('cost',0,'s',0,'a_prev',amin,'Pavg',double(init.Pgrid_avg0_W), ...
    'elapsed_s',0,'rho_seq',[],'desired_seq',[]);
if strcmp(bus.model_scope,'dynamic_capacitor_common_dc_v1'), b0.bus_state=init.bus_state0; end
beam=b0;nEval=0;nFeas=0;nPlant=0;nRejected=0;maxLabels=1;
for i=2:Ns
    nxt=repmat(b0,0,1);
    for bi=1:numel(beam)
        if beam(bi).s>=sTargets(i)-1e-12
            nxt(end+1,1)=beam(bi); %#ok<AGROW>
            continue;
        end
        for ci=1:Na
            nEval=nEval+1; b=beam(bi);ok=true;
            b.rho_seq=[b.rho_seq b.s/max(traj.Tf,eps)]; %#ok<AGROW>
            b.desired_seq=[b.desired_seq aGrid(ci)]; %#ok<AGROW>
            while b.s<sTargets(i)-1e-12
                if b.elapsed_s+dt>maxTime+1e-12,ok=false;break;end
                [b2,met,ok]=spinn3d_r2_nominal_policy_step(b,aGrid(ci),traj,robot,limits,bus,cfg.FRIC,cfg.ALPHA_REG,dt,cfg.TS,double(O.hard_tolerance));
                nPlant=nPlant+met.n_plant_evaluations;
                if ~ok||met.progress<=1e-14,ok=false;break;end
                b2.cost=b.cost+double(O.progress_time_weight)*met.elapsed_s+ ...
                    double(O.soft_power_weight)*met.soft_integral+ ...
                    double(O.slew_weight)*met.alpha_change_squared;
                b2.elapsed_s=b.elapsed_s+met.elapsed_s;b=b2;
            end
            if ok
                nFeas=nFeas+1;nxt(end+1,1)=b; %#ok<AGROW>
            else
                nRejected=nRejected+1;
            end
        end
    end
    if isempty(nxt)
        error('spinn3d_bus_ptp_dp_r2:NoPath', ...
            'Bounded nominal search has no extension at stage %d/%d (attempts %d, rejected %d); no plan claimed.',i,Ns,nEval,nRejected);
    end
    [~,ord]=sort([nxt.cost]);beam=nxt(ord(1:min(Bn,numel(ord))));maxLabels=max(maxLabels,numel(beam));
end
[best,ib]=min([beam.cost]);b=beam(ib);

rho=b.rho_seq;aDes=b.desired_seq;
[rho,ia]=unique(rho,'stable');aDes=aDes(ia);
if rho(end)<1,rho=[rho 1];aDes=[aDes aDes(end)];end
plan=struct('rho_knots',rho,'a_knots',aDes,'rho',rho,'a_des_seq',aDes, ...
    's_norm',rho,'a_des',aDes,'lookup_mode','previous', ...
    'command_domain','desired_alpha','direct_desired_replay',true);
sol=struct('plan',plan,'objective',best,'Ns',Ns,'Na',Na,'beamN',Bn, ...
    'n_transition_evaluations',nEval,'n_feasible_transitions',nFeas, ...
    'n_rejected_transitions',nRejected,'n_plant_evaluations',nPlant, ...
    'maximum_retained_labels',maxLabels,'nominal_elapsed_s',b.elapsed_s, ...
    'nominal_Pavg_end_W',b.Pavg,'completed',true,'global_optimality_claim',false, ...
    'prediction_model','nominal_reference_1ms_substeps','schema',char(O.schema), ...
    'seed_kind','bus-ptp-bounded-beam-r97');
end
function y=row(x),y=double(x(:)).';end
