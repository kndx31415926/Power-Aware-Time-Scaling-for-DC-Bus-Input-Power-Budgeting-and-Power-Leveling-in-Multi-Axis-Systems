function [a_des,info] = spinn3d_mpc_rg_grid_step_r2(state,traj,robot,caps,bus,fric,areg,policy_dt,opts)
%SPINN3D_MPC_RG_GRID_STEP_R2 Bounded nominal-reference receding-horizon RG.
% Not nonlinear measured-state MPC: q_meas/dq_meas are intentionally unused.
% Each desired command is held at 10 ms; every 1-ms governor and nominal
% inverse-dynamics/electrical substep is screened. The common measured-state
% executor remains authoritative. No global optimality claim is made.
t0=tic;
required={'Na','H','beamN','alpha_floor','alpha_max','soft_power_weight', ...
    'progress_weight','slew_weight','raw_peak_weight','hard_tolerance'};

Na=max(3,round(double(opts.Na)));H=max(1,round(double(opts.H)));beamN=max(1,round(double(opts.beamN)));
amin=double(opts.alpha_floor);amax=double(opts.alpha_max);

cand=linspace(amin,amax,Na);
Ts=double(getf(bus,'plant_Ts_s',1e-3));
b0=struct('cost',0,'s',double(state.s),'a_prev',double(state.a_prev),'Pavg',double(state.Pgrid_avg),'seq',[]);

if strcmp(bus.model_scope,'dynamic_capacitor_common_dc_v1')

    b0.bus_state=state.bus_state;
end
beam=b0;nEval=0;nReject=0;nPlant=0;depth=0;
for h=1:H
    nxt=repmat(b0,0,1);
    for bi=1:numel(beam)
        for ci=1:numel(cand)
            nEval=nEval+1;
            [b2,met,ok]=spinn3d_r2_nominal_policy_step(beam(bi),cand(ci),traj,robot,caps,bus,fric,areg,policy_dt,Ts,double(opts.hard_tolerance));
            nPlant=nPlant+met.n_plant_evaluations;
            if ~ok,nReject=nReject+1;continue;end
            b2.cost=beam(bi).cost-double(opts.progress_weight)*met.progress+ ...
                double(opts.soft_power_weight)*met.soft_integral+ ...
                double(opts.slew_weight)*met.alpha_change_squared+ ...
                double(opts.raw_peak_weight)*met.raw_integral;
            b2.seq=[beam(bi).seq cand(ci)]; %#ok<AGROW>
            nxt(end+1,1)=b2; %#ok<AGROW>
        end
    end
    if isempty(nxt),beam=repmat(b0,0,1);break;end
    [~,ord]=sort([nxt.cost]);beam=nxt(ord(1:min(beamN,numel(ord))));depth=h;
end
failed=isempty(beam)||depth<H;
info=struct('active',true,'failed',failed,'Na',Na,'H',H,'beamN',beamN, ...
    'completed_horizon_steps',depth,'horizon_s',H*policy_dt,'n_eval',nEval, ...
    'n_hard_reject',nReject,'n_feasible_transitions',nEval-nReject, ...
    'n_plant_evaluations',nPlant,'prediction_model','nominal_reference_1ms_substeps', ...
    'global_optimality_claim',false,'soft_target_W',double(getf(bus,'P_total_max',inf)), ...
    'raw_peak_cap_W',double(getf(bus,'P_grid_peak_max',inf)), ...
    'fallback','','best_cost',NaN,'best_sequence',[],'predicted_Pavg_end_W',NaN);
if failed
    % A reduced horizon is NOT relabelled as a successful H-step solve.
    % This desired floor is still subject to the common actual physical guard.
    a_des=amin;info.fallback='desired_alpha_floor_common_physical_guard';
else
    [~,ib]=min([beam.cost]);best=beam(ib);a_des=best.seq(1);
    info.best_cost=best.cost;info.best_sequence=best.seq;info.predicted_Pavg_end_W=best.Pavg;
end
info.solve_wall_s=toc(t0);
end
function v=getf(S,f,d),if isstruct(S)&&isfield(S,f)&&~isempty(S.(f)),v=S.(f);else,v=d;end,end
