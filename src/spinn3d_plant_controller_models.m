function P = spinn3d_plant_controller_models(cfg,robot_nominal,bus_nominal,q0)
%SPINN3D_PLANT_CONTROLLER_MODELS Keep undisclosed plant parameters private.
% robot_nominal already contains cfg.PAYLOAD.DEMO. Only simulation code gets
% P.robot/P.fric/P.bus; ctl.step and reference/feature builders keep cfg and
% robot_nominal. Known-parameter sweeps change cfg upstream. Unknown-plant
% sweeps use PLANT_OVERRIDE and never alter the controller's model.
P=struct('nominal_robot',robot_nominal,'robot',robot_nominal, ...
    'nominal_fric',cfg.FRIC,'fric',cfg.FRIC, ...
    'nominal_bus',bus_nominal,'bus',bus_nominal,'unknown',false);
nominal_payload=cfg.PAYLOAD.DEMO; true_payload=nominal_payload;
mode='nominal';
if isfield(cfg,'REVISION_PERTURBATION') && isfield(cfg.REVISION_PERTURBATION,'mode')
    mode=char(cfg.REVISION_PERTURBATION.mode);
end
if isfield(cfg,'PLANT_OVERRIDE') && ~isempty(cfg.PLANT_OVERRIDE)

    O=cfg.PLANT_OVERRIDE;

    P.fric=O.FRIC; P.bus=spinn3d_bus_defaults(O.BUS);

    true_payload=O.PAYLOAD.DEMO;
    [P.robot,~]=spinn3d_inject_payload_last_body(cfg.ROBOT,true_payload);
    P.unknown=true;
end
P.measurement_uncertain=isfield(cfg,'NOISE') && ...
    ((isfield(cfg.NOISE,'sigma_q') && any(double(cfg.NOISE.sigma_q)~=0)) || ...
     (isfield(cfg.NOISE,'sigma_dq') && any(double(cfg.NOISE.sigma_dq)~=0)));
P.separated_execution=P.unknown || P.measurement_uncertain;
% A task continuation supplies observed V, even when actuator/circuit
% parameters are unknown. Reconstruct only the true energy alias with true C
% inside the plant, keeping that parameter out of controller configuration.
if isfield(bus_nominal,'model_scope') && ...
        strcmp(bus_nominal.model_scope,'dynamic_capacitor_common_dc_v1') && ...
        isfield(bus_nominal,'dc_initial_state_mode') && ...
        strcmp(bus_nominal.dc_initial_state_mode,'explicit_measured_state')

    P.bus.dc_initial_state_mode='explicit_measured_state';
    P.bus.dc_state=spinn3d_dc_state_for_model(bus_nominal.dc_state,P.bus);
    P.bus.V_init_V=P.bus.dc_state.Vdc;
    if isfield(bus_nominal,'initial_Pgrid_avg_W')
        P.bus.initial_Pgrid_avg_W=bus_nominal.initial_Pgrid_avg_W;
    elseif isfield(P.bus,'initial_Pgrid_avg_W')
        P.bus=rmfield(P.bus,'initial_Pgrid_avg_W');
    end
end
P.power_init=spinn3d_static_hold_power_init( ...
    P.robot,q0,P.fric,P.bus,cfg.P_TOTAL_MAX,cfg.TS);
P.bus_state=P.power_init.bus_state0;
P.observed_steps=0;
P.meta=struct('schema','separate_nominal_true_plant_v1','mode',mode, ...
    'unknown_parameters',P.unknown,'noisy_state_measurements',P.measurement_uncertain, ...
    'controller_receives_true_dynamics',false, ...
    'true_plant_motion_oracle_projection',false, ...
    'controller_friction',cfg.FRIC,'plant_friction',P.fric, ...
    'friction_boundary','external joint/load drag; internal motor drag stays in electrical conversion', ...
    'payload_com_perturbation_meaning','rigid workpiece grasp/placement offset including collision geometry', ...
    'controller_payload',nominal_payload,'plant_payload',true_payload, ...
    'controller_bus',bus_nominal,'plant_bus',P.bus, ...
    'execution_contract',['nominal measured-state safety projection; ', ...
      'unknown plant uses actuator-only saturation and actual-state audit']);
end
