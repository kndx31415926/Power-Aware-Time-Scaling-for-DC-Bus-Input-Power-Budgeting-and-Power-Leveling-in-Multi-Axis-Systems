function cfg=reviewer_configuration(root,entry,study,case_id)
% Load a recorded task and apply the prescribed physical perturbation.
cfg=entry.cfg;cfg.ROBOT=copy(entry.cfg.ROBOT);
modeldir=fullfile(root,'models','r8_eq');
cfg.MODEL_PATH=fullfile(modeldir,'trf_phy.mat');
cfg.MODEL_PATH_ALPHA=cfg.MODEL_PATH;
cfg.MODEL_PATH_ALPHA_MLP=fullfile(modeldir,'mlp_phy.mat');
cfg.MODEL_PATH_ALPHA_SUPERVISED=fullfile(modeldir,'trf_sup.mat');
cfg.MODEL_PATH_ALPHA_MLP_SUPERVISED=fullfile(modeldir,'mlp_sup.mat');
if isfield(cfg,'MODEL_SHA256'),cfg=rmfield(cfg,'MODEL_SHA256');end
cfg.ALPHA_EXPECTED_MODEL_ROLE='final';
cfg.REVISION_R6=struct('enabled',false);
cfg.DISABLE_PLOT=true;cfg.PLOT.enable=false;
if strcmp(study,'main')
    cfg.PTP_R2.Ns=61;cfg.PTP_R2.Na=49;cfg.PTP_R2.beamN=32;
    return;
end
C=reviewer_conditions(study);C=C(case_id);original=cfg;
cfg.NOISE.sigma_q=0;cfg.NOISE.sigma_dq=0;cfg.NOISE.tau_std=0;
cfg.NOISE.rng_mode='local';cfg.NOISE.seed=entry.noise_seed;
if strcmp(study,'robustness')
    D=entry.unknown_draw;
    O=struct('FRIC',cfg.FRIC,'PAYLOAD',cfg.PAYLOAD,'BUS',cfg.BUS);
    if any(strcmp(C.family,{'friction','combined'}))
        O.FRIC.B=cfg.FRIC.B*D.friction_scale;O.FRIC.Fc=cfg.FRIC.Fc*D.friction_scale;
    end
    if any(strcmp(C.family,{'payload_com','combined'}))
        O.PAYLOAD.DEMO.mass=cfg.PAYLOAD.DEMO.mass*D.payload_mass_inertia_scale;
        O.PAYLOAD.DEMO.inertia=cfg.PAYLOAD.DEMO.inertia*D.payload_mass_inertia_scale;
        O.PAYLOAD.DEMO.com=cfg.PAYLOAD.DEMO.com+D.payload_com_delta_m;
    end
    if any(strcmp(C.family,{'voltage','combined'}))
        O.BUS.V_source_V=D.source_voltage_V;
        O.BUS.P_grid_peak_max=min(cfg.BUS.P_grid_peak_max,D.source_voltage_V*cfg.BUS.I_supply_max_A);
        O.BUS=spinn3d_bus_defaults(O.BUS);
    end
    if any(strcmp(C.family,{'joint_noise','combined'}))
        cfg.NOISE.sigma_q=1e-4*C.value;cfg.NOISE.sigma_dq=1e-4*C.value;
        cfg.NOISE.tau_std=1e-4*C.value;
    end
    cfg.PLANT_OVERRIDE=O;
    interpretation='unknown_true_parameters_only_nominal_control_unchanged';
else
    if ~strcmp(C.family,'nominal')
        if strcmp(C.assignment,'multiply_nominal')
            cfg.BUS.(C.parameter)=original.BUS.(C.parameter)*C.value;
        else
            cfg.BUS.(C.parameter)=C.value;
        end
        if strcmp(C.parameter,'V_source_V')
            cfg.BUS.P_grid_peak_max=min(original.BUS.P_grid_peak_max, ...
                cfg.BUS.V_source_V*cfg.BUS.I_supply_max_A);
        end
        cfg.BUS=spinn3d_bus_defaults(cfg.BUS);
        cfg.CAPS.bus=cfg.BUS;cfg.CAPS.P_grid_peak_max=cfg.BUS.P_grid_peak_max;
        if strcmp(C.family,'filter'),cfg.CAPS.P_avg_window_s=cfg.BUS.P_avg_window_s;end
    end
    cfg.PLANT_OVERRIDE=struct('FRIC',cfg.FRIC,'PAYLOAD',cfg.PAYLOAD,'BUS',cfg.BUS);
    interpretation='known_parameters_nominal_and_true_synchronized_zero_added_noise';
end
cfg.REVISION_PERTURBATION=struct('mode','unknown_plant','interpretation',interpretation);
end
