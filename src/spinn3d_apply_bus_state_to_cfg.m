function cfg2 = spinn3d_apply_bus_state_to_cfg(cfg, st_bus)
%SPINN3D_APPLY_BUS_STATE_TO_CFG Write explicit bus-state init values into cfg.BUS.

    cfg2 = cfg;
    if nargin < 2 || isempty(st_bus)
        return;
    end
    if ~isfield(cfg2, 'BUS') || ~isstruct(cfg2.BUS)
        cfg2.BUS = struct();
    end
    if isfield(cfg2.BUS,'model_scope') && ...
            strcmp(char(cfg2.BUS.model_scope),'dynamic_capacitor_common_dc_v1')

        cfg2.BUS.dc_initial_state_mode='explicit_measured_state';
        cfg2.BUS.V_init_V=double(st_bus.Vdc);
        cfg2.BUS.Ebuf_init_J=0.5*double(cfg2.BUS.C_dc_F)*double(st_bus.Vdc)^2;
        cfg2.BUS.Ereuse_init_J=0;
        cfg2.BUS.dc_state=st_bus;
        cfg2.BUS.dc_state.Tbrk=0;
        cfg2.BUS.dc_state.Ebuf=cfg2.BUS.Ebuf_init_J;
        cfg2.BUS.dc_state.Ereuse=0; cfg2.BUS.dc_state.Eregen_task=0;
        if isfield(st_bus,'Pgrid_avg_lp') && isscalar(st_bus.Pgrid_avg_lp) && ...
                isfinite(double(st_bus.Pgrid_avg_lp))
            cfg2.BUS.initial_Pgrid_avg_W=double(st_bus.Pgrid_avg_lp);
        elseif isfield(cfg2.BUS,'initial_Pgrid_avg_W')
            cfg2.BUS=rmfield(cfg2.BUS,'initial_Pgrid_avg_W');
        end
        return;
    end
    if isfield(st_bus, 'Ebuf') && ~isempty(st_bus.Ebuf)
        cfg2.BUS.Ebuf_init_J = double(st_bus.Ebuf);
    end
    if isfield(st_bus, 'Ereuse') && ~isempty(st_bus.Ereuse)
        cfg2.BUS.Ereuse_init_J = double(st_bus.Ereuse);
    end
    if isfield(st_bus, 'Tbrk') && ~isempty(st_bus.Tbrk)
        cfg2.BUS.Tbrk_init_C = double(st_bus.Tbrk);
    end
end
