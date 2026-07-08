function bus = get_bus_from_cfg(cfg, P_grid_budget_fallback)
%GET_BUS_FROM_CFG  Build/synchronize revised DC-bus parameter struct from cfg.
%
% Preferred names:
%   cfg.P_GRID_BUDGET    active average/filtered grid-input budget [W]
%   cfg.P_GRID_PEAK_MAX  instantaneous hardware/rating cap [W]
% Legacy names P_TOTAL_MAX/Pmax are treated as budget aliases.
    if nargin < 2 || isempty(P_grid_budget_fallback)
        P_grid_budget_fallback = getfield_def(cfg,'P_GRID_BUDGET', getfield_def(cfg,'P_TOTAL_MAX', inf));
    end
    bus = struct();
    if isfield(cfg,'BUS') && isstruct(cfg.BUS)
        bus = cfg.BUS;
    end

    if isfield(cfg,'P_GRID_BUDGET') && ~isempty(cfg.P_GRID_BUDGET)
        bus.P_grid_budget = double(cfg.P_GRID_BUDGET);
    elseif isfield(cfg,'P_TOTAL_MAX') && ~isempty(cfg.P_TOTAL_MAX)
        bus.P_grid_budget = double(cfg.P_TOTAL_MAX);
    else
        bus.P_grid_budget = double(P_grid_budget_fallback);
    end

    if isfield(cfg,'P_GRID_PEAK_MAX') && ~isempty(cfg.P_GRID_PEAK_MAX)
        bus.P_grid_peak_max = double(cfg.P_GRID_PEAK_MAX);
    end
    if isfield(cfg,'P_HW_RATING') && ~isempty(cfg.P_HW_RATING)
        bus.P_hw_rating = double(cfg.P_HW_RATING);
    elseif isfield(cfg,'P_SUPPLY_RATING') && ~isempty(cfg.P_SUPPLY_RATING)
        bus.P_hw_rating = double(cfg.P_SUPPLY_RATING);
    end
    if isfield(cfg,'V_BUS_NOMINAL') && ~isempty(cfg.V_BUS_NOMINAL)
        bus.V_nom = double(cfg.V_BUS_NOMINAL);
    elseif isfield(cfg,'VBUS_NOMINAL') && ~isempty(cfg.VBUS_NOMINAL)
        bus.V_nom = double(cfg.VBUS_NOMINAL);
    elseif isfield(cfg,'V_BUS') && ~isempty(cfg.V_BUS)
        bus.V_nom = double(cfg.V_BUS);
    end
    if isfield(cfg,'I_SUPPLY_RATING') && ~isempty(cfg.I_SUPPLY_RATING)
        bus.I_rating = double(cfg.I_SUPPLY_RATING);
    elseif isfield(cfg,'IBUS_RATING') && ~isempty(cfg.IBUS_RATING)
        bus.I_rating = double(cfg.IBUS_RATING);
    elseif isfield(cfg,'I_BUS_RATING') && ~isempty(cfg.I_BUS_RATING)
        bus.I_rating = double(cfg.I_BUS_RATING);
    end

    bus.P_total_max = bus.P_grid_budget;       % legacy alias
    bus = spinn3d_bus_defaults(bus);
end

function v = getfield_def(S, fn, def)
if isstruct(S) && isfield(S,fn) && ~isempty(S.(fn)), v = S.(fn); else, v = def; end
end
