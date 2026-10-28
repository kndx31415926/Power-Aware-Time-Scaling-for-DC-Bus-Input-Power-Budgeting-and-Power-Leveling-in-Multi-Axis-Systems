function bus = get_bus_from_cfg(cfg, P_total_max_fallback)
%GET_BUS_FROM_CFG  Build a unified BUS struct from cfg.BUS with defaults.
%
% Bus fields:
%   P_total_max, eta_share, P_brk_peak
%
% Explicit-state fields:
%   enable_state
%   Ebuf_max_J, Ebuf_init_J, Ereuse_init_J
%   Pbuf_chg_max_W, Pbuf_dis_max_W
%   eta_buf_chg, eta_buf_dis
%   Tamb_C, Tbrk_init_C, Tbrk_max_C
%   Cth_brk_J_per_C, Rth_brk_C_per_W
%
% Supported aliases:
%   P_dump_max <-> P_brk_peak

    if nargin < 2 || isempty(P_total_max_fallback)
        P_total_max_fallback = getfield_def(cfg, 'P_TOTAL_MAX', inf);
    end

    if isfield(cfg, 'BUS') && isstruct(cfg.BUS)
        % Use the supplied bus settings.

        
        bus = cfg.BUS;
    else
        bus = struct('P_total_max', double(P_total_max_fallback));
    end

    % The explicit function argument is the already selected central/trial
    % budget (for example a centrally sampled paper task).  Apply it after
    % merging cfg.BUS so PB.bus.P_total_max remains a compatibility copy of
    % PB.power.P_TOTAL_MAX rather than masking the selected trial budget.
    if ~isempty(P_total_max_fallback)
        bus.P_total_max = double(P_total_max_fallback);
    end

    bus = spinn3d_bus_defaults(bus);
end

function v = getfield_def(S, fn, def)
    if isstruct(S) && isfield(S, fn) && ~isempty(S.(fn))
        v = S.(fn);
    else
        v = def;
    end
end
