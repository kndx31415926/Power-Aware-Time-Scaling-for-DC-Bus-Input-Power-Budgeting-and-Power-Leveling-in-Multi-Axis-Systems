function [Pavg_next, ok_avg, Pavg_cap, Ppeak_cap, beta_avg] = spinn3d_bus_avg_guard(Pgrid, Pcap, bus, Ts)
%SPINN3D_BUS_AVG_GUARD  Filtered grid-power budget + raw peak cap.
%
% Filtered-power semantics:
%   bus.P_total_max       : filtered grid-power budget [W]
%   bus.P_grid_peak_max   : instantaneous raw-grid-power cap [W]
%   bus.P_avg_window_s    : filtered/low-pass time constant [s]
%   bus.P_avg_target_frac : filtered target/cap as fraction of P_total_max

%                           the current physical-feasibility paths ignore it
%   bus.Pgrid_avg_prev    : previous filtered/averaged Pgrid [W]
%
% The update is a causal first-order filter.  Pavg_next/Pavg_cap are the
% authoritative KPI values; ok_avg must not be folded into current physical
% execution, data-acceptance or paper-success decisions.
%   Pavg_next = beta*Pavg_prev + (1-beta)*Pgrid,
%   beta = exp(-Ts/P_avg_window_s).

    if nargin < 3 || isempty(bus), bus = struct(); end
    ts_missing = nargin < 4 || isempty(Ts);
    if nargin < 2 || isempty(Pcap), Pcap = getfield_def(bus, 'P_total_max', inf); end

    bus = spinn3d_bus_defaults(bus);
    strict_config = isfield(bus,'strict_config') && ...
        logical(bus.strict_config);
    if strict_config

    elseif ts_missing
        Ts = 1e-3;
    end
    if strict_config
        Ts = double(Ts);
    else
        Ts = max(double(Ts), 1e-12);
    end
    Pcap = double(Pcap);
    if strict_config

    end

    if strict_config
        Ppeak_cap = double(bus.P_grid_peak_max);

    else
        Ppeak_cap = getfield_def(bus, 'P_grid_peak_max', Pcap);
        if isempty(Ppeak_cap) || ~isfinite(Ppeak_cap)
            Ppeak_cap = inf;
        end
        Ppeak_cap = double(Ppeak_cap);
    end
    if isfinite(Pcap) && isfinite(Ppeak_cap) && Pcap > Ppeak_cap + 1e-12
        error('spinn3d_bus_avg_guard:BudgetExceedsPhysicalPeak', ...
            ['Filtered budget %.9g W exceeds the physical source peak ', ...
             '%.9g W. The peak limit is never raised implicitly.'], ...
            Pcap,Ppeak_cap);
    end

    if strict_config
        target_frac = double(bus.P_avg_target_frac);

    else
        target_frac = getfield_def(bus, 'P_avg_target_frac', 1.0);
        target_frac = min(max(double(target_frac), 0.0), 1.5);
    end
    Pavg_cap = target_frac * Pcap;
    if ~isfinite(Pcap)
        Pavg_cap = inf;
    end

    if strict_config
        tau_avg = double(bus.P_avg_window_s);

    else
        tau_avg = getfield_def(bus, 'P_avg_window_s', 0.50);
        tau_avg = max(double(tau_avg), 1e-6);
    end
    beta_avg = exp(-Ts / tau_avg);

    if strict_config

        Pavg_prev = double(bus.Pgrid_avg_prev);

        Pgrid_use = double(Pgrid);
    else
        Pavg_prev = getfield_def(bus, 'Pgrid_avg_prev', 0.0);
        if isempty(Pavg_prev) || ~isfinite(Pavg_prev)
            Pavg_prev = 0.0;
        end
        Pgrid_use = max(double(Pgrid),0.0);
    end
    Pavg_next = beta_avg * double(Pavg_prev) + ...
        (1.0 - beta_avg) * Pgrid_use;

    if strict_config
        hard_enable = logical(bus.P_avg_hard_enable);
    else
        hard_enable = logical(getfield_def(bus, ...
            'P_avg_hard_enable', false));
    end
    ok_avg = (~hard_enable) || (~isfinite(Pavg_cap)) || (Pavg_next <= Pavg_cap + 1e-9);
end

function v = getfield_def(S, fn, def)
    if isstruct(S) && isfield(S, fn) && ~isempty(S.(fn))
        v = S.(fn);
    else
        v = def;
    end
end
