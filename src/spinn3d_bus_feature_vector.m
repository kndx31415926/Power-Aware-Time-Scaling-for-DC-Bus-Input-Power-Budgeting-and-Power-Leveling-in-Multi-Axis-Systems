function x = spinn3d_bus_feature_vector(bus, st_bus, Pcap, Pgrid_avg_prev)
%SPINN3D_BUS_FEATURE_VECTOR Normalized bus-state/context feature vector.
% x = [Ebuf_norm, Ebuf_slack, Ereuse_norm, Tbrk_norm, Tbrk_slack,
%      Pbuf_dis_norm, Pbuf_chg_norm, Pbrk_norm, eta_share,
%      Pgrid_avg_norm, Pgrid_avg_slack, Vdc_over_source, Vdc_upper_slack]

% algebraic profiles retain their separate 11-column compatibility layout.
% Voltage and temperature are never clipped: an invalid state stays visible.
%
% The last two entries expose the filtered grid-power state used by
% spinn3d_bus_avg_guard. Older callers can omit Pgrid_avg_prev; in that case
% the value is read from bus.Pgrid_avg_prev / st_bus.Pgrid_avg_lp or defaults
% to zero, preserving backward compatibility while avoiding a hidden-state label.

    if nargin < 1 || isempty(bus)
        bus = struct();
    end
    bus = spinn3d_bus_defaults(bus);

    if nargin < 2 || isempty(st_bus)
        st_bus = spinn3d_bus_state_init(bus);
    end

    if nargin < 3 || isempty(Pcap) || ~isfinite(Pcap) || Pcap <= 0
        Pcap = bus.P_total_max;
    end
    if isempty(Pcap) || ~isfinite(Pcap) || Pcap <= 0
        Pcap = 1.0;
    end

    if nargin < 4 || isempty(Pgrid_avg_prev) || ~isfinite(double(Pgrid_avg_prev))
        Pgrid_avg_prev = getfield_def(st_bus, 'Pgrid_avg_lp', getfield_def(bus, 'Pgrid_avg_prev', 0.0));
    end
    Pgrid_avg_prev = double(Pgrid_avg_prev);

    if strcmp(char(bus.model_scope),'dynamic_capacitor_common_dc_v1')

        
        Emax=max(double(bus.Ebuf_max_J),1e-9); Pnorm=max(double(Pcap),1e-9);
        Ebuf=0.5*double(bus.C_dc_F)*double(st_bus.Vdc)^2;

        Ereuse=0;
        x=double([Ebuf/Emax,(double(bus.Ebuf_max_J)-Ebuf)/Emax,Ereuse/Emax, ...
            clamp(double(bus.Pbuf_dis_max_W)/Pnorm,0,4), ...
            clamp(double(bus.Pbuf_chg_max_W)/Pnorm,0,4), ...
            clamp(double(bus.eta_share),0,1), ...
            Pgrid_avg_prev/Pnorm,(double(Pcap)-Pgrid_avg_prev)/Pnorm, ...
            double(st_bus.Vdc)/double(bus.V_source_V), ...
            (double(bus.V_max_V)-double(st_bus.Vdc))/ ...
            (double(bus.V_max_V)-double(bus.V_min_V))]);
        return;
    end
    Emax  = max(double(bus.Ebuf_max_J), 1e-9);
    Tspan = max(double(bus.Tbrk_max_C) - double(bus.Tamb_C), 1e-9);
    Pnorm = max(double(Pcap), 1e-9);

    dynamic_dc = strcmp(char(bus.model_scope),'dynamic_capacitor_common_dc_v1');
    Ebuf   = clamp(double(getfield_def(st_bus, 'Ebuf',   bus.Ebuf_init_J)), 0.0, bus.Ebuf_max_J);
    Ereuse = clamp(double(getfield_def(st_bus, 'Ereuse', bus.Ereuse_init_J)), 0.0, Ebuf);
    Tbrk   = clamp(double(getfield_def(st_bus, 'Tbrk',   bus.Tbrk_init_C)), bus.Tamb_C, bus.Tbrk_max_C);

    if dynamic_dc

        Ebuf = 0.5*double(bus.C_dc_F)*double(st_bus.Vdc)^2;
        Ereuse = 0;
        Tbrk = double(st_bus.Tbrk);
    end

    Ebuf_norm    = clamp(Ebuf / Emax, 0.0, 1.0);
    Ebuf_slack   = clamp((bus.Ebuf_max_J - Ebuf) / Emax, 0.0, 1.0);
    Ereuse_norm  = clamp(Ereuse / Emax, 0.0, 1.0);
    Tbrk_norm    = clamp((Tbrk - bus.Tamb_C) / Tspan, 0.0, 1.0);
    Tbrk_slack   = clamp((bus.Tbrk_max_C - Tbrk) / Tspan, 0.0, 1.0);
    Pbuf_dis_norm= clamp(double(bus.Pbuf_dis_max_W) / Pnorm, 0.0, 4.0);
    Pbuf_chg_norm= clamp(double(bus.Pbuf_chg_max_W) / Pnorm, 0.0, 4.0);
    Pbrk_norm    = clamp(double(bus.P_brk_peak) / Pnorm, 0.0, 4.0);
    eta_share    = clamp(double(bus.eta_share), 0.0, 1.0);
    Pavg_norm    = clamp(Pgrid_avg_prev / Pnorm, 0.0, 4.0);
    Pavg_slack   = clamp((double(Pcap) - Pgrid_avg_prev) / Pnorm, -4.0, 4.0);

    if dynamic_dc
        Ebuf_norm=Ebuf/Emax; Ebuf_slack=(bus.Ebuf_max_J-Ebuf)/Emax;
        Tbrk_norm=(Tbrk-bus.Tamb_C)/Tspan;
        Tbrk_slack=(bus.Tbrk_max_C-Tbrk)/Tspan;
        Pavg_norm=Pgrid_avg_prev/Pnorm;
        Pavg_slack=(double(Pcap)-Pgrid_avg_prev)/Pnorm;
    end
    x = double([Ebuf_norm, Ebuf_slack, Ereuse_norm, Tbrk_norm, Tbrk_slack, ...
                Pbuf_dis_norm, Pbuf_chg_norm, Pbrk_norm, eta_share, ...
                Pavg_norm, Pavg_slack]);
    if dynamic_dc
        x=[x, double(st_bus.Vdc)/double(bus.V_source_V), ...
            (double(bus.V_max_V)-double(st_bus.Vdc))/ ...
            (double(bus.V_max_V)-double(bus.V_min_V))];
    end
end

function x = getfield_def(S, fn, def)
    if isstruct(S) && isfield(S, fn) && ~isempty(S.(fn))
        x = S.(fn);
    else
        x = def;
    end
end

function y = clamp(x, lo, hi)
    y = min(hi, max(lo, x));
end
