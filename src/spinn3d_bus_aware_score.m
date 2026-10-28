function [J, terms] = spinn3d_bus_aware_score(metrics, W, opt)
%SPINN3D_BUS_AWARE_SCORE Unified bus-aware objective for alpha/joint planners.
%
% metrics fields are expected in raw physical units unless otherwise noted:
%   t_reach / cycle_time, Egrid, Edump, Ereuse_total, Espill,
%   peakPgrid / grid_peak / Pgrid_peak,
%   peakPdump / dump_peak / Pdump_peak,
%   peakPspill / spill_peak / Pspill_peak,
%   over_int (raw integral) or over_budget_term (normalized),
%   util_mean, util_peak, pavg_util_mean, pavg_under_term, pavg_over_term, Ebuf_min,
%   pgrid_var_term, dpdt_term, jerk_term, cap_loss_term, headroom_term,
%   theta_dev, rho_smooth, end_error, reached.
%
% opt fields:
%   Pcap, Tref, bus, use_state, max_time_factor,
%   buf_floor_frac, incomplete_penalty.
%
% Weight aliases supported for compatibility:
%   time <-> cycle
%   reuse_credit <-> share_reward

    

    W = normalize_weights(W);
    required_opt = {'Pcap','Tref','bus','use_state', ...
        'max_time_factor','buf_floor_frac','incomplete_penalty'};

    Pcap = double(opt.Pcap);
    Tref = double(opt.Tref);
    bus = opt.bus;
    use_state_raw = opt.use_state;
    max_time_factor = double(opt.max_time_factor);
    buf_floor_frac = double(opt.buf_floor_frac);
    incomplete_penalty = double(opt.incomplete_penalty);

    

    
    use_state = logical(use_state_raw);

    

    t_reach      = get_metric(metrics, {'t_reach','cycle_time'}, Tref);
    Egrid        = get_metric(metrics, {'Egrid','grid_energy'}, 0.0);
    Edump        = get_metric(metrics, {'Edump','dump_energy'}, 0.0);
    Ereuse_total = get_metric(metrics, {'Ereuse_total','reuse_energy'}, 0.0);
    Espill       = get_metric(metrics, {'Espill','spill_energy'}, 0.0);

    peakPgrid  = get_metric(metrics, {'peakPgrid','grid_peak','Pgrid_peak'}, 0.0);
    peakPdump  = get_metric(metrics, {'peakPdump','dump_peak','Pdump_peak'}, 0.0);
    peakPspill = get_metric(metrics, {'peakPspill','spill_peak','Pspill_peak'}, 0.0);

    % squared positive EMA excess, already normalized by the target power.
    % Prefer the explicit causal-EMA rollout metric.  The raw-integral path

    over_budget_term = getdef(metrics, 'pavg_over_term', []);
    if isempty(over_budget_term)
        over_budget_term = getdef(metrics, 'over_budget_term', []);
    end
    if isempty(over_budget_term)
        over_int = getdef(metrics, 'over_int', 0.0);
        over_budget_term = over_int / max(Tref, 1e-12);
    end

    reuse_ratio = getdef(metrics, 'reuse_ratio', []);
    if isempty(reuse_ratio)
        reuse_ratio = Ereuse_total / max(Ereuse_total + Edump, 1e-12);
    end

    util_mean = getdef(metrics, 'util_mean', 0.0);
    util_peak = getdef(metrics, 'util_peak', 0.0);
    pavg_util_mean  = getdef(metrics, 'pavg_util_mean', util_mean);
    pavg_under_term = getdef(metrics, 'pavg_under_term', max(1.0 - pavg_util_mean, 0.0)^2);
    pavg_over_term  = getdef(metrics, 'pavg_over_term', 0.0);

    theta_dev  = getdef(metrics, 'theta_dev', 0.0);
    rho_smooth = getdef(metrics, 'rho_smooth', 0.0);
    end_error  = getdef(metrics, 'end_error', 0.0);
    reached    = logical(getdef(metrics, 'reached', true));

    pgrid_var_term = getdef(metrics, 'pgrid_var_term', 0.0);
    dpdt_term      = getdef(metrics, 'dpdt_term', 0.0);
    jerk_term      = getdef(metrics, 'jerk_term', 0.0);
    cap_loss_term  = getdef(metrics, 'cap_loss_term', 0.0);
    headroom_term  = getdef(metrics, 'headroom_term', 0.0);

    pcap_safe = max(Pcap, 1e-12);
    time_term   = t_reach / max(Tref, 1e-12);
    egrid_term  = Egrid / max(Pcap*Tref, 1e-12);
    edump_term  = Edump / max(Pcap*Tref, 1e-12);
    ereuse_term = Ereuse_total / max(Pcap*Tref, 1e-12);
    spill_term  = Espill / max(Pcap*Tref, 1e-12);
    Pgrid_peak_ref = getdef(bus, 'P_grid_peak_max', pcap_safe);
    if ~isfinite(Pgrid_peak_ref) || Pgrid_peak_ref <= 0
        Pgrid_peak_ref = pcap_safe;
    end
    grid_peak_term = max(0.0, peakPgrid / max(Pgrid_peak_ref,1e-12) - 1.0);

    Pdump_ref = getdef(bus, 'P_brk_peak', pcap_safe);
    if ~isfinite(Pdump_ref) || Pdump_ref <= 0
        Pdump_ref = max([peakPdump, peakPspill, pcap_safe, 1.0]);
    end
    dump_peak_term  = peakPdump / max(Pdump_ref, 1e-12);
    spill_peak_term = peakPspill / max(Pdump_ref, 1e-12);

    buf_term   = 0.0;
    if use_state
        Emax = getdef(bus, 'Ebuf_max_J', 0.0);
        Ebuf_min = getdef(metrics, 'Ebuf_min', Emax);
        if Emax > 0
            buf_term = max((buf_floor_frac * Emax - Ebuf_min) / Emax, 0.0)^2;
        end
    end

    J = 0.0;
    J = J + W.time         * time_term;
    J = J + W.over_budget  * over_budget_term;
    J = J + W.grid_energy  * egrid_term;
    J = J + W.dump_energy  * edump_term;
    J = J + W.spill_energy * spill_term;
    J = J - W.reuse_credit * ereuse_term;
    J = J + W.grid_peak    * grid_peak_term^2;
    J = J + W.dump_peak    * dump_peak_term^2;
    J = J + W.spill_peak   * spill_peak_term^2;
    J = J + W.buf_floor    * buf_term;
    J = J + W.pgrid_var    * pgrid_var_term;
    J = J + W.dpdt         * dpdt_term;
    J = J + W.jerk         * jerk_term;
    J = J + W.cap_loss     * cap_loss_term;
    J = J + W.headroom     * headroom_term;
    J = J + W.avg_under    * pavg_under_term;
    J = J + W.avg_over     * pavg_over_term;
    J = J + W.theta_dev    * theta_dev;
    J = J + W.rho_smooth   * rho_smooth;
    J = J + W.end_error    * end_error;
    J = J - W.util_reward      * util_mean;
    J = J - W.avg_util_reward  * pavg_util_mean;

    if isfinite(max_time_factor) && t_reach > max_time_factor * Tref + 1e-9
        J = J + incomplete_penalty * (t_reach / max(Tref,1e-12) - max_time_factor)^2;
    end
    if ~reached
        J = J + incomplete_penalty;
    end

    terms = struct();
    terms.time_term        = time_term;
    terms.over_budget_term = over_budget_term;
    terms.soft_pavg_metric_schema = ...
        'mean_normalized_squared_pavg_excess_v1';
    terms.grid_energy_term = egrid_term;
    terms.dump_energy_term = edump_term;
    terms.spill_energy_term = spill_term;
    terms.reuse_term       = ereuse_term;
    terms.grid_peak_term   = grid_peak_term;
    terms.dump_peak_term   = dump_peak_term;
    terms.spill_peak_term  = spill_peak_term;
    terms.buf_floor_term   = buf_term;
    terms.pgrid_var_term   = pgrid_var_term;
    terms.dpdt_term        = dpdt_term;
    terms.jerk_term        = jerk_term;
    terms.cap_loss_term    = cap_loss_term;
    terms.headroom_term    = headroom_term;
    terms.theta_dev_term   = theta_dev;
    terms.rho_smooth_term  = rho_smooth;
    terms.end_error_term   = end_error;
    terms.util_mean        = util_mean;
    terms.util_peak        = util_peak;
    terms.pavg_util_mean   = pavg_util_mean;
    terms.pavg_under_term  = pavg_under_term;
    terms.pavg_over_term   = pavg_over_term;
    terms.reuse_ratio      = reuse_ratio;
    terms.objective        = J;
end

function W = normalize_weights(W)
    required = {'time','over_budget','grid_energy','dump_energy', ...
        'spill_energy','reuse_credit','grid_peak','dump_peak', ...
        'spill_peak','buf_floor','pgrid_var','dpdt','jerk', ...
        'cap_loss','headroom','theta_dev','rho_smooth','end_error', ...
        'util_reward','avg_under','avg_over','avg_util_reward'};

    for i=1:numel(required)
        value = W.(required{i});

    end
end

function v = get_metric(S, names, def)
    v = def;
    for i = 1:numel(names)
        if isstruct(S) && isfield(S, names{i}) && ~isempty(S.(names{i}))
            v = S.(names{i});
            return;
        end
    end
end

function v = getdef(S, field, def)
    if isstruct(S) && isfield(S, field) && ~isempty(S.(field))
        v = S.(field);
    else
        v = def;
    end
end
