function M = spinn3d_compute_power_metrics(out, Pbudget, opts)
%SPINN3D_COMPUTE_POWER_METRICS Standalone metric helper for paper scripts.
%
% v5 convention:
%   - Raw Pgrid is instantaneous electrical/source telemetry.
%   - The primary paper/control metric is ONE causal first-order LPF:
%       tau_P dPbar/dt + Pbar = Pgrid
%       Pbar_k = lambda Pbar_{k-1} + (1-lambda) Pgrid_k,
%       lambda = exp(-Ts/tau_P).
%   - Default tau_P = 0.05 s.  No moving average and no dual fast/slow
%     evaluation convention are used for the main results.
if nargin < 3 || isempty(opts), opts = struct(); end
assert(isfield(out,'log') && isfield(out.log,'t') && isfield(out.log,'Pgrid'), 'bad output log');

t = out.log.t(:);
P_raw = out.log.Pgrid(:);
if isempty(t) || numel(t)<2
    dt = getfield_def(getfield_def(out,'cfg',struct()),'TS',1e-3);
else
    dt = median(diff(t));
end
if ~isfinite(dt) || dt <= 0, dt = 1e-3; end

rho = getfield_def(opts,'Pbudget_margin_frac',getfield_def(opts,'Pmax_margin_frac',getfield_def(opts,'rho',0.05)));
Pthr = Pbudget*(1+rho);

tau = getfield_def(opts,'power_tau_s', getfield_def(opts,'power_eval_tau_s', 0.05));
% Compatibility: allow old names, but map everything to the single LPF.
if ~isfield(opts,'power_tau_s') && ~isfield(opts,'power_eval_tau_s')
    tau = getfield_def(opts,'power_fast_tau_s', getfield_def(opts,'power_slow_tau_s', tau));
end
if ~isfield(opts,'power_tau_s') && ~isfield(opts,'power_eval_tau_s') && isfield(opts,'power_eval_fc_hz')
    fc = getfield_def(opts,'power_eval_fc_hz',[]);
    if ~isempty(fc) && isfinite(fc) && fc > 0
        tau = 1/max(2*pi*double(fc),eps);
    end
end
tau = max(double(tau),1e-6);
P_lpf = spinn3d_lpf1_causal(P_raw, dt, tau, P_raw(1));

mode = lower(char(getfield_def(opts,'power_eval_mode','lpf')));
if any(strcmp(mode,{'raw','instantaneous'}))
    P_eval = P_raw;
    mode = 'raw';
elseif any(strcmp(mode,{'movmean','avg','ma'}))
    % Retained only for legacy experiments; not recommended for manuscript tables.
    win_s = getfield_def(opts,'power_eval_win_s',tau);
    win = max(1,round(win_s/dt));
    P_eval = movmean_local(P_raw,win);
    mode = 'movmean_legacy';
else
    P_eval = P_lpf;
    mode = 'lpf';
end

if isfield(out,'t_reach') && ~isempty(out.t_reach) && isfinite(out.t_reach)
    t_reach = out.t_reach;
else
    t_reach = t(end);
end

ramp_lpf = diff(P_lpf)/dt;
ramp_eval = diff(P_eval)/dt;
Pdump = zeros(size(P_raw)); if isfield(out.log,'Pdump'), Pdump = out.log.Pdump(:); end

M = struct();
M.t_reach = t_reach;
M.eval_mode = string(mode);
M.Pbudget_W = Pbudget;
M.Pthr_W = Pthr;
M.Pbudget_margin_frac = rho;
M.power_tau_s = tau;
M.power_eval_tau_s = tau;
M.power_lpf_fc_hz = 1/(2*pi*tau);

% Raw and LPF power metrics.
M.P_peak_raw = max(P_raw);
M.P_p99_raw = pctile(P_raw,99);
M.P_peak_lpf = max(P_lpf);
M.P_p99_lpf = pctile(P_lpf,99);
M.P_mean_lpf = mean(P_lpf,'omitnan');
M.P_peak_lpf_norm = M.P_peak_lpf / max(Pbudget,eps);
M.P_p99_lpf_norm  = M.P_p99_lpf  / max(Pbudget,eps);
M.P_mean_lpf_norm = M.P_mean_lpf / max(Pbudget,eps);

% Compatibility aliases for old table/plot code.  In v5 these all refer to the
% same single LPF, not to separate fast/slow monitors.
M.P_peak_fast = M.P_peak_lpf;       M.P_p99_fast = M.P_p99_lpf;
M.P_peak_slow = M.P_peak_lpf;       M.P_p99_slow = M.P_p99_lpf;
M.P_mean_slow = M.P_mean_lpf;
M.P_peak_fast_norm = M.P_peak_lpf_norm; M.P_p99_fast_norm = M.P_p99_lpf_norm;
M.P_peak_slow_norm = M.P_peak_lpf_norm; M.P_p99_slow_norm = M.P_p99_lpf_norm;
M.P_mean_slow_norm = M.P_mean_lpf_norm;
M.Praw_peak_W = M.P_peak_raw;
M.Praw_p99_W = M.P_p99_raw;
M.Plpf_peak_W = M.P_peak_lpf;
M.Plpf_p99_W = M.P_p99_lpf;
M.Plpf_mean_W = M.P_mean_lpf;
M.Pfast_peak_W = M.P_peak_lpf; M.Pfast_p99_W = M.P_p99_lpf;
M.Pslow_peak_W = M.P_peak_lpf; M.Pslow_p99_W = M.P_p99_lpf; M.Pslow_mean_W = M.P_mean_lpf;

% Primary evaluated metrics retained for legacy table code.
M.P_peak = max(P_eval);
M.P_p99 = pctile(P_eval,99);
M.P_peak_norm = M.P_peak / max(Pbudget,eps);
M.P_p99_norm  = M.P_p99  / max(Pbudget,eps);
M.P_mean_norm = mean(P_eval,'omitnan') / max(Pbudget,eps);
M.P_std = std(P_eval,'omitnan');
M.P_std_norm = M.P_std/max(Pbudget,eps);
win_det = max(3, round(getfield_def(opts,'detrend_win_s',0.20)/max(dt,eps)));
P_det = P_eval - movmean_local(P_eval, win_det);
M.P_std_detrended = std(P_det,'omitnan');
M.P_rms_to_Pbudget = sqrt(mean((P_eval-Pbudget).^2,'omitnan'));
M.P_rms_to_Pmax = M.P_rms_to_Pbudget;

% Exceedance.
M.E_excess = trapz(t,max(P_eval-Pthr,0));
M.E_excess_norm_s = M.E_excess / max(Pbudget,eps);
M.E_excess_lpf = trapz(t,max(P_lpf-Pthr,0));
M.E_excess_lpf_norm_s = M.E_excess_lpf / max(Pbudget,eps);
M.E_excess_slow = M.E_excess_lpf; M.E_excess_slow_norm_s = M.E_excess_lpf_norm_s;
M.E_excess_fast = M.E_excess_lpf; M.E_excess_fast_norm_s = M.E_excess_lpf_norm_s;

dt_vec = diff(t);
if isempty(dt_vec)
    M.T_excess = 0; M.T_excess_lpf = 0; M.T_excess_slow = 0; M.r_viol = 0; M.r_viol_lpf = 0; M.r_viol_slow = 0;
else
    M.T_excess = sum(dt_vec.*double(P_eval(1:end-1)>Pthr));
    M.T_excess_lpf = sum(dt_vec.*double(P_lpf(1:end-1)>Pthr));
    M.T_excess_slow = M.T_excess_lpf;
    M.r_viol = M.T_excess/max(sum(dt_vec),eps);
    M.r_viol_lpf = M.T_excess_lpf/max(sum(dt_vec),eps);
    M.r_viol_slow = M.r_viol_lpf;
end

M.ramp_rms = sqrt(mean(ramp_eval.^2,'omitnan'));
M.ramp_p95 = pctile(abs(ramp_eval),95);
M.ramp_max = max(abs(ramp_eval));
M.ramp_p95_lpf = pctile(abs(ramp_lpf),95);
M.ramp_max_lpf = max(abs(ramp_lpf));
M.ramp_p95_slow = M.ramp_p95_lpf; M.ramp_p95_fast = M.ramp_p95_lpf;
M.ramp_max_slow = M.ramp_max_lpf; M.ramp_max_fast = M.ramp_max_lpf;

% Normalized ramp-rate metrics.  The manuscript tables report
% d(Pgrid/Pbudget)/dt with units s^-1, not W/s.  Keep the W/s fields above
% for diagnostics and add explicit normalized aliases for paper tables.
M.ramp_rms_norm = M.ramp_rms / max(Pbudget,eps);
M.ramp_p95_norm = M.ramp_p95 / max(Pbudget,eps);
M.ramp_max_norm = M.ramp_max / max(Pbudget,eps);
M.ramp_p95_lpf_norm = M.ramp_p95_lpf / max(Pbudget,eps);
M.ramp_max_lpf_norm = M.ramp_max_lpf / max(Pbudget,eps);
M.ramp_p95_slow_norm = M.ramp_p95_lpf_norm; M.ramp_p95_fast_norm = M.ramp_p95_lpf_norm;
M.ramp_max_slow_norm = M.ramp_max_lpf_norm; M.ramp_max_fast_norm = M.ramp_max_lpf_norm;

% Compact pbar aliases used by revision tables.
M.pbar_peak = M.P_peak_norm;
M.pbar_p99 = M.P_p99_norm;
M.pbar_std = M.P_std_norm;
M.pbar_lpf_peak = M.P_peak_lpf_norm;
M.pbar_lpf_p99 = M.P_p99_lpf_norm;
M.pbar_ramp_p95 = M.ramp_p95_norm;
M.pbar_ramp_max = M.ramp_max_norm;

% Energy/current metrics use raw telemetry.
M.E_grid = trapz(t,max(P_raw,0));
M.E_dump = trapz(t,max(Pdump,0));
% Reused energy is defined as integral of the reuse power Pshare/Preuse.
% Older logs stored log.Ereuse as a per-step increment, which makes
% er(end)-er(1) wrong.  Prefer the logged reuse power when available; only use
% cumulative-energy fields as a fallback.
if isfield(out.log,'Preuse_total') && ~isempty(out.log.Preuse_total)
    M.E_reuse = trapz(t,max(out.log.Preuse_total(:),0));
elseif isfield(out.log,'Ereuse_cum') && ~isempty(out.log.Ereuse_cum)
    er = out.log.Ereuse_cum(:); M.E_reuse = er(end);
elseif isfield(out.log,'Ereuse') && ~isempty(out.log.Ereuse)
    er = out.log.Ereuse(:);
    if all(diff(er(isfinite(er))) >= -1e-12)
        M.E_reuse = er(end);             % legacy cumulative log sampled after each step
    else
        M.E_reuse = sum(max(er,0),'omitnan'); % legacy per-step energy increment [J]
    end
else
    M.E_reuse = NaN;
end
if isfield(out.log,'Pi_raw')
    Pi = out.log.Pi_raw;
    M.E_mech_pos = trapz(t,sum(max(Pi,0),2));
    M.E_mech_regen = trapz(t,sum(max(-Pi,0),2));
else
    M.E_mech_pos = NaN; M.E_mech_regen = NaN;
end
if isfield(out.log,'Igrid'), I = out.log.Igrid(:); else, I = P_raw/max(12,eps); end
M.I_rms = sqrt(mean(I.^2,'omitnan'));
M.I_peak = max(abs(I));
if isfield(out.log,'Vdc'), M.Vdc_min = min(out.log.Vdc(:)); else, M.Vdc_min = NaN; end
if isfield(out.log,'Ploss'), M.P_loss_energy = trapz(t,max(out.log.Ploss(:),0)); else, M.P_loss_energy = NaN; end
if isfield(out.log,'Pspill'), M.E_spill = trapz(t,max(out.log.Pspill(:),0)); else, M.E_spill = NaN; end
if isfield(out.log,'Pgrid_request')
    Preq = out.log.Pgrid_request(:);
    M.P_request_peak = max(Preq,[],'omitnan');
    M.P_request_p99 = pctile(Preq,99);
    M.E_grid_request = trapz(t,max(Preq,0));
else
    M.P_request_peak = NaN; M.P_request_p99 = NaN; M.E_grid_request = NaN;
end
if isfield(out.log,'Psource_shortfall')
    Psf = out.log.Psource_shortfall(:);
    M.Psource_shortfall_peak = max(Psf,[],'omitnan');
    M.E_source_shortfall = trapz(t,max(Psf,0));
else
    M.Psource_shortfall_peak = NaN; M.E_source_shortfall = NaN;
end
if isfield(out.log,'Psource_limited')
    M.source_limited_frac = mean(double(out.log.Psource_limited(:))>0,'omitnan');
else
    M.source_limited_frac = NaN;
end
end

function y = movmean_local(x,win)
if exist('movmean','file')==2, y=movmean(x(:),win); else, y=conv(x(:),ones(win,1)/win,'same'); end
end
function p = pctile(x,q)
x=x(isfinite(x)); x=sort(x(:)); if isempty(x), p=NaN; return; end
q=max(0,min(100,q)); pos=1+(numel(x)-1)*q/100; lo=floor(pos); hi=ceil(pos);
if lo==hi, p=x(lo); else, p=x(lo)+(pos-lo)*(x(hi)-x(lo)); end
end
function x = getfield_def(S,fn,def)
if isstruct(S) && isfield(S,fn) && ~isempty(S.(fn)), x=S.(fn); else, x=def; end
end
