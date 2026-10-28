function C = spinn3d_fair_time_contract(cfg, t_candidate, t_noguard_fallback)
%SPINN3D_FAIR_TIME_CONTRACT Shared NoGuard-relative arrival-time contract.
%   C = spinn3d_fair_time_contract(cfg, t_candidate, t_noguard_fallback)
%
% Every optimized/guarded method is acceptable only when
%
%   t_candidate <= t_NoGuard + cfg.FAIRNESS.max_delay_vs_noguard_s .
%
% paper_power_eval injects the paired, full-plant NoGuard arrival time into
% cfg.FAIRNESS.t_reach_noguard_s. Public stand-alone demos obtain this by

% a missing measured reference.

if nargin < 1 || ~isstruct(cfg), cfg = struct(); end
if nargin < 2 || isempty(t_candidate), t_candidate = NaN; end
if nargin < 3 || isempty(t_noguard_fallback), t_noguard_fallback = NaN; end

F = struct();
if isfield(cfg,'FAIRNESS') && isstruct(cfg.FAIRNESS)
    F = cfg.FAIRNESS;
end

margin = double(F.max_delay_vs_noguard_s);

t_ng = getdef_local(F,'t_reach_noguard_s',NaN);
source = 'paired-full-plant';
independent_missing=logical(getdef_local(F,'revision_independent_failure_eval',false)) && ...
    ~logical(getdef_local(F,'noguard_reference_available',false));
if independent_missing
    t_ng=NaN;t_noguard_fallback=NaN;
end
if ~isnumeric(t_ng) || ~isscalar(t_ng) || ~isfinite(double(t_ng)) || double(t_ng) <= 0
    t_ng = t_noguard_fallback;
    source = 'nominal-cartesian-proxy';
end
if ~isnumeric(t_ng) || ~isscalar(t_ng) || ~isfinite(double(t_ng)) || double(t_ng) <= 0
    t_ng = NaN;
    source = 'missing';
else
    t_ng = double(t_ng);
end

if isfinite(t_ng)
    deadline = t_ng + margin;
else
    deadline = NaN;
end

if isnumeric(t_candidate) && isscalar(t_candidate) && isfinite(double(t_candidate)) && double(t_candidate) > 0
    t_candidate = double(t_candidate);
    delay = t_candidate - t_ng;
    pass = isfinite(deadline) && t_candidate <= deadline + 1e-9;
else
    t_candidate = NaN;
    delay = NaN;
    pass = false;
end

C = struct('schema','noguard_configured_margin_v2', ...
           'max_delay_vs_noguard_s',margin, ...
           't_reach_noguard_s',t_ng, ...
           'deadline_s',deadline, ...
           't_candidate_s',t_candidate, ...
           'delay_vs_noguard_s',delay, ...
           'pass',logical(pass), ...
           'reference_source',source, ...
           'reference_available',isfinite(t_ng));
end

function v = getdef_local(S,fn,def)
if isstruct(S) && isfield(S,fn) && ~isempty(S.(fn))
    v = S.(fn);
else
    v = def;
end
end
