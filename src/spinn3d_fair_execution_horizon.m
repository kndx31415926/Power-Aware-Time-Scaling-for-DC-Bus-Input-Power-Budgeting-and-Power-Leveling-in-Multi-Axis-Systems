function [Tcap, info] = spinn3d_fair_execution_horizon(cfg, traj_Tf, deadline_s)

%
% The paired NoGuard+margin value is an acceptance deadline, not a simulator
% truncation time.  Guarded methods are allowed to continue to a common,

% t_reach.  Acceptance is still decided by spinn3d_fair_time_contract.

Ts = positive_scalar_local(cfg.TS,NaN);

traj_Tf = positive_scalar_local(traj_Tf,Ts);
alpha_floor = positive_scalar_local(cfg.ALPHA_MIN,NaN);

alpha_floor = min(alpha_floor,1.0);

H = cfg.EXECUTION_HORIZON;

% Worst nominal progress time at alpha_floor, plus a common settling and
% sub-floor recovery reserve used by every guarded comparison method.
floor_progress_s = traj_Tf/max(alpha_floor,1e-6);
settle_s = nonnegative_scalar_local( ...
    getdef(H,'settle_allowance_s',NaN),'settle_allowance_s');
recovery_min_s = nonnegative_scalar_local( ...
    getdef(H,'recovery_allowance_min_s',NaN),'recovery_allowance_min_s');
recovery_factor = nonnegative_scalar_local( ...
    getdef(H,'recovery_allowance_factor',NaN),'recovery_allowance_factor');
recovery_s = max(recovery_min_s,recovery_factor*floor_progress_s);
diagnostic_s = floor_progress_s + settle_s + recovery_s;

absolute_cap_s = positive_scalar_local( ...
    getdef(H,'absolute_cap_s',NaN),NaN);
if isstruct(cfg) && isfield(cfg,'THETA_TRAIN') && isstruct(cfg.THETA_TRAIN)
    absolute_cap_s = positive_scalar_local( ...
        getdef(cfg.THETA_TRAIN,'max_t_reach_s',absolute_cap_s),absolute_cap_s);
end
diagnostic_s = min(diagnostic_s,absolute_cap_s);

if ~isnumeric(deadline_s) || ~isscalar(deadline_s) || ~isfinite(double(deadline_s)) || deadline_s <= 0
    deadline_s = traj_Tf;
else
    deadline_s = double(deadline_s);
end

Tcap = max(deadline_s+Ts,diagnostic_s);
Tcap = max(Ts,ceil(Tcap/Ts)*Ts);

info = struct('deadline_s',deadline_s,'floor_progress_s',floor_progress_s, ...
    'settle_s',settle_s,'recovery_s',recovery_s, ...
    'absolute_cap_s',absolute_cap_s,'Tcap_s',Tcap, ...
    'acceptance_separate_from_horizon',true);
end

function v = positive_scalar_local(v,def)
if ~isnumeric(v) || ~isscalar(v) || ~isfinite(double(v)) || double(v) <= 0
    v = def;
else
    v = double(v);
end
end

function v = nonnegative_scalar_local(v,name)

v = double(v);
end

function v = getdef(S,field,def)
if isstruct(S) && isfield(S,field) && ~isempty(S.(field))
    v = S.(field);
else
    v = def;
end
end
