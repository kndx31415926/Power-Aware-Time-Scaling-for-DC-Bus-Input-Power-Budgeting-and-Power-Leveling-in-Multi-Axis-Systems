function V = spinn3d_joint_limit_viability( ...
        q,dq,q_min,q_max,qdd_limit,contract,position_tolerance_rad)

%
% V = spinn3d_joint_limit_viability(q,dq,q_min,q_max,qdd_limit, ...
%       contract,position_tolerance_rad)
%
% q and dq are N-by-nJ executed states.  A state is viable only when every
% outward joint velocity can be stopped, under the declared acceleration
% magnitude limit, before the corresponding position boundary.  The
% stopping reserve is the standard constant-acceleration distance
%
%                         d_stop = v_out^2/(2*a_max).
%
% Callers remain responsible for checking the current integration step
% itself (including its midpoint-integrated endpoint).  This helper checks
% the successor state's braking reserve and supports both one-row online

% position_tolerance_rad and numeric_tolerance_rad are acceptance tolerances

% must use the physical q limits and nonpositive raw braking shortage, and
% therefore must not consume either tolerance as planned motion headroom.

if nargin~=7
    error('spinn3d_joint_limit_viability:BadInputCount', ...
        ['Expected q, dq, q_min, q_max, qdd_limit, the central ', ...
         'joint-limit-viability contract and position tolerance.']);
end
if ~isnumeric(q) || ~isnumeric(dq) || ~ismatrix(q) || ...
        ~isequal(size(q),size(dq)) || isempty(q) || size(q,2)<1 || ...
        ~isreal(q) || ~isreal(dq)
    error('spinn3d_joint_limit_viability:BadState', ...
        'q and dq must be equally sized, nonempty, real N-by-nJ matrices.');
end
q = double(q);
dq = double(dq);
[nS,nJ] = size(q);

qlo = expand_signed_limit_local(q_min,nJ,'q_min');
qhi = expand_signed_limit_local(q_max,nJ,'q_max');
alim = expand_nonnegative_limit_local(qdd_limit,nJ,'qdd_limit');
if any(qlo>qhi)
    error('spinn3d_joint_limit_viability:BadPositionLimits', ...
        'Every lower joint limit must be no greater than its upper limit.');
end

required_contract = {'schema','enable','reserve_rad', ...
    'numeric_tolerance_rad','bisection_iterations', ...
    'acceleration_tolerance_rad_s2'};
if ~isstruct(contract) || ~isscalar(contract) || ...
        ~all(isfield(contract,required_contract))
    error('spinn3d_joint_limit_viability:MissingContract', ...
        ['The central joint-limit-viability contract must define schema, ', ...
         'enable, reserve_rad and numeric_tolerance_rad.']);
end
schema = char(string(contract.schema));
if ~strcmp(schema,'discrete_braking_viability_v4')
    error('spinn3d_joint_limit_viability:BadSchema', ...
        'Unsupported joint-limit-viability schema `%s`.',schema);
end
if ~(islogical(contract.enable) && isscalar(contract.enable))
    error('spinn3d_joint_limit_viability:BadEnable', ...
        'joint_limit_viability.enable must be one logical scalar.');
end
enabled = logical(contract.enable);
reserve = nonnegative_scalar_local(contract.reserve_rad,'reserve_rad');
numeric_tol = nonnegative_scalar_local( ...
    contract.numeric_tolerance_rad,'numeric_tolerance_rad');
bound_iterations = positive_integer_local( ...
    contract.bisection_iterations,'bisection_iterations');
acceleration_tol = nonnegative_scalar_local( ...
    contract.acceleration_tolerance_rad_s2, ...
    'acceleration_tolerance_rad_s2');
position_tol = nonnegative_scalar_local( ...
    position_tolerance_rad,'position_tolerance_rad');

finite_state = isfinite(q) & isfinite(dq);
v_upper = max(dq,0);
v_lower = max(-dq,0);
d_upper = braking_distance_local(v_upper,alim);
d_lower = braking_distance_local(v_lower,alim);

qlo_grid = repmat(qlo,nS,1);
qhi_grid = repmat(qhi,nS,1);
available_upper = qhi_grid-q;
available_lower = q-qlo_grid;
required_upper = d_upper+reserve;
required_lower = d_lower+reserve;

% position_tolerance_rad is the same centrally owned tolerance used by the

% roundoff in this derived stopping-distance comparison.
upper_shortage_raw = required_upper-(available_upper+position_tol);
lower_shortage_raw = required_lower-(available_lower+position_tol);
upper_shortage_raw(:,~isfinite(qhi)) = -inf;
lower_shortage_raw(:,~isfinite(qlo)) = -inf;
upper_shortage_raw(~finite_state) = inf;
lower_shortage_raw(~finite_state) = inf;

upper_reserve = -upper_shortage_raw;
lower_reserve = -lower_shortage_raw;
minimum_reserve = min([lower_reserve(:);upper_reserve(:)]);
if isempty(minimum_reserve), minimum_reserve=-inf; end

upper_violation = max(upper_shortage_raw-numeric_tol,0);
lower_violation = max(lower_shortage_raw-numeric_tol,0);
if ~enabled
    % The central switch is explicit experiment metadata.  Disabled means
    % that this helper reports the reserve evidence without rejecting a
    % finite state; ordinary q-position checks remain the caller's duty.
    upper_violation(:) = 0;
    lower_violation(:) = 0;
end

all_violation = [lower_violation(:);upper_violation(:)];
[violation,linear_index] = max(all_violation);
if isempty(violation), violation=inf; linear_index=1; end

offending_sample = 0;
offending_joint = 0;
offending_side = 'none';
offending_velocity = 0;
offending_stopping_distance = 0;
offending_available_margin = inf;
if violation>0
    side_count = nS*nJ;
    if linear_index<=side_count
        [offending_sample,offending_joint] = ind2sub( ...
            [nS,nJ],linear_index);
        offending_side = 'lower';
        offending_velocity = v_lower(offending_sample,offending_joint);
        offending_stopping_distance = ...
            d_lower(offending_sample,offending_joint);
        offending_available_margin = ...
            available_lower(offending_sample,offending_joint);
    else
        upper_index = linear_index-side_count;
        [offending_sample,offending_joint] = ind2sub( ...
            [nS,nJ],upper_index);
        offending_side = 'upper';
        offending_velocity = v_upper(offending_sample,offending_joint);
        offending_stopping_distance = ...
            d_upper(offending_sample,offending_joint);
        offending_available_margin = ...
            available_upper(offending_sample,offending_joint);
    end
end

V = struct( ...
    'schema',schema, ...
    'enabled',enabled, ...
    'pass',logical(all(finite_state,'all') && violation<=0), ...
    'finite_state',logical(all(finite_state,'all')), ...
    'position_tolerance_rad',position_tol, ...
    'reserve_rad',reserve, ...
    'numeric_tolerance_rad',numeric_tol, ...
    'bisection_iterations',bound_iterations, ...
    'acceleration_tolerance_rad_s2',acceleration_tol, ...
    'minimum_reserve_rad',double(minimum_reserve), ...
    'violation_rad',double(violation), ...
    'offending_sample',double(offending_sample), ...
    'offending_joint',double(offending_joint), ...
    'offending_side',offending_side, ...
    'offending_velocity_rad_s',double(offending_velocity), ...
    'offending_stopping_distance_rad', ...
        double(offending_stopping_distance), ...
    'offending_available_margin_rad',double(offending_available_margin), ...
    'lower_violation_rad',lower_violation, ...
    'upper_violation_rad',upper_violation, ...
    'lower_raw_shortage_rad',lower_shortage_raw, ...
    'upper_raw_shortage_rad',upper_shortage_raw, ...
    'lower_reserve_rad',lower_reserve, ...
    'upper_reserve_rad',upper_reserve, ...
    'lower_stopping_distance_rad',d_lower, ...
    'upper_stopping_distance_rad',d_upper, ...
    'lower_available_margin_rad',available_lower, ...
    'upper_available_margin_rad',available_upper);
end

function d = braking_distance_local(v,alim)
nS = size(v,1);
A = repmat(alim,nS,1);
d = zeros(size(v));
finite_positive = isfinite(A) & A>0;
d(finite_positive) = v(finite_positive).^2 ./ ...
    (2*A(finite_positive));
no_brake = (A==0) & v>0;
d(no_brake) = inf;
% An infinite acceleration limit has zero braking distance under this
% declared abstraction.  NaN and negative limits were rejected earlier.
end

function v = expand_signed_limit_local(v,n,name)
if ~isnumeric(v) || ~isreal(v) || isempty(v)
    error('spinn3d_joint_limit_viability:BadLimit', ...
        '%s must be a nonempty real numeric scalar or vector.',name);
end
v = double(v(:).');
if isscalar(v), v=repmat(v,1,n); end
if numel(v)~=n || any(isnan(v))
    error('spinn3d_joint_limit_viability:BadLimit', ...
        '%s must contain exactly %d non-NaN values.',name,n);
end
end

function v = expand_nonnegative_limit_local(v,n,name)
if ~isnumeric(v) || ~isreal(v) || isempty(v)
    error('spinn3d_joint_limit_viability:BadLimit', ...
        '%s must be a nonempty real numeric scalar or vector.',name);
end
v = double(v(:).');
if isscalar(v), v=repmat(v,1,n); end
if numel(v)~=n || any(isnan(v)) || any(v<0)
    error('spinn3d_joint_limit_viability:BadLimit', ...
        '%s must contain exactly %d nonnegative, non-NaN values.',name,n);
end
end

function v = nonnegative_scalar_local(v,name)
v = double(v);
if ~isreal(v) || ~isscalar(v) || ~isfinite(v) || v<0
    error('spinn3d_joint_limit_viability:BadContractValue', ...
        '%s must be one finite nonnegative scalar.',name);
end
end

function v = positive_integer_local(v,name)
v = double(v);
if ~isreal(v) || ~isscalar(v) || ~isfinite(v) || v<1 || ...
        abs(v-round(v))>eps(max(1,abs(v)))
    error('spinn3d_joint_limit_viability:BadContractValue', ...
        '%s must be one positive integer.',name);
end
v = round(v);
end
