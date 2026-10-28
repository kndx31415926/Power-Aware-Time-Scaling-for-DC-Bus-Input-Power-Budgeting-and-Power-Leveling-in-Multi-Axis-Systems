function [a_lower,a_upper,R] = ...
    spinn3d_joint_limit_viable_acceleration_bounds( ...
        q,dq,Ts,q_min,q_max,qdd_limit,a_lower,a_upper, ...
        contract,position_tolerance_rad)
%SPINN3D_JOINT_LIMIT_VIABLE_ACCELERATION_BOUNDS Exact viable qdd box.
%
% The one-step successor under constant acceleration a is
%
%   dq1 = dq + Ts*a,
%   q1  = q  + Ts*dq + 0.5*Ts^2*a.
%
% For a lower position limit, the successor braking shortage is monotone
% non-increasing in a; for an upper limit it is monotone non-decreasing.
% Consequently the complete per-joint successor-viability set is an
% interval.  This routine intersects those intervals with the caller's
% existing q/qd/qdd acceleration box.  The raw physical shortage is the
% bisection objective.  Neither position_tolerance_rad nor
% numeric_tolerance_rad is planned as successor-state headroom: the former
% belongs only to independent incoming/final state audits, while the latter
% only decides whether a saturated physical endpoint is absent by roundoff.

if nargin~=10
    error('spinn3d_joint_limit_viable_acceleration_bounds:BadInputCount', ...
        ['Expected q, dq, Ts, q_min, q_max, qdd_limit, lower/upper ', ...
         'acceleration bounds, contract and position tolerance.']);
end
q = row_finite_local(q,'q');
dq = row_finite_local(dq,'dq');
nJ = numel(q);
if numel(dq)~=nJ
    error('spinn3d_joint_limit_viable_acceleration_bounds:BadState', ...
        'q and dq must contain the same number of joints.');
end
Ts = double(Ts);
if ~isreal(Ts) || ~isscalar(Ts) || ~isfinite(Ts) || Ts<=0
    error('spinn3d_joint_limit_viable_acceleration_bounds:BadStep', ...
        'Ts must be one positive finite scalar.');
end
qlo = expand_signed_local(q_min,nJ,'q_min');
qhi = expand_signed_local(q_max,nJ,'q_max');
A = expand_nonnegative_local(qdd_limit,nJ,'qdd_limit');
a_lower = expand_signed_local(a_lower,nJ,'a_lower');
a_upper = expand_signed_local(a_upper,nJ,'a_upper');
if any(qlo>qhi) || any(a_lower>a_upper)
    error('spinn3d_joint_limit_viable_acceleration_bounds:BadBounds', ...
        'Position and incoming acceleration bounds must be ordered.');
end

required = {'schema','enable','reserve_rad','numeric_tolerance_rad', ...
    'bisection_iterations','acceleration_tolerance_rad_s2'};
if ~isstruct(contract) || ~isscalar(contract) || ...
        ~all(isfield(contract,required))
    error('spinn3d_joint_limit_viable_acceleration_bounds:MissingContract', ...
        'The complete central joint-limit-viability contract is required.');
end
schema = char(string(contract.schema));
if ~strcmp(schema,'discrete_braking_viability_v4')
    error('spinn3d_joint_limit_viable_acceleration_bounds:BadSchema', ...
        ['Unsupported joint-limit-viability schema `%s`; the viable ', ...
         'acceleration-box contract requires discrete_braking_viability_v4.'], ...
        schema);
end
if ~(islogical(contract.enable) && isscalar(contract.enable))
    error('spinn3d_joint_limit_viable_acceleration_bounds:BadEnable', ...
        'joint_limit_viability.enable must be one logical scalar.');
end
reserve = nonnegative_scalar_local(contract.reserve_rad,'reserve_rad');
numeric_tol = nonnegative_scalar_local( ...
    contract.numeric_tolerance_rad,'numeric_tolerance_rad');
position_tol = nonnegative_scalar_local( ...
    position_tolerance_rad,'position_tolerance_rad');
bisection_iterations = positive_integer_local( ...
    contract.bisection_iterations,'bisection_iterations');
acceleration_tol = nonnegative_scalar_local( ...
    contract.acceleration_tolerance_rad_s2, ...
    'acceleration_tolerance_rad_s2');

incoming_lower = a_lower;
incoming_upper = a_upper;
lower_tightened = false(1,nJ);
upper_tightened = false(1,nJ);
lower_roundoff_only = false(1,nJ);
upper_roundoff_only = false(1,nJ);
current_lower_raw_shortage = nan(1,nJ);
current_upper_raw_shortage = nan(1,nJ);
lower_full_endpoint_raw_shortage = nan(1,nJ);
upper_full_endpoint_raw_shortage = nan(1,nJ);
for j = 1:nJ
    current_lower_raw_shortage(j) = side_shortage_raw_local( ...
        'lower',0,q(j),dq(j),0,qlo(j),qhi(j),A(j), ...
        reserve,position_tol);
    current_upper_raw_shortage(j) = side_shortage_raw_local( ...
        'upper',0,q(j),dq(j),0,qlo(j),qhi(j),A(j), ...
        reserve,position_tol);
    lower_full_endpoint_raw_shortage(j) = side_shortage_raw_local( ...
        'lower',incoming_upper(j),q(j),dq(j),Ts,qlo(j),qhi(j),A(j), ...
        reserve,position_tol);
    upper_full_endpoint_raw_shortage(j) = side_shortage_raw_local( ...
        'upper',incoming_lower(j),q(j),dq(j),Ts,qlo(j),qhi(j),A(j), ...
        reserve,position_tol);
end
infeasible_joint = 0;
infeasible_side = 'none';
if logical(contract.enable)
    for j = 1:nJ
        lo = a_lower(j);
        hi = a_upper(j);

        % Lower-limit shortage decreases with acceleration.  Maintain a
        % bad lower endpoint and a good upper endpoint, then retain the
        % conservative good endpoint.
        f_lo = side_shortage_raw_local( ...
            'lower',lo,q(j),dq(j),Ts,qlo(j),qhi(j),A(j), ...
            reserve,position_tol);
        f_hi = side_shortage_raw_local( ...
            'lower',hi,q(j),dq(j),Ts,qlo(j),qhi(j),A(j), ...
            reserve,position_tol);
        lower_full_endpoint_raw_shortage(j) = f_hi;
        if f_hi>numeric_tol
            infeasible_joint = j;
            infeasible_side = 'lower';
            a_lower(j) = hi;
            a_upper(j) = lo;
            break;
        elseif f_lo>0
            if f_hi>0
                % The best inward endpoint misses the raw zero boundary only

                % saturated singleton instead of inverting the interval.
                lo = hi;
                lower_roundoff_only(j) = true;
            else
                bad = lo;
                good = hi;
                for it = 1:bisection_iterations
                    if good-bad<=acceleration_tol, break; end
                    mid = finite_midpoint_local(bad,good);
                    f_mid = side_shortage_raw_local( ...
                        'lower',mid,q(j),dq(j),Ts,qlo(j),qhi(j),A(j), ...
                        reserve,position_tol);
                    if f_mid<=0
                        good = mid;
                    else
                        bad = mid;
                    end
                end
                lo = good;
            end
            a_lower(j) = lo;
            lower_tightened(j) = true;
        end

        % Upper-limit shortage increases with acceleration.  Maintain a
        % good lower endpoint and a bad upper endpoint.
        f_lo = side_shortage_raw_local( ...
            'upper',lo,q(j),dq(j),Ts,qlo(j),qhi(j),A(j), ...
            reserve,position_tol);
        f_hi = side_shortage_raw_local( ...
            'upper',hi,q(j),dq(j),Ts,qlo(j),qhi(j),A(j), ...
            reserve,position_tol);
        upper_full_endpoint_raw_shortage(j) = f_lo;
        if f_lo>numeric_tol
            infeasible_joint = j;
            infeasible_side = 'upper';
            a_lower(j) = hi;
            a_upper(j) = lo;
            break;
        elseif f_hi>0
            if f_lo>0
                % Symmetric upper-limit roundoff case: retain the most
                % negative saturated endpoint as a singleton.
                hi = lo;
                upper_roundoff_only(j) = true;
            else
                good = lo;
                bad = hi;
                for it = 1:bisection_iterations
                    if bad-good<=acceleration_tol, break; end
                    mid = finite_midpoint_local(good,bad);
                    f_mid = side_shortage_raw_local( ...
                        'upper',mid,q(j),dq(j),Ts,qlo(j),qhi(j),A(j), ...
                        reserve,position_tol);
                    if f_mid<=0
                        good = mid;
                    else
                        bad = mid;
                    end
                end
                hi = good;
            end
            a_upper(j) = hi;
            upper_tightened(j) = true;
        end
    end
end

pass = infeasible_joint==0 && all(a_lower<=a_upper);
lower_shortage_at_bound = nan(1,nJ);
upper_shortage_at_bound = nan(1,nJ);
if pass
    for j = 1:nJ
        lower_shortage_at_bound(j) = side_shortage_raw_local( ...
            'lower',a_lower(j),q(j),dq(j),Ts,qlo(j),qhi(j),A(j), ...
            reserve,position_tol);
        upper_shortage_at_bound(j) = side_shortage_raw_local( ...
            'upper',a_upper(j),q(j),dq(j),Ts,qlo(j),qhi(j),A(j), ...
            reserve,position_tol);
    end
end

R = struct( ...
    'schema','successor_viable_acceleration_box_v3', ...
    'enabled',logical(contract.enable), ...
    'pass',logical(pass), ...
    'incoming_lower_bound',incoming_lower, ...
    'incoming_upper_bound',incoming_upper, ...
    'lower_bound',a_lower, ...
    'upper_bound',a_upper, ...
    'lower_tightened',lower_tightened, ...
    'upper_tightened',upper_tightened, ...
    'current_lower_raw_shortage_rad',current_lower_raw_shortage, ...
    'current_upper_raw_shortage_rad',current_upper_raw_shortage, ...
    'lower_full_endpoint_raw_shortage_rad', ...
        lower_full_endpoint_raw_shortage, ...
    'upper_full_endpoint_raw_shortage_rad', ...
        upper_full_endpoint_raw_shortage, ...
    'lower_roundoff_only_saturated_endpoint',lower_roundoff_only, ...
    'upper_roundoff_only_saturated_endpoint',upper_roundoff_only, ...
    'roundoff_only_saturated_endpoint', ...
        lower_roundoff_only | upper_roundoff_only, ...
    'lower_shortage_at_bound_rad',lower_shortage_at_bound, ...
    'upper_shortage_at_bound_rad',upper_shortage_at_bound, ...
    'infeasible_joint',double(infeasible_joint), ...
    'infeasible_side',infeasible_side, ...
    'position_planning_tolerance_rad',0, ...
    'position_audit_tolerance_rad',position_tol, ...
    'numeric_tolerance_rad',numeric_tol, ...
    'bisection_iterations',bisection_iterations, ...
    'acceleration_tolerance_rad_s2',acceleration_tol);
end

function shortage = side_shortage_raw_local( ...
        side,a,q,dq,Ts,qlo,qhi,A,reserve,position_tol)
q1 = q+Ts*dq+0.5*Ts*Ts*a;
dq1 = dq+Ts*a;
switch side
    case 'lower'
        if ~isfinite(qlo)
            shortage = -inf;
            return;
        end
        v_out = max(-dq1,0);
        available = q1-qlo;
    case 'upper'
        if ~isfinite(qhi)
            shortage = -inf;
            return;
        end
        v_out = max(dq1,0);
        available = qhi-q1;
    otherwise
        error('spinn3d_joint_limit_viable_acceleration_bounds:BadSide', ...
            'Unknown joint-limit side `%s`.',side);
end
if isfinite(A) && A>0
    d_stop = v_out*v_out/(2*A);
elseif A==0 && v_out>0
    d_stop = inf;
else
    % A=Inf declares zero stopping distance; A=0 with no outward velocity
    % also requires no stopping distance.
    d_stop = 0;
end
% The physical successor set ends at qlo/qhi.  position_tol is deliberately
% not added to available distance: doing so lets the command constructor
% target qlo-position_tol or qhi+position_tol and consumes the complete

shortage = d_stop+reserve-available;
end

function mid = finite_midpoint_local(lo,hi)
if ~isfinite(lo) || ~isfinite(hi)
    error('spinn3d_joint_limit_viable_acceleration_bounds:UnboundedRoot', ...
        ['An active braking-viability boundary requires finite incoming ', ...
         'acceleration bounds.']);
end
mid = lo+0.5*(hi-lo);
end

function v = row_finite_local(v,name)
if ~isnumeric(v) || ~isreal(v) || isempty(v)
    error('spinn3d_joint_limit_viable_acceleration_bounds:BadState', ...
        '%s must be one nonempty real numeric vector.',name);
end
v = double(v(:).');
if any(~isfinite(v))
    error('spinn3d_joint_limit_viable_acceleration_bounds:BadState', ...
        '%s must contain only finite values.',name);
end
end

function v = expand_signed_local(v,n,name)
if ~isnumeric(v) || ~isreal(v) || isempty(v)
    error('spinn3d_joint_limit_viable_acceleration_bounds:BadBound', ...
        '%s must be a nonempty real numeric scalar or vector.',name);
end
v = double(v(:).');
if isscalar(v), v=repmat(v,1,n); end
if numel(v)~=n || any(isnan(v))
    error('spinn3d_joint_limit_viable_acceleration_bounds:BadBound', ...
        '%s must contain exactly %d non-NaN values.',name,n);
end
end

function v = expand_nonnegative_local(v,n,name)
v = expand_signed_local(v,n,name);
if any(v<0)
    error('spinn3d_joint_limit_viable_acceleration_bounds:BadBound', ...
        '%s must contain nonnegative values.',name);
end
end

function v = nonnegative_scalar_local(v,name)
v = double(v);
if ~isreal(v) || ~isscalar(v) || ~isfinite(v) || v<0
    error('spinn3d_joint_limit_viable_acceleration_bounds:BadContractValue', ...
        '%s must be one finite nonnegative scalar.',name);
end
end

function v = positive_integer_local(v,name)
v = double(v);
if ~isreal(v) || ~isscalar(v) || ~isfinite(v) || v<1 || ...
        abs(v-round(v))>eps(max(1,abs(v)))
    error('spinn3d_joint_limit_viable_acceleration_bounds:BadContractValue', ...
        '%s must be one positive integer.',name);
end
v = round(v);
end
