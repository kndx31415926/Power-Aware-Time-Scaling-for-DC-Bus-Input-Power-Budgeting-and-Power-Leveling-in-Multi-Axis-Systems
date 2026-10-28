function [a_best, report] = spinn3d_box_torque_feasible_anchor( ...
        M,h,a_lo,a_hi,tau_lim,a_preferred,a_excluded)
%SPINN3D_BOX_TORQUE_FEASIBLE_ANCHOR Exact small-arm torque-box anchor.
% Find a point in the convex polytope
%
%   a_lo <= a <= a_hi,  -tau_lim <= h + M*a <= tau_lim.
%
% No Optimization Toolbox is used.  The acceleration box makes the
% polytope bounded, so every nonempty feasible set has a vertex.  For the
% four-axis RoArm contract, enumerating all full-rank active sets means at
% most nchoosek(16,4)=1820 tiny linear solves.  This helper only supplies a
% deterministic recovery anchor; the caller must still run the complete

M = double(M);
h = double(h(:));
a_lo = double(a_lo(:));
a_hi = double(a_hi(:));
tau_lim = double(tau_lim(:));
a_preferred = double(a_preferred(:));
nJ = numel(h);
if nargin<7 || isempty(a_excluded)
    a_excluded = zeros(0,nJ);
else
    a_excluded = double(a_excluded);
end

% A*a <= b: acceleration box followed by every finite torque slab.
A = [eye(nJ);-eye(nJ)];
b = [a_hi;-a_lo];
finite_tau = find(isfinite(tau_lim));
for k = 1:numel(finite_tau)
    i = finite_tau(k);
    A = [A;M(i,:);-M(i,:)]; %#ok<AGROW>
    b = [b;tau_lim(i)-h(i);tau_lim(i)+h(i)]; %#ok<AGROW>
end

row_scale = sqrt(sum(A.^2,2));

As = A./row_scale;
bs = b./row_scale;
check_tol = 256*eps(max(1,abs(bs)));

a_seed = min(a_hi,max(a_lo,a_preferred));
[seed_ok,seed_violation] = feasible_local(As,bs,a_seed,check_tol);
a_best = a_seed;
best_distance = inf;
best_violation = seed_violation;
vertices_tested = 0;
feasible_vertices = 0;
excluded_feasible_points = 0;
seed_excluded = excluded_local(a_seed,a_excluded);
if seed_ok && ~seed_excluded
    report = report_local(true,true,0,0,0,seed_violation, ...
        'preferred_point',0);
    a_best = a_best.';
    return;
end
if seed_ok && seed_excluded
    excluded_feasible_points = excluded_feasible_points+1;
end

% An intersection of a nonempty bounded polyhedron has a vertex.  Enumerate
% all nJ-constraint bases, reject rank-deficient bases, and retain the
% feasible vertex nearest the preferred acceleration in box-normalized
% Euclidean distance.  Stable lexicographic enumeration makes ties exactly
% reproducible.
active_sets = nchoosek(1:size(As,1),nJ);
box_width = max(a_hi-a_lo,1e-12);
found = false;
polytope_feasible = seed_ok;
for k = 1:size(active_sets,1)
    idx = active_sets(k,:);
    Aeq = As(idx,:);
    if rcond(Aeq) <= 1e-12
        continue;
    end
    vertices_tested = vertices_tested + 1;
    a = Aeq\bs(idx);
    if any(~isfinite(a))
        continue;
    end
    [ok,max_violation] = feasible_local(As,bs,a,check_tol);
    best_violation = min(best_violation,max_violation);
    if ~ok
        continue;
    end
    polytope_feasible = true;
    feasible_vertices = feasible_vertices + 1;
    if excluded_local(a,a_excluded)
        excluded_feasible_points = excluded_feasible_points+1;
        continue;
    end
    distance = norm((a-a_preferred)./box_width,2);
    if ~found || distance < best_distance-64*eps(max(1,best_distance))
        found = true;
        best_distance = distance;
        a_best = a;
        best_violation = max_violation;
    end
end

source = 'none';
if found, source = 'active_set_vertex'; end
if polytope_feasible && ~found, source = 'excluded_only'; end
report = report_local(polytope_feasible,found,vertices_tested, ...
    feasible_vertices,size(active_sets,1),best_violation,source, ...
    excluded_feasible_points);
a_best = a_best.';
end

function tf = excluded_local(a,excluded)
if isempty(excluded)
    tf = false;
    return;
end
scale = max(1,max(abs([a(:);excluded(:)])));
tf = any(all(abs(excluded-a(:).')<=1e-12*scale,2));
end

function [ok,max_violation] = feasible_local(A,b,a,tol)
residual = A*a-b-tol;
max_violation = max([0;residual]);
ok = logical(all(residual<=0));
end

function R = report_local(polytope_feasible,novel_candidate,tested, ...
        nfeasible,total,max_v,source,nexcluded)
R = struct('schema','box_torque_active_set_v1', ...
    'feasible',logical(polytope_feasible), ...
    'novel_candidate',logical(novel_candidate), ...
    'vertices_tested',double(tested), ...
    'feasible_vertices',double(nfeasible), ...
    'excluded_feasible_points',double(nexcluded), ...
    'active_sets_total',double(total), ...
    'max_normalized_constraint_violation',double(max_v), ...
    'source',char(source));
end
