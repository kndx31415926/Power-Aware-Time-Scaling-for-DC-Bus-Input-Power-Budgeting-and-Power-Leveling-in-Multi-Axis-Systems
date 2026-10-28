function [a_best, report] = spinn3d_box_torque_min_anchor( ...
        M,h,a_lo,a_hi,seeds,tau_scale,n_sweeps)
%SPINN3D_BOX_TORQUE_MIN_ANCHOR Box-constrained n-D unloading anchor.
% This is a small deterministic coordinate-descent solver for
%
%   min_a sum_i ((h_i + M(i,:)*a)/tau_scale_i)^2
%   subject to a_lo <= a <= a_hi.
%
% It has no Optimization Toolbox dependency.  The result is only a search
% anchor: the caller must still perform the complete actuator, motion and

M = double(M);
h = double(h(:));
a_lo = double(a_lo(:));
a_hi = double(a_hi(:));
tau_scale = double(tau_scale(:));
nJ = numel(h);

n_sweeps = max(1,round(double(n_sweeps)));

if isempty(seeds)
    seeds = zeros(1,nJ);
else
    seeds = double(seeds);
    if isvector(seeds), seeds = reshape(seeds,1,[]); end
end

fallback_scale = max([abs(h),ones(nJ,1)],[],2);
bad_scale = ~isfinite(tau_scale) | tau_scale<=0;
tau_scale(bad_scale) = fallback_scale(bad_scale);
w = 1./(tau_scale.^2);

best_phi = inf;
a_best = min(a_hi,max(a_lo,zeros(nJ,1)));
coordinate_updates = 0;
for iseed = 1:size(seeds,1)
    a = min(a_hi,max(a_lo,seeds(iseed,:).'));
    tau = h + M*a;
    for isweep = 1:n_sweeps
        changed = false;
        for j = 1:nJ
            col = M(:,j);
            denom = sum(w.*(col.^2));
            if ~(isfinite(denom) && denom>eps)
                continue;
            end
            delta = -sum(w.*col.*tau)/denom;
            a_new_j = min(a_hi(j),max(a_lo(j),a(j)+delta));
            if abs(a_new_j-a(j)) > 1e-14*max(1,abs(a(j)))
                tau = tau + col*(a_new_j-a(j));
                a(j) = a_new_j;
                coordinate_updates = coordinate_updates + 1;
                changed = true;
            end
        end
        if ~changed, break; end
    end
    phi = sum((tau./tau_scale).^2);
    if phi < best_phi
        best_phi = phi;
        a_best = a;
    end
end

a_best = a_best.';
report = struct('schema','box_weighted_torque_coordinate_descent_v1', ...
    'objective',best_phi,'seed_count',size(seeds,1), ...
    'sweeps_requested',n_sweeps, ...
    'coordinate_updates',coordinate_updates);
end
