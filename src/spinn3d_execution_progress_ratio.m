function [ratio,modified,basis] = spinn3d_execution_progress_ratio( ...
        a_req,a_exec,a_lo,a_hi,qdd_lim,ratio_tol)
%SPINN3D_EXECUTION_PROGRESS_RATIO Conservative, well-conditioned rate proxy.
% Compare requested and finally executed joint acceleration in central-qdd
% units.  When the requested acceleration is essentially zero, acceleration
% alone cannot identify a directional retention ratio.  Use absolute
% normalized command change in that branch: tiny corrections retain nearly
% full progress without turning roundoff into a zero-alpha deadlock, while a
% completely rewritten command can never be reported as full progress.

a_req = double(a_req(:).');
a_exec = double(a_exec(:).');
a_lo = double(a_lo(:).');
a_hi = double(a_hi(:).');
qdd_lim = double(qdd_lim(:).');
ratio_tol = double(ratio_tol);
nJ = numel(a_req);

a_freeze = min(a_hi,max(a_lo,zeros(size(a_req))));
bad_scale = ~isfinite(qdd_lim) | qdd_lim<=1e-9;
fallback = max([abs(a_req(:)),abs(a_exec(:)),ones(nJ,1)],[],2).';
qdd_lim(bad_scale) = fallback(bad_scale);
delta = (a_exec-a_req)./qdd_lim;
modified = norm(delta,inf)>ratio_tol;
if ~modified
    ratio = 1.0;
    basis = 'unmodified_within_central_tolerance';
    return;
end

v = (a_req-a_freeze)./qdd_lim;
u = (a_exec-a_freeze)./qdd_lim;
vv = dot(v,v);
if sqrt(vv)<=ratio_tol
    ratio = max(0.0,min(1.0,1.0-norm(delta,inf)));
    basis = 'near_zero_request_absolute_change';
else
    r_parallel = dot(u,v)/vv;
    r_distance = 1.0-norm(u-v,2)/sqrt(vv);
    ratio = max(0.0,min(1.0,min(r_parallel,r_distance)));
    basis = 'normalized_acceleration_retention';
end

end
