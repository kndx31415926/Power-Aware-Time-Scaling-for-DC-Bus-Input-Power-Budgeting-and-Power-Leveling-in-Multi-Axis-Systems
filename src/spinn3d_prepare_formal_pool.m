function p = spinn3d_prepare_formal_pool(profile,n_workers,idle_timeout_min,varargin)
% Use the current parallel pool or create one for the planner.
p=gcp('nocreate');
if isempty(p)
    if isempty(n_workers)
        p=parpool(profile);
    else
        p=parpool(profile,n_workers);
    end
end
p.IdleTimeout=double(idle_timeout_min);
end
