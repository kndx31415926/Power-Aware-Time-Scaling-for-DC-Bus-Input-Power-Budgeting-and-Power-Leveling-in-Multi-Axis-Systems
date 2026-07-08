function pool = paper_open_parallel_pool(opts)
%PAPER_OPEN_PARALLEL_POOL Open/reuse a MATLAB parallel pool for paper runs.
%
% Important options:
%   opts.use_parallel      true/false
%   opts.num_workers       [] or positive integer
%   opts.pool_type         'local' (process workers, high CPU occupancy) or 'threads'
%   opts.force_recreate    recreate the pool if worker count/type differs
%   opts.idle_timeout_min  pool idle timeout in minutes
%
% For the long paper-generation runs, 'local' is the default because it is
% closer to the high-CPU behavior of the upgraded/checkpointed version and is
% usually more robust for Robotics System Toolbox and heavy rollout code.

if nargin < 1 || isempty(opts), opts = struct(); end
use_parallel = logical(getfield_def(opts,'use_parallel',true));
if ~use_parallel
    pool = [];
    return;
end
if exist('gcp','file') ~= 2 || exist('parpool','file') ~= 2
    warning('Parallel Computing Toolbox not available; running serially.');
    pool = [];
    return;
end

pool_type = lower(string(getfield_def(opts,'pool_type','local')));
if pool_type == "process", pool_type = "local"; end
if pool_type ~= "local" && pool_type ~= "threads"
    warning('Unknown pool_type=%s; using local.', pool_type);
    pool_type = "local";
end
nw = getfield_def(opts,'num_workers',[]);
if ~isempty(nw)
    nw = max(1, round(double(nw)));
end
force_recreate = logical(getfield_def(opts,'force_recreate',false));
idle_timeout_min = double(getfield_def(opts,'idle_timeout_min',1000));

pool = gcp('nocreate');
if ~isempty(pool)
    existing_type = pool_type_of(pool);
    mismatch_type = existing_type ~= pool_type;
    mismatch_nw = ~isempty(nw) && isprop(pool,'NumWorkers') && pool.NumWorkers ~= nw;
    if force_recreate && (mismatch_type || mismatch_nw)
        fprintf('[parallel] Closing existing %s pool with %d workers.\n', existing_type, pool.NumWorkers);
        delete(pool);
        pool = [];
    else
        fprintf('[parallel] Reusing existing %s pool with %d workers.\n', existing_type, pool.NumWorkers);
    end
end

if isempty(pool)
    try
        if pool_type == "threads"
            if isempty(nw)
                pool = parpool('threads');
            else
                pool = parpool('threads', nw);
            end
        else
            if isempty(nw)
                pool = parpool('local');
            else
                pool = parpool('local', nw);
            end
        end
    catch ME
        if pool_type == "threads"
            warning('Could not start threads pool (%s). Falling back to local/process pool.', ME.message);
            if isempty(nw), pool = parpool('local'); else, pool = parpool('local', nw); end
        else
            rethrow(ME);
        end
    end
end

try
    pool.IdleTimeout = idle_timeout_min;
catch
end
fprintf('[parallel] Active pool: %s, workers=%d\n', pool_type_of(pool), pool.NumWorkers);
end

function t = pool_type_of(pool)
cls = string(class(pool));
if contains(lower(cls),'thread')
    t = "threads";
else
    t = "local";
end
end

function x = getfield_def(S,fn,def)
if isstruct(S) && isfield(S,fn) && ~isempty(S.(fn)), x=S.(fn); else, x=def; end
end
