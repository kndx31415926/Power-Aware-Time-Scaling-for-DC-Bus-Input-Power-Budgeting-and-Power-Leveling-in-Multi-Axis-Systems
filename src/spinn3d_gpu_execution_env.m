function [useGPU, execEnv, dev] = spinn3d_gpu_execution_env(spec, verbose, tag)
%SPINN3D_GPU_EXECUTION_ENV Resolve CPU/GPU execution for custom loops.
%
%   [useGPU, execEnv, dev] = spinn3d_gpu_execution_env(spec, verbose, tag)
%
% spec can be:
%   - "auto" / "gpu" / "cpu"
%   - a struct with fields ExecutionEnvironment, executionEnvironment,
%     UseGPU, or useGPU
%   - a logical scalar, where true means "gpu" and false means "cpu"
%
% The function is intentionally conservative: "auto" falls back to CPU if a
% supported GPU is not available, while "gpu" raises an error.

    if nargin < 1 || isempty(spec), spec = 'auto'; end
    if nargin < 2 || isempty(verbose), verbose = false; end
    if nargin < 3 || isempty(tag), tag = 'gpu'; end

    execEnv = local_parse_env(spec);
    useGPU = false;
    dev = [];

    switch execEnv
        case 'cpu'
            useGPU = false;
        case 'gpu'
            [ok, dev, msg] = local_try_gpu();
            if ~ok
                error('[%s] ExecutionEnvironment="gpu" but no supported GPU is available: %s', tag, msg);
            end
            useGPU = true;
        case 'auto'
            [ok, dev, ~] = local_try_gpu();
            useGPU = ok;
            if useGPU
                execEnv = 'gpu';
            else
                execEnv = 'cpu';
            end
        otherwise
            error('spinn3d:UnknownExecutionEnvironment', ...
                'Internal execution-environment enum is invalid: ''%s''.', ...
                execEnv);
    end

    if verbose
        if useGPU
            fprintf('[%s] GPU enabled: %s, compute capability %s, %.2f GB available\n', ...
                tag, dev.Name, dev.ComputeCapability, dev.AvailableMemory/1024^3);
        else
            fprintf('[%s] GPU disabled: using CPU\n', tag);
        end
    end
end

function execEnv = local_parse_env(spec)
    if isstruct(spec)
        if isfield(spec,'ExecutionEnvironment') && ~isempty(spec.ExecutionEnvironment)
            spec = spec.ExecutionEnvironment;
        elseif isfield(spec,'executionEnvironment') && ~isempty(spec.executionEnvironment)
            spec = spec.executionEnvironment;
        elseif isfield(spec,'UseGPU') && ~isempty(spec.UseGPU)
            spec = logical(spec.UseGPU);
        elseif isfield(spec,'useGPU') && ~isempty(spec.useGPU)
            spec = logical(spec.useGPU);
        else
            error('spinn3d:UnknownExecutionEnvironment', ...
                ['Execution-environment struct must contain one of ', ...
                 'ExecutionEnvironment, executionEnvironment, UseGPU, or ', ...
                 'useGPU.']);
        end
    end

    if islogical(spec) || isnumeric(spec)

        if logical(spec)
            execEnv = 'gpu';
        else
            execEnv = 'cpu';
        end
        return;
    end

    execEnv = lower(strtrim(char(string(spec))));
    switch execEnv
        case {'gpu','cuda','parallel-gpu'}
            execEnv = 'gpu';
        case {'cpu','host'}
            execEnv = 'cpu';
        case {'auto',''}
            execEnv = 'auto';
        otherwise
            error('spinn3d:UnknownExecutionEnvironment', ...
                ['Unknown execution environment ''%s''. Use only ''auto'', ', ...
                 '''gpu'', or ''cpu'' in spinn3d_params_block.m.'],execEnv);
    end
end

function [ok, dev, msg] = local_try_gpu()
    ok = false;
    dev = [];
    msg = '';
    try
        e = exist('canUseGPU','file');
        if e == 2 || e == 5 || e == 6
            ok = logical(canUseGPU);
        else
            ok = false;
            msg = 'canUseGPU is not available';
            return;
        end
        if ok
            dev = gpuDevice(1);
        else
            msg = 'canUseGPU returned false';
        end
    catch ME
        ok = false;
        msg = ME.message;
    end
end
