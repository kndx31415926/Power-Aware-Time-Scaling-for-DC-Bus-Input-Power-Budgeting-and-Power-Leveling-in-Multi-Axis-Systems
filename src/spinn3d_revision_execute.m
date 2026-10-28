function [out,record]=spinn3d_revision_execute(cfg,method_name,seed,callback,opts)
% Execute one seeded method on the common physical task.
% The study entry point saves trajectories and resumes completed methods.
if nargin<5,opts=struct();end
previous_rng=rng;restore_rng=onCleanup(@()rng(previous_rng)); %#ok<NASGU>
close_figs=isfield(opts,'close_figs')&&logical(opts.close_figs);
before=[];if close_figs,before=findall(0,'Type','figure');end
restore_figs=onCleanup(@()close_new_figures(close_figs,before)); %#ok<NASGU>
out=[];returned=false;callback_error=[];
rng(double(seed),'twister');timer=tic;
console_log=evalc('try; out=callback(cfg); returned=true; catch callback_error; end');
record=struct('method_label',char(method_name),'seed',double(seed), ...
    'callback_returned',returned,'callback_wall_s',toc(timer), ...
    'out',out,'console_log',console_log,'cache_reused',false,'archive_file','', ...
    'error',struct('identifier','','message',''));
if ~returned
    record.error=struct('identifier',callback_error.identifier,'message',callback_error.message);
    if isempty(record.error.identifier),record.error.identifier='simulation:MethodError';end
end
end

function close_new_figures(enabled,before)
if ~enabled,return;end
after=findall(0,'Type','figure');fresh=after(~ismember(after,before));
if ~isempty(fresh),close(fresh);end
end
