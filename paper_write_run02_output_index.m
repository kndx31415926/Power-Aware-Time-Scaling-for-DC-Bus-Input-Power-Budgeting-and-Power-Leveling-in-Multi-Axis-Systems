function T = paper_write_run02_output_index(results_root)
%PAPER_WRITE_RUN02_OUTPUT_INDEX  Write a root-level index of RUN_02 outputs.
if nargin < 1 || isempty(results_root), results_root = fullfile(pwd,'paper_outputs_chunked_100'); end
defs = struct([]);
defs(1).label = 'MAIN4'; defs(1).subdir = 'main_4methods_N100_seed1';
defs(2).label = 'FAIR6'; defs(2).subdir = 'fair_6methods_N100_seed102';
rows = {};
for i=1:numel(defs)
    d = defs(i);
    stage_dir = fullfile(results_root, d.subdir);
    files = { ...
        'raw_trials','fair_baseline_trials.csv','Raw per-method per-trial metrics; contains rollout_wall_time_s for full simulated rollout wall time, not neural inference latency'; ...
        'raw_summary','fair_baseline_summary.csv','Raw method summary over completed paired trials'; ...
        'raw_paired','fair_baseline_paired_vs_reference.csv','Raw paired comparison versus NoGuard'; ...
        'canonical_trials',sprintf('RUN02_%s_trials_canonical.csv',d.label),'Canonical per-method per-trial table with clear units and manuscript-oriented names'; ...
        'canonical_summary',sprintf('RUN02_%s_summary_canonical.csv',d.label),'Canonical method summary table with metric dictionary fields'; ...
        'canonical_paired',sprintf('RUN02_%s_paired_vs_NoGuard_canonical.csv',d.label),'Canonical paired comparison table'; ...
        'variable_dictionary',sprintf('RUN02_%s_variable_dictionary.csv',d.label),'Variable dictionary and mapping from raw metric names to manuscript names'};
    for k=1:size(files,1)
        r = struct();
        r.stage = string(d.label);
        r.kind = string(files{k,1});
        r.path = string(fullfile(stage_dir, files{k,2}));
        r.exists = exist(char(r.path),'file')==2;
        r.description = string(files{k,3});
        rows{end+1,1} = r; %#ok<AGROW>
    end
end
if isempty(rows)
    T = table();
else
    T = struct2table(vertcat(rows{:}));
end
if exist(results_root,'dir')~=7, mkdir(results_root); end
writetable(T, fullfile(results_root,'RUN02_OUTPUT_INDEX.csv'));
fprintf('[RUN02-index] wrote %s\n', fullfile(results_root,'RUN02_OUTPUT_INDEX.csv'));
end
