%% RUN_02_CHECK_PROGRESS.m
% Check progress for the clean resumable RUN_02 workflow.

clear; clc;
project_dir = fileparts(mfilename('fullpath'));
if isempty(project_dir), project_dir = pwd; end
results_root = fullfile(project_dir,'paper_outputs_chunked_100');

defs = struct([]);
defs(1).label = 'MAIN4';
defs(1).subdir = 'main_4methods_N100_seed1';
defs(1).methods = {'NoGuard','Feasible-RG','MLP-PRG','TRF-PRG'};
defs(1).total_N = 100;
defs(2).label = 'FAIR6';
defs(2).subdir = 'fair_6methods_N100_seed102';
defs(2).methods = {'NoGuard','Feasible-RG','TRF-PRG','MLP-PRG','Bus-PTP-DP','MPC-RG-grid'};
defs(2).total_N = 100;

fprintf('\nRUN_02 progress check at %s\n', datestr(now));
fprintf('Results root: %s\n', results_root);
idx_csv = fullfile(results_root,'RUN02_OUTPUT_INDEX.csv');
if exist(idx_csv,'file')==2, fprintf('Output index: %s\n', idx_csv); end

for d = 1:numel(defs)
    export_dir = fullfile(results_root, defs(d).subdir);
    csv_trials = fullfile(export_dir,'fair_baseline_trials.csv');
    fprintf('\n[%s] %s\n', defs(d).label, export_dir);
    if exist(csv_trials,'file') ~= 2
        fprintf('  No trial CSV found yet.\n');
        continue;
    end
    T = readtable(csv_trials);
    if ismember('runtime_s',T.Properties.VariableNames)
        fprintf('  NOTE: legacy runtime_s column detected; current code rewrites it as rollout_wall_time_s.\n');
    end
    if ismember('method',T.Properties.VariableNames), T.method = string(T.method); end
    if ~ismember('success',T.Properties.VariableNames), T.success = false(height(T),1); end
    done_ids = completed_ids_local(T, defs(d).methods);
    missing = setdiff(1:defs(d).total_N, done_ids, 'stable');
    fprintf('  Rows in CSV: %d\n', height(T));
    fprintf('  Complete paired trials: %d/%d\n', numel(done_ids), defs(d).total_N);
    fprintf('  Completed IDs: %s\n', compact_id_list_local(done_ids));
    if isempty(missing)
        fprintf('  Status: COMPLETE.\n');
    else
        fprintf('  Next missing starts at: %d\n', missing(1));
        fprintf('  Next 10 would be: %s\n', compact_id_list_local(missing(1:min(10,numel(missing)))));
        fprintf('  Next 20 would be: %s\n', compact_id_list_local(missing(1:min(20,numel(missing)))));
    end
    canon_csv = fullfile(export_dir, sprintf('RUN02_%s_trials_canonical.csv', defs(d).label));
    if exist(canon_csv,'file')==2
        fprintf('  Canonical trials CSV: %s\n', canon_csv);
    else
        fprintf('  Canonical trials CSV not found yet; it will be written by the next RUN_02 chunk or RUN_03.\n');
    end
    dict_csv = fullfile(export_dir, sprintf('RUN02_%s_variable_dictionary.csv', defs(d).label));
    if exist(dict_csv,'file')==2
        fprintf('  Variable dictionary: %s\n', dict_csv);
    end
    if any(~logical(T.success))
        Tf = T(~logical(T.success),:);
        fprintf('  Failed method-runs: %d\n', height(Tf));
        disp(Tf(:, intersect({'trial_id','method','error'}, Tf.Properties.VariableNames, 'stable')));
    end
    try
        key = string(T.trial_id) + string('|') + string(T.method);
        [u,~,ic] = unique(key);
        counts = accumarray(ic,1);
        ndups = sum(counts > 1);
        if ndups > 0
            fprintf('  WARNING: duplicate (trial_id,method) keys found: %d\n', ndups);
            disp(u(counts>1));
        end
    catch
    end
end

run03_status = fullfile(results_root,'RUN03_revision_exports','RUN03_STATUS.mat');
if exist(run03_status,'file')~=2
    run03_status = fullfile(results_root,'RUN03_revision_exports','AUX_STATUS.mat');
end
fprintf('\n[RUN03] %s\n', fullfile(results_root,'RUN03_revision_exports'));
if exist(run03_status,'file')==2
    try
        S = load(run03_status,'status');
        if isfield(S.status,'done') && S.status.done
            fprintf('  Status: COMPLETE.\n');
        else
            fprintf('  Status: exists but not fully complete. Inspect RUN03_STATUS.mat.\n');
        end
        if isfield(S.status,'steps')
            disp(S.status.steps);
        end
    catch ME
        fprintf('  Could not read RUN03_STATUS.mat: %s\n', ME.message);
    end
else
    fprintf('  Not run yet. Run RUN_03_EXPORT_REVISION_RESULTS after MAIN4 and FAIR6 are complete.\n');
end

function ids = completed_ids_local(T,methods)
ids = [];
if isempty(T) || ~istable(T) || ~ismember('trial_id',T.Properties.VariableNames) || ~ismember('method',T.Properties.VariableNames)
    return;
end
mcol = string(T.method);
all_ids = unique(T.trial_id(:).');
for id = all_ids
    ok = true;
    for k = 1:numel(methods)
        idx = T.trial_id==id & mcol==string(methods{k}) & logical(T.success);
        if ~any(idx), ok=false; break; end
    end
    if ok, ids(end+1)=id; end %#ok<AGROW>
end
ids = sort(unique(ids));
end

function s = compact_id_list_local(ids)
ids = ids(:).';
if isempty(ids), s='(none)'; return; end
ids = sort(unique(ids));
out = {}; a = ids(1); b = ids(1);
for k=2:numel(ids)
    if ids(k)==b+1
        b = ids(k);
    else
        out{end+1}=range_str_local(a,b); %#ok<AGROW>
        a = ids(k); b = ids(k);
    end
end
out{end+1}=range_str_local(a,b);
s = strjoin(out, ',');
end
function s = range_str_local(a,b)
if a==b, s=sprintf('%d',a); else, s=sprintf('%d:%d',a,b); end
end
