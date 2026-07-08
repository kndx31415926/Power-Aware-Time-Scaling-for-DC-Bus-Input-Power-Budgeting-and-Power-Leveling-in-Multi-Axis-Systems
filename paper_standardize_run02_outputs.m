function R = paper_standardize_run02_outputs(stage_dir, stage_label, methods)
%PAPER_STANDARDIZE_RUN02_OUTPUTS  Write human-readable RUN_02 canonical CSVs.
%
% This helper keeps the original raw RUN_02 files for backward compatibility:
%   fair_baseline_trials.csv
%   fair_baseline_summary.csv
%   fair_baseline_paired_vs_reference.csv
%
% It additionally writes canonical manuscript-facing files with unambiguous
% names, units, and a variable dictionary:
%   RUN02_<STAGE>_trials_canonical.csv
%   RUN02_<STAGE>_summary_canonical.csv
%   RUN02_<STAGE>_paired_vs_NoGuard_canonical.csv
%   RUN02_<STAGE>_variable_dictionary.csv
%
% Key convention:
%   rollout_wall_time_s is the MATLAB wall-clock time for the full simulated
%   closed-loop rollout of one method. It is NOT neural inference latency and
%   should not be used in Table 6. RUN03 latency files provide per-policy-update
%   feature/neural/governor timing.

if nargin < 1 || isempty(stage_dir), stage_dir = pwd; end
if nargin < 2 || isempty(stage_label)
    [~,nm] = fileparts(stage_dir);
    if contains(lower(nm),'main'), stage_label = 'MAIN4'; elseif contains(lower(nm),'fair'), stage_label = 'FAIR6'; else, stage_label = upper(nm); end
end
if nargin < 3 || isempty(methods)
    if strcmpi(stage_label,'MAIN4')
        methods = {'NoGuard','Feasible-RG','MLP-PRG','TRF-PRG'};
    else
        methods = {'NoGuard','Feasible-RG','TRF-PRG','MLP-PRG','Bus-PTP-DP','MPC-RG-grid'};
    end
end
stage_label = char(stage_label);

raw_trials = fullfile(stage_dir,'fair_baseline_trials.csv');
raw_summary = fullfile(stage_dir,'fair_baseline_summary.csv');
raw_paired = fullfile(stage_dir,'fair_baseline_paired_vs_reference.csv');

assert(exist(raw_trials,'file')==2, 'RUN02 raw trials CSV not found: %s', raw_trials);

Traw = normalize_trials_table_local(readtable(raw_trials));
Tc = make_trials_canonical_local(Traw, stage_label);

trials_out = fullfile(stage_dir, sprintf('RUN02_%s_trials_canonical.csv', stage_label));
writetable(Tc, trials_out);

Sraw = table(); Sc = table(); summary_out = '';
if exist(raw_summary,'file')==2
    Sraw = readtable(raw_summary);
    Sc = make_summary_canonical_local(Sraw, stage_label);
    summary_out = fullfile(stage_dir, sprintf('RUN02_%s_summary_canonical.csv', stage_label));
    writetable(Sc, summary_out);
end

Praw = table(); Pc = table(); paired_out = '';
if exist(raw_paired,'file')==2
    Praw = readtable(raw_paired);
    Pc = make_paired_canonical_local(Praw, stage_label);
    paired_out = fullfile(stage_dir, sprintf('RUN02_%s_paired_vs_NoGuard_canonical.csv', stage_label));
    writetable(Pc, paired_out);
end

D = variable_dictionary_local();
dict_out = fullfile(stage_dir, sprintf('RUN02_%s_variable_dictionary.csv', stage_label));
writetable(D, dict_out);

% Also provide concise aliases with stable names inside every stage folder.
try, copyfile(trials_out, fullfile(stage_dir,'RUN02_trials_canonical.csv')); catch, end
if ~isempty(summary_out), try, copyfile(summary_out, fullfile(stage_dir,'RUN02_summary_canonical.csv')); catch, end, end
if ~isempty(paired_out), try, copyfile(paired_out, fullfile(stage_dir,'RUN02_paired_vs_NoGuard_canonical.csv')); catch, end, end
try, copyfile(dict_out, fullfile(stage_dir,'RUN02_variable_dictionary.csv')); catch, end

R = struct();
R.stage_label = string(stage_label);
R.stage_dir = string(stage_dir);
R.raw_trials = string(raw_trials);
R.trials_canonical = string(trials_out);
R.summary_canonical = string(summary_out);
R.paired_canonical = string(paired_out);
R.variable_dictionary = string(dict_out);
R.n_rows_raw = height(Traw);
R.n_rows_canonical = height(Tc);
R.methods = string(methods(:));

fprintf('[RUN02-standardize] %s canonical trials: %s (%d rows)\n', stage_label, trials_out, height(Tc));
if ~isempty(summary_out), fprintf('[RUN02-standardize] %s canonical summary: %s\n', stage_label, summary_out); end
if ~isempty(paired_out), fprintf('[RUN02-standardize] %s canonical paired: %s\n', stage_label, paired_out); end
fprintf('[RUN02-standardize] %s variable dictionary: %s\n', stage_label, dict_out);
end

function T = normalize_trials_table_local(T)
if isempty(T) || ~istable(T), return; end
if ismember('method',T.Properties.VariableNames), T.method = string(T.method); end
if ismember('error',T.Properties.VariableNames), T.error = string(T.error); end
if ismember('success',T.Properties.VariableNames)
    try, T.success = logical(T.success); catch, end
end
% Migrate legacy ambiguous runtime_s to the explicit rollout_wall_time_s name.
if ismember('runtime_s',T.Properties.VariableNames)
    if ~ismember('rollout_wall_time_s',T.Properties.VariableNames)
        T.rollout_wall_time_s = T.runtime_s;
    else
        idx = ~isfinite(T.rollout_wall_time_s) & isfinite(T.runtime_s);
        try, T.rollout_wall_time_s(idx) = T.runtime_s(idx); catch, end
    end
    T.runtime_s = [];
end
if ismember('offline_solve_time_s',T.Properties.VariableNames)
    if ~ismember('offline_profile_solve_wall_time_s',T.Properties.VariableNames)
        T.offline_profile_solve_wall_time_s = T.offline_solve_time_s;
    else
        idx = ~isfinite(T.offline_profile_solve_wall_time_s) & isfinite(T.offline_solve_time_s);
        try, T.offline_profile_solve_wall_time_s(idx) = T.offline_solve_time_s(idx); catch, end
    end
    T.offline_solve_time_s = [];
end
end

function Tc = make_trials_canonical_local(T, stage_label)
n = height(T);
Tc = table();
Tc.stage = repmat(string(stage_label),n,1);
Tc.trial_id = col_local(T,'trial_id',NaN(n,1));
Tc.method = string_col_local(T,'method',strings(n,1));
Tc.method_role = method_role_local(Tc.method);
Tc.success = logical_col_local(T,'success',false(n,1));
Tc.error_message = string_col_local(T,'error',strings(n,1));
Tc.payload_mass_kg = numeric_col_local(T,'payload_mass_kg',NaN(n,1));
Tc.P_grid_budget_W = numeric_col_local(T,'P_grid_budget_W',NaN(n,1));
Tc.rollout_wall_time_s = numeric_col_local(T,'rollout_wall_time_s',NaN(n,1));
Tc.offline_profile_solve_wall_time_s = numeric_col_local(T,'offline_profile_solve_wall_time_s',NaN(n,1));
Tc.timing_note = repmat("rollout_wall_time_s is full MATLAB closed-loop simulation wall time; use RUN03 latency_policy_steps.csv for neural inference latency",n,1);
for j=1:4
    Tc.(sprintf('q0_deg_j%d',j)) = numeric_col_local(T,sprintf('q0_deg_j%d',j),NaN(n,1));
    Tc.(sprintf('qf_deg_j%d',j)) = numeric_col_local(T,sprintf('qf_deg_j%d',j),NaN(n,1));
end
% Canonical paper metrics.
Tc.t_reach_s = numeric_col_local(T,'t_reach',NaN(n,1));
Tc.P_grid_lpf_peak_W = first_numeric_col_local(T,{'P_peak_lpf','Plpf_peak_W','P_peak'},NaN(n,1));
Tc.P_grid_lpf_p99_W = first_numeric_col_local(T,{'P_p99_lpf','Plpf_p99_W','P_p99'},NaN(n,1));
Tc.pbar_peak_norm = first_numeric_col_local(T,{'pbar_peak','P_peak_lpf_norm','P_peak_norm'},NaN(n,1));
Tc.pbar_p99_norm = first_numeric_col_local(T,{'pbar_p99','P_p99_lpf_norm','P_p99_norm'},NaN(n,1));
Tc.E_excess_norm_s = numeric_col_local(T,'E_excess_norm_s',NaN(n,1));
Tc.pbar_ramp_p95_per_s = first_numeric_col_local(T,{'pbar_ramp_p95','ramp_p95_lpf_norm','ramp_p95_norm'},NaN(n,1));
Tc.pbar_ramp_max_per_s = first_numeric_col_local(T,{'pbar_ramp_max','ramp_max_lpf_norm','ramp_max_norm'},NaN(n,1));
Tc.E_grid_J = numeric_col_local(T,'E_grid',NaN(n,1));
Tc.E_reuse_J = numeric_col_local(T,'E_reuse',NaN(n,1));
Tc.E_dump_J = numeric_col_local(T,'E_dump',NaN(n,1));
Tc.E_spill_J = numeric_col_local(T,'E_spill',NaN(n,1));
Tc.P_grid_req_peak_W = numeric_col_local(T,'P_request_peak',NaN(n,1));
Tc.P_src_short_peak_W = numeric_col_local(T,'Psource_shortfall_peak',NaN(n,1));
Tc.E_src_shortfall_J = numeric_col_local(T,'E_source_shortfall',NaN(n,1));
Tc.I_grid_rms_A = numeric_col_local(T,'I_rms',NaN(n,1));
Tc.I_grid_peak_A = numeric_col_local(T,'I_peak',NaN(n,1));
Tc.Vdc_min_V = numeric_col_local(T,'Vdc_min',NaN(n,1));
Tc.source_limited_fraction = numeric_col_local(T,'source_limited_frac',NaN(n,1));
end

function Sc = make_summary_canonical_local(S, stage_label)
if ismember('method',S.Properties.VariableNames), S.method = string(S.method); end
if ismember('metric',S.Properties.VariableNames), S.metric = string(S.metric); end
D = variable_dictionary_local();
Sc = table();
Sc.stage = repmat(string(stage_label),height(S),1);
Sc.method = string_col_local(S,'method',strings(height(S),1));
Sc.method_role = method_role_local(Sc.method);
Sc.metric_raw = string_col_local(S,'metric',strings(height(S),1));
Sc.metric_canonical = strings(height(S),1);
Sc.symbol = strings(height(S),1);
Sc.unit = strings(height(S),1);
Sc.description = strings(height(S),1);
Sc.smaller_is_better = strings(height(S),1);
for i=1:height(S)
    raw = string(S.metric(i));
    idx = find(D.raw_source_metric == raw,1);
    if isempty(idx)
        Sc.metric_canonical(i) = raw;
        Sc.symbol(i) = "";
        Sc.unit(i) = "";
        Sc.description(i) = "Raw metric not in dictionary";
        Sc.smaller_is_better(i) = "";
    else
        Sc.metric_canonical(i) = D.canonical_variable(idx);
        Sc.symbol(i) = D.manuscript_symbol(idx);
        Sc.unit(i) = D.unit(idx);
        Sc.description(i) = D.description(idx);
        Sc.smaller_is_better(i) = D.smaller_is_better(idx);
    end
end
copyVars = {'N','mean','std','median','p05','p95'};
for k=1:numel(copyVars)
    v = copyVars{k};
    Sc.(v) = numeric_col_local(S,v,NaN(height(S),1));
end
end

function Pc = make_paired_canonical_local(P, stage_label)
if isempty(P), Pc = table(); return; end
if ismember('reference',P.Properties.VariableNames), P.reference = string(P.reference); end
if ismember('method',P.Properties.VariableNames), P.method = string(P.method); end
if ismember('metric',P.Properties.VariableNames), P.metric = string(P.metric); end
D = variable_dictionary_local();
Pc = table();
Pc.stage = repmat(string(stage_label),height(P),1);
Pc.reference_method = string_col_local(P,'reference',strings(height(P),1));
Pc.method = string_col_local(P,'method',strings(height(P),1));
Pc.reference_role = method_role_local(Pc.reference_method);
Pc.method_role = method_role_local(Pc.method);
Pc.metric_raw = string_col_local(P,'metric',strings(height(P),1));
Pc.metric_canonical = strings(height(P),1);
Pc.symbol = strings(height(P),1);
Pc.unit = strings(height(P),1);
for i=1:height(P)
    idx = find(D.raw_source_metric == string(P.metric(i)),1);
    if isempty(idx)
        Pc.metric_canonical(i) = string(P.metric(i));
    else
        Pc.metric_canonical(i) = D.canonical_variable(idx);
        Pc.symbol(i) = D.manuscript_symbol(idx);
        Pc.unit(i) = D.unit(idx);
    end
end
copyVars = {'N','ref_minus_method_mean','ref_minus_method_std','method_improved_count','method_worse_count'};
for k=1:numel(copyVars)
    v = copyVars{k}; Pc.(v) = numeric_col_local(P,v,NaN(height(P),1));
end
Pc.paired_difference_definition = repmat("reference minus method; positive means the method is lower than reference for smaller-is-better metrics",height(P),1);
end

function role = method_role_local(method)
method = string(method);
role = strings(size(method));
for ii=1:numel(method)
    switch string(method(ii))
        case "MLP-PRG"
            role(ii) = "proposed compact non-attention learned PRG surrogate";
        case "TRF-PRG"
            role(ii) = "proposed look-ahead sequence learned PRG surrogate";
        case "Feasible-RG"
            role(ii) = "online feasibility-search reference governor";
        case "NoGuard"
            role(ii) = "budget-agnostic reference case";
        case "Bus-PTP-DP"
            role(ii) = "offline path time-parameterization reference";
        case "MPC-RG-grid"
            role(ii) = "receding-horizon reference-governor reference";
        otherwise
            role(ii) = "other";
    end
end
end

function D = variable_dictionary_local()
rows = {
'rollout_wall_time_s','', 's','MATLAB wall-clock time for one complete closed-loop simulation rollout and table/logging overhead; not neural inference latency','runtime_s legacy','no';
'offline_profile_solve_wall_time_s','', 's','Offline profile solver wall-clock time when a method such as Bus-PTP-DP records it','offline_solve_time_s legacy','no';
't_reach_s','t_reach','s','Goal reaching time','t_reach','yes';
'P_grid_lpf_peak_W','max P_grid,LPF','W','Peak low-pass filtered delivered grid-input power','P_peak_lpf','yes';
'P_grid_lpf_p99_W','p99(P_grid,LPF)','W','99th percentile of low-pass filtered delivered grid-input power','P_p99_lpf','yes';
'pbar_peak_norm','max pbar','-','Peak filtered normalized delivered grid-input power, P_grid/P_grid_budget after the evaluation LPF','pbar_peak','yes';
'pbar_p99_norm','p99(pbar)','-','99th percentile of filtered normalized delivered grid-input power','pbar_p99','yes';
'E_excess_norm_s','E_excess tilde','s','Integrated normalized budget exceedance above 1+rho','E_excess_norm_s','yes';
'pbar_ramp_p95_per_s','p95(|dpbar/dt|)','s^-1','95th percentile absolute ramp rate of filtered normalized delivered grid-input power','pbar_ramp_p95','yes';
'pbar_ramp_max_per_s','max |dpbar/dt|','s^-1','Maximum absolute ramp rate of filtered normalized delivered grid-input power','pbar_ramp_max','yes';
'E_grid_J','E_grid','J','Delivered source-side grid-input energy','E_grid','context';
'E_reuse_J','E_reuse','J','Immediately reused regenerative energy','E_reuse','context';
'E_dump_J','E_dump','J','Energy dissipated by braking/dump branch','E_dump','context';
'E_spill_J','E_spill','J','Regenerative energy not reused/buffered/dumped under modeled limits','E_spill','context';
'P_grid_req_peak_W','max P_grid,req','W','Peak pre-limit servo/bus electrical demand before source-current limiter','P_request_peak','diagnostic';
'P_src_short_peak_W','max P_src,short','W','Peak source-power shortfall between requested and delivered grid power','Psource_shortfall_peak','yes';
'E_src_shortfall_J','E_src,short','J','Integrated source-power shortfall','E_source_shortfall','yes';
'I_grid_rms_A','I_rms','A','RMS delivered source current','I_rms','yes';
'I_grid_peak_A','I_peak','A','Peak delivered source current','I_peak','yes';
'Vdc_min_V','Vdc,min','V','Minimum modeled DC-link voltage; larger is better','Vdc_min','no';
'source_limited_fraction','source-limited fraction','-','Fraction of servo samples in which source-current limiting was active','source_limited_frac','yes'};
D = cell2table(rows,'VariableNames',{'canonical_variable','manuscript_symbol','unit','description','raw_source_metric','smaller_is_better'});
D.canonical_variable = string(D.canonical_variable);
D.manuscript_symbol = string(D.manuscript_symbol);
D.unit = string(D.unit);
D.description = string(D.description);
D.raw_source_metric = string(D.raw_source_metric);
D.smaller_is_better = string(D.smaller_is_better);
end

function v = col_local(T,name,def)
if ismember(name,T.Properties.VariableNames), v = T.(name); else, v = def; end
end
function v = numeric_col_local(T,name,def)
v = col_local(T,name,def);
try, v = double(v); catch, v = def; end
if isrow(v), v = v.'; end
end
function v = first_numeric_col_local(T,names,def)
v = def;
for k=1:numel(names)
    if ismember(names{k},T.Properties.VariableNames)
        v = numeric_col_local(T,names{k},def); return;
    end
end
end
function v = string_col_local(T,name,def)
v = col_local(T,name,def);
try, v = string(v); catch, v = string(def); end
if isrow(v), v = v.'; end
end
function v = logical_col_local(T,name,def)
v = col_local(T,name,def);
try, v = logical(v); catch, v = def; end
if isrow(v), v = v.'; end
end
