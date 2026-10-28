function run_all_results()
% Reproduce the main comparison, ablation, perturbation and sensitivity data.
% Saved method trajectories are reused when this command is run again.
root=fileparts(mfilename('fullpath'));
addpath(root,fullfile(root,'src'),fullfile(root,'plots'));
run_study('main');
run_study('robustness');
run_study('parameter');
run_study('filter');
compute_report();
end
