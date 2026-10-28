% Power-aware time scaling for shared-DC-bus multi-axis systems.
%
% Reproduction
%   run_all_results          - Run the studies, summaries and MATLAB figures.
%   run_study                - Run a study or selected task IDs.
%   compute_report           - Calculate paired effects and report tables.
%   reviewer_train_models    - Generate teacher data and train the policies.
%
% Task and experimental settings
%   reviewer_configuration   - Bind models and physical settings to a task.
%   reviewer_conditions      - Specify perturbation and sensitivity levels.
%   reviewer_filter          - Recalculate metrics on saved power traces.
%   reviewer_inference       - Optional batch-one network forward timing.
%   reviewer_computational_workload - Calculate network MACs and search bounds.
%
% Geometry and coupled physical model (src)
%   spinn3d_build_robot       - Construct the rigid-body robot model.
%   make_traj_cart_bezier_dls - Construct the fixed path and its derivatives.
%   spinn3d_plant_step        - Integrate robot motion and electrical states.
%   spinn3d_motor_dc_kernel   - Evaluate motor current and electrical power.
%   spinn3d_dc_link_kernel    - Integrate the source-capacitor power balance.
%
% Policies and time scaling (src)
%   spinn3d_controller_ctcgov - Track the time-scaled path using computed torque.
%   spinn3d_control_schedule  - Calculate the policy-update cadence.
%   alpha_feasible           - Select a feasible progress-rate command.
%   spinn3d_bus_ptp_dp_r2     - Plan time scaling with a bounded beam search.
%   spinn3d_mpc_rg_grid_step_r2 - Update a nominal-reference grid policy.
%
% Learning (src)
%   spinn3d_build_alpha_network       - Build the Transformer or MLP policy.
%   spinn3d_train_alpha_gov_trf        - Optimize the selected learning loss.
%   spinn3d_alpha_teacher_state       - Calculate state-dependent teacher rates.
%   spinn3d_alpha_policy_power_loss   - Evaluate the differentiable physical loss.
%   spinn3d_features_alpha_plus_payload_gov - Form policy input features.
%
% See README.txt for experiment counts, output paths and manuscript mapping.

% representative_motion_data - Extract the common-task motion and power histories.
