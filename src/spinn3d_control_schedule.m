function contract=spinn3d_control_schedule(Ts,policy,~,actual_guard,alpha_mode)
% Compute the policy cadence and shared physical-admission schedule.
if nargin<4,actual_guard=[];end
if nargin<5,alpha_mode='';end
contract=struct('policy_dt',double(policy.dt), ...
    'policy_stride',round(double(policy.dt)/double(Ts)), ...
    'policy_hold',lower(strtrim(char(policy.hold))), ...
    'power_budget_semantics','central_filtered_experiment_scan_value', ...
    'fixed_budget_value_required',false);
if ~isempty(alpha_mode)
    alpha_mode=lower(strtrim(char(alpha_mode)));
    modes=cellstr(string(actual_guard.apply_to_modes));
    contract.alpha_mode=alpha_mode;
    contract.actual_guard_applies=logical(actual_guard.enable)&&any(strcmp(alpha_mode,modes));
end
end
