function [a_exec, ratio] = spinn3d_execution_progress_alpha(a_command,step_audit,alpha_max)
%SPINN3D_EXECUTION_PROGRESS_ALPHA Commit post-projection path progress.
% The controller proposes a_command, while the final post-noise safety layer
% may retain only part of its requested acceleration.  Reference progress and
% controller a_prev must use the same executed fraction as the plant step.

required_audit = {'execution_progress_ratio', ...
    'final_state_verified_before_commit','final_motion'};

verified = step_audit.final_state_verified_before_commit;
motion_pass = step_audit.final_motion.pass;
verified_ok = (islogical(verified) || isnumeric(verified)) && ...
    isreal(verified) && isscalar(verified) && ...
    isfinite(double(verified)) && double(verified)==1;
motion_ok = (islogical(motion_pass) || isnumeric(motion_pass)) && ...
    isreal(motion_pass) && isscalar(motion_pass) && ...
    isfinite(double(motion_pass)) && double(motion_pass)==1;

ratio = double(step_audit.execution_progress_ratio);

a_exec = double(a_command)*ratio;
end
