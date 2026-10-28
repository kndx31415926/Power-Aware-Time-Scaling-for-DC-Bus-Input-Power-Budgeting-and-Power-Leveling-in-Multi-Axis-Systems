function [L,parts] = spinn3d_soft_power_speed_loss( ...
        Pavg_trace_W,Pavg_target_W,alpha_post,alpha_teacher_post, ...
        alpha_min,alpha_max,lambda_pavg_sq,lambda_progress)
%SPINN3D_SOFT_POWER_SPEED_LOSS Elastic soft-power/teacher-progress objective.
%
%   L = lambda_pavg_sq * mean([Pavg/Ptarget - 1]_+^2)
%       + lambda_progress * mean( ...
%           ([alpha_teacher_post-alpha_post]_+/(alpha_max-alpha_min))^2)
%
% Pavg_trace_W must already be the causal EMA replay/prediction. Power below
% the target contributes exactly zero: there is deliberately no under-budget
% penalty and no reward for riding the limit. The independent progress term
% penalizes only needless progress shortfall relative to the physical/soft-
% power teacher. It neither anchors Alpha at one nor rewards a prediction for
% exceeding its teacher. Numeric arrays and dlarray values are both supported.

% This is a soft objective, not a hard feasibility test. Finite unavoidable
% exceedance therefore remains representable and receives a finite cost.

target_safe = max(Pavg_target_W,single(1e-6));
normalized_excess = max(Pavg_trace_W./target_safe-single(1),single(0));
power_term = mean(normalized_excess.^2,'all');
alpha_span = single(double(alpha_max)-double(alpha_min));
progress_shortfall = max(alpha_teacher_post-alpha_post,single(0))./alpha_span;
progress_term = mean(progress_shortfall.^2,'all');
L = single(lambda_pavg_sq).*power_term + ...
    single(lambda_progress).*progress_term;

if nargout>1
    parts = struct('normalized_square_excess',power_term, ...
        'teacher_relative_progress_shortfall',progress_term, ...
        'progress_cost',progress_term, ...
        'alpha_min',double(alpha_min),'alpha_max',double(alpha_max), ...
        'schema', ...
        'causal_ema_normalized_square_excess_plus_teacher_progress_v2');
end
end
