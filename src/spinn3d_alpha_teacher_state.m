function varargout=spinn3d_alpha_teacher_state(action,varargin)
%SPINN3D_ALPHA_TEACHER_STATE Measured-state, finite-preview Alpha teacher.
% State S contains q,dq,s,a_prev,bus,Pavg. No elapsed clock or deadline is used.
% The known model produces CTC torque and causal midpoint plant/DC updates.
% A high request is never admitted because a physical projector rescued it:
% only the shared kinematic governor precedes unprojected hard admission.
% select evaluates every parent on the SAME total preview duration. Future
% policy ticks use a deterministic physical continuation heuristic (hold first,
% then local/global alternatives), not an exhaustive or optimal MPC search.
switch char(action)
    case 'select'
        [varargout{1:nargout}]=select_local(varargin{:});
    case 'preview'
        varargout{1}=preview_local(varargin{:});
    case 'step'
        [varargout{1:nargout}]=step_local(varargin{:});
    otherwise
        error('spinn3d:UnknownStateTeacherAction','Unknown teacher action.');
end
end

function [command,D]=select_local(robot,T,S,C)
validate_state_local(S,T,C);
commands=candidates_local(T,S,C);
trials=cell(1,numel(commands));
successor_evals=0;
for k=1:numel(commands)
    trials{k}=rollout_local(robot,T,S,commands(k),C);
    successor_evals=successor_evals+trials{k}.successor_candidate_evaluations;
end
refined=0;
adaptive=false;
% Retain the centrally defined micro-command offsets from the previous
% teacher: a wholly infeasible coarse grid cannot bracket a narrow island.
if ~any(cellfun(@(d)d.physical_ok,trials))
    adaptive=true;
    extra=unique(min(C.alpha_max,max(C.alpha_min, ...
        S.a_prev+C.refine_offsets)),'stable');
    for k=1:numel(extra)
        if any(abs(commands-extra(k))<1e-13), continue; end
        Q=rollout_local(robot,T,S,extra(k),C);
        commands(end+1)=extra(k); trials{end+1}=Q; %#ok<AGROW>
        successor_evals=successor_evals+Q.successor_candidate_evaluations;
    end
end
% Narrow braking islands retain the earlier contract's deterministic
% transition refinement. Feasibility is never assumed monotone globally.
if ~any(cellfun(@(d)d.physical_ok,trials))
    [sorted,order]=sort(commands); old=trials(order);
    for k=1:numel(sorted)-1
        left=sorted(k); right=sorted(k+1); L=old{k}; R=old{k+1};
        if L.current_interval_ok==R.current_interval_ok, continue; end
        for j=1:C.transition_refine_iterations
            mid=(left+right)/2;
            Q=rollout_local(robot,T,S,mid,C);
            commands(end+1)=mid; trials{end+1}=Q; %#ok<AGROW>
            refined=refined+1;
            successor_evals=successor_evals+Q.successor_candidate_evaluations;
            if Q.current_interval_ok==L.current_interval_ok
                left=mid; L=Q;
            else
                right=mid;
            end
        end
    end
end
good=find(cellfun(@(d)d.physical_ok,trials));
parent_refinement=struct('used',false,'evaluations',0,'plant_steps',0, ...
    'successor_evaluations',0,'successor_plant_steps',0,'current_safe_candidates',0, ...
    'max_evaluations',64,'max_depth',12,'budget_exhausted',false);
refinement=struct('used',false,'parents',0,'evaluations',0,'plant_steps',0, ...
    'budget_exhausted_parents',0,'max_evaluations_per_parent',64,'max_depth',12);
% A normally admitted selection never enters either bounded recovery stage.
if isempty(good)
    parents=find(cellfun(@(d)d.current_interval_ok && ...
        isfield(d,'failed_successor_trials'),trials));
    [trials,successor_evals,refinement]=recover_successors_local( ...
        robot,T,S,C,parents,trials,successor_evals,refinement);
    good=find(cellfun(@(d)d.physical_ok,trials));
end
if isempty(good)
    % Even with some current-safe parents, a different narrow parent island
    % may have a viable continuation. Refine only adjacent current-UNSAFE
    % commands with different first violated constraints, without plan memory.
    first_new=numel(commands)+1;
    [new_commands,new_trials,parent_refinement]=refine_parent_boundaries_local( ...
        robot,T,S,C,commands,trials,parent_refinement);
    commands=[commands,new_commands];trials=[trials,new_trials];
    successor_evals=successor_evals+parent_refinement.successor_evaluations;
    good=find(cellfun(@(d)d.physical_ok,trials));
    if isempty(good)
        % Only new parents need successor recovery; never repeat old work.
        parents=first_new:numel(commands);
        parents=parents(cellfun(@(d)d.current_interval_ok && ...
            isfield(d,'failed_successor_trials'),trials(parents)));
        [trials,successor_evals,refinement]=recover_successors_local( ...
            robot,T,S,C,parents,trials,successor_evals,refinement);
        good=find(cellfun(@(d)d.physical_ok,trials));
    end
end
if isempty(good)
    % This is a rejection, never a teacher label. Runtime may forward the
    % finite hold request to its authoritative physical recovery layer;
    % generators must reject it before admitting a row to the corpus.
    command=min(C.alpha_max,max(C.alpha_min,S.a_prev));
    D=trials{1}; D.physical_ok=false; D.recursive_viable=false;
    D.a_des_command=command; D.score=inf;
    D.candidate_evaluations=numel(commands);
    D.successor_candidate_evaluations=successor_evals;
    D.transition_refinement_evaluations=refined;
    D.adaptive_expansion_used=adaptive || refined>0;
    D.candidate_commands=commands;
    D.candidate_physical_ok=false(size(commands));
    D.candidate_scores=inf(size(commands));
    D.selection_mode='rejected_no_unprojected_hard_safe_continuation';
    D=selection_diagnostics_local(D,trials,T,S,C,false);
    D.successor_refinement=refinement;
    D.parent_refinement=parent_refinement;
    return;
end
% Stable deterministic tie handling: exact same candidate set and physical
% continuation are used in NoGuard; only the power score term is disabled.
scores=cellfun(@(d)d.score,trials(good));
[~,ii]=min(scores); pick=good(ii);
command=commands(pick); D=trials{pick};
D.a_des_command=command; D.candidate_evaluations=numel(commands);
D.successor_candidate_evaluations=successor_evals;
D.transition_refinement_evaluations=refined;
D.adaptive_expansion_used=adaptive || refined>0;
D.recursive_viable=true; D.seed_selected=false;
D.selection_mode='measured_ctc_equal_horizon_weighted_progress_softpavg';
D.candidate_commands=commands;
D.candidate_physical_ok=cellfun(@(d)d.physical_ok,trials);
D.candidate_scores=cellfun(@(d)d.score,trials);
D=selection_diagnostics_local(D,trials,T,S,C,true);
D.successor_refinement=refinement;
    D.parent_refinement=parent_refinement;
end

function D=selection_diagnostics_local(D,trials,T,S,C,selected)
% Versioned observational fields. The merged rollout fields retain their

D.diagnostic_schema='measured_ctc_selector_diagnostics_v1';
D.selected_preview_available=logical(selected);
D.fallback_requested=~selected;
D.desired_dt_s=C.policy_dt; D.applied_dt_s=C.Ts;
D.horizon_requested_dt_s=C.horizon_policy_ticks*C.policy_dt;
D.progress_upper_bound=min(max(T.Tf-S.s,0),C.alpha_max*C.policy_dt);
D.current_plant_steps_checked=sum(cellfun( ...
    @(p)p.first_interval.plant_steps_checked,trials));
D.successor_plant_steps_checked=sum(cellfun( ...
    @(p)p.rollout_successor_plant_steps_checked,trials));
D.selected_first_interval=struct(); D.selected_first_successor=struct();
D.selected_horizon=struct();
if selected
    D.selected_first_interval=D.first_interval;
    D.selected_first_successor=D.first_successor;
    D.selected_horizon=interval_diagnostics_local(D);
end
end

function commands=candidates_local(T,S,C)
[~,v,a]=T.eval(S.s); v=row(v); a=row(a);
kin=min([C.alpha_max,min(expand_local(C.caps.qd_max,numel(v))./max(abs(v),1e-12)), ...
    min(sqrt(expand_local(C.caps.qdd_max,numel(v))./max(abs(a),1e-12)))]);
commands=[C.alpha_max,S.a_prev,S.a_prev+C.local_offsets(:).', ...
    linspace(C.alpha_min,C.alpha_max,C.grid_points),kin];
commands=unique(min(C.alpha_max,max(C.alpha_min,double(commands))),'stable');
end

function D=rollout_local(robot,T,S,command,C)
initial=S; D=preview_local(robot,T,S,command,C,C.policy_steps);
D.current_interval_ok=D.physical_ok;
% Capture the first interval before merge_preview_local extends the rollout.
D.first_interval=interval_diagnostics_local(D);
D.first_successor=struct();
D.successor_candidate_evaluations=0;
D.rollout_successor_plant_steps_checked=0;
D.successor_failure_reason='none';
D.successor_commands=[];
if ~D.physical_ok, return; end
S=D.state_final; previous=command;
for tick=2:C.horizon_policy_ticks
    successors=[previous,S.a_prev,S.a_prev+C.local_offsets(:).', ...
        S.a_prev+C.refine_offsets, ...
        linspace(C.alpha_min,C.alpha_max,C.grid_points)];
    successors=unique(min(C.alpha_max,max(C.alpha_min,successors)),'stable');
    found=false; child_trials=cell(1,numel(successors));
    for j=1:numel(successors)
        child=preview_local(robot,T,S,successors(j),C,C.policy_steps);
        child_trials{j}=child;
        D.successor_candidate_evaluations=D.successor_candidate_evaluations+1;
        D.rollout_successor_plant_steps_checked= ...
            D.rollout_successor_plant_steps_checked+child.plant_steps_checked;
        if child.physical_ok
            found=true; previous=successors(j); break;
        end
    end
    if ~found
        D.physical_ok=false; D.recursive_viable=false; D.score=inf;
        D.successor_failure_reason=child.failure_reason;
        D.failure_reason=['successor_' child.failure_reason];
        D.failed_successor_commands=successors;
        D.failed_successor_trials=child_trials;
        D.failed_successor_tick=tick;
        D.failed_successor_previous=previous;
        return;
    end
    if tick==2
        D.first_successor=interval_diagnostics_local(child);
    end
    D=merge_preview_local(D,child);
    D.successor_commands(end+1)=previous;
    S=child.state_final;
end
D.recursive_viable=true;
D=score_local(D,T,initial,C);
end

function [trials,successor_evals,N]=recover_successors_local( ...
    robot,T,S,C,parents,trials,successor_evals,N)
for parent_index=parents
    old=trials{parent_index};
    [Q,R]=refine_failed_successor_local(robot,T,S,old,C,N);
    N.used=true;N.parents=N.parents+1;
    N.evaluations=N.evaluations+R.evaluations;N.plant_steps=N.plant_steps+R.plant_steps;
    N.budget_exhausted_parents=N.budget_exhausted_parents+R.budget_exhausted;
    successor_evals=successor_evals+Q.successor_candidate_evaluations-old.successor_candidate_evaluations;
    trials{parent_index}=Q;
end
end

function [new_commands,new_trials,N]=refine_parent_boundaries_local( ...
    robot,T,S,C,commands,trials,N)
% Only adjacent current-unsafe parents seed this bounded recovery.
% Every midpoint still receives the complete normal equal-horizon rollout.
[values,order]=sort(commands);trials=trials(order);queue=cell(0,1);
for k=1:numel(values)-1
    if ~trials{k}.current_interval_ok && ~trials{k+1}.current_interval_ok && ...
            ~strcmp(trials{k}.failure_reason,trials{k+1}.failure_reason)
        queue{end+1,1}=struct('left',values(k),'right',values(k+1), ...
            'L',trials{k},'R',trials{k+1},'depth',0); %#ok<AGROW>
    end
end
new_commands=zeros(1,0);new_trials=cell(1,0);N.used=true;head=1;
while head<=numel(queue) && N.evaluations<N.max_evaluations
    b=queue{head};head=head+1;
    if b.depth>=N.max_depth || b.right-b.left<=1e-12,continue;end
    mid=(b.left+b.right)/2;Q=rollout_local(robot,T,S,mid,C);
    new_commands(end+1)=mid;new_trials{end+1}=Q; %#ok<AGROW>
    N.evaluations=N.evaluations+1;
    N.plant_steps=N.plant_steps+Q.first_interval.plant_steps_checked;
    N.successor_evaluations=N.successor_evaluations+Q.successor_candidate_evaluations;
    N.successor_plant_steps=N.successor_plant_steps+Q.rollout_successor_plant_steps_checked;
    if Q.current_interval_ok
        N.current_safe_candidates=N.current_safe_candidates+1;
        % Continue normal horizon admission/successor recovery in select.
        % This bracket now contains an observed safe parent; do not count
        % hypothetical monotonicity as further feasible commands.
        continue;
    end
    if ~strcmp(b.L.failure_reason,Q.failure_reason)
        queue{end+1,1}=struct('left',b.left,'right',mid, ...
            'L',b.L,'R',Q,'depth',b.depth+1); %#ok<AGROW>
    end
    if ~strcmp(Q.failure_reason,b.R.failure_reason)
        queue{end+1,1}=struct('left',mid,'right',b.right, ...
            'L',Q,'R',b.R,'depth',b.depth+1); %#ok<AGROW>
    end
end
N.budget_exhausted=head<=numel(queue) && N.evaluations>=N.max_evaluations;
end

function [D,R]=refine_failed_successor_local(robot,T,initial,D,C,budget)

R=struct('evaluations',0,'plant_steps',0,'budget_exhausted',false);
S=D.state_final; previous=D.failed_successor_previous;
start_tick=D.failed_successor_tick;
old_commands=D.failed_successor_commands; old_trials=D.failed_successor_trials;
D=rmfield(D,{'failed_successor_commands','failed_successor_trials', ...
    'failed_successor_tick','failed_successor_previous'});
for tick=start_tick:C.horizon_policy_ticks
    if tick==start_tick
        commands=old_commands; children=old_trials; found=false;
    else
        commands=unique(min(C.alpha_max,max(C.alpha_min,[previous,S.a_prev, ...
            S.a_prev+C.local_offsets(:).',S.a_prev+C.refine_offsets, ...
            linspace(C.alpha_min,C.alpha_max,C.grid_points)])),'stable');
        children=cell(1,numel(commands));found=false;
        for j=1:numel(commands)
            child=preview_local(robot,T,S,commands(j),C,C.policy_steps);
            children{j}=child;
            D.successor_candidate_evaluations=D.successor_candidate_evaluations+1;
            D.rollout_successor_plant_steps_checked=D.rollout_successor_plant_steps_checked+child.plant_steps_checked;
            if child.physical_ok,found=true;previous=commands(j);break;end
        end
    end
    if ~found
        left_budget=budget.max_evaluations_per_parent-R.evaluations;
        [found,child,command,N]=refine_failure_boundaries_local( ...
            robot,T,S,C,commands,children,left_budget,budget.max_depth);
        R.evaluations=R.evaluations+N.evaluations;R.plant_steps=R.plant_steps+N.plant_steps;
        R.budget_exhausted=R.budget_exhausted || N.budget_exhausted;
        D.successor_candidate_evaluations=D.successor_candidate_evaluations+N.evaluations;
        D.rollout_successor_plant_steps_checked=D.rollout_successor_plant_steps_checked+N.plant_steps;
        if found,previous=command;end
    end
    if ~found
        D.physical_ok=false;D.recursive_viable=false;D.score=inf;return;
    end
    if tick==2,D.first_successor=interval_diagnostics_local(child);end
    D=merge_preview_local(D,child);D.successor_commands(end+1)=previous;
    S=child.state_final;
end
D.physical_ok=true;D.recursive_viable=true;
D.failure_reason='feasible';D.successor_failure_reason='none';
D=score_local(D,T,initial,C);
end

function [found,child,command,N]=refine_failure_boundaries_local( ...
    robot,T,S,C,commands,trials,max_evaluations,max_depth)
% A different first violated constraint brackets a possible feasible island,
% not a proof of monotone feasibility. Every proposed midpoint is simulated.
[values,order]=sort(commands);trials=trials(order);
queue=cell(0,1);
for k=1:numel(values)-1
    if ~strcmp(trials{k}.failure_reason,trials{k+1}.failure_reason)
        queue{end+1,1}=struct('left',values(k),'right',values(k+1), ...
            'L',trials{k},'R',trials{k+1},'depth',0); %#ok<AGROW>
    end
end
found=false;command=NaN;child=struct();
N=struct('evaluations',0,'plant_steps',0,'budget_exhausted',false);
head=1;
while head<=numel(queue) && N.evaluations<max_evaluations
    b=queue{head};head=head+1;
    if b.depth>=max_depth || b.right-b.left<=1e-12,continue;end
    mid=(b.left+b.right)/2;
    Q=preview_local(robot,T,S,mid,C,C.policy_steps);
    N.evaluations=N.evaluations+1;N.plant_steps=N.plant_steps+Q.plant_steps_checked;
    if Q.physical_ok,found=true;command=mid;child=Q;return;end
    if ~strcmp(b.L.failure_reason,Q.failure_reason)
        queue{end+1,1}=struct('left',b.left,'right',mid, ...
            'L',b.L,'R',Q,'depth',b.depth+1); %#ok<AGROW>
    end
    if ~strcmp(Q.failure_reason,b.R.failure_reason)
        queue{end+1,1}=struct('left',mid,'right',b.right, ...
            'L',Q,'R',b.R,'depth',b.depth+1); %#ok<AGROW>
    end
end
N.budget_exhausted=head<=numel(queue) && N.evaluations>=max_evaluations;
end

function P=interval_diagnostics_local(D)

% The objective's measured physical_path_advance remains a distinct field.
P=struct('command',D.a_des_command,'physical_ok',D.physical_ok, ...
    'end_alpha',D.a_final,'progress',D.s_advance, ...
    'plant_steps_checked',D.plant_steps_checked,'duration_s',D.duration_s);
end

function D=preview_local(robot,T,S,command,C,nsteps)
if nargin<6, nsteps=C.policy_steps; end
validate_state_local(S,T,C);

initial=S;
D=struct('physical_ok',true,'failure_reason','feasible','state_final',S, ...
    'power_enabled',logical(C.power_enabled), ...
    'plant_steps_checked',0,'duration_s',0,'score',inf,'score_terms',struct(), ...
    'Pgrid_avg_peak_W',S.Pavg,'Pavg_excess_peak_W',0, ...
    'Pavg_excess_sq_normalized_integral_s',0,'Pavg_excess_integral_J',0, ...
    'tracking_integral_s',0,'smooth_integral_s',0,'reserve_integral_s',0, ...
    'tau_ratio_peak',0,'qdd_ratio_peak',0,'qd_ratio_peak',0, ...
    'motor_ratio_peak',0,'axis_power_ratio_peak',0,'grid_peak_ratio_peak',0, ...
    'candidate_evaluations',0,'successor_candidate_evaluations',0, ...
    'transition_refinement_evaluations',0,'adaptive_expansion_used',false, ...
    'recursive_viable',false,'seed_selected',false, ...
    'a_des_command',command,'s_initial',S.s,'q_initial',row(S.q));
for k=1:nsteps
    [next,I]=step_local(robot,T,S,command,C);
    D.plant_steps_checked=k; D.last_step=I;
    if ~I.physical_ok
        D.physical_ok=false; D.failure_reason=I.failure_reason;
        D.failure_step=k; D.score=inf; break;
    end
    S=next; D.duration_s=D.duration_s+C.Ts;
    D.Pgrid_avg_peak_W=max(D.Pgrid_avg_peak_W,S.Pavg);
    D.Pavg_excess_peak_W=max(D.Pavg_excess_peak_W,I.Pavg_excess_W);
    D.Pavg_excess_sq_normalized_integral_s= ...
        D.Pavg_excess_sq_normalized_integral_s+C.Ts*(I.Pavg_excess_W/I.Pavg_target_W)^2;
    D.Pavg_excess_integral_J=D.Pavg_excess_integral_J+C.Ts*I.Pavg_excess_W;
    D.tracking_integral_s=D.tracking_integral_s+C.Ts*I.tracking_cost;
    D.smooth_integral_s=D.smooth_integral_s+C.Ts*I.smooth_cost;
    D.reserve_integral_s=D.reserve_integral_s+C.Ts*I.reserve_cost;
    for name={'tau_ratio_peak','qdd_ratio_peak','qd_ratio_peak', ...
            'motor_ratio_peak','axis_power_ratio_peak','grid_peak_ratio_peak'}
        D.(name{1})=max(D.(name{1}),I.(name{1}));
    end
end
D.state_final=S; D.s_final=S.s; D.a_final=S.a_prev;
D.st_final=S.bus; D.Pavg_final_W=S.Pavg; D.s_advance=S.s-initial.s;
D.tau_reserve_met=D.tau_ratio_peak<=C.tau_reserve_ratio;
D.qdd_reserve_met=D.qdd_ratio_peak<=C.qdd_reserve_ratio;
D=score_local(D,T,initial,C);
end

function D=merge_preview_local(D,Q)
for name={'duration_s','plant_steps_checked','Pavg_excess_sq_normalized_integral_s', ...
    'Pavg_excess_integral_J','tracking_integral_s','smooth_integral_s','reserve_integral_s'}
    D.(name{1})=D.(name{1})+Q.(name{1});
end
for name={'Pgrid_avg_peak_W','Pavg_excess_peak_W','tau_ratio_peak','qdd_ratio_peak', ...
    'qd_ratio_peak','motor_ratio_peak','axis_power_ratio_peak','grid_peak_ratio_peak'}
    D.(name{1})=max(D.(name{1}),Q.(name{1}));
end
D.state_final=Q.state_final; D.s_final=Q.s_final; D.a_final=Q.a_final;
D.st_final=Q.st_final; D.Pavg_final_W=Q.Pavg_final_W; D.last_step=Q.last_step;
D.s_advance=D.s_final-D.s_initial;
D.tau_reserve_met=D.tau_reserve_met && Q.tau_reserve_met;
D.qdd_reserve_met=D.qdd_reserve_met && Q.qdd_reserve_met;
end

function D=score_local(D,T,S,C)
if ~D.physical_ok, D.score=inf; return; end
h=D.duration_s; endstate=D.state_final;
[q0,~,~]=T.eval(S.s);
ds=min(max(T.Tf-S.s,0),C.alpha_max*h);
[qf,~,~]=T.eval(min(T.Tf,S.s+ds));
chord=row(qf)-row(q0); displacement=row(endstate.q)-row(S.q);
% Signed actual displacement along the local forward reference chord. The
% floor regularizes zero endpoint derivatives; progress cannot be obtained
% by advancing reference s while the physical arm remains still.
den=max(sum(chord.^2),(C.progress_velocity_floor_rad_s*h)^2);
physical_progress=ds*sum(displacement.*chord)/den;
progress=physical_progress/max(C.alpha_max*h,eps);
[qtarget,~,~]=T.eval(T.Tf);
terminal_gate=max(0,1-max(T.Tf-endstate.s,0)/C.terminal_window_s);
err=row(qtarget)-row(endstate.q); vel=row(endstate.dq);
acc=expand_local(C.caps.qdd_cmd_max_actual,numel(err));
toward=max(vel.*sign(err),0);
stopdist=toward.^2./max(2*acc,eps);
shortage=max(stopdist-abs(err),0);
terminal=terminal_gate*mean((err/C.tracking_q_scale_rad).^2+ ...
    (vel/C.tracking_dq_scale_rad_s).^2+ ...
    (shortage/C.tracking_q_scale_rad).^2);
terms=struct('power',D.Pavg_excess_sq_normalized_integral_s/max(h,eps), ...
    'progress',progress,'tracking',D.tracking_integral_s/max(h,eps), ...
    'smooth',D.smooth_integral_s/max(h,eps), ...
    'reserve',D.reserve_integral_s/max(h,eps),'terminal',terminal);
W=C.weights;
D.score=double(C.power_enabled)*W.power*terms.power-W.progress*terms.progress+ ...
    W.tracking*terms.tracking+W.smooth*terms.smooth+ ...
    W.reserve*terms.reserve+W.terminal*terms.terminal;
D.score_terms=terms; D.physical_path_advance=physical_progress;
D.terminal_stop_penalty_is_certificate=false;
end

function [next,I]=step_local(robot,T,S,command,C)
validate_state_local(S,T,C); q=row(S.q); dq=row(S.dq); n=numel(q);
[qr,qs,qss]=T.eval(S.s); qr=row(qr); qs=row(qs); qss=row(qss);
[ap,~,~,adot,G]=spinn3d_alpha_governor_step(command,S.a_prev, ...
    C.beta,C.up,C.dn,C.Ts,qs,qss,C.caps,C.alpha_max);
dqr=ap*qs; ddqr=ap*ap*qss+adot*qs;
% Endpoint derivatives are contractually zero. CTC feedback remains active
% at s==Tf: neither a completed path clock nor a nonzero Alpha proves stop.
v=ddqr+expand_local(C.Kd,n).*(dqr-dq)+expand_local(C.Kp,n).*(qr-q);
M=massMatrix(robot,q); c=row(velocityProduct(robot,q,dq));
g=row(gravityTorque(robot,q));
f=expand_local(C.fric.B,n).*dq+expand_local(C.fric.Fc,n).* ...
    tanh(dq/max(C.fric.vel_eps,1e-9));
tau=(M*v(:)).'+c+g+f;
ddq=(M\(tau(:)-c(:)-g(:)-f(:))).';
dqn=dq+C.Ts*ddq; qmid=q+.5*C.Ts*dq+.125*C.Ts^2*ddq;
qn=q+C.Ts*dq+.5*C.Ts^2*ddq;
[qlo_step,qhi_step]=position_extrema_local(q,dq,ddq,C.Ts);
bus=spinn3d_bus_with_state(C.bus,S.bus,C.Ts);
E=spinn3d_motor_interval_audit(tau,dq,dqn,bus);
flow=E.flow_mid; st=E.bus_state_next;
if isempty(st)
    [flow,st]=spinn3d_bus_step(tau,.5*(dq+dqn),bus,S.bus,C.Ts);
end
bavg=bus; bavg.Pgrid_avg_prev=S.Pavg;
[pavg,~,target,peak]=spinn3d_bus_avg_guard(flow.Pgrid,C.Pbud,bavg,C.Ts);
A=C.caps.execution_audit;

% ratio tolerance: the latter must never become planned physical headroom.
tol=double(C.caps.alpha_feasible.feasibility_tolerance);
qlo=expand_local(C.caps.q_min,n); qhi=expand_local(C.caps.q_max,n);
taumax=expand_local(C.caps.tau_max,n); vmax=expand_local(C.caps.qd_max,n);
amax=expand_local(C.caps.qdd_cmd_max_actual,n); paxmax=expand_local(C.caps.P_axis_max,n);
V=spinn3d_joint_limit_viability(qn,dqn,qlo,qhi,amax, ...
    A.joint_limit_viability,A.position_planning_tolerance_rad);
geometry=spinn3d_physical_geometry_check(robot,[q;qmid;qn],C.workspace);
power_tol=A.physical_power_abs_tolerance_W+ ...
    A.physical_power_rel_tolerance*max(1,abs(peak));
if ~isfinite(peak), power_tol=A.physical_power_abs_tolerance_W; end
dynamic_dc=strcmp(char(bus.model_scope),'dynamic_capacitor_common_dc_v1');
if dynamic_dc,pbrk=0;else,pbrk=bus.P_brk_peak;end
qdpeak=max(abs([dq;dqr;dqn]),[],1);
finite=all(isfinite([q,dq,ap,adot,tau,ddq,qn,dqn,pavg,flow.Pgrid,flow.Pspill]));
checks=[finite,ap>=0 && ap<=C.alpha_max+tol, ...
    all(abs(tau)<=taumax+tol), ...
    E.envelope_ok && all(E.torque_speed_ratio<=1+tol), ...
    all(qdpeak<=vmax+tol), ...
    all(abs(ddq)<=amax+tol), ...
    all(qlo_step>=qlo-A.joint_limit_viability.numeric_tolerance_rad) && ...
    all(qhi_step<=qhi+A.joint_limit_viability.numeric_tolerance_rad), ...
    V.pass,geometry.pass, ...
    all(E.Paxis_motoring_peak_W<=paxmax+tol), ...
    E.Pgrid_admission_upper_bound_W<=peak+power_tol, ...
    E.Pdump_peak_W<=pbrk+power_tol,flow.Pspill<=A.spill_power_tolerance_W, ...
    E.dynamic_state_pass];
names={'nonfinite','alpha_range','tau','motor_torque_speed','qd','qdd', ...
    'joint_position_step','joint_limit_braking_viability','physical_geometry', ...
    'axis_power','grid_peak','brake_peak','spill','dynamic_dc_state'};
if dynamic_dc,names{12}='no_brake_dissipation';end
if bus.enable_state
    if ~dynamic_dc
        checks=[checks,isfinite(st.Tbrk) && st.Tbrk<=bus.Tbrk_max_C+tol];
        names=[names,{'brake_thermal'}];
    end
    checks=[checks,isfinite(st.Ebuf) && st.Ebuf>=bus.Ebuf_min_J-tol && st.Ebuf<=bus.Ebuf_max_J+tol];
    names=[names,{'buffer_energy'}];
end
failed=find(~checks,1); reason='feasible';
if ~isempty(failed), reason=names{failed}; end
next=S; next.q=qn; next.dq=dqn; next.s=min(T.Tf,S.s+C.Ts*ap);
next.a_prev=ap; next.bus=st; next.Pavg=pavg;
I=struct('physical_ok',all(checks),'failure_reason',reason,'a_post',ap, ...
    'adot',adot,'governor',G,'dq_cmd',dqr,'ddq_cmd',ddqr,'ddq_actual',ddq, ...
    'tau_cmd',tau,'q_next',qn,'dq_next',dqn,'flow',flow,'st_next',st, ...
    'Pgrid_avg_next_W',pavg,'Pavg_target_W',target,'Pavg_excess_W',max(pavg-target,0), ...
    'electrical_interval',E,'joint_limit_viability',V,'geometry',geometry, ...
    'tau_ratio_peak',max(ratio_local(abs(tau),taumax)), ...
    'qdd_ratio_peak',max(ratio_local(abs(ddq),amax)), ...
    'qd_ratio_peak',max(ratio_local(qdpeak,vmax)), ...
    'motor_ratio_peak',max(E.torque_speed_ratio), ...
    'axis_power_ratio_peak',max(ratio_local(E.Paxis_motoring_peak_W,paxmax)), ...
    'grid_peak_ratio_peak',ratio_local(E.Pgrid_admission_upper_bound_W,peak));
I.tracking_cost=mean(((q-qr)/C.tracking_q_scale_rad).^2+ ...
    ((dq-dqr)/C.tracking_dq_scale_rad_s).^2);
I.smooth_cost=((ap-S.a_prev)/max(C.alpha_max,eps))^2;
I.reserve_cost=max(I.tau_ratio_peak-C.tau_reserve_ratio,0)^2+ ...
    max(I.qdd_ratio_peak-C.qdd_reserve_ratio,0)^2;

if ~I.physical_ok, next=S; end
end

function validate_state_local(S,T,C)

if strcmp(char(C.bus.model_scope),'dynamic_capacitor_common_dc_v1')

end
end

function [lo,hi]=position_extrema_local(q,dq,a,dt)
qn=q+dt*dq+.5*dt^2*a; lo=min(q,qn); hi=max(q,qn);
for j=1:numel(q)
    if a(j)==0, continue; end
    ts=-dq(j)/a(j);
    if ts>0 && ts<dt
        z=q(j)+ts*dq(j)+.5*ts^2*a(j); lo(j)=min(lo(j),z); hi(j)=max(hi(j),z);
    end
end
end
function r=ratio_local(x,m)
x=row(x); m=row(m); r=zeros(size(x)); finite=isfinite(m) & m>0;
r(finite)=x(finite)./m(finite); r(m==0 & x>0)=inf; r(~isfinite(x))=inf;
end
function v=expand_local(v,n)
v=row(v); if isscalar(v), v=repmat(v,1,n); end

end
function v=row(v)
v=double(v(:).');
end
