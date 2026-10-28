function [T, log] = spinn3d_actual_guard_telemetry(info, log, k, step_audit)
% Collect applied-rate and physical-admission quantities along a trajectory.

if nargin<1, info=struct(); end
if nargin<4, step_audit=struct(); end
T=extract_local(info);
T.actuator_saturation=binary_local(field_local( ...
    step_audit,'actuator_saturation_applied',NaN));
T.motion_projection=binary_local(field_local( ...
    step_audit,'motion_projection_applied',NaN));
if nargin>=4
    T.applied=or_known_local([T.intervened, ...
        T.actuator_saturation,T.motion_projection]);
else
    % Extraction-only calls report controller intervention; no plant claim.
    T.applied=T.intervened;
end
if nargout<2, return; end

N=numel(log.t);
names=fieldnames(T);
for ii=1:numel(names)
    name=names{ii};
    if any(strcmp(name,{'schema','actuator_saturation','motion_projection'}))
        continue;
    end
    if any(strcmp(name,{'alpha_policy_target','alpha_pre_actual'}))
        dest=name;
    else
        dest=['actual_guard_',name];
    end
    value=T.(name);
    if ischar(value)
        if ~isfield(log,dest), log.(dest)=repmat({'unavailable'},N,1); end
        log.(dest){k,1}=value;
    elseif isnumeric(value) || islogical(value)
        if ~isfield(log,dest)
            log.(dest)=nan(N,numel(value));
        elseif islogical(log.(dest))
            % Preserve NaN rather than coercing unavailable into logical true.
            log.(dest)=double(log.(dest));
        end
        log.(dest)(k,:)=double(value(:).');
    end
end
for name={'actuator_saturation','motion_projection'}
    dest=name{1};
    if ~isfield(log,dest), log.(dest)=nan(N,1); end
    if islogical(log.(dest)), log.(dest)=double(log.(dest)); end
    log.(dest)(k)=T.(dest);
end
% These predictions are held between policy ticks. Sum evaluation counts
% only on nominal_policy_tick rows, not on all 1-ms copies of one search.
% This is a selector-executed flag: a policy clock tick during latched
% recovery can bypass selection and must not recount the old preview.
P=nominal_local(info);
names=fieldnames(P);
for ii=1:numel(names)
    f=names{ii}; dest=['nominal_policy_',f]; value=P.(f);
    if ischar(value)
        if ~isfield(log,dest), log.(dest)=repmat({'unavailable'},N,1); end
        log.(dest){k,1}=value;
    else
        if ~isfield(log,dest), log.(dest)=nan(N,1); end
        if islogical(log.(dest)), log.(dest)=double(log.(dest)); end
        log.(dest)(k)=double(value);
    end
end
end

function V=nominal_local(info)
% Explicit producer schemas prevent a full-horizon endpoint or duration from
% masquerading as the first held policy interval. Each availability flag
% describes only its named evidence; an unrecognized nonempty struct is not
% an available selector result. Evaluation counts include rejected previews.
flags={'physical_ok','recursive_viable','fallback_requested'};
counts={'candidate_evaluations','successor_evaluations', ...
    'current_plant_steps_checked','successor_plant_steps_checked', ...
    'selected_plant_steps_checked','horizon_plant_steps_checked'};
scalars={'selected_command','selected_end_alpha','selected_progress', ...
    'selected_duration_s','successor_command','successor_end_alpha', ...
    'successor_progress','successor_duration_s','progress_upper_bound', ...
    'desired_dt_s','applied_dt_s','horizon_end_alpha','horizon_progress', ...
    'horizon_duration_s','horizon_requested_dt_s'};
V=struct('schema','unavailable','available',false, ...
    'selection_available',false,'cadence_available',false, ...
    'work_available',false,'first_interval_available',false, ...
    'successor_available',false,'horizon_available',false, ...
    'tick',binary_local(field_local(info,'noguard_policy_tick',NaN)));
for f=[flags,counts,scalars], V.(f{1})=NaN; end
for f={'selection_mode','failure_reason','successor_failure_reason'}
    V.(f{1})='unavailable';
end
P=field_local(info,'noguard_policy',struct());
if ~(isstruct(P) && isscalar(P)), return; end
schema=text_local(field_local(P,'diagnostic_schema','unavailable'));
legacy_schema=text_local(field_local(P,'schema','unavailable'));
measured=strcmp(schema,'measured_ctc_selector_diagnostics_v1');
legacy=strcmp(schema,'unavailable') && ...
    strcmp(legacy_schema,'noguard_full_policy_nominal_progress_v1');
% Before the original selector had a schema, these fields identified its

unversioned=~isfield(P,'diagnostic_schema') && ~isfield(P,'schema') && ...
    all(isfield(P,{'physical_ok','recursive_viable','fallback_requested', ...
        'candidate_evaluations','selection_mode'}));
if ~(measured || legacy || unversioned), return; end
if measured
    V.schema=schema;
    Q=P;
    Q.selected_command=field_local(P,'a_des_command',NaN);
    Q.successor_evaluations=field_local(P,'successor_candidate_evaluations',NaN);
    % A rejected selector still returns a finite hold request, but its
    % representative rejected trial cannot predict that returned request.
    if isequal(binary_local(field_local(P,'selected_preview_available',NaN)),1)
        first=field_local(P,'selected_first_interval',struct());
        successor=field_local(P,'selected_first_successor',struct());
        horizon=field_local(P,'selected_horizon',struct());
    else
        first=struct(); successor=struct(); horizon=struct();
    end
    Q.selected_end_alpha=field_local(first,'end_alpha',NaN);
    Q.selected_progress=field_local(first,'progress',NaN);
    Q.selected_duration_s=field_local(first,'duration_s',NaN);
    Q.selected_plant_steps_checked=field_local(first,'plant_steps_checked',NaN);
    Q.successor_command=field_local(successor,'command',NaN);
    Q.successor_end_alpha=field_local(successor,'end_alpha',NaN);
    Q.successor_progress=field_local(successor,'progress',NaN);
    Q.successor_duration_s=field_local(successor,'duration_s',NaN);
    Q.horizon_end_alpha=field_local(horizon,'end_alpha',NaN);
    Q.horizon_progress=field_local(horizon,'progress',NaN);
    Q.horizon_duration_s=field_local(horizon,'duration_s',NaN);
    Q.horizon_plant_steps_checked=field_local(horizon,'plant_steps_checked',NaN);
else
    Q=P;
    if legacy, V.schema=legacy_schema;
    else, V.schema='legacy_nominal_selector_unversioned'; end
end
for f=flags, V.(f{1})=binary_local(field_local(Q,f{1},NaN)); end
for f=counts, V.(f{1})=count_local(field_local(Q,f{1},NaN)); end
for f=scalars, V.(f{1})=scalar_local(field_local(Q,f{1},NaN)); end
for f={'selection_mode','failure_reason','successor_failure_reason'}
    V.(f{1})=text_local(field_local(Q,f{1},'unavailable'));
end
% The runtime validates policy_dt against the teacher config. It is a valid
% cadence source when an older record omits the producer's desired_dt_s.
% Neither the accumulated rollout duration nor the current row spacing is a
% substitute for the plant period.
runtime_dt=scalar_local(field_local(info,'policy_dt',NaN));
if ~isfield(Q,'desired_dt_s') && runtime_dt>0
    V.desired_dt_s=runtime_dt;
elseif runtime_dt>0 && isfinite(V.desired_dt_s) && ...
        abs(runtime_dt-V.desired_dt_s)>64*eps(max(1,abs(runtime_dt)))
    V.desired_dt_s=NaN;
end
for f={'desired_dt_s','applied_dt_s','horizon_requested_dt_s'}
    if V.(f{1})<=0, V.(f{1})=NaN; end
end
% Every attempted preview checks at least one plant step. Unknown or
% inconsistent work must remain unknown, never become an invented zero.
V.current_plant_steps_checked=work_count_local( ...
    V.current_plant_steps_checked,V.candidate_evaluations);
V.successor_plant_steps_checked=work_count_local( ...
    V.successor_plant_steps_checked,V.successor_evaluations);
V.available=all(isfinite([V.physical_ok,V.recursive_viable, ...
    V.fallback_requested,V.candidate_evaluations]));
V.selection_available=V.available && isfinite(V.selected_command);
stride=V.desired_dt_s/V.applied_dt_s;
V.cadence_available=all(isfinite([V.desired_dt_s,V.applied_dt_s,stride])) && ...
    stride>=1 && abs(stride-round(stride))<1e-9;
V.work_available=all(isfinite([V.candidate_evaluations, ...
    V.successor_evaluations,V.current_plant_steps_checked, ...
    V.successor_plant_steps_checked]));
V.first_interval_available=V.selection_available && ...
    V.physical_ok==1 && V.recursive_viable==1 && V.fallback_requested==0 && ...
    all(isfinite([V.selected_end_alpha,V.selected_progress, ...
        V.selected_plant_steps_checked])) && V.selected_plant_steps_checked>=1;
V.successor_available=V.first_interval_available && ...
    all(isfinite([V.successor_command,V.successor_end_alpha,V.successor_progress]));
V.horizon_available=measured && V.first_interval_available && ...
    all(isfinite([V.horizon_end_alpha,V.horizon_progress,V.horizon_duration_s, ...
        V.horizon_plant_steps_checked,V.horizon_requested_dt_s])) && ...
    V.horizon_plant_steps_checked>=V.selected_plant_steps_checked && ...
    abs(V.horizon_duration_s-V.horizon_requested_dt_s)<1e-12;
if measured
    V.first_interval_available=V.first_interval_available && V.cadence_available && ...
        isequal(binary_local(field_local(first,'physical_ok',NaN)),1) && ...
        isequal(scalar_local(field_local(first,'command',NaN)),V.selected_command) && ...
        isfinite(V.selected_duration_s) && ...
        abs(V.selected_duration_s-V.desired_dt_s)<1e-12 && ...
        V.selected_plant_steps_checked==round(stride);
    V.successor_available=V.successor_available && V.first_interval_available && ...
        isequal(binary_local(field_local(successor,'physical_ok',NaN)),1) && ...
        isfinite(V.successor_duration_s) && ...
        abs(V.successor_duration_s-V.desired_dt_s)<1e-12;
    V.horizon_available=V.horizon_available && V.first_interval_available && ...
        isequal(binary_local(field_local(horizon,'physical_ok',NaN)),1) && ...
        isequal(scalar_local(field_local(horizon,'command',NaN)),V.selected_command) && ...
        abs(V.horizon_plant_steps_checked*V.applied_dt_s-V.horizon_duration_s)<1e-12;
end
end

function v=count_local(v)
v=scalar_local(v);
if ~(isfinite(v) && v>=0 && v==fix(v)), v=NaN; end
end

function n=work_count_local(n,evaluations)
if isfinite(n) && isfinite(evaluations) && ...
        (n<evaluations || (evaluations==0 && n~=0))
    n=NaN;
end
end

function T=extract_local(info)
T=struct('schema','actual_guard_liveness_v3', ...
    'alpha_policy_target',NaN,'alpha_pre_actual',NaN, ...
    'alpha_selected',NaN,'alpha_delta',NaN, ...
    'executed',NaN,'intervention_available',false, ...
    'alpha_changed',NaN,'alpha_lowered',NaN,'alpha_raised',NaN, ...
    'control_gain_changed',NaN,'intervened',NaN,'applied',NaN, ...
    'safe_hold',NaN,'floor_infeasible',NaN, ...
    'hold_ok',NaN,'below_requested_floor',NaN, ...
    'position_gain_scale',NaN,'damping_gain_scale',NaN, ...
    'q_error_norm',NaN,'dq_error_norm',NaN, ...
    'nominal_ok',NaN,'nominal_rejected',NaN, ...
    'nominal_violation',NaN,'nominal_reason','unavailable', ...
    'positive_position_gain_found',NaN, ...
    'positive_damping_gain_found',NaN, ...
    'zero_position_gain_selected',NaN, ...
    'zero_damping_gain_selected',NaN, ...
    'position_grid_evaluations',NaN,'position_refine_evaluations',NaN, ...
    'damping_grid_evaluations',NaN,'damping_refine_evaluations',NaN, ...
    'total_evaluations',NaN, ...
    'position_bracket_initial',[NaN NaN], ...
    'position_bracket_final',[NaN NaN], ...
    'damping_bracket_initial',[NaN NaN], ...
    'damping_bracket_final',[NaN NaN], ...
    'search_mode','unavailable','fallback_kind','unavailable', ...
    'trigger_reason','unavailable','hold_reason','unavailable');
if ~(isstruct(info) && isscalar(info)), return; end
T.alpha_policy_target=scalar_local(field_local(info,'a0',NaN));
T.alpha_selected=scalar_local(field_local(info,'a_max',NaN));
enabled=binary_local(field_local(info,'actual_guard_enabled',NaN));
G=field_local(info,'guard',struct());
T.alpha_pre_actual=scalar_local(field_local(G,'a_nom_pre_actual',NaN));
A=field_local(G,'actual',struct());
has_actual=isstruct(A) && isscalar(A) && ~isempty(fieldnames(A));
if ~has_actual
    if isequal(enabled,0)
        T.executed=0;
        T.alpha_changed=0; T.alpha_lowered=0; T.alpha_raised=0;
        T.control_gain_changed=0; T.safe_hold=0; T.intervened=0;
        T.intervention_available=true;
    end
    return;
end
T.executed=1;
T.safe_hold=binary_local(field_local(A,'freeze_on_infeasible',NaN));
T.floor_infeasible=binary_local(field_local(A,'floor_infeasible',NaN));
T.below_requested_floor=binary_local( ...
    field_local(A,'below_requested_floor',NaN));
T.hold_ok=binary_local(field_local(A,'hold_ok',NaN));
T.position_gain_scale=scalar_local(field_local(A,'position_gain_scale',NaN));
T.damping_gain_scale=scalar_local(field_local(A,'damping_gain_scale',NaN));
T.q_error_norm=scalar_local(field_local(A,'q_err_norm',NaN));
T.dq_error_norm=scalar_local(field_local(A,'dq_err_norm',NaN));
if all(isfinite([T.alpha_pre_actual,T.alpha_selected]))
    T.alpha_delta=T.alpha_selected-T.alpha_pre_actual;
    tol=1e-10;
    T.alpha_changed=double(abs(T.alpha_delta)>tol);
    T.alpha_lowered=double(T.alpha_delta < -tol);
    T.alpha_raised=double(T.alpha_delta > tol);
end
if all(isfinite([T.position_gain_scale,T.damping_gain_scale]))
    T.control_gain_changed=double(any(abs( ...
        [T.position_gain_scale,T.damping_gain_scale]-1)>1e-10));
end
T.intervened=or_known_local([T.alpha_changed, ...
    T.control_gain_changed,T.safe_hold]);
T.intervention_available=all(isfinite([T.alpha_changed, ...
    T.control_gain_changed,T.safe_hold]));
T.nominal_ok=binary_local(field_local(A,'nominal_ok',NaN));
if isfinite(T.nominal_ok), T.nominal_rejected=1-T.nominal_ok; end
T.nominal_violation=scalar_local(field_local(A,'nominal_violation',NaN));
T.nominal_reason=text_local(field_local(A,'nominal_reason','unavailable'));
T.search_mode=text_local(field_local(A,'actual_search_mode','unavailable'));
T.fallback_kind=text_local(field_local(A,'fallback_kind','none'));
% The safe-hold trigger is separate from a rejected nominal candidate.
T.trigger_reason=text_local(field_local(A,'trigger_reason','none'));
T.hold_reason=text_local(field_local(A,'hold_reason','none'));
flags={'positive_position_gain_found','positive_damping_gain_found', ...
    'zero_position_gain_selected','zero_damping_gain_selected'};
counts={'position_grid_evaluations','position_refine_evaluations', ...
    'damping_grid_evaluations','damping_refine_evaluations','total_evaluations'};
brackets={'position_bracket_initial','position_bracket_final', ...
    'damping_bracket_initial','damping_bracket_final'};
for ii=1:numel(flags)
    f=flags{ii};
    T.(f)=binary_local(field_local(A,['safe_hold_',f],NaN));
end
for ii=1:numel(counts)
    f=counts{ii};
    v=scalar_local(field_local(A,['safe_hold_',f],NaN));
    if v>=0 && v==round(v), T.(f)=v; end
end
for ii=1:numel(brackets)
    f=brackets{ii}; v=field_local(A,['safe_hold_',f],[NaN NaN]);
    if isnumeric(v) && isreal(v) && numel(v)==2
        T.(f)=double(v(:).');
    end
end
end

function value=field_local(S,name,default)
value=default;
if isstruct(S) && isscalar(S) && isfield(S,name) && ~isempty(S.(name))
    value=S.(name);
end
end

function value=scalar_local(value)
if (isnumeric(value) || islogical(value)) && isreal(value) && ...
        isscalar(value) && isfinite(double(value))
    value=double(value);
else
    value=NaN;
end
end

function value=binary_local(value)
value=scalar_local(value);
if ~any(value==[0 1]), value=NaN; end
end

function value=or_known_local(values)
if any(values==1)
    value=1;
elseif all(values==0)
    value=0;
else
    value=NaN;
end
end

function value=text_local(value)
if isstring(value) && isscalar(value), value=char(value); end
if ~ischar(value) || size(value,1)>1 || isempty(strtrim(value))
    value='unavailable';
else
    value=strtrim(value);
end
end
