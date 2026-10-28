function [M,Meta]=spinn3d_manuscript_metrics(out,Pbud,opts)
%SPINN3D_MANUSCRIPT_METRICS Explicit paper KPIs on completed [0,t_reach].
% Log rows describe intervals (t(k-1),t(k)], with t(0)=0. Integrals include
% the first interval and exclude post-arrival settling/hold. Arrival must
% coincide with a logged boundary; capacitor states are never interpolated.

% p=Pavg/Pbud; excess integrates max(p-(1+rho),0), in seconds, whereas R_soft
% elsewhere remains a separate squared, time-averaged zero-margin metric.
% Equal-duration samples use inclusive/type-7 percentiles. Nonuniform
% intervals use the inverse duration-weighted empirical CDF (no smoothing).
% The ramp includes the initial reconstructed EMA boundary and uses each dt.
% Reached and accepted are distinct: acceptance selects pairs downstream;
% accepted=false does not erase an observed arrival and its physical KPIs.
if nargin<3 || isempty(opts),opts=struct();end

rho=get_local(opts,'manuscript_margin_frac',.05);

reanalysis_tau=get_local(opts,'filter_reanalysis_window_s',[]);
if ~isempty(reanalysis_tau)

end
names={'p_filtered_peak','p_filtered_p99','excess_norm_integral_s', ...
    'ramp_norm_p95_per_s','Egrid_to_reach_J', ...
    'Egrid_storage_adjusted_to_reach_J','Ecap_delta_to_reach_J', ...
    'Ereuse_to_reach_J','Psource_request_peak_W', ...
    'Isource_peak_A','Isource_interval_peak_A', ...
    'Vdc_min_to_reach_V','Vdc_max_to_reach_V'};
M=struct();availability=struct();reasons=struct();
for j=1:numel(names)
    M.(names{j})=NaN;availability.(names{j})=false;
    reasons.(names{j})='missing_or_invalid_evidence';
end
Meta=struct('definition_version','manuscript_to_reach_v1', ...
    'window_start_s',0,'window_end_s',NaN,'window_intervals',0, ...
    'window_boundary_valid',false,'Pbud_W',double(Pbud), ...
    'margin_frac',double(rho),'excess_threshold_W',double(Pbud)*(1+double(rho)), ...
    'reached',NaN,'accepted',NaN,'audit_pass',NaN, ...
    'filter_source','execution_hard_audit_pavg_replay_v2_normalized_square', ...
    'authoritative_filter_available',false,'filter_aggregate_consistency_pass',false, ...
    'controller_filter_telemetry_pass',NaN,'filter_window_s',NaN, ...
    'executed_filter_window_s',NaN,'filter_scope','executed_causal_ema', ...
    'original_authoritative_filter_available',false, ...
    'percentile_method','unavailable','ramp_initial_boundary_included',true, ...
    'request_definition','same_state_unclipped_source_droop_upstream_power_v1', ...
    'current_peak_definition','electrical_substep_maximum_upstream_current', ...
    'interval_current_peak_definition','maximum_completed_interval_average_upstream_current', ...
    'capacitor_closure_pass',false,'available',false,'complete',false, ...
    'reason','missing_or_invalid_output','metric_available',availability, ...
    'metric_reason',reasons);
if ~isstruct(out)||~isscalar(out),return;end
Meta.reached=binary_or_nan_local(get_local(out,'reached',NaN));
Meta.accepted=binary_or_nan_local(get_local(out,'accepted',NaN));
A=get_local(out,'execution_audit',struct());
Meta.audit_pass=binary_or_nan_local(get_local(A,'pass',NaN));
if Meta.reached~=1
    Meta=unavailable_local(Meta,names,'explicit_arrival_not_established');return;
end
tr=get_local(out,'t_reach',NaN);L=get_local(out,'log',struct());
if ~finite_scalar_local(tr)||tr<=0||~isstruct(L)||~isscalar(L)||~isfield(L,'t')
    Meta=unavailable_local(Meta,names,'invalid_or_zero_duration_arrival');return;
end
t=L.t;
if ~isnumeric(t)||~isreal(t)||~isvector(t)||isempty(t)||any(~isfinite(t(:)))
    Meta=unavailable_local(Meta,names,'invalid_interval_end_timebase');return;
end
t=double(t(:));dt=[t(1);diff(t)];N=numel(t);
if any(dt<=0)
    Meta=unavailable_local(Meta,names,'invalid_interval_end_timebase');return;
end
[distance,K]=min(abs(t-double(tr)));tol=max(1e-10,64*eps(max(1,t(end))));
if distance>tol
    Meta=unavailable_local(Meta,names,'arrival_not_a_completed_interval_boundary');return;
end
tw=t(1:K);dw=dt(1:K);
Meta.window_end_s=tw(end);Meta.window_intervals=K;Meta.window_boundary_valid=true;
if equal_duration_local(dw)
    Meta.percentile_method='inclusive_type7_equal_duration';
else
    Meta.percentile_method='duration_weighted_inverse_empirical_cdf';
end
cfg=get_local(out,'cfg',struct());bus=get_local(cfg,'BUS',struct());
tau=get_local(bus,'P_avg_window_s',NaN);
if finite_scalar_local(tau)&&tau>0
    Meta.filter_window_s=double(tau);Meta.executed_filter_window_s=double(tau);
end
[pavg,p0,filter_reason,aggregate_ok]=audited_filter_local(A,t);
Meta.filter_aggregate_consistency_pass=aggregate_ok;
Meta.controller_filter_telemetry_pass=binary_or_nan_local(get_local(A,'Pavg_telemetry_pass',NaN));
Meta.original_authoritative_filter_available=~isempty(pavg);
if ~isempty(reanalysis_tau)
    Meta.filter_scope='fixed_log_filter_reanalysis';
    Meta.filter_source='verified_source_and_independent_initial_ema_fixed_log_reanalysis_v1';
    Meta.filter_window_s=double(reanalysis_tau);
    raw=log_prefix_local(L,'Pgrid',N,N);
    replay_ok=~isempty(pavg)&&finite_scalar_local(p0)&&p0>=0&& ...
        finite_scalar_local(tau)&&tau>0&&~isempty(raw)&&all(raw>=0)&& ...
        binary_or_nan_local(get_local(A,'telemetry_pass',NaN))==1;
    if replay_ok
        original=causal_ema_local(raw,dt,p0,double(tau));
        replay_ok=all(abs(original-pavg)<=1e-8+1e-8*max(1,max(abs(pavg))));
    end
    if replay_ok
        pavg=causal_ema_local(raw,dt,p0,double(reanalysis_tau));
        filter_reason='available_fixed_log_reanalysis_not_executed_feedback';
    else
        pavg=[];filter_reason='fixed_log_reanalysis_requires_verified_executed_source_and_initial_ema';
    end
end
filter_names=names(1:4);
if ~isempty(pavg)
    Meta.authoritative_filter_available=isempty(reanalysis_tau);p=pavg(1:K)/double(Pbud);
    [M,Meta]=put_local(M,Meta,filter_names{1},max(p),filter_reason);
    [M,Meta]=put_local(M,Meta,filter_names{2},percentile_local(p,dw,99),filter_reason);
    [M,Meta]=put_local(M,Meta,filter_names{3},sum(dw.*max(p-(1+double(rho)),0)),filter_reason);
    if finite_scalar_local(p0)&&p0>=0
        ramps=abs(diff([double(p0)/double(Pbud);p])./dw);
        [M,Meta]=put_local(M,Meta,filter_names{4},percentile_local(ramps,dw,95),filter_reason);
    else
        Meta.metric_reason.(filter_names{4})='missing_independent_initial_ema_boundary';
    end
else
    for j=1:numel(filter_names),Meta.metric_reason.(filter_names{j})=filter_reason;end
end
energy_inputs={'Pgrid','Preuse_total'};
energy_outputs={'Egrid_to_reach_J','Ereuse_to_reach_J'};
for j=1:numel(energy_inputs)
    x=log_prefix_local(L,energy_inputs{j},N,K);
    if ~isempty(x)&&all(x>=0)
        [M,Meta]=put_local(M,Meta,energy_outputs{j},sum(dw.*x),'available');
    else
        Meta.metric_reason.(energy_outputs{j})=['missing_or_invalid_log_' energy_inputs{j}];
    end
end
% The logged incremental and boundary capacitor energies must telescope.
ecap=log_prefix_local(L,'Ebuf',N,K);
dE=log_prefix_local(L,'Ecap_delta_J',N,K);
e0=initial_local(L,'Ecap_pre_t0_J',N);
if ~isempty(ecap)&&~isempty(dE)&&finite_scalar_local(e0)&&e0>=0&&all(ecap>=0)
    closure=max(abs(double(e0)+cumsum(dE)-ecap));
    Meta.capacitor_closure_pass=closure<=1e-8;
    if Meta.capacitor_closure_pass
        delta=ecap(end)-double(e0);
        [M,Meta]=put_local(M,Meta,'Ecap_delta_to_reach_J',delta,'available');
        if isfinite(M.Egrid_to_reach_J)
            [M,Meta]=put_local(M,Meta,'Egrid_storage_adjusted_to_reach_J', ...
                M.Egrid_to_reach_J-delta,'available');
        end
    else
        Meta.metric_reason.Ecap_delta_to_reach_J='capacitor_prefix_energy_closure_failed';
        Meta.metric_reason.Egrid_storage_adjusted_to_reach_J='capacitor_prefix_energy_closure_failed';
    end
end
peak_inputs={'Psource_request_peak_W','Isource_peak_A','Isource'};
peak_outputs={'Psource_request_peak_W','Isource_peak_A','Isource_interval_peak_A'};
for j=1:numel(peak_inputs)
    x=log_prefix_local(L,peak_inputs{j},N,K);
    if j<3
        [verified,why]=source_diagnostic_local(A,L,peak_inputs{j},N,K);
        if ~verified,Meta.metric_reason.(peak_outputs{j})=why;continue;end
    end
    if ~isempty(x)&&all(x>=0)
        [M,Meta]=put_local(M,Meta,peak_outputs{j},max(x),'available');
    else
        Meta.metric_reason.(peak_outputs{j})=['missing_or_invalid_log_' peak_inputs{j}];
    end
end
vmin=log_prefix_local(L,'Vdc_min',N,K);vmax=log_prefix_local(L,'Vdc_max',N,K);
v0=initial_local(L,'Vdc_pre_t0_V',N);
if ~isempty(vmin)&&~isempty(vmax)&&finite_scalar_local(v0)&&v0>0&& ...
        all(vmin>0)&&all(vmax>=vmin)
    [M,Meta]=put_local(M,Meta,'Vdc_min_to_reach_V',min([v0;vmin]),'available');
    [M,Meta]=put_local(M,Meta,'Vdc_max_to_reach_V',max([v0;vmax]),'available');
end
present=struct2cell(Meta.metric_available);
Meta.available=any([present{:}]);Meta.complete=all([present{:}]);
if Meta.complete,Meta.reason='complete';else,Meta.reason='partial_metrics_see_metric_reason';end
end

function y=causal_ema_local(raw,dt,p0,tau)
y=zeros(size(raw));prev=double(p0);
for k=1:numel(raw)
    beta=exp(-dt(k)/tau);prev=beta*prev+(1-beta)*raw(k);y(k)=prev;
end
end

function [ok,reason]=source_diagnostic_local(A,L,name,N,K)
ok=false;reason='missing_independent_source_diagnostic_replay';
D=get_local(A,'dynamic_dc',struct());
schema=get_local(D,'source_diagnostic_schema','');
if ~(ischar(schema)||(isstring(schema)&&isscalar(schema)))|| ...
        ~strcmp(char(schema),'same_state_unclipped_source_droop_v1')|| ...
        binary_or_nan_local(get_local(D,'source_diagnostic_available',NaN))~=1
    return;
end
if binary_or_nan_local(get_local(D,'source_diagnostic_pass',NaN))~=1
    reason='source_diagnostic_replay_failed';return;
end
x=log_prefix_local(L,name,N,K);
y=log_prefix_local(D,[name '_replay'],N,K);
if isempty(x)||isempty(y)||any(x<0)||any(y<0),return;end
if any(abs(x-y)>1e-8+1e-8*max(1,max(abs(y))))
    reason='source_diagnostic_log_replay_mismatch';return;
end
ok=true;reason='available';
end

function [trace,p0,reason,aggregate_ok]=audited_filter_local(A,t)
trace=[];p0=NaN;reason='missing_authoritative_execution_audit_ema';aggregate_ok=false;
schema='execution_hard_audit_pavg_replay_v2_normalized_square';
if ~isstruct(A)||~isscalar(A),return;end
flags={'Pavg_hard_enforced','Pgrid_avg_hard_enforced','filtered_power_hard_enforced', ...
    'Pavg_replay_available','Pavg_replay_authoritative'};
desired=[0 0 0 1 1];
for j=1:numel(flags)
    if binary_or_nan_local(get_local(A,flags{j},NaN))~=desired(j),return;end
end
label=get_local(A,'Pavg_replay_schema','');
if ~(ischar(label)||(isstring(label)&&isscalar(label)))||~strcmp(char(label),schema),return;end
x=get_local(A,'Pgrid_avg_replay_W',[]);
if ~isnumeric(x)||~isreal(x)||~isvector(x)||numel(x)~=numel(t)|| ...
        any(~isfinite(x(:)))||any(x(:)<0),return;end
x=double(x(:));native=get_local(A,'Pgrid_avg_budget_W',NaN);
if ~finite_scalar_local(native)||native<0,return;end
dt=[t(1);diff(t)];e=max(x-double(native),0);q=(e/max(double(native),eps)).^2;
expected=[max(x),sum(dt.*double(x>native)),sum(dt.*double(x>native))/sum(dt), ...
    sum(dt.*e),sum(dt.*q),sum(dt.*q)/sum(dt)];
keys={'Pgrid_avg_peak_W','Pgrid_avg_excess_duration_s','Pgrid_avg_excess_time_fraction', ...
    'Pgrid_avg_excess_integral_J','Pgrid_avg_excess_sq_norm_integral_s','Pgrid_avg_excess_sq_norm_mean'};
actual=nan(size(expected));
for j=1:numel(keys)
    a=get_local(A,keys{j},NaN);
    if ~finite_scalar_local(a)||a<0,reason='missing_native_ema_aggregate_evidence';return;end
    actual(j)=double(a);
end
aggregate_ok=all(abs(actual-expected)<=1e-9+1e-8*max(1,max(abs(expected))));
if ~aggregate_ok,reason='authoritative_ema_aggregate_mismatch';return;end
trace=x;reason='available';
origin=get_local(A,'Pavg_replay_initial_source','');
if (ischar(origin)||(isstring(origin)&&isscalar(origin)))&& ...
        strcmp(char(origin),'independent_static_hold_reconstruction_v1')
    initial=get_local(A,'Pgrid_avg_initial_replay_W',NaN);
    if finite_scalar_local(initial)&&initial>=0,p0=double(initial);end
end
end

function q=percentile_local(x,dt,pct)
if equal_duration_local(dt)
    q=spinn3d_percentile_inclusive(x,pct);
else
    [xs,order]=sort(x);cdf=cumsum(dt(order))/sum(dt);
    k=find(cdf>=pct/100,1,'first');
    if isempty(k),k=numel(xs);end
    q=xs(k);
end
end
function tf=equal_duration_local(dt)
tf=max(abs(dt-dt(1)))<=max(64*eps(max(1,max(dt))),1e-10*max(dt));
end
function x=log_prefix_local(L,name,N,K)
x=[];
if ~isfield(L,name),return;end
y=L.(name);
if ~isnumeric(y)||~isreal(y)||~isvector(y)||numel(y)~=N,return;end
y=double(y(:));y=y(1:K);
if any(~isfinite(y)),return;end
x=y;
end
function x=initial_local(L,name,N)
x=NaN;if ~isfield(L,name),return;end
y=L.(name);
if ~isnumeric(y)||~isreal(y)||~isvector(y)||~any(numel(y)==[1 N])|| ...
        any(~isfinite(y(:))),return;end
y=double(y(:));
if any(abs(y-y(1))>1e-10*max(1,abs(y(1)))),return;end
x=y(1);
end
function [M,Meta]=put_local(M,Meta,name,value,reason)
M.(name)=double(value);Meta.metric_available.(name)=isfinite(value);
Meta.metric_reason.(name)=reason;
end
function Meta=unavailable_local(Meta,names,reason)
Meta.reason=reason;
for j=1:numel(names),Meta.metric_reason.(names{j})=reason;end
end
function v=get_local(S,name,default)
v=default;
if isstruct(S)&&isscalar(S)&&isfield(S,name)&&~isempty(S.(name)),v=S.(name);end
end
function tf=finite_scalar_local(x)
tf=isnumeric(x)&&isreal(x)&&isscalar(x)&&isfinite(x);
end
function x=binary_or_nan_local(x)
if ~(islogical(x)||isnumeric(x))||~isreal(x)||~isscalar(x)|| ...
        ~isfinite(double(x))||~any(double(x)==[0 1])
    x=NaN;
else
    x=double(x);
end
end
