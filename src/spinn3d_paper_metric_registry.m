function R=spinn3d_paper_metric_registry(T,prefixes)

% Define manuscript metrics and select the required table columns.

% A 1e-9 native-unit denominator floor is numerical, not a physical tolerance.
R=struct('name',{},'unit',{},'scope',{},'definition',{},'direction',{}, ...
    'allow_percent',{},'percent_denominator_min',{},'group',{}, ...
    'required_for_primary_mask',{});
R(end+1)=row_local('t_reach','s','accepted_task_to_reach','First accepted reach time','lower',true,'legacy',true);
R(end+1)=row_local('R_soft','1','legacy_export_window','Time mean of squared normalized positive budget excess','lower',true,'legacy',true);
R(end+1)=row_local('Pgrid_avg_peak','W','legacy_export_window','Peak causal filtered source power in existing export window','lower',true,'legacy',true);
R(end+1)=row_local('Pgrid_peak','W','legacy_export_window','Peak logged interval source power in existing export window','lower',true,'legacy',true);
R(end+1)=row_local('Egrid_total','J','legacy_export_window','Source energy in existing export window','lower',true,'legacy',true);
R(end+1)=row_local('Egrid_storage_adjusted_J','J','legacy_export_window','Source energy minus capacitor energy change','lower',true,'dynamic',true);
R(end+1)=row_local('Ecap_delta_J','J','legacy_export_window','Signed final minus initial capacitor energy','descriptive',false,'dynamic',true);
R(end+1)=row_local('p_filtered_peak','1','start_to_reach','Peak causal filtered source power divided by task budget','lower',true,'manuscript',true);
R(end+1)=row_local('p_filtered_p99','1','start_to_reach','Time-weighted p99 of causal filtered source power divided by task budget','lower',true,'manuscript',true);
R(end+1)=row_local('excess_norm_integral_s','s','start_to_reach','Integral max(filtered source power / budget - (1 + report tolerance),0)','lower',true,'manuscript',true);
R(end+1)=row_local('ramp_norm_p95_per_s','s^-1','start_to_reach','Time-weighted p95 of absolute causal normalized-power interval slope','lower',true,'manuscript',true);
R(end+1)=row_local('Egrid_to_reach_J','J','start_to_reach','Integral of upstream source power through reach time','lower',true,'manuscript',true);
R(end+1)=row_local('Egrid_storage_adjusted_to_reach_J','J','start_to_reach','Source energy through reach minus capacitor energy change through reach','lower',true,'manuscript',true);
R(end+1)=row_local('Ecap_delta_to_reach_J','J','start_to_reach','Signed capacitor energy at reach minus initial capacitor energy','descriptive',false,'manuscript',true);
R(end+1)=row_local('Ereuse_to_reach_J','J','start_to_reach','Reused energy through reach; no monotone benefit is assumed','descriptive',false,'manuscript',true);
R(end+1)=row_local('Psource_request_peak_W','W','start_to_reach','Peak same-state unclipped source-droop request diagnostic; not delivered source power','lower',true,'manuscript',false);
R(end+1)=row_local('Isource_peak_A','A','start_to_reach','Maximum actual source current sampled at electrical solver substeps','lower',true,'manuscript',false);
R(end+1)=row_local('Isource_interval_peak_A','A','start_to_reach','Maximum actual source current averaged over a logged plant interval','lower',true,'manuscript',false);
R(end+1)=row_local('Vdc_min_to_reach_V','V','start_to_reach','Minimum recorded DC-node voltage through reach; report signed absolute change','higher',false,'manuscript',true);
R(end+1)=row_local('Vdc_max_to_reach_V','V','start_to_reach','Maximum recorded DC-node voltage through reach; interpret against physical voltage limits','descriptive',false,'manuscript',true);
if nargin==0,return;end

prefixes=cellstr(string(prefixes));vars=T.Properties.VariableNames;
present=false(size(R));
for k=1:numel(R),present(k)=any(endsWith(vars,['_',R(k).name]));end
hasManuscript=any(present(strcmp({R.group},'manuscript')));
hasDynamic=any(present(strcmp({R.group},'dynamic')));
selected=strcmp({R.group},'legacy') | (hasDynamic&strcmp({R.group},'dynamic')) | ...
    (hasManuscript&strcmp({R.group},'manuscript'));
R=R(selected);
for j=1:numel(prefixes)
    for k=1:numel(R)
        f=[prefixes{j},'_',R(k).name];
        if strcmp(R(k).group,'manuscript'),id='spinn3d:MissingManuscriptMetric';
        elseif strcmp(R(k).group,'dynamic'),id='spinn3d:MissingDynamicFinalMetric';
        else,id='spinn3d:MissingFinalMetric';end

        
    end
end
end

function M=row_local(name,unit,scope,definition,direction,allow_percent,group,required)
M=struct('name',name,'unit',unit,'scope',scope,'definition',definition, ...
    'direction',direction,'allow_percent',logical(allow_percent), ...
    'percent_denominator_min',1e-9,'group',group,'required_for_primary_mask',logical(required));
end
