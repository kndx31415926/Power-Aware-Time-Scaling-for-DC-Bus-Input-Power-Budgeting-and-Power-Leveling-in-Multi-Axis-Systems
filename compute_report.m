function Report=compute_report(packageRoot)
%COMPUTE_REPORT Summarize the paired simulations for Tables 2-10 and Figures 2-9.
% Results are grouped by task, payload interval and perturbation condition.
% Main and ablation means use accepted arrivals. Perturbation means use
% physical completion, including late arrivals. Counts use all planned tasks.
if nargin<1,packageRoot=fileparts(mfilename('fullpath'));end
addpath(fullfile(packageRoot,'plots'));
resultDir=fullfile(packageRoot,'results');
reportDir=fullfile(resultDir,'report');
figureDir=fullfile(reportDir,'figures');
if ~isfolder(reportDir),mkdir(reportDir);end
if ~isfolder(figureDir),mkdir(figureDir);end
metadata=readtable(fullfile(packageRoot,'inputs','final_test_manifest.csv'),'TextType','string');
draws=load(fullfile(packageRoot,'inputs','bootstrap_indices.mat'));
protocol=jsondecode(fileread(fullfile(packageRoot,'inputs','protocol.json')));
[workloadTable,computationalWorkload]=reviewer_computational_workload(packageRoot,reportDir);
mainAll=read_study(resultDir,'main',metadata);
mainMethods={'Fixed-alpha1','RG-0','RG-P','Transformer-PR','MLP-PR','PTP','MPC-RG'};
ablationMethods={'RG-0','RG-P','Transformer-PR','MLP-PR','Transformer-SUP','MLP-SUP'};
core={'RG-0','RG-P','Transformer-PR','MLP-PR'};
main=mainAll(ismember(mainAll.method,mainMethods),:);
ablation=mainAll(ismember(mainAll.method,ablationMethods),:);
robust=read_study(resultDir,'robustness',metadata);
parameter=read_study(resultDir,'parameter',metadata);

% Paired effects resample whole task pairs with the published 2000 draws.
mainPairs={};
for a={'Transformer-PR','MLP-PR'}
    for b={'RG-0','RG-P','Fixed-alpha1','MPC-RG','PTP'}
        mainPairs{end+1}=paired(main,a{1},b{1},'accepted',draws); %#ok<AGROW>
    end
end
mainPairs=[mainPairs{:}];ablationPairs={};
pairMethods={'Transformer-PR','Transformer-SUP';'MLP-PR','MLP-SUP';...
    'Transformer-PR','RG-P';'MLP-PR','RG-P';'Transformer-PR','MLP-PR'};
for i=1:size(pairMethods,1)
    ablationPairs{end+1}=paired(ablation,pairMethods{i,1},pairMethods{i,2},'accepted',draws); %#ok<AGROW>
end
ablationPairs=[ablationPairs{:}];
Report=struct('main_rows',table2struct(main),'ablation_rows',table2struct(ablation),...
    'robust_rows',table2struct(robust),'parameter_rows',table2struct(parameter),...
    'main_paired',mainPairs,'ablation_paired',ablationPairs,...
    'computational_workload',computationalWorkload);
write_json(fullfile(reportDir,'analysis_data.json'),Report);

% Tables 2-10 describe the study settings and numerical results.
design=design_table();
writetable(design,fullfile(reportDir,'table_02_study_settings.csv'));
mainTable=summary_table(main,mainMethods,'accepted');
writetable(mainTable,fullfile(reportDir,'table_03_main_results.csv'));
mainEffects=effect_table(mainPairs);
writetable(mainEffects,fullfile(reportDir,'table_04_paired_effects.csv'));
payload=payload_data(main,draws);
write_json(fullfile(reportDir,'payload_tables.json'),payload);
payloadTable=payload_table(payload);
writetable(payloadTable,fullfile(reportDir,'table_05_payload_results.csv'));
ablationEffects=effect_table(ablationPairs(1:4));
writetable(ablationEffects,fullfile(reportDir,'table_06_ablation.csv'));
[robustCounts,robustEffects]=condition_tables(robust,protocol.robustness_cases,core,draws);
writetable(robustCounts,fullfile(reportDir,'table_07_robustness_completion.csv'));
writetable(robustEffects,fullfile(reportDir,'table_08_robustness_effects.csv'));
parameterTable=parameter_table(parameter,protocol.parameter_cases,core);
writetable(parameterTable,fullfile(reportDir,'table_09_parameter_sensitivity.csv'));
filter=filter_data(resultDir);
write_json(fullfile(reportDir,'filter_metric_analysis.json'),filter);

% Extract the common-task histories, then draw Figures 2-9.
representative_motion_data(packageRoot,reportDir);
% The same plotting functions produce vector PDF, PNG and editable FIG.
make_representative_figure(reportDir,figureDir);
make_payload_figures(reportDir,figureDir);
make_supplement_figures(reportDir,figureDir);
save(fullfile(reportDir,'report.mat'),'Report','mainTable','mainEffects',...
    'payloadTable','ablationEffects','robustCounts','robustEffects','parameterTable','filter',...
    'workloadTable','computationalWorkload');
fprintf('Tables 2-10 and Figures 2-9 written to %s\n',reportDir);
end

function T=read_study(resultDir,study,metadata)
Q=load(fullfile(resultDir,study,'trials.mat'),'T'); X=Q.T;
T=table(double(X.task_id),string(X.method),double(X.case_id),...
    double(X.physical_success),double(X.accepted),double(X.exception),...
    double(X.R_soft),double(X.t_reach),double(X.Egrid_storage_adjusted_to_reach_J),...
    'VariableNames',{'task_id','method','case_id','physical_success','accepted',...
    'exception','R_soft','T_s','Eadj_J'});
for i=1:height(T),T.method(i)=method_name(T.method(i));end
T.study=repmat(string(study),height(T),1);
T.mass_kg=nan(height(T),1);T.payload_stratum=zeros(height(T),1);
T.template_name=strings(height(T),1);
if ismember('payload_mass_kg',X.Properties.VariableNames),T.mass_kg=double(X.payload_mass_kg);end
if strcmp(study,'main')
    for i=1:height(T)
        j=find(double(metadata.task_id)==T.task_id(i),1);
        if ~isempty(j)
            T.mass_kg(i)=double(metadata.payload_mass_kg(j));
            T.payload_stratum(i)=double(metadata.payload_group(j));
            T.template_name(i)=string(metadata.template_name(j));
        end
    end
end
optional={'reached','hard_pass','time_contract_available','time_contract_pass',...
    'Egrid_to_reach_J','Ereuse_to_reach_J','p_filtered_peak','p_filtered_p99',...
    'excess_norm_integral_s','ramp_norm_p95_per_s','Psource_request_peak_W',...
    'Isource_peak_A','Vdc_min_to_reach_V','Vdc_max_to_reach_V','error_id'};
for k=1:numel(optional)
    f=optional{k};if ismember(f,X.Properties.VariableNames),T.(f)=X.(f);end
end
T=sortrows(T,{'case_id','task_id','method'});
end

function label=method_name(label)
switch char(label)
    case 'NoGuard',label="RG-0";
    case 'Feasible-RG',label="RG-P";
    case {'TRF-PRG','Alpha-PRG'},label="Transformer-PR";
    case 'MLP-PRG',label="MLP-PR";
    case 'Baseline',label="Fixed-alpha1";
    case 'Nominal-RG-grid',label="MPC-RG";
    case 'Bus-PTP-beam',label="PTP";
    case 'TRF-supervised',label="Transformer-SUP";
    case 'MLP-supervised',label="MLP-SUP";
end
label=string(label);
end

function P=paired(T,method,reference,criterion,draws)
A=T(T.method==string(method)&T.(criterion)==1,:);
B=T(T.method==string(reference)&T.(criterion)==1,:);
[ids,ia,ib]=intersect(A.task_id,B.task_id);
P=struct('method',method,'reference',reference,'n',numel(ids),...
    'task_ids',ids(:)','criterion',criterion,'metrics',struct());
for metric={'R_soft','T_s','Eadj_J'}
    f=metric{1};a=A.(f)(ia);b=B.(f)(ib);n=numel(a);
    V=struct('method_mean',NaN,'reference_mean',NaN,'difference',NaN,...
        'difference_CI95',[NaN NaN],'reduction_pct',NaN,'reduction_pct_CI95',[NaN NaN]);
    if n>0&&all(isfinite(a))&&all(isfinite(b))
        V.method_mean=mean(a);V.reference_mean=mean(b);V.difference=mean(a)-mean(b);
        % Stored integer indices contain resampling draws, not result values.
        ix=double(draws.(sprintf('n%d',n)));
        ba=mean(reshape(a(ix),size(ix)),2);bb=mean(reshape(b(ix),size(ix)),2);
        V.difference_CI95=percentile95(ba-bb);
        if mean(b)~=0
            V.reduction_pct=100*(1-mean(a)/mean(b));
            if all(bb~=0),V.reduction_pct_CI95=percentile95(100*(1-ba./bb));end
        end
    end
    P.metrics.(f)=V;
end
end

function q=percentile95(x)
% Linear interpolation of order statistics (the inclusive/type-7 rule).
x=sort(x(:));q=nan(1,2);n=numel(x);
if n==0,return;end
for j=1:2
    p=[.025 .975];h=1+(n-1)*p(j);lo=floor(h);hi=ceil(h);
    q(j)=x(lo)+(h-lo)*(x(hi)-x(lo));
end
end

function S=summary_table(T,methods,criterion)
rows=cell(1,numel(methods));
for i=1:numel(methods)
    X=T(T.method==string(methods{i}),:);Y=X(X.(criterion)==1,:);
    rows{i}=table(string(methods{i}),height(X),sum(X.physical_success==1),...
        sum(X.accepted==1),height(Y),mean(Y.R_soft),mean(Y.T_s),mean(Y.Eadj_J),...
        'VariableNames',{'Method','Planned','Physical','Accepted','Mean_sample_count',...
        'R_soft','Arrival_time_s','Adjusted_energy_J'});
end
S=vertcat(rows{:});
end

function E=effect_table(P)
rows=cell(1,numel(P));
for i=1:numel(P)
    a=P(i);r=a.metrics.R_soft;t=a.metrics.T_s;e=a.metrics.Eadj_J;
    rows{i}=table(string(a.method),string(a.reference),a.n,...
        r.reduction_pct,r.reduction_pct_CI95(1),r.reduction_pct_CI95(2),...
        t.difference,t.difference_CI95(1),t.difference_CI95(2),...
        e.difference,e.difference_CI95(1),e.difference_CI95(2),...
        'VariableNames',{'Method','Reference','Paired_tasks','R_reduction_pct',...
        'R_CI_low','R_CI_high','Time_difference_s','Time_CI_low','Time_CI_high',...
        'Energy_difference_J','Energy_CI_low','Energy_CI_high'});
end
E=vertcat(rows{:});
end

function D=payload_data(T,draws)
ids={'Fixed-alpha1','RG-0','RG-P','Transformer-PR','MLP-PR','MPC-RG'};
labels={'Baseline','RG-0','RG-P','TRF-PR','MLP-PR','RG-grid'};
colors=[154 110 75;118 124 130;48 134 107;36 117 174;222 138 50;192 93 121];
methods=cell(1,6);
for j=1:6,methods{j}=struct('id',ids{j},'label',labels{j},'color',colors(j,:));end
methods=[methods{:}];
D=struct('schema','payload_publication_plots_v1','source','Fresh simulation summaries',...
    'methods',methods,'strata',struct([]),'statistics',...
    struct('resamples',2000,'seed',20261023,'unit','paired task'),...
    'notes','Means use accepted tasks; paired effects use common task IDs.');
labelsLoad={'[0.120,0.150] kg','[0.150,0.180] kg','[0.180,0.210] kg','[0.210,0.240] kg'};
for s=1:4
    X=T(T.payload_stratum==s,:);means=cell(1,6);pairs=cell(1,2);
    for j=1:6
        Y=X(X.method==string(ids{j})&X.accepted==1,:);
        means{j}=struct('method',ids{j},'n',height(Y),'R_soft',mean(Y.R_soft),...
            'T_s',mean(Y.T_s),'Eadj_J',mean(Y.Eadj_J));
    end
    for j=1:2
        a=paired(X,ids{j+3},'Fixed-alpha1','accepted',draws);
        r=a.metrics.R_soft;t=a.metrics.T_s;e=a.metrics.Eadj_J;
        pairs{j}=struct('method',ids{j+3},'n',a.n,'R_mean',r.method_mean,...
            'R_decrease',-r.difference,'R_decrease_CI95',-fliplr(r.difference_CI95),...
            'time_reduction_pct',t.reduction_pct,'time_CI95',t.reduction_pct_CI95,...
            'energy_reduction_pct',e.reduction_pct,'energy_CI95',e.reduction_pct_CI95);
    end
    P=X(X.method=="PTP",:);ptp=counts(P);ptpTemplates=cell(1,3);
    templates={'forward','reverse','yaw_shift'};
    for j=1:3
        c=counts(P(P.template_name==string(templates{j}),:));c.name=templates{j};
        ptpTemplates{j}=c;
    end
    ptp.templates=[ptpTemplates{:}];
    value=struct('id',s,'label',labelsLoad{s},'means',[means{:}],'paired',[pairs{:}],'ptp',ptp);
    if s==1,D.strata=value;else,D.strata(s)=value;end
end
end

function C=counts(T)
C=struct('scheduled',height(T),'physical',sum(T.physical_success==1),...
    'accepted',sum(T.accepted==1),'no_path',sum(T.physical_success~=1),...
    'physical_only',sum(T.physical_success==1&T.accepted~=1));
end

function T=payload_table(D)
rows={};
for s=1:4
    for j=1:2
        p=D.strata(s).paired(j);
        rows{end+1}=table(string(D.strata(s).label),string(p.method),p.n,p.R_mean,...
            p.R_decrease,p.R_decrease_CI95(1),p.R_decrease_CI95(2),...
            p.time_reduction_pct,p.time_CI95(1),p.time_CI95(2),...
            p.energy_reduction_pct,p.energy_CI95(1),p.energy_CI95(2),...
            'VariableNames',{'Payload_interval','Method','Paired_tasks','R_soft',...
            'R_decrease','R_decrease_CI_low','R_decrease_CI_high','Time_reduction_pct',...
            'Time_CI_low','Time_CI_high','Energy_reduction_pct','Energy_CI_low','Energy_CI_high'}); %#ok<AGROW>
    end
end
T=vertcat(rows{:});
end

function [S,E]=condition_tables(T,cases,methods,draws)
sr={};er={};
for c=1:numel(cases)
    X=T(T.case_id==cases(c).id,:);
    V=summary_table(X,methods,'physical_success');
    V=addvars(V,repmat(cases(c).id,height(V),1),...
        repmat(string(cases(c).name),height(V),1),'Before',1,...
        'NewVariableNames',{'Condition_id','Condition'});sr{end+1}=V; %#ok<AGROW>
    for j=3:4
        P=paired(X,methods{j},'RG-P','physical_success',draws);V=effect_table(P);
        V=addvars(V,cases(c).id,string(cases(c).name),'Before',1,...
            'NewVariableNames',{'Condition_id','Condition'});
        V.Policy_arrival_time_s=P.metrics.T_s.method_mean;
        er{end+1}=V; %#ok<AGROW>
    end
end
S=vertcat(sr{:});E=vertcat(er{:});
end

function S=parameter_table(T,cases,methods)
rows={};
for c=1:numel(cases)
    X=T(T.case_id==cases(c).id,:);
    V=summary_table(X,methods,'physical_success');
    V=addvars(V,repmat(cases(c).id,height(V),1),...
        repmat(string(cases(c).name),height(V),1),'Before',1,...
        'NewVariableNames',{'Condition_id','Condition'});
    rows{end+1}=V; %#ok<AGROW>
end
S=vertcat(rows{:});
end

function F=filter_data(resultDir)
Q=load(fullfile(resultDir,'filter','trials.mat'),'T');T=Q.T;
for i=1:height(T),T.method(i)=method_name(T.method(i));end
taus=[.025 .05 .1 .2 .5];epsilons=[0 .025 .05 .1];rows={};
methods={'RG-0','RG-P','Transformer-PR','MLP-PR','PTP','MPC-RG','Fixed-alpha1'};
for i=1:numel(methods)
    for t=taus
        for e=epsilons
            X=T(string(T.method)==string(methods{i})&T.accepted==1&T.metric_valid==1&...
                abs(T.window_s-t)<1e-10&abs(T.tolerance_frac-e)<1e-10,:);
            rows{end+1}=struct('method',methods{i},'tau_s',t,'epsilon',e,...
                'n',height(X),'R_soft_mean',mean(X.R_soft),...
                'I_epsilon_mean_s',mean(X.excess_norm_integral_s)); %#ok<AGROW>
        end
    end
end
F=struct('taus_s',taus,'epsilons',epsilons,'summary',[rows{:}]);
end

function T=design_table()
Study=["Robustness";"Robustness";"Robustness";"Robustness";"Robustness";...
    "Robustness";"Known-parameter sensitivity";"Known-parameter sensitivity";...
    "Known-parameter sensitivity";"Closed-loop filtering";"Observation filtering";"Exceedance tolerance"];
Condition=["Joint-position, velocity and torque noise";"Boundary noise";...
    "Unknown friction";"Unknown payload and center of mass";"Unknown source voltage";...
    "Combined perturbations";"Motor endpoints";"Capacitance and source resistance";...
    "Source voltage";"Power observation time constant";"Power observation time constant";"Reporting threshold"];
Settings=["Standard deviations 3e-5 rad, 3e-5 rad/s and 3e-5 N m";...
    "Standard deviations 5e-5 rad, 5e-5 rad/s and 5e-5 N m";"+/-5 percent";...
    "Mass +/-5 percent; center-of-mass offset +/-1 mm";"11.7 to 12.3 V";...
    "Low joint noise with friction, payload/COM and voltage perturbations";...
    "Stall current, no-load current, stall torque and no-load speed: +/-5 percent";...
    "Capacitance and source resistance: +/-10 percent";"11.7 and 12.3 V";...
    "25 and 100 ms, relative to nominal 50 ms";"25, 50, 100, 200 and 500 ms";"0, 2.5, 5 and 10 percent"];
T=table(Study,Condition,Settings);
end

function write_json(path,value)
f=fopen(path,'w','n','UTF-8');clean=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s',jsonencode(value,'PrettyPrint',true));
end
