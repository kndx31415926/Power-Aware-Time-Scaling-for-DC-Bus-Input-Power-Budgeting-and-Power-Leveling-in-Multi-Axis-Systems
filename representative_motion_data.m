function D=representative_motion_data(packageRoot,reportDir)
% Extract the seven-method time histories used in the representative figure.
% The selected task is fixed in inputs/representative_task.json. Curves use
% the recorded 1 ms samples through each method's arrival at the target.
if nargin<1,packageRoot=fileparts(mfilename('fullpath'));end
if nargin<2,reportDir=fullfile(packageRoot,'results','report');end
choice=jsondecode(fileread(fullfile(packageRoot,'inputs','representative_task.json')));
taskId=choice.task_id;
saved={'Baseline','NoGuard','Feasible-RG','Nominal-RG-grid',...
    'Bus-PTP-beam','TRF-PRG','MLP-PRG'};
keys={'Baseline','RG-0','RG-P','Nominal-RG-grid',...
    'Bus-PTP-beam','Transformer-PR','MLP-PR'};
labels={'Baseline','RG-0','RG-P','RG-grid','PTP','TRF-PR','MLP-PR'};
folder=fullfile(packageRoot,'results','main',sprintf('task_%03d',taskId));
metadata=readtable(fullfile(packageRoot,'inputs','final_test_manifest.csv'),'TextType','string');
row=metadata(metadata.task_id==taskId,:);
for m=1:numel(saved)
    S=load(fullfile(folder,[matlab.lang.makeValidName(saved{m}) '.mat']),'Result');
    out=S.Result.out; L=out.log; cfg=out.cfg;
    A=out.execution_audit;
    if isfield(A,'Pavg_replay_authoritative') && A.Pavg_replay_authoritative
        filtered=A.Pgrid_avg_replay_W;
    else
        filtered=L.Pgrid_avg;
    end
    M=struct('key',keys{m},'label',labels{m},'t',double(L.t(:)),...
        'ee',double(L.ee),'P_source',double(L.Pgrid(:)),...
        'P_filtered',double(filtered(:)),'P_joint',double(L.Pi_raw),...
        'initial_skeleton',joint_points(L.q_pre(1,:),cfg.ROBOT_PARAMS),...
        'final_skeleton',joint_points(L.q(end,:),cfg.ROBOT_PARAMS),...
        'q0',double(L.q_pre(1,:)),'qfinal',double(L.q(end,:)),...
        'q',double(L.q),'dq_pwr',double(L.dq_pwr),'tau',double(L.tau));
    D.methods(m)=M;
end
D.meta=struct('task_id',taskId,'payload_kg',double(row.payload_mass_kg),...
    'template_label',char(row.template_name),'P_budget',10.5,'filter_tau_s',.05,...
    'mechanical_power_definition','P_joint = tau .* dq_pwr (midpoint velocity)',...
    'source_power_definition','Source input power, Pgrid',...
    'filtered_power_definition','Causal 50 ms low-pass source power',...
    'sample_semantics','Step-end time; 1 ms robot and bus coupling step',...
    'link_lengths_m',double(cfg.ROBOT_PARAMS.L),...
    'base_joint_xyz_m',double(cfg.ROBOT_PARAMS.base_joint_xyz));
if ~isfolder(reportDir),mkdir(reportDir);end
save(fullfile(reportDir,'representative_motion.mat'),'D','-v7');
end

function X=joint_points(q,P)
% Reduced four-joint geometry: base yaw followed by three pitch joints.
q=double(q(:).'); lengths=double(P.L(:).');
pitch=cumsum(q(2:4)); point=double(P.base_joint_xyz(:).');
X=zeros(5,3);X(2,:)=point;
for j=1:3
    direction=[cos(q(1))*cos(pitch(j)),sin(q(1))*cos(pitch(j)),-sin(pitch(j))];
    point=point+lengths(j)*direction;
    X(j+2,:)=point;
end
end
