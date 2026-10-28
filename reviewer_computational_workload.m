function [T,W]=reviewer_computational_workload(packageRoot,outputDir)
%REVIEWER_COMPUTATIONAL_WORKLOAD Calculate the manuscript's Table 10.
% The supplied architecture and search dimensions are in
% inputs/computational_workload.json. No trajectories or network evaluations
% are required: matrix MACs and structural search bounds follow analytically.
% Online methods are counted per policy update; PTP is counted per plan.
if nargin<1,packageRoot=fileparts(mfilename('fullpath'));end
if nargin<2,outputDir=fullfile(packageRoot,'results','report');end
W=jsondecode(fileread(fullfile(packageRoot,'inputs','computational_workload.json')));
N=W.input.preview_tokens;D=W.input.retained_features_per_token;

% Flatten all preview tokens for the MLP, followed by its scalar output.
widths=[N*D,reshape(W.networks.MLP.hidden_widths,1,[]),1];
mlpMACs=sum(widths(1:end-1).*widths(2:end));
mlpParameters=mlpMACs+sum(widths(2:end));

% Full-sequence attention precedes first-token selection and the output head.
trf=W.networks.Transformer;
d=trf.projection_width;L=trf.encoder_blocks;
f=trf.feedforward_width;o=trf.output_head_width;
trfMACs=N*D*d+L*(4*N*d^2+2*N^2*d+2*N*d*f)+d*o+o;
% Parameters include dense biases and two layer-normalization scale/offset
% pairs per encoder. The attention matrix products have no added parameters.
encoderParameters=4*d^2+4*d+2*d*f+f+d+4*d;
trfParameters=D*d+d+L*encoderParameters+d*o+o+o+1;
W.networks.MLP.matrix_MACs=mlpMACs;
W.networks.MLP.parameter_count=mlpParameters;
W.networks.Transformer.matrix_MACs=trfMACs;
W.networks.Transformer.parameter_count=trfParameters;

% Search bounds count candidate transitions independently of neural MACs.
grid=W.search.Nominal_RG_grid;
gridCandidates=grid.candidate_rates*(1+(grid.horizon_intervals-1)*grid.beam_width);
gridSteps=gridCandidates*grid.prediction_steps_per_interval_max;
ptp=W.search.Bus_PTP_beam;
ptpCandidates=ptp.candidate_rates*(1+(ptp.path_nodes-2)*ptp.beam_width);
W.search.Nominal_RG_grid.candidate_transition_upper_bound=gridCandidates;
W.search.Nominal_RG_grid.prediction_step_upper_bound=gridSteps;
W.search.Bus_PTP_beam.candidate_edge_upper_bound=ptpCandidates;

Method=["Baseline";"RG-0";"RG-P";"Transformer-PR";"MLP-PR";...
    "Nominal-RG-grid";"Bus-PTP-beam"];
Scope=[repmat("per policy update",6,1);"per complete motion plan"];
Neural_preview_tokens=[0;0;0;N;N;0;0];
Network_matrix_MACs=[0;0;0;trfMACs;mlpMACs;0;0];
Candidate_transitions=["0";"N_eval^0";"N_eval^P";"0";"0";...
    "<="+string(gridCandidates);"<="+string(ptpCandidates)];
Internal_prediction_steps=["0";"W_RG^0";"W_RG^P";"0";"0";...
    "<="+string(gridSteps);"W_P"];
Count_description=["fixed request";...
    "measured-state prediction; state-dependent count";...
    "measured-state prediction; state-dependent count";...
    "network matrix MAC count";"network matrix MAC count";...
    "nominal-reference prediction; structural upper bounds";...
    "nominal-reference prediction; edge upper bound, variable steps per edge"];
T=table(Method,Scope,Neural_preview_tokens,Network_matrix_MACs,...
    Candidate_transitions,Internal_prediction_steps,Count_description);

if ~isfolder(outputDir),mkdir(outputDir);end
writetable(T,fullfile(outputDir,'table_10_computational_workload.csv'));
fid=fopen(fullfile(outputDir,'computational_workload.json'),'w','n','UTF-8');
fprintf(fid,'%s\n',jsonencode(W,PrettyPrint=true));fclose(fid);
end
