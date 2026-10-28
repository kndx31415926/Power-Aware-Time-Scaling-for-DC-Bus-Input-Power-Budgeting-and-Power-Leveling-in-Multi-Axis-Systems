function run_matlab_figures(dataDir,outDir)
%RUN_MATLAB_FIGURES Draw Figures 2-9 from computed result summaries and histories.
if nargin<1,dataDir=fullfile(fileparts(fileparts(mfilename('fullpath'))),'results','report');end
if nargin<2,outDir=fullfile(dataDir,'figures');end
make_representative_figure(dataDir,outDir);
make_payload_figures(dataDir,outDir);
make_supplement_figures(dataDir,outDir);
fprintf('Figures 2-9 written to %s\n',outDir);
end
