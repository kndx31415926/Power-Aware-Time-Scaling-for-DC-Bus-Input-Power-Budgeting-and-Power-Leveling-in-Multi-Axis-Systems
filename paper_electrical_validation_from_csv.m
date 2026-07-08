function R = paper_electrical_validation_from_csv(meas_csv, sim_mat_or_out, opts)
%PAPER_ELECTRICAL_VALIDATION_FROM_CSV  Compare measured bus signals with model prediction.
%
% Expected measurement CSV columns (names are flexible but should include):
%   t or time_s, Vdc_meas or Vbus, Igrid_meas or Ibus, optional Pgrid_meas.
% Prediction can be an out struct, a .mat file containing out, or a .mat file
% containing log.  Prediction columns are read from out.log.Pgrid/Igrid/Vdc.

if nargin < 3 || isempty(opts), opts = struct(); end
opts.export_dir = getfield_def(opts,'export_dir',fullfile(pwd,'paper_results_electrical_validation'));
ensure_dir(opts.export_dir);
M = readtable(meas_csv);
out = load_out(sim_mat_or_out);
log = out.log;
tm = get_col(M,{'t','time','time_s','Time'});
Vm = get_col(M,{'Vdc_meas','Vbus_meas','Vbus','Vdc','V'});
Im = get_col(M,{'Igrid_meas','Ibus_meas','Ibus','Igrid','I'});
Pm = [];
try, Pm = get_col(M,{'Pgrid_meas','Pbus_meas','Pbus','Pgrid','P'}); catch, end
if isempty(Pm), Pm = Vm.*Im; end

ts = log.t(:);
Pp = interp1(ts,log.Pgrid(:),tm,'linear','extrap');
if isfield(log,'Igrid'), Ip = interp1(ts,log.Igrid(:),tm,'linear','extrap'); else, Ip = Pp./max(interp1(ts,log.Vdc(:),tm,'linear','extrap'),eps); end
if isfield(log,'Vdc'), Vp = interp1(ts,log.Vdc(:),tm,'linear','extrap'); else, Vp = median(Vm,'omitnan')*ones(size(tm)); end

rows = {err_row('Pgrid',Pm,Pp,true); err_row('Igrid',Im,Ip,false); err_row('Vdc',Vm,Vp,false)};
T = struct2table(cell2mat(rows));
writetable(T,fullfile(opts.export_dir,'electrical_validation_errors.csv'));
Tout = table(tm, Pm, Pp, Im, Ip, Vm, Vp, 'VariableNames',{'t','Pgrid_meas','Pgrid_pred','Igrid_meas','Igrid_pred','Vdc_meas','Vdc_pred'});
writetable(Tout,fullfile(opts.export_dir,'electrical_validation_aligned_timeseries.csv'));
try
    f=figure('Name','Electrical validation','Visible',getfield_def(opts,'figure_visible','off'));
    tiledlayout(f,3,1,'Padding','compact','TileSpacing','compact');
    nexttile; plot(tm,Pm,'k-',tm,Pp,'r--'); grid on; ylabel('P_{grid} [W]'); legend('meas','model');
    nexttile; plot(tm,Im,'k-',tm,Ip,'r--'); grid on; ylabel('I_{grid} [A]');
    nexttile; plot(tm,Vm,'k-',tm,Vp,'r--'); grid on; ylabel('V_{dc} [V]'); xlabel('Time [s]');
    saveas(f,fullfile(opts.export_dir,'electrical_validation_timeseries.png'));
catch
end
R = struct('error_table',T,'aligned_table',Tout,'export_dir',opts.export_dir);
end

function r = err_row(name,y,yp,energyFlag)
e = yp-y;
r = struct('signal',name,'N',numel(y),'MAE',mean(abs(e),'omitnan'),'RMSE',sqrt(mean(e.^2,'omitnan')), ...
    'MaxAbsError',max(abs(e)),'PeakError',max(yp)-max(y),'R2',1-sum(e.^2,'omitnan')/max(sum((y-mean(y,'omitnan')).^2,'omitnan'),eps), ...
    'EnergyError_pct',NaN);
if energyFlag
    r.EnergyError_pct = 100*(trapz(max(yp,0))-trapz(max(y,0)))/max(trapz(max(y,0)),eps);
end
end
function out = load_out(x)
if isstruct(x)
    if isfield(x,'out'), out=x.out; else, out=x; end
else
    S=load(x);
    if isfield(S,'out'), out=S.out; elseif isfield(S,'log'), out=struct('log',S.log); else, error('MAT file must contain out or log'); end
end
end
function v = get_col(T,names)
v=[];
for i=1:numel(names)
    if any(strcmp(T.Properties.VariableNames,names{i}))
        v = T.(names{i})(:); return;
    end
end
error('Missing expected column. Tried: %s', strjoin(names,','));
end
function x=getfield_def(S,fn,def), if isstruct(S)&&isfield(S,fn)&&~isempty(S.(fn)), x=S.(fn); else, x=def; end, end
function ensure_dir(d), if exist(d,'dir')~=7, mkdir(d); end, end
