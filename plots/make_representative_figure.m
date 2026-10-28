function make_representative_figure(dataDir, outDir)
% Compare motion and power histories for one common robot task.
% Input: representative_motion.mat, containing the structure D.
% Every curve uses its recorded samples and its own completion time.
if nargin < 1 || isempty(dataDir), dataDir = fileparts(mfilename('fullpath')); end
if nargin < 2 || isempty(outDir), outDir = fullfile(dataDir,'figures'); end
if ~isfolder(outDir), mkdir(outDir); end
S = load(fullfile(dataDir,'representative_motion.mat'),'D');
D = S.D;
nMethods = numel(D.methods);
labels = cell(1,nMethods);
C = zeros(nMethods,3); lineStyles = cell(nMethods,1);
for m = 1:nMethods
    [C(m,:),lineStyles{m},labels{m}] = methodStyle(D.methods(m),m);
end

% Colors are shared with the payload-stratified figures in the manuscript.
f = figure('Visible','off','Color','white','Units','centimeters', ...
    'Position',[2 2 17 21],'PaperUnits','centimeters', ...
    'PaperSize',[17 21],'PaperPosition',[0 0 17 21], ...
    'InvertHardcopy','off','Renderer','painters');
pos = [.10 .650 .36 .235; .59 .650 .37 .235; ...
       .10 .360 .37 .230; .59 .360 .37 .230; ...
       .10 .070 .37 .230; .59 .070 .37 .230];
axs = gobjects(6,1);
for k = 1:6
    axs(k) = axes(f,'Position',pos(k,:));
    styleAxes(axs(k)); hold(axs(k),'on');
end

% A single recorded trace represents the common geometric path.
ax = axs(1);
M = D.methods(1);
allPoints = M.ee;
plot3(ax,M.ee(:,1),M.ee(:,2),M.ee(:,3), ...
    'Color',[.14 .46 .62],'LineStyle','-','LineWidth',1.3, ...
    'HandleVisibility','off');
initial = D.methods(1).initial_skeleton;
final = D.methods(1).final_skeleton;
plot3(ax,initial(:,1),initial(:,2),initial(:,3),'-o', ...
    'Color',[.40 .43 .46],'LineWidth',1.15,'MarkerSize',3.2, ...
    'MarkerFaceColor','white','HandleVisibility','off');
plot3(ax,final(:,1),final(:,2),final(:,3),'--s', ...
    'Color',[.22 .24 .27],'LineWidth',1.05,'MarkerSize',3.0, ...
    'MarkerFaceColor','white','HandleVisibility','off');
allPoints = [allPoints; initial; final];
xlim(ax,paddedRange(allPoints(:,1),.09));
ylim(ax,paddedRange(allPoints(:,2),.09));
zlim(ax,paddedRange(allPoints(:,3),.09));
daspect(ax,[1 1 1]); view(ax,[-39 24]);
% The transverse dimension is narrow; two ticks keep its true scale readable.
yTicks = ax.YTick;
if numel(yTicks) > 2, yticks(ax,yTicks([1 end])); end
ax.XGrid='on'; ax.YGrid='on'; ax.ZGrid='on';
xlabel(ax,'x (m)','FontSize',8);
ylabel(ax,'y (m)','FontSize',8);
zlabel(ax,'z (m)','FontSize',8);
panelTitle(ax,'(a) Common end-effector path');

% The bus panel shows the same causal filtered source-power signal used in
% the experimental power metric. No additional display filter is applied.
ax = axs(2);
maxTime = max(arrayfun(@(M) max(M.t(:)),D.methods));
allBus = D.meta.P_budget;
h = gobjects(nMethods,1);
for m = 1:nMethods
    M = D.methods(m);
    h(m) = plot(ax,M.t,M.P_filtered,'Color',C(m,:),'LineStyle',lineStyles{m}, ...
        'LineWidth',.90);
    allBus = [allBus; M.P_filtered(:)]; %#ok<AGROW>
end
yline(ax,D.meta.P_budget,':','Color',[.18 .20 .23], ...
    'LineWidth',1.0,'HandleVisibility','off');
xlim(ax,[0 1.025*maxTime]); ylim(ax,paddedRange([0;allBus],.07));
text(ax,.99*maxTime,D.meta.P_budget,sprintf(' %.1f W',D.meta.P_budget), ...
    'Units','data','HorizontalAlignment','right','VerticalAlignment','bottom', ...
    'FontName','Arial','FontSize',7.1,'Color',[.18 .20 .23]);
xlabel(ax,'Time (s)','FontSize',8);
ylabel(ax,'Filtered source power (W)','FontSize',8);
panelTitle(ax,'(b) DC-bus source power');

% Signed joint mechanical power is tau_j * dq_j. All samples, including
% short transients and regenerative intervals, remain in the exported plot.
for j = 1:4
    ax = axs(j+2);
    allPower = 0;
    for m = 1:nMethods
        M = D.methods(m);
        plot(ax,M.t,M.P_joint(:,j),'Color',C(m,:),'LineStyle',lineStyles{m}, ...
            'LineWidth',.78);
        allPower = [allPower; M.P_joint(:,j)]; %#ok<AGROW>
    end
    yline(ax,0,'-','Color',[.78 .80 .82],'LineWidth',.45, ...
        'HandleVisibility','off');
    xlim(ax,[0 1.025*maxTime]); ylim(ax,paddedRange(allPower,.07));
    xlabel(ax,'Time (s)','FontSize',8);
    ylabel(ax,sprintf('P_{mech,%d} (W)',j),'Interpreter','tex','FontSize',8);
    panelTitle(ax,sprintf('(%c) Joint %d mechanical power','b'+j,j));
end

lg = legend(axs(2),h,labels,'NumColumns',4,'Box','off','FontSize',8, ...
    'Interpreter','none','Orientation','horizontal');
lg.Units='normalized'; lg.Position=[.055 .938 .91 .052];
lg.ItemTokenSize=[21 12];
subtitle = 'Progress regulation along a fixed path';
annotation(f,'textbox',[.08 .902 .86 .024],'String',subtitle, ...
    'EdgeColor','none','Margin',0,'HorizontalAlignment','center', ...
    'VerticalAlignment','middle','FontName','Arial','FontSize',7.7, ...
    'Interpreter','none','Color',[.25 .28 .31]);

drawnow;
exportgraphics(f,fullfile(outDir,'representative_motion.pdf'), ...
    'ContentType','vector','BackgroundColor','white', ...
    'Units','centimeters','Width',17,'Height',21,'Padding','figure');
exportgraphics(f,fullfile(outDir,'representative_motion.png'), ...
    'Resolution',300,'BackgroundColor','white', ...
    'Units','centimeters','Width',17,'Height',21,'Padding','figure');
savefig(f,fullfile(outDir,'representative_motion.fig'));
close(f);
fprintf('Representative motion figure exported to %s.\n',outDir);
end

function [color,lineStyle,label] = methodStyle(M,index)
% Preserve the paper's method colors even if the input order changes.
colors = [154 110 75; 118 124 130; 48 134 107; 192 93 121; ...
          120 83 163; 36 117 174; 222 138 50]/255;
styles = {'-','--','-.',':','-.','-','--'};
names = {'Baseline','RG-0','RG-P','RG-grid','PTP','TRF-PR','MLP-PR'};
name = lower(regexprep([char(string(M.key)) ' ' char(string(M.label))], ...
    '[^a-zA-Z0-9]',''));
if contains(name,'fixed') || contains(name,'baseline')
    id=1;
elseif contains(name,'rg0')
    id=2;
elseif contains(name,'rgp')
    id=3;
elseif contains(name,'grid') || contains(name,'mpc')
    id=4;
elseif contains(name,'ptp')
    id=5;
elseif contains(name,'trf') || contains(name,'transformer')
    id=6;
elseif contains(name,'mlp')
    id=7;
else
    id = 1 + mod(index-1,size(colors,1));
end
color=colors(id,:); lineStyle=styles{id}; label=names{id};
end

function styleAxes(ax)
ax.FontName='Arial'; ax.FontSize=7.5; ax.LineWidth=.60;
ax.Box='off'; ax.TickDir='out'; ax.TickLength=[.014 .014];
ax.YGrid='on'; ax.XGrid='off'; ax.GridColor=[.4 .45 .5];
ax.GridAlpha=.14; ax.Layer='top';
end

function panelTitle(ax,label)
title(ax,label,'FontName','Arial','FontSize',8.4, ...
    'FontWeight','normal','HorizontalAlignment','left');
ax.Title.Units='normalized'; ax.Title.Position(1)=0;
end

function lim = paddedRange(values,fraction)
values = values(isfinite(values));
lo = min(values); hi = max(values);
span = hi-lo;
if span == 0, span=max(1,abs(hi)); end
lim = [lo-fraction*span, hi+fraction*span];
end
