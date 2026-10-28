function make_payload_figures(dataDir, outDir)
% Payload-stratified power, arrival time, energy and completion figures.
% Numerical inputs are written by compute_report from the simulation outcomes.
% Reproduction: make_payload_figures(sourceFolder, outputFolder)
if nargin < 1 || isempty(dataDir), dataDir = fullfile(fileparts(fileparts(mfilename('fullpath'))),'results','report'); end
if nargin < 2 || isempty(outDir), outDir = fullfile(dataDir,'figures'); end
if ~isfolder(outDir), mkdir(outDir); end
dataFile = fullfile(dataDir,'payload_tables.json');
D = jsondecode(fileread(dataFile));
C = zeros(6,3);
for m=1:6, C(m,:)=D.methods(m).color(:)'/255; end
labels = {D.methods.label};
markers = {'o','s','^','d','v','p'};
shortLoads = {'S1','S2','S3','S4'};
loadLabels = {'0.120-0.150','0.150-0.180','0.180-0.210','0.210-0.240'};

% Method means use the accepted task population in each payload interval.
f = canvas(160,148);
pos = [.14 .665 .835 .220; .14 .385 .835 .220; .14 .105 .835 .220];
metrics = {'R_soft','T_s','Eadj_J'};
scales = [1000 1 1];
ylabs = {'R_{soft} (10^{-3})','T_a (s)','E_{adj} (J)'};
titles = {'(a) Mean soft-budget excess','(b) Mean arrival time', ...
          '(c) Mean storage-adjusted source energy'};
axesTrade = gobjects(3,1);
for k=1:3
    ax = axes(f,'Position',pos(k,:)); axesTrade(k)=ax; style(ax); hold(ax,'on');
    h = gobjects(6,1);
    for m=1:6
        y=zeros(1,4);
        for s=1:4, y(s)=D.strata(s).means(m).(metrics{k})*scales(k); end
        % Larger hollow markers expose near-coincident curves without jitter.
        h(m)=plot(ax,1:4,y,'-','Color',C(m,:),'LineWidth',1.15, ...
            'Marker',markers{m},'MarkerSize',5.2,'MarkerFaceColor','white', ...
            'MarkerEdgeColor',C(m,:));
    end
    xlim(ax,[.88 4.12]); xticks(ax,1:4);
    if k==1
        ylim(ax,[-.25 10]); yticks(ax,0:2:10);
    elseif k==2
        ylim(ax,[2.45 3.75]); yticks(ax,2.5:.25:3.75);
    else
        ylim(ax,[12.5 29]); yticks(ax,15:5:30);
    end
    if k<3, xticklabels(ax,shortLoads); else, xticklabels(ax,loadLabels); end
    ylabel(ax,ylabs{k},'Interpreter','tex','FontSize',8.5);
    title(ax,titles{k},'FontWeight','normal','FontSize',9,'HorizontalAlignment','left');
    ax.Title.Units='normalized'; ax.Title.Position(1)=0;
    if k==1
        lg=legend(ax,h,labels,'NumColumns',3,'Box','off','FontSize',8, ...
                  'Interpreter','none','Orientation','horizontal');
        lg.Units='normalized'; lg.Position=[.15 .925 .82 .058];
    end
end
xlabel(axesTrade(3),'Payload interval (kg)','FontSize',8.5);
savePlot(f,outDir,'payload_tradeoff',148);

% Baseline-minus-method risk decrease avoids a percentage at zero baseline.
f = canvas(160,93);
pos=[.18 .27 .225 .53; .465 .27 .225 .53; .75 .27 .225 .53];
means={'R_decrease','time_reduction_pct','energy_reduction_pct'};
cis={'R_decrease_CI95','time_CI95','energy_CI95'};
titles={'(a) Soft-budget excess','(b) Arrival time','(c) Source energy'};
xlabs={{'R_{soft}: Baseline - policy','(10^{-3})'}, ...
       {'Reduction in T_a','(%)'},{'Reduction in E_{adj}','(%)'}};
scales=[1000 1 1];
for k=1:3
    ax=axes(f,'Position',pos(k,:)); style(ax); hold(ax,'on');
    ax.XGrid='on'; ax.YGrid='off';
    xline(ax,0,':','Color',[.42 .45 .48],'LineWidth',.9);
    for m=1:2
        yy=(1:4)+(m-1.5)*.22; vals=zeros(1,4); lo=vals; hi=vals;
        for s=1:4
            pair=D.strata(s).paired(m); vals(s)=pair.(means{k})*scales(k);
            ci=pair.(cis{k})*scales(k); lo(s)=ci(1); hi(s)=ci(2);
        end
        h(m)=errorbar(ax,vals,yy,vals-lo,hi-vals,'horizontal','LineStyle','none', ...
            'Color',C(m+3,:),'Marker',markers{m+3},'MarkerSize',5, ...
            'MarkerFaceColor','white','LineWidth',1.2,'CapSize',4);
    end
    ylim(ax,[.55 4.45]); ax.YDir='reverse'; yticks(ax,1:4);
    if k==1
        yticks(ax,1:4); yticklabels(ax,{'S1  0.120-0.150','S2  0.150-0.180', ...
            'S3  0.180-0.210','S4  0.210-0.240'}); ax.YAxis.FontSize=7.8;
        xlim(ax,[-.65 4.1]); xticks(ax,0:1:4);
    elseif k==2
        yticklabels(ax,{}); xlim(ax,[-.5 14]); xticks(ax,0:4:12);
    else
        yticklabels(ax,{}); xlim(ax,[-4 12]); xticks(ax,-4:4:12);
    end
    xlabel(ax,xlabs{k},'Interpreter','tex','FontSize',8.2);
    title(ax,titles{k},'FontWeight','normal','FontSize',8.5,'HorizontalAlignment','left');
    ax.Title.Units='normalized'; ax.Title.Position(1)=0;
    if k==2
        lg=legend(ax,h(1:2),{'TRF-PR','MLP-PR'},'Orientation','horizontal', ...
                  'Box','off','FontSize',8.5,'Interpreter','none');
        lg.Units='normalized'; lg.Position=[.38 .88 .4 .07];
    end
end
note(f,[.03 .025 .94 .055], ...
    'Positive = lower metric than Baseline; bars show paired 95% bootstrap intervals.',8);
savePlot(f,outDir,'payload_effects',93);

% Planned denominators are retained, including unsuccessful bounded searches.
f=canvas(160,126);
ax=axes(f,'Position',[.14 .57 .835 .285]); style(ax); hold(ax,'on');
counts=zeros(4,3);
for s=1:4
    q=D.strata(s).ptp;
    counts(s,:)=[q.accepted q.physical_only q.no_path];
end
maxTasks=max(sum(counts,2));
b=bar(ax,1:4,counts,.55,'stacked','EdgeColor','white','LineWidth',.5);
barC=[.48 .36 .65; .89 .65 .25; .80 .82 .84];
for k=1:3, b(k).FaceColor=barC(k,:); end
for s=1:4
    q=D.strata(s).ptp;
    text(ax,s,q.accepted/2,sprintf('%d/%d',q.accepted,q.scheduled), ...
        'HorizontalAlignment','center','Color','white','FontName','Arial', ...
        'FontSize',8.5,'FontWeight','bold');
    if q.no_path>0
        labelX=s; align='center';
        if q.no_path==1, labelX=s+.33; align='left'; end
        text(ax,labelX,q.accepted+q.physical_only+q.no_path/2,sprintf('%d',q.no_path), ...
            'HorizontalAlignment',align,'Color',[.18 .2 .22], ...
            'FontName','Arial','FontSize',8);
    end
    if q.physical_only>0
        text(ax,s-.31,q.accepted+q.physical_only/2,sprintf('%d only',q.physical_only),'FontSize',7.8, ...
            'HorizontalAlignment','right','FontName','Arial','Color',[.42 .30 .10]);
    end
    text(ax,s,1.048*maxTasks,sprintf('Physical %d/%d',q.physical,q.scheduled), ...
        'HorizontalAlignment','center','FontName','Arial','FontSize',7.8);
end
xlim(ax,[.55 4.45]); ylim(ax,[0 1.18*maxTasks]); yticks(ax,linspace(0,maxTasks,6));
xticks(ax,1:4); xticklabels(ax,shortLoads); ylabel(ax,'Planned tasks','FontSize',8.5);
title(ax,'(a) Bus-PTP-beam outcomes by payload stratum','FontSize',9, ...
    'FontWeight','normal','HorizontalAlignment','left');
ax.Title.Units='normalized'; ax.Title.Position(1)=0;
lg=legend(ax,b,{'Accepted','Physical only','Incomplete'},'Box','off', ...
    'Orientation','horizontal','FontSize',8,'Interpreter','none');
lg.Units='normalized'; lg.Position=[.16 .91 .81 .045];

ax=axes(f,'Position',[.14 .17 .835 .265]); hold(ax,'on');
base=[.96 .95 .98]; dark=[.48 .36 .65];
for s=1:4
    for t=1:3
        q=D.strata(s).ptp.templates(t); rate=q.accepted/max(1,q.scheduled);
        col=base+(dark-base)*rate;
        rectangle(ax,'Position',[s-.5 t-.5 1 1],'FaceColor',col, ...
            'EdgeColor','white','LineWidth',1.5);
        ink=[.18 .17 .21]; if rate>.6, ink=[1 1 1]; end
        text(ax,s,t,sprintf('%d/%d',q.accepted,q.scheduled), ...
            'Color',ink,'HorizontalAlignment','center','FontName','Arial', ...
            'FontSize',9,'FontWeight','bold');
    end
end
ax.FontName='Arial'; ax.FontSize=8; ax.TickLength=[0 0]; ax.Box='off';
ax.YDir='reverse'; xlim(ax,[.5 4.5]); ylim(ax,[.5 3.5]);
xticks(ax,1:4); xticklabels(ax,loadLabels);
yticks(ax,1:3); yticklabels(ax,{'Forward','Reverse','Yaw shift'});
xlabel(ax,'Payload stratum (kg)','FontSize',8.5);
title(ax,'(b) Accepted / planned by motion template','FontSize',9, ...
    'FontWeight','normal','HorizontalAlignment','left');
ax.Title.Units='normalized'; ax.Title.Position(1)=0;
note(f,[.04 .025 .92 .055], ...
    {'Counts include every scheduled task; template cells show accepted / planned.', ...
     'Physical-only outcomes reach the endpoint outside the relative-time acceptance criterion.'},7.8);
savePlot(f,outDir,'payload_completion',126);
fprintf('Payload figures exported: payload_tradeoff, payload_effects, payload_completion.\n');
end

function f=canvas(w,h)
f=figure('Visible','off','Color','white','Units','centimeters', ...
    'Position',[2 2 w/10 h/10],'PaperUnits','centimeters', ...
    'PaperSize',[w/10 h/10],'PaperPosition',[0 0 w/10 h/10], ...
    'InvertHardcopy','off');
end

function style(ax)
ax.FontName='Arial'; ax.FontSize=8; ax.LineWidth=.65;
ax.Box='off'; ax.TickDir='out'; ax.TickLength=[.012 .012];
ax.YGrid='on'; ax.XGrid='off'; ax.GridColor=[.4 .45 .5]; ax.GridAlpha=.14;
ax.Layer='top';
end

function note(f,p,s,sz)
annotation(f,'textbox',p,'String',s,'EdgeColor','none','Margin',0, ...
    'HorizontalAlignment','center','VerticalAlignment','middle', ...
    'FontName','Arial','FontSize',sz,'Interpreter','tex','Color',[.25 .28 .31]);
end

function savePlot(f,outDir,name,h)
drawnow;
exportgraphics(f,fullfile(outDir,[name '.pdf']),'ContentType','vector', ...
    'BackgroundColor','white','Units','centimeters','Width',16,'Height',h/10,'Padding','figure');
exportgraphics(f,fullfile(outDir,[name '.png']),'Resolution',300, ...
    'BackgroundColor','white','Units','centimeters','Width',16,'Height',h/10,'Padding','figure');
savefig(f,fullfile(outDir,[name '.fig']));
close(f);
end
