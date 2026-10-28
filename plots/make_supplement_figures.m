function make_supplement_figures(dataDir,outDir)
% Plot ablation, perturbation and sensitivity results from compute_report.
% Outputs are vector PDF, 300-dpi PNG and editable MATLAB FIG files.
if nargin<1, dataDir=fullfile(fileparts(fileparts(mfilename('fullpath'))),'results','report'); end
if nargin<2, outDir=fullfile(dataDir,'figures'); end
if ~exist(outDir,'dir'), mkdir(outDir); end
D=jsondecode(fileread(fullfile(dataDir,'analysis_data.json')));
F=jsondecode(fileread(fullfile(dataDir,'filter_metric_analysis.json')));
methods={'RG-0','RG-P','Transformer-PR','MLP-PR'};
labels={'RG-0','RG-P','TRF-PR','MLP-PR'};
colors=[118 124 130;48 134 107;36 117 174;222 138 50]/255;
old=get(groot,{'defaultAxesFontName','defaultTextFontName','defaultAxesFontSize','defaultTextFontSize'});
cleanup=onCleanup(@()set(groot,{'defaultAxesFontName','defaultTextFontName','defaultAxesFontSize','defaultTextFontSize'},old)); %#ok<NASGU>
set(groot,'defaultAxesFontName','Arial','defaultTextFontName','Arial', ...
    'defaultAxesFontSize',8,'defaultTextFontSize',8);

% Physical regularization: differences within each accepted task pair.
f=newfig(82); ax=gobjects(1,3); rng(20261024,'twister');
metrics={'R_soft','T_s','Eadj_J'}; scales=[1000 1000 1];
ylabs={'\Delta R_{soft} (10^{-3})','\Delta T_a (ms)','\Delta E_{adj} (J)'};
titles={'(a) Power excess','(b) Arrival time','(c) Source energy'};
prs={'Transformer-PR','MLP-PR'}; sups={'Transformer-SUP','MLP-SUP'};
pairCounts=zeros(1,2);
jitter=(rand(100,2)-.5)*.30;
for j=1:3
    ax(j)=axes(f,'Position',[.10+(j-1)*.315 .24 .235 .62]); hold(ax(j),'on'); prettify(ax(j));
    yline(ax(j),0,'--','Color',[.42 .45 .47],'LineWidth',.7);
    for k=1:2
        a=select(D.ablation_rows,prs{k},[], 'accepted'); b=select(D.ablation_rows,sups{k},[],'accepted');
        ia=values(a,'task_id'); ib=values(b,'task_id');
        [ids,ai,bi]=intersect(ia,ib); pairCounts(k)=numel(ids);
        va=values(a,'dummy',metrics{j}); vb=values(b,'dummy',metrics{j});
        v=(va(ai)-vb(bi))*scales(j);
        scatter(ax(j),k+jitter(1:numel(v),k),v,8,colors(k+2,:),'filled','MarkerFaceAlpha',.35,'MarkerEdgeAlpha',0);
        p=pair(D.ablation_paired,prs{k},sups{k}); q=p.metrics.(metrics{j});
        mu=q.difference*scales(j); ci=q.difference_CI95(:)*scales(j);
        errorbar(ax(j),k,mu,mu-ci(1),ci(2)-mu,'d','Color',colors(k+2,:), ...
            'MarkerFaceColor','w','MarkerSize',4.5,'LineWidth',1.25,'CapSize',6);
    end
    xlim(ax(j),[.5 2.5]); xticks(ax(j),[1 2]); xticklabels(ax(j),{'TRF','MLP'});
    ylabel(ax(j),ylabs{j},'Interpreter','tex'); title(ax(j),titles{j},'FontWeight','normal','FontSize',8.5);
end
annotation(f,'textbox',[.08 .92 .86 .055],'String',sprintf('PR minus SUP: %d TRF and %d MLP task pairs',pairCounts), ...
    'EdgeColor','none','HorizontalAlignment','center','FontSize',8.5,'FontName','Arial');
annotation(f,'textbox',[.04 .035 .94 .10],'String', ...
    'Dots: task differences    Diamonds and bars: mean and paired 95% interval', ...
    'EdgeColor','none','HorizontalAlignment','center','FontSize',7.5,'FontName','Arial');
savefigs(f,outDir,'ablation');

% Unknown perturbations: all scheduled executions remain in the denominator.
physical=zeros(7,4); accepted=physical; scheduled=physical;
for c=1:7
    for k=1:4
        r=select(D.robust_rows,methods{k},c,''); scheduled(c,k)=numel(r);
        physical(c,k)=sum(values(r,'physical_success')); accepted(c,k)=sum(values(r,'accepted'));
    end
end
f=newfig(89); cases={'Nominal','Low joint noise','High joint noise','Friction mismatch', ...
    'Payload / COM mismatch','Source-voltage mismatch','Combined perturbations'};
cmap=interp1([0 .5 1],[.95 .97 .98;.42 .67 .79;.086 .30 .45],linspace(0,1,256));
for j=1:2
    a=axes(f,'Position',[.31+(j-1)*.335 .26 .30 .62]);
    if j==1, count=physical; ttl='(a) Physical completion'; else, count=accepted; ttl='(b) Accepted completion'; end
    imagesc(a,100*count./scheduled,[0 100]); colormap(a,cmap); hold(a,'on');
    for r=1:7
        for c=1:4
            ink=[.09 .22 .30]; if count(r,c)/max(1,scheduled(r,c))>=2/3, ink=[1 1 1]; end
            text(a,c,r,sprintf('%d/%d',count(r,c),scheduled(r,c)),'HorizontalAlignment','center','Color',ink,'FontSize',8);
        end
    end
    for q=.5:1:4.5, xline(a,q,'w','LineWidth',1.1); end
    for q=.5:1:7.5, yline(a,q,'w','LineWidth',1.1); end
    xticks(a,1:4); xticklabels(a,labels); xtickangle(a,30); yticks(a,1:7);
    if j==1, yticklabels(a,cases); else, yticklabels(a,{}); end
    set(a,'TickLength',[0 0],'Box','off');
    title(a,ttl,'FontWeight','normal','FontSize',8.5);
end
apos=a.Position; cb=colorbar(a,'southoutside'); a.Position=apos;
cb.Position=[.31 .095 .635 .028]; cb.Ticks=[0 25 50 75 100];
cb.Label.String='Completion (%)'; cb.Label.FontSize=8; cb.FontSize=7.5; cb.Box='off';
savefigs(f,outDir,'operating_region');

% Known parameters and closed-loop filter: retain late physical completions.
pm=zeros(17,4,3); pn=zeros(17,4); pa=pn;
for c=1:17
    for k=1:4
        r=select(D.parameter_rows,methods{k},c,'physical_success');
        pn(c,k)=numel(r); pa(c,k)=sum(values(r,'accepted'));
        for j=1:3, pm(c,k,j)=mean(values(r,'dummy',metrics{j})); end
    end
end
f=newfig(143); starts=[1 2 4 6 8 10 12 14 16]; stops=[1 3 5 7 9 11 13 15 17];
ticks={'Nom.','.95','1.05','.95','1.05','.95','1.05','.95','1.05','.90','1.10','.90','1.10','11.7','12.3','25','100'};
groupnames={'','I_{stall}','I_{0,s}','\tau_{stall}','\omega_0','C_{dc}','R_s','V_s (V)','\tau_P (ms)'};
ylabs={'R_{soft} (10^{-3})','T_a (s)','E_{adj} (J)'};
titles={'(a) Soft-budget excess','(b) Arrival time','(c) Storage-adjusted source energy'};
scales=[1000 1 1]; markers={'o','s','^','d'}; hh=gobjects(1,4);
for j=1:3
    a=axes(f,'Position',[.105 .22+(3-j)*.235 .87 .18]); hold(a,'on'); prettify(a);
    for q=1.5:2:15.5, xline(a,q,'Color',[.86 .89 .90],'LineWidth',.5); end
    for k=1:4
        for g=1:numel(starts)
            ii=starts(g):stops(g);
            h=plot(a,ii+(k-2.5)*.065,pm(ii,k,j)*scales(j),['-' markers{k}], ...
                'Color',colors(k,:),'LineWidth',1,'MarkerSize',3.5,'MarkerFaceColor','w');
            if g==1, hh(k)=h; end
        end
    end
    xlim(a,[.5 17.5]); ylim(a,[0 inf]); ylabel(a,ylabs{j},'Interpreter','tex');
    tt=title(a,titles{j},'FontWeight','normal','FontSize',8.5,'HorizontalAlignment','left');
    tt.Units='normalized'; tt.Position(1)=0;
    xticks(a,1:17);
    if j==3
        xticklabels(a,ticks); a.XAxis.FontSize=7.5;
        for g=1:numel(starts)
            text(a,(mean([starts(g) stops(g)])-.5)/17,-.28,groupnames{g},'Units','normalized', ...
                'Interpreter','tex','HorizontalAlignment','center','VerticalAlignment','top','FontSize',8);
        end
    else, xticklabels(a,{}); end
end
lg=legend(hh,labels,'Orientation','horizontal','Box','off','FontSize',8);
lg.Position=[.23 .935 .65 .035];
annotation(f,'textbox',[.08 .052 .9 .055],'String', ...
    'Multipliers of nominal values; voltage and filter time shown in stated units', ...
    'EdgeColor','none','HorizontalAlignment','center','FontSize',7.5,'FontName','Arial');
savefigs(f,outDir,'parameter_sensitivity');

% Observation-only sensitivity on accepted core-method trajectories.
taus=F.taus_s(:)'; epsilons=F.epsilons(:)'; fm=zeros(4,numel(taus)); fi=zeros(4,numel(epsilons));
for k=1:4
    for j=1:numel(taus)
        s=filterSummary(F.summary,methods{k},taus(j),.05);
        fm(k,j)=s.R_soft_mean;
    end
    for j=1:numel(epsilons)
        s=filterSummary(F.summary,methods{k},.05,epsilons(j));
        fi(k,j)=s.I_epsilon_mean_s;
    end
end
f=newfig(82); h=gobjects(1,4);
a=axes(f,'Position',[.115 .28 .365 .57]); hold(a,'on'); prettify(a);
for k=1:4, h(k)=plot(a,taus*1000,fm(k,:)*1000,['-' markers{k}],'Color',colors(k,:),'LineWidth',1.2,'MarkerSize',4,'MarkerFaceColor','w'); end
set(a,'XScale','log','XMinorTick','off'); xticks(a,taus*1000); xticklabels(a,compose('%g',taus*1000)); ylim(a,[0 inf]);
xlabel(a,'Filter time constant (ms)'); ylabel(a,'Mean R_{soft} (10^{-3})','Interpreter','tex');
title(a,'(a) Fixed-trajectory filtering','FontWeight','normal','FontSize',8.5);
b=axes(f,'Position',[.61 .28 .365 .57]); hold(b,'on'); prettify(b);
for k=1:4, plot(b,epsilons*100,fi(k,:),['-' markers{k}],'Color',colors(k,:),'LineWidth',1.2,'MarkerSize',4,'MarkerFaceColor','w'); end
xticks(b,epsilons*100); ylim(b,[0 inf]); xlabel(b,'Exceedance tolerance \epsilon (%)','Interpreter','tex');
ylabel(b,'Mean I_{\epsilon} (s)','Interpreter','tex'); title(b,'(b) Tolerance at 50 ms','FontWeight','normal','FontSize',8.5);
lg=legend(h,labels,'Orientation','horizontal','Box','off','FontSize',8); lg.Position=[.22 .055 .67 .07];
savefigs(f,outDir,'filter_metrics');
fprintf('Supplementary MATLAB figures exported to %s\n',outDir);
end

function f=newfig(h)
f=figure('Visible','off','Color','w','Units','centimeters','Position',[2 2 16 h/10], ...
    'PaperUnits','centimeters','PaperPosition',[0 0 16 h/10],'PaperSize',[16 h/10]);
end
function prettify(a)
set(a,'Box','off','LineWidth',.65,'TickDir','out','Layer','top','GridColor',[.85 .88 .90], ...
    'GridAlpha',.65,'YGrid','on','XGrid','off','FontName','Arial','FontSize',8);
end
function savefigs(f,outDir,name)
drawnow; exportgraphics(f,fullfile(outDir,[name '.pdf']),'ContentType','vector','BackgroundColor','white');
exportgraphics(f,fullfile(outDir,[name '.png']),'Resolution',300,'BackgroundColor','white');
savefig(f,fullfile(outDir,[name '.fig'])); close(f);
end
function r=ascells(r)
if ~iscell(r), r=num2cell(r); end
r=r(:);
end
function out=select(rows,method,cid,criterion)
rows=ascells(rows); keep=false(size(rows));
for i=1:numel(rows)
    r=rows{i}; ok=strcmp(r.method,method);
    if ~isempty(cid), ok=ok && r.case_id==cid; end
    if ~isempty(criterion), ok=ok && logical(r.(criterion)); end
    keep(i)=ok;
end
out=rows(keep);
end
function v=values(rows,key,varargin)
if nargin>2, key=varargin{1}; end
rows=ascells(rows); v=cellfun(@(r)double(r.(key)),rows);
end
function out=pair(rows,method,ref)
rows=ascells(rows); out=[];
for i=1:numel(rows), r=rows{i}; if strcmp(r.method,method)&&strcmp(r.reference,ref), out=r; break; end, end
end
function out=filterSummary(rows,method,tau,eps)
rows=ascells(rows); out=[];
for i=1:numel(rows)
    r=rows{i}; if strcmp(r.method,method)&&abs(r.tau_s-tau)<1e-10&&abs(r.epsilon-eps)<1e-10, out=r; break; end
end
end
