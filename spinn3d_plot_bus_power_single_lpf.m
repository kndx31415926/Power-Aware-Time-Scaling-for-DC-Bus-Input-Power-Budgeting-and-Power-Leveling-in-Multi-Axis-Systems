function Plpf = spinn3d_plot_bus_power_single_lpf(ax, t, Pgrid, Pdump, Pbudget, cfg, varargin)
%SPINN3D_PLOT_BUS_POWER_SINGLE_LPF Plot raw and one causal LPF bus-power trace.
% v5 convention: one academic first-order LPF is used for the paper metric:
% Pbar[k] = lambda Pbar[k-1] + (1-lambda) Pgrid[k], lambda=exp(-Ts/tauP).

opts = parse_opts(varargin{:});
t = double(t(:));
Pgrid = double(Pgrid(:));
if nargin < 4 || isempty(Pdump), Pdump = zeros(size(Pgrid)); else, Pdump = double(Pdump(:)); end
if nargin < 5 || isempty(Pbudget), Pbudget = NaN; end
if nargin < 6 || isempty(cfg), cfg = struct(); end
Ts = getfield_def(cfg,'TS',[]);
if isempty(Ts) || ~isfinite(Ts) || Ts <= 0
    if numel(t) > 1, Ts = median(diff(t)); else, Ts = 1e-3; end
end
tauP = spinn3d_get_power_lpf_tau(cfg, 0.05);
if isempty(Pgrid)
    Plpf = Pgrid; return;
end
Plpf = spinn3d_lpf1_causal(Pgrid, Ts, tauP, Pgrid(1));

hold(ax,'on'); grid(ax,'on');
hRaw = plot(ax, t, Pgrid, 'LineWidth',0.8, 'DisplayName','P_{grid} raw');
hRaw.Color = 0.70*[1 1 1];
if ~isempty(Pdump)
    hDump = plot(ax, t, Pdump, 'LineWidth',0.8, 'DisplayName','P_{dump}');
    hDump.Color = 0.85*[1 1 1];
end
plot(ax, t, Plpf, 'k-', 'LineWidth',opts.lpfLineWidth, ...
    'DisplayName',sprintf('P_{grid} LPF (\\tau=%.2fs)', tauP));
if isfinite(Pbudget)
    yline(ax, Pbudget, '--r', 'P_{grid,budget}', 'DisplayName','P_{grid,budget}');
end
allP = [Pgrid(:); Pdump(:); Plpf(:); Pbudget(:)];
ymax = max(allP(isfinite(allP)))*1.15;
if isempty(ymax) || ~isfinite(ymax) || ymax <= 0, ymax = 1; end
ylim(ax,[0,ymax]);
if ~isempty(t), xlim(ax,[t(1), t(end)]); end
xlabel(ax,'t [s]'); ylabel(ax,'Power [W]');
title(ax,opts.titleText);
legend(ax,'Location','best');
end

function opts = parse_opts(varargin)
opts = struct('titleText','BUS Power','lpfLineWidth',1.8);
i = 1;
while i <= numel(varargin)
    if ischar(varargin{i}) || isstring(varargin{i})
        key = char(varargin{i});
        if i < numel(varargin)
            opts.(key) = varargin{i+1};
        end
        i = i + 2;
    else
        i = i + 1;
    end
end
end
function v = getfield_def(S,fn,def)
if isstruct(S) && isfield(S,fn) && ~isempty(S.(fn)), v = S.(fn); else, v = def; end
end
