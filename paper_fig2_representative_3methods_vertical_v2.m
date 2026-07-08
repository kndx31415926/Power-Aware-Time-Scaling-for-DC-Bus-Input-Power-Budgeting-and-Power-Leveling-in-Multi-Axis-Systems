function fig = paper_fig2_representative_3methods_vertical_v2(cfg, opts)
%PAPER_FIG2_REPRESENTATIVE_3METHODS_VERTICAL_V2
% Create a manuscript-ready representative Figure 2 with three methods arranged
% from top to bottom:
%   Row 1: Baseline / NoGuard
%   Row 2: TRF-PRG
%   Row 3: MLP-PRG
%
% Each row contains three panels:
%   Column 1: end-effector trajectory + initial/final robot skeleton
%   Column 2: DC-bus input power, dump power, budget line, and LPF power
%   Column 3: per-joint mechanical power
%
% Put this file in the same folder as the project .m files, e.g. MathSource78
% after extraction, and make sure the trained models are available in data_alpha/.
%
% Usage:
%   addpath(genpath(pwd));
%   fig = paper_fig2_representative_3methods_vertical_v2;
%
% Optional usage:
%   PB  = spinn3d_params_block();
%   cfg = spinn3d_params_to_cfg(PB, 'demo_alpha');
%   opts.save_png = 'Figure02_representative_3methods_vertical.png';
%   opts.save_fig = 'Figure02_representative_3methods_vertical.fig';
%   fig = paper_fig2_representative_3methods_vertical_v2(cfg, opts);
%
% Main options:
%   opts.rng_seed               default 1; set [] to avoid resetting RNG
%   opts.force_budget_W         default 13; set [] to keep cfg budget unchanged
%   opts.pgrid_tau_s            default 0.05 s
%   opts.save_png               default 'Figure02_representative_3methods_vertical.png'
%   opts.save_fig               default 'Figure02_representative_3methods_vertical.fig'
%   opts.close_demo_figs        default true
%   opts.bus_ylim_focus_lpf     default true; ignore raw spikes for bus y-limit
%   opts.joint_ylim_focus_quant default []; e.g. 99.5 to clip joint-power outliers
%   opts.show_repeated_legends  default false; legends only in the top row
%   opts.save_dpi               default 300; PNG export resolution
%   opts.figure_visible         default 'on'; use 'off' for batch RUN04
%
% Dependencies used from the original code base:
%   spinn3d_params_block, spinn3d_params_to_cfg,
%   spinn3d_demo_ctcgov_online_nonn_nolimit,
%   spinn3d_demo_ctcgov_online_nn,
%   spinn3d_demo_ctcgov_online_mlp,
%   spinn3d_lpf1_causal.

%% ---------------- defaults ----------------
if nargin < 2 || isempty(opts), opts = struct(); end
rng_seed               = get_opt(opts, 'rng_seed', 1);
force_budget_W         = get_opt(opts, 'force_budget_W', 13);
pgrid_tau_s            = get_opt(opts, 'pgrid_tau_s', 0.05);
save_png               = string(get_opt(opts, 'save_png', 'Figure02_representative_3methods_vertical.png'));
save_fig               = string(get_opt(opts, 'save_fig', 'Figure02_representative_3methods_vertical.fig'));
close_demo_figs         = get_opt(opts, 'close_demo_figs', true);
bus_ylim_focus_lpf      = get_opt(opts, 'bus_ylim_focus_lpf', true);
bus_ylim_margin         = get_opt(opts, 'bus_ylim_margin', 0.18);
joint_ylim_focus_quant  = get_opt(opts, 'joint_ylim_focus_quant', []);
joint_ylim_margin       = get_opt(opts, 'joint_ylim_margin', 0.10);
show_repeated_legends   = get_opt(opts, 'show_repeated_legends', false);
fig_size_cm             = get_opt(opts, 'fig_size_cm', [2 2 23 24]);
fig_name                = string(get_opt(opts, 'fig_name', 'Figure02_representative_3methods_vertical'));
save_dpi                = get_opt(opts, 'save_dpi', 300);
figure_visible          = char(get_opt(opts, 'figure_visible', 'on'));

%% ---------------- build cfg when not provided ----------------
if nargin < 1 || isempty(cfg)
    must_exist('spinn3d_params_block');
    must_exist('spinn3d_params_to_cfg');
    PB = spinn3d_params_block();
    cfg = spinn3d_params_to_cfg(PB, 'demo_alpha');
end

% Use the 13 W nominal case for the revised manuscript unless disabled.
if ~isempty(force_budget_W) && isnumeric(force_budget_W) && isscalar(force_budget_W)
    cfg = set_active_budget(cfg, double(force_budget_W));
end

assert(isfield(cfg,'TS') && isnumeric(cfg.TS) && isscalar(cfg.TS), 'cfg.TS missing/invalid.');
assert(isfield(cfg,'ROBOT') && ~isempty(cfg.ROBOT), 'cfg.ROBOT missing.');

robot = cfg.ROBOT;
try, robot.DataFormat = 'row'; catch, end
if isfield(cfg,'EE_NAME') && ~isempty(cfg.EE_NAME)
    eeName = cfg.EE_NAME;
else
    eeName = robot.BodyNames{end};
end

q0 = deg2rad_local(row(cfg.Q0_DEG));
qf = deg2rad_local(row(cfg.QF_DEG));
qf = unwrap_to_near(q0, qf);
ee_start = ee_pos(robot, q0, eeName);
ee_goal  = ee_pos(robot, qf, eeName);

Pbud = get_power_budget(cfg);

%% ---------------- run the three methods on the same representative case ----------------
% Reset RNG before each run so stochastic noise, if enabled, is matched across methods.
% Use a cell array rather than a MATLAB struct array here.  The demo
% routines return nested output/log structures with method-specific fields;
% a cell array avoids "Subscripted assignment between dissimilar structures"
% on some MATLAB releases.
runs = cell(1,3);
runs{1} = run_one_method('Baseline', 'NoGuard', @run_baseline, cfg, rng_seed, close_demo_figs);
runs{2} = run_one_method('TRF-PRG',  'TRF',     @run_trf,      cfg, rng_seed, close_demo_figs);
runs{3} = run_one_method('MLP-PRG',  'MLP',     @run_mlp,      cfg, rng_seed, close_demo_figs);

% Extract signals and LPF power.
for i = 1:numel(runs)
    R = runs{i};
    R.t      = vec(R.out.log.t);
    R.Pgrid  = vec(getfield_def(R.out.log, 'Pgrid', []));
    R.Pdump  = vec(getfield_def(R.out.log, 'Pdump', zeros(size(R.Pgrid))));
    R.Pj     = getfield_def(R.out.log, 'Pi_raw', []);
    R.q_end  = row(R.out.log.q(end,:));
    R.ee     = getfield_def(R.out.log, 'ee', []);
    if isempty(R.ee)
        R.ee = compute_ee_trace(robot, R.out.log.q, eeName);
    end
    R.Pgrid_lpf = lpf1_power(R.Pgrid, cfg.TS, pgrid_tau_s);
    runs{i} = R;
end

%% ---------------- common axis limits ----------------
[skel_xlim, skel_ylim, skel_zlim] = skeleton_limits(robot, eeName, q0, qf, runs, ee_start, ee_goal);
bus_ylim = common_bus_ylim(runs, Pbud, bus_ylim_focus_lpf, bus_ylim_margin);
joint_ylim = common_joint_ylim(runs, joint_ylim_focus_quant, joint_ylim_margin);

%% ---------------- plot ----------------
fig = figure('Name', fig_name, 'Color', 'w', 'Units', 'centimeters', 'Position', fig_size_cm, 'Visible', figure_visible);
tlo = tiledlayout(fig, 3, 3, 'Padding', 'compact', 'TileSpacing', 'compact');

for r = 1:3
    % ---- Col 1: trajectory + skeleton ----
    ax1 = nexttile(tlo, (r-1)*3 + 1);
    hold(ax1, 'on'); grid(ax1, 'on'); axis(ax1, 'equal');
    plot_skeleton(ax1, robot, eeName, q0, runs{r}.q_end, runs{r}.ee, ee_start, ee_goal, r==1 || show_repeated_legends);
    title(ax1, sprintf('(%c) %s: EE trajectory + skeleton', char('a'+r-1), runs{r}.label), 'Interpreter', 'none');
    xlim(ax1, skel_xlim); ylim(ax1, skel_ylim); zlim(ax1, skel_zlim);
    view(ax1, 135, 20);

    % ---- Col 2: DC-bus power ----
    ax2 = nexttile(tlo, (r-1)*3 + 2);
    hold(ax2, 'on'); grid(ax2, 'on');
    plot_bus(ax2, runs{r}.t, runs{r}.Pgrid, runs{r}.Pdump, runs{r}.Pgrid_lpf, Pbud, pgrid_tau_s, r==1 || show_repeated_legends);
    title(ax2, sprintf('%s: DC-bus input power', runs{r}.label), 'Interpreter', 'none');
    xlabel(ax2, 't [s]'); ylabel(ax2, 'Power [W]');
    ylim(ax2, bus_ylim);

    % ---- Col 3: per-joint mechanical power ----
    ax3 = nexttile(tlo, (r-1)*3 + 3);
    hold(ax3, 'on'); grid(ax3, 'on');
    plot_joint_power(ax3, runs{r}.t, runs{r}.Pj, r==1 || show_repeated_legends);
    title(ax3, sprintf('%s: per-joint mechanical power', runs{r}.label), 'Interpreter', 'none');
    xlabel(ax3, 't [s]'); ylabel(ax3, 'P_j [W]');
    if ~isempty(joint_ylim), ylim(ax3, joint_ylim); end
end

sgtitle(tlo, 'Representative trial: Baseline, TRF-PRG, and MLP-PRG under the same 13 W active budget');

% Font and line consistency.
axs = findall(fig, 'Type', 'axes');
set(axs, 'FontName', 'Times New Roman', 'FontSize', 8, 'LineWidth', 0.75);

%% ---------------- save ----------------
if strlength(save_png) > 0
    exportgraphics(fig, save_png, 'Resolution', save_dpi);
    fprintf('[Fig2-3methods] Saved PNG: %s\n', save_png);
end
if strlength(save_fig) > 0
    savefig(fig, save_fig);
    fprintf('[Fig2-3methods] Saved FIG: %s\n', save_fig);
end

end

%% ========================================================================
%  Method wrappers
% ========================================================================
function [out, figs] = run_baseline(cfg)
    must_exist('spinn3d_demo_ctcgov_online_nonn_nolimit');
    figs_before = findall(0, 'Type', 'figure');
    [out, fig0] = spinn3d_demo_ctcgov_online_nonn_nolimit(cfg);
    figs_after = findall(0, 'Type', 'figure');
    figs = unique([fig0; setdiff(figs_after, figs_before)]);
end

function [out, figs] = run_trf(cfg)
    must_exist('spinn3d_demo_ctcgov_online_nn');
    figs_before = findall(0, 'Type', 'figure');
    out = spinn3d_demo_ctcgov_online_nn(cfg);
    figs_after = findall(0, 'Type', 'figure');
    figs = setdiff(figs_after, figs_before);
end

function [out, figs] = run_mlp(cfg)
    must_exist('spinn3d_demo_ctcgov_online_mlp');
    figs_before = findall(0, 'Type', 'figure');
    out = spinn3d_demo_ctcgov_online_mlp(cfg);
    figs_after = findall(0, 'Type', 'figure');
    figs = setdiff(figs_after, figs_before);
end

function S = run_one_method(label, short_label, runner, cfg, rng_seed, close_demo_figs)
    if ~isempty(rng_seed), rng(rng_seed); end
    [out, figs] = runner(cfg);
    req_struct(out, {'log'});
    req_struct(out.log, {'t','q'});
    if close_demo_figs
        safe_close_list(figs);
    end
    S = struct('label', label, 'short_label', short_label, 'out', out);
end

%% ========================================================================
%  Plot helpers
% ========================================================================
function plot_skeleton(ax, robot, eeName, q0, q_end, ee_traj, ee_start, ee_goal, show_legend)
    [X0,Y0,Z0] = chain_skeleton_xyz(robot, q0, eeName);
    if ~isempty(X0)
        plot3(ax, X0, Y0, Z0, '-o', 'LineWidth', 1.0, 'MarkerSize', 4, 'DisplayName', 'Initial');
    end

    [Xf,Yf,Zf] = chain_skeleton_xyz(robot, q_end, eeName);
    if ~isempty(Xf)
        plot3(ax, Xf, Yf, Zf, 'm-o', 'MarkerFaceColor', 'm', 'LineWidth', 1.0, 'MarkerSize', 4, 'DisplayName', 'Final');
    end

    if ~isempty(ee_traj) && size(ee_traj,2) >= 3
        plot3(ax, ee_traj(:,1), ee_traj(:,2), ee_traj(:,3), 'c-', 'LineWidth', 1.4, 'DisplayName', 'EE traj');
        plot3(ax, ee_traj(1,1), ee_traj(1,2), ee_traj(1,3), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 5, 'DisplayName', 'Start');
        plot3(ax, ee_traj(end,1), ee_traj(end,2), ee_traj(end,3), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 5, 'DisplayName', 'Final EE');
    else
        plot3(ax, ee_start(1), ee_start(2), ee_start(3), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 5, 'DisplayName', 'Start');
    end

    plot3(ax, ee_goal(1), ee_goal(2), ee_goal(3), 'co', 'MarkerFaceColor', 'c', 'MarkerSize', 5, 'DisplayName', 'Target');
    xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]'); zlabel(ax, 'Z [m]');
    if show_legend
        legend(ax, 'Location', 'best', 'FontSize', 7);
    end
end

function plot_bus(ax, t, Pgrid, Pdump, Pfilt, Pbud, tauP, show_legend)
    h1 = plot(ax, t, Pgrid, 'LineWidth', 0.55, 'DisplayName', 'P_{grid} raw');
    h1.Color = 0.72 * [1 1 1];
    h2 = plot(ax, t, Pdump, 'LineWidth', 0.55, 'DisplayName', 'P_{dump}');
    h2.Color = 0.86 * [1 1 1];

    if isfinite(Pbud)
        plot(ax, [t(1), t(end)], [Pbud, Pbud], 'r--', 'LineWidth', 1.0, 'DisplayName', 'P_{grid,budget}');
    end

    hF = plot(ax, t, Pfilt, 'k-', 'LineWidth', 1.8, ...
        'DisplayName', sprintf('P_{grid} LPF (\\tau=%.2fs)', tauP));
    try, uistack(hF, 'top'); catch, end

    if ~isempty(t), xlim(ax, [t(1), t(end)]); end
    if show_legend
        legend(ax, 'Location', 'best', 'FontSize', 7);
    end
end

function plot_joint_power(ax, t, Pj, show_legend)
    if isempty(Pj) || isempty(t), return; end
    if size(Pj,1) ~= numel(t) && size(Pj,2) == numel(t)
        Pj = Pj.';
    end
    nJ = size(Pj,2);
    for j = 1:nJ
        plot(ax, t, Pj(:,j), 'LineWidth', 0.8, 'DisplayName', sprintf('J%d', j));
    end
    if ~isempty(t), xlim(ax, [t(1), t(end)]); end
    if show_legend
        legend(ax, 'Location', 'best', 'FontSize', 7);
    end
end

%% ========================================================================
%  Axis-limit helpers
% ========================================================================
function [xl, yl, zl] = skeleton_limits(robot, eeName, q0, qf, runs, ee_start, ee_goal)
    pts = [ee_start(:)'; ee_goal(:)'];
    for i = 1:numel(runs)
        if isfield(runs{i}, 'ee') && ~isempty(runs{i}.ee) && size(runs{i}.ee,2) >= 3
            pts = [pts; runs{i}.ee(:,1:3)]; %#ok<AGROW>
        end
        [X0,Y0,Z0] = chain_skeleton_xyz(robot, q0, eeName);
        [Xf,Yf,Zf] = chain_skeleton_xyz(robot, runs{i}.q_end, eeName);
        pts = [pts; [X0(:),Y0(:),Z0(:)]; [Xf(:),Yf(:),Zf(:)]]; %#ok<AGROW>
    end
    pts = pts(all(isfinite(pts),2),:);
    if isempty(pts), pts = zeros(2,3); end
    pad = 0.08;
    xl = padded_lim(pts(:,1), pad);
    yl = padded_lim(pts(:,2), pad);
    zl = padded_lim(pts(:,3), pad);
end

function lim = common_bus_ylim(runs, Pbud, focus_lpf, margin)
    vals = [];
    for i = 1:numel(runs)
        if focus_lpf
            vals = [vals; runs{i}.Pgrid_lpf(:)]; %#ok<AGROW>
        else
            vals = [vals; runs{i}.Pgrid(:); runs{i}.Pdump(:); runs{i}.Pgrid_lpf(:)]; %#ok<AGROW>
        end
    end
    vals = vals(isfinite(vals));
    if isempty(vals), ymax = 1; else, ymax = max(vals); end
    if isfinite(Pbud), ymax = max(ymax, Pbud); end
    ymax = max(ymax, 1);
    lim = [0, (1+margin)*ymax];
end

function lim = common_joint_ylim(runs, qprc, margin)
    vals = [];
    for i = 1:numel(runs)
        Pj = runs{i}.Pj;
        vals = [vals; Pj(:)]; %#ok<AGROW>
    end
    vals = vals(isfinite(vals));
    if isempty(vals)
        lim = [];
        return;
    end
    if ~isempty(qprc) && isnumeric(qprc) && isscalar(qprc) && qprc > 0 && qprc < 100
        lo = prctile(vals, 100-qprc);
        hi = prctile(vals, qprc);
        maxabs = max(abs([lo hi]));
    else
        maxabs = max(abs(vals));
    end
    if maxabs <= 0 || ~isfinite(maxabs), maxabs = 1; end
    maxabs = (1+margin) * maxabs;
    lim = [-maxabs, maxabs];
end

function lim = padded_lim(x, pad_frac)
    x = x(isfinite(x));
    if isempty(x), lim = [-1,1]; return; end
    xmin = min(x); xmax = max(x);
    if abs(xmax-xmin) < 1e-9
        lim = [xmin-0.1, xmax+0.1];
    else
        pad = pad_frac * (xmax - xmin);
        lim = [xmin-pad, xmax+pad];
    end
end

%% ========================================================================
%  Data and math helpers
% ========================================================================
function y = lpf1_power(x, Ts, tau)
    x = vec(x);
    if isempty(x), y = x; return; end
    if exist('spinn3d_lpf1_causal', 'file') == 2
        y = spinn3d_lpf1_causal(x, Ts, tau, x(1));
    else
        a = exp(-Ts/tau);
        y = zeros(size(x));
        y(1) = x(1);
        for k = 2:numel(x)
            y(k) = a*y(k-1) + (1-a)*x(k);
        end
    end
end

function ee = compute_ee_trace(robot, qlog, eeName)
    ee = zeros(size(qlog,1), 3);
    for k = 1:size(qlog,1)
        ee(k,:) = ee_pos(robot, row(qlog(k,:)), eeName);
    end
end

function p = ee_pos(robot, q, eeName)
    p = tform2trvec_local(getTransform(robot, row(q), eeName));
end

function [X,Y,Z] = chain_skeleton_xyz(robot, q, eeName)
    X = []; Y = []; Z = [];
    try
        b = robot.getBody(eeName);
        names = {};
        while ~strcmp(b.Name, robot.BaseName)
            names{end+1} = b.Name; %#ok<AGROW>
            b = robot.getBody(b.Parent);
        end
        names = fliplr(names);
    catch
        names = robot.BodyNames;
    end

    for i = 1:numel(names)
        try
            Tb = getTransform(robot, row(q), names{i});
            p = tform2trvec_local(Tb);
            X(end+1) = p(1); %#ok<AGROW>
            Y(end+1) = p(2); %#ok<AGROW>
            Z(end+1) = p(3); %#ok<AGROW>
        catch
        end
    end

    try
        Tee = getTransform(robot, row(q), eeName);
        p = tform2trvec_local(Tee);
        if isempty(X) || norm([X(end)-p(1), Y(end)-p(2), Z(end)-p(3)]) > 1e-12
            X(end+1) = p(1); %#ok<AGROW>
            Y(end+1) = p(2); %#ok<AGROW>
            Z(end+1) = p(3); %#ok<AGROW>
        end
    catch
    end
end


function p = tform2trvec_local(T)
    % Robust replacement for Robotics System Toolbox tform2trvec.
    % Works for standard 4x4 homogeneous transforms.
    if exist('tform2trvec', 'file') == 2
        try
            p = tform2trvec(T);
            return;
        catch
        end
    end
    p = row(T(1:3,4));
end

function cfg = set_active_budget(cfg, Pbud)
    cfg.P_GRID_BUDGET = Pbud;
    cfg.P_TOTAL_MAX = Pbud; % legacy alias used by some routines
    if isfield(cfg, 'CAPS') && isstruct(cfg.CAPS)
        cfg.CAPS.P_grid_budget = Pbud;
        cfg.CAPS.P_total_max = Pbud;
    end
    if isfield(cfg, 'BUS') && isstruct(cfg.BUS)
        cfg.BUS.P_grid_budget = Pbud;
        cfg.BUS.P_total_max = Pbud;
        if isfield(cfg.BUS,'V_nom') && isfinite(cfg.BUS.V_nom) && cfg.BUS.V_nom > 0
            cfg.BUS.I_budget = Pbud / cfg.BUS.V_nom;
        end
    end
end

function Pbud = get_power_budget(cfg)
    Pbud = NaN;
    if isfield(cfg,'P_GRID_BUDGET') && isnumeric(cfg.P_GRID_BUDGET) && isscalar(cfg.P_GRID_BUDGET)
        Pbud = double(cfg.P_GRID_BUDGET);
    elseif isfield(cfg,'P_TOTAL_MAX') && isnumeric(cfg.P_TOTAL_MAX) && isscalar(cfg.P_TOTAL_MAX)
        Pbud = double(cfg.P_TOTAL_MAX);
    elseif isfield(cfg,'CAPS') && isstruct(cfg.CAPS) && isfield(cfg.CAPS,'P_grid_budget')
        Pbud = double(cfg.CAPS.P_grid_budget);
    end
end

function v = get_opt(opts, name, default)
    if ~isstruct(opts) || ~isfield(opts, name) || isempty(opts.(name))
        v = default;
    else
        v = opts.(name);
    end
end

function v = getfield_def(S, name, default)
    if isstruct(S) && isfield(S, name) && ~isempty(S.(name))
        v = S.(name);
    else
        v = default;
    end
end

function req_struct(S, fns)
    for i = 1:numel(fns)
        assert(isstruct(S) && isfield(S, fns{i}), 'Missing field: %s', fns{i});
    end
end

function must_exist(fn)
    assert(exist(fn, 'file') == 2, 'Required function not found on MATLAB path: %s', fn);
end

function safe_close_list(hs)
    try
        for k = 1:numel(hs)
            if ~isempty(hs(k)) && isvalid(hs(k))
                close(hs(k));
            end
        end
    catch
    end
end

function r = row(v)
    r = v(:).';
end

function v = vec(v)
    v = v(:);
end

function x = deg2rad_local(d)
    x = pi/180 * row(d);
end

function qf = unwrap_to_near(q0, qf)
    d = qf - q0;
    qf = q0 + atan2(sin(d), cos(d));
end
