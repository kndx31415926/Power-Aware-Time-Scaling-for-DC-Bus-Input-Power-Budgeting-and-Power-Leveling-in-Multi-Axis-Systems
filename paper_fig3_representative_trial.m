function fig = paper_fig3_representative_trial(cfg, opts)
%PAPER_FIG3_REPRESENTATIVE_TRIAL  Paper-ready representative trial figure.
%
% Generates a Baseline vs TRF comparison and (optionally) includes the
% end-effector trajectory + robot skeleton schematic for BOTH methods.
%
% IMPORTANT FIX:
%   spinn3d_demo_ctcgov_online_nn returns ONLY ONE output (out). This script
%   calls it with ONE output to avoid "Too many output arguments".
%
% Usage:
%   paper_fig3_representative_trial
%   fig = paper_fig3_representative_trial(cfg)
%   fig = paper_fig3_representative_trial(cfg, opts)
%
% opts (optional):
%   opts.rng_seed            : integer, default 1 (set [] to skip rng)
%   opts.pgrid_tau_s         : causal LPF time constant, default 0.05 s
%   opts.bus_ylim_focus      : true clips raw spikes to highlight avg, default true
%   opts.bus_ylim_margin     : upper y margin fraction, default 0.20
%   opts.close_demo_figs     : close figures created by demos, default true
%   opts.save_png            : "" (no save) or filename, e.g. "Fig3.png"
%   opts.save_skeleton_png   : "" (no save) or filename for 1x2 skeleton compare
%   opts.fig_name            : figure name, default "Fig3_representative_trial"
%   opts.show_budget_line    : true draws Pmax line, default true
%   opts.include_skeleton    : true includes skeleton row, default true
%   opts.include_joint_power : true includes joint-power row, default true

% ---------------- defaults ----------------
if nargin < 2 || isempty(opts), opts = struct(); end
rng_seed            = get_opt(opts,'rng_seed',1);
pgrid_tau_s         = get_opt(opts,'pgrid_tau_s', get_opt(opts,'pgrid_fast_tau_s', get_opt(opts,'pgrid_slow_tau_s',0.05))); % single LPF
bus_ylim_focus      = get_opt(opts,'bus_ylim_focus',true);
bus_ylim_margin     = get_opt(opts,'bus_ylim_margin',0.20);
close_demo_figs     = get_opt(opts,'close_demo_figs',true);
save_png            = string(get_opt(opts,'save_png',""));
save_skeleton_png   = string(get_opt(opts,'save_skeleton_png',""));
fig_name            = string(get_opt(opts,'fig_name',"Fig3_representative_trial"));
show_budget_line    = get_opt(opts,'show_budget_line',true);
include_skeleton    = get_opt(opts,'include_skeleton',true);
include_joint_power = get_opt(opts,'include_joint_power',true);

% ---------------- cfg ----------------
if nargin < 1 || isempty(cfg)
    blk = [];
    if exist('spinn3d_params_block','file')==2
        blk = spinn3d_params_block();
    elseif evalin('base',"exist(''spinn3d_params_block'',''var'')==1")
        blk = evalin('base','spinn3d_params_block');
    end
    assert(~isempty(blk), '[Fig3] No cfg provided and spinn3d_params_block not found.');
    cfg = spinn3d_params_to_cfg(blk, 'demo_alpha');
end

assert(isfield(cfg,'TS') && isnumeric(cfg.TS) && isscalar(cfg.TS), '[Fig3] cfg.TS missing/invalid.');
assert(isfield(cfg,'ROBOT') && ~isempty(cfg.ROBOT), '[Fig3] cfg.ROBOT missing.');

robot = cfg.ROBOT; try, robot.DataFormat='row'; end
eeName = get_ee_name(cfg, robot);

% initial/goal configs (for skeleton markers)
q0 = deg2rad_local(row(cfg.Q0_DEG));
qf = deg2rad_local(row(cfg.QF_DEG));
qf = unwrap_to_near(q0, qf);
ee_start = ee_pos(robot, q0, eeName);
ee_goal  = ee_pos(robot, qf, eeName);

% ---------------- run Baseline ----------------
must_exist('spinn3d_demo_ctcgov_online_nonn_nolimit');
if ~isempty(rng_seed), rng(rng_seed); end
figs_before = findall(0,'Type','figure');
[outB, figB] = spinn3d_demo_ctcgov_online_nonn_nolimit(cfg);
figs_after  = findall(0,'Type','figure');
new_figs_B  = setdiff(figs_after, figs_before);

% ---------------- run TRF (ONE output!) ----------------
must_exist('spinn3d_demo_ctcgov_online_nn');
if ~isempty(rng_seed), rng(rng_seed); end
figs_before = findall(0,'Type','figure');
outT = spinn3d_demo_ctcgov_online_nn(cfg);  % <-- ONE output
figs_after  = findall(0,'Type','figure');
new_figs_T  = setdiff(figs_after, figs_before);

% optionally close demo figures (we redraw our own)
if close_demo_figs
    safe_close(figB);
    safe_close_list(new_figs_B);
    safe_close_list(new_figs_T);
end

% ---------------- extract signals ----------------
req_struct(outB, {'log'});
req_struct(outT, {'log'});
req_struct(outB.log, {'t','Pi_raw','Pgrid','Pdump','q','ee'});
req_struct(outT.log, {'t','Pi_raw','Pgrid','Pdump','q','ee'});

tB = outB.log.t(:);
PBj = outB.log.Pi_raw;
qB_end = row(outB.log.q(end,:));
eeB = outB.log.ee;

PgridB = outB.log.Pgrid(:);
PdumpB = outB.log.Pdump(:);

tT = outT.log.t(:);
PTj = outT.log.Pi_raw;
qT_end = row(outT.log.q(end,:));
eeT = outT.log.ee;

PgridT = outT.log.Pgrid(:);
PdumpT = outT.log.Pdump(:);

% Single causal LPF for plotting and paper metrics.
PgridB_lpf = spinn3d_lpf1_causal(PgridB, cfg.TS, pgrid_tau_s, PgridB(1));
PgridT_lpf = spinn3d_lpf1_causal(PgridT, cfg.TS, pgrid_tau_s, PgridT(1));

% power budget line (cfg first; fall back to outB.limits if present)
Pmax = NaN;
if isfield(cfg,'P_GRID_BUDGET') && isnumeric(cfg.P_GRID_BUDGET) && isscalar(cfg.P_GRID_BUDGET)
    Pmax = double(cfg.P_GRID_BUDGET);
elseif isfield(cfg,'P_TOTAL_MAX') && isnumeric(cfg.P_TOTAL_MAX) && isscalar(cfg.P_TOTAL_MAX)
    Pmax = double(cfg.P_TOTAL_MAX);
elseif isfield(outB,'limits') && isstruct(outB.limits) && isfield(outB.limits,'P_total_max')
    Pmax = double(outB.limits.P_total_max);
end

% ---------------- optional: save skeleton-only compare ----------------
if strlength(save_skeleton_png) > 0
    figS = figure('Name', fig_name+"_skeleton", 'Color','w');
    tloS = tiledlayout(figS, 1, 2, 'Padding','compact', 'TileSpacing','compact');

    axS1 = nexttile(tloS, 1); hold(axS1,'on'); grid(axS1,'on'); axis(axS1,'equal');
    plot_skeleton(axS1, robot, eeName, q0, qB_end, eeB, ee_start, ee_goal);
    title(axS1, 'Baseline');

    axS2 = nexttile(tloS, 2); hold(axS2,'on'); grid(axS2,'on'); axis(axS2,'equal');
    plot_skeleton(axS2, robot, eeName, q0, qT_end, eeT, ee_start, ee_goal);
    title(axS2, 'TRF');

    sgtitle(tloS, 'EE trajectory and robot skeleton (initial/final)');
    exportgraphics(figS, save_skeleton_png, 'Resolution', 300);
    fprintf('[Fig3] Saved skeleton compare: %s\n', save_skeleton_png);
end

% ---------------- redraw main Fig.3 ----------------
fig = figure('Name', fig_name, 'Color', 'w');

if include_skeleton && include_joint_power
    % 3 rows x 2 cols:
    % row1: skeleton (Baseline / TRF)
    % row2: BUS power (Baseline / TRF)
    % row3: joint power (Baseline / TRF)
    tlo = tiledlayout(fig, 3, 2, 'Padding','compact', 'TileSpacing','compact');

    % ===== skeleton row =====
    ax1 = nexttile(tlo, 1); hold(ax1,'on'); grid(ax1,'on'); axis(ax1,'equal');
    plot_skeleton(ax1, robot, eeName, q0, qB_end, eeB, ee_start, ee_goal);
    title(ax1, '(a) Baseline: EE traj + skeleton');

    ax2 = nexttile(tlo, 2); hold(ax2,'on'); grid(ax2,'on'); axis(ax2,'equal');
    plot_skeleton(ax2, robot, eeName, q0, qT_end, eeT, ee_start, ee_goal);
    title(ax2, '(b) TRF: EE traj + skeleton');

    % ===== BUS row =====
    ax3 = nexttile(tlo, 3); hold(ax3,'on'); grid(ax3,'on');
    title(ax3, 'Baseline: DC bus power');
    plot_bus(ax3, tB, PgridB, PdumpB, PgridB_lpf, Pmax, show_budget_line, pgrid_tau_s);
    ylabel(ax3,'Power [W]'); xlabel(ax3,'t [s]');

    ax4 = nexttile(tlo, 4); hold(ax4,'on'); grid(ax4,'on');
    title(ax4, 'TRF: DC bus power');
    plot_bus(ax4, tT, PgridT, PdumpT, PgridT_lpf, Pmax, show_budget_line, pgrid_tau_s);
    ylabel(ax4,'Power [W]'); xlabel(ax4,'t [s]');

    % ===== joint power row =====
    ax5 = nexttile(tlo, 5); hold(ax5,'on'); grid(ax5,'on');
    title(ax5, 'Baseline: per-joint mechanical power');
    plot_joint_power(ax5, tB, PBj);
    ylabel(ax5,'P_j [W]'); xlabel(ax5,'t [s]');

    ax6 = nexttile(tlo, 6); hold(ax6,'on'); grid(ax6,'on');
    title(ax6, 'TRF: per-joint mechanical power');
    plot_joint_power(ax6, tT, PTj);
    ylabel(ax6,'P_j [W]'); xlabel(ax6,'t [s]');

    % ---- apply BUS y-limits (same scale for Baseline/TRF) ----
    apply_bus_ylim(ax3, ax4, PgridB, PdumpB, PgridT, PdumpT, PgridB_lpf, PgridT_lpf, Pmax, bus_ylim_focus, bus_ylim_margin);

    sgtitle(tlo, 'Representative trial: kinematics and power (Baseline vs TRF)');

elseif include_skeleton && ~include_joint_power
    % 2 rows x 2 cols:
    % row1: skeleton, row2: BUS
    tlo = tiledlayout(fig, 2, 2, 'Padding','compact', 'TileSpacing','compact');

    ax1 = nexttile(tlo, 1); hold(ax1,'on'); grid(ax1,'on'); axis(ax1,'equal');
    plot_skeleton(ax1, robot, eeName, q0, qB_end, eeB, ee_start, ee_goal);
    title(ax1, '(a) Baseline: EE traj + skeleton');

    ax2 = nexttile(tlo, 2); hold(ax2,'on'); grid(ax2,'on'); axis(ax2,'equal');
    plot_skeleton(ax2, robot, eeName, q0, qT_end, eeT, ee_start, ee_goal);
    title(ax2, '(b) TRF: EE traj + skeleton');

    ax3 = nexttile(tlo, 3); hold(ax3,'on'); grid(ax3,'on');
    title(ax3, 'Baseline: DC bus power');
    plot_bus(ax3, tB, PgridB, PdumpB, PgridB_lpf, Pmax, show_budget_line, pgrid_tau_s);
    ylabel(ax3,'Power [W]'); xlabel(ax3,'t [s]');

    ax4 = nexttile(tlo, 4); hold(ax4,'on'); grid(ax4,'on');
    title(ax4, 'TRF: DC bus power');
    plot_bus(ax4, tT, PgridT, PdumpT, PgridT_lpf, Pmax, show_budget_line, pgrid_tau_s);
    ylabel(ax4,'Power [W]'); xlabel(ax4,'t [s]');

    apply_bus_ylim(ax3, ax4, PgridB, PdumpB, PgridT, PdumpT, PgridB_lpf, PgridT_lpf, Pmax, bus_ylim_focus, bus_ylim_margin);
    sgtitle(tlo, 'Representative trial: kinematics and DC-bus power (Baseline vs TRF)');

else
    % fallback: old 2x2 (BUS + joint power)
    tlo = tiledlayout(fig, 2, 2, 'Padding','compact', 'TileSpacing','compact');

    ax1 = nexttile(tlo, 1); hold(ax1,'on'); grid(ax1,'on');
    title(ax1, '(a) Baseline: DC bus power');
    plot_bus(ax1, tB, PgridB, PdumpB, PgridB_lpf, Pmax, show_budget_line, pgrid_tau_s);
    ylabel(ax1,'Power [W]'); xlabel(ax1,'t [s]');

    ax2 = nexttile(tlo, 2); hold(ax2,'on'); grid(ax2,'on');
    title(ax2, '(b) TRF: DC bus power');
    plot_bus(ax2, tT, PgridT, PdumpT, PgridT_lpf, Pmax, show_budget_line, pgrid_tau_s);
    ylabel(ax2,'Power [W]'); xlabel(ax2,'t [s]');

    ax3 = nexttile(tlo, 3); hold(ax3,'on'); grid(ax3,'on');
    title(ax3, 'Baseline: per-joint mechanical power');
    plot_joint_power(ax3, tB, PBj);
    ylabel(ax3,'P_j [W]'); xlabel(ax3,'t [s]');

    ax4 = nexttile(tlo, 4); hold(ax4,'on'); grid(ax4,'on');
    title(ax4, 'TRF: per-joint mechanical power');
    plot_joint_power(ax4, tT, PTj);
    ylabel(ax4,'P_j [W]'); xlabel(ax4,'t [s]');

    apply_bus_ylim(ax1, ax2, PgridB, PdumpB, PgridT, PdumpT, PgridB_lpf, PgridT_lpf, Pmax, bus_ylim_focus, bus_ylim_margin);
    sgtitle(tlo, 'Representative trial: joint mechanical power and DC bus power (Baseline vs TRF)');
end

% save main fig
if strlength(save_png) > 0
    exportgraphics(fig, save_png, 'Resolution', 300);
    fprintf('[Fig3] Saved: %s\n', save_png);
end

end

% ---------------- plot helpers ----------------
function plot_skeleton(ax, robot, eeName, q0, q_end, ee_traj, ee_start, ee_goal)
    view(ax, 135, 20);
    [X0,Y0,Z0] = chain_skeleton_xyz(robot, q0, eeName);
    if ~isempty(X0)
        plot3(ax, X0, Y0, Z0, '-o', 'LineWidth', 1.0, 'DisplayName', 'Initial');
    end
    [Xf,Yf,Zf] = chain_skeleton_xyz(robot, q_end, eeName);
    if ~isempty(Xf)
        plot3(ax, Xf, Yf, Zf, 'm-o', 'MarkerFaceColor','m', 'LineWidth', 1.0, 'DisplayName', 'Final');
    end
    if ~isempty(ee_traj) && size(ee_traj,2) >= 3
        plot3(ax, ee_traj(:,1), ee_traj(:,2), ee_traj(:,3), 'c-', 'LineWidth', 1.5, 'DisplayName', 'EE traj');
        plot3(ax, ee_traj(1,1), ee_traj(1,2), ee_traj(1,3), 'go', 'MarkerFaceColor','g', 'DisplayName', 'Start');
        plot3(ax, ee_traj(end,1), ee_traj(end,2), ee_traj(end,3), 'ro', 'MarkerFaceColor','r', 'DisplayName', 'Final EE');
    else
        plot3(ax, ee_start(1), ee_start(2), ee_start(3), 'go', 'MarkerFaceColor','g', 'DisplayName', 'Start');
    end
    plot3(ax, ee_goal(1), ee_goal(2), ee_goal(3), 'co', 'MarkerFaceColor','c', 'DisplayName', 'Target');
    xlabel(ax,'X [m]'); ylabel(ax,'Y [m]'); zlabel(ax,'Z [m]');
    legend(ax, 'Location','best');
end

function plot_bus(ax, t, Pgrid, Pdump, Pfilt, Pmax, show_budget_line, tauP)
    % v5: one causal first-order LPF is used for the bus-power metric.
    h1 = plot(ax, t, Pgrid, 'LineWidth', 0.8, 'DisplayName','P_{grid} raw');
    h1.Color = 0.70*[1 1 1];
    h2 = plot(ax, t, Pdump, 'LineWidth', 0.8, 'DisplayName','P_{dump}');
    h2.Color = 0.85*[1 1 1];

    if show_budget_line && isfinite(Pmax)
        plot(ax, [t(1), t(end)], [Pmax, Pmax], 'r--', 'LineWidth', 1.2, 'DisplayName','P_{grid,budget}');
    end

    hF = plot(ax, t, Pfilt, 'k-', 'LineWidth', 2.2, ...
        'DisplayName',sprintf('P_{grid} LPF (\tau=%.2fs)', tauP));
    uistack(hF,'top');

    if ~isempty(t)
        xlim(ax, [t(1), t(end)]);
    end
    legend(ax, 'Location','best');
end

function plot_joint_power(ax, t, Pj)
    if isempty(Pj) || isempty(t), return; end
    if size(Pj,1) ~= numel(t) && size(Pj,2) == numel(t)
        Pj = Pj.'; % transpose if needed
    end
    nJ = size(Pj,2);
    for j = 1:nJ
        plot(ax, t, Pj(:,j), 'LineWidth', 1.0, 'DisplayName', sprintf('J%d', j));
    end
    xlim(ax, [t(1), t(end)]);
    legend(ax, 'Location','best');
end

function apply_bus_ylim(axL, axR, PgridL, PdumpL, PgridR, PdumpR, PlpfL, PlpfR, Pmax, focus_avg, margin)
    if focus_avg
        yref = max([max(PlpfL), max(PlpfR)]);
        if isfinite(Pmax), yref = max(yref, Pmax); end
    else
        yref = max([max(PgridL), max(PdumpL), max(PgridR), max(PdumpR)]);
        if isfinite(Pmax), yref = max(yref, Pmax); end
    end
    if ~isfinite(yref) || yref <= 0, yref = 1; end
    ymax = (1 + margin) * yref;
    ylim(axL, [0, ymax]);
    ylim(axR, [0, ymax]);
end

% ---------------- utility helpers ----------------
function v = get_opt(opts, name, default)
    if ~isstruct(opts) || ~isfield(opts, name) || isempty(opts.(name))
        v = default;
    else
        v = opts.(name);
    end
end

function v = get_nested_opt(S, path, default)
    v = default;
    if ~isstruct(S), return; end
    parts = strsplit(path,'.');
    cur = S;
    for ii=1:numel(parts)
        p = parts{ii};
        if isstruct(cur) && isfield(cur,p) && ~isempty(cur.(p))
            cur = cur.(p);
        else
            return;
        end
    end
    v = cur;
end

function must_exist(fn)
    assert(exist(fn,'file')==2, '[Fig3] Required function not found on path: %s', fn);
end

function req_struct(s, fns)
    for i = 1:numel(fns)
        assert(isfield(s, fns{i}), '[Fig3] Missing field: %s', fns{i});
    end
end

function safe_close(h)
    try
        if ~isempty(h) && isvalid(h)
            close(h);
        end
    catch
    end
end

function safe_close_list(hs)
    try
        for k = 1:numel(hs)
            safe_close(hs(k));
        end
    catch
    end
end

function r = row(v), r = v(:)'; end
function x = deg2rad_local(d), x = pi/180 * row(d); end

function qf = unwrap_to_near(q0, qf)
    d = qf - q0;
    qf = q0 + atan2(sin(d), cos(d));
end

function eeName = get_ee_name(cfg, robot)
    if isfield(cfg,'EE_NAME') && ~isempty(cfg.EE_NAME)
        eeName = cfg.EE_NAME;
    else
        eeName = robot.BodyNames{end};
    end
end

function p = ee_pos(robot, q, eeName)
    p = tform2trvec(getTransform(robot, q, eeName));
end

function [X,Y,Z] = chain_skeleton_xyz(robot, q, eeName)
    X=[]; Y=[]; Z=[];
    try
        b = robot.getBody(eeName); names={};
        while ~strcmp(b.Name, robot.BaseName)
            names{end+1} = b.Name; %#ok<AGROW>
            b = robot.getBody(b.Parent);
        end
        names = fliplr(names);
    catch
        names = robot.BodyNames;
    end
    for i=1:numel(names)
        try
            Tb = getTransform(robot, q, names{i});
            p  = tform2trvec(Tb);
            X(end+1)=p(1); Y(end+1)=p(2); Z(end+1)=p(3); %#ok<AGROW>
        catch
        end
    end
    try
        Tee = getTransform(robot, q, eeName);
        p = tform2trvec(Tee);
        if isempty(X) || norm([X(end)-p(1),Y(end)-p(2),Z(end)-p(3)])>1e-12
            X(end+1)=p(1); Y(end+1)=p(2); Z(end+1)=p(3);
        end
    catch
    end
end
