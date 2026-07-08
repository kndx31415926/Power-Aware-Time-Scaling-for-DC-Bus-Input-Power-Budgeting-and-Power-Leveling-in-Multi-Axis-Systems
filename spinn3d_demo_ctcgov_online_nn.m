function out = spinn3d_demo_ctcgov_online_nn(cfg)
% CTC + Governor + NN-α（在线推理；Transformer look-ahead 模型）
% - 默认模型路径来自 PB.paths.model_alpha（建议指向 model_alpha_gov_trf.mat）
% - 特征：@spinn3d_features_alpha_plus_payload_gov（基础+payload10+尾巴6）
% - payload：本轨迹内固定，struct(mass/com/inertia) 同训练口径
% - BUS 口径护栏 + 统一绘图风格（骨架/EE轨迹/功率）

%% -------- cfg / 必需字段 --------
if nargin < 1 || isempty(cfg)
    blk = [];
    if exist('spinn3d_params_block','file')==2
        blk = spinn3d_params_block();
    elseif evalin('base',"exist(''spinn3d_params_block'',''var'')==1")
        blk = evalin('base','spinn3d_params_block');
    end
    assert(~isempty(blk), '[alpha-demo] 未提供 cfg，且未找到 spinn3d_params_block。');
    cfg = spinn3d_params_to_cfg(blk, 'demo_alpha');  % 从 PB 提取 TS/DT/限值/噪声等
end

req(cfg, {'TS','DT','VMAX','AMAX','Q0_DEG','QF_DEG', ...
          'P_TOTAL_MAX','ALPHA_MIN','ALPHA_ITMAX','MODEL_PATH'});
% Budget-aware runs must keep a low alpha floor.  The fast no-power baseline
% sets its own local floor and should not leak into Feasible/TRF/MPC runs.
cfg.ALPHA_MIN = max(min(cfg.ALPHA_MIN, 0.15), 0.06);   % peak-fix budgeted floor
assert(isfield(cfg,'ROBOT') && ~isempty(cfg.ROBOT), 'cfg.ROBOT 缺失。');
assert(isfield(cfg,'FRIC')  && all(isfield(cfg.FRIC,{'B','Fc','vel_eps'})), 'cfg.FRIC 不完整。');
assert(isfield(cfg,'NOISE') && all(isfield(cfg.NOISE,{'sigma_q','sigma_dq','tau_std'})), 'cfg.NOISE 不完整。');

% 限值 / 增益 / 停止设置
limits = cfg.CAPS;   gains = cfg.CTRL;
req(limits, {'tau_max','qd_max','qdd_max','P_axis_max'}); req(gains,{'Kp','Kd'});
Pbud = getfield_def(cfg,'P_GRID_BUDGET', cfg.P_TOTAL_MAX);
Ppeak = getfield_def(cfg,'P_GRID_PEAK_MAX', inf);
limits.P_grid_budget = Pbud;
limits.P_total_max = Pbud;              % legacy alias
limits.P_grid_peak_max = Ppeak;
if ~isfield(cfg,'STOP') || ~isstruct(cfg.STOP), cfg.STOP = struct(); end
if ~isfield(cfg.STOP,'NEAR_TOL') || isempty(cfg.STOP.NEAR_TOL), cfg.STOP.NEAR_TOL = 0.02; end
if ~isfield(cfg.STOP,'VEL_TOL')  || isempty(cfg.STOP.VEL_TOL),  cfg.STOP.VEL_TOL  = 0.03; end

%% -------- 机器人 / 起终位形 --------
robot = cfg.ROBOT; try, robot.DataFormat='row'; end
nJ    = numel(homeConfiguration(robot));
q0    = deg2rad_local(row(cfg.Q0_DEG));
qf    = deg2rad_local(row(cfg.QF_DEG)); qf = unwrap_to_near(q0, qf);

% 末端名
if isfield(cfg,'EE_NAME') && ~isempty(cfg.EE_NAME), eeName = cfg.EE_NAME;
else, eeName = robot.BodyNames{end}; end

%% -------- 注入 payload（本次轨迹固定为 struct） --------
pay_cfg = []; 
if isfield(cfg,'PAYLOAD') && isstruct(cfg.PAYLOAD) && isfield(cfg.PAYLOAD,'DEMO')
    pay_cfg = cfg.PAYLOAD.DEMO;
end
% ★ 对比/复现实验时务必固定 payload 注入口径，避免 nn/nonn 机器人模型不一致
if isstruct(pay_cfg)
    % 默认：payload 给的是 COM 处惯量（更符合 URDF/CAD 常规），再平行轴移到 body 原点
    if ~isfield(pay_cfg,'about') || isempty(pay_cfg.about), pay_cfg.about = 'com'; end
    if ~isfield(pay_cfg,'mode')  || isempty(pay_cfg.mode),  pay_cfg.mode  = 'replace'; end
end
[robot, pay_v10] = inject_payload_last_body(robot, pay_cfg); % 工具惯量注入（可选）
payload = v10_to_struct(pay_v10);                             % struct 给特征用（与训练一致）
% ↑ 与数据集/训练保持一致的 payload 口径；否则 NN 口径漂移。

%% -------- 笛卡尔路径 → 关节参考 --------
traj  = make_traj_cart_bezier_dls(robot, q0, qf, limits, cfg);

%% -------- 装载模型（dlnetwork + preproc + keepBaseIdx） --------
M = load_model_alpha_strict(cfg.MODEL_PATH);   % 必须含 net/preproc/keepBaseIdx
% 推理侧严格按训练侧压列+标准化：

%% -------- 控制器（严格推理） --------
areg = struct('BETA',[], 'A_DOT_UP',[], 'A_DOT_DN',[]);
if isfield(cfg,'ALPHA_REG')
    fns = fieldnames(areg);
    for i=1:numel(fns)
        if isfield(cfg.ALPHA_REG,fns{i})
            areg.(fns{i}) = cfg.ALPHA_REG.(fns{i});
        end
    end
end

bus = get_bus_from_cfg(cfg, limits.P_total_max);  % BUS 结构（总线效率/制动电阻等），用于控制器与功率护栏统一口径

opts = struct();
opts.Ts      = cfg.TS;                 % 1 ms plant/CTC/governor step
opts.policyTs= getfield_def(cfg,'POLICY_DT', cfg.TS); % TRF/teacher update period
opts.fric    = cfg.FRIC;
opts.bus     = bus;
opts.alphaReg= areg;
if isfield(cfg,'BUDGET_CTRL') && isstruct(cfg.BUDGET_CTRL)
    opts.budgetCtrl = cfg.BUDGET_CTRL;
end
opts.nnAlpha = struct( ...
    'enable',      true, ...
    'mode',        'nn', ...
    'model',       M, ...
    'alpha_floor', cfg.ALPHA_MIN, ...
    'featureFcn',  @spinn3d_features_alpha_plus_payload_gov, ... % governor 版特征（末尾 6 维固定顺序）
    'payload',     payload, ...
    'gate',        struct('enable', false));  % pure TRF; no online feasible search mixed in


ctl = spinn3d_controller_ctcgov(robot, traj, gains, limits, opts);  % 内部做压列/标准化/前向

%% -------- 仿真步数预算 --------
Tcap = traj.Tf/max(cfg.ALPHA_MIN,1e-6) + 0.5;
Nt   = max(2, ceil(Tcap/cfg.TS) + 1);

%% -------- 常量与 BUS 口径 --------
B  = row(cfg.FRIC.B);   Fc = row(cfg.FRIC.Fc);   ve = cfg.FRIC.vel_eps;
nq = cfg.NOISE.sigma_q; ndq=cfg.NOISE.sigma_dq; ntau=cfg.NOISE.tau_std;
Tservo      = getfield_def(cfg,'SERVO_HOLD_T', 0.02);
noise_hold  = zeros(1,nJ);
noise_timer = 0;

tau_max   = vec_lim(limits,'tau_max',    nJ, inf);
P_axismax = vec_lim(limits,'P_axis_max', nJ, inf);


%% -------- 主循环 --------
t=0; s=0; [q_ref0, dq_ref0, ddq_ref0] = traj.eval(0); %#ok<ASGLU>
q = row(q_ref0); dq = zeros(1,nJ);

log.t=zeros(Nt,1); log.s=zeros(Nt,1);
log.q=zeros(Nt,nJ); log.dq=zeros(Nt,nJ);
log.tau=zeros(Nt,nJ); log.alpha=zeros(Nt,1);
log.Pi_raw=zeros(Nt,nJ); log.Pgrid=zeros(Nt,1); log.Pdump=zeros(Nt,1);
log.Ppos=zeros(Nt,1); log.Pneg=zeros(Nt,1); log.Pspill=zeros(Nt,1);
log.Vdc=zeros(Nt,1); log.Igrid=zeros(Nt,1); log.Ibatt=zeros(Nt,1); log.Ploss=zeros(Nt,1);
log.Egrid=zeros(Nt,1); log.Edump=zeros(Nt,1); log.Ereuse=zeros(Nt,1); log.Espill=zeros(Nt,1);
log.Preuse_total=zeros(Nt,1); log.Ereuse_cum=zeros(Nt,1);
% Source-limited electrical-bus diagnostic fields.  These fields must be
% preallocated because MATLAB otherwise creates 1-by-k row vectors on first
% indexed assignment, which later breaks truncate_log(log,k) after early goal
% reaching.
log.Pgrid_request=zeros(Nt,1);
log.Psource_shortfall=zeros(Nt,1);
log.Psource_limited=false(Nt,1);
log.Paux=zeros(Nt,1);
log.policy_updated=false(Nt,1);
% Controller-side budget servo diagnostics (for tuning TRF budget tracking).
log.a0_raw=nan(Nt,1);
log.Pgrid_ctrl_lp=nan(Nt,1);
log.Pgrid_ctrl_fast=nan(Nt,1);
log.budget_ratio_ctrl=nan(Nt,1);
log.budget_fast_ratio_ctrl=nan(Nt,1);
log.budget_trim=false(Nt,1);
log.budget_boost=false(Nt,1);
log.budget_predcap=false(Nt,1);
log.budget_floor_infeasible=false(Nt,1);
log.budget_floor_recovery=false(Nt,1);
log.ee=nan(Nt,3);

ee_start = ee_pos(robot,q0,eeName);
ee_goal  = ee_pos(robot,qf,eeName);
busState = spinn3d_bus_state_init(bus);

t_reach=NaN;

for k=1:Nt
    [q_r, dq_r, ddq_r] = traj.eval(s); q_r=row(q_r); dq_r=row(dq_r); ddq_r=row(ddq_r);

    % —— 测量噪声 ——
    q_m  = q  + nq  * randn(1,nJ);
    dq_m = dq + ndq * randn(1,nJ);

    % —— 参考命令（名义，不外层缩放） ——
    dq_ref_cmd  = dq_r;
    ddq_ref_cmd = ddq_r;

    % 控制器（内部求 α，返回 info.a_max）
    % Use noisy measured state for the controller, while the plant state below
    % remains the true q/dq.  Otherwise sensor-noise robustness is not tested.
    [tau_cmd, info] = ctl.step(t, q_m, dq_m, q_r, dq_r, ddq_r, robot, s);
    if getfield_def(cfg,'LOG_AFEAS', getfield_def(cfg,'LOG_FEASIBLE_ORACLE',false))
        a_feas = alpha_feasible(q_r, dq_r, ddq_r, 1.0, robot, limits, ...
                            limits.P_total_max, cfg.ALPHA_MIN, cfg.ALPHA_ITMAX, cfg.FRIC);
    else
        a_feas = NaN;
    end
    if ~isfield(log,'a0'), log.a0=zeros(Nt,1); log.afeas=zeros(Nt,1); end
    log.a0(k)    = info.a0;        % held/updated NN desired command
    log.alpha(k) = info.a_max;     % governor 后实际用的
    log.afeas(k) = a_feas;         % optional diagnostic only; off by default for latency fairness
    tau_cmd = row(tau_cmd);

    % ===== 命令级护栏：τ 限幅 + BUS 口径功率缩放 =====
    tau_cmd = clamp_tau(tau_cmd, tau_max);
    %tau_cmd = clamp_power_axis_total_bus(tau_cmd, dq, P_axismax, bus);  % 项目内现有函数（总线功率口径）

    % —— 伺服保持噪声（梳齿）——
    noise_timer = noise_timer + cfg.TS;
    if noise_timer >= Tservo
        noise_hold  = ntau * randn(1,nJ);
        noise_timer = noise_timer - Tservo;
    end
    tau_m = tau_cmd + noise_hold;

    % ===== 执行级：仅扭矩限幅 =====
    tau_m = clamp_tau(tau_m, tau_max);

    % —— 对象动力学积分（含摩擦）——
    Mq = massMatrix(robot, q); c = velocityProduct(robot, q, dq); g = gravityTorque(robot, q);
    tau_fric = B.*dq + Fc.*tanh(dq/max(ve,1e-9));
    ddq = (Mq \ (tau_m(:) - c(:) - g(:) - tau_fric(:))).';
    dq  = dq + cfg.TS*ddq; q = q + cfg.TS*dq;

    % —— 几何时间推进 —— 
    s = min(traj.Tf, s + cfg.TS * info.a_max);

    % —— 记录：electrical BUS 功率/能量/电流/电压 —— 
    [busFlow,busState] = spinn3d_bus_step_elec(tau_m, dq, bus, busState, cfg.TS);
    log.t(k)=t; log.s(k)=s; log.q(k,:)=q; log.dq(k,:)=dq; log.tau(k,:)=tau_m;
    log.alpha(k)=info.a_max; log.Pi_raw(k,:)=tau_m.*dq;
    log.Ppos(k)=busFlow.Ppos; log.Pneg(k)=busFlow.Pneg;
    log.Pgrid(k)=busFlow.Pgrid; log.Pdump(k)=busFlow.Pdump; log.Pspill(k)=busFlow.Pspill;
    log.Vdc(k)=busFlow.Vdc; log.Igrid(k)=busFlow.Igrid; log.Ibatt(k)=busFlow.Ibatt; log.Ploss(k)=busFlow.Ploss;
    log.Pgrid_request(k)=getfield_def(busFlow,'Pgrid_request',busFlow.Pgrid);
    log.Psource_shortfall(k)=getfield_def(busFlow,'Psource_shortfall',0);
    log.Psource_limited(k)=double(getfield_def(busFlow,'Psource_limited',false));
    log.Paux(k)=getfield_def(busFlow,'Paux',0);
    log.Egrid(k)=busFlow.Egrid; log.Edump(k)=busFlow.Edump; log.Espill(k)=busFlow.Espill;
    log.Preuse_total(k)=getfield_def(busFlow,'Preuse_total',0);
    log.Ereuse(k)=getfield_def(busFlow,'Ereuse_total',getfield_def(busState,'Ereuse_total',NaN));
    log.Ereuse_cum(k)=log.Ereuse(k);
    log.policy_updated(k)=logical(getfield_def(info,'policy_updated',false));
    log.a0_raw(k)=getfield_def(info,'a0_raw',getfield_def(info,'a0',NaN));
    log.Pgrid_ctrl_lp(k)=getfield_def(info,'Pgrid_lp',NaN);
    log.Pgrid_ctrl_fast(k)=getfield_def(info,'Pgrid_fast',NaN);
    log.budget_ratio_ctrl(k)=getfield_def(info,'budget_ratio',NaN);
    log.budget_fast_ratio_ctrl(k)=getfield_def(info,'budget_fast_ratio',NaN);
    bi=getfield_def(info,'budget_info',struct());
    baction=lower(char(getfield_def(bi,'action','')));
    log.budget_trim(k)=~isempty(strfind(baction,'trim'));
    log.budget_boost(k)=~isempty(strfind(baction,'boost'));
    if isfield(log,'budget_predcap'), log.budget_predcap(k)=~isempty(strfind(baction,'predcap')); end
    if isfield(log,'budget_floor_infeasible')
        pcinfo=getfield_def(bi,'predictive_cap',struct());
        log.budget_floor_infeasible(k)=logical(getfield_def(pcinfo,'floor_infeasible',false));
        if isfield(log,'budget_floor_recovery'), log.budget_floor_recovery(k)=logical(getfield_def(pcinfo,'floor_recovery',false)); end
    end
    Tee = getTransform(robot, q, eeName); log.ee(k,:) = tform2trvec_safe(Tee);

    % —— 停止：几何时间到尾 + 末端近邻 + 低速 —— 
    ee_err = norm(log.ee(k,:) - ee_goal);
    if (s >= traj.Tf) && (ee_err <= cfg.STOP.NEAR_TOL) && (norm(dq) <= cfg.STOP.VEL_TOL)
        t_reach = t; log = truncate_log(log, k); break;
    end

    t = t + cfg.TS;
end

out = struct('log',log,'limits',limits,'gains',gains,'cfg',cfg,'t_reach',t_reach,'payload',payload);
if getfield_def(cfg,'NO_FIGURE',false) || getfield_def(cfg,'DISABLE_FIGURES',false)
    return;
end

%% -------- 绘图（统一 nonn 风格） --------
fig = figure('Name','CTC+Gov (online NN-α, BUS power)','Visible','on');
tlo = tiledlayout(fig,2,2,'Padding','compact','TileSpacing','compact');

% === 3D：机械臂骨架 + EE 轨迹 ===
ax3 = nexttile(tlo,[2 1]); hold(ax3,'on'); grid(ax3,'on'); axis(ax3,'equal'); view(ax3,135,20);
[X0,Y0,Z0] = chain_skeleton_xyz(robot, q0, eeName);
if ~isempty(X0), plot3(ax3, X0, Y0, Z0, '-o','LineWidth',1.0,'DisplayName','Initial'); end
try, show(robot, log.q(end,:), 'Parent', ax3, 'PreservePlot', true, 'Frames','off','Visuals','on'); catch, end
[Xf,Yf,Zf] = chain_skeleton_xyz(robot, log.q(end,:), eeName);
if ~isempty(Xf), plot3(ax3, Xf, Yf, Zf, 'm-o','MarkerFaceColor','m','LineWidth',1.0,'DisplayName','Final'); end
plot3(ax3, log.ee(:,1), log.ee(:,2), log.ee(:,3), 'c-','LineWidth',1.5,'DisplayName','EE traj');
plot3(ax3, ee_start(1),ee_start(2),ee_start(3),'go','MarkerFaceColor','g','DisplayName','Start');
plot3(ax3, ee_goal(1), ee_goal(2), ee_goal(3),'co','MarkerFaceColor','c','DisplayName','Target');
plot3(ax3, log.ee(end,1),log.ee(end,2),log.ee(end,3),'ro','MarkerFaceColor','r','DisplayName','Final EE');
xlabel(ax3,'X [m]'); ylabel(ax3,'Y [m]'); zlabel(ax3,'Z [m]');
title(ax3,'EE Trajectory + Robot Skeleton (Initial/Final; NN)'); legend(ax3,'Location','best');

% === 各关节功率 ===
axU = nexttile(tlo); hold(axU,'on'); grid(axU,'on');
for j=1:nJ, plot(axU, log.t, log.Pi_raw(:,j), 'DisplayName', sprintf('J%d',j)); end
xlabel(axU,'t [s]'); ylabel(axU,'Joint Power P_j [W]');
title(axU,'Per-Joint Power'); legend(axU,'show','Location','best');
if ~isempty(log.t), xlim(axU,[log.t(1), log.t(end)]); end

% === 总线功率 ===
axD = nexttile(tlo); hold(axD,'on'); grid(axD,'on');
Praw_plot = double(log.Pgrid(:));
Pdump_plot = double(log.Pdump(:));
Pbudget_plot = NaN;
if exist('Pmax_plot','var') && ~isempty(Pmax_plot), Pbudget_plot = Pmax_plot; end
if ~isfinite(Pbudget_plot) && exist('limits','var') && isstruct(limits) && isfield(limits,'P_total_max') && ~isempty(limits.P_total_max)
    Pbudget_plot = limits.P_total_max;
end
if ~isfinite(Pbudget_plot) && isfield(cfg,'P_GRID_BUDGET') && ~isempty(cfg.P_GRID_BUDGET)
    Pbudget_plot = cfg.P_GRID_BUDGET;
end
spinn3d_plot_bus_power_single_lpf(axD, log.t, Praw_plot, Pdump_plot, Pbudget_plot, cfg, 'titleText','BUS Power');

end % ====== 文件末尾 ======

% ----------------- helpers（局部，避免再缺依赖） -----------------
function req(S, keys)
for i=1:numel(keys), assert(isfield(S,keys{i}) && ~isempty(S.(keys{i})), '缺字段：%s', keys{i}); end
end
function x = getfield_def(S,fn,def), if isstruct(S)&&isfield(S,fn)&&~isempty(S.(fn)), x=S.(fn); else, x=def; end, end
function r = row(v), r=v(:)'; end
function x = deg2rad_local(d), x = pi/180 * row(d); end
function qf = unwrap_to_near(q0, qf)
d = qf - q0; qf = q0 + atan2(sin(d), cos(d));
end
function tau = clamp_tau(tau, tau_max)
tau = row(tau); tmax = row(tau_max); tau = max(-tmax, min(tmax, tau));
end
function v = vec_lim(lim, fn, n, def)
if isfield(lim,fn) && ~isempty(lim.(fn)), v=row(lim.(fn)); else, v=def*ones(1,n); end
if isscalar(v), v = repmat(v,1,n); end
v = v(1:n);
end
function [X,Y,Z] = chain_skeleton_xyz(robot, q, eeName)
X=[];Y=[];Z=[];
try
    b = robot.getBody(eeName); names={};
    while ~strcmp(b.Name, robot.BaseName)
        names{end+1} = b.Name;
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
    Tee = getTransform(robot, q, eeName); p = tform2trvec(Tee);
    if isempty(X) || norm([X(end)-p(1),Y(end)-p(2),Z(end)-p(3)])>1e-12
        X(end+1)=p(1); Y(end+1)=p(2); Z(end+1)=p(3);
    end
catch
end
end
function p = ee_pos(robot, q, eeName), p = tform2trvec(getTransform(robot,q,eeName)); end
function p = tform2trvec_safe(T), try, p=tform2trvec(T); catch, p=[NaN NaN NaN]; end, end
function log = truncate_log(log, k)
%TRUNCATE_LOG Robustly trim time-series fields after early goal reaching.
% Some diagnostic fields may be scalar metadata or row vectors if created by
% legacy code.  Only truncate fields whose first dimension or vector length
% can safely cover k samples; leave non-timeseries metadata untouched.
f = fieldnames(log);
for i = 1:numel(f)
    v = log.(f{i});
    if isempty(v)
        continue;
    end
    try
        if (isnumeric(v) || islogical(v)) && ndims(v) <= 2
            if size(v,1) >= k
                log.(f{i}) = v(1:k,:);
            elseif isvector(v) && numel(v) >= k
                vv = v(:);
                log.(f{i}) = vv(1:k,:);
            end
        end
    catch
        % Keep non-standard diagnostic/meta fields unchanged.
    end
end
end
function bus = get_bus_from_cfg_local(cfg, P_total_max)
bus = struct('P_total_max', P_total_max);
if isfield(cfg,'BUS') && isstruct(cfg.BUS)
    if isfield(cfg.BUS,'eta_share'),  bus.eta_share  = cfg.BUS.eta_share;  end
    if isfield(cfg.BUS,'P_dump_max'), bus.P_dump_max = cfg.BUS.P_dump_max; end
end
end
function S = v10_to_struct(v)
v = row(v); v = [v, zeros(1,10-numel(v))];  % 容错到 10 维
S = struct('mass',v(1), 'com',v(2:4), 'inertia',v(5:10));
end
function [robot_out, pay_vec] = inject_payload_last_body(robot_in, pay)
% 注入 payload（若提供），返回 1×10：
%   [m, com(1:3), Ixx Iyy Izz Iyz Ixz Ixy]   (Robotics System Toolbox 口径)
%
% pay 支持：
%   - 1×10 数值向量
%   - struct，字段 mass/com/inertia，可选 about/mode
%
% about:
%   - 'com'    : I6 相对 payload COM（默认；会平行轴换算到 body 原点）
%   - 'origin' : I6 相对 body 原点（不做平行轴换算）
% mode:
%   - 'replace': 覆盖末端刚体惯量（默认）
%   - 'add'    : 在末端刚体基础上累加

    robot_out = robot_in;
    pay_vec   = zeros(1,10);
    if isempty(pay), return; end

    if isnumeric(pay)
        v = double(pay(:)).';
        if numel(v) < 10, v = [v, zeros(1,10-numel(v))]; end
        m  = v(1); com = v(2:4); I6 = v(5:10);
        about = 'com';    % ★ 默认按 COM 惯量解释
        mode  = 'replace';
    elseif isstruct(pay)
        m   = getfield_def(pay,'mass',0);
        com = getfield_def(pay,'com',[0 0 0]);
        I6  = getfield_def(pay,'inertia',[0 0 0 0 0 0]);
        about = lower(getfield_def(pay,'about','com'));
        mode  = lower(getfield_def(pay,'mode','replace'));
    else
        return;
    end
    pay_vec = [double(m), double(com(:)).', double(I6(:)).'];

    try
        b  = robot_out.Bodies{end};
        m0 = double(b.Mass);
        c0 = double(b.CenterOfMass(:)).';
        I0 = I6ToMat(double(b.Inertia(:)).');

        Ic = I6ToMat(double(I6(:)).');
        if strcmpi(about,'com')
            r  = double(com(:)); r = r(:);
            I1 = Ic + double(m) * ((r.'*r)*eye(3) - (r*r.'));
        else
            I1 = Ic;
        end

        switch mode
            case 'add'
                m_new  = m0 + m;
                if m_new <= 0
                    c_new = [0 0 0];
                else
                    c_new = (m0*c0 + m*com(:).') / m_new;
                end
                I_new = I0 + I1;
            otherwise % 'replace'
                m_new  = m;
                c_new  = com(:).';
                I_new  = I1;
        end

        b.Mass         = m_new;
        b.CenterOfMass = c_new;
        b.Inertia      = ImatTo6(I_new).';
    catch
        % 静默跳过（仅影响动力学）
    end
end

function I = I6ToMat(v6)
% RST: v6 = [Ixx Iyy Izz Iyz Ixz Ixy]
    v6 = double(v6(:)).';
    Ixx=v6(1); Iyy=v6(2); Izz=v6(3);
    Iyz=v6(4); Ixz=v6(5); Ixy=v6(6);
    I = [ Ixx Ixy Ixz;
          Ixy Iyy Iyz;
          Ixz Iyz Izz ];
end

function v6 = ImatTo6(I)
% RST: v6 = [Ixx Iyy Izz Iyz Ixz Ixy]
    v6 = [ I(1,1), I(2,2), I(3,3), I(2,3), I(1,3), I(1,2) ];
end
function tau = get_eval_tau_slow_local(cfg)
    tau = 0.50;
    if isstruct(cfg) && isfield(cfg,'EVAL') && isstruct(cfg.EVAL)
        if isfield(cfg.EVAL,'power_slow_tau_s') && ~isempty(cfg.EVAL.power_slow_tau_s)
            tau = cfg.EVAL.power_slow_tau_s;
        elseif isfield(cfg.EVAL,'power_eval_tau_s') && ~isempty(cfg.EVAL.power_eval_tau_s)
            tau = cfg.EVAL.power_eval_tau_s;
        end
    end
end

