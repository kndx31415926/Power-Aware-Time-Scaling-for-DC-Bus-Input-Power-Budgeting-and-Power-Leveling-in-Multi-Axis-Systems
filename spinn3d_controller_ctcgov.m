function ctl = spinn3d_controller_ctcgov(robot, traj, gains, limits, opts)
%SPINN3D_CONTROLLER_CTCGOV  CTC + progress-rate governor.
%
% Revised timing logic:
%   - opts.Ts is the plant/CTC/governor servo step, typically 1 ms.
%   - opts.policyTs (preferred), opts.policyDt (legacy), or opts.policyPeriodSteps controls how often the NN or
%     feasibility teacher is recomputed, typically 10 ms.
%   - Between policy updates, a_des is zero-order held while the deterministic
%     governor (slew + alpha LPF) and inner CTC still run every servo step.
%
% Revised method separation:
%   - mode='nn' is pure learned TRF/MLP inference by default.  No online
%     feasibility search is mixed into the proposed method unless the user
%     explicitly enables nnAlpha.gate.fallback='feasible_cap'.
%   - mode='feasible' is the non-learning reference governor baseline.

try, robot.DataFormat = 'row'; end
nJ = local_num_joints(robot);

assert(isfield(opts,'Ts') && opts.Ts > 0, 'opts.Ts must be provided and >0');
Ts = double(opts.Ts);
policyN = 1;
if isfield(opts,'policyPeriodSteps') && ~isempty(opts.policyPeriodSteps)
    policyN = max(1, round(double(opts.policyPeriodSteps)));
elseif isfield(opts,'policyTs') && ~isempty(opts.policyTs)
    policyN = max(1, round(double(opts.policyTs) / Ts));
elseif isfield(opts,'policyDt') && ~isempty(opts.policyDt)
    policyN = max(1, round(double(opts.policyDt) / Ts));
end
policyDt = policyN * Ts;

Kp = vec_row(reqf(gains,'Kp'), nJ);
Kd = vec_row(reqf(gains,'Kd'), nJ);

assert(isfield(opts,'nnAlpha') && isstruct(opts.nnAlpha), 'missing opts.nnAlpha');
nn = opts.nnAlpha;
assert(isfield(nn,'enable') && nn.enable == true, 'nnAlpha must be enabled');
if ~isfield(nn,'mode') || isempty(nn.mode), nn.mode = 'nn'; end
nn.mode = char(nn.mode);
assert(ismember(nn.mode, {'nn','feasible','profile','ptp','bus_ptp_dp','topp_dp','mpc','mpc_rg','mpc_rg_grid'}), ...
       'nnAlpha.mode must be nn/feasible/profile/mpc_rg_grid');
if ~isfield(nn,'alpha_floor') || isempty(nn.alpha_floor), nn.alpha_floor = 0; end
if ~isfield(nn,'itmax') || isempty(nn.itmax), nn.itmax = 20; end

if strcmp(nn.mode,'nn')
    assert(isfield(nn,'model') && isfield(nn.model,'net') && isfield(nn.model,'preproc'), 'nnAlpha.model incomplete');
    % TRF uses dlnetwork; MLP-PRG may use SeriesNetwork/DAGNetwork.
    assert(isfield(nn,'featureFcn') && isa(nn.featureFcn,'function_handle'), 'missing governor feature function');
    assert(isfield(nn,'payload') && isstruct(nn.payload), 'nnAlpha.payload must be a struct');
end

req_fields = {'A_DOT_UP','A_DOT_DN','BETA'};
areg = struct();
assert(isfield(opts,'alphaReg') && isstruct(opts.alphaReg), 'missing opts.alphaReg');
for i = 1:numel(req_fields)
    k = req_fields{i};
    assert(isfield(opts.alphaReg,k) && ~isempty(opts.alphaReg.(k)), ['missing opts.alphaReg.' k]);
    areg.(k) = double(opts.alphaReg.(k));
end
areg.KINEMATIC_CAP_BEFORE_FILTER = logical(getfield_def(opts.alphaReg,'KINEMATIC_CAP_BEFORE_FILTER',true));

bus = spinn3d_bus_defaults(getfield_def(opts,'bus',struct()));
fric = getfield_def(opts,'fric', struct('B',zeros(1,nJ),'Fc',zeros(1,nJ),'vel_eps',1e-3));
fric.B = vec_row(getfield_def(fric,'B',zeros(1,nJ)), nJ);
fric.Fc = vec_row(getfield_def(fric,'Fc',zeros(1,nJ)), nJ);
fric.vel_eps = getfield_def(fric,'vel_eps',1e-3);

caps = normalize_caps(limits, nJ);
Pbud = getfield_def(limits,'P_grid_budget', getfield_def(limits,'P_total_max', bus.P_grid_budget));
if isnan(Pbud), Pbud = bus.P_grid_budget; end
bus.P_grid_budget = Pbud;
bus.P_total_max = Pbud;
if isfield(limits,'P_grid_peak_max') && ~isempty(limits.P_grid_peak_max)
    bus.P_grid_peak_max = limits.P_grid_peak_max;
end
bus = spinn3d_bus_defaults(bus);

% Optional slack gate.  Default is OFF to keep TRF pure.
gate = getfield_def(nn, 'gate', struct());
if ~isfield(gate,'enable') || isempty(gate.enable), gate.enable = false; end
if ~isfield(gate,'on_ratio') || isempty(gate.on_ratio), gate.on_ratio = 0.60; end
if ~isfield(gate,'off_ratio') || isempty(gate.off_ratio), gate.off_ratio = 0.80; end
if gate.off_ratio < gate.on_ratio, gate.off_ratio = min(1.0, gate.on_ratio + 0.05); end
if ~isfield(gate,'lp_tau_s') || isempty(gate.lp_tau_s), gate.lp_tau_s = 0.50; end
if ~isfield(gate,'a_nom') || isempty(gate.a_nom), gate.a_nom = 1.0; end
if ~isfield(gate,'skip_forward') || isempty(gate.skip_forward), gate.skip_forward = true; end
if ~isfield(gate,'fallback') || isempty(gate.fallback), gate.fallback = 'a_nom'; end
assert(ismember(gate.fallback, {'a_nom','feasible_cap'}), 'bad gate.fallback');

% Scalar budget-utilization feedback around the feed-forward policy command.
% This is deliberately much cheaper than feasibility search: it only uses one
% causal first-order LPF power telemetry state to trim/boost the desired alpha
% before the deterministic slew/LPF governor is applied.
bctrl = budget_ctrl_defaults(getfield_def(opts,'budgetCtrl',getfield_def(nn,'budgetCtrl',struct())));

state = struct();
state.a_prev = clamp(double(nn.alpha_floor), 0, 1);
state.a_des_hold = 1.0;
state.a_des_hold_raw = 1.0;
state.step_count = 0;
if bctrl.enable && isfinite(Pbud) && Pbud > 0
    state.Pgrid_lp   = double(Pbud) * bctrl.init_lp_ratio;
else
    state.Pgrid_lp   = 0.0;
end
state.Pgrid_fast = state.Pgrid_lp;  % legacy alias: v5 uses one LPF only
state.Pgrid_last = state.Pgrid_lp;  % previous-step raw/electrical draw estimate for hard trim
state.gate_active = false;
state.policy_updates = 0;

ctl = struct('step',@step, 'Kp',Kp, 'Kd',Kd, 'limits',caps, 'nnAlpha',nn, ...
             'Ts',Ts, 'policyDt',policyDt, 'policyTs',policyDt, 'policyN',policyN, ...
             'alphaReg',areg, 'bus',bus, 'budgetCtrl',bctrl);

    function [tau_cmd, info] = step(t, q_meas, dq_meas, q_ref, dq_ref, ddq_ref, robot_in, s) %#ok<INUSD>
        q_meas = row(q_meas); dq_meas = row(dq_meas);
        q_ref  = row(q_ref);  dq_ref  = row(dq_ref);  ddq_ref = row(ddq_ref);

        do_policy = (mod(state.step_count, policyN) == 0);
        policy_info = struct();
        mode_str = nn.mode;

        if do_policy
            [a0_new, policy_info, mode_str] = compute_policy_command(q_ref, dq_ref, ddq_ref, s);
            state.a_des_hold_raw = clamp(double(a0_new), double(nn.alpha_floor), 1.0);
            [state.a_des_hold, budget_info] = budget_alpha_command(state.a_des_hold_raw, mode_str, policyDt);
            state.policy_updates = state.policy_updates + 1;
        else
            if bctrl.enable && bctrl.servo_rate_feedback
                % Re-apply the scalar LPF/raw budget guard at the 1 ms servo rate.
                % Network/feasible inference remains at the policy rate; only the
                % cheap budget trim is updated every servo cycle.
                [state.a_des_hold, budget_info] = budget_alpha_command(state.a_des_hold_raw, mode_str, Ts);
                budget_info.action = ['servo_' char(budget_info.action)];
            else
                budget_info = struct('enabled',false,'action','hold','ratio_lpf',NaN, ...
                                     'ratio_slow',NaN,'ratio_fast',NaN,'raw_ratio',NaN, ...
                                     'a_raw',state.a_des_hold_raw,'a_cmd',state.a_des_hold);
            end
        end

        a0_raw = state.a_des_hold_raw;
        a0 = state.a_des_hold;

        % Optional predictive hard cap.  The scalar budget servo above is
        % feedback-only and can react after the LPF has started to rise.  This
        % guard predicts the next-step CTC torque and DC-bus power for candidate
        % progress-rate commands, then caps a0 by bisection so that the same
        % causal LPF metric used in the paper remains below the active budget
        % whenever the current pose is feasible.
        [a0, pcinfo] = predictive_budget_cap(a0, mode_str, q_meas, dq_meas, q_ref, dq_ref, ddq_ref);
        if isfield(pcinfo,'applied') && pcinfo.applied
            state.a_des_hold = a0;
            if isfield(budget_info,'action') && ~isempty(budget_info.action)
                budget_info.action = [char(budget_info.action) '+predcap'];
            else
                budget_info.action = 'predcap';
            end
        end
        budget_info.predictive_cap = pcinfo;

        [a, a_slew, a_filt, adot, guard] = governor_alpha_step(a0, state.a_prev, Ts, dq_ref, ddq_ref);

        % CTC with alpha-dot term included at the 1 ms servo step.
        dq_cmd  = a   * dq_ref;
        ddq_cmd = a*a * ddq_ref + adot * dq_ref;
        e  = q_ref  - q_meas;
        ed = dq_cmd - dq_meas;
        v  = ddq_cmd + Kd.*ed + Kp.*e;

        Mq = massMatrix(robot, q_meas);
        c  = velocityProduct(robot, q_meas, dq_meas);
        g  = gravityTorque(robot, q_meas);
        tau_cmd = (Mq * v(:) + c(:) + g(:)).';
        if any(fric.B) || any(fric.Fc)
            tau_cmd = tau_cmd + fric.B.*dq_meas + fric.Fc.*tanh(dq_meas./max(fric.vel_eps,1e-9));
        end

        % Diagnostics / budget-feedback LPF.  This is a standard causal first-order
        % low-pass filter: tau*dPbar/dt + Pbar = Pgrid.  It is not an online
        % feasibility search state and it is the same signal used for paper metrics.
        [~,~,Pgrid_est,~,~,~] = spinn3d_bus_power(tau_cmd, dq_meas, bus);
        betaP = exp(-Ts / max(double(bctrl.tau), 1e-6));
        state.Pgrid_lp = betaP * state.Pgrid_lp + (1-betaP) * Pgrid_est;
        state.Pgrid_fast = state.Pgrid_lp;  % legacy alias: v5 uses one LPF only
        state.Pgrid_last = Pgrid_est;

        state.a_prev = a;
        state.step_count = state.step_count + 1;

        info = struct('a_max',a, 'a0',a0, 'a0_raw',a0_raw, 'a_slew',a_slew, 'a_filt',a_filt, ...
                      'adot',adot, 'guard',guard, 'mode',mode_str, ...
                      'Pgrid_est',Pgrid_est, 'Pgrid_lp',state.Pgrid_lp, 'Pgrid_fast',state.Pgrid_fast, ...
                      'budget_ratio',state.Pgrid_lp/max(Pbud,1e-12), ...
                      'budget_fast_ratio',state.Pgrid_lp/max(Pbud,1e-12), ...
                      'budget_info',budget_info, ...
                      'policy_updated',do_policy, 'policy_dt',policyDt, 'policy_Ts',policyDt, ...
                      'policy_N',policyN, 'policy_updates',state.policy_updates, ...
                      'policy_info',policy_info, ...
                      'gate_active',getfield_def(policy_info,'gate_active',false), ...
                      'gate_ratio',getfield_def(policy_info,'gate_ratio',NaN));
    end

    function [a0, pinfo, mode_str] = compute_policy_command(q_ref, dq_ref, ddq_ref, s)
        pinfo = struct('policy_updated',true,'gate_active',false,'gate_ratio',NaN);
        mode_str = nn.mode;
        mode_l = lower(char(nn.mode));

        if any(strcmp(mode_l, {'profile','ptp','bus_ptp_dp','topp_dp'}))
            assert(isfield(nn,'profile') && isstruct(nn.profile), 'profile mode requires nn.profile');
            a0 = profile_alpha_lookup(nn.profile, s, traj.Tf, double(nn.alpha_floor));
            pinfo.profile = rmfield_safe(nn.profile, {'a','s'});
            mode_str = 'bus_ptp_dp_policy';
            return;
        end

        if any(strcmp(mode_l, {'mpc','mpc_rg','mpc_rg_grid'}))
            mpc_opts = getfield_def(nn,'mpc',struct());
            mpc_state = struct('s',s,'a_prev',state.a_prev,'Pgrid_lp',state.Pgrid_lp);
            [a0, minfo] = spinn3d_mpc_rg_grid_step(mpc_state, traj, robot, caps, bus, fric, areg, policyDt, mpc_opts);
            pinfo.mpc_info = minfo;
            mode_str = 'mpc_rg_grid_policy';
            return;
        end

        if strcmp(mode_l,'feasible')
            [a_star, finfo] = feasible_alpha(q_ref, dq_ref, ddq_ref);
            a0 = alpha_des_teacher(a_star, state.a_prev, areg.BETA, areg.A_DOT_UP, areg.A_DOT_DN, Ts);
            pinfo.a_star = a_star;
            pinfo.feasible_info = finfo;
            mode_str = 'feasible_policy';
            return;
        end

        % Optional gate; default is disabled.  If fallback='feasible_cap', this
        % intentionally becomes a separate TRF+guard variant rather than pure TRF.
        gate_hit = false;
        gate_ratio = NaN;
        if gate.enable && isfinite(Pbud) && Pbud > 0
            gate_ratio = state.Pgrid_lp / max(Pbud, 1e-12);
            if state.gate_active
                if gate_ratio > gate.off_ratio, state.gate_active = false; end
            else
                if gate_ratio < gate.on_ratio, state.gate_active = true; end
            end
            gate_hit = state.gate_active;
        else
            state.gate_active = false;
        end
        pinfo.gate_active = gate_hit;
        pinfo.gate_ratio = gate_ratio;

        if gate_hit && gate.skip_forward
            if strcmp(gate.fallback,'feasible_cap')
                [a_star, finfo] = feasible_alpha(q_ref, dq_ref, ddq_ref);
                a0 = alpha_des_teacher(a_star, state.a_prev, areg.BETA, areg.A_DOT_UP, areg.A_DOT_DN, Ts);
                pinfo.a_star_gate = a_star;
                pinfo.feasible_info = finfo;
                mode_str = 'nn_gate_feasible_policy';
            else
                a0 = clamp(double(gate.a_nom), double(nn.alpha_floor), 1.0);
                mode_str = 'nn_gate_nom_policy';
            end
        else
            a0 = nn_forward_alpha(q_ref, dq_ref, ddq_ref, s);
            mode_str = 'nn_policy';
        end
    end

    function [a_star, finfo] = feasible_alpha(q_ref, dq_ref, ddq_ref)
        caps_feas = caps;
        caps_feas.bus = bus;
        caps_feas.eta_share = bus.eta_share;
        caps_feas.P_brk_peak = bus.P_brk_peak;
        caps_feas.P_grid_peak_max = bus.P_grid_peak_max;
        caps_feas.alpha_prev = state.a_prev;
        caps_feas.alpha_Ts = Ts;
        caps_feas.use_alpha_dot = logical(getfield_def(nn,'feasible_use_alpha_dot',true));
        caps_feas.feasible_grid_N = getfield_def(nn,'feasible_grid_N',61);
        [a_star, finfo] = alpha_feasible(q_ref, dq_ref, ddq_ref, 1.0, robot, caps_feas, Pbud, ...
                                         double(nn.alpha_floor), double(nn.itmax), fric);
        a_star = clamp(double(a_star), double(nn.alpha_floor), 1.0);
    end
    function [a_adj, binfo] = budget_alpha_command(a_in, mode_str_local, callDt)
        % Low-cost budget servo applied at the policy update rate.  It trims
        % optimistic commands when the single LPF power monitor is high and
        % recovers unused budget when the same LPF is below target.
        %
        % Important: this helper is called both at policy updates (Tpol) and,
        % when servo_rate_feedback is enabled, at every 1-ms servo step.  The
        % recovery/cut increments below are therefore scaled by the actual call
        % interval.  Older code used policyDt even in the servo-rate call, which
        % made the boost channel roughly Tpol/Ts times too aggressive.
        if nargin < 3 || ~isfinite(callDt) || callDt <= 0
            callDt = policyDt;
        end
        dtScale = max(0, min(1, double(callDt) / max(double(policyDt), eps)));

        a_raw0 = clamp(double(a_in), double(nn.alpha_floor), 1.0);
        a_raw = apply_policy_command_scale(a_raw0, mode_str_local);
        a_adj = a_raw;
        ratio_lpf = state.Pgrid_lp / max(Pbud, 1e-12);
        ratio_slow = ratio_lpf;  % legacy alias
        ratio_fast = ratio_lpf;  % legacy alias
        raw_ratio = state.Pgrid_last / max(Pbud, 1e-12);
        binfo = struct('enabled',false,'action','pass','mode',char(mode_str_local), ...
                       'ratio_lpf',ratio_lpf,'ratio_slow',ratio_slow,'ratio_fast',ratio_fast, ...
                       'raw_ratio',raw_ratio, ...
                       'target_ratio',bctrl.target_ratio, 'a_raw_unscaled',a_raw0, ...
                       'call_dt',double(callDt), 'dt_scale',dtScale, ...
                       'a_raw',a_raw, 'a_cmd',a_adj);

        if ~bctrl.enable || ~isfinite(Pbud) || Pbud <= 0
            return;
        end

        ml = lower(char(mode_str_local));
        apply = false;
        if ~isempty(strfind(ml,'nn')),       apply = apply || bctrl.apply_to_nn; end %#ok<STREMP>
        if ~isempty(strfind(ml,'feasible')), apply = apply || bctrl.apply_to_feasible; end %#ok<STREMP>
        if ~isempty(strfind(ml,'profile')) || ~isempty(strfind(ml,'ptp')) || ~isempty(strfind(ml,'topp'))
            apply = apply || bctrl.apply_to_profile;
        end
        if ~isempty(strfind(ml,'mpc')),      apply = apply || bctrl.apply_to_mpc; end %#ok<STREMP>
        if ~apply
            return;
        end

        target = min(max(double(bctrl.target_ratio),0.05),1.20);
        high   = min(max(double(bctrl.soft_high_ratio),target),1.50);
        low    = min(max(double(bctrl.recover_low_ratio),0.01),target);
        gamma  = max(double(bctrl.gamma),0.25);
        action = 'pass';

        % Hard trim channel: the paper metric is the single LPF signal, but a
        % raw electrical draw far above the same budget is an early warning that
        % the LPF will rise.  This cheap guard makes the TRF/MLP command react
        % before the filtered monitor has already crossed the line.
        if (isfinite(ratio_lpf) && ratio_lpf > double(bctrl.hard_high_ratio)) || ...
           (isfinite(raw_ratio)  && raw_ratio  > double(bctrl.raw_high_ratio))
            a_adj = double(nn.alpha_floor);
            action = 'hard_trim';
        elseif isfinite(ratio_lpf) && ratio_lpf > high
            % LPF-measured power is above the desired envelope: cap the command
            % using a local P~alpha^gamma scaling.  The normal governor still
            % enforces slew-rate/alpha smoothing at the 1-ms servo rate.
            excess = ratio_lpf / max(target,1e-9);
            scale = excess.^(-double(bctrl.cut_gain)/gamma);
            a_cap = state.a_prev * scale;
            if isfinite(bctrl.max_cut_per_policy) && bctrl.max_cut_per_policy > 0
                maxCutThisCall = double(bctrl.max_cut_per_policy) * dtScale;
                a_cap = max(a_cap, state.a_prev - maxCutThisCall);
            end
            a_adj = min(a_raw, a_cap);
            action = 'trim';
        elseif bctrl.enable_boost && isfinite(ratio_lpf) && ratio_lpf < low
            % Budget is under-used: recover speed gradually.  This prevents a
            % permanently conservative TRF/MLP from leaving the bus budget unused.
            inc = double(bctrl.boost_gain) * max(target - ratio_lpf, 0) * double(callDt);
            if isfinite(bctrl.max_boost_per_policy) && bctrl.max_boost_per_policy > 0
                maxBoostThisCall = double(bctrl.max_boost_per_policy) * dtScale;
                inc = min(inc, maxBoostThisCall);
            end
            a_boost = min(1.0, state.a_prev + max(inc,0));
            a_adj = max(a_raw, a_boost);
            action = 'boost';
        end

        a_adj = clamp(a_adj, double(nn.alpha_floor), 1.0);
        binfo.enabled = true;
        binfo.action = action;
        binfo.a_cmd = a_adj;
    end



    function [a_cap, pcinfo] = predictive_budget_cap(a_in, mode_str_local, q_meas_l, dq_meas_l, q_ref_l, dq_ref_l, ddq_ref_l)
        %PREDICTIVE_BUDGET_CAP  One-step LPF/raw power cap for progress-rate commands.
        %
        % Peak-fix revision:
        %   The previous cap assumed that a lower alpha always reduces bus power.
        %   With the revised electrical model this is not always true: torque/load
        %   current and CTC alpha-dot transients can make alpha_floor itself a high
        %   power point.  Therefore, when the requested command is not immediately
        %   acceptable we scan a small scalar grid and choose the highest feasible
        %   command.  If no command is feasible, we choose the candidate with the
        %   lowest predicted one-step LPF/raw objective instead of blindly clamping
        %   to alpha_floor.  This keeps the method a deterministic scalar governor,
        %   but avoids the 20--30 W floor-infeasible plateau seen in fast13.
        a_cap = clamp(double(a_in), double(nn.alpha_floor), 1.0);
        pcinfo = struct('enabled',false,'applied',false,'floor_infeasible',false, ...
                        'floor_recovery',false,'grid_used',false,'grid_has_ok',false, ...
                        'mode',char(mode_str_local),'a_in',a_cap,'a_cap',a_cap, ...
                        'Ppred_hi',NaN,'Ppred_floor',NaN,'Ppred_best',NaN, ...
                        'Pbar_next_hi',NaN,'Pbar_next_floor',NaN,'Pbar_next_best',NaN, ...
                        'Praw_limit',NaN,'Plpf_limit',NaN,'J_best',NaN,'a_grid_hi',NaN);

        if ~bctrl.enable || ~logical(getfield_def(bctrl,'predictive_cap_enable',false)) || ...
           ~isfinite(Pbud) || Pbud <= 0
            return;
        end
        ml = lower(char(mode_str_local));
        apply = false;
        if ~isempty(strfind(ml,'nn')),       apply = apply || bctrl.apply_to_nn; end %#ok<STREMP>
        if ~isempty(strfind(ml,'feasible')), apply = apply || bctrl.apply_to_feasible; end %#ok<STREMP>
        if ~isempty(strfind(ml,'profile')) || ~isempty(strfind(ml,'ptp')) || ~isempty(strfind(ml,'topp'))
            apply = apply || bctrl.apply_to_profile;
        end
        if ~isempty(strfind(ml,'mpc')),      apply = apply || bctrl.apply_to_mpc; end %#ok<STREMP>
        if ~apply, return; end

        pcinfo.enabled = true;
        betaP = exp(-Ts / max(double(bctrl.tau), 1e-6));
        lpf_ratio = double(getfield_def(bctrl,'predictive_lpf_ratio',min(0.98,double(bctrl.target_ratio))));
        raw_ratio = double(getfield_def(bctrl,'predictive_raw_ratio',1.05));
        if ~isfinite(lpf_ratio) || lpf_ratio <= 0, lpf_ratio = min(0.98,double(bctrl.target_ratio)); end
        if ~isfinite(raw_ratio) || raw_ratio <= 0, raw_ratio = inf; end
        Plpf_limit = lpf_ratio * double(Pbud);
        Praw_limit = raw_ratio * double(Pbud);
        pcinfo.Plpf_limit = Plpf_limit;
        pcinfo.Praw_limit = Praw_limit;

        [ok_hi, pred_hi] = candidate_budget_ok(a_cap, betaP, Plpf_limit, Praw_limit, ...
                                               q_meas_l, dq_meas_l, q_ref_l, dq_ref_l, ddq_ref_l);
        pcinfo.Ppred_hi = pred_hi.Ppred;
        pcinfo.Pbar_next_hi = pred_hi.Pbar_next;
        if ok_hi
            return;
        end

        lo = double(nn.alpha_floor);
        [ok_lo, pred_lo] = candidate_budget_ok(lo, betaP, Plpf_limit, Praw_limit, ...
                                               q_meas_l, dq_meas_l, q_ref_l, dq_ref_l, ddq_ref_l);
        pcinfo.Ppred_floor = pred_lo.Ppred;
        pcinfo.Pbar_next_floor = pred_lo.Pbar_next;

        % Robust non-monotone scalar scan.  This replaces the old assumption
        % that feasibility in alpha is always a single monotone interval.
        gridN = max(0, round(double(getfield_def(bctrl,'predictive_grid_N',0))));
        if gridN >= 3
            % If the scalar feedback has already hard-trimmed a_in to alpha_floor,
            % scanning only [floor,a_in] would never discover that a slightly faster
            % pass-through command can reduce torque/load-current dwell.  Use the
            % held raw policy command and current applied alpha as the upper scan
            % bound, while the final selected command is still checked by the same
            % CTC+bus predictor.
            hi_scan = clamp(max([a_cap, double(state.a_des_hold_raw), double(state.a_prev)]), lo, 1.0);
            pcinfo.a_grid_hi = hi_scan;
            [a_sel, sinfo] = predictive_grid_select(lo, hi_scan, gridN, betaP, Plpf_limit, Praw_limit, ...
                                                     q_meas_l, dq_meas_l, q_ref_l, dq_ref_l, ddq_ref_l);
            pcinfo.grid_used = true;
            pcinfo.grid_has_ok = sinfo.has_ok;
            pcinfo.floor_recovery = sinfo.floor_recovery;
            pcinfo.Ppred_best = sinfo.Ppred_best;
            pcinfo.Pbar_next_best = sinfo.Pbar_next_best;
            pcinfo.J_best = sinfo.J_best;
            a_cap = clamp(a_sel, double(nn.alpha_floor), 1.0);
            pcinfo.a_cap = a_cap;
            pcinfo.floor_infeasible = ~sinfo.has_ok && ~ok_lo;
            pcinfo.applied = abs(a_cap - double(a_in)) > 1e-12;
            return;
        end

        % Backward-compatible bisection path if grid scan is disabled.
        if ~ok_lo
            a_cap = lo;
            pcinfo.applied = (a_cap < double(a_in) - 1e-12);
            pcinfo.floor_infeasible = true;
            pcinfo.a_cap = a_cap;
            return;
        end

        hi = a_cap;
        nit = max(4, round(double(getfield_def(bctrl,'predictive_iters',14))));
        for jj = 1:nit
            mid = 0.5*(lo + hi);
            [ok_mid, ~] = candidate_budget_ok(mid, betaP, Plpf_limit, Praw_limit, ...
                                              q_meas_l, dq_meas_l, q_ref_l, dq_ref_l, ddq_ref_l);
            if ok_mid
                lo = mid;
            else
                hi = mid;
            end
        end
        a_cap = clamp(lo, double(nn.alpha_floor), 1.0);
        pcinfo.applied = (a_cap < double(a_in) - 1e-12);
        pcinfo.a_cap = a_cap;
    end

    function [a_sel, sinfo] = predictive_grid_select(lo, hi, gridN, betaP, Plpf_limit, Praw_limit, q_meas_g, dq_meas_g, q_ref_g, dq_ref_g, ddq_ref_g)
        aa = linspace(double(lo), double(hi), max(3,gridN));
        okv = false(size(aa));
        Jv = inf(size(aa));
        Ppredv = nan(size(aa));
        Pbarv = nan(size(aa));
        apostv = nan(size(aa));
        raw_w   = max(0, double(getfield_def(bctrl,'floor_fallback_raw_weight',0.50)));
        move_w  = max(0, double(getfield_def(bctrl,'floor_fallback_move_weight',0.03)));
        speed_w = max(0, double(getfield_def(bctrl,'floor_fallback_speed_weight',0.01)));
        for ii = 1:numel(aa)
            [ok_i, pred_i] = candidate_budget_ok(aa(ii), betaP, Plpf_limit, Praw_limit, ...
                                                 q_meas_g, dq_meas_g, q_ref_g, dq_ref_g, ddq_ref_g);
            okv(ii) = ok_i;
            Ppredv(ii) = pred_i.Ppred;
            Pbarv(ii) = pred_i.Pbar_next;
            apostv(ii) = pred_i.a_post;
            raw_excess = max(pred_i.Ppred/max(Praw_limit,1e-12) - 1.0, 0.0);
            Jv(ii) = pred_i.Pbar_next/max(Plpf_limit,1e-12) + raw_w*raw_excess.^2 + ...
                     move_w*abs(pred_i.a_post - state.a_prev) - speed_w*pred_i.a_post;
        end
        if any(okv)
            % Choose the largest command that is predicted to satisfy both raw
            % and LPF limits.  This preserves speed without relying on monotonicity.
            idx_ok = find(okv);
            [~, jj] = max(apostv(idx_ok));
            idx = idx_ok(jj);
            has_ok = true;
        else
            % No feasible candidate exists.  Do not blindly hold alpha_floor;
            % choose the command with the smallest predicted power objective.
            [~, idx] = min(Jv);
            has_ok = false;
        end
        a_sel = aa(idx);
        sinfo = struct('has_ok',has_ok, 'floor_recovery',(~has_ok && a_sel > double(nn.alpha_floor)+1e-12), ...
                       'Ppred_best',Ppredv(idx), 'Pbar_next_best',Pbarv(idx), ...
                       'J_best',Jv(idx), 'a_post_best',apostv(idx));
    end

    function [ok, pred] = candidate_budget_ok(a_des_c, betaP, Plpf_limit, Praw_limit, q_meas_c, dq_meas_c, q_ref_c, dq_ref_c, ddq_ref_c)
        [a_c, ~, ~, adot_c, ~] = governor_alpha_step(a_des_c, state.a_prev, Ts, dq_ref_c, ddq_ref_c);
        dq_cmd_c  = a_c * dq_ref_c;
        ddq_cmd_c = a_c*a_c * ddq_ref_c + adot_c * dq_ref_c;
        e_c  = q_ref_c - q_meas_c;
        ed_c = dq_cmd_c - dq_meas_c;
        v_c  = ddq_cmd_c + Kd.*ed_c + Kp.*e_c;

        Mq_c = massMatrix(robot, q_meas_c);
        c_c  = velocityProduct(robot, q_meas_c, dq_meas_c);
        g_c  = gravityTorque(robot, q_meas_c);
        tau_c = (Mq_c * v_c(:) + c_c(:) + g_c(:)).';
        if any(fric.B) || any(fric.Fc)
            tau_c = tau_c + fric.B.*dq_meas_c + fric.Fc.*tanh(dq_meas_c./max(fric.vel_eps,1e-9));
        end
        tau_c = clamp_vec(tau_c, -caps.tau_max, caps.tau_max);

        [~,~,P0,~,~,~] = spinn3d_bus_power(tau_c, dq_meas_c, bus);
        Ppred = double(P0);
        if logical(getfield_def(bctrl,'predictive_use_dq_next',true))
            tau_fric_c = fric.B.*dq_meas_c + fric.Fc.*tanh(dq_meas_c./max(fric.vel_eps,1e-9));
            ddq_dyn_c = (Mq_c \ (tau_c(:) - c_c(:) - g_c(:) - tau_fric_c(:))).';
            dq_next_c = dq_meas_c + Ts * ddq_dyn_c;
            [~,~,P1,~,~,~] = spinn3d_bus_power(tau_c, dq_next_c, bus);
            Ppred = max(Ppred, double(P1));
        end
        Pbar_next = betaP * state.Pgrid_lp + (1-betaP) * Ppred;
        ok = isfinite(Ppred) && isfinite(Pbar_next) && ...
             (Ppred <= Praw_limit + 1e-9) && (Pbar_next <= Plpf_limit + 1e-9);
        pred = struct('a_des',a_des_c,'a_post',a_c,'adot',adot_c, ...
                      'Ppred',Ppred,'Pbar_next',Pbar_next);
    end

    function a_scaled = apply_policy_command_scale(a_raw_in, mode_str_local)
        % Optional calibrated shrinkage for learned feed-forward commands.
        % The scalar LPF budget servo can still boost the command when the
        % measured bus power is below target, so this prevents only the launch
        % overshoot seen in TRF rollouts rather than forcing permanent
        % under-utilization.
        a_scaled = a_raw_in;
        ml = lower(char(mode_str_local));
        sc = 1.0;
        if ~isempty(strfind(ml,'nn')) %#ok<STREMP>
            if is_mlp_model()
                sc = double(bctrl.mlp_command_scale);
            else
                sc = double(bctrl.trf_command_scale);
            end
        elseif ~isempty(strfind(ml,'feasible')) %#ok<STREMP>
            sc = double(bctrl.feasible_command_scale);
        end
        if isfinite(sc) && abs(sc-1.0) > 1e-12
            sc = max(sc,0);
            a_scaled = double(nn.alpha_floor) + sc * (a_raw_in - double(nn.alpha_floor));
            a_scaled = clamp(a_scaled, double(nn.alpha_floor), 1.0);
        end
    end

    function tf = is_mlp_model()
        tf = false;
        try
            mk = lower(char(getfield_def(nn.model,'type', getfield_def(nn.model.preproc,'kind',''))));
            tf = ~isempty(strfind(mk,'mlp')) || logical(getfield_def(nn.model.preproc,'flatten',false)); %#ok<STREMP>
        catch
            tf = false;
        end
    end


    function a0 = nn_forward_alpha(q_ref, dq_ref, ddq_ref, s)
        assert(isfield(nn,'model') && isstruct(nn.model), 'nn mode requires nn.model');
        assert(isfield(nn.model,'preproc') && isstruct(nn.model.preproc), 'model missing preproc');
        assert(isfield(nn.model.preproc,'keepBaseIdx') && ~isempty(nn.model.preproc.keepBaseIdx), ...
               'model missing preproc.keepBaseIdx');

        kb = logical(nn.model.preproc.keepBaseIdx(:).');
        Klook = getfield_def(nn.model.preproc,'K',1);
        Klook = max(1, round(double(Klook)));

        % Build the same K-token normalized feature tensor used by TRF.
        Xfull = [];
        if Klook > 1
            ds = getfield_def(nn.model.preproc,'ds', Ts);
            if ~isfinite(ds) || ds <= 0
                % If ds is unavailable, span the preview over the remaining normalized path.
                ds = max(traj.Tf,1e-9) / max(Klook-1,1);
            end
            s_norm0 = min(1.0, s / max(traj.Tf, 1e-9));
            x1 = nn.featureFcn(q_ref, dq_ref, ddq_ref, robot, nn.payload, Pbud, caps, ...
                               state.a_prev, areg.BETA, areg.A_DOT_UP, areg.A_DOT_DN, Ts, s_norm0);
            x1 = row(double(x1));
            Dtot = numel(x1); Dtail = 6; Dbase = Dtot - Dtail;
            assert(numel(kb)==Dbase, 'keepBaseIdx length mismatch');
            Xfull = zeros(Klook, Dtot);
            Xfull(1,:) = x1;
            for ti = 2:Klook
                s_i = min(traj.Tf, s + (ti-1)*ds);
                [qi, dqgi, ddqgi] = traj.eval(s_i);
                s_norm_i = min(1.0, s_i / max(traj.Tf, 1e-9));
                xi = nn.featureFcn(row(qi), row(dqgi), row(ddqgi), robot, nn.payload, Pbud, caps, ...
                                   state.a_prev, areg.BETA, areg.A_DOT_UP, areg.A_DOT_DN, Ts, s_norm_i);
                Xfull(ti,:) = row(double(xi));
            end
        else
            s_norm = min(1.0, s / max(traj.Tf, 1e-9));
            x_full = nn.featureFcn(q_ref, dq_ref, ddq_ref, robot, nn.payload, Pbud, caps, ...
                                   state.a_prev, areg.BETA, areg.A_DOT_UP, areg.A_DOT_DN, Ts, s_norm);
            x_full = row(double(x_full));
            Dtot = numel(x_full); Dtail = 6; Dbase = Dtot - Dtail;
            assert(numel(kb)==Dbase, 'keepBaseIdx length mismatch');
            Xfull = reshape(x_full,1,[]);
        end

        Dtail = 6; Dtot = size(Xfull,2); Dbase = Dtot-Dtail;
        Xbase = Xfull(:,1:Dbase);
        Xtail = Xfull(:,Dbase+1:end);
        Xc = [Xbase(:,kb), Xtail];
        Din = size(Xc,2);

        model_kind = lower(char(getfield_def(nn.model,'type', getfield_def(nn.model.preproc,'kind','trf_lookahead'))));
        is_mlp = contains(model_kind,'mlp') || getfield_def(nn.model.preproc,'flatten',false);

        if is_mlp
            % MLP-PRG: flatten the same look-ahead tensor and run the compact non-attention learned surrogate.
            xflat = reshape(Xc.', 1, []);  % token-major: [token1 features, token2 features, ...]
            mu = row(double(nn.model.preproc.mu));
            sig = row(double(nn.model.preproc.sig));
            assert(numel(mu)==numel(xflat) && numel(sig)==numel(xflat), 'MLP normalization dimension mismatch');
            z = (xflat - mu) ./ max(sig, 1e-12);
            y = predict_alpha_from_any_net(nn.model.net, z);
            if getfield_def(nn.model.preproc,'target_normalized',false)
                amin_m = double(getfield_def(nn.model.preproc,'alpha_min',double(nn.alpha_floor)));
                y = amin_m + (1-amin_m).*double(y);
            end
        elseif Klook > 1
            mu = row(double(nn.model.preproc.mu));
            sig = row(double(nn.model.preproc.sig));
            assert(numel(mu)==Din && numel(sig)==Din, 'feature normalization dimension mismatch');
            Z = (Xc - mu) ./ max(sig, 1e-12);
            dlX = dlarray(single(reshape(Z.', [Din, 1, Klook])), 'CBT');
            y = extractdata(forward(nn.model.net, dlX));
        else
            x = Xc(1,:);
            mu = row(double(nn.model.preproc.mu));
            sig = row(double(nn.model.preproc.sig));
            assert(numel(mu)==numel(x) && numel(sig)==numel(x), 'feature normalization dimension mismatch');
            z = (x - mu) ./ max(sig, 1e-12);
            dlX = dlarray(single(z'), 'CB');
            y = extractdata(forward(nn.model.net, dlX));
        end

        y = double(y(:));
        if isempty(y), y = double(nn.alpha_floor); end
        a0 = clamp(y(1), double(nn.alpha_floor), 1.0);
    end

    function [a, a_slew, a_filt, adot, G] = governor_alpha_step(a_des, a_prev, Ts_local, dq_ref, ddq_ref)
        a_des = clamp(a_des, double(nn.alpha_floor), 1.0);
        sf_qd  = min_safe(caps.qd_max  ./ max(abs(dq_ref),  1e-12));
        sf_qdd = min_safe(sqrt(caps.qdd_max ./ max(abs(ddq_ref),1e-12)));
        a_kin  = min([1.0, sf_qd, sf_qdd]);

        % Important ordering: cap the target before the slew/LPF.  The old
        % ordering filtered a high command and then hard-clipped by a_kin,
        % which produced saw-tooth alpha and large adot*q_s torque pulses.
        a_target = clamp(min(a_des, a_kin), double(nn.alpha_floor), 1.0);

        a_slew = min(a_target, a_prev + areg.A_DOT_UP*Ts_local);
        a_slew = max(a_slew,  a_prev - areg.A_DOT_DN*Ts_local);
        a_filt = areg.BETA*a_prev + (1-areg.BETA)*a_slew;
        a = clamp(a_filt, double(nn.alpha_floor), 1.0);
        if a > a_kin
            a = clamp(a_kin, double(nn.alpha_floor), 1.0);
        end
        adot = (a - a_prev) / max(Ts_local, 1e-12);
        G = struct('a_des',a_des,'a_target',a_target,'a_slew',a_slew,'a_filt',a_filt,'a_kin',a_kin);
    end

end


function b = budget_ctrl_defaults(b)
%BUDGET_CTRL_DEFAULTS  Defaults for scalar budget-utilization feedback.
if nargin < 1 || isempty(b), b = struct(); end
if ~isfield(b,'enable') || isempty(b.enable), b.enable = false; end
if ~isfield(b,'apply_to_nn') || isempty(b.apply_to_nn), b.apply_to_nn = true; end
if ~isfield(b,'apply_to_feasible') || isempty(b.apply_to_feasible), b.apply_to_feasible = true; end
if ~isfield(b,'apply_to_profile') || isempty(b.apply_to_profile), b.apply_to_profile = true; end
if ~isfield(b,'apply_to_mpc') || isempty(b.apply_to_mpc), b.apply_to_mpc = false; end
if ~isfield(b,'tau') || isempty(b.tau)
    if isfield(b,'tau_power') && ~isempty(b.tau_power)
        b.tau = b.tau_power;
    elseif isfield(b,'tau_fast') && ~isempty(b.tau_fast)
        b.tau = b.tau_fast;
    elseif isfield(b,'tau_slow') && ~isempty(b.tau_slow)
        b.tau = b.tau_slow;
    else
        b.tau = 0.05;
    end
end
if ~isfield(b,'tau_power') || isempty(b.tau_power), b.tau_power = b.tau; end
if ~isfield(b,'tau_slow') || isempty(b.tau_slow), b.tau_slow = b.tau; end
if ~isfield(b,'tau_fast') || isempty(b.tau_fast), b.tau_fast = b.tau; end
if ~isfield(b,'target_ratio') || isempty(b.target_ratio), b.target_ratio = 1.00; end
if ~isfield(b,'soft_high_ratio') || isempty(b.soft_high_ratio), b.soft_high_ratio = 1.02; end
if ~isfield(b,'fast_high_ratio') || isempty(b.fast_high_ratio), b.fast_high_ratio = 1.12; end
if ~isfield(b,'recover_low_ratio') || isempty(b.recover_low_ratio), b.recover_low_ratio = 0.97; end
if ~isfield(b,'gamma') || isempty(b.gamma), b.gamma = 2.0; end
if ~isfield(b,'cut_gain') || isempty(b.cut_gain), b.cut_gain = 1.2; end
if ~isfield(b,'boost_gain') || isempty(b.boost_gain), b.boost_gain = 8.0; end
if ~isfield(b,'max_boost_per_policy') || isempty(b.max_boost_per_policy), b.max_boost_per_policy = 0.12; end
if ~isfield(b,'max_cut_per_policy') || isempty(b.max_cut_per_policy), b.max_cut_per_policy = 0.30; end
if ~isfield(b,'enable_boost') || isempty(b.enable_boost), b.enable_boost = true; end
if ~isfield(b,'init_lp_ratio') || isempty(b.init_lp_ratio), b.init_lp_ratio = 0.60; end
if ~isfield(b,'servo_rate_feedback') || isempty(b.servo_rate_feedback), b.servo_rate_feedback = true; end
if ~isfield(b,'raw_high_ratio') || isempty(b.raw_high_ratio), b.raw_high_ratio = 1.05; end
if ~isfield(b,'hard_high_ratio') || isempty(b.hard_high_ratio), b.hard_high_ratio = 1.02; end
if ~isfield(b,'trf_command_scale') || isempty(b.trf_command_scale), b.trf_command_scale = 1.0; end
if ~isfield(b,'mlp_command_scale') || isempty(b.mlp_command_scale), b.mlp_command_scale = 1.0; end
if ~isfield(b,'feasible_command_scale') || isempty(b.feasible_command_scale), b.feasible_command_scale = 1.0; end
if ~isfield(b,'predictive_cap_enable') || isempty(b.predictive_cap_enable), b.predictive_cap_enable = false; end
if ~isfield(b,'predictive_lpf_ratio') || isempty(b.predictive_lpf_ratio), b.predictive_lpf_ratio = min(0.98,double(b.target_ratio)); end
if ~isfield(b,'predictive_raw_ratio') || isempty(b.predictive_raw_ratio), b.predictive_raw_ratio = 1.05; end
if ~isfield(b,'predictive_iters') || isempty(b.predictive_iters), b.predictive_iters = 14; end
if ~isfield(b,'predictive_use_dq_next') || isempty(b.predictive_use_dq_next), b.predictive_use_dq_next = true; end
if ~isfield(b,'predictive_grid_N') || isempty(b.predictive_grid_N), b.predictive_grid_N = 0; end
if ~isfield(b,'floor_fallback_mode') || isempty(b.floor_fallback_mode), b.floor_fallback_mode = 'min_power_grid'; end
if ~isfield(b,'floor_fallback_raw_weight') || isempty(b.floor_fallback_raw_weight), b.floor_fallback_raw_weight = 0.50; end
if ~isfield(b,'floor_fallback_move_weight') || isempty(b.floor_fallback_move_weight), b.floor_fallback_move_weight = 0.03; end
if ~isfield(b,'floor_fallback_speed_weight') || isempty(b.floor_fallback_speed_weight), b.floor_fallback_speed_weight = 0.01; end
b.enable = logical(b.enable);
b.apply_to_nn = logical(b.apply_to_nn);
b.apply_to_feasible = logical(b.apply_to_feasible);
b.apply_to_profile = logical(b.apply_to_profile);
b.apply_to_mpc = logical(b.apply_to_mpc);
b.enable_boost = logical(b.enable_boost);
b.servo_rate_feedback = logical(b.servo_rate_feedback);
b.predictive_cap_enable = logical(b.predictive_cap_enable);
b.predictive_use_dq_next = logical(b.predictive_use_dq_next);
b.tau = max(double(b.tau),1e-6);
b.tau_power = max(double(b.tau_power),1e-6);
% If tau_power was supplied, it is the authoritative single LPF time constant.
if isfield(b,'tau_power') && isfinite(b.tau_power)
    b.tau = b.tau_power;
end
b.tau_slow = b.tau;  % legacy alias
b.tau_fast = b.tau;  % legacy alias
b.target_ratio = double(b.target_ratio);
b.soft_high_ratio = double(b.soft_high_ratio);
b.fast_high_ratio = double(b.fast_high_ratio);
b.recover_low_ratio = double(b.recover_low_ratio);
b.gamma = max(double(b.gamma),0.25);
b.cut_gain = max(double(b.cut_gain),0);
b.boost_gain = max(double(b.boost_gain),0);
b.max_boost_per_policy = max(double(b.max_boost_per_policy),0);
b.max_cut_per_policy = max(double(b.max_cut_per_policy),0);
b.init_lp_ratio = min(max(double(b.init_lp_ratio),0),1.5);
b.raw_high_ratio = max(double(b.raw_high_ratio),0.05);
b.hard_high_ratio = max(double(b.hard_high_ratio),0.05);
b.trf_command_scale = max(double(b.trf_command_scale),0);
b.mlp_command_scale = max(double(b.mlp_command_scale),0);
b.feasible_command_scale = max(double(b.feasible_command_scale),0);
b.predictive_lpf_ratio = max(double(b.predictive_lpf_ratio),0.01);
b.predictive_raw_ratio = max(double(b.predictive_raw_ratio),0.01);
b.predictive_iters = max(4, round(double(b.predictive_iters)));
b.predictive_grid_N = max(0, round(double(b.predictive_grid_N)));
b.floor_fallback_raw_weight = max(0,double(b.floor_fallback_raw_weight));
b.floor_fallback_move_weight = max(0,double(b.floor_fallback_move_weight));
b.floor_fallback_speed_weight = max(0,double(b.floor_fallback_speed_weight));
end


function y = predict_alpha_from_any_net(net, z)
%PREDICT_ALPHA_FROM_ANY_NET  Support dlnetwork, SeriesNetwork/DAGNetwork, or function handle.
if isa(net,'function_handle')
    y = net(z);
    return;
end
try
    if isa(net,'dlnetwork')
        y = extractdata(forward(net, dlarray(single(z'), 'CB')));
    else
        y = predict(net, single(z));
    end
catch
    % Some network objects expect double rows.
    y = predict(net, z);
end
end

function a = profile_alpha_lookup(profile, s, Tf, alpha_floor)
sp = double(profile.s(:));
ap = double(profile.a(:));
assert(~isempty(sp) && numel(sp)==numel(ap), 'bad profile fields s/a');
% Accept either absolute s-grid [0,Tf] or normalized [0,1].
if max(sp) <= 1.0001 && Tf > 1.0001
    sq = s / max(Tf,1e-12);
else
    sq = s;
end
a = interp1(sp, ap, sq, 'linear', 'extrap');
a = min(1.0, max(alpha_floor, a));
end

function S = rmfield_safe(S, fields)
for i = 1:numel(fields)
    if isfield(S, fields{i}), S = rmfield(S, fields{i}); end
end
end

% ================= helpers =================
function n = local_num_joints(robot), n = numel(homeConfiguration(robot)); end
function caps = normalize_caps(lim, nJ)
caps = struct();
fn = {'tau_max','qd_max','qdd_max','P_axis_max'};
for i = 1:numel(fn)
    if isfield(lim,fn{i}) && ~isempty(lim.(fn{i}))
        v = double(lim.(fn{i})(:).');
        if isscalar(v), v = repmat(v,1,nJ); end
        caps.(fn{i}) = v(1:nJ);
    else
        caps.(fn{i}) = inf(1,nJ);
    end
end
if isfield(lim,'countRegen'), caps.countRegen = logical(lim.countRegen); end
if isfield(lim,'P_grid_budget'), caps.P_grid_budget = lim.P_grid_budget; end
if isfield(lim,'P_total_max'), caps.P_total_max = lim.P_total_max; end
if isfield(lim,'P_grid_peak_max'), caps.P_grid_peak_max = lim.P_grid_peak_max; end
end
function v = vec_row(x, n), x = double(x); if isscalar(x), v = repmat(x,1,n); else, v = x(:).'; end, v = v(1:n); end
function v = reqf(S, k), assert(isfield(S,k) && ~isempty(S.(k)), ['missing field: ',k]); v = S.(k); end
function x = getfield_def(S, k, d), if isstruct(S)&&isfield(S,k)&&~isempty(S.(k)), x=S.(k); else, x=d; end, end
function r = row(v), r = double(v(:)).'; end
function y = clamp(x, lo, hi), y = min(hi, max(lo, x)); end
function y = clamp_vec(x, lo, hi), y = min(hi, max(lo, x)); end
function m = min_safe(v), v = v(isfinite(v)); if isempty(v), m = 1.0; else, m = min(v(:)); end, end
