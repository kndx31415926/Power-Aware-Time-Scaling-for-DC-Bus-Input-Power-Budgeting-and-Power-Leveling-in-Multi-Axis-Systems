function tauP = spinn3d_get_power_lpf_tau(cfg, defaultTau)
%SPINN3D_GET_POWER_LPF_TAU Return the single v5 bus-power LPF time constant.
if nargin < 2 || isempty(defaultTau), defaultTau = 0.05; end
tauP = defaultTau;
if isstruct(cfg)
    if isfield(cfg,'EVAL') && isstruct(cfg.EVAL)
        E = cfg.EVAL;
        tauP = getfield_def(E,'power_tau_s', getfield_def(E,'power_lpf_tau_s', getfield_def(E,'power_eval_tau_s', tauP)));
    end
    if isfield(cfg,'BUDGET_CTRL') && isstruct(cfg.BUDGET_CTRL)
        B = cfg.BUDGET_CTRL;
        tauP = getfield_def(B,'tau_power', getfield_def(B,'tau', tauP));
    end
end
tauP = max(double(tauP), 1e-6);
end
function v = getfield_def(S,fn,def)
if isstruct(S) && isfield(S,fn) && ~isempty(S.(fn)), v = S.(fn); else, v = def; end
end
