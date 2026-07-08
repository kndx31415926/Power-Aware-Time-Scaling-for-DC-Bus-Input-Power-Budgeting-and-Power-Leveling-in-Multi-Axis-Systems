function tau2 = clamp_power_axis_total_bus(tau, dq, P_axis_max, bus)
%CLAMP_POWER_AXIS_TOTAL_BUS  Torque scaling guard using revised bus names.
%
% The active budget is bus.P_grid_budget.  bus.P_total_max remains a legacy
% alias.  This guard first enforces per-axis positive mechanical power and
% then scales all torques if the predicted grid draw exceeds the active
% budget or the instantaneous peak cap.
    if nargin < 4 || isempty(bus), bus = struct(); end
    bus = spinn3d_bus_defaults(bus);
    tau = row(tau); dq = row(dq); nJ = numel(tau);
    Pax_lim = row(P_axis_max);
    if isscalar(Pax_lim), Pax_lim = repmat(Pax_lim,1,nJ); end
    Pax_lim = Pax_lim(1:nJ);

    tau2 = tau;
    Pj = tau2 .* dq;
    for j=1:nJ
        if isfinite(Pax_lim(j)) && Pax_lim(j) >= 0 && Pj(j) > Pax_lim(j) + 1e-12
            denom = max(abs(dq(j)), 1e-12);
            tau2(j) = sign(tau2(j)) * min(abs(tau2(j)), Pax_lim(j)/denom);
        end
    end

    [~,~,Pgrid,~,~] = spinn3d_bus_power(tau2, dq, bus);
    Pcap = min(getfield_def(bus,'P_grid_budget',inf), getfield_def(bus,'P_grid_peak_max',inf));
    if isfinite(Pcap) && Pcap >= 0 && Pgrid > Pcap + 1e-9
        gamma = Pcap / max(Pgrid, 1e-12);
        tau2 = gamma * tau2;
    end
end

function y = row(x), y = x(:)'; end
function v = getfield_def(S,fn,def), if isstruct(S)&&isfield(S,fn)&&~isempty(S.(fn)), v=S.(fn); else, v=def; end, end
