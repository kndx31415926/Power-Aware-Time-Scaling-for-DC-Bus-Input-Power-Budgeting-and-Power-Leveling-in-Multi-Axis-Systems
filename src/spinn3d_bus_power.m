function [Ppos, Pneg, Pgrid, Pdump, Paxis, flow] = spinn3d_bus_power(tau, dq, bus)
% Evaluate motor power, inter-axis sharing and common-source demand.

    if nargin < 3 || isempty(bus), bus = struct(); end
    bus = spinn3d_bus_defaults(bus);

    tau = row(tau);
    dq  = row(dq);
    if strcmp(bus.model_scope,'dynamic_capacitor_common_dc_v1')

        [flow,~]=spinn3d_bus_step(tau,dq,bus,bus.dc_state,bus.dc_step_s);
    else

        [flow,~] = spinn3d_bus_step(tau,dq,bus,[],1.0);
    end
    Paxis = flow.Paxis;
    Ppos = flow.Ppos;
    Pneg = flow.Pneg;
    Pgrid = flow.Pgrid;

    % Dynamic Pdump is a zero compatibility port; this link has no resistor.
    Pdump = flow.Pdump;
end

function y = row(x)
    y = x(:)';
end
