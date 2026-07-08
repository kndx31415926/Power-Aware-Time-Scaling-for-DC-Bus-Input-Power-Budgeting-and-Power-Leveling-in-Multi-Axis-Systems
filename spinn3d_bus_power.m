function [Ppos, Pneg, Pgrid, Pdump, Paxis, flow] = spinn3d_bus_power(tau, dq, bus)
%SPINN3D_BUS_POWER  Compatibility wrapper for DC-bus power.
% If bus.use_electrical_model is true (default in the revised parameter
% block), this uses spinn3d_bus_power_elec without state propagation.  The
% first five outputs keep the old function signature; the optional sixth
% output is a telemetry struct.
    if nargin < 3 || isempty(bus), bus = struct(); end
    bus = spinn3d_bus_defaults(bus);
    if isfield(bus,'use_electrical_model') && bus.use_electrical_model
        [sig,~] = spinn3d_bus_power_elec(tau, dq, bus, [], 1e-3);
        Ppos  = sig.Ppos;
        Pneg  = sig.Pneg;
        Pgrid = sig.Pgrid;
        Pdump = sig.Pdump;
        Paxis = sig.Paxis;
        if nargout > 5, flow = sig; end
        return;
    end

    tau   = row(tau);
    dq    = row(dq);
    Paxis = tau .* dq;
    Ppos  = sum(max(Paxis,0));
    Pneg  = sum(max(-Paxis,0));
    eta   = getfield_def(bus,'eta_share',1.0);
    share = min(eta * Pneg, Ppos);
    Pgrid = max(Ppos - eta * Pneg, 0);
    Pdump = max(Pneg - share, 0);
    if nargout > 5
        Vnom = getfield_def(bus,'V_nom',12);
        flow = struct('Paxis',Paxis,'Ppos',Ppos,'Pneg',Pneg,'Pgrid',Pgrid,'Pdump',Pdump, ...
                      'Vdc',Vnom,'Igrid',Pgrid/max(Vnom,eps),'Ibatt',Pgrid/max(Vnom,eps), ...
                      'Ploss',0,'Pspill',0);
    end
end

function y = row(x), y = x(:)'; end
function v = getfield_def(S,fn,def), if isstruct(S)&&isfield(S,fn)&&~isempty(S.(fn)), v=S.(fn); else, v=def; end, end
