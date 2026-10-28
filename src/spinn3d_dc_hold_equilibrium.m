function E=spinn3d_dc_hold_equilibrium(bus,Pload_axis)
%SPINN3D_DC_HOLD_EQUILIBRIUM Static high-voltage source/load balance.
% This initialization calculation is numeric only. The differentiated
% transient step uses the shared Newton energy kernel, never this bisection.
% Source-side auxiliary power consumes source current but is not a DC-node
% load. No braking resistor or energy sink is present.
bus=spinn3d_bus_defaults(bus);

Vs=bus.V_source_V; Rs=bus.R_source_ohm;
Icap=max(min(bus.I_supply_max_A,bus.P_grid_peak_max/Vs)-bus.P_idle_W/Vs,0);
E=struct('found',false,'Vdc',NaN,'Ilink',NaN,'Isource',NaN, ...
    'Pgrid',NaN,'Pdump',NaN,'Pbus_source',NaN, ...
    'Psource_loss',NaN,'residual_W',NaN);
% On this branch V*I(V)-D is nonincreasing. Lower
% constant-current equilibria are not stable high-voltage hold solutions.
lo=max([bus.V_min_V,Vs-Rs*Icap,Vs/2]);
hi=min(Vs,bus.V_max_V);
if lo>hi, return; end
flo=balance(lo); fhi=balance(hi);
tol=bus.dc_solver_tolerance_W;
if flo < -tol || fhi > tol, return; end
for k=1:80
    mid=(lo+hi)/2;
    if balance(mid)>0, lo=mid; else, hi=mid; end
end
v=(lo+hi)/2;
[i,pd]=ports(v);
residual=v*i-Pload_axis-pd;
E.found=isfinite(v) && abs(residual)<=tol && ...
    v>=bus.V_min_V && v<=bus.V_max_V;
if ~E.found, return; end
E.Vdc=v; E.Ilink=i;
E.Isource=i+bus.P_idle_W/Vs;
E.Pgrid=Vs*i+bus.P_idle_W;
E.Pdump=pd; E.Pbus_source=v*i;
E.Psource_loss=(Vs-v)*i;
E.residual_W=residual;
    function f=balance(vv)
        [ii,pp]=ports(vv);
        f=vv*ii-Pload_axis-pp;
    end
    function [ii,pp]=ports(vv)
        ii=min(max((Vs-vv)/Rs,0),Icap);
        pp=0; % compatibility port: absent in this topology
    end
end
