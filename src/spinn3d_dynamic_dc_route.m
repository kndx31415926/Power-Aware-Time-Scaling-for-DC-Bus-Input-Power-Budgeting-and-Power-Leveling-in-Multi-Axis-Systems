function R = spinn3d_dynamic_dc_route(Pterminal,bus)
%SPINN3D_DYNAMIC_DC_ROUTE Shared numeric/dlarray common-link drive routing.
% Pshare is only a simultaneous-use statistic. The injected regeneration is
% added ONCE to the capacitor node balance. Pshare_peak_max_W retains its

% Source-side auxiliaries cannot consume regenerated power.
R=struct();
R.Pload_axis=sum(max(Pterminal,0),1);
R.Pterminal_generation=sum(max(-Pterminal,0),1);
R.Pregen_converted=min(R.Pterminal_generation.*bus.eta_regen,bus.Pregen_peak_max_W);
R.Pregen_available=R.Pregen_converted.*bus.eta_share;
if bus.regen_enable && bus.direct_share_enable && ...
        strcmp(char(bus.routing_mode),'demand_limited_direct')
    R.Pregen_injected=min(R.Pregen_available,bus.Pshare_peak_max_W);
else
    R.Pregen_injected=R.Pregen_available.*0;
end
R.Pshare_direct=min(R.Pregen_injected,R.Pload_axis);
R.Plocal_brake=R.Pregen_available-R.Pregen_injected;
R.Pconversion_loss=R.Pterminal_generation-R.Pregen_converted;
R.Psharing_loss=R.Pregen_converted-R.Pregen_available;
end
