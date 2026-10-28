function ax=spinn3d_plot_dynamic_bus(tlo,log,bus)
%SPINN3D_PLOT_DYNAMIC_BUS Voltage state and stored capacitor energy, including t=0.
ax=[];
if ~isfield(log,'Vdc') || isempty(log.Vdc),return;end
ax=nexttile(tlo,[1 2]);hold(ax,'on');grid(ax,'on');
t=[0;log.t(:)];v=[log.Vdc_pre_t0_V(1);log.Vdc(:)];
yyaxis(ax,'left');
plot(ax,t,v,'LineWidth',1.3,'DisplayName','V_{dc}');
yline(ax,bus.V_min_V,'--','V_{min}','HandleVisibility','off');
yline(ax,bus.V_max_V,'--','V_{max}','HandleVisibility','off');
ylabel(ax,'DC-link voltage [V]');
yyaxis(ax,'right');
plot(ax,t,0.5*bus.C_dc_F*v.^2,'DisplayName','E_{cap}');
ylabel(ax,'Capacitor energy [J]');xlabel(ax,'t [s]');
S=spinn3d_dynamic_bus_summary(log);
title(ax,sprintf('Dynamic DC link | Delta Ecap %.4g J | source-adjusted energy %.4g J', ...
    S.Ecap_delta_J,S.Egrid_storage_adjusted_J));
legend(ax,'show','Location','best');
end
