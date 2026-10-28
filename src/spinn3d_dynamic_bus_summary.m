function S=spinn3d_dynamic_bus_summary(log,bus)
%SPINN3D_DYNAMIC_BUS_SUMMARY Source energy and explicit capacitor end correction.
% Egrid - DeltaEcap is net source-plus-initial-capacitor energy delivered;
% it is not an estimate of the energy of an unexecuted recharge procedure.
S=struct('Ecap_initial_J',NaN,'Ecap_final_J',NaN,'Ecap_delta_J',NaN, ...
    'Egrid_storage_adjusted_J',NaN,'Vdc_min_V',NaN,'Vdc_max_V',NaN, ...
    'Esource_loss_J',NaN,'Eline_loss_J',NaN,'Ecurrent_limit_loss_J',NaN, ...
    'dc_solver_residual_max_W',NaN,'dc_state_pass',NaN, ...
    'Ecap_increment_sum_J',NaN,'Ecap_closure_error_J',NaN,'dc_energy_consistent',false, ...
    'no_brake_port_zero',false);
% Missing telemetry must keep the full result schema without certifying a
% zero brake port. This also covers callbacks that fail before returning.
if ~isfield(log,'Vdc') || isempty(log.Vdc),return;end
t=double(log.t(:));dt=[t(1);diff(t)];
S.Ecap_initial_J=double(log.Ecap_pre_t0_J(1));
S.Ecap_final_J=double(log.Ebuf(end));
S.Ecap_delta_J=S.Ecap_final_J-S.Ecap_initial_J;
S.Ecap_increment_sum_J=sum(double(log.Ecap_delta_J(:)));
S.Ecap_closure_error_J=abs(S.Ecap_delta_J-S.Ecap_increment_sum_J);
S.dc_energy_consistent=isfinite(S.Ecap_closure_error_J) && S.Ecap_closure_error_J<=1e-8;
if S.dc_energy_consistent
    S.Egrid_storage_adjusted_J=sum(dt.*double(log.Pgrid(:)))-S.Ecap_delta_J;
end
S.Vdc_min_V=min(double(log.Vdc_min(:)));S.Vdc_max_V=max(double(log.Vdc_max(:)));
S.Esource_loss_J=sum(dt.*double(log.Psource_loss(:)));
S.Eline_loss_J=sum(dt.*double(log.Pline_loss(:)));
S.Ecurrent_limit_loss_J=sum(dt.*double(log.Pcurrent_limit_loss(:)));
S.dc_solver_residual_max_W=max(double(log.dc_solver_residual_max_W(:)));
S.dc_state_pass=double(all(log.dc_state_pass(:)==1));
S.no_brake_port_zero=all(double(log.Pdump(:))==0);
end
