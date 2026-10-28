function st = spinn3d_dc_state_for_model(measured,bus)
%SPINN3D_DC_STATE_FOR_MODEL Use measured V with this model's capacitance.
% An unknown-plant capacitance is not an observable controller parameter.
% Ebuf is reconstructed from measured voltage and NOMINAL C for a nominal
% preview; the plant's real energy remains in its own state and flow log.
st=measured;
if isfield(bus,'model_scope') && ...
        strcmp(bus.model_scope,'dynamic_capacitor_common_dc_v1')

    st.Ebuf=0.5*double(bus.C_dc_F)*double(measured.Vdc)^2;
    st.Ereuse=0; st.Eregen_task=0; st.Tbrk=0;
    st.has_brake_thermal_state=false;
end
end
