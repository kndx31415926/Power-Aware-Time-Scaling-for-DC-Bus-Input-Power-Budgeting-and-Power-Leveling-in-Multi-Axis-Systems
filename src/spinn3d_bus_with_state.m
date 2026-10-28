function bus = spinn3d_bus_with_state(bus,st,Ts)
%SPINN3D_BUS_WITH_STATE Bind a measured/preview DC state to a config copy.
% V_source_V and fixed motor constants remain unchanged. V_bus_V is the
% available motor-drive voltage, not the upstream source rating.
if nargin<3 || isempty(Ts), Ts=1e-3; end
if ~isstruct(bus) || ~isfield(bus,'model_scope') || ...
        ~strcmp(bus.model_scope,'dynamic_capacitor_common_dc_v1')
    return;
end
if nargin<2 || isempty(st), st=spinn3d_bus_state_init(bus); end

bus.V_bus_V=st.Vdc;
bus.dc_state=st;
% A nominal predictor may observe voltage from a plant with a different C.
% Its capacitor-energy view is derived from its own C and measured V; this
% does not mutate the supplied true-plant state or reinitialize voltage.
bus.dc_state.Ebuf=0.5*bus.C_dc_F*st.Vdc^2;
bus.dc_step_s=Ts;
end
