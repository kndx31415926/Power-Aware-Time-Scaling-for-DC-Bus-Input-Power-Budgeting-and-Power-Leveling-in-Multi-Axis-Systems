function st = spinn3d_bus_state_init(bus)
%SPINN3D_BUS_STATE_INIT  Initialize DC-bus energy/thermal/voltage state.
if nargin < 1 || isempty(bus), bus = struct(); end
bus = spinn3d_bus_defaults(bus);

st = struct();
st.Ebuf   = min(max(double(bus.Ebuf_init_J), bus.Ebuf_min_J), bus.Ebuf_max_J);
st.Ereuse = min(max(double(bus.Ereuse_init_J), 0.0), st.Ebuf);
st.Tbrk   = min(max(double(bus.Tbrk_init_C), double(bus.Tamb_C)), double(bus.Tbrk_max_C));
st.Vdc    = min(max(double(bus.V_nom), double(bus.V_min)), double(bus.V_max));
st.Egrid_total  = 0.0;
st.Edump_total  = 0.0;
st.Espill_total = 0.0;
st.Ereuse_total = 0.0;
st.Eloss_total  = 0.0;
end
