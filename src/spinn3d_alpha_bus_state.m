function st = spinn3d_alpha_bus_state(bus,st_in)
% Canonical teacher/planner state; never clip a dynamic voltage or energy.
if nargin<2 || isempty(st_in)
    st=spinn3d_bus_state_init(bus);
    return;
end

st=spinn3d_bus_state_init(bus);
if isfield(st_in,'Pgrid_avg_lp') && ~isempty(st_in.Pgrid_avg_lp)

    st.Pgrid_avg_lp=double(st_in.Pgrid_avg_lp);
end
if strcmp(char(bus.model_scope),'dynamic_capacitor_common_dc_v1')

    check=spinn3d_bus_with_state(bus,st_in,1e-3); %#ok<NASGU>
    st.Vdc=double(st_in.Vdc);
    st.Ebuf=0.5*double(bus.C_dc_F)*st.Vdc^2;
    if isfield(st_in,'Ebuf') && ~isempty(st_in.Ebuf)

    end
    for name={'Ereuse','Eregen_task'}
        if isfield(st_in,name{1}) && ~isempty(st_in.(name{1}))

        end
    end
    return;
end
% Explicit historical profiles retain their former compatibility semantics.
for name={'Ebuf','Ereuse','Eregen_task','Tbrk'}
    if isfield(st_in,name{1}) && ~isempty(st_in.(name{1}))
        st.(name{1})=double(st_in.(name{1}));
    end
end
if bus.enable_state
    st.Ebuf=min(max(st.Ebuf,0),bus.Ebuf_max_J);
    st.Ereuse=min(max(st.Ereuse,0),st.Ebuf);
    st.Tbrk=min(max(st.Tbrk,bus.Tamb_C),bus.Tbrk_max_C);
end
end
