function st = spinn3d_bus_state_init(bus)

% The formal dynamic state contains Vdc only; Ebuf
% is derived capacitor energy, while stored regeneration credits remain zero.
% Historical stateless profiles return zero-energy compatibility placeholders.
%
% st fields:
%   st.Vdc    : dynamic capacitor voltage [V], dynamic scope only

%   st.Ereuse : reused/regenerative portion inside Ebuf [J]
%   st.Tbrk   : zero compatibility placeholder in the no-brake dynamic model
%   st.Eregen_task : current-task regenerated energy still stored [J]

    if nargin < 1 || isempty(bus), bus = struct(); end
    bus = spinn3d_bus_defaults(bus);

    st = struct('model_scope',bus.model_scope,'has_voltage_state',false, ...
        'has_brake_thermal_state',false, ...
        'has_identified_brake_thermal_state',false);
    if strcmp(char(bus.model_scope),'dynamic_capacitor_common_dc_v1')
        st.has_voltage_state=true;
        st.has_brake_thermal_state=false;
        st.Vdc=double(bus.V_init_V);
        st.Ebuf=0.5*double(bus.C_dc_F)*st.Vdc^2;
        st.Ereuse=0.0;
        st.Eregen_task=0.0;
        st.Tbrk=0;
        return;
    end
    if ~bus.enable_state
        st.Ebuf=0.0; st.Ereuse=0.0; st.Eregen_task=0.0;
        st.Tbrk=double(bus.Tamb_C);
        return;
    end
    Emax = double(bus.Ebuf_max_J);
    Emin = min(max(double(getfield_def_local(bus,'Ebuf_min_J',0.0)), 0.0), Emax);
    st.Ebuf   = min(max(double(bus.Ebuf_init_J), Emin), Emax);
    st.Ereuse = min(max(double(bus.Ereuse_init_J), 0.0), st.Ebuf);
    st.Eregen_task = 0.0;
    st.Tbrk   = min(max(double(bus.Tbrk_init_C), double(bus.Tamb_C)), double(bus.Tbrk_max_C));
end

function v = getfield_def_local(S, fn, def)
    if isstruct(S) && isfield(S, fn) && ~isempty(S.(fn))
        v = S.(fn);
    else
        v = def;
    end
end
