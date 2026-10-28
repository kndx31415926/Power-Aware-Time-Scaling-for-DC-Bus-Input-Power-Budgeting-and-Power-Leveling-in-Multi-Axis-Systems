function x = spinn3d_features_alpha_plus_payload_gov(q,dq,ddq, robot, payload10_or_struct, Pbud, caps, a_prev, BETA, A_DOT_UP, A_DOT_DN, Ts, s_norm, varargin)
%SPINN3D_FEATURES_ALPHA_PLUS_PAYLOAD_GOV
% Base feature layout:
%   [alpha_plus_payload, optional_bus_features, a_prev, BETA, A_UP, A_DN, Ts, s_norm]
%
% The last 6 governor features are kept fixed so existing preprocessing code
% can keep treating them as the uncompressed tail. When bus/state features are
% used, they are inserted before the fixed governor tail.

    if nargin < 7
        error('spinn3d_features_alpha_plus_payload_gov requires at least 7 inputs.');
    end
    if nargin < 8 || isempty(a_prev),    a_prev   = 0;     end
    if nargin < 9 || isempty(BETA),      BETA     = 0.85;  end
    if nargin < 10 || isempty(A_DOT_UP), A_DOT_UP = 2.5;   end
    if nargin < 11 || isempty(A_DOT_DN), A_DOT_DN = 5.0;   end
    if nargin < 12 || isempty(Ts),       Ts       = 0.002; end
    if nargin < 13 || isempty(s_norm),   s_norm   = NaN;   end

    if isnumeric(payload10_or_struct)
        v = double(payload10_or_struct(:)).';
        if numel(v) < 10
            v = [v, zeros(1, 10-numel(v))];
        end
        pay = struct('mass',v(1), 'com',v(2:4), 'inertia',v(5:10));
    elseif isstruct(payload10_or_struct)
        pay = payload10_or_struct;
    else
        pay = struct('mass',0, 'com',[0 0 0], 'inertia',zeros(1,6));
    end

    bus = [];
    st_bus = [];
    Pgrid_avg_prev = [];
    if numel(varargin) >= 1 && ~isempty(varargin{1})
        bus = varargin{1};
    end
    if numel(varargin) >= 2 && ~isempty(varargin{2})
        st_bus = varargin{2};
    end
    if numel(varargin) >= 3 && ~isempty(varargin{3})
        Pgrid_avg_prev = varargin{3};
    elseif isstruct(bus) && isfield(bus,'Pgrid_avg_prev') && ~isempty(bus.Pgrid_avg_prev)
        Pgrid_avg_prev = bus.Pgrid_avg_prev;
    end

    x_base = spinn3d_features_alpha_plus_payload(q,dq,ddq, robot, pay, Pbud, caps);

    x_bus = [];
    if ~isempty(bus)
        x_bus = spinn3d_bus_feature_vector(bus, st_bus, Pbud, Pgrid_avg_prev);
    end

    x_state=spinn3d_alpha_observed_state_features(q,dq,a_prev,robot,varargin{4});
    schema=spinn3d_alpha_state_feature_layout(numel(q));

    x = double([x_base, x_bus, x_state, a_prev, BETA, A_DOT_UP, A_DOT_DN, Ts, s_norm]);

end
