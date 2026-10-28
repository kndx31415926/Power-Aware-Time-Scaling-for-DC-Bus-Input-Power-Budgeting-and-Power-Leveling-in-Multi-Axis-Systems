function x = spinn3d_features_alpha_plus_payload(q,dq,ddq, robot, payload, Pbud, caps)
% x = [ features_from_spinn3d_features_alpha_plus , payload_vec(10) ]

q=row(q); dq=row(dq); ddq=row(ddq);

if exist('spinn3d_features_alpha_plus','file')~=2
    error('spinn3d:MissingCanonicalAlphaFeatures', ...
        'spinn3d_features_alpha_plus.m is required.');
end
x0 = spinn3d_features_alpha_plus(q,dq,ddq,robot,[],Pbud,caps);

pv = zeros(1,10);
if nargin>=5 && ~isempty(payload)
    if isstruct(payload)
        if isfield(payload,'mass')    && ~isempty(payload.mass),     pv(1)=payload.mass; end
        if isfield(payload,'com')     && numel(payload.com)>=3,      pv(2:4)=row(payload.com(1:3)); end
        if isfield(payload,'inertia') && numel(payload.inertia)>=6,  pv(5:10)=row(payload.inertia(1:6)); end
    elseif isnumeric(payload)
        v = row(double(payload)); if numel(v) < 10, v=[v, zeros(1,10-numel(v))]; end
        pv = v(1:10);
    end
end

x = double([x0, pv]);
end
function r=row(v), r=v(:)'; end
