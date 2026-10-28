function q = spinn3d_percentile_inclusive(x,p)
%SPINN3D_PERCENTILE_INCLUSIVE Toolbox-free inclusive percentiles.
%   Q = SPINN3D_PERCENTILE_INCLUSIVE(X,P) removes nonfinite X values,
%   sorts the remaining samples and evaluates each percentile in P using
%   h = 1 + (n-1)*P/100 with linear interpolation.  This is the explicit
%   inclusive/type-7 convention already used by the paper code; it avoids
%   a Statistics and Machine Learning Toolbox dependency and prevents
%   MATLAB-version-dependent PRCTILE defaults from changing paper results.

if ~(isnumeric(x) || islogical(x)) || ~isreal(x)
    error('spinn3d:InvalidPercentileSamples', ...
        'Percentile samples must be a real numeric or logical array.');
end
if ~isnumeric(p) || ~isreal(p) || any(~isfinite(double(p(:)))) || ...
        any(double(p(:))<0) || any(double(p(:))>100)
    error('spinn3d:InvalidPercentileQuery', ...
        'Percentile queries must be finite real numeric values in [0,100].');
end

p_shape = size(p);
p = double(p);
x = double(x(:));
x = sort(x(isfinite(x)));
if isempty(x)
    q = NaN(p_shape);
    return;
end

position = 1 + (numel(x)-1).*p/100;
lower_index = floor(position);
upper_index = ceil(position);
weight = position-lower_index;
q = x(lower_index(:)) + weight(:).*( ...
    x(upper_index(:))-x(lower_index(:)));
q = reshape(q,p_shape);
end
