function y = spinn3d_lpf1_causal(x, Ts, tau, y0)
%SPINN3D_LPF1_CAUSAL  Causal first-order low-pass filter for power telemetry.
%
%   y_k = lambda*y_{k-1} + (1-lambda)*x_k,
%   lambda = exp(-Ts/tau).
%
% This helper is used for evaluation/plotting of source-side DC-bus input
% power.  It is causal and online-realizable, unlike a centered moving average.
%
% Inputs
%   x   : input vector
%   Ts  : sampling time [s]
%   tau : filter time constant [s]
%   y0  : optional initial value.  If omitted, y0=x(1).
%
% Output
%   y   : filtered vector with the same shape as x.

orig_size = size(x);
x = double(x(:));
if nargin < 2 || isempty(Ts) || ~isfinite(Ts) || Ts <= 0
    Ts = 1e-3;
end
if nargin < 3 || isempty(tau) || ~isfinite(tau) || tau <= 0
    tau = 0.05;
end
if nargin < 4 || isempty(y0) || ~isfinite(y0)
    if isempty(x), y0 = 0; else, y0 = x(1); end
end

y = zeros(size(x));
if isempty(x)
    y = reshape(y, orig_size);
    return;
end

lambda = exp(-double(Ts)/max(double(tau), eps));
lambda = min(max(lambda,0),1);
y(1) = double(y0);
for k = 2:numel(x)
    y(k) = lambda*y(k-1) + (1-lambda)*x(k);
end

y = reshape(y, orig_size);
end
