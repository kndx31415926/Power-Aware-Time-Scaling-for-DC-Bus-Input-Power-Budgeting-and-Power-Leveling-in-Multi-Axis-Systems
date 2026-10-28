function value = spinn3d_interval_integral(t,x)
%SPINN3D_INTERVAL_INTEGRAL Integrate end-of-step telemetry without losing t0..t1.
% Each logged sample is the value applied over the interval ending at t(k).
% Therefore the correct weights are [t(1); diff(t)], not trapezoids between
% sample centres.  This convention is shared by all 1-ms execution logs.

t=double(t(:));x=double(x);
if isvector(x), x=x(:); end

dt=[t(1);diff(t)];
value=sum(dt.*x,1);
if isscalar(value), value=double(value); end
end
