
function [a_post, a_slew, a_filt] = governor_step(a_des, a_prev, BETA, A_DOT_UP, A_DOT_DN, Ts)

    lo = a_prev - A_DOT_DN*Ts;
    hi = a_prev + A_DOT_UP*Ts;
    a_slew = min(hi, max(lo, a_des));
    a_filt = BETA*a_prev + (1-BETA)*a_slew;
    a_post = a_filt;
end
