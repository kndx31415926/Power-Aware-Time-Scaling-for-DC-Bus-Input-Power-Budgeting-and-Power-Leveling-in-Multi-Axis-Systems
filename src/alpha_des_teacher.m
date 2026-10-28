
function a_des = alpha_des_teacher(a_star, a_prev, BETA, A_DOT_UP, A_DOT_DN, Ts, alpha_max)

    epsl = 1e-12;
    if (1-BETA) < epsl
        a_lin = a_prev + A_DOT_UP*Ts;
    else
        a_lin = (a_star - BETA*a_prev) / (1 - BETA);
    end
    lo = a_prev - A_DOT_DN*Ts;
    hi = a_prev + A_DOT_UP*Ts;
    a_des = min(hi, max(lo, a_lin));
    a_des = min(double(alpha_max), max(0.0, a_des));
end
