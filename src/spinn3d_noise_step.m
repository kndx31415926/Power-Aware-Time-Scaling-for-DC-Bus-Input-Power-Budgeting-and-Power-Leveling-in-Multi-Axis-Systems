function [q_meas,dq_meas,tau_noise,state] = ...
        spinn3d_noise_step(q,dq,Ts,state)
%SPINN3D_NOISE_STEP Apply one shared centrally configured noise sample.
q=double(q(:).');
dq=double(dq(:).');

q_meas = q + state.sigma_q*noise_randn_local(state,1,state.nJ);
dq_meas = dq + state.sigma_dq*noise_randn_local(state,1,state.nJ);
state.elapsed_s = state.elapsed_s + double(Ts);
if state.elapsed_s+1e-15 >= state.servo_hold_s
    state.tau_hold = state.tau_std*noise_randn_local(state,1,state.nJ);
    state.elapsed_s = mod(state.elapsed_s,state.servo_hold_s);
end
tau_noise = state.tau_hold;
end

function x = noise_randn_local(state,varargin)
if strcmp(state.rng_mode,'local')
    x = randn(state.stream,varargin{:});
else
    x = randn(varargin{:});
end
end
