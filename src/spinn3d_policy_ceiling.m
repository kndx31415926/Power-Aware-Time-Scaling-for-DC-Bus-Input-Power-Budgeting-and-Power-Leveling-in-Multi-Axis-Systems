function ceiling = spinn3d_policy_ceiling(ceiling,policy_tick,nominal,admitted,alpha_max)
% Latch only a genuine safety reduction inside a desired-command ZOH.
% Normal slew increases are permitted at every plant sample. The next policy
% boundary releases this short latch; safe-hold recovery owns its own latch.
if policy_tick, ceiling=double(alpha_max); end
if double(admitted)<double(nominal)-1e-12
    ceiling=min(double(ceiling),max(0,double(admitted)));
end
end
