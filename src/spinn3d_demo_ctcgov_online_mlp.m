function [out,fig]=spinn3d_demo_ctcgov_online_mlp(cfg)
%SPINN3D_DEMO_CTCGOV_ONLINE_MLP MLP-PRG on the common learned Alpha runner.
% Identical features, governor, gains, source model and physical guard to TRF.
if nargin<1||isempty(cfg)
    cfg=spinn3d_params_to_cfg(spinn3d_params_block(),'demo_alpha');
end
cfg=spinn3d_select_alpha_architecture(cfg,'mlp');
[out,fig]=spinn3d_demo_ctcgov_online_nn(cfg);
end
