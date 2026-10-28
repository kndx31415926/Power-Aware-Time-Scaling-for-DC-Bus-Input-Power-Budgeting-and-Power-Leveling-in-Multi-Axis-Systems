function cfg=spinn3d_select_alpha_architecture(cfg,architecture)
% Select the policy architecture with its corresponding model file.
architecture=lower(char(architecture));
if strcmp(architecture,'mlp')
    method='mlp';pathfield='MODEL_PATH_ALPHA_MLP';
else
    method='nn';pathfield='MODEL_PATH_ALPHA';
end
cfg.MODEL_PATH=cfg.(pathfield);
cfg.ALPHA_EXPECTED_ARCHITECTURE=architecture;
cfg.ALPHA_METHOD_ID=method;
end
