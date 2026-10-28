function [out,fig]=spinn3d_demo_ctcgov_online_nominal_alpha1(cfg)
%BASELINE Request nominal alpha=1; retain the same physical execution guards.
% Distinct callback/config identities prevent reuse of search-based RG-0 records.
if nargin<1||isempty(cfg),cfg=spinn3d_params_to_cfg(spinn3d_params_block(),'demo_alpha');end

cfg.METHODS.nonn_nolimit.alpha_controller_enable=false;
[out,fig]=spinn3d_demo_ctcgov_online_nonn_nolimit(cfg);
out.method_id='nominal_alpha1';
out.method_label='Baseline';
out.nominal_request_definition='alpha_req=1; common physical guards may lower actual alpha';
end
