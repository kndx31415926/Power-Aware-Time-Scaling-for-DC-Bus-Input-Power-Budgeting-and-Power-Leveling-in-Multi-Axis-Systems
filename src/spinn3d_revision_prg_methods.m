function [callbacks,labels]=spinn3d_revision_prg_methods()
%SPINN3D_REVISION_PRG_METHODS Same four physical executors in all supplements.
% Architecture comparisons use the two real architectures, not two loss
% settings of the Transformer. Order keeps the shared NoGuard reference first.
callbacks={@spinn3d_demo_ctcgov_online_nonn_nolimit, ...
    @spinn3d_demo_ctcgov_online_nonn,@spinn3d_demo_ctcgov_online_nn, ...
    @spinn3d_demo_ctcgov_online_mlp};
labels={'NoGuard','Feasible-RG','TRF-PRG','MLP-PRG'};
end
