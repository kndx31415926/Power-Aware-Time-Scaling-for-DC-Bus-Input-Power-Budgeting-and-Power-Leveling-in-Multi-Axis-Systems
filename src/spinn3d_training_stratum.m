function S=spinn3d_training_stratum(PB,index)
% Fixed accepted-ID stratum; a rejected candidate is redrawn in this stratum.

edges=double(PB.training.payload_bin_edges_kg(:).');
nt=numel(PB.revision.task_domain.templates);

template_index=1+mod(double(index)-1,nt);
payload_group=1+mod(floor((double(index)-1)/nt),numel(edges)-1);
S=struct('schema',char(PB.training.task_sampling_schema), ...
    'stratum_index',(payload_group-1)*nt+template_index, ...
    'payload_group',payload_group,'template_index',template_index, ...
    'mass_bounds_kg',edges(payload_group:payload_group+1));
end
