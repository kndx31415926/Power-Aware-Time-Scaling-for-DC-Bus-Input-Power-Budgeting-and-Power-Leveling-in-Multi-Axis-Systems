function S = spinn3d_alpha_state_feature_layout(n)

S.schema='alpha_plus_payload_gov_ctc_nobrake_v7';
S.state_schema='observed_ctc_eq_edq_mass_bias_v1';
S.base_dim=14*n+12; S.payload_dim=10; S.bus_dim=10;
S.bus_schema='dynamic_dc_observation_nobrake_v2';
S.bus_names={'Ebuf_norm','Ebuf_slack','Ereuse_norm','Pbuf_dis_norm', ...
    'Pbuf_chg_norm','eta_share','Pgrid_avg_norm','Pgrid_avg_slack', ...
    'Vdc_over_source','Vdc_upper_slack'};
S.bus=S.base_dim+S.payload_dim+(1:S.bus_dim);
S.Pavg_norm=S.bus(7);S.Vdc_norm=S.bus(9);
S.state_dim=n*n+3*n;
i=S.base_dim+S.payload_dim+S.bus_dim;
S.eq=i+(1:n); S.edq=i+n+(1:n);
S.M=i+2*n+(1:n*n); S.h=i+2*n+n*n+(1:n);
S.state=i+(1:S.state_dim);
S.tail=i+S.state_dim+(1:6);
S.total_dim=S.tail(end);
end
