function C=reviewer_conditions(study)
% Perturbation levels used in the revised manuscript.
blank=struct('id',0,'name','','family','','parameter','','assignment','','value',0);
switch char(study)
case 'robustness'
    names={'nominal','joint_noise_0p3','joint_noise_0p5_boundary', ...
        'unknown_friction','unknown_payload_com','unknown_voltage','unknown_combined'};
    families={'nominal','joint_noise','joint_noise','friction','payload_com','voltage','combined'};
    C=repmat(blank,7,1);
    for k=1:7,C(k).id=k;C(k).name=names{k};C(k).family=families{k};end
    C(2).value=.3;C(3).value=.5;C(7).value=.3;
case 'parameter'
    C=repmat(blank,17,1);
    C(1)=struct('id',1,'name','shared_nominal','family','nominal', ...
        'parameter','','assignment','identity','value',1);
    fields={'I_stall_A','I_noload_A','tau_stall_single_Nm','omega_noload_rad_s'};
    k=1;
    for j=1:4
        for value=[.95 1.05]
            k=k+1;C(k)=struct('id',k,'name',sprintf('%s_x%.2f',fields{j},value), ...
                'family','motor','parameter',fields{j},'assignment','multiply_nominal','value',value);
        end
    end
    for field={'C_dc_F','R_source_ohm'}
        for value=[.9 1.1]
            k=k+1;C(k)=struct('id',k,'name',sprintf('%s_x%.2f',field{1},value), ...
                'family','dc','parameter',field{1},'assignment','multiply_nominal','value',value);
        end
    end
    for value=[11.7 12.3]
        k=k+1;C(k)=struct('id',k,'name',sprintf('V_source_%.1fV',value), ...
            'family','voltage','parameter','V_source_V','assignment','set','value',value);
    end
    for value=[.025 .1]
        k=k+1;C(k)=struct('id',k,'name',sprintf('feedback_EWMA_%dms',round(value*1000)), ...
            'family','filter','parameter','P_avg_window_s','assignment','set','value',value);
    end
otherwise
    C=blank;C.id=1;C.name='nominal';C.family='nominal';
end
end
