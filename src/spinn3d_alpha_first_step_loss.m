function [Lphys,PavgTrace,PavgTargetTrace,DC] = ...
        spinn3d_alpha_first_step_loss(alpha, alpha_dot, plantTs, ...
        dlDQG, dlDDQG, dlTAUFF, dlTAUADOT, dlG, dlPbud, ...
        dlPavg0Norm, pinn, dlVdc0, dlSTATE)
% alpha: 1×B (CB)
% alpha_dot: 1×B (CB), from the actual plant-rate governor transition
% dlDQG/dlDDQG/dlTAUFF/dlTAUADOT/dlG: nJ×B×K (CBT)
% dlPbud: 1×B (CB)

    eps0 = 1e-12;
    dynamic_dc=isfield(pinn,'dynamic_dc') && logical(pinn.dynamic_dc);
    DC=struct();
    if dynamic_dc

    end
    nJ = pinn.nJ;

    dqg=dlDQG(:,:,1); ddqg=dlDDQG(:,:,1);
    dt=reshape(plantTs,1,[]);
    kp=reshape(pinn_compute_field(pinn,'ctc_Kp'),[],1);
    kd=reshape(pinn_compute_field(pinn,'ctc_Kd'),[],1);
    [tau,dq_start,dq_mid,dq_next,ddq]=spinn3d_alpha_observed_ctc_kernel( ...
        alpha,alpha_dot,dt,dqg,ddqg,dlSTATE,kp,kd);
    dq=dqg.*alpha;
    % h already contains velocityProduct, gravity and friction at the
    % observed state. Adding nominal friction again would double count it.

    % token weight
    wtoken = pinn_compute_field(pinn, 'wtoken');
    wT = dlarray(wtoken, 'CBT');   % 1×1×K
    sw = sqrt(wT);

    % tau
    tau_lim_raw = pinn_compute_field(pinn, 'tau_lim');
    tau_lim = reshape(tau_lim_raw(1:nJ), [], 1, 1);
    r_tau = max(abs(tau)./max(tau_lim,eps0) - 1, 0) .* sw;
    dc=pinn.dc_constants;
    dc_fields=fieldnames(dc);
    for idc=1:numel(dc_fields)
        name=dc_fields{idc};
        value=dc.(name);
        if isnumeric(value)
            % +0*tau ensures dtype/device agree without extracting AD data.
            dc.(name)=reshape(single(value),[],1,1)+tau(:,1,1).*single(0);
        end
    end
    electrical=spinn3d_motor_dc_kernel(tau,dq_mid,dc);
    % Admission checks endpoints as well as the power midpoint.
    e_start=spinn3d_motor_dc_kernel(tau,dq_start,dc);
    e_end=spinn3d_motor_dc_kernel(tau,dq_next,dc);
    if dynamic_dc
        route=spinn3d_dynamic_dc_route(electrical.Pterminal_W,pinn.bus);
        V0=reshape(dlVdc0,1,[],1);
        DC=spinn3d_dc_link_kernel(route.Pload_axis,route.Pregen_injected, ...
            V0,0,pinn.bus,pinn.plant_Ts);
        route_start=spinn3d_dynamic_dc_route(e_start.Pterminal_W,pinn.bus);
        route_end=spinn3d_dynamic_dc_route(e_end.Pterminal_W,pinn.bus);
        DCstart=spinn3d_dc_link_kernel(route_start.Pload_axis, ...
            route_start.Pregen_injected,V0,0,pinn.bus,pinn.plant_Ts);
        DCend=spinn3d_dc_link_kernel(route_end.Pload_axis, ...
            route_end.Pregen_injected,V0,0,pinn.bus,pinn.plant_Ts);
        % These are independent conservative admission previews from one
        % observed state, exactly as in motor_interval_audit. Only the
        % midpoint preview advances the source-power EMA below.
        DC.admission_Vdc_min=min(DC.Vdc_min,min(DCstart.Vdc_min,DCend.Vdc_min));
        DC.admission_Vdc_max=max(DC.Vdc_max,max(DCstart.Vdc_max,DCend.Vdc_max));
        DC.admission_Isource=max(DC.Isource,max(DCstart.Isource,DCend.Isource));
        DC.admission_Pgrid=max(DC.Pgrid,max(DCstart.Pgrid,DCend.Pgrid));
        % On admissible states use the actual minimum link voltage. Below
        % V_min the command is already physically inadmissible: extend this
        % training residual with V_min as its scale, while L_DC separately
        % penalizes the actual undervoltage. Dividing by 1e-6 at a failed
        % root creates an artificial 1e12-scale loss, not useful physics.
        required_voltage=max(abs(electrical.V_terminal_V), ...
            max(abs(e_start.V_terminal_V),abs(e_end.V_terminal_V)));
        voltage_ratio=required_voltage./max(DC.admission_Vdc_min, ...
            single(pinn.bus.V_min_V));
        current_ratio=max(electrical.r_current,max(e_start.r_current,e_end.r_current));
        speed_ratio=max(electrical.r_speed,max(e_start.r_speed,e_end.r_speed));
        r_env=max(voltage_ratio,max(current_ratio,speed_ratio));
    else
        r_env=max(electrical.envelope_ratio,max(e_start.envelope_ratio,e_end.envelope_ratio));
    end
    r_motor_env=max(r_env-1,0).*sw;
    r_tau=max(r_tau,r_motor_env);
    L_tau = mean(r_tau.^2,'all');

    % qd/qdd
    qd_lim_raw = pinn_compute_field(pinn, 'qd_lim');
    qdd_lim_raw = pinn_compute_field(pinn, 'qdd_lim');
    qd_lim = reshape(qd_lim_raw(1:nJ), [], 1, 1);
    qdd_lim= reshape(qdd_lim_raw(1:nJ),[], 1, 1);

    if all(isfinite(pinn.qd_lim))
        r_qd = max(max(abs(dq),max(abs(dq_start),abs(dq_next)))./max(qd_lim,eps0) - 1, 0) .* sw;
        L_qd = mean(r_qd.^2,'all');
    else
        L_qd = 0;
    end
    if all(isfinite(pinn.qdd_lim))
        r_qdd = max(abs(ddq)./max(qdd_lim,eps0) - 1, 0) .* sw;
        L_qdd = mean(r_qdd.^2,'all');
    else
        L_qdd = 0;
    end

    % power
    Pax_lim_raw = pinn_compute_field(pinn, 'Pax_lim');
    Pax_lim = reshape(Pax_lim_raw(1:nJ), [], 1, 1);
    Pj = tau .* dq_mid;
    Pax = max(max(max(tau.*dq_start,tau.*dq_mid),tau.*dq_next),0);
    r_Pax = max(Pax./max(Pax_lim,eps0) - 1, 0) .* sw;
    L_Pax = mean(r_Pax.^2,'all');

    if dynamic_dc
        Pgrid=DC.Pgrid; Pregen_bus=route.Pregen_injected;
        Pshare_direct=route.Pshare_direct;
    else
        [Pgrid,Pregen_bus,Pshare_direct]=nominal_dc_route_dl_local(electrical.Pterminal_W,pinn);
    end
    % Same conservative interval PSU bound as teacher/runtime. Positive
    % terminal draw is maximized per axis at velocity endpoints, with no
    % regenerative credit in the hardware 60-W admission constraint.
    Pgrid_interval_peak=single(pinn.P_idle_W)+sum(max( ...
        max(e_start.Pterminal_W,0),max(e_end.Pterminal_W,0)),1);
    if dynamic_dc
        % Actual source power already includes current limiting and series
        % loss. Capacitor support can legitimately exceed the source rating
        % at the load port, so an unshared load bound is not a source cap.
        Pgrid_interval_peak=DC.admission_Pgrid;
    end
    pavg_prev=reshape(dlPavg0Norm.*dlPbud,1,[],1);
    beta_pavg=exp(-dt./single(pinn.Pavg_tau_s));
    PavgTrace=beta_pavg.*pavg_prev+(single(1)-beta_pavg).*Pgrid;
    pavg_target = reshape(dlPbud.*single(pinn.Pavg_target_frac),1,[],1);
    PavgTargetTrace = pavg_target + single(0).*Pgrid;

    if isfield(pinn,'Pgrid_peak_cap') && isfinite(pinn.Pgrid_peak_cap) && pinn.Pgrid_peak_cap > 0
        % Keep the cap on the same device/type as dlPbud by deriving it from dlPbud.
        Pgrid_cap_eff = reshape(dlPbud .* 0 + single(pinn.Pgrid_peak_cap), 1, [], 1);
    else
        Pgrid_cap_eff = reshape(max(dlPbud, 1e-6), 1, [], 1);  
    end
    r_Pgrid = max(Pgrid_interval_peak./max(Pgrid_cap_eff, 1e-6) - 1, 0) .* sw;
    L_Pgrid = mean(r_Pgrid.^2,'all');

    L_DC=0;
    if dynamic_dc
        vspan=max(pinn.bus.V_max_V-pinn.bus.V_min_V,1e-6);
        r_vlow=max((pinn.bus.V_min_V-DC.admission_Vdc_min)./vspan,0);
        r_vhigh=max((DC.admission_Vdc_max-pinn.bus.V_max_V)./vspan,0);
        r_current=max(DC.admission_Isource./pinn.bus.I_supply_max_A-1,0);
        L_DC=mean(r_vlow.^2+r_vhigh.^2+r_current.^2,'all');
        % No braking resistor is modeled; surplus energy increases Vdc.
        % A single-precision Newton

        L_Pdump=0;
    elseif isfield(pinn,'bus_model_st3215') && pinn.bus_model_st3215
        L_Pdump = 0;
    elseif isfinite(pinn.P_brk_peak)
        Pdump=max(Pregen_bus-Pshare_direct,0);
        r_Pdump = max(Pdump./max(pinn.P_brk_peak,1e-6) - 1, 0) .* sw;
        L_Pdump = mean(r_Pdump.^2,'all');
    else
        L_Pdump = 0;
    end

    Lphys = pinn.w.tau*L_tau + pinn.w.qd*L_qd + pinn.w.qdd*L_qdd + ...
            pinn.w.Pax*L_Pax + pinn.w.Pgrid*(L_Pgrid+L_DC);
    DC.loss_components=struct('tau',pinn.w.tau*L_tau, ...
        'qd',pinn.w.qd*L_qd,'qdd',pinn.w.qdd*L_qdd, ...
        'axis_power',pinn.w.Pax*L_Pax,'source_peak',pinn.w.Pgrid*L_Pgrid, ...
        'dc_state',pinn.w.Pgrid*L_DC);
    if ~dynamic_dc
        Lphys=Lphys+pinn.w.Pdump*L_Pdump;
        DC.loss_components.dump=pinn.w.Pdump*L_Pdump;
    end
    DC.ctc_tau=tau; DC.ctc_ddq_actual=ddq; DC.ctc_dq_start=dq_start;
    DC.ctc_dq_next=dq_next;
end

function v = pinn_compute_field(pinn, f)
    fd = [f '_dev'];
    if isfield(pinn, fd) && ~isempty(pinn.(fd))
        v = pinn.(fd);
    else
        v = single(pinn.(f));
    end
end

function [Pgrid,Pregen_bus,Pshare_direct]=nominal_dc_route_dl_local(Pterminal,pinn)
Pmot=sum(max(Pterminal,0),1);
Pregen_dc_raw=sum(max(-Pterminal,0),1).*single(pinn.eta_regen);
if pinn.regen_enable
    available=min(Pregen_dc_raw,single(pinn.Pregen_peak_max_W));
    Pregen_bus=single(pinn.eta_share).*available;
else
    Pregen_bus=Pregen_dc_raw.*0;
end
if pinn.regen_enable && pinn.direct_share_enable && pinn.demand_limited_direct
    Pshare_direct=min(min(Pregen_bus,Pmot),single(pinn.Pshare_peak_max_W));
else
    Pshare_direct=Pregen_bus.*0;
end
Pgrid=single(pinn.P_idle_W)+max(Pmot-Pshare_direct,0);
end
