function exportSimData(obj, fullfile_export)
    % export simulation data
    
    % save robot object
    robot = obj;
    save([fullfile_export, '.mat'], 'robot')

    
    d = obj.sim_data;
    n_data = length(d.t);
    
    dt = diff(d.t(1:2));
    thetadd = diff(d.x(:,11:14),1,1)/dt; % calculate angular accel
    thetadd = [thetadd; zeros(1,4)]; % add last row of zeros 
    

    
    % [time (s), 
    % pres1: right knee(kPa), pres2: right hip (kPa), pres3: left hip (kPa), pres4: left knee(kPa)
    % cont1: right knee (cm), cont2: right hip (cm), cont3: left hip(cm), cont4: left knee (cm)
    % elong1: right knee (cm), elong2: right hip (cm), elong3: left hip (cm), elong4: left knee (cm)
    % force1: right knee (N), force2: right hip (N), force3: left hip (N), force4: left knee (N)
    % tau2: right knee (Nm), tau3: right hip (Nm), tau4: left hip (Nm), tau5: left knee (Nm)
    % theta2: right knee (rad), theta3: right hip (rad), theta4: left hip (rad), theta5: left knee (rad)
    % theta2d: right knee (rad/s), theta3d: right hip (rad/s), theta4d: left hip (rad/s), theta5d: left knee (rad/s)
    % theta2dd: right knee (rad/s^2), theta3dd: right hip (rad/s^2), theta4dd: left hip (rad/s^2), theta5dd: left knee (rad/s^2)
    
    header = ['time, ',...
        'm-kr, m-hr, m-hl, m-kl, ',...
        'mdot-kr, mdot-hr, mdot-hl, mdot-kl, ',...
        'vol-kr, vol-hr, vol-hl, vol-kl, ',...
        'pres-kr, pres-hr, pres-hl, pres-kl, ',...
        'cont-kr, cont-hr, cont-hl, cont-kl, ',... 
        'elong-kr, elong-hr, elong-hl, elong-kl, ',...
        'force-kr, force-hr, force-hl, force-kl, ',...
        'torque-kr, torque-hr, torque-hl, torque-kl, ',...
        'theta-kr, theta-hr, theta-hl, theta-kl, ',...
        'thetad-kr, thetad-hr, thetad-hl, thetad-kl, ',...
        'thetadd-kr, thetadd-hr, thetadd-hl, thetadd-kl, ',...
        'm-source, ',...
        'p-source, ',...
        't-aerial'];
    
    data_export = [d.t',... % time
        d.joints.knee_right.m_musc, d.joints.hip_right.m_musc,... % muscle air masses
            d.joints.hip_left.m_musc, d.joints.knee_left.m_musc,...
        d.joints.knee_right.mdot_musc, d.joints.hip_right.mdot_musc,... % muscle mass flow rates
            d.joints.hip_left.mdot_musc, d.joints.knee_left.mdot_musc,...
        d.joints.knee_right.vol_musc, d.joints.hip_right.vol_musc,... % muscle volumes
            d.joints.hip_left.vol_musc, d.joints.knee_left.vol_musc,...
        d.joints.knee_right.p_musc, d.joints.hip_right.p_musc,... % muscle pressures
            d.joints.hip_left.p_musc, d.joints.knee_left.p_musc,... 
        d.joints.knee_right.contract_musc, d.joints.hip_right.contract_musc,... % muscle contraction lengths
            d.joints.hip_left.contract_musc, d.joints.knee_left.contract_musc,... 
        d.joints.knee_right.elong_tendon, d.joints.hip_right.elong_tendon,... % tendon elongations
            d.joints.hip_left.elong_tendon, d.joints.knee_left.elong_tendon,... 
        d.joints.knee_right.force_mtu, d.joints.hip_right.force_mtu,... % muscle forces
            d.joints.hip_left.force_mtu, d.joints.knee_left.force_mtu,... 
        d.joints.knee_right.torque, d.joints.hip_right.torque,... % joint torques
            d.joints.hip_left.torque ,d.joints.knee_left.torque,... 
        d.x(:,4), d.x(:,5), d.x(:,6), d.x(:,7),... % joint angles
        d.x(:,11), d.x(:,12), d.x(:,13), d.x(:,14),... % joint angular velocities
        thetadd(:,1), thetadd(:,2), thetadd(:,3), thetadd(:,4),... % joint angular accelerations
        d.m_source,... % source tank mass
        d.p_source,... % source tank pressure
        [d.info_aerial.t; zeros(n_data-1,1)]]; % aerial time
        
    
%     file_export = 'jump_data';
%     file_export_path = fullfile(repo_path,'modeling','sim-data',[file_export, '.csv']);    
%     fid = fopen(file_export_path, 'w'); 

    fid = fopen([fullfile_export, '.csv'], 'w'); 
    fprintf(fid,'%s\n',header);
    fclose(fid);
    dlmwrite([fullfile_export, '.csv'], data_export, '-append');

end