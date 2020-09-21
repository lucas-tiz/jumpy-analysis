function updateMtu(obj, p_musc)
    % calculate muscle contraction length, muscle volume, 
    % corresponding tendon displacement, and muscle force  
    
    % load pressure-contraction-force interpolation data
    % TODO: tune 'negative contraction' muscle stiffness
    persistent interp_data
    if isempty(interp_data)
        fprintf('loading force interp data...\n') %DEBUG

        interp_data_path = fullfile(obj.repo_path,...
            'modeling', '@MuscleJoint', 'muscle-data', 'force-calibration',...
            'force_makima_interp_data.mat');
    	interp_data = load(interp_data_path, 'X','Y','Z');
    end
    
%     pres = obj.state.p_musc - 14.7*6.89476; % (kPa) gauge pressure
    pres = max(0, p_musc - 101.325); % (kPa) gauge pressure
    disp = obj.state.linear_displace;

    force_musc = max(0, interp2(interp_data.X, interp_data.Y, interp_data.Z,...
        -disp, pres, 'makima'));
    
    if (force_musc > 0) && (disp > -8)
        cont_musc = -disp;
    else
        cont_guess = -1:0.001:8;
        force_guess = max(0, interp2(interp_data.X, interp_data.Y, interp_data.Z,...
            cont_guess, pres, 'makima'));
    
        idx = find(force_guess<=0, 1, 'first');
        cont_musc = cont_guess(idx);
        force_musc = force_guess(idx);
    end
    
%     if (cont_musc > 6.0) && (obj.joint_param.rad0 < 6)
%         fprintf('pause\n')
%     end
    
    elong_tendon = 0;
   
    % calculate muscle volume
    vol_musc = obj.calcMuscVolume(cont_musc);

    % update state 
    obj.state.contract_musc_prev = obj.state.contract_musc;
    obj.state.contract_musc = cont_musc;
    obj.state.vol_musc = vol_musc;
    obj.state.elong_tendon = elong_tendon;
    obj.state.force_mtu = force_musc; 
end


% function updateMtu(obj)
%     % calculate muscle contraction length, muscle volume, 
%     % corresponding tendon displacement, and muscle force  
%     
%     % load pressure-contraction-force interpolation data
%     % TODO: tune 'negative contraction' muscle stiffness
%     persistent interp_data
%     if isempty(interp_data)
%         fprintf('loading force interp data...\n') %DEBUG
% 
%         interp_data_path = fullfile(obj.repo_path,...
%             'modeling', '@MuscleJoint', 'muscle-data', 'force-calibration',...
%             'force_makima_interp_data.mat');
%     	interp_data = load(interp_data_path, 'X','Y','Z');
%     end
%     
%     pres = obj.state.p_musc - 14.7*6.89476; % (kPa) gauge pressure
%     disp = obj.state.linear_displace;
% 
%     
%     % simple search over contraction vector
%     persistent cont_guess
%     if isempty(cont_guess)
%         cont_guess = -1:0.001:7.5;
%     end
%         
%     force_musc_guess = interp2(interp_data.X, interp_data.Y, interp_data.Z,...
%         cont_guess, pres, 'makima');
%     force_tendon_guess = max(0,(cont_guess+disp)*obj.joint_param.k_tendon);
%             
%     [~,idx] = min((force_musc_guess - force_tendon_guess).^2);
%     
% %     fprintf('%f %f \n', disp, force_tendon_guess(idx));
%     
%     cont_musc = cont_guess(idx);
%     force_musc = force_musc_guess(idx);
%     elong_tendon = force_musc/obj.joint_param.k_tendon;
%    
%     
%     % calculate muscle volume
%     vol_musc = obj.calcMuscVolume(cont_musc);
% 
%     % update state 
%     obj.state.contract_musc_prev = obj.state.contract_musc;
%     obj.state.contract_musc = cont_musc;
%     obj.state.vol_musc = vol_musc;
%     obj.state.elong_tendon = elong_tendon;
%     obj.state.force_mtu = force_musc; 
% end