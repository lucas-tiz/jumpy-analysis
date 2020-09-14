function updateMtu(obj)
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
    
    pres = obj.state.p_musc - 14.7*6.89476; % (kPa) gauge pressure
    disp = obj.state.linear_displace;

    
    % simple search over contraction vector
    persistent cont_guess
    if isempty(cont_guess)
        cont_guess = -1:0.01:8;
    end
        
    force_musc_guess = interp2(interp_data.X, interp_data.Y, interp_data.Z,...
        cont_guess, pres, 'makima');
    force_tendon_guess = cont_guess*obj.joint_param.k_tendon + ...
        disp*obj.joint_param.k_tendon;
    [~,idx] = min((force_musc_guess - force_tendon_guess).^2);

    cont_musc = cont_guess(idx);
    force_musc = force_musc_guess(idx);
    elong_tendon = force_musc/obj.joint_param.k_tendon;
   
    
    % calculate muscle volume
    vol_musc = obj.calcMuscVolume(cont_musc);

    % update state 
    obj.state.contract_musc_prev = obj.state.contract_musc;
    obj.state.contract_musc = cont_musc;
    obj.state.vol_musc = vol_musc;
    obj.state.elong_tendon = elong_tendon;
    obj.state.force_mtu = force_musc; 
end