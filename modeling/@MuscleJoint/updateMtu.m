%% Muscle-tendon in series
function updateMtu(obj, p_musc)
    % Calculate muscle contraction length, muscle volume, 
    % corresponding tendon displacement, and muscle force  
    
    % load pressure-contraction-force interpolation data
    persistent force_data
    if isempty(force_data)
        path_interp_data = fullfile(obj.repo_path,...
            'modeling', '@MuscleJoint', 'muscle-data', 'force-calibration',...
            'force_makima_interp_data.mat');
    	force_data = load(path_interp_data, 'X','Y','Z');
    end
    
    % load mesh stiffness data
    persistent mesh_data
    if isempty(mesh_data)
        path_cfit_mesh = fullfile(obj.repo_path,...
            'modeling', '@MuscleJoint', 'muscle-data', 'mesh-characterization',...
            'mesh_stiffness_data-40cm.mat');
        mesh_data = load(path_cfit_mesh, 'ext', 'force');
    end
        
    pres = max(0, p_musc - 101.325); % (kPa) gauge pressure
    disp = obj.state.linear_displace;

    % simple search over contraction vector
    persistent cont_guess
    if isempty(cont_guess)
        cont_guess = -2:0.001:7.5;
    end
        
    force_musc_guess = max(0, interp2(force_data.X, force_data.Y, force_data.Z,...
        cont_guess, pres, 'makima'));
    force_tendon_guess =  max(0,interp1(mesh_data.ext,...
        mesh_data.force, (cont_guess+disp), 'makima'));
    %TODO: include series tendon with muscle (obj.joint_param.k_tendon)
            
    [~,idx] = min((force_musc_guess - force_tendon_guess).^2);
    cont_musc = cont_guess(idx);
    force_musc = force_musc_guess(idx);
    elong_tendon = max(0, cont_musc+disp);

    % calculate muscle volume
    vol_musc = obj.calcMuscVolume(cont_musc);

    % update state 
    obj.state.contract_musc_prev = obj.state.contract_musc;
    obj.state.contract_musc = cont_musc;
    obj.state.vol_musc = vol_musc;
    obj.state.elong_tendon = elong_tendon;
    obj.state.force_mtu = force_musc; 
end


%% Muscle only
% function updateMtu(obj, p_musc)
%     % Calculate muscle contraction length, muscle volume, 
%     % and muscle force  
%     
%     % load pressure-contraction-force interpolation data
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
%     pres = max(0, p_musc - 101.325); % (kPa) gauge pressure
%     disp = obj.state.linear_displace;
% 
%     % calculate muscle force
%     force_musc = max(0, interp2(interp_data.X, interp_data.Y, interp_data.Z,...
%         -disp, pres, 'makima'));
%     
%     if (force_musc > 0) && (-disp < 8) % if non-zero force & inside muscle char range
%         cont_musc = -disp;
%     else % if zero force or outside muscle char range
%         cont_guess = -1:0.001:8;
%         force_guess = max(0, interp2(interp_data.X, interp_data.Y, interp_data.Z,...
%             cont_guess, pres, 'makima'));
%     
%         idx = find(force_guess<=0, 1, 'first');
%         cont_musc = cont_guess(idx);
%         force_musc = force_guess(idx);
%     end
%     elong_tendon = 0;
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

