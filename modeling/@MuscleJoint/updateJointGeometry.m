function updateJointGeometry(obj, theta, theta0)
    % Update moment arm & joint displacement at current joint angle
    % (interp/extrap from saved data)

    key = obj.cam_map_key();
    if ~obj.cam_map.isKey(key) % if key doesn't already exist, calculate data
        fprintf('NOT KEY: %s\n', key)
        obj.calculateCamData([obj.joint_param.rad0, obj.joint_param.slope]);
    end

    cam_data = obj.cam_map(key); % get cam data
    moment_arm_vec = cam_data{1}; % get effective moment arm data
    linear_displace_vec = cam_data{2};% get linear joint displacement data
    beta_vec = obj.cam_param.beta_vec; % get beta angle data

    beta0 = obj.convert_joint_angle_to_cam_angle(theta0); % convert joint angle to cam angle
    beta = obj.convert_joint_angle_to_cam_angle(theta); % convert joint angle to cam angle
    obj.state.moment_arm = interp1(beta_vec, moment_arm_vec, beta,... % calculate ema at current angle
        'linear', 'extrap');             

    l_disp0 = interp1(beta_vec, linear_displace_vec, beta0,... % calculate displacement from initial angle to sim starting angle
        'linear', 'extrap'); 
    l_disp = interp1(beta_vec, linear_displace_vec,... % calculate displacement from initial angle to current angle
        beta, 'linear', 'extrap');
    obj.state.linear_displace = l_disp - l_disp0; % switch sign so that inc joint disp corresponds to dec MTU disp 
end 


%% Helper functions
% function joint_angle_sign = get_joint_angle_sign(theta_range)
%     % Cetermine sign of joint angle relative to cam parameterization angle
%     joint_angle_sign = sign(theta_range(2) - theta_range(1));
% end
% 
% 
% function cam_angle = convert_joint_angle_to_cam_angle(theta_range, joint_angle)
%     % Convert joint angle to corresponding cam angle
%     cam_angle = (joint_angle - theta_range(1))*get_joint_angle_sign(theta_range);
% end
% 
%                 