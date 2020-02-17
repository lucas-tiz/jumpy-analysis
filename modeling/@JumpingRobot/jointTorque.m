function [tau, joint_state] = jointTorque(obj,t,x)
% calculate joint torques applied by muscle-tendon units
tau = zeros(7,length(t));
% joint_state = cell(1,4);


% pneumatic muscle-tendon control
sign_tau = [-1,1,1,-1];
for idx_j = 1:4 % loop over index corresponding to actuated joint
    idx_q = idx_j+3; % index corresponding to joint angle state
    idx_xd = idx_j+3+7; % index corresponding to joint angular velocity state
    ang0 = obj.state.x0(idx_q); % initial joint angle
    ang_cur = x(:,idx_q); % current joint angle
    angvel_cur = x(:,idx_xd); % current joint angular velocity

    joint = obj.config.model.joint_order{idx_j}; % joint name
    
    % muscle torque
    joint_state.(joint) = obj.joints.(joint).calcJointState(ang0, ang_cur,...
        t-obj.config.control.t_musc_activate(idx_j), obj.config.control.p_max);    
    joint_state.(joint).torque = sign_tau(idx_j)*joint_state.(joint).torque; 
    tau_muscle = joint_state.(joint).torque; 

    
    % spring torque
    joint_angle_sign = obj.joints.(joint).get_joint_angle_sign();
    truth_stiff = joint_angle_sign*ang_cur >= ...
        joint_angle_sign*obj.config.model.joint_stiff_lims(1,idx_j);
    tau_stiff = -truth_stiff.*obj.config.model.joint_stiff(idx_j)*...
        (ang_cur-obj.config.model.joint_stiff_lims(1,idx_j));
    
    % damping torque
    truth_damp = joint_angle_sign*ang_cur >= ...
        joint_angle_sign*obj.config.model.joint_damp_lims(idx_j);
    tau_damp = -truth_damp.*obj.config.model.joint_damp(idx_j)*angvel_cur;
        
    % total torque
    tau(idx_q,:) = tau_muscle + tau_stiff + tau_damp;
    
%     if idx_j == 1
%         fprintf('%0.2f, %i \n', t, truth_stiff)
%     end
    
    
end


% for i = 1:length(t)
%     for j = 1:4
%         idx_x = j+1;
%         theta_init = model.x0(idx_x); % initial joint angle
%         theta = x(i,idx_x); % current joint angle
% 
%         joint_state{i,j} = torque_musc(model.joint_param{j}, theta_init,...
%             theta, t(i)-model.t_musc_activate(j), model.joint_state{j}.muscle_contract);
%         tau(i,idx_x) = joint_state{i,j}.torque; 
%     end
% end

% fprintf('c: %0.3f, d: %0.3f\n', model.joint_state{1}.muscle_contract,...
%     model.joint_state{1}.linear_displace)
    
    
    
% fixed torque onset timing control
% dec_round = 4;
% for i = 1:length(t)
%     idx_ctrl = find(round(model.tau_fixed(:,1),dec_round) <= round(t(i),dec_round), 1, 'last');
%     tau(i,:) = model.tau_fixed(idx_ctrl,2:end);
% end
% 
% end