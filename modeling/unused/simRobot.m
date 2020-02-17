function [t_vec, tau_arr, x_arr, f_arr, j_state_arr, v_com_arr, info_aerial]... 
    = simRobot(robot, sim_param)
% simulate robot 


%% Simulation
% initialize
t_vec = 0:sim_param.dt:sim_param.t_sim;
n_time = length(t_vec);
tau_arr = zeros(n_time,7);
x_arr = zeros(n_time,14);
f_arr = zeros(n_time,3*2);
j_state_arr = {MuscleJoint.createJointStruct(n_time),...
    MuscleJoint.createJointStruct(n_time),...
    MuscleJoint.createJointStruct(n_time),...
    MuscleJoint.createJointStruct(n_time)};
v_com_arr = zeros(n_time,2);
info_aerial.t = NaN;
info_aerial.v = [0,0];

x_arr(1,:) = robot.x0;
aerial.r = 0; 
aerial.l = 0;
aerial.state = 0; % 0=ground, 1=aerial

% simulate over time vector
for idx_t = 2:length(t_vec)
 
    % get current state
    t = t_vec(idx_t);
    x = x_arr(idx_t-1,:);
    q = x(1:7)';
    qd = x(8:14)';
    
%     tic
    % calculate joint torques / joint states
    [tau, j_state] = jointTorque(t,x,robot);
    tau_arr(idx_t,:) = tau;
%     toc; tic
    
    fn = fieldnames(j_state{1});
    for idx_j = 1:4
        for idx_fn = 1:numel(fn)
           j_state_arr{idx_j}.(fn{idx_fn})(idx_t) = j_state{idx_j}.(fn{idx_fn});
        end
    end
%     toc; tic
    
    % simulate forward dynamics
    [x_plus,f_plus,err_lim_bool] = forwardDynamicsLcp(robot, q, qd, tau, sim_param.dt);
    if err_lim_bool == 1
        break
    end
    x_arr(idx_t,:) = x_plus;
    f_arr(idx_t,:) = f_plus;
%     toc; tic
    
    % calculate center-of-mass velocity
    v_com = comVel(robot,x_plus);
    v_com_arr(idx_t,:) = v_com;
    
    % check liftoff
    q_plus = x_arr(idx_t,1:7)';
    foot_pos = footPos(robot,q_plus);
    xA0 = foot_pos(1);
    xA5 = foot_pos(2);
    
    ground_contact_tol = 1e-3;
    if xA0 > ground_contact_tol
        aerial.r = 1;
    end
    if xA5 > ground_contact_tol
        aerial.l = 1;
    end
    
    % save liftoff info
    if aerial.r == 1 && aerial.l == 1 && aerial.state == 0
        info_aerial.t = t; % save liftoff time
        info_aerial.v = v_com_arr(idx_t,:); % save liftoff velocity
        aerial.state = 1; % set robot aerial state to true
        
        if sim_param.liftoff_stop == 1
            break % stop sim if liftoff_stop option turned on
        end
    end
    
    % end sim if torso gets to ground
    torso_pos = torsoPos(robot, x_plus);
    if torso_pos(1) <= 0
        break
    end
%     toc;
end

end

