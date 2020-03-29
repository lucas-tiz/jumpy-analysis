function simRobot(obj, sim_param)
    % Simulate robot: LCP

    % initialize
    obj.sim_data.t = 0:sim_param.dt:sim_param.t_sim;
    n_time = length(obj.sim_data.t);
    obj.sim_data.tau = zeros(n_time,7);
    obj.sim_data.x = zeros(n_time,14);
    obj.sim_data.grf.r = zeros(n_time,2);
    obj.sim_data.grf.l = zeros(n_time,2);
    
    joints = obj.config.model.joint_order;
    for i = 1:length(joints)
        obj.sim_data.j_state.(joints{i}) = ...
            MuscleJoint.createJointStruct(n_time);
    end
   
    obj.sim_data.info_aerial.t = NaN;
    obj.sim_data.info_aerial.v = [0,0];
    
    obj.sim_data.traj.pos_torso = zeros(n_time,2);
    obj.sim_data.traj.v_com = zeros(n_time,2);

    aerial.r = 0; 
    aerial.l = 0;
    aerial.state = 0; % 0=ground, 1=aerial
    
    % set values for t = 0
    obj.sim_data.x(1,:) = obj.state.x0;
    obj.sim_data.traj.pos_torso(1,:) = obj.torsoPos(obj.state.x0);

    % simulate over time vector
    for idx_t = 2:length(obj.sim_data.t)

        % get current state
        t = obj.sim_data.t(idx_t);
        x = obj.sim_data.x(idx_t-1,:);
        q = x(1:7)';
        qd = x(8:14)';

    %     tic
        % calculate joint torques / joint states
        [tau, j_state] = obj.jointTorque(t,x);
        obj.sim_data.tau(idx_t,:) = tau;
    %     toc; tic

        state_fns = fieldnames(j_state.(joints{1})); % fieldnames of 'joint_state_struct'
        for idx_j = 1:length(joints) % loop over joints
            joint = joints{idx_j}; % joint name
            
            for idx_fn = 1:numel(state_fns) % loop over struct fieldnames
                obj.sim_data.j_state.(joint).(state_fns{idx_fn})(idx_t) ...
                    = j_state.(joint).(state_fns{idx_fn});                
            end
        end
    %     toc; tic

        % simulate forward dynamics
        [x_plus,f_plus,err_lim_bool] = obj.forwardDynamicsLcp(q, qd, tau, sim_param.dt);
        if err_lim_bool == 1
            break
        end
        obj.sim_data.x(idx_t,:) = x_plus;
        obj.sim_data.grf.r(idx_t,:) = [f_plus(1), f_plus(3)-f_plus(4)]; % right foot GRFs [normal (x-dir), tangential (y_dir)]
        obj.sim_data.grf.l(idx_t,:) = [f_plus(2), f_plus(5)-f_plus(6)]; % right foot GRFs [normal (x-dir), tangential (y_dir)]
    %     toc; tic

        % calculate center-of-mass velocity %TODO: could do this after sim?
        v_com = obj.comVel(x_plus);
        obj.sim_data.traj.v_com(idx_t,:) = v_com;

        % check liftoff
        q_plus = obj.sim_data.x(idx_t,1:7)';
        foot_pos = obj.footPos(q_plus);
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
            obj.sim_data.info_aerial.t = t; % save liftoff time
            obj.sim_data.info_aerial.v = obj.sim_data.traj.v_com(idx_t,:); % save liftoff velocity
            aerial.state = 1; % set robot aerial state to true

            if sim_param.liftoff_stop == 1
                break % stop sim if liftoff_stop option turned on
            end
        end

        % end sim if torso gets to ground
        obj.sim_data.traj.pos_torso(idx_t,:) = obj.torsoPos(x_plus);
        if obj.sim_data.traj.pos_torso(idx_t,1) <= 0
            break
        end
        
        % end sim if knees go past 180 deg
        theta2 = x_plus(4);
        theta5 = x_plus(7);
        if (theta2 > pi) || (theta5 > pi)
            break
        end
        
    %     toc;
    end
    
end

