classdef JumpingRobot < matlab.mixin.Copyable
    
    properties (SetAccess = private)
        % read-only properties
        repo_folder = 'jumpy-analysis';
        repo_path
        
        config
        joints
       
        state
        sim_data
    end
    properties (Access = private)
        % private properties
        spatialRobot
    end
       
    
    methods
        % Prototypes
        simRobot(obj, sim_param)
        [tau, joint_state] = jointTorque(obj,t,x)
        [x,pos_constraint,err_lim_bool] = forwardDynamicsLcp(obj, q, qd, tau, dt)
        foot_pos = footPos(obj, x)
        v_com = comVel(obj,x)
        torso_pos = torsoPos(obj, x)
        animTrajectory(obj, dt, anim_delay, fig_num, vid, t_span) %TODO: rename (animJump?)
        drawRobot(obj, t, varargin)
        dispJumpSeq(obj, t_seq, dim_subplot, fig_no)
        plotTrajectory(obj)
        exportSimData(obj, fullfile_export)
 

        function obj = JumpingRobot(config_file)
            % Constructor
            obj.repo_path = obj.get_repo_path(); % set repo path
         
            % read YAML configuration file
            obj.config = ReadYaml(config_file);
                        
            % create RBDA struct for library - separate function
            [model, morphology] = obj.buildSpatialRobot(obj.config.morphology);
            obj.spatialRobot = model;
            obj.config.morphology = morphology;
            
            % create knee & hip joints %TODO: just use copy function?
            obj.joints.knee_right = MuscleJoint(obj.config.knee, obj.config.cam);
            obj.joints.knee_left = MuscleJoint(obj.config.knee, obj.config.cam);
            obj.joints.hip_right = MuscleJoint(obj.config.hip, obj.config.cam);
            obj.joints.hip_left = MuscleJoint(obj.config.hip, obj.config.cam);

            % create state
%             theta2 = obj.config.state0.q0(2);
%             theta3 = obj.config.state0.q0(3);
%             theta1 = pi/2 - deg2rad(theta2 + theta3);
%             obj.state.x0 = [0, 0, theta1, deg2rad(obj.config.state0.q0(2:end)),... 
%                             0, 0, deg2rad(obj.config.state0.qd0)];
%             
%             
            % create initial state vector, update inital robot pose, and
            % initialize 'sim_data'
            obj.updateInitialState();
% 
%             % initialize simulation time %TODO: replace 'sim_data' with 'traj'?
% 
%             obj.sim_data.t = 0; %TODO: only for anim? maybe just have option for [] in anim
%             obj.sim_data.x = obj.state.x0;
        end
        
        
        function repo_path = get_repo_path(obj)
            % get path to repository folder
            class_path = which('JumpingRobot');
            idx_repo = strfind(class_path, obj.repo_folder);
            repo_path = class_path(1:(idx_repo-1 + length(obj.repo_folder)));
        end
        
                
        function updateConfig(obj, config)
            config_fns = fieldnames(config);
            for i = 1:numel(config_fns) % loop over fieldnames in 'config'
                config_fn = config_fns{i};
                
                % update all 'config' struct parameters 
                fns = fieldnames(config.(config_fn));
                for j = 1:numel(fns)
                    obj.config.(config_fn).(fns{j}) = config.(config_fn).(fns{j});
                end
                
                % update joint object parameters
                if strcmp(config_fn, 'cam')
                    joints_fns = fieldnames(obj.joints);
                    for j = 1:numel(joints_fns) % update cam parameters for all joints
                        obj.joints.(joints_fns{j}).set_cam_param(obj.config.cam);
                    end
                end
                if strcmp(config_fn, 'knee')                    
                    sides = {'right','left'};
                    for k = 1:numel(sides) % update joint-specific parameters for knees
                        obj.joints.(['knee_', sides{k}]).set_joint_param(obj.config.knee);
                    end
                end
                if strcmp(config_fn, 'hip')                    
                    sides = {'right','left'};
                    for k = 1:numel(sides) % update joint-specific parameters for hips
                        obj.joints.(['hip_', sides{k}]).set_joint_param(obj.config.hip);
                    end
                end
                    
                % update robot morphology parameters (create new spatial)
                if strcmp(config_fn, 'morphology')
                    [model, morphology] = obj.buildSpatialRobot(obj.config.morphology);
                    obj.spatialRobot = model;
                    obj.config.morphology = morphology;                
                end
                   
                % update initial state (reset 'sim_data')
                obj.updateInitialState();                
            end
        end

        
        function updateInitialState(obj)
            % Update initial state vector & robot pose, and initialize 'sim_data'
            theta2 = obj.config.state0.q0(2);
            theta3 = obj.config.state0.q0(3);
            theta1 = pi/2 - deg2rad(theta2 + theta3); % ground-shank joint angle calc for symmetric frame & level torso
            obj.state.x0 = [0, 0, theta1, deg2rad(obj.config.state0.q0(2:end)),... 
                            0, 0, deg2rad(obj.config.state0.qd0)];
            
            %TODO: replace 'sim_data' with 'traj'?
            obj.sim_data.x = obj.state.x0; % this allows for plotting of initial pose
            obj.sim_data.t = 0; % this allows for plotting of initial pose
        end
        
        
        function calcJumpTrajectory(obj)
            % calculate jump trajectory
            obj.sim_data.traj.pos_jump = [obj.sim_data.traj.pos_torso(:,1) - ... % jump height trajectory
                (obj.config.morphology.l(1) + obj.config.morphology.l(2)), ...
                obj.sim_data.traj.pos_torso(:,2) - obj.sim_data.traj.pos_torso(1,2)];

            [h_jump, idx_jump] = max(obj.sim_data.traj.pos_jump(:,1));
            obj.sim_data.info_jump.height = h_jump;
            obj.sim_data.info_jump.t_max_height = obj.sim_data.t(idx_jump);
            obj.sim_data.info_jump.dist = obj.sim_data.traj.pos_torso(end,2);
        end

        
        
        
        % functions:
        % exportConfig

        

        


        
    end
    
    
    methods (Static)
        % prototypes
        [model, morphology] = buildSpatialRobot(morphology)
    end
    
    
    methods (Access = protected)
        function cpObj = copyElement(obj)
            % Override copyElement method
            cpObj = copyElement@matlab.mixin.Copyable(obj);
            
            % copy joint objects
            fn = fieldnames(obj.joints);
            for i = 1:numel(fn)
                cpObj.joints.(fn{i}) = copy(obj.joints.(fn{i}));
            end
        end
    end
    
    

        
    
end % end class