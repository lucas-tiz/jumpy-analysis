classdef MuscleJoint < matlab.mixin.Copyable % copyable handle class
    % revolute joint actuated by pneumatic muscle
    properties        
%         joint_param % joint-specific parameters
%         cam_param % cam parameters for calculating cam_map data
%         cam_map % moment arm and linear displacement data
    end
    
    properties (SetAccess = private) % read-only properties
        repo_folder = 'jumpy-analysis';
        repo_path

        joint_param % joint-specific parameters
        cam_param % cam parameters for calculating cam_map data
        cam_map % moment arm and linear displacement data
        pneu_param % pneumatic parameters
        
        state % joint state
    end
    
    properties (Access = private) % private properties
        musc_pres_curvefit % time-pressure curvefit (transient response)
        musc_force_surffit % pressure-length-force surface fit
        gas_props % gas properties (versus pressure)
        musc_vol_curvefit % muscle volume versus contraction length
    end
        
    
    %%
    methods
        % Prototypes
        calculateCamData(obj, sweep_arr);
        
        getMuscPresFromCurvefit(obj, t, robot);
        integrateMuscPres(obj, t, robot);
        mdot_out = fminconTubeMassFlow(obj, p1, p2, l_tube, d_tube,...
            eps_tube, n_tube_seg, gas_props);
        mdot_tube= tubeMassFlowConstMu(obj, p1, p2)
        
        updateJointGeometry(obj, theta, theta0);
        updateMtu(obj, p_musc);

        
        function obj = MuscleJoint(joint_param, cam_param, pneu_param,...
                gas_props)
            % Constructor
            obj.repo_path = obj.get_repo_path(); % get path to repository
            
            % set parameters passed in
            obj.set_joint_param(joint_param); % set joint-specific parameters & convert angles
            obj.set_cam_param(cam_param); % set object cam parameters & convert angles
            obj.set_pneumatic_param(pneu_param); % set pneumatic paramters
            obj.set_gas_props(gas_props); % set gas properties
            
            % create/load cam map if it exists
            obj.cam_map = containers.Map; % initialize to map
            obj.load_cam_data(); % load cam data if exists           
 
            % initialize joint state
            obj.state = obj.createJointStateStruct(1);       
            obj.state.p_musc = 101.325; % (kPa)
            obj.state.vol_musc = obj.calcMuscVolume(0);
            obj.state.rho_musc = interp1(obj.gas_props.pres, obj.gas_props.rho, ...
                obj.state.p_musc*1e-3, 'makima')*1000; % (p: kPa to MPA) (rho: g/mL to kg/m^3)
            obj.state.m_musc = (obj.state.p_musc*obj.state.vol_musc)/...
                (obj.gas_props.Rs*obj.gas_props.T/1000); % (Pa to kPa)  
                        
%             % add robot parent class if provided
%             if nargin > 2
%                 obj.robot = varargin{1};
%             end
        end
        
        
        function repo_path = get_repo_path(obj)
            class_path = which('MuscleJoint');
            idx_repo = strfind(class_path, obj.repo_folder);
            repo_path = class_path(1:(idx_repo-1 + length(obj.repo_folder)));
        end
        
        
        function filename_cam_data = create_cam_filename(obj)
            % create file name for cam data from cam parameters            
            filename_cam_data = fullfile(obj.repo_path, 'modeling',... % cam data file name
                '@MuscleJoint', 'cam-data',... 
                ['cam_data_',...
                num2str(obj.cam_param.d), 'd_',...
                num2str(rad2deg(obj.cam_param.phi_range(1))),'to',...
                num2str(rad2deg(obj.cam_param.phi_range(2))),'phi_',...
                num2str(rad2deg(obj.cam_param.beta_vec(1))),'to',...
                num2str(rad2deg(obj.cam_param.beta_vec(end))),'beta',...
                '.mat']);
        end
        
        
        function load_cam_data(obj)
            % load cam data from .mat file if corresponding data exists
            if isfile(obj.cam_param.filename) % check file already exists
                load(obj.cam_param.filename, 'cam_map_saved', 'cam_param_saved')
                if ((cam_param_saved.d == obj.cam_param.d) &&... % check parameters are the same TODO: necessary
                    (isequal(cam_param_saved.phi_range, obj.cam_param.phi_range)) &&...
                    (isequal(cam_param_saved.beta_vec, obj.cam_param.beta_vec)) == 1)

                    obj.cam_map = cam_map_saved; % update cam  map with loaded data
                end
            end
        end
                  
        
        %% cam/joint geometry stuff
        function joint_angle_sign = get_joint_angle_sign(obj)
            % Determine sign of joint angle relative to cam parameterization angle
            joint_angle_sign = sign(obj.joint_param.theta_range(2)-...
                obj.joint_param.theta_range(1));
        end
       
       
        function cam_angle = convert_joint_angle_to_cam_angle(obj, joint_angle)
            % Donvert joint angle to corresponding cam angle
            cam_angle = (joint_angle - obj.joint_param.theta_range(1))*obj.get_joint_angle_sign();
        end
  
        function key = cam_map_key(obj, varargin)
            % Format 'cam_map' key
            if nargin > 1
                rad0 = varargin{1};
                slope = varargin{2};
            else
                rad0 = obj.joint_param.rad0;
                slope = obj.joint_param.slope;
            end
%             key = [num2str(obj.joint_param.rad0),',', ...
%                 num2str(obj.joint_param.slope)]; % NOTE: no rounding
            key = [num2str(round(rad0,1)), ',', num2str(round(slope,1))]; %NOTE: rounding
        end
        
        
        %% MTU stuff
        function vol_musc = calcMuscVolume(obj, cont_musc)
            % load muscle volume curvefit
            if isempty(obj.musc_vol_curvefit) 
                musc_vol_curvefit_path = fullfile(obj.repo_path,...
                    'modeling', '@MuscleJoint', 'muscle-data', 'volume',...
                    'volume_curve_fit-poly3-water2.mat');
                curvefit = load(musc_vol_curvefit_path, 'fit3_water2');
                obj.musc_vol_curvefit = curvefit.('fit3_water2');
            end
            
            % Calculate muscle volume
            min_vol = obj.musc_vol_curvefit(0); % min musc vol (shouldn't change w/ neg contract)
            vol_musc = max(min_vol, obj.musc_vol_curvefit(cont_musc))*1e-6; % (mL to m^3) muscle vol from curvefit
            
            l_tube = obj.pneu_param.l_tube_valve_musc*0.0254; % (in to m) tube length
            d_tube = obj.pneu_param.d_tube_valve_musc*0.0254; % (in to m) tube inner diameter
            vol_musc = vol_musc + l_tube*pi*(d_tube/2)^2; % (m^3) add tubing volume to muscle TODO: is this right?           
        end
        
        
        %% Joint torque
        function updateJointTorque(obj)
            % Update joint torque generated by MTU
            obj.state.torque = obj.state.force_mtu ...
                *obj.state.moment_arm/100; % (Nm) calculate joint torque 
        end

        
        %% main method
        function updateJointState(obj, theta0, theta, t_valve, robot)
            % Compute joint torque created by pneumatic actuator & cam
            % (initial joint angle, current joint angle,
            % time relative to valve open)
                
            obj.updateJointGeometry(theta, theta0); % calculate moment arm & displacement
            obj.integrateMuscPres(t_valve, robot); % integrate pressure in time
                % includes obj.updateMtu()
            updateJointTorque(obj); % (Nm) calculate joint torque 

%             obj.updateJointGeometry(theta, theta0); % calculate moment arm & displacement
%             obj.getMuscPresFromCurvefit(t_valve, robot); % get pressure from response curve
%             obj.updateMtu(); % calculate contraction force & muscle/tendon lengths
%             updateJointTorque(obj); % (Nm) calculate joint torque 
        end
        
        
        %% Getters
        function cam_param = get_cam_param(obj)
            cam_param = obj.cam_param;
        end
        
        
        function cam_map = get_cam_map(obj)
            cam_map = obj.cam_map;
        end
        
        
        function musc_force_surffit = get_musc_force_surffit(obj)
            musc_force_surffit = obj.musc_force_surffit;
        end
        
        
        %% Setters
        function set_joint_param(obj, joint_param)
            obj.joint_param.rad0 = joint_param.rad0;
            obj.joint_param.slope = joint_param.slope;
            obj.joint_param.theta_range = deg2rad(joint_param.theta_range);
            obj.joint_param.k_tendon = joint_param.k_tendon; 
        end
        
        
        function set_cam_param(obj, cam_param)
            obj.cam_param.d = cam_param.d;
            obj.cam_param.phi_range = deg2rad(cam_param.phi_range);
            obj.cam_param.beta_range = deg2rad(cam_param.beta_range);
            obj.cam_param.beta_inc = deg2rad(cam_param.beta_inc);
            obj.cam_param.beta_vec = deg2rad(cam_param.beta_range(1):...
                cam_param.beta_inc:cam_param.beta_range(2));
            
            obj.cam_param.filename = obj.create_cam_filename(); % construct cam file name from parameters
        end
        
        function set_pneumatic_param(obj, pneu_param)
            obj.pneu_param = pneu_param; % set pneumatic parameters
        end                    

        function set_gas_props(obj, gas_props)
            obj.gas_props = gas_props; % set gas properties
        end
            
        
    end % end methods
      
    
    %%
    methods (Static)       
        function geom = calc_ema(beta, r0, m, d, phi_range)
            % calculate effective moment arm for cam/joint parameters &
            % calculate linear joint displacement
            % units: (rad, cm, cm/rad, cm, rad)
            persistent phi_sym
            if isempty(phi_sym)
                phi_sym = sym('phi_sym');
            end

            % if no cam profile exists, just return zeros
            if (r0 == 0) && (m == 0)
                ema = 0;
                r_tan = 0;
                phi = 0;
                psi = 0;
                l_cam = 0;
            else
                % formulate slopes
                slope_cam = ( m*cos(beta-phi_sym) + (r0+m*phi_sym)*sin(beta-phi_sym) ) /...
                    ( (r0+m*phi_sym)*cos(beta-phi_sym) - m*sin(beta-phi_sym) );
                slope_muscle = -( (r0+m*phi_sym)*cos(beta-phi_sym) ) /...
                    (-d + (r0+m*phi_sym)*sin(beta-phi_sym) );

                % solve for tangent angle
                phi_solve = zeros(3,1);
                phi_solve(1) = max([-100,vpasolve(slope_cam == slope_muscle, phi_sym, deg2rad(0))]);
                phi_solve(2) = max([-100,vpasolve(slope_cam == slope_muscle, phi_sym, deg2rad(90))]);
                phi_solve(3) = max([-100,vpasolve(slope_cam == slope_muscle, phi_sym, deg2rad(180))]);

                % filter out negative tangent radii (not physically possible)
                r_tan_solve = r0 + m*phi_solve; % radius at tangent point
                idx = r_tan_solve >= 0;
                phi_solve = phi_solve(idx);

                % remove negative slopes
                slope_solve = -( (r0+m*phi_solve).*cos(beta-phi_solve) ) ./...
                    (-d + (r0+m*phi_solve).*sin(beta-phi_solve) );
                idx = slope_solve >= 0;
                phi_solve = phi_solve(idx);
                slope_solve = slope_solve(idx);

                % filter by max slope
                [~,idx] = max(slope_solve);
                phi = phi_solve(idx); % cam parameterization angle at tangent point 
                slope = slope_solve(idx); % slope at tangent point

                % adjust for tangent points outside cam parameterization range
                if phi < phi_range(1)
                    phi = phi_range(1);
                    slope = -( (r0+m*phi)*cos(beta-phi) ) / (-d + (r0+m*phi)*sin(beta-phi) );
                elseif phi > phi_range(2)
                    phi = phi_range(2);
                    slope = -( (r0+m*phi)*cos(beta-phi) ) / (-d + (r0+m*phi)*sin(beta-phi) );
                end

                % calculate final values
                r_tan = r0 + m*phi; % radius at tangent point
                psi = atan(slope); % psi angle
                ema = d*sin(psi); % effective moment arm
                
                % calculate cam displacement length
                if (m == 0)
                    l_cam = r0*phi;
                else
                    l_cam = (m*log(r0 + sqrt((r0+m*phi).^2 + m^2) + m*phi))/2 + ...
                        ((r0+m*phi).*sqrt((r0+m*phi).^2 + m^2))/(2*m);
                end
            end

            % if no physically possible values found
            if isempty(ema)
                ema = 0;
                r_tan = 0;
                phi = 0;
                psi = 0;
                l_cam = 0;
            end
            
            % calculate muscle length
            l_musc = sqrt( (-(r0+m*phi)*sin(beta-phi) + d)^2 ... 
                + ((r0+m*phi)*cos(beta-phi))^2 );
            
            % linear displacement: subtract l_disp0 = -(l_cam0 - l_musc0) for relative displacement
            l_disp = -(l_cam - l_musc); % switch sign so that positive joint disp corresponds to negative MTU disp
            
            % add data to return struct
            geom.ema = ema; % (cm)
            geom.l_disp = l_disp; % (cm)
            geom.r_tan = r_tan; % (cm)
            geom.phi = phi; % (rad)
            geom.psi = psi; % (rad)
        end

        
        function state = createJointStateStruct(n_elem)
            % updateJointGeometry():
            state.linear_displace = zeros(n_elem,1);
            state.moment_arm = zeros(n_elem,1);
            
            % integrateMuscPres():
            state.pdot_musc = zeros(n_elem,1);
            state.mdot_musc = zeros(n_elem,1);
            state.p_musc = ones(n_elem,1)*101.325;
            state.m_musc = zeros(n_elem,1);
            state.rho_musc = zeros(n_elem,1);
            state.valve = zeros(n_elem,1);
            % updateMtu():
            state.contract_musc_prev = zeros(n_elem,1);
            state.contract_musc = zeros(n_elem,1);
            state.vol_musc = zeros(n_elem,1);
            state.elong_tendon = zeros(n_elem,1);
            state.force_mtu = zeros(n_elem,1);
            
            % updateJointTorque():
            state.torque = zeros(n_elem,1);
        end
        
    end % end methods (static)
end % end class
    
    


