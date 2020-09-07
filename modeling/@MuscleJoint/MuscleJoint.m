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
        getMuscPresFromCurvefit(obj, t, robot);
        integrateMuscPres(obj, t, robot);
        mdot_out = fminconTubeMassFlow(obj, p1, p2, l_tube, d_tube,...
            eps_tube, n_tube_seg, gas_props);
        mdot_tube= tubeMassFlowConstMu(obj, p1, p2)

        
        function obj = MuscleJoint(joint_param, cam_param, pneu_param)
            % Constructor
            obj.repo_path = obj.get_repo_path(); % get path to repository
            
            % set up object joint-specific parameters & convert angles
            obj.set_joint_param(joint_param);

            % set up object cam parameters & convert angles
            obj.set_cam_param(cam_param);
            
            % create/load cam map if it exists
            obj.cam_map = containers.Map; % initialize to map
            obj.load_cam_data(); % load cam data if exists
            
            % load pneumatic data
            obj.pneu_param = pneu_param;
            obj.loadPneumaticData()
            
            % initialize joint state
            obj.state = obj.createJointStateStruct(1);       
            obj.state.muscle_pressure = (14.7)*6.89476; % (psi to kPa)
            obj.state.muscle_volume = obj.calcMuscVolume(0);
            obj.state.rho_gas = interp1(obj.gas_props.pres, obj.gas_props.rho, ...
                obj.state.muscle_pressure*1e-3, 'makima')*1000; % (p: kPa to MPA) (rho: g/mL to kg/m^3)
            obj.state.m_gas = obj.state.rho_gas*obj.state.muscle_volume; % (kg/m^3 * m^3 = kg)
            
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
          
        function loadPneumaticData(obj)
            % Load data: muscle volume, gas properties
            
            % load muscle volume curvefit
            if isempty(obj.musc_vol_curvefit) 
                musc_vol_curvefit_path = fullfile(obj.repo_path,...
                    'modeling', '@MuscleJoint', 'muscle-data', 'volume',...
                    'volume_curve_fit-poly3-water2.mat');
                curvefit = load(musc_vol_curvefit_path, 'fit3_water2');
                obj.musc_vol_curvefit = curvefit.('fit3_water2');
            end

            % load gas properties
            if isempty(obj.gas_props)
                gas_data_path = fullfile(obj.repo_path,...
                    'modeling', '@MuscleJoint', 'gas-data',...
                    [obj.pneu_param.gas, '.txt']);
                fileId = fopen(gas_data_path);
                (textscan(fileId, '%s',14,'Delimiter','\t\')); % header
                rawData = textscan(fileId, '%f %f %f %f %f %f %f %f %f %f %f %f %f %s');
                numData = cell2mat(rawData(:,1:13));
                obj.gas_props.pres = numData(:,2); % (MPa)
                obj.gas_props.rho = numData(:,3); % (g/mL)
                obj.gas_props.mu = numData(:,12); % (Pa*s)
                fclose(fileId);
            end
        end
        
        
        %% cam/joint geometry stuff
        function joint_angle_sign = get_joint_angle_sign(obj)
            % determine sign of joint angle relative to cam parameterization angle
            joint_angle_sign = sign(obj.joint_param.theta_range(2)-...
                obj.joint_param.theta_range(1));
        end
       
       
        function cam_angle = convert_joint_angle_to_cam_angle(obj, joint_angle)
            % convert joint angle to corresponding cam angle
            cam_angle = (joint_angle - obj.joint_param.theta_range(1))*obj.get_joint_angle_sign();
        end

                
        function update_cam_data(obj, sweep_arr)
            % calculate effective moment arm for all cam geometries
            % not previously saved; sweep_vec = [radius0, slope]
            fprintf('calculating cam data...\n')
            d = obj.cam_param.d;
            phi_range = obj.cam_param.phi_range;
            
%             obj.load_cam_data(); % re-load cam data (for parallel i think) TODO: required if saving is an option?
            s = size(sweep_arr); % get sweep array size
            n_sweep = s(1); % get sweep array length (rows)
            beta_vec = obj.cam_param.beta_vec;
            n_beta = length(beta_vec);
            tic
            for idx_sweep = 1:n_sweep % loop over sweep vector
                cam_rad0_tmp = sweep_arr(idx_sweep,1); % (cm) cam radius at zero degrees
                cam_slope_tmp = sweep_arr(idx_sweep,2); % (cm/rad) cam profile radius slope
                key = obj.cam_map_key(cam_rad0_tmp, cam_slope_tmp);
                
                if ~obj.cam_map.isKey(key) % if key doesn't already exist, calculate data
                    ema_vec = zeros(1,n_beta);
                    linear_displace_vec = zeros(1,n_beta);

                    calc_ema = @(beta,r0,m,d,phi_range)obj.calc_ema(beta,r0,m,d,phi_range); % create anonymous fcn for parfor
                    parfor idx_beta = 1:n_beta % loop over joint angles TODO: change back to parfor
                        beta = beta_vec(idx_beta);
                        geom = calc_ema(beta,cam_rad0_tmp,cam_slope_tmp,d,phi_range); % calculate cam geometry
                        ema_vec(idx_beta) = geom.ema; % add effective moment arm
                        linear_displace_vec(idx_beta) = geom.l_disp; % add linear joint displacement                         
                    end
            
                    obj.cam_map(key) = {ema_vec, linear_displace_vec}; % update temp cam map
                    
                    fprintf('    updated %i of %i cam profiles, %4.1f elapsed\n',...
                        idx_sweep, n_sweep, toc)
                end
            end
            fprintf('all cam profiles updated, %0.2f s elapsed\n\n', toc)

            %TODO: make saving an option?
            cam_map_saved = obj.cam_map; % create cam map save var
            cam_param_saved = obj.cam_param; % create param save var
            save(obj.cam_param.filename, 'cam_map_saved', 'cam_param_saved') % save cam map & params
        end
        
        
        function updateGeom(obj, theta, theta0)
            % Update moment arm & joint displacement at current joint angle
            % (interp/extrap from saved data)
            
            key = obj.cam_map_key();
            if ~obj.cam_map.isKey(key) % if key doesn't already exist, calculate data
                fprintf('NOT KEY: %s\n', key)
                obj.update_cam_data([obj.joint_param.rad0, obj.joint_param.slope]);
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
            obj.state.linear_displace = l_disp - l_disp0; % switch sign so that positive joint disp corresponds to negative MTU disp 
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
            % Calculate muscle volume
            l_tube = obj.pneu_param.l_tube_valve_musc*0.0254; % (in to m) tube length
            d_tube = obj.pneu_param.d_tube_valve_musc*0.0254; % (in to m) tube inner diameter
            vol_musc = obj.musc_vol_curvefit(cont_musc)*1e-6; % (mL to m^3) muscle vol from curvefit
            vol_musc = vol_musc + l_tube*pi*(d_tube/2)^2; % (m^3) add tubing volume to muscle
        end
        
        
        function updateMtu(obj)
            % calculate muscle contraction length, muscle volume, 
            % corresponding tendon displacement, and muscle force            
            persistent psf
            if isempty(obj.musc_force_surffit) % load force-length-pressure surface fit
                surf_fit_path = fullfile(obj.repo_path,...
                    'modeling', '@MuscleJoint', 'muscle-data', 'force-calibration',...
                    'surface_fit-clean_cut_1p25D_crimp_over_tube_sleeve_345kpa.mat');
                
                sf = load(surf_fit_path, 'sf'); 
                obj.musc_force_surffit = sf.('sf');
                psf = [obj.musc_force_surffit.p30,...
                       obj.musc_force_surffit.p20, obj.musc_force_surffit.p21,...
                       obj.musc_force_surffit.p10, obj.musc_force_surffit.p11, obj.musc_force_surffit.p12,...
                       obj.musc_force_surffit.p00, obj.musc_force_surffit.p01, obj.musc_force_surffit.p02, obj.musc_force_surffit.p03];
            end

            pres = obj.state.muscle_pressure - 14.7*6.89476; % (kPa) gauge pressure
            disp = obj.state.linear_displace;
            
            % calculate muscle contraction from muscle-tendon force balance
            %   (p00 + p01*pres + p02*pres^2 + + p03*pres^3) 
            % + (p10 + p11*pres + p12*pres^2)*cont 
            % + (p20 + p21*pres)*cont^2 
            % + p30*cont^3 
            % = k*disp + k*cont
            p = [psf(1),... % cont^3
                 (psf(2) + psf(3)*pres),... % cont^2
                 (psf(4) + psf(5)*pres + psf(6)*pres^2 - obj.joint_param.k_tendon),... % cont
                 (psf(7) + psf(8)*pres + psf(9)*pres^2 + psf(10)*pres^3 - obj.joint_param.k_tendon*disp)]; % constant 
            r = roots(p);
            cont_musc = r(3); % (cm)
       
%             if pres == 0 % if no pressure, no contraction
%                 cont_musc = 0;
%             end
            
            % calculate corresponding MTU force and tendon stretch
            if cont_musc <= 0 % if zero / negative contraction
                %TODO: rethink infinitely stiff/no tendon 
                if obj.joint_param.k_tendon >= 1e5 % if tendon is approximately infinitely stiff                    
                    force = max(0,obj.musc_force_surffit(0,pres)) + 1e4*(-cont_musc); % make muscle very stiff in tension (1e2 N/cm)
                    stretch_tendon = 0;
                else
                    cont_musc = 0; % set contraction to zero (assume no stretch)
                    stretch_tendon = max(0,disp); % disregard negative displacement
                    force = obj.joint_param.k_tendon*stretch_tendon;
                end
            elseif cont_musc >= (6.5 + pres/300) % if beyond range of surf fit in contraction direction
                cont_musc = 6.5 + pres/300;
                force = 0;
                stretch_tendon = 0;
            else % if positive contraction in surface fit range
                force = max(0,obj.musc_force_surffit(cont_musc,pres)); % disregard negative forces from fit
                stretch_tendon = force/obj.joint_param.k_tendon;
            end
            
            % calculate muscle volume
            vol_musc = obj.calcMuscVolume(cont_musc);
            
            % update state 
            obj.state.muscle_contract_prev = obj.state.muscle_contract;
            obj.state.muscle_contract = cont_musc;
            obj.state.muscle_volume = vol_musc;
            obj.state.tendon_stretch = stretch_tendon;
            obj.state.mtu_force = force; 
        end
        
        
        %% Joint torque
        function updateJointTorque(obj)
            % Update joint torque generated by MTU
            obj.state.torque = obj.state.mtu_force ...
                *obj.state.moment_arm/100; % (Nm) calculate joint torque 
        end

        
        %% main method
        function updateJointState(obj, theta0, theta, t_valve, robot)
            % Compute joint torque created by pneumatic actuator & cam
            % (initial joint angle, current joint angle,
            % time relative to valve open)
                
            obj.integrateMuscPres(t_valve, robot); % integrate pressure in time
%             obj.getMuscPresFromCurvefit(t_valve, robot); % get pressure from response curve
            % obj.state.muscle_pressure

            obj.updateGeom(theta, theta0); % calculate moment arm & displacement
            % obj.state.moment_arm
            % obj.state.linear_displace

            obj.updateMtu(); % calculate contraction force & muscle/tendon lengths
            % obj.state.muscle_contract_prev
            % obj.state.muscle_contract
            % obj.state.muscle_volume 
            % obj.state.tendon_stretch
            % obj.state.mtu_force

            updateJointTorque(obj); % (Nm) calculate joint torque 
            % obj.state.torque
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
            state.rho_gas = zeros(n_elem,1);
            state.m_gas = zeros(n_elem,1);
            state.mdot_gas = zeros(n_elem,1);
            state.muscle_pressure = zeros(n_elem,1);
            state.linear_displace = zeros(n_elem,1);
            state.ema = zeros(n_elem,1);
            state.muscle_contract_prev = zeros(n_elem,1);
            state.muscle_contract = zeros(n_elem,1);
            state.muscle_volume = zeros(n_elem,1);
            state.tendon_stretch = zeros(n_elem,1);
            state.mtu_force = zeros(n_elem,1);
            state.torque = zeros(n_elem,1);
        end
        
    end % end methods (static)
end % end class
    
    


