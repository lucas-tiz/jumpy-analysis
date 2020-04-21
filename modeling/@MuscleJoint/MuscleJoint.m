classdef MuscleJoint < matlab.mixin.Copyable %handle
    % revolute joint actuated by pneumatic muscle
    properties        
%         joint_param % joint-specific parameters
%         cam_param % cam parameters for calculating cam_map data
%         cam_map % moment arm and linear displacement data
    end
    
    properties (SetAccess = private)
        % read-only properties
        repo_folder = 'jumpy-analysis';
        repo_path

        joint_param % joint-specific parameters
        cam_param % cam parameters for calculating cam_map data
        cam_map % moment arm and linear displacement data
    end
    
    properties (Access = private)
        % private properties
        pressure_curvefit % time-pressure curvefit (transient response)
        step_resp_f_steady % steady-state force for pressure step response
        force_sf % pressure-length-force surface fit
    end
        
    
    %%
    methods
        function obj = MuscleJoint(joint_param, cam_param)
            % Constructor
            obj.repo_path = obj.get_repo_path(); % get path to repository
            
            % set up object joint-specific parameters & convert angles
            obj.set_joint_param(joint_param);

            % set up object cam parameters & convert angles
            obj.set_cam_param(cam_param);

            % create/load cam map if it exists
            obj.cam_map = containers.Map; % initialize to map
            obj.load_cam_data(); % load cam data if exists
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
        
        
        %% pressure stuff
        function pres = calc_pres_response(obj, t, p_max)
            % estimate transient pressure response during muscle inflation
            % using Instron force response curve
            if isempty(obj.pressure_curvefit) % create pressure-time curve fit
                pres_response_path = fullfile(obj.repo_path,...
                    'modeling', '@MuscleJoint', 'muscle-data', 'pressure-response',...
                    'response-clean_cut_1p25D_0p19inID_quickconnect_noslack-raw_data.csv');
              
                [t_vec,f_vec,~,f_steady] = obj.get_step_data(pres_response_path,...
                    [1 4], 0.1, 1.5);
                obj.step_resp_f_steady = f_steady;                
                pres_vec = f_vec;
                obj.pressure_curvefit = fit(t_vec*3.5,pres_vec,'linearinterp'); %NOTE: 3.5 used to slow response down
            end

            if t < 0
                pres = 0;
            elseif t > obj.pressure_curvefit.p.breaks(end) % if time is beyond transient
                pres = obj.pressure_curvefit.p.coefs(end,2); % set pressure to max
            else
                pres = obj.pressure_curvefit(t); % set pressure based on curve fit
            end
            
            pres = pres*p_max/obj.step_resp_f_steady; % scale curve to p_max
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

                key = [num2str(cam_rad0_tmp),',',num2str(cam_slope_tmp)]; % map key
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
%             fprintf('all cam profiles updated, %0.2f s elapsed\n\n', toc)

            %TODO: make saving an option?
%             cam_map_saved = obj.cam_map; % create cam map save var
%             cam_param_saved = obj.cam_param; % create param save var
%             save(obj.cam_param.filename, 'cam_map_saved', 'cam_param_saved') % save cam map & params
        end
        
        
        function [ema, linear_displace] = get_ema(obj, theta, theta0)
            % get effective moment arm at current joint angle
            % (interp/extrap from saved data)
            
            %DEBUG: trying rounding
%             key = [num2str(obj.joint_param.rad0),',',num2str(obj.joint_param.slope)]; % map key
            key = [num2str(round(obj.joint_param.rad0),1), ',', ...
                num2str(round(obj.joint_param.slope),1)]; % map key
            
            if ~obj.cam_map.isKey(key) % if key doesn't already exist, calculate data
                fprintf('NOT KEY: %s\n', key)
                obj.update_cam_data([obj.joint_param.rad0, obj.joint_param.slope]);
            end

            cam_data = obj.cam_map(key); % get cam data
            ema_vec = cam_data{1}; % get effective moment arm data
            linear_displace_vec = cam_data{2};% get linear joint displacement data
            beta_vec = obj.cam_param.beta_vec; % get beta angle data
            
            beta0 = obj.convert_joint_angle_to_cam_angle(theta0); % convert joint angle to cam angle
            beta = obj.convert_joint_angle_to_cam_angle(theta); % convert joint angle to cam angle
            ema = interp1(beta_vec, ema_vec, beta,... % calculate ema at current angle
                'linear', 'extrap');             
                                                    
            l_disp0 = interp1(beta_vec, linear_displace_vec, beta0,... % calculate displacement from initial angle to sim starting angle
                'linear', 'extrap'); 
            l_disp = interp1(beta_vec, linear_displace_vec,... % calculate displacement from initial angle to current angle
                beta, 'linear', 'extrap');
            linear_displace = l_disp - l_disp0; % switch sign so that positive joint disp corresponds to negative MTU disp 
        end
        
        
        %% MTU stuff
        function [cont_musc, stretch_tendon, force] = calcMtuState(obj, pres, disp)
            % calculate muscle contraction & corresponding tendon displacement
            persistent psf
            if isempty(obj.force_sf) % load force-length-pressure surface fit
                surf_fit_path = fullfile(obj.repo_path,...
                    'modeling', '@MuscleJoint', 'muscle-data', 'force-calibration',...
                    'surface_fit-clean_cut_1p25D_crimp_over_tube_sleeve_345kpa.mat');
                
                sf = load(surf_fit_path, 'sf'); 
                obj.force_sf = sf.('sf');
                psf = [obj.force_sf.p30,...
                       obj.force_sf.p20, obj.force_sf.p21,...
                       obj.force_sf.p10, obj.force_sf.p11, obj.force_sf.p12,...
                       obj.force_sf.p00, obj.force_sf.p01, obj.force_sf.p02, obj.force_sf.p03];
            end

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
            cont_musc = r(3);
       
%             if pres == 0 % if no pressure, no contraction
%                 cont_musc = 0;
%             end
            
            if cont_musc <= 0 % if zero / negative contraction
                %TODO: rethink infinitely stiff/no tendon 
                if obj.joint_param.k_tendon >= 1e5 % if tendon is approximately infinitely stiff                    
                    force = max(0,obj.force_sf(0,pres)) + 1e4*(-cont_musc); % make muscle very stiff in tension (1e2 N/cm)
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
                force = max(0,obj.force_sf(cont_musc,pres)); % disregard negative forces from fit
                stretch_tendon = force/obj.joint_param.k_tendon;
            end           
        end
        
        
        %% main method
        function joint_state_struct = calcJointState(obj, theta0, theta, t_act, p_max)
            % compute joint torque created by pneumatic actuator & cam
            % (cam & tendon parameters, initial joint angle, current joint angle,
            % time from actuation start)

            len_t = length(t_act);
            muscle_pressure = zeros(len_t, 1);
            linear_displace = zeros(len_t, 1);
            ema = zeros(len_t, 1);
            muscle_contract = zeros(len_t, 1);
            tendon_stretch = zeros(len_t, 1);
            mtu_force = zeros(len_t, 1);
            torque = zeros(len_t, 1);
            
%             obj.load_cam_data(); % re-load cam data

            for i = 1:length(t_act)
                muscle_pressure(i) = obj.calc_pres_response(t_act(i),p_max); % get pressure from transient pressure response data (step response)
                
                [ema(i), linear_displace(i)] = obj.get_ema(theta(i), theta0); % calculate effective moment arm & displacement
                
                [muscle_contract(i), tendon_stretch(i), mtu_force(i)] = ...
                    obj.calcMtuState(muscle_pressure(i), linear_displace(i)); % calculate contraction force & muscle/tendon lengths

                torque(i) = mtu_force(i)*ema(i)/100; % calculate joint torque (Nm)
            end

            joint_state_struct.muscle_pressure = muscle_pressure;
            joint_state_struct.linear_displace = linear_displace;
            joint_state_struct.ema = ema;
            joint_state_struct.muscle_contract = muscle_contract;
            joint_state_struct.tendon_stretch = tendon_stretch;
            joint_state_struct.mtu_force = mtu_force;
            joint_state_struct.torque = torque;
        end
        
        
        %% Getters
        function cam_param = get_cam_param(obj)
            cam_param = obj.cam_param;
        end
        
        
        function cam_map = get_cam_map(obj)
            cam_map = obj.cam_map;
        end
        
        
        function force_sf = get_force_sf(obj)
            force_sf = obj.force_sf;
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
        %TODO: just put this inside pressure function
        function [t_step,f_step,t_peak,f_steady] = get_step_data(filename,...
                t_baseline_range, f_thresh, t_steady)
            % align step response start curve with t = 0
            fid = fopen(filename, 'r');
            fgets(fid);
            fgets(fid);
            d = textscan(fid, '"%f","%f"', 'Delimiter', ',');
            fclose(fid);

            t = d{:,1};
            f = d{:,2};

            truth_baseline = (t>=t_baseline_range(1) & t<=t_baseline_range(2));
            f_baseline = f(truth_baseline);
            f_baseline_avg = mean(f_baseline);

            idx_step = find(f >= f_baseline_avg + f_thresh);
            t_step = t(idx_step) - t(idx_step(1));
            f_step = f(idx_step);

            idx_steady = find(t_step >= t_steady, 1);
            f_steady = f_step(idx_steady);
            idx_peak = find(f_step >= f_steady, 1);
            t_peak = t_step(idx_peak);
        end
        
        
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
                    l_cam = (m*log(r0 + sqrt((r0+m*phi)^2 + m^2) + m*phi))/2 + ...
                        ((r0+m*phi)*sqrt((r0+m*phi)^2 + m^2))/(2*m);
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

        
        function joint_state_struct = createJointStruct(n_elem)            
            joint_state_struct.muscle_pressure = zeros(n_elem,1);
            joint_state_struct.linear_displace = zeros(n_elem,1);
            joint_state_struct.ema = zeros(n_elem,1);
            joint_state_struct.muscle_contract = zeros(n_elem,1);
            joint_state_struct.tendon_stretch = zeros(n_elem,1);
            joint_state_struct.mtu_force = zeros(n_elem,1);
            joint_state_struct.torque = zeros(n_elem,1);
        end
        
    end % end methods (static)
end % end class
    
    


