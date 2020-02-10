classdef MuscleJoint < matlab.mixin.Copyable %handle
    % revolute joint actuated by pneumatic muscle
    properties        
        cam_rad0 % initial cam radius
        cam_slope % cam profile slope
        cam_theta_range % joint angle (theta) range corresponding to cam angle (beta) range
        k_tendon % tendon stiffness
        
        cam_param % cam parameters for calculating cam_map data
        cam_map % moment arm and linear displacement data
        
        t1 = 0;
        t2 = 0;
        t3 = 0;
    end
    
    properties (Access = private)
        user % NOTE: change this for use on different computers

        pressure_curvefit % time-pressure curvefit (transient response)
        step_resp_f_steady % steady-state force for pressure step response
        force_sf % pressure-length-force surface fit
    end
        
    
    %%
    methods
        function obj = MuscleJoint(cam_rad0, cam_slope, cam_theta_range,...
                k_tendon, cam_param, user)
            % set public properties
            obj.cam_rad0 = cam_rad0;
            obj.cam_slope = cam_slope;
            obj.cam_theta_range = cam_theta_range;
            obj.k_tendon = k_tendon;
            
            % set private properties
            obj.cam_param = cam_param;
            obj.cam_param.filename = obj.create_cam_filename(cam_param); % construct cam file name from parameters
            obj.user = user;
            
            % initialize data
            obj.cam_map = containers.Map; % initialize to map
            obj.load_cam_data(); % load cam data if exists
        end
        
        
        function load_cam_data(obj)
            % load cam data from .mat file if corresponding data exists
            if isfile(obj.cam_param.filename) % check data already exists
                load(obj.cam_param.filename, 'cam_map_saved', 'cam_param_saved')
                if ((cam_param_saved.d == obj.cam_param.d) &&... % check parameters are the same
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
                folder_pres_response = ['C:\Users\',obj.user,'\Dropbox (GaTech)\',...
                    'Research\Hexapod\testing\bandwidth characterization\',...
                    'clean_cut_1p25D_0p19inID_quickconnect_noslack.is_tens',...
                    '_RawData\instron data\bandwidth_clean_cut_1p25D_0p19',...
                    'inID_quickconnect_noslack.is_tens_RawData'];
                file_pres_response = 'Specimen_RawData_1.csv';

                [t_vec,f_vec,~,f_steady] = obj.get_step_data(fullfile(folder_pres_response,...
                    file_pres_response), [1 4], 0.1, 1.5);
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
            joint_angle_sign = sign(obj.cam_theta_range(2)-...
                obj.cam_theta_range(1));
        end
       
       
        function cam_angle = convert_joint_angle_to_cam_angle(obj, joint_angle)
            % convert joint angle to corresponding cam angle
            cam_angle = (joint_angle - obj.cam_theta_range(1))*obj.get_joint_angle_sign();
        end

        
        function [ema,r_tan,phi,psi] = calc_ema(obj, beta, cam_rad0, cam_slope)
            % calculate effective moment arm for cam/joint parameters
            persistent phi_sym
            if isempty(phi_sym)
                phi_sym = sym('phi_sym');
            end

            % parameters
            d = obj.cam_param.d; % (cm) link length
            r0 = cam_rad0; % (cm) cam radius at zero degrees
            m = cam_slope; % (cm/rad) cam profile radius slope
            phi_range = obj.cam_param.phi_range; % (rad) cam angle range for which cam geometry exists

            % if no cam profile exists, just return zeros
            if (r0 == 0) && (m == 0)
                ema = 0;
                r_tan = 0;
                phi = 0;
                psi = 0;
                return
            end
           
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

            % if no physically possible values found
            if isempty(ema)
                ema = 0;
                r_tan = 0;
                phi = 0;
                psi = 0;
            end
        end
            
        
        function update_cam_data(obj, sweep_vec)
            % calculate effective moment arm for all cam geometries
            % not previously saved; sweep_vec = [radius0, slope]
            tic
            fprintf('calculating cam data...\n')
            
            obj.load_cam_data(); % re-load cam data
            obj.joint_angle_sign = obj.get_joint_angle_sign(obj); % update angle sign

            s = size(sweep_vec);
            n_sweep = s(1);
            beta_vec = obj.cam_param.beta_vec;
            n_beta = length(beta_vec);
            tic
            for idx_sweep = 1:n_sweep % loop over sweep vector
                cam_rad0_tmp = sweep_vec(idx_sweep,1); % (cm) cam radius at zero degrees
                cam_slope_tmp = sweep_vec(idx_sweep,2); % (cm/rad) cam profile radius slope

                key = [num2str(cam_rad0_tmp),',',num2str(cam_slope_tmp)]; % map key
                if ~obj.cam_map.isKey(key) % if key doesn't already exist, calculate data

                    ema_vec = zeros(1,n_beta);
                    r_tan_vec = zeros(1,n_beta);
                    parfor idx_beta = 1:n_beta % loop over joint angles TODO: change back to parfor
                        beta = beta_vec(idx_beta);
                        [ema_vec(idx_beta),r_tan_vec(idx_beta),~,~] =... % calculate effective moment arm & tangent radius
                            obj.calc_ema(beta,cam_rad0_tmp,cam_slope_tmp);
                    end
                    
                    linear_displace_vec = zeros(1,n_beta);
                    for idx_beta = 2:n_beta
                        r_tan_prev = r_tan_vec(idx_beta-1);
                        r_tan_cur = r_tan_vec(idx_beta);
                        linear_displace_vec(idx_beta) = linear_displace_vec(idx_beta-1)... % calculate linear displacement 
                            + ((r_tan_cur+r_tan_prev)/2)*abs(beta_vec(idx_beta)-beta_vec(idx_beta-1)); 
                    end
            
                    obj.cam_map(key) = {ema_vec, linear_displace_vec}; % update temp cam map
                    
                    fprintf('    updated %i of %i cam profiles, %4.1f elapsed\n',...
                        idx_sweep, n_sweep, toc)
                end
            end
            fprintf('all cam profiles updated, %0.2f s elapsed\n\n', toc)

            cam_map_saved = obj.cam_map; % create cam map save var
            cam_param_saved = obj.cam_param; % create param save var
            save(obj.cam_param.filename, 'cam_map_saved', 'cam_param_saved') % save cam map & params
        end
        
        
        function ema = get_ema(obj, theta)
            % get effective moment arm at current joint angle
            % (interp/extrap from saved data)
            key = [num2str(obj.cam_rad0),',',num2str(obj.cam_slope)]; % map key
            
            if ~obj.cam_map.isKey(key) % if key doesn't already exist, calculate data
                fprintf('NOT KEY: %s\n', key)
                obj.update_cam_data([obj.cam_rad0, obj.cam_slope]);
            end

            cam_data = obj.cam_map(key); % get cam data
            ema_vec = cam_data{1}; % get effective moment arm data
            beta_vec = obj.cam_param.beta_vec; % get beta angle data
            
            beta = obj.convert_joint_angle_to_cam_angle(theta); % convert joint angle to cam angle

            ema = interp1(beta_vec, ema_vec, beta,... % calculate ema at current angle
                'linear', 'extrap');             
        end
        

        function disp = get_joint_displace(obj, theta0, theta_cur)
            % get joint displacement at current joint angle from initial angle
            % (interp/extrap from saved data)
            key = [num2str(obj.cam_rad0),',',num2str(obj.cam_slope)]; % map key
            cam_data = obj.cam_map(key);
            linear_displace_vec = cam_data{2};            
            beta_vec = obj.cam_param.beta_vec;
            
            beta0 = obj.convert_joint_angle_to_cam_angle(theta0); % convert joint angle to cam angle
            beta = obj.convert_joint_angle_to_cam_angle(theta_cur); % convert joint angle to cam angle
                                        
            disp0 = interp1(beta_vec, linear_displace_vec, beta0,... % calculate displacement from initial angle to sim starting angle
                'linear', 'extrap'); 
            disp_cur = interp1(beta_vec, linear_displace_vec,... % calculate displacement from initial angle to current angle
                beta, 'linear', 'extrap');
            disp = disp0-disp_cur; % switch sign so that positive joint disp corresponds to negative MTU disp
        end
        
        
        %% MTU stuff
        function [cont_musc, stretch_tendon, force] = calc_mtu_state(obj, pres, disp)
            % calculate muscle contraction & corresponding tendon displacement
            persistent psf

            if isempty(obj.force_sf) % load force-length-pressure surface fit
                folder_surf_fit = ['C:\Users\',obj.user,'\Dropbox (GaTech)\',...
                    'Research\Hexapod\testing\force characterization\',...
                    'clean_cut_1p25D_crimp_over_tube_sleeve'];
                file_surf_fit = ['surface_fit_clean_cut_1p25D_crimp_over_',...
                    'tube_sleeve_345kpa.mat'];
                sf = load(fullfile(folder_surf_fit, file_surf_fit), 'sf'); 
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
                 (psf(4) + psf(5)*pres + psf(6)*pres^2 - obj.k_tendon),... % cont
                 (psf(7) + psf(8)*pres + psf(9)*pres^2 + psf(10)*pres^3 - obj.k_tendon*disp)]; % constant 
            r = roots(p);
            cont_musc = r(3);
       
%             if pres == 0 % if no pressure, no contraction
%                 cont_musc = 0;
%             end
            
            if cont_musc <= 0 % if zero / negative contraction
                %TODO: rethink infinitely stiff/no tendon 
                if obj.k_tendon >= 1e5 % if tendon is approximately infinitely stiff                    
                    force = max(0,obj.force_sf(0,pres)) + 1e4*(-cont_musc); % make muscle very stiff in tension (1e2 N/cm)
                    stretch_tendon = 0;
                else
                    cont_musc = 0; % set contraction to zero (assume no stretch)
                    stretch_tendon = max(0,disp); % disregard negative displacement
                    force = obj.k_tendon*stretch_tendon;
                end
            elseif cont_musc >= (6.5 + pres/300) % if beyond range of surf fit in contraction direction
                cont_musc = 6.5 + pres/300;
                force = 0;
                stretch_tendon = 0;
            else % if positive contraction in surface fit range
                force = max(0,obj.force_sf(cont_musc,pres)); % disregard negative forces from fit
                stretch_tendon = force/obj.k_tendon;
            end           
        end
        
        
        %% main method
        function joint_state_struct = torque_musc(obj, theta0, theta, t_act, p_max)
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
                
                ema(i) = obj.get_ema(theta(i)); % calculate effective moment arm
                
                linear_displace(i) = obj.get_joint_displace(theta0, theta(i)); % calculate displacement

                [muscle_contract(i), tendon_stretch(i), mtu_force(i)] = ...
                    obj.calc_mtu_state(muscle_pressure(i), linear_displace(i)); % calculate contraction force & muscle/tendon lengths

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
        
        
        %% getters
        function cam_param = get_cam_param(obj)
            cam_param = obj.cam_param;
        end
        
        
        function cam_map = get_cam_map(obj)
            cam_map = obj.cam_map;
        end
        
        
        function force_sf = get_force_sf(obj)
            force_sf = obj.force_sf;
        end

        
    end % end methods
      
    
    %%
    methods (Static)
        function filename_cam_data = create_cam_filename(cam_param)
            % create file name for cam data from cam parameters
            filename_cam_data = ['cam_data\', 'cam_data_',... % cam data file name
                num2str(cam_param.d), 'd_',...
                num2str(rad2deg(cam_param.phi_range(1))),'to',...
                num2str(rad2deg(cam_param.phi_range(2))),'phi_',...
                num2str(rad2deg(cam_param.beta_vec(1))),'to',...
                num2str(rad2deg(cam_param.beta_vec(end))),'beta',...
                '.mat'];
        end
        

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
    
    


