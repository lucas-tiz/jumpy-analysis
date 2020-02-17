function joint_state = torque_musc(param, theta_init, theta, t_act, cont_prev)
    % compute joint torque created by pneumatic actuator & cam
    % (cam & tendon parameters, initial joint angle, current joint angle,
    % time from actuation start)

    %{ 
    TODO:
    - update empirical/analytical pressure response curve (transient pressure)
    - update length-pressure-force calibration
    %}
    
    len_t = length(t_act);
    muscle_pressure = zeros(len_t, 1);
    linear_displace = zeros(len_t, 1);
    cam_radius = zeros(len_t, 1);
    muscle_contract = zeros(len_t, 1);
    tendon_stretch = zeros(len_t, 1);
    mtu_force = zeros(len_t, 1);
    torque = zeros(len_t, 1);
    
    for i = 1:length(t_act)
    
        muscle_pressure(i) = pres_response(t_act(i)); % get pressure from transient pressure response data (step response)

        [linear_displace(i), cam_radius(i)] = cam_displacement(theta_init, theta(i), param); % calculate displacement & cam radius (moment arm)

        [muscle_contract(i), tendon_stretch(i), mtu_force(i)] = ...
            mtu_state(muscle_pressure(i), linear_displace(i), param, cont_prev); % calculate contraction force & muscle/tendon lengths

        torque(i) = mtu_force(i)*cam_radius(i)/100; % calculate joint torque (Nm)
        
        cont_prev = muscle_contract(i);
        
    end

    joint_state.muscle_pressure = muscle_pressure;
    joint_state.linear_displace = linear_displace;
    joint_state.cam_radius = cam_radius;
    joint_state.muscle_contract = muscle_contract;
    joint_state.tendon_stretch = tendon_stretch;
    joint_state.mtu_force = mtu_force;
    joint_state.torque = torque;
    
end


%% Functions
function r = cam_radius(theta, param)
    % calculate cam radius for given joint angle
    r = param.cam.m*theta + param.cam.r0;
end


function [ds,r_cur] = cam_displacement(theta_init, theta, param)
    % calculate tangent displacement due to cam: % ds = r_avg * delta_theta
    if (theta-theta_init == 0)
        theta_vec = [theta, theta];
    else
        sign_ang = sign(theta-theta_init); % sign of angle movement
        dth = % delta
        theta_vec = theta_init:(sign_ang*min(0.001, abs(theta-theta_init))):theta;
    end
    
    ds = 0;
    for i = 2:length(theta_vec)
        r_prev = cam_radius(theta_vec(i-1), param);
        r_cur = cam_radius(theta_vec(i), param);
        
        ds = ds + ((r_cur+r_prev)/2)*abs(theta_vec(i)-theta_vec(i-1)); 
    end
end


function p = pres_response(t)
    % estimate transient pressure response during muscle inflation
    persistent curvefit
    if isempty(curvefit) % create curve fit
        folder_pres_response = ['C:\Users\Lucas\Dropbox (GaTech)\Research\',...
            'Hexapod\analysis\muscle force analysis'];
        file_pres_response = 'cleanCut1s8_mcu_1_2019-08-12_18-21-44.csv';
        t_start = 5.1;
        t_end = 5.44;

        fullfile(folder_pres_response, file_pres_response)
        data = csvread(fullfile(folder_pres_response, file_pres_response),1); 
        t_vec = data(:,1); % (s)
        pres_vec = data(:,3)*6.89476; % (kPa)

        idx = find(t_vec >= t_start & t_vec <= t_end);
        t_vec = t_vec(idx) - t_vec(idx(1));
        pres_vec = pres_vec(idx);

        curvefit = fit(t_vec,pres_vec,'linearinterp');
    end
    
    if t < 0
        p = 0;
    elseif t > curvefit.p.breaks(end) % if time is beyond transient
        p = curvefit.p.coefs(end,2); % set pressure to max
    else
        p = curvefit(t); % set pressure based on curve fit
    end
end


function [cont_musc, stretch_tendon, force] = mtu_state(pres, disp, param, cont_musc_prev_)
    % calculate muscle contraction & corresponding tendon displacement
    persistent cont_musc_prev
    if isempty(cont_musc_prev)
        cont_musc_prev = 0;
    end
    
    persistent sf
    if isempty(sf)
        fprintf('sf\n')
        folder_surf_fit = ['C:\Users\Lucas\Dropbox (GaTech)\Research\',...
            'Hexapod\analysis\muscle force analysis\force calibration data'];
        file_surf_fit = 'A1_surface_fit.mat';
        load(fullfile(folder_surf_fit, file_surf_fit), 'sf'); % load force-length-pressure surface fit
    end

    fun = @(cont) (sf(cont,pres) - param.k_tendon*max(0,(cont-disp)))^2;
    cont_musc = max(0,fminsearch(fun, cont_musc_prev));
    
%     A = [];
%     b = [];
%     Aeq = [];
%     beq = [];
%     lb = [0];
%     ub = [5];
%     nonlcon = [];
%     options = optimoptions('fmincon','Display','off');
%     cont_musc = fmincon(fun, cont_musc_prev, A, b, Aeq, beq, lb, ub, nonlcon, options);

    force = sf(cont_musc,pres);
    if force < 0 % if force is negative
        force = 0;
        stretch_tendon = 0; % tendon stretch is zero
    else
        stretch_tendon = force/param.k_tendon;
    end


%     cont_musc = max(0,disp);
%     force = sf(cont_musc,pres);
%     stretch_tendon = 0;

    cont_musc_prev = cont_musc;

end
















