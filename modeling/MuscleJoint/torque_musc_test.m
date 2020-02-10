%
clear, clc
%{ 
TODO:
- update empirical/analytical pressure response curve (transient pressure)
- update length-pressure-force calibration

cam equations:
x_vec = (m*th_vec + r_init).*cos(th_vec);
y_vec = (m*th_vec + r_init).*sin(th_vec);
%}
    
tic

% design parameters
cam_rad0 = 1; % (cm) cam radius at zero degrees
cam_slope = 4;%-1.5; % -0.5 (cm/rad) cam profile radius slope
cam_theta_range = deg2rad([180 0]); % joint angle (theta) range corresponding to cam angle (beta)
k_tendon = 250; % (N cm) tendon stiffness

cam_param.d = 55; % (cm) link length
cam_param.phi_range = deg2rad([0 180]); % cam parameterization angle range
cam_param.beta_vec = deg2rad(-20:10:200); % joint angle (cam) vector for cam profile calculations

% parameter sweep
rad_knee_vec = -1:1:1; %-2.5:0.25:-0.5;%-3:0.5:1;%0:0.5:3;
rad_hip_vec = nan; %rad_knee_vec;
slope_knee_vec = 4:5; %4:0.25:6.5;%2:0.5:8;%-2:0.5:5;
slope_hip_vec = nan; % slope_knee_vec;5

sweep_vec_knee = combvec(rad_knee_vec, slope_knee_vec)';
sweep_vec_hip = combvec(rad_hip_vec, slope_hip_vec)';
sweep_vec = [sweep_vec_knee; sweep_vec_hip(~isnan(sweep_vec_hip))];
sweep_vec = unique(sweep_vec,'rows');
s = size(sweep_vec);
n = s(1);





% instantiate joint
knee_joint = MuscleJoint(cam_rad0, cam_slope, cam_theta_range, k_tendon, cam_param);
toc
knee_joint.cam_theta_range = deg2rad([0 180]);
% knee_joint.update_cam_data(sweep_vec);

% hip_joint = copy(knee_joint);
% hip_joint.cam_theta_range = deg2rad([0 180]);
% hip_joint.update_cam_data(sweep_vec);
toc

% knee_joint.cam_slope = 6;
% cam_param.theta_range = deg2rad([0 180]);
% hip_joint = MuscleJoint(cam_param, k_tendon);


% knee_joint = MuscleJoint(cam_angle_slope, cam_rad0, deg2rad(180), deg2rad(0), k_tendon);
% hip_joint = MuscleJoint(cam_angle_slope, cam_rad0, deg2rad(0), deg2rad(180), k_tendon);

% knee_joint.cam_radius(deg2rad(150));
% hip_joint.cam_radius(deg2rad(180));


% update cam geometry
% knee_joint.cam_angle_start = deg2rad(90);
% knee_joint.cam_update()
% knee_joint.cam_displacement(deg2rad(180), deg2rad(0));

% change tendon
% [cont_musc, stretch_tendon, force] = knee_joint.mtu_state(200,1);
% knee_joint.k_tendon = 100;
% [cont_musc, stretch_tendon, force] = knee_joint.mtu_state(100,0)

joint_state = knee_joint.torque_musc(deg2rad(180), deg2rad(150), 0.2, 0);
joint_state



% sim parameters
theta0 = deg2rad(0); % starting joint angle
contract0 = 0; % starting muscle contraction length
p_max = 300;


%% testing
t_vec = -0.2:0.01:1;
th_vec = zeros(length(t_vec),1);
th_vec(1) = theta0;
pres_vec = zeros(length(t_vec),1);
disp_vec = zeros(length(t_vec),1); % displacement from joint rotation (not contraction)
% rad_vec = zeros(length(t_vec),1);
ema_vec = zeros(length(t_vec),1);
ema_vec(1) = knee_joint.get_ema(theta0);
% rad_vec(1) = knee_joint.cam_radius(theta0);
cont_vec = zeros(length(t_vec),1);
stretch_vec = zeros(length(t_vec),1);
force_vec = zeros(length(t_vec),1);
torque_vec = zeros(length(t_vec),1);

cont_prev = contract0;

tic;
for i = 2:length(t_vec) %TODO: this would be ode45 loop
    
%     th_vec(i) = th_vec(i-1) + exp(-10 + i*0.075); %TODO: dynamics iteration -> new joint angle
    th_vec(i) = th_vec(i-1) + 0.02;

    joint_state = knee_joint.torque_musc(th_vec(1), th_vec(i), t_vec(i), p_max);
    
    pres_vec(i) = joint_state.muscle_pressure;
    disp_vec(i) = joint_state.linear_displace;
%     rad_vec(i) = joint_state.cam_radius;
    ema_vec(i) = joint_state.ema;
    stretch_vec(i) = joint_state.tendon_stretch;
    cont_vec(i) = joint_state.muscle_contract;
    force_vec(i) = joint_state.mtu_force;
    torque_vec(i) = joint_state.torque;
    
end
toc    
% 



% %% loop (this will be ODE solver)
% t_vec = 0:0.01:1;
% th_vec = zeros(length(t_vec),1);
% th_vec(1) = theta0;
% pres_vec = zeros(length(t_vec),1);
% disp_vec = zeros(length(t_vec),1); % displacement from joint rotation (not contraction)
% rad_vec = zeros(length(t_vec),1);
% rad_vec(1) = cam_radius(theta0, param);
% cont_vec = zeros(length(t_vec),1);
% stretch_tendon_vec = zeros(length(t_vec),1);
% force_vec = zeros(length(t_vec),1);
% torque_vec = zeros(length(t_vec),1);
% 
% tic
% for i = 2:length(t_vec) %TODO: this would be ode45 loop
%     
%     th_vec(i) = th_vec(i-1) + 0.01; %+ exp(-10 + i*0.075); %TODO: dynamics iteration -> new joint angle
% 
%     pres_vec(i) = pres_response(t_vec(i)); % get pressure from pressure response data
%     
%     [disp_vec(i), rad_vec(i)] = cam_displacement(theta0, th_vec(i), param); % get displacement & cam radius (moment arm)
%     
%     [cont_vec(i), stretch_tendon_vec(i), force_vec(i)] = ...
%         mtu_state(pres_vec(i), disp_vec(i), param, cont_vec(i-1));
% 
%     torque_vec(i) = force_vec(i)*rad_vec(i)/100; % calculate torque (Nm)
% end
% toc    


%%
f11 = figure(11); clf
hold on
grid on
% plot(th_vec, rad_vec)
plot(t_vec, disp_vec)
xlabel('Time (s)')
ylabel('Displacement (cm)')
title('Joint Displacement')


f12 = figure(12); clf
pax = polaraxes;
hold on
polarplot(th_vec, ema_vec)
polarplot(th_vec(1), ema_vec(1), '.g')
polarplot(th_vec(end), ema_vec(end), '.r')
title('Moment arm vs. Angle')


f14 = figure(14); clf
subplot(4,1,1)
hold on
grid on
plot(t_vec, disp_vec)
plot(t_vec, -cont_vec)
plot(t_vec, stretch_vec)
plot(t_vec, -cont_vec+stretch_vec, ':k', 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('Length (cm)')
legend('Joint Dislplacement', 'Muscle Contraction', 'Tendon Elongation', 'Diff cont+elong')

subplot(4,1,2)
hold on
grid on
plot(t_vec, pres_vec)
xlabel('Time (s)')
ylabel('Pressure (kPa)')

subplot(4,1,3)
hold on
grid on
plot(t_vec, force_vec)
xlabel('Time (s)')
ylabel('Force (N)')

subplot(4,1,4)
hold on
grid on
plot(t_vec, torque_vec)
xlabel('Time (s)')
ylabel('Torque (Nm)')


%% Functions
% function r = cam_radius(theta, param)
%     % calculate cam radius for given joint angle
%     r = param.cam.m*theta + param.cam.r0;
% end
% 
% 
% function [ds,r_cur] = cam_displacement(theta_init, theta, param)
%     % calculate tangent displacement due to cam: % ds = r_avg * delta_theta
%     theta_vec = theta_init:0.001:theta;
%     ds = 0;
%     for i = 2:length(theta_vec)
%         r_prev = cam_radius(theta_vec(i-1), param);
%         r_cur = cam_radius(theta_vec(i), param);
%         
%         ds = ds + ((abs(r_cur)+abs(r_prev))/2)*abs(theta_vec(i)-theta_vec(i-1)); 
%     end
% end
% 
% 
% function p = pres_response(t)
%     % estimate transient pressure response during muscle inflation
%     persistent curvefit
%     if isempty(curvefit) % create curve fit
%         file_pres_response = 'cleanCut1s8_mcu_1_2019-08-12_18-21-44.csv';
%         t_start = 5.0;
%         t_end = 5.44;
% 
%         data = csvread(file_pres_response,1); 
%         t_vec = data(:,1); % (s)
%         pres_vec = data(:,3)*6.89476; % (kPa)
% 
%         idx = find(t_vec >= t_start & t_vec <= t_end);
%         t_vec = t_vec(idx) - t_vec(idx(1));
%         pres_vec = pres_vec(idx);
%         
%         curvefit = fit(t_vec,pres_vec,'linearinterp');
%     end
%     
%     if t > curvefit.p.breaks(end)
%         p = curvefit.p.coefs(end,2);
%     else
%         p = curvefit(t);
%     end
% end
% 
% 
% function [cont_musc, stretch_tendon, force] = mtu_state(pres, disp, param, cont_musc_prev)
%     % calculate muscle contraction & corresponding tendon displacement
%     persistent sf
%     if isempty(sf)
%         load('force calibration data\A1_surface_fit.mat', 'sf'); % load force-length-pressure surface fit
%     end
%     
%     fun = @(cont) (sf(cont,pres) - param.k_tendon*(cont-disp))^2;
%     
%     cont_musc = max(0,fminsearch(fun, cont_musc_prev));
% 
% %     A = [];
% %     b = [];
% %     Aeq = [];
% %     beq = [];
% %     lb = [0];
% %     ub = [5];
% %     nonlcon = [];
% %     options = optimoptions('fmincon','Display','off');
% %     cont_musc = fmincon(fun, cont_musc_prev, A, b, Aeq, beq, lb, ub, nonlcon, options);
% 
%     force = sf(cont_musc,pres);
%     if force < 0 % if force is negative
%         force = 0;
%         stretch_tendon = 0; % tendon stretch is zero
%     else
%         stretch_tendon = force/param.k_tendon;
%     end
% end













