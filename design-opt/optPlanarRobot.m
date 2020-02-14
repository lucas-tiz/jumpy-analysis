%
clear, clc
% user = 'lucas';

repo_folder = 'jumpy-analysis';
current_path = pwd;
idx_repo = strfind(pwd, repo_folder);
repo_path = current_path(1:(idx_repo-1 + length(repo_folder)));

addpath(fullfile(repo_path, 'modeling'));
addpath(fullfile(repo_path, 'modeling', 'MuscleJoint'));
% addpath(genpath( 'C:\Program Files\MATLAB\spatial_v2' )); %TODO: make this relative?

global t_comp0
t_comp0 = now;

%% Config...
% import config
% pass config to build robot function
% keep config and robot separate objects
% edit functions that use config & robot structure fields
% remove 'planar' from function names?
% update YAMLmatlab on PC

% config_file = fullfile(repo_path,'modeling','robot_config.yaml');
% config = ReadYaml(config_file);


%% Parameters
% cam parameters (used for all joints)
cam_param.d = 55; % (cm) link length
cam_param.phi_range = deg2rad([0 180]); % cam parameterization angle range
cam_param.beta_vec = deg2rad(-20:10:200); % joint angle (cam) vector for cam profile calculations

% knee joint parameters
knee.rad0 = 5.0; % (cm) cam radius at zero degrees
knee.slope = 0; % (cm/rad) cam profile radius slope
knee.theta_range = deg2rad([180 0]); % joint angle (theta) range corresponding to cam angle (beta)
knee.k_tendon = 500; % (N cm) tendon stiffness

% hip joint parameters
hip.rad0 = 5.0; 
hip.slope = 0;
hip.theta_range = deg2rad([0 180]);
hip.k_tendon = 500;

% optimization parameter sets
% global opt_param
% global opt_param_discrete
opt_param_discrete = {'rad_hip', 'rad_knee', 'slope_hip', 'slope_knee'};

% opt_param.rad_knee = -3:0.1:5; %0:1:6; % (cm)
% opt_param.rad_hip = -3:0.1:5; %0:1:6; % (cm)
% opt_param.slope_knee = -1:0.1:4; %-4:1:4; 
% opt_param.slope_hip = -1:0.1:4; %-4:1:4;
% opt_param.k_tendon_knee = 1e6; %linspace(10, 2000, 10); % (N cm)
% opt_param.k_tendon_hip = 1e6; %linspace(10, 2000, 10); % (N cm)
% opt_param.t_knee = 0.0:0.1:0.5;
% opt_param.t_hip = 0.0:0.1:0.5;
% opt_type = 3;

opt_param.rad_knee = 2; %3.40;
opt_param.rad_hip = 2; %4.80;
opt_param.slope_knee = 0; %-0.90;
opt_param.slope_hip = 0; %-0.60;
opt_param.k_tendon_knee = 50; %61.65;
opt_param.k_tendon_hip = 50; %37.53;
opt_param.t_knee = 0; %0.27;
opt_param.t_hip = 0; %0.29;
opt_type = 1;

global opt_fieldnames
opt_fieldnames = sort(fieldnames(opt_param));

% simulation parameters
sim_param.t_sim = 1; % simulation time
sim_param.dt = 1e-3; % delta t
sim_param.liftoff_stop = 1; % stop simulation at liftoff 

% initial config.
hexapod.wb = 0.55; % (m) hexapod base width (food width apart) TODO: needed?
% y0 = deg2rad([20; 20; 140]); % [theta3, theta4, theta5] [60, 60, 60]
% y0 = deg2rad([-30 -30 240]);

% control parameters [knee_r, hip_r, hip_l, knee_l]
hexapod.p_max = 241.3; % (kPa)
hexapod.t_musc_activate = [0.0, 0.0, 0.0, 0.0]; % muscle activation timings (set via sweep)
% hexapod.joint_damp = [1.5, 2.0, 2.0, 1.5]*0.4;
hexapod.joint_damp = [0.0, 0.0, 0.0, 0.0];
hexapod.joint_damp_lims = deg2rad([0, 90, 90, 0]);
% hexapod.joint_stiff = [1, 3, 3, 1]*5; % (Nm/rad)
hexapod.joint_stiff = [0, 0, 0, 0]; % (Nm/rad)
hexapod.joint_stiff_lims = deg2rad([10,  80,  80,  10]);
% hexapod.joint_stiff_lims = deg2rad([10,  70,  70,  10;
hexapod.mu = 0.999; % 0.9

% animation
animation = 1;
anim_delay = 0.00;
vid_export = 0;       % animation export on
vid_path = 'C:\Users\Lucas\Dropbox (GaTech)\Research\Hexapod\analysis\anim'; %TODO: relative path?
vid_file = 'anim_export_test'; % animation video file name

% save info
global f_save
global save_data
f_save = 'test';
save_data = 0; %TODO


%% Build hexapod parallel model 
hexapod = buildPlanarRobot(hexapod); % build planar hexapod robot

%TODO: how to determine initial pose
% x0 = [0, 0, deg2rad(-70), deg2rad(140), deg2rad(20), deg2rad(20), deg2rad(140)...
%       0, 0, deg2rad(0),   deg2rad(0),   deg2rad(0),  deg2rad(0),  deg2rad(0)];
x0 = [0, 0, deg2rad(-45), deg2rad(90), deg2rad(45), deg2rad(45), deg2rad(90)...
      0, 0, deg2rad(0),   deg2rad(0),   deg2rad(0),  deg2rad(0),  deg2rad(0)];  
% x0 = [0, 0, deg2rad(-5), deg2rad(10), deg2rad(85), deg2rad(85), deg2rad(10)...
%       0, 0, deg2rad(0),   deg2rad(0),   deg2rad(0),  deg2rad(0),  deg2rad(0)];
% x0 = [0, 0, deg2rad(-30), deg2rad(140), deg2rad(-30), deg2rad(-30), deg2rad(140)...
%       0, 0, deg2rad(0),   deg2rad(0),   deg2rad(0),  deg2rad(0),  deg2rad(0)];
hexapod.x0 = x0; % add initial state to hexapod object
animPlanarRobot(hexapod,0,x0,sim_param.dt,anim_delay,1); % display model in initial state

% fprintf('initial joint angles (deg): %0.2f %0.2f %0.2f %0.2f %0.2f\n\n', rad2deg(q0))


%% Create & initialize joint objects
knee_joint = MuscleJoint(knee.rad0, knee.slope, knee.theta_range,...
    knee.k_tendon, cam_param); %, user);
hip_joint = MuscleJoint(hip.rad0, hip.slope, hip.theta_range,...
    hip.k_tendon, cam_param); %, user);

% calculate cam data before simulations (should only have to do for knee
% since knee & hip share cam_param file)
sweep_arr_knee = combvec(opt_param.rad_knee, opt_param.slope_knee)';
sweep_arr_hip = combvec(opt_param.rad_hip, opt_param.slope_hip)';
sweep_arr_joint = [sweep_arr_knee; sweep_arr_hip(~isnan(sweep_arr_hip(:,1)) & ~isnan(sweep_arr_hip(:,2)),:)];
sweep_arr_joint = unique(sweep_arr_joint,'rows'); % sweep vector of just knee/hip cam radius & slope
knee_joint.update_cam_data(sweep_arr_joint);
hip_joint.update_cam_data(sweep_arr_joint);

% add joint objects
hexapod.joint_objs = {copy(knee_joint), copy(hip_joint),...
    copy(hip_joint), copy(knee_joint)}; % (k_r, h_r, h_l, k_l)


%% Optimize
% grid search
if opt_type == 1
    
    fn = sort(fieldnames(opt_param));
    combvec_cell = cell(numel(fn),1);
    for idx_fn = 1:numel(fn)
        combvec_cell{idx_fn} = opt_param.(fn{idx_fn});
    end
    sweep_arr = combvec(combvec_cell{:})';
    
    % global sweep_vec
%     sweep_vec = combvec(...
%         opt_param.rad_knee_vec,...
%         opt_param.rad_hip_vec,...
%         opt_param.slope_knee_vec,...
%         opt_param.slope_hip_vec,...
%         opt_param.k_tendon_knee_vec,...
%         opt_param.k_tendon_hip_vec,...
%         opt_param.t_knee_vec,...
%         opt_param.t_hip_vec)';    
    s = size(sweep_arr);
    global n
    n = s(1);
    fprintf('n: %i, estimated run time: %0.1f min\n\n', n, n*sim_param.t_sim*2.1/60)

    global v_vert_jump_max;
    v_vert_jump_max = 0;
    opt_res_arr = zeros(n,1);

    global p
    global h
    p = 0;
    D = parallel.pool.DataQueue;
    h = waitbar(0, 'Running...');
    afterEach(D, @nUpdateWaitbar);

    tic
    for i = 1:n %DEBUG
    % parfor (i = 1:n, 32) 
        hex_i = hexapod; % create copy of hexapod
        hex_i.joint_objs = {copy(knee_joint), copy(hip_joint),... % add joint objects
            copy(hip_joint), copy(knee_joint)}; % (k_r, h_r, h_l, k_l)
        hex_i = update_hexapod_params(hex_i, sweep_arr(i,:), opt_param); % update parameters

        [t_vec, tau_arr, x_arr, f_arr, j_state_arr, v_com_arr, info_aerial]... 
            = simRobot(hex_i, sim_param);

        opt_res_arr(i) = info_aerial.v(1);
        send(D, [i, info_aerial.t, info_aerial.v(1), sweep_arr(i,:)]);
    end
    close(h);
    [vy_jump_max_confirm, idx_jump_max] = max(opt_res_arr);

    fprintf('\ntotal elapsed time: %s h:m:s\n', datestr(now-t_comp0,13))    
    fprintf('optimal velocity: %0.3f m/s\n', vy_jump_max_confirm)
    fprintf('optimal params:\n')    
    for idx_fn = 1:numel(fn)
        fprintf('    %s:  %0.2f\n', fn{idx_fn}, sweep_arr(idx_jump_max,idx_fn));
    end
        
%     fprintf('    knee delay:  %0.2f\n', sweep_arr(idx_jump_max,1))
%     fprintf('    knee delay:  %0.2f\n', sweep_arr(idx_jump_max,1))
%     fprintf('    tendon knee: %0.2f\n', sweep_arr(idx_jump_max,2))
%     fprintf('    tendon hip:  %0.2f\n', sweep_arr(idx_jump_max,3))
%     fprintf('    radius knee: %0.2f\n', sweep_arr(idx_jump_max,4))
%     fprintf('    radius hip:  %0.2f\n', sweep_arr(idx_jump_max,5))
%     fprintf('    slope knee:  %0.2f\n', sweep_arr(idx_jump_max,6))
%     fprintf('    slope hip:   %0.2f\n\n', sweep_arr(idx_jump_max,7))
    
    param_vec_best = sweep_arr(idx_jump_max,:);
end

% fminsearch
% if opt_type == 2
%     knee_delay0 = 0.0; %(s)
%     k_tendon_knee0 = 500; % (N cm)
%     k_tendon_hip0 = 500; % (N cm)
%     rad_knee0 = 2; % (cm)
%     rad_hip0 = 2; % (cm)
%     slope_knee0 = 0; 
%     slope_hip0 = 0;
% 
%     par_vec0 = [knee_delay0, k_tendon_knee0, k_tendon_hip0, rad_knee0, rad_hip0,...
%                 slope_knee0, slope_hip0];
%             
%     figure(1111); clf
%     hold on
%     grid on
%     ylim([0 7])
% 
%     objFun = @(par_vec)negJumpVel(par_vec, hexapod, knee_joint, hip_joint, sim_param);
% 
%     options = optimset('Display', 'iter', 'OutputFcn', @outputFcn, 'MaxIter', 100);
%     
%     [par_vec, vy_neg] = fminsearch(objFun, par_vec0, options);
% end

% genetic algorithm
if opt_type == 3
    % variable bounds: [knee_delay, k_tendon_knee, k_tendon_hip, rad_knee,
    % rad_hip, slope_knee, slope_hip]
    
    fn = sort(fieldnames(opt_param));
    n_fn = numel(fn);
    lb = zeros(n_fn,1);
    ub = zeros(n_fn,1);
    idx_discrete = [];
    for idx_fn = 1:n_fn
        if any(strcmp(fn{idx_fn}, opt_param_discrete)) % if discretized param
            lb(idx_fn) = 1;
            ub(idx_fn) = length(opt_param.(fn{idx_fn}));
            idx_discrete = [idx_discrete, idx_fn];
        else % if continuous param
            lb(idx_fn) = min(opt_param.(fn{idx_fn}));
            ub(idx_fn) = max(opt_param.(fn{idx_fn}));
        end
    end
    
    
%     lb = [min(opt_param.knee_delay),...
%           min(opt_param.k_tendon_knee),...
%           min(opt_param.k_tendon_hip),...
%           1,...
%           1,...
%           1,...
%           1];
%     ub = [max(opt_param.knee_delay),...
%           max(opt_param.k_tendon_knee),...
%           max(opt_param.k_tendon_hip),...
%           length(opt_param.rad_knee),...
%           length(opt_param.rad_hip),...
%           length(opt_param.slope_knee),...
%           length(opt_param.slope_hip)];
    
    opts = optimoptions(@ga, ...
                        'PopulationSize', 2500, ... %2500
                        'MaxGenerations', 40, ... %40
                        'EliteCount', 0.05*2500, ... %0.05*2500
                        'FunctionTolerance', 1e-8, ...
                        'PlotFcn', @gaplotbestf, ...
                        'OutputFcn', @(options,state,flag)gaOutputFcn(options,state,flag,opt_param), ...
                        'UseParallel', true);
    
    objFun = @(param_vec)gaObjFcn(param_vec, hexapod, knee_joint, hip_joint,...
        sim_param, opt_param);
                                        
%     rng(0, 'twister');
    rng('shuffle');
    [xbest, fbest, exitflag] = ga(objFun, n_fn, [], [], [], [], ...
        lb, ub, [], idx_discrete, opts);
    
    param_vec_best = camMapVars(xbest, opt_param);
end


%% Data
% simulate optimal config
fprintf('\nsimulating optimal config\n')
sim_param.t_sim = 1;
sim_param.dt = 5e-4;
sim_param.liftoff_stop = 0;
hexapod = update_hexapod_params(hexapod, param_vec_best, opt_param);
[t_vec, tau_arr, x_arr, f_arr, j_state_arr, v_com_arr, info_aerial]... 
    = simRobot(hexapod, sim_param);

% calculate jump height
torso_pos = torsoPos(hexapod, x_arr);
vert_jump = torso_pos(:,1) - (hexapod.l(1) + hexapod.l(2)); % jump height trajectory
[h_jump, idx_jump] = max(vert_jump);
t_jump = t_vec(idx_jump);
fprintf('optimized jump height: %0.3f\n', h_jump)

% organize ground reaction forces
grf.r = [f_arr(:,1), f_arr(:,3)-f_arr(:,4)]; % right foot GRFs [normal (x-dir), tangential (y_dir)]
grf.l = [f_arr(:,2), f_arr(:,5)-f_arr(:,6)]; % left foot GRFs [normal (x-dir), tangential (y_dir)]

% % save optimal data
% if save_data == 1
%     save(['data_opt/optimal_data_', f_save, '.mat'], 't_vec', 'x_arr', 'torque_arr',...
%         'j_state_arr', 'react_arr', 'info_aerial', 'y_torso', 't_jump',...
%         'h_jump')
% end

%% Export data
% 
% thetadd = diff(x_arr(:,11:14),1,1)/sim_param.dt;
% thetadd = [thetadd; zeros(1,4)];
% 
% f123 = figure(123); clf
% hold on
% grid on
% plot(t_vec, thetadd(:,1))
% plot(t_vec, thetadd(:,2))
% plot(t_vec, thetadd(:,3))
% plot(t_vec, thetadd(:,4))
% 
% 
% % [time (s), 
% % pres1: right knee(kPa), pres2: right hip (kPa), pres3: left hip (kPa), pres4: left knee(kPa)
% % cont1: right knee (cm), cont2: right hip (cm), cont3: left hip(cm), cont4: left knee (cm)
% % elong1: right knee (cm), elong2: right hip (cm), elong3: left hip (cm), elong4: left knee (cm)
% % force1: right knee (N), force2: right hip (N), force3: left hip (N), force4: left knee (N)
% % tau2: right knee (Nm), tau3: right hip (Nm), tau4: left hip (Nm), tau5: left knee (Nm)
% % theta2: right knee (rad), theta3: right hip (rad), theta4: left hip (rad), theta5: left knee (rad)
% % theta2d: right knee (rad/s), theta3d: right hip (rad/s), theta4d: left hip (rad/s), theta5d: left knee (rad/s)
% % theta2dd: right knee (rad/s^2), theta3dd: right hip (rad/s^2), theta4dd: left hip (rad/s^2), theta5dd: left knee (rad/s^2)
% 
% header = ['time, ',...
%     'pres-kr, pres-hr, pres-hl, pres-kl, ',...
%     'cont-kr, cont-hr, cont-hl, cont-kl, ',... 
%     'elong-kr, elong-hr, elong-hl, elong-kl, ',...
%     'force-kr, force-hr, force-hl, force-kl, ',...
%     'tau-kr, tau-hr, tau-hl, tau-kl, ',...
%     'theta-kr, theta-hr, theta-hl, theta-kl ',...
%     'thetad-kr, thetad-hr, thetad-hl, thetad-kl ',...
%     'thetadd-kr, thetadd-hr, thetadd-hl, thetadd-kl'];
% 
% data_export = [t_vec',... % time
%     j_state_arr{1}.muscle_pressure, j_state_arr{2}.muscle_pressure, j_state_arr{3}.muscle_pressure, j_state_arr{4}.muscle_pressure,... % muscle pressures
%     j_state_arr{1}.muscle_contract, j_state_arr{2}.muscle_contract, j_state_arr{3}.muscle_contract, j_state_arr{4}.muscle_contract,... % muscle contraction lengths
%     j_state_arr{1}.tendon_stretch, j_state_arr{2}.tendon_stretch, j_state_arr{3}.tendon_stretch, j_state_arr{4}.tendon_stretch,... % tendon elongations
%     j_state_arr{1}.mtu_force, j_state_arr{2}.mtu_force, j_state_arr{3}.mtu_force, j_state_arr{4}.mtu_force,... % muscle forces
%     j_state_arr{1}.torque, j_state_arr{2}.torque, j_state_arr{3}.torque, j_state_arr{4}.torque,... % joint torques
%     x_arr(:,4), x_arr(:,5), x_arr(:,6), x_arr(:,7),... % joint angles
%     x_arr(:,11), x_arr(:,12), x_arr(:,13), x_arr(:,14),... % joint angular velocities
%     thetadd(:,1), thetadd(:,2), thetadd(:,3), thetadd(:,4)]; % joint angular accelerations
%     
% 
% 
% file_export = 'jump_data';
% 
% 
% file_export_path = fullfile(repo_path,'modeling','sim-data',[file_export, '.csv']);    
% fid = fopen(file_export_path, 'w'); 
% fprintf(fid,'%s\n',header);
% fclose(fid);
% dlmwrite(file_export_path, data_export, '-append');
    

    



%% Plots
co = get(0, 'DefaultAxesColorOrder');

% jump plot
f8 = figure(8); clf
subplot(2,1,1)
title('Jump Position (Torso)')
xlabel('Time (s)')
ylabel('Distance (m)')
hold on
grid on
plot(t_vec, vert_jump)
plot(t_vec, torso_pos(:,2))
plot(t_jump, h_jump, '.', 'MarkerSize', 12)
ylims = get(gca, 'YLim');
plot([info_aerial.t,info_aerial.t], [ylims(1), ylims(2)], '--k', 'HandleVisibility','off')
legend('vertical', 'horizontal')

subplot(2,1,2)
title('Jump Velocity (COM)')
xlabel('Time (s)')
ylabel('Velocity (m/s)')
hold on
grid on
plot(t_vec, v_com_arr(:,1))
plot(t_vec, v_com_arr(:,2))
legend('vertical', 'horizontal')

% joint plots
f2 = figure(3); clf
subplot(2,1,1)
title('Joint Angles')
xlabel('Time (s)')
ylabel('Angle (deg)')
hold on
grid on
plot(t_vec, rad2deg(x_arr(:,4)), 'Color', co(1,:))
plot(t_vec, rad2deg(x_arr(:,7)), ':', 'Color', co(1,:), 'LineWidth', 1.5)
plot(t_vec, rad2deg(x_arr(:,5)), 'Color', co(2,:))
plot(t_vec, rad2deg(x_arr(:,6)), ':', 'Color', co(2,:), 'LineWidth', 1.5)
legend('\Theta_2 (knee right)','\Theta_5 (knee left)',...
    '\Theta_3 (hip right)','\Theta_4 (hip left)', 'Location', 'Best')
ylims = get(gca, 'YLim');
% ylim([-360 360])
plot([info_aerial.t,info_aerial.t], [ylims(1), ylims(2)], '--k', 'HandleVisibility','off')

subplot(2,1,2)
title('Joint Angular Velocities')
xlabel('Time (s)')
ylabel('Angular Velocity (deg/s)')
hold on
grid on
plot(t_vec, rad2deg(x_arr(:,11)), 'Color', co(1,:))
plot(t_vec, rad2deg(x_arr(:,14)), ':', 'Color', co(1,:), 'LineWidth', 1.5)
plot(t_vec, rad2deg(x_arr(:,12)), 'Color', co(2,:))
plot(t_vec, rad2deg(x_arr(:,13)), ':', 'Color', co(2,:), 'LineWidth', 1.5)
legend('\Theta_2d (knee right)','\Theta_5d (knee left)',...
    '\Theta_3d (hip right)','\Theta_4d (hip left)', 'Location', 'Best')
ylims = get(gca, 'YLim');
% ylim([-360 360])
plot([info_aerial.t,info_aerial.t], [ylims(1), ylims(2)], '--k', 'HandleVisibility','off')

% joint state plots -------------------------------------------------------
f14 = figure(14); clf
subplot(4,2,1)
title('Knee Dynamics')
hold on
grid on
plot(t_vec, j_state_arr{1}.linear_displace, 'Color', co(1,:))
plot(t_vec, j_state_arr{4}.linear_displace, ':', 'Color', co(1,:), 'LineWidth', 1.5)
plot(t_vec, j_state_arr{1}.muscle_contract, 'Color', co(2,:))
plot(t_vec, j_state_arr{4}.muscle_contract, ':', 'Color', co(2,:), 'LineWidth', 1.5)
plot(t_vec, j_state_arr{1}.tendon_stretch, 'Color', co(3,:))
plot(t_vec, j_state_arr{4}.tendon_stretch, ':', 'Color', co(3,:), 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('Length (cm)')
legend('Joint Disp 1', 'Joint Disp 4', 'Muscle Contract 1', 'Muscle Contract 4',...
    'Tendon Stretch 1', 'Tendon Stretch 4', 'Location', 'NorthEast')
ylims = get(gca, 'YLim');
plot([info_aerial.t,info_aerial.t], [ylims(1), ylims(2)], '--k', 'HandleVisibility','off')

subplot(4,2,3)
hold on
grid on
plot(t_vec, j_state_arr{1}.muscle_pressure, 'Color', co(1,:))
plot(t_vec, j_state_arr{4}.muscle_pressure, ':', 'Color', co(1,:), 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('Pressure (kPa)')
legend('Pressure 1', 'Pressure 4')
ylims = get(gca, 'YLim');
plot([info_aerial.t,info_aerial.t], [ylims(1), ylims(2)], '--k', 'HandleVisibility','off')

subplot(4,2,5)
hold on
grid on
plot(t_vec, j_state_arr{1}.mtu_force, 'Color', co(1,:))
plot(t_vec, j_state_arr{4}.mtu_force, ':', 'Color', co(1,:), 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('Force (N)')
legend('Force 1', 'Force 4')
ylims = get(gca, 'YLim');
plot([info_aerial.t,info_aerial.t], [ylims(1), ylims(2)], '--k', 'HandleVisibility','off')

subplot(4,2,7)
hold on
grid on
plot(t_vec, j_state_arr{1}.torque, 'Color', co(1,:))
plot(t_vec, j_state_arr{4}.torque, ':', 'Color', co(1,:), 'LineWidth', 1.5)
plot(t_vec, tau_arr(:,4), 'Color', co(2,:))
plot(t_vec, tau_arr(:,7), ':', 'Color', co(2,:), 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('Torque (Nm)')
legend('muscle r', 'muscle l', 'total r', 'total l')
ylims = get(gca, 'YLim');
% ylim([0 20]);
plot([info_aerial.t,info_aerial.t], [ylims(1), ylims(2)], '--k', 'HandleVisibility','off')
%
subplot(4,2,2)
title('Hip Dynamics')
hold on
grid on
plot(t_vec, j_state_arr{2}.linear_displace, 'Color', co(1,:))
plot(t_vec, j_state_arr{3}.linear_displace, ':', 'Color', co(1,:), 'LineWidth', 1.5)
plot(t_vec, j_state_arr{2}.muscle_contract, 'Color', co(2,:))
plot(t_vec, j_state_arr{3}.muscle_contract, ':', 'Color', co(2,:), 'LineWidth', 1.5)
plot(t_vec, j_state_arr{2}.tendon_stretch, 'Color', co(3,:))
plot(t_vec, j_state_arr{3}.tendon_stretch, ':', 'Color', co(3,:), 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('Length (cm)')
legend('Joint Disp 1', 'Joint Disp 4', 'Muscle Contract 1', 'Muscle Contract 4',...
    'Tendon Stretch 1', 'Tendon Stretch 4', 'Location', 'NorthEast')
ylims = get(gca, 'YLim');
plot([info_aerial.t,info_aerial.t], [ylims(1), ylims(2)], '--k', 'HandleVisibility','off')

subplot(4,2,4)
hold on
grid on
plot(t_vec, j_state_arr{2}.muscle_pressure, 'Color', co(1,:))
plot(t_vec, j_state_arr{3}.muscle_pressure, ':', 'Color', co(1,:), 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('Pressure (kPa)')
legend('Pressure 1', 'Pressure 4')
ylims = get(gca, 'YLim');
plot([info_aerial.t,info_aerial.t], [ylims(1), ylims(2)], '--k', 'HandleVisibility','off')

subplot(4,2,6)
hold on
grid on
plot(t_vec, j_state_arr{2}.mtu_force, 'Color', co(1,:))
plot(t_vec, j_state_arr{3}.mtu_force, ':', 'Color', co(1,:), 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('Force (N)')
legend('Force 1', 'Force 4')
ylims = get(gca, 'YLim');
plot([info_aerial.t,info_aerial.t], [ylims(1), ylims(2)], '--k', 'HandleVisibility','off')

subplot(4,2,8)
hold on
grid on
plot(t_vec, j_state_arr{2}.torque, 'Color', co(1,:))
plot(t_vec, j_state_arr{3}.torque, ':', 'Color', co(1,:), 'LineWidth', 1.5)
plot(t_vec, tau_arr(:,5), 'Color', co(2,:))
plot(t_vec, tau_arr(:,6), ':', 'Color', co(2,:), 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('Torque (Nm)')
legend('muscle r', 'muscle l', 'total r', 'total l')
ylims = get(gca, 'YLim');
plot([info_aerial.t,info_aerial.t], [ylims(1), ylims(2)], '--k', 'HandleVisibility','off')


% reaction force plots ----------------------------------------------------
f4 = figure(4); clf
subplot(2,1,1)
title('Normal GRFs')
hold on
grid on
ylim([0 50])
h_r = plot(t_vec, grf.r(:,1), 'Color', co(1,:));
h_l = plot(t_vec, grf.l(:,1), ':', 'Color', co(2,:), 'LineWidth', 1.5);
ylims = get(gca, 'YLim');
plot([info_aerial.t,info_aerial.t], [ylims(1), ylims(2)], '--k')
legend([h_r, h_l], 'right foot', 'left foot')

subplot(2,1,2)
title('Tangential GRFs')
hold on
grid on
ylim([-50 50])
plot(t_vec, grf.r(:,2), 'Color', co(1,:))
plot(t_vec, grf.l(:,2), ':', 'Color', co(2,:), 'LineWidth', 1.5)
ylims = get(gca, 'YLim');
plot([info_aerial.t,info_aerial.t], [ylims(1), ylims(2)], '--k')
legend('right foot', 'left foot')


%% Animation
% showmotion(hexapod, t_vec, x_arr(:,1:hexapod.NB)') 

sim_param.dt = 1e-3;
if animation == 1
    if vid_export == 1
        vid = VideoWriter(fullfile(vid_path,vid_file),'MPEG-4');
        vid.Quality = 50;
        vid.FrameRate = 60;
        open(vid);
        
        animPlanarRobot(hexapod,t_vec,x_arr,sim_param.dt,anim_delay,5,vid);
        
        close(vid);
    else
        animPlanarRobot(hexapod,t_vec,x_arr,sim_param.dt,anim_delay,5);
    end
    
end


%%
function nUpdateWaitbar(sim_info)
    global opt_param
    global v_vert_jump_max
    global f_save
    global save_data
    global p
    global n
    global h
    
    waitbar(p/n, h);
    p = p + 1;
    fprintf('%10.0f %10.2f%%, %10.2f s\n', sim_info(1), p/n*100, toc) 
    
     if sim_info(3) > v_vert_jump_max
        idx_jump_max = sim_info(1);
        t_jump_max = sim_info(2);
        v_vert_jump_max = sim_info(3);
        fprintf('jump velocity: %0.3f, NEW MAX ----------\n', v_vert_jump_max)
        
        if save_data == 1
            sweep = sim_info(4:end);
            
            fid = fopen(['data/optimal_params_',f_save,'.txt'],'w');
            fprintf(fid, ['optimal index: %i\n',...
                'takeoff time: %0.3f s\n',...
                'takeoff velocity: %0.3f m/s\n'], idx_jump_max, t_jump_max, vy_jump_max);
                
            fn = sort(fieldnames(opt_param));
            for idx_fn = 1:numel(fn)
                fprintf(fid, '    %s:  %0.2f\n', fn{idx_fn}, sweep(idx_fn));
            end
                
                
%                 'knee delay:  %0.2f\n',...
%                 'tendon knee: %0.2f\n',...
%                 'tendon hip:  %0.2f\n',...
%                 'radius knee: %0.2f\n',...
%                 'radius hip:  %0.2f\n',...
%                 'slope knee:  %0.2f\n',...
%                 'slope hip:   %0.2f\n\n'],...
%                 idx_jump_max, t_jump_max, vy_jump_max,...
%                 sweep(1), sweep(2),...
%                 sweep(3), sweep(4),...
%                 sweep(5), sweep(6),...
%                 sweep(7));
                fclose(fid);
        end         
     end
end


function hexapod = update_hexapod_params(hexapod, sweep, opt_param)
    %
    fn = sort(fieldnames(opt_param));
    op = struct(); % initialize
    for idx_fn = 1:numel(fn)
        op.(fn{idx_fn}) = sweep(idx_fn);
    end

%     knee_delay = sweep(1);
%     k_tendon_knee = sweep(2);
%     k_tendon_hip = sweep(3);
%     rad_knee = sweep(4);
%     rad_hip = sweep(5);
%     slope_knee = sweep(6);
%     slope_hip = sweep(7);
    
    if isnan(op.k_tendon_hip)
        op.k_tendon_hip = op.k_tendon_knee;
    end
    if isnan(op.rad_hip)
        op.rad_hip = op.rad_knee;
    end
    if isnan(op.slope_hip)
        op.slope_hip = op.slope_knee;
    end

    % set muscle activation times
%     tma = [knee_delay, 0, 0, knee_delay];
%     hexapod.t_musc_activate = tma - min(tma);
%     hexapod.t_musc_activate = [0.0, 0.2, 0.32, 0.12];
    hexapod.t_musc_activate = [op.t_knee, op.t_hip, op.t_hip, op.t_knee];
    
    % set knee tendon stiffness
    hexapod.joint_objs{1}.k_tendon = op.k_tendon_knee;
    hexapod.joint_objs{4}.k_tendon = op.k_tendon_knee;
        
    % set hip tendon stiffness
    hexapod.joint_objs{2}.k_tendon = op.k_tendon_hip;
    hexapod.joint_objs{3}.k_tendon = op.k_tendon_hip;

    % set knee radius (starting radius)
    hexapod.joint_objs{1}.cam_rad0 = op.rad_knee;
    hexapod.joint_objs{4}.cam_rad0 = op.rad_knee;

    % set hip radius (starting radius)
    hexapod.joint_objs{2}.cam_rad0 = op.rad_hip;
    hexapod.joint_objs{3}.cam_rad0 = op.rad_hip;
    
    % set knee cam slope
    hexapod.joint_objs{1}.cam_slope = op.slope_knee;
    hexapod.joint_objs{4}.cam_slope = op.slope_knee;

    % set hip cam slope
    hexapod.joint_objs{2}.cam_slope = op.slope_hip;
    hexapod.joint_objs{3}.cam_slope = op.slope_hip;
end


% function vy_jump_neg = negJumpVel(par_vec, hex, knee_joint, hip_joint, sim_param)
%     % function to minimize: returns negative vertical velocity at liftoff
%     % par_vec = [knee_delay, k_tendon_knee, k_tendon_hip, rad_knee, rad_hip,
%     %            slope_knee, slope_hip];
% 
%     hex.joint_objs = {copy(knee_joint), copy(hip_joint),... % add joint objects
%         copy(hip_joint), copy(knee_joint)}; % (k_r, h_r, h_l, k_l)
%     hex = update_hexapod_params(hex, par_vec); % update parameters
%     
%     [~, ~, ~, ~, ~, info_aerial] = sim_2leg(hex, sim_param); % simulate
% 
%     vy_jump_neg = -info_aerial.vy;
% end


% function out = outputFcn(x, optimValues, state)
%     global t_comp0
% 
%     fprintf('\ntotal elapsed time: %s h:m:s\n', datestr(now-t_comp0,13))    
%     fprintf('velocity: %0.3f m/s\n', -optimValues.fval)
%     fprintf('optimal params:\n')
%     fprintf('    knee delay:  %0.2f\n', x(1))
%     fprintf('    tendon knee: %0.2f\n', x(2))
%     fprintf('    tendon hip:  %0.2f\n', x(3))
%     fprintf('    radius knee: %0.2f\n', x(4))
%     fprintf('    radius hip:  %0.2f\n', x(5))
%     fprintf('    slope knee:  %0.2f\n', x(6))
%     fprintf('    slope hip:   %0.2f\n\n', x(7))
%     
%     plot(optimValues.iteration, -optimValues.fval, '.')
%     drawnow
%     
%     out = false;
% end


function cost = gaObjFcn(param_vec, hex, knee_joint, hip_joint, sim_param, opt_param)
    % function to minimize: returns negative vertical velocity at liftoff
    % par_vec = [knee_delay, k_tendon_knee, k_tendon_hip, rad_knee, rad_hip,
    %            slope_knee, slope_hip];

    param_vec = camMapVars(param_vec, opt_param);
    
    hex.joint_objs = {copy(knee_joint), copy(hip_joint),... % add joint objects
        copy(hip_joint), copy(knee_joint)}; % (k_r, h_r, h_l, k_l)
    hex = update_hexapod_params(hex, param_vec, opt_param); % update parameters
    
    
    [~, ~, ~, ~, ~, ~, info_aerial] = simRobot(hex, sim_param);
    
%     [~, ~, ~, ~, ~, info_aerial] = sim_2leg(hex, sim_param); % simulate

    cost = -info_aerial.v(1); % negative vertical velocity
end


function [state,options,optchanged] = gaOutputFcn(options,state,~,opt_param)
    global t_comp0
    
    [v_vert_neg, idx] = min(state.Score);
    
    param_vec = camMapVars(state.Population(idx,:), opt_param);

    fprintf('\ntotal elapsed time: %s h:m:s\n', datestr(now-t_comp0,13))    
    fprintf('velocity: %0.3f m/s\n', -v_vert_neg)
    fprintf('optimal params:\n')
%     global opt_fieldnames
%     fn = opt_fieldnames;
    fn = sort(fieldnames(opt_param));
    n_fn = numel(fn);
    for idx_fn = 1:n_fn
        fprintf('    %s:  %0.2f\n', fn{idx_fn}, param_vec(idx_fn));
    end
    
%     fprintf('    knee delay:  %0.2f\n', param_vec(1))
%     fprintf('    tendon knee: %0.2f\n', param_vec(2))
%     fprintf('    tendon hip:  %0.2f\n', param_vec(3))
%     fprintf('    radius knee: %0.2f\n', param_vec(4))
%     fprintf('    radius hip:  %0.2f\n', param_vec(5))
%     fprintf('    slope knee:  %0.2f\n', param_vec(6))
%     fprintf('    slope hip:   %0.2f\n\n', param_vec(7))

    optchanged = false;
end


function param_vec = camMapVars(param_vec_disc, opt_param)
    % map cam indices to variables
    opt_param_discrete = {'rad_hip', 'rad_knee', 'slope_hip', 'slope_knee'};
    
    param_vec = param_vec_disc;
    fn = sort(fieldnames(opt_param));
    n_fn = numel(fn);
    for idx_fn = 1:n_fn
         if any(strcmp(fn{idx_fn}, opt_param_discrete)) % if discretized param
            param_vec(idx_fn) = opt_param.(fn{idx_fn})(param_vec_disc(idx_fn)); 
         end
    end
    
%     param_vec(4) = opt_param.rad_knee_vec(param_vec(4));
%     param_vec(5) = opt_param.rad_hip_vec(param_vec(5));
%     param_vec(6) = opt_param.slope_knee_vec(param_vec(6));
%     param_vec(7) = opt_param.slope_hip_vec(param_vec(7));
    
end




