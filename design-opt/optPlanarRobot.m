%TODO:
%{
test grid search
test genetic algorithm
check speed of simulation calcs
export optimization results during runtime
export data
export config
%}

% Design optimization of robot
clear, clc

repo_folder = 'jumpy-analysis';
current_path = pwd;
idx_repo = strfind(pwd, repo_folder);
repo_path = current_path(1:(idx_repo-1 + length(repo_folder)));
addpath(fullfile(repo_path, 'modeling')); % path to JumpingRobot class

global t_comp0
t_comp0 = now;


%% Parameters
config_file = fullfile(repo_path, 'modeling', 'robot_config_default.yaml');


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

% animation parameters
animation = 1;
anim_delay = 0.00; % animation delay between frames
vid.export = 0; % animation export on
% vid.path = 'C:\Users\Lucas\Dropbox (GaTech)\Research\Hexapod\analysis\anim'; %TODO: relative path?
vid.path ='/Users/Lucas/Dropbox (GaTech)/Research/Hexapod/analysis/anim';
vid.file = 'anim_export_test_17'; % animation video file name
vid.Quality = 50;
vid.FrameRate = 60;

% data export parameters %TODO
global f_save
global save_data
f_save = 'test';
save_data = 0; %TODO


%% Instantiate robot
robot = JumpingRobot(config_file);

v.export = 0;
robot.animTrajectory(sim_param.dt,anim_delay,1,v); % display model in initial state


%% Calculate cam data before simulations
sweep_arr_knee = combvec(opt_param.rad_knee, opt_param.slope_knee)';
sweep_arr_hip = combvec(opt_param.rad_hip, opt_param.slope_hip)';
sweep_arr_joint = [sweep_arr_knee; sweep_arr_hip(~isnan(sweep_arr_hip(:,1)) & ~isnan(sweep_arr_hip(:,2)),:)];
sweep_arr_joint = unique(sweep_arr_joint,'rows'); % sweep vector of just knee/hip cam radius & slope
joint = MuscleJoint(robot.config.knee, robot.config.cam); % create dummy joint
joint.update_cam_data(sweep_arr_joint); % update cam data


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
        robot_i = copy(robot); % create copy of robot

        updateOptParams(robot_i, sweep_arr(i,:), opt_param); % update parameters

        robot_i.simRobot(sim_param); % simulate

        opt_res_arr(i) = robot_i.sim_data.info_aerial.v(1);
        send(D, [i, robot_i.sim_data.info_aerial.t, ...
            robot_i.sim_data.info_aerial.v(1), sweep_arr(i,:)]);
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
sim_param.dt = 5e-3;
sim_param.liftoff_stop = 0;

updateOptParams(robot, sweep_arr(i,:), opt_param); % update parameters
robot.simRobot(sim_param);
robot.calcJumpTrajectory();
fprintf('optimized jump height: %0.3f\n', robot.sim_data.info_jump.height)


% % save optimal data
% if save_data == 1
%     save(['data_opt/optimal_data_', f_save, '.mat'], 't_vec', 'x_arr', 'torque_arr',...
%         'j_state_arr', 'react_arr', 'obj.sim_data.info_aerial', 'y_torso', 't_jump',...
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
    

    


%% Plot
robot.plotTrajectory()
        

%% Animation
% showmotion(hexapod, t_vec, x_arr(:,1:hexapod.NB)') 

sim_param.dt = 1e-3;
robot.animTrajectory(sim_param.dt,anim_delay,5,vid);


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


function  updateOptParams(robot, sweep_vec, opt_param)
    % Update robot with optimization parameter vector

    % convert sweep vector back to 'opt_param' struct
    fn = sort(fieldnames(opt_param));
    op = struct(); % initialize
    for idx_fn = 1:numel(fn)
        op.(fn{idx_fn}) = sweep_vec(idx_fn);
    end
    
    % if hip params not set, use knee params
    if isnan(op.k_tendon_hip)
        op.k_tendon_hip = op.k_tendon_knee;
    end
    if isnan(op.rad_hip)
        op.rad_hip = op.rad_knee;
    end
    if isnan(op.slope_hip)
        op.slope_hip = op.slope_knee;
    end

    % convert 'opt_param' struct to 'config' struct
    config.control.t_musc_activate = [op.t_knee, op.t_hip, op.t_hip, op.t_knee];    
    config.knee.k_tendon = op.k_tendon_knee;
    config.hip.k_tendon = op.k_tendon_hip;
    config.knee.rad0 = op.rad_knee;
    config.hip.rad0 = op.rad_hip;
    config.knee.slope = op.slope_knee;
    config.hip.slope = op.slope_hip;

    robot.updateConfig(config)
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
%     [~, ~, ~, ~, ~, obj.sim_data.info_aerial] = sim_2leg(hex, sim_param); % simulate
% 
%     vy_jump_neg = -obj.sim_data.info_aerial.vy;
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
    hex = updateOptParams(hex, param_vec, opt_param); % update parameters
    
    
    [~, ~, ~, ~, ~, ~, obj.sim_data.info_aerial] = simRobot(hex, sim_param);
    
%     [~, ~, ~, ~, ~, obj.sim_data.info_aerial] = sim_2leg(hex, sim_param); % simulate

    cost = -obj.sim_data.info_aerial.v(1); % negative vertical velocity
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




