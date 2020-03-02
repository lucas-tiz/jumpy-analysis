%TODO:
%{
*test genetic algorithm
check speed of simulation calcs
export optimization results during runtime
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

datetime_opt = char(datetime('now','Format','yyyy-MM-dd-HH-mm-ss'));

%% Globals
global opt_param
% global opt_param_discrete
global export_fname 
global export_opt_params


%% Parameters
% file names
config_fname = fullfile(repo_path, 'modeling', 'robot_config_default.yaml');
export_path = '/Users/Lucas/Dropbox (GaTech)/Research/Hexapod/analysis/export'; %TODO: relative path?
export_fname = 'export-test'; % name prefix for export files

% optimization parameter sets
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

opt_param.rad_knee = 4:5; %3.40;
opt_param.rad_hip = 4; %4.80;
opt_param.slope_knee = 2; %-0.90;
opt_param.slope_hip = 0; %-0.60;
opt_param.k_tendon_knee = 500; %61.65;
opt_param.k_tendon_hip = 500; %37.53;
opt_param.t_knee = 0.1; %0.27;
opt_param.t_hip = 0.2; %0.29;
opt_type = 1;

% global opt_fieldnames
% opt_fieldnames = sort(fieldnames(opt_param));

% simulation parameters
sim_param.t_sim = 1; % simulation time
sim_param.dt = 1e-3; % delta t
sim_param.liftoff_stop = 1; % stop simulation at liftoff 

% data export parameters
export_opt_params = 0; % export opt params: 1=export final, 2=export updates TODO
export_sim_data = 1; % export simulation data
export_anim = 1; % export animation to video

% animation parameters
anim_delay = 0.00; % animation delay between frames
vid.Quality = 50;
vid.FrameRate = 60;


%% Instantiate robot
robot = JumpingRobot(config_fname);

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
    
    fn = sort(fieldnames(opt_param)); % sort optimization parameter fieldnames
    combvec_cell = cell(numel(fn),1); % cell array to store opt param values
    for idx_fn = 1:numel(fn)
        combvec_cell{idx_fn} = opt_param.(fn{idx_fn}); % add all values corresponding to fieldname
    end
    sweep_arr = combvec(combvec_cell{:})'; % create sweep array of all opt param combos
    s = size(sweep_arr);
    global n 
    n = s(1); % number of simulation combinations
    
    fprintf('n: %i, estimated run time: %0.1f min\n\n', n, n*sim_param.t_sim*2.1/60)

    global v_vert_jump_max;
    v_vert_jump_max = 0;
    opt_res_arr = zeros(n,1);

    global p
    p = 0; % number of simulations run
    D = parallel.pool.DataQueue;
    global h_bar
    h_bar = waitbar(0, 'Running...');
    afterEach(D, @nUpdateWaitbar);

    tic
%     for i = 1:n %DEBUG
    parfor (i = 1:n) 
        robot_i = copy(robot); % create copy of robot

        updateOptParams(robot_i, sweep_arr(i,:), opt_param); % update parameters

        robot_i.simRobot(sim_param); % simulate

        opt_res_arr(i) = robot_i.sim_data.info_aerial.v(1);
        send(D, [i, robot_i.sim_data.info_aerial.t, ...
            robot_i.sim_data.info_aerial.v(1), sweep_arr(i,:)]);
    end
    close(h_bar);
    [vy_jump_max_confirm, idx_jump_max] = max(opt_res_arr);

    fprintf('\ntotal elapsed time: %s h:m:s\n', datestr(now-t_comp0,13))    
    fprintf('optimal velocity: %0.3f m/s\n', vy_jump_max_confirm)
    fprintf('optimal params:\n')    
    for idx_fn = 1:numel(fn)
        fprintf('    %s:  %0.2f\n', fn{idx_fn}, sweep_arr(idx_jump_max,idx_fn));
    end    
end


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

updateOptParams(robot, sweep_arr(idx_jump_max,:), opt_param); % update parameters
robot.simRobot(sim_param);
robot.calcJumpTrajectory();
fprintf('optimized jump height: %0.3f\n', robot.sim_data.info_jump.height)


% % save optimal data todo
%     save(['data_opt/optimal_data_', export_fname, '.mat'], 't_vec', 'x_arr', 'torque_arr',...
%         'j_state_arr', 'react_arr', 'obj.sim_data.info_aerial', 'y_torso', 't_jump',...
%         'h_jump')
% end


%% Export data
if export_sim_data == 1
    file_export = [datetime_opt, ' - ', export_fname, '_sim-data.csv'];
    fullfile_export = fullfile(export_path, 'sim-data', file_export);
    robot.exportData(fullfile_export);
end


%% Plot
robot.plotTrajectory()
        

%% Animation
% showmotion(hexapod, t_vec, x_arr(:,1:hexapod.NB)') 

vid.export = export_anim;
vid.path =  fullfile(export_path, 'anim');
vid.file = [datetime_opt, ' - ', export_fname, '_anim']; %'anim_export_test_17'; % animation video file name

sim_param.dt = 1e-3;
robot.animTrajectory(sim_param.dt,anim_delay,5,vid);


%%
function nUpdateWaitbar(sim_info)
    global opt_param
    global v_vert_jump_max
    global export_fname
    global export_opt_params
    global p
    global n
    global h_bar
    
    waitbar(p/n, h_bar);
    p = p + 1;
    fprintf('%10.0f %10.2f%%, %10.2f s', sim_info(1), p/n*100, toc) 
    
     if sim_info(3) > v_vert_jump_max
        idx_jump_max = sim_info(1);
        t_jump_max = sim_info(2);
        v_vert_jump_max = sim_info(3);
        fprintf(', NEW MAX jump velocity: %0.3f\n', v_vert_jump_max)
        
%         sweep = sim_info(4:end);
%         fn = sort(fieldnames(opt_param));
%         for idx_fn = 1:numel(fn)
%             fprintf('         %s:  %0.2f\n', fn{idx_fn}, sweep(idx_fn));
%         end
%         fprintf('\n')
        
        if export_opt_params == 1
            fid = fopen(['data/optimal_params_',export_fname,'.txt'],'w');
            fprintf(fid, ['optimal index: %i\n',...
                'takeoff time: %0.3f s\n',...
                'takeoff velocity: %0.3f m/s\n'], idx_jump_max, t_jump_max, vy_jump_max);
                
            sweep = sim_info(4:end);
            fn = sort(fieldnames(opt_param));
            for idx_fn = 1:numel(fn)
                fprintf(fid, '    %s:  %0.2f\n', fn{idx_fn}, sweep(idx_fn));
            end
            
            fclose(fid);
        end         
     end
     fprintf('\n')
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




