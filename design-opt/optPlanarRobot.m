%TODO:
%{
*global cam_map so that map doesn't have to be copied a bunch?

*normalize inputs

update leg masses to reflect measurements + maybe add artificial torso
    weight
adjust leg mass based on leg length

check speed of simulation calcs

export config

create parallelized meta-optimization?

*higher-res cam data

option for turning on/off cam param rounding
%}

% Design optimization of robot
clear
clear global
% clc

repo_folder = 'jumpy-analysis';
current_path = pwd;
idx_repo = strfind(pwd, repo_folder);
repo_path = current_path(1:(idx_repo-1 + length(repo_folder)));
addpath(fullfile(repo_path, 'modeling')); % path to JumpingRobot class

global t_comp0
t_comp0 = now;

datetime_opt_start = char(datetime('now','Format','yyyy-MM-dd-HH-mm-ss'));


%% Globals
global opt_param %TODO: somehow remove this?
global export_fname 
global export_opt_params
global fullfile_export_opt_params


%% Parameters
% file names
config_fname = fullfile(repo_path, 'modeling', 'robot_config_mod.yaml');
export_path = '/Users/Lucas/Dropbox (GaTech)/Research/Hexapod/analysis/export'; %TODO: relative path?
export_fname = 'TEST'; % name prefix for export files

% optimization parameter sets
opt_param_discrete = {'rad_hip', 'rad_knee', 'slope_hip', 'slope_knee'};

opt_param.k_tendon_hip = linspace(10, 3000, 10)/1000; % (kN cm) %DEBUG
opt_param.k_tendon_knee = linspace(10, 3000, 10)/1000; % (kN cm) %DEBUG
opt_param.l_shank = 0.25:0.1:0.55; % (m)
opt_param.l_thigh = 0.25:0.1:0.55; % (m)
opt_param.rad_hip = -3:0.1:6; %0:1:6; % (cm)
opt_param.rad_knee = -3:0.1:6; %0:1:6; % (cm)
opt_param.slope_hip = -1:0.1:4; %-4:1:4; % (cm/rad)
opt_param.slope_knee = -1:0.1:4; %-4:1:4; % (cm/rad)
opt_param.t_hip = 0.0:0.1:0.5; % (s)
opt_param.t_knee = 0.0:0.1:0.5; % (s)
opt_param.theta0_hip = -90:10:90; % (deg)
opt_param.theta0_knee = 0:10:180; % (deg)
opt_type = 2;
% rng(0, 'twister'); % 'twister' for repeatable or 'shuffle'
rng('shuffle');

											
opt_param.k_tendon_hip = 0.1; %0.8009; %37.53;
opt_param.k_tendon_knee = 0.1;%0.10; %1.9989; %61.65;
opt_param.l_shank = 0.55; %0.55;
opt_param.l_thigh = 0.55; %0.25:0.1:0.55;
opt_param.rad_hip = 4; %4.80;
opt_param.rad_knee = 4; %3.40;
opt_param.slope_hip = 0; %-0.60;
opt_param.slope_knee = 0; %-0.90;
opt_param.t_hip = 0.0;%0.26; %0.29;
opt_param.t_knee = 0.0;%0.35; %0.27;
opt_param.theta0_hip = -60;
opt_param.theta0_knee = 158;
opt_type = 1;

% simulation parameters for design optimization
sim_param.t_sim = 1; % (s) simulation time
sim_param.dt = 1e-3; % (s) 1e-3 delta t
sim_param.liftoff_stop = 1; % stop simulation at liftoff 
sim_param.n_tube_seg = 3; % number of tube discretization segments

% data export parameters
export_param.opt_param = 0; % export opt params: 1=export final, 2=export updates
export_param.sim_data = 0; % export simulation data
export_param.anim = 0; % export animation to video

% animation parameters
anim_delay = 0.00; % animation delay between frames
vid.Quality = 50;
vid.FrameRate = 60;


%% Files
file_export_sim_data = [datetime_opt_start, ' - ', export_fname, ' sim-data'];
fullfile_export_sim_data = fullfile(export_path, 'sim-data', file_export_sim_data); %TODO

file_export_opt_params = [datetime_opt_start, ' - ', export_fname, ' opt-data.txt'];
fullfile_export_opt_params = fullfile(export_path, 'opt-data', file_export_opt_params);

export_param.fullfile_opt_param = fullfile_export_opt_params; %TODO


%% Instantiate robot
robot = JumpingRobot(config_fname, sim_param); % instantiate
v.export = 0; robot.animTrajectory(0.01,anim_delay,16,v,[0,0]); % display model in initial state


%% Calculate cam data before simulations
sweep_arr_knee = combvec(opt_param.rad_knee, opt_param.slope_knee)';
sweep_arr_hip = combvec(opt_param.rad_hip, opt_param.slope_hip)';
sweep_arr_joint = [sweep_arr_knee; sweep_arr_hip(~isnan(sweep_arr_hip(:,1))...
    & ~isnan(sweep_arr_hip(:,2)),:)]; % sweep vector of knee/hip cam radius & slope
sweep_arr_joint = unique(sweep_arr_joint,'rows'); % get unique rows
joint = MuscleJoint(robot.config.knee, robot.config.cam, robot.config.pneumatic); % create dummy joint
joint.update_cam_data(sweep_arr_joint); % update cam data


%% Optimization
optimizer = OptDesign(robot, opt_param, export_param);


% options.MaxIter = 3;
% options.UseParallel = true;
% [param_vec_optimal, vy_jump_opt]  = optimizer.optimize('fmin', options);

% options.MaxIter = 3;
% options.Display = 'iter';
% [param_vec_optimal, vy_jump_opt]  = optimizer.optimize('sa', options);

% options.SwarmSize = 5;%2000;
% options.MaxIter = 3;%100;
% options.UseParallel = true;
% [param_vec_optimal, vy_jump_opt]  = optimizer.optimize('swarm', options);


%TODO:
%{
- change elite count
- change crossover fraction
- crossover function: heuristic? arithemtic? etc.
%}
% options.PopulationSize = 5;
% options.MaxGenerations = 3;
% options.EliteCount = 2;
% options.UseParallel = true;
% [param_vec_optimal, vy_jump_opt]  = optimizer.optimize('ga', options);




% update robot to first index of 'opt_param'
% fn = sort(fieldnames(opt_param));
% param_vec_optimal = zeros(numel(fn),1);
% for idx_fn = 1:numel(fn)
%     opt_param_vec = opt_param.(fn{idx_fn});
%     param_vec_optimal(idx_fn) = opt_param_vec(1);
% end
% optimizer.updateOptParams(robot, param_vec_optimal)





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
    opt_res_arr = zeros(n,2);

    global p
    p = 0; % number of simulations run
    D = parallel.pool.DataQueue;
    global h_bar
    h_bar = waitbar(0, 'Running...');
    afterEach(D, @gridSearchUpdate);

    tic
    for idx_opt = 1:n %DEBUG
%     parfor (idx_opt = 1:n) 
        robot_i = copy(robot); % create copy of robot

        optimizer.updateOptParams(robot_i, sweep_arr(idx_opt,:)); % update parameters
%         updateOptParams(robot_i, sweep_arr(idx_opt,:), opt_param); % update parameters

        robot_i.simRobot(); % simulate

        opt_res_arr(idx_opt,:) = [robot_i.sim_data.info_aerial.t,... 
            robot_i.sim_data.info_aerial.v(1)]; % save velocity
                
        send(D, [idx_opt, robot_i.sim_data.info_aerial.t,...
            robot_i.sim_data.info_aerial.v, sweep_arr(idx_opt,:)]); % send data to update function
    end
    close(h_bar);
    [vy_jump_max_confirm, idx_jump_max] = max(opt_res_arr(:,2));
    t_jump_max_confirm = opt_res_arr(idx_jump_max,1);
    param_vec_optimal = sweep_arr(idx_jump_max,:);
    
    fprintf('\ntotal elapsed time: %s h:m:s\n\n', datestr(now-t_comp0,13))
    fprintf('optimal index: %i \n', idx_jump_max)
    fprintf('takeoff time: %0.3f s \n', t_jump_max_confirm)
    fprintf('takeoff velocity (y): %0.3f m/s\n', vy_jump_max_confirm)
    fprintf('optimal params:\n')    
    for idx_fn = 1:numel(fn)
        fprintf('    %s:  %0.2f\n', fn{idx_fn}, param_vec_optimal(idx_fn));
    end    
    
    if export_opt_params == 1
        exportOptParams(idx_jump_max, t_jump_max_confirm,...
            vy_jump_max_confirm, param_vec_optimal)
    end
end


% % genetic algorithm
% if opt_type == 3    
%     fn = sort(fieldnames(opt_param));
%     n_fn = numel(fn);
%     lb = zeros(n_fn,1);
%     ub = zeros(n_fn,1);
%     idx_discrete = [];
%     for idx_fn = 1:n_fn
%         if any(strcmp(fn{idx_fn}, opt_param_discrete)) % if discretized param
%             lb(idx_fn) = 1;
%             ub(idx_fn) = length(opt_param.(fn{idx_fn}));
%             idx_discrete = [idx_discrete, idx_fn];
%         else % if continuous param
%             lb(idx_fn) = min(opt_param.(fn{idx_fn}));
%             ub(idx_fn) = max(opt_param.(fn{idx_fn}));
%         end
%     end
%     
%     %TODO:
%     %{
%     - change elite count
%     - change crossover fraction
%     - crossover function: heuristic? arithemtic? etc.
%     %}
%     
%     opts = optimoptions(@ga, ...
%         'PopulationSize', 5, ... %20000
%         'MaxGenerations', 3, ... %10
%         'EliteCount', 2, ... %0.05*20000
%         'FunctionTolerance', 1e-8, ...
%         'PlotFcn', @gaplotbestf, ...
%         'OutputFcn', @(options,state,flag)gaOutputFcn(options,state,flag,...
%             opt_param, opt_param_discrete),...
%         'UseParallel', true);
%     
%     objFun = @(param_vec)gaObjFcn(param_vec, robot, sim_param, opt_param,...
%         opt_param_discrete);
%                                         
%     [xbest, fbest, exitflag] = ga(objFun, n_fn, [], [], [], [], ...
%         lb, ub, [], idx_discrete, opts);
%     
%     param_vec_optimal = camMapVars(xbest, opt_param, opt_param_discrete);
%     
%     if export_opt_params == 1
%         exportOptParams(nan, nan,...
%             -fbest, param_vec_optimal)
%     end
% end


%% Simulate optimal config
fprintf('\nsimulating optimal config\n')
robot.sim_param.t_sim = 1;
robot.sim_param.dt = 5e-4;
robot.sim_param.liftoff_stop = 0;

optimizer.updateOptParams(robot, param_vec_optimal); % update parameters
v.export = 0; robot.animTrajectory(sim_param.dt,anim_delay,15,v,[0,0]); % display model in initial state
robot.simRobot();
robot.calcJumpTrajectory();
fprintf('optimized jump height: %0.3f\n', robot.sim_data.info_jump.height)


%% Export simulation data
if export_param.sim_data == 1
    robot.exportSimData(fullfile_export_sim_data);
end


%% Plot simulation data & show jumping frames
robot.plotTrajectory()
% robot.dispJumpSeq(0:1/9:1, [2,5], 10);
        

%% Animation
% showmotion(robot.spatialRobot, robot.sim_data.t,...
%     robot.sim_data.x(:,1:robot.spatialRobot.NB)');

vid.export = export_param.anim;
vid.path =  fullfile(export_path, 'anim');
vid.file = [datetime_opt_start, ' - ', export_fname, ' anim']; %'anim_export_test_17'; % animation video file name

dt = robot.sim_param.dt*5;
% anim_delay = 0.05;
robot.animTrajectory(dt,anim_delay,21,vid,[0 robot.sim_param.t_sim]);


% robot.animTrajectory(dt,anim_delay,16,vid,[0.15 0.35]);


%% Functions
function gridSearchUpdate(sim_info)
    global v_vert_jump_max
    global export_opt_params
    global p
    global n
    global h_bar
    
    waitbar(p/n, h_bar); % update waitbar progress
    p = p + 1; % update optimization counter
    fprintf('%10.0f %10.2f%%, %10.2f s', sim_info(1), p/n*100, toc) % print progress
    
    if sim_info(3) > v_vert_jump_max % if new max, update
        idx_jump_max = sim_info(1);
        v_vert_jump_max = sim_info(3);
        fprintf(', NEW MAX jump velocity: %0.3f', v_vert_jump_max)
        
        if export_opt_params == 2            
            sweep = sim_info(4:end);
            exportOptParams(idx_jump_max, sim_info(2), sim_info(3), sweep)
        end         
     end
     fprintf('\n')
end


function exportOptParams(idx_jump, t_jump, vy_jump, param_vec)  
    global fullfile_export_opt_params
    global opt_param

    fid = fopen(fullfile_export_opt_params, 'w');
    fprintf(fid, ['optimal index: %i\r\n',...
    'takeoff time: %0.3f s\r\n',...
    'takeoff velocity (y): %0.3f m/s\r\n'], idx_jump, t_jump, vy_jump);

    fn = sort(fieldnames(opt_param));
    fprintf(fid, 'optimal params:\r\n');
    for idx_fn = 1:numel(fn)
        fprintf(fid, '    %s:  %0.2f\r\n', fn{idx_fn}, param_vec(idx_fn));
    end

    fclose(fid);
end


function  updateOptParams(robot, param_vec, opt_param)
    % Update robot with optimization parameter vector

    % convert sweep vector back to 'opt_param' struct
    fn = sort(fieldnames(opt_param));
    op = struct(); % initialize
    for idx_fn = 1:numel(fn)
        op.(fn{idx_fn}) = param_vec(idx_fn);
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
    config.knee.k_tendon = op.k_tendon_knee*1000;
    config.hip.k_tendon = op.k_tendon_hip*1000;
    config.knee.rad0 = op.rad_knee;
    config.hip.rad0 = op.rad_hip;
    config.knee.slope = op.slope_knee;
    config.hip.slope = op.slope_hip;
    if any(contains(fn,'l_'))
        config.morphology.l = [op.l_shank, op.l_thigh,...
            robot.config.morphology.l(3), op.l_thigh, op.l_shank];
    end
    robot.updateConfig(config)
end


% function cost = gaObjFcn(param_vec, robot, sim_param, opt_param,...
%     opt_param_discrete)
%     % function to minimize: returns negative vertical velocity at liftoff
%     
%     robot_i = copy(robot); % create copy of robot
%     param_vec = camMapVars(param_vec, opt_param, opt_param_discrete);
%     updateOptParams(robot_i, param_vec, opt_param); % update parameters
%     
%     robot_i.simRobot(sim_param);
%     
%     cost = -robot_i.sim_data.info_aerial.v(1); % negative vertical velocity
% end
% 
% 
% function [state,options,optchanged] = gaOutputFcn(options,state,~,opt_param,...
%     opt_param_discrete)
%     global t_comp0
%     global export_opt_params
%     
%     [v_vert_neg, idx] = min(state.Score); % get best of population
%     param_vec = camMapVars(state.Population(idx,:), opt_param,...
%         opt_param_discrete); % get corresponding params
% 
%     fprintf('\ntotal elapsed time: %s h:m:s\n', datestr(now-t_comp0,13))    
%     fprintf('velocity (y): %0.3f m/s\n', -v_vert_neg)
%     fprintf('optimal params:\n')
%     fn = sort(fieldnames(opt_param));
%     n_fn = numel(fn);
%     for idx_fn = 1:n_fn
%         fprintf('    %s:  %0.2f\n', fn{idx_fn}, param_vec(idx_fn));
%     end
% 
%     if export_opt_params == 2            
%         exportOptParams(state.Generation, nan, -v_vert_neg, param_vec)
%     end     
%     
%     optchanged = false;
% end


function param_vec = camMapVars(param_vec_disc, opt_param, opt_param_discrete)
    % map cam indices to variables

    param_vec = param_vec_disc;
    fn = sort(fieldnames(opt_param));
    n_fn = numel(fn);
    for idx_fn = 1:n_fn
         if any(strcmp(fn{idx_fn}, opt_param_discrete)) % if discretized param
            param_vec(idx_fn) = opt_param.(fn{idx_fn})(param_vec_disc(idx_fn)); 
         end
    end    
end




