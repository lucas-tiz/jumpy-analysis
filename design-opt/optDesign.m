function [param_vec_opt, vy_jump_opt]  = optDesign(robot, opt_type, sim_param, opt_param)

    global t_comp0
    global obj_offset
    t_comp0 = now; % start time
    obj_offset = 10; 


    % get fieldnames
    fn = sort(fieldnames(opt_param));
    n_fn = numel(fn);

    % set bounds
    lb = zeros(n_fn,1); % lower bounds
    ub = zeros(n_fn,1); % upper bounds
    for idx_fn = 1:n_fn
        lb(idx_fn) = min(opt_param.(fn{idx_fn}));
        ub(idx_fn) = max(opt_param.(fn{idx_fn}));
    end
    
    % set initial optimization parameters: choose mid-value
    param_vec0 = zeros(n_fn,1);
    for idx_fn = 1:n_fn
        vals_vec = opt_param.(fn{idx_fn});            
        param_vec0(idx_fn) = (vals_vec(end) + vals_vec(1))/2;
    end
    
    % create figure for plotting optimization progress
    figure(1111); clf 
    hold on

    
    %% fmincon gradient-based optimization
    if strcmp(opt_type, 'fmin')

        fcn = @(param_vec)objFcn(param_vec, robot, sim_param, opt_param,...
            obj_offset);
        
        options = optimset('Display', 'iter',...
            'MaxIter', 100,... %TODO
            'UseParallel', true,... %TODO
            'OutputFcn', {@fminOutputFcn,@fminPlotFcn});

        A = []; b = [];
        Aeq = []; beq = [];
        nonlcon = [];
        [param_vec_opt, vy_jump_opt_neg] = fmincon(fcn, param_vec0,...
            A,b,Aeq,beq,lb,ub,nonlcon,options);
        vy_jump_opt = -vy_jump_opt_neg;
    end
    
    
    %% simulated annealing optimization
    if strcmp(opt_type, 'sa')
        
        fcn = @(param_vec)objFcn(param_vec, robot, sim_param, opt_param,...
            obj_offset);

        options = optimoptions('simulannealbnd',...
            'MaxIter', 1000,... %TODO
            'Display', 'iter',...
            'OutputFcn', @saOutputFcn,...
            'PlotFcns',{@saplotbestx,@saplotbestfCustom,@saplotx,@saplotfCustom,...
                @saplottemperature});
        
        [param_vec_opt, vy_jump_opt_neg] = simulannealbnd(fcn, param_vec0,...
            lb,ub,options);
        vy_jump_opt = -vy_jump_opt_neg;
    end
    
    
    %% particle swarm optimization
    if strcmp(opt_type, 'swarm')
        
        nvars = n_fn;
        
        fcn = @(param_vec)objFcn(param_vec, robot, sim_param, opt_param,...
            obj_offset);

        options = optimoptions('particleswarm',...
            'SwarmSize',5,... %TODO
            'MaxIter', 200*nvars,... % (200*nvar default) TODO
            'Display', 'iter',...
            'OutputFcn', @swarmOutputFcn,...
            'PlotFcns',{@pswplotbestfCustom},...
            'UseParallel', true);
        
        [param_vec_opt, vy_jump_opt_neg] = particleswarm(fcn, nvars, lb, ub, options);

        vy_jump_opt = -vy_jump_opt_neg;
    end
    
end


%% Functions
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
    config.knee.k_tendon = op.k_tendon_knee*1000; %DEBUG
    config.hip.k_tendon = op.k_tendon_hip*1000; %DEBUG
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


function cost = objFcn(param_vec, robot, sim_param, opt_param, obj_offset)
    % Objective for fmincon, simulatedannealbnd: 
    % negative vertical velocity at liftoff    
    robot_copy = copy(robot); % create copy of robot
    updateOptParams(robot_copy, param_vec, opt_param); % update parameters
    robot_copy.simRobot(sim_param);
    cost = obj_offset - robot_copy.sim_data.info_aerial.v(1);
end


function stop = fminOutputFcn(x, optimvals, ~)
    % Output for fmincon
    global opt_param
    global t_comp0
    global obj_offset
    
    param_vec = x;
    vy_jump = obj_offset - optimvals.fval;
    
    fn = sort(fieldnames(opt_param));
    n_fn = numel(fn);
    fprintf('\ntotal elapsed time: %s h:m:s\n', datestr(now-t_comp0,13))  
    fprintf('velocity (y): %0.3f m/s\n', vy_jump)        
    fprintf('optimal params:\n')


    for idx_fn = 1:n_fn
        fprintf('    %s:  %0.2f\n', fn{idx_fn}, param_vec(idx_fn));
    end
    %TODO:
%         if export_opt_params == 2            
%             exportOptParams(state.Generation, nan, -v_vert_neg, param_vec)
%         end     

    stop = false;
end


function stop = fminPlotFcn(~,optimvals,~)
    global obj_offset
    vy_jump = obj_offset - optimvals.fval;
    plot(optimvals.iteration, vy_jump, '.b') % TODO: move to plot function? existing fcn available?
    drawnow
    stop = false;
end


function [stop,options,optchanged] = saOutputFcn(options, optimvals, ~)
    % Output for simulatedannealbnd
    
    % create fmin-output-style optimvals struct
    optimvals_fmin.x = optimvals.bestx;
    optimvals_fmin.fval = optimvals.bestfval;
    optimvals_fmin.iteration = optimvals.iteration;
    
    fminOutputFcn(optimvals.bestx, optimvals_fmin, nan);
    
    stop = false;
    optchanged = false;
end


function stop = saplotfCustom(~,optimvalues,flag)
    % Based on 'saplotf'
    persistent thisTitle
    global obj_offset

    stop = false;
    optimvalues.fval = obj_offset - optimvalues.fval;
    
    switch flag
        case 'init'
            plotBest = plot(optimvalues.iteration,optimvalues.fval, '.b');
            set(plotBest,'Tag','saplotf');
%             hold on
            xlabel('Iteration','interp','none'); 
            ylabel('Function value','interp','none')
            thisTitle = title(sprintf('Current Function Value: %g',optimvalues.fval),'interp','none');
        case 'iter'
            plotBest = findobj(get(gca,'Children'),'Tag','saplotf');
            newX = [get(plotBest,'Xdata') optimvalues.iteration];
            newY = [get(plotBest,'Ydata') optimvalues.fval];
            set(plotBest,'Xdata',newX, 'Ydata',newY);        
            if isempty(thisTitle)
                set(get(gca,'Title'),'String',sprintf('Current Function Value: %g',optimvalues.fval));
            else
                set(thisTitle,'String',sprintf('Current Function Value: %g',optimvalues.fval));
            end
    end
end


function stop = saplotbestfCustom(~,optimvalues,flag)
    % Based on 'saplotbestf'
    persistent thisTitle
    global obj_offset

    stop = false;
    optimvalues.bestfval = obj_offset - optimvalues.bestfval;
    
    switch flag
        case 'init'
            plotBest = plot(optimvalues.iteration,optimvalues.bestfval, '.b');
            set(plotBest,'Tag','saplotbestf');
            xlabel('Iteration','interp','none');
            ylabel('Function value','interp','none')
            thisTitle = title(sprintf('Best Function Value: %g',optimvalues.bestfval),'interp','none');
        case 'iter'
            plotBest = findobj(get(gca,'Children'),'Tag','saplotbestf');
            newX = [get(plotBest,'Xdata') optimvalues.iteration];
            newY = [get(plotBest,'Ydata') optimvalues.bestfval];
            set(plotBest,'Xdata',newX, 'Ydata',newY);
            if isempty(thisTitle)
                set(get(gca,'Title'),'String',sprintf('Best Function Value: %g',optimvalues.bestfval));
            else
                set(thisTitle,'String',sprintf('Best Function Value: %g',optimvalues.bestfval));
            end
    end
end


function stop = swarmOutputFcn(optimvals, ~)
    % Output for particleswarm

    % create fmin-output-style optimvals struct
    optimvals_fmin.x = optimvals.bestx;
    optimvals_fmin.fval = optimvals.bestfval;
    optimvals_fmin.iteration = optimvals.iteration;    

    fminOutputFcn(optimvals.bestx, optimvals_fmin, nan);
    
    stop = false;
end


function stop = pswplotbestfCustom(optimvalues,state)
    %Based on 'pswplotbestf'
    global obj_offset
    stop = false;
    optimvalues.bestfval = obj_offset - optimvalues.bestfval;
    switch state
        case 'init'
            plotBest = plot(optimvalues.iteration,optimvalues.bestfval, '.b');
            set(plotBest,'Tag','psoplotbestf');
            xlabel('Iteration','interp','none');
            ylabel('Function value','interp','none')
            title(sprintf('Best Function Value: %g',optimvalues.bestfval),'interp','none');
        case 'iter'
            plotBest = findobj(get(gca,'Children'),'Tag','psoplotbestf');
            newX = [get(plotBest,'Xdata') optimvalues.iteration];
            newY = [get(plotBest,'Ydata') optimvalues.bestfval];
            set(plotBest,'Xdata',newX, 'Ydata',newY);
            set(get(gca,'Title'),'String',sprintf('Best Function Value: %g',optimvalues.bestfval));
        case 'done'
            % No clean up tasks required for this plot function.        
    end    
end


