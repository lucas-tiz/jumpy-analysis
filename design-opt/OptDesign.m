classdef OptDesign %< matlab.mixin.Copyable
    
    properties 
        opt_param % robot optimization parameters
        export_param % optimization data export parameters
        objective_offset = 10; % objective function offset value 
    end
    
    properties (SetAccess = private)
        t_comp0 % computation start time
        fn
        n_fn
        robot
    end

    
    methods
        function obj = OptDesign(robot, opt_param, export_param)
            obj.robot = copy(robot); % add a copy of the robot
            obj.opt_param = opt_param;
            obj.export_param = export_param;
            
            obj.fn = sort(fieldnames(obj.opt_param));
            obj.n_fn = numel(obj.fn);
        end
        
        
        function [param_vec_opt, vy_jump_opt]  = optimize(obj, opt_type, varargin)
            obj.t_comp0 = now; % start time

            % set bounds
            lb = zeros(obj.n_fn,1); % lower bounds
            ub = zeros(obj.n_fn,1); % upper bounds
            for idx_fn = 1:obj.n_fn
                lb(idx_fn) = min(obj.opt_param.(obj.fn{idx_fn}));
                ub(idx_fn) = max(obj.opt_param.(obj.fn{idx_fn}));
            end

            % set initial optimization parameters: choose mid-value
            param_vec0 = zeros(obj.n_fn,1);
            for idx_fn = 1:obj.n_fn
                vals_vec = obj.opt_param.(obj.fn{idx_fn});            
                param_vec0(idx_fn) = (vals_vec(end) + vals_vec(1))/2;
            end


            %% fmincon gradient-based optimization
            if strcmp(opt_type, 'fmin')

                fcn = @(param_vec)obj.objFcn(param_vec); %TODO: can i use the same one everywhere?
      
                options = optimset('Display', 'iter',...
                    'MaxIter', 100,...
                    'UseParallel', true,...
                    'OutputFcn', {@obj.fminOutputFcn,@obj.fminPlotFcn});
                
                if length(varargin) == 1
                    options_mod = varargin{1};
                    fn_options = fieldnames(options_mod);
                    for i = 1:length(fn_options)
                        options.(fn_options{i}) = options_mod.(fn_options{i});
                    end
                end

                A = []; b = [];
                Aeq = []; beq = [];
                nonlcon = [];
                [param_vec_opt, fval] = fmincon(fcn, param_vec0,...
                    A,b,Aeq,beq,lb,ub,nonlcon,options);
                
                vy_jump_opt = obj.objective_offset - fval;
            end %x


            %% simulated annealing optimization
            if strcmp(opt_type, 'sa')

                fcn = @(param_vec)obj.objFcn(param_vec);
                    
                options = optimoptions('simulannealbnd',...
                    'MaxIter', 1000,... 
                    'Display', 'iter',...
                    'OutputFcn', @obj.saOutputFcn,...
                    'PlotFcns',{@saplotbestx,@obj.saplotbestfCustom,...
                        @saplotx,@obj.saplotfCustom,@saplottemperature});
                
                if length(varargin) == 1
                    options_mod = varargin{1};
                    fn_options = fieldnames(options_mod);
                    for i = 1:length(fn_options)
                        options.(fn_options{i}) = options_mod.(fn_options{i});
                    end
                end

                [param_vec_opt, fval] = simulannealbnd(fcn, param_vec0,...
                    lb,ub,options);
                
                vy_jump_opt = obj.objective_offset - fval;
            end %x


            %% particle swarm optimization
            if strcmp(opt_type, 'swarm')

                nvars = obj.n_fn;

                fcn = @(param_vec)obj.objFcn(param_vec);

                options = optimoptions('particleswarm',...
                    'SwarmSize', 2000,... 
                    'MaxIter', 100,... % (200*nvar default)
                    'Display', 'iter',...
                    'OutputFcn', @obj.swarmOutputFcn,...
                    'PlotFcns',{@obj.pswplotbestfCustom},...
                    'UseParallel', true);
                
                if length(varargin) == 1
                    options_mod = varargin{1};
                    fn_options = fieldnames(options_mod);
                    for i = 1:length(fn_options)
                        options.(fn_options{i}) = options_mod.(fn_options{i});
                    end
                end
                    
                [param_vec_opt, fval] = particleswarm(fcn, nvars, lb, ub, options);
                
                vy_jump_opt = obj.objective_offset - fval;
            end %x

            
            %% genetic algorithm
            %TODO: currently just relying on rounding in MuscleJoint for
            %discrete cam parameters
            if strcmp(opt_type, 'ga')
%                 lb = zeros(obj.n_fn,1);
%                 ub = zeros(obj.n_fn,1);
%                 idx_discrete = [];
%                 for idx_fn = 1:obj.n_fn
%                     if any(strcmp(obj.fn{idx_fn}, opt_param_discrete)) % if discretized param
%                         lb(idx_fn) = 1;
%                         ub(idx_fn) = length(opt_param.(obj.fn{idx_fn}));
%                         idx_discrete = [idx_discrete, idx_fn];
%                     else % if continuous param
%                         lb(idx_fn) = min(opt_param.(obj.fn{idx_fn}));
%                         ub(idx_fn) = max(opt_param.(obj.fn{idx_fn}));
%                     end
%                 end

                options = optimoptions(@ga, ...
                    'PopulationSize', 2000, ... %20000
                    'MaxGenerations', 100, ... %10
                    'EliteCount', 0.05*2000, ... %0.05*20000
                    'FunctionTolerance', 1e-8, ...
                    'PlotFcn', @obj.gaplotbestfCustom, ...
                    'OutputFcn', @obj.gaOutputFcn,...
                    'UseParallel', true);
                
                if length(varargin) == 1
                    options_mod = varargin{1};
                    fn_options = fieldnames(options_mod);
                    for i = 1:length(fn_options)
                        options.(fn_options{i}) = options_mod.(fn_options{i});
                    end
                end
                
                fcn = @(param_vec)obj.objFcn(param_vec);

                [param_vec_opt, fval, ~] = ga(fcn, obj.n_fn, [], [], [], [], ...
                    lb, ub, [], options);
                
                vy_jump_opt = obj.objective_offset - fval;
            end %x
            
            
            %% grid search TODO
            if strcmp(opt_type, 'grid')
                combvec_cell = cell(numel(obj.fn),1); % cell array to store opt param values
                for idx_fn = 1:numel(obj.fn)
                    combvec_cell{idx_fn} = opt_param.(obj.fn{idx_fn}); % add all values corresponding to fieldname
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
%                 for idx_opt = 1:n %DEBUG
                parfor (idx_opt = 1:n) 
                    robot_i = copy(robot); % create copy of robot

                    obj.updateOptParams(robot_i, sweep_arr(idx_opt,:), opt_param); % update parameters

                    robot_i.simRobot(sim_param); % simulate

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
                for idx_fn = 1:numel(obj.fn)
                    fprintf('    %s:  %0.2f\n', obj.fn{idx_fn}, param_vec_optimal(idx_fn));
                end    

                if export_opt_params == 1
                    exportOptParams(idx_jump_max, t_jump_max_confirm,...
                        vy_jump_max_confirm, param_vec_optimal)
                end
                fprintf('\n')
                
            end

            
            %% export optimal parameters
            if obj.export_param.opt_param == 1
                obj.exportOptParams(nan, nan, vy_jump_opt, param_vec_opt)
            end
        end


        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        %% Helper functions
        function  updateOptParams(obj, robot, param_vec)
            % Update robot with optimization parameter vector

            % convert sweep vector back to 'opt_param' struct
            op = struct(); % initialize
            for idx_fn = 1:obj.n_fn
                op.(obj.fn{idx_fn}) = param_vec(idx_fn);
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
            % shift activation times so that first activation is at t=0
            t_musc_activate = [op.t_knee, op.t_hip, op.t_hip, op.t_knee]; 
            config.control.t_musc_activate = t_musc_activate - min(t_musc_activate);
%             config.control.t_musc_activate = [op.t_knee, op.t_hip, op.t_hip, op.t_knee];    
            
            config.knee.k_tendon = op.k_tendon_knee;
            config.hip.k_tendon = op.k_tendon_hip; 
            config.knee.rad0 = op.rad_knee;
            config.hip.rad0 = op.rad_hip;
            config.knee.slope = op.slope_knee;
            config.hip.slope = op.slope_hip;
            if any(contains(obj.fn,'l_'))
                config.morphology.l = [op.l_shank, op.l_thigh,...
                    robot.config.morphology.l(3), op.l_thigh, op.l_shank];
            end
            if any(contains(obj.fn,'theta0'))
                config.state0.q0 = [0, op.theta0_knee, op.theta0_hip,...
                    op.theta0_hip, op.theta0_knee];
            end
            
            robot.updateConfig(config);
        end %x


        function exportOptParams(obj, idx_jump, t_jump, vy_jump, param_vec)  
            fid = fopen(obj.export_param.fullfile_opt_param, 'w');
            fprintf(fid, ['optimal index: %i\r\n',...
            'takeoff time: %0.3f s\r\n',...
            'takeoff velocity (y): %0.3f m/s\r\n'], idx_jump, t_jump, vy_jump);

            fprintf(fid, 'optimal params:\r\n');
            for idx_fn = 1:obj.n_fn
                fprintf(fid, '    %s:  %0.2f\r\n', obj.fn{idx_fn}, param_vec(idx_fn));
            end

            fclose(fid);
        end %x


        function cost = objFcn(obj, param_vec)
            % Objective for fmincon, simulatedannealbnd: 
            % negative vertical velocity at liftoff
%             tic
            robot_copy = copy(obj.robot); % create copy of robot
            obj.updateOptParams(robot_copy, param_vec); % update parameters
%             toc
            robot_copy.simRobot();
            cost = obj.objective_offset - robot_copy.sim_data.info_aerial.v(1);
        end %x


        function stop = fminOutputFcn(obj, x, optimvals, ~)
            % Output for fmincon
            param_vec = x;
            vy_jump = obj.objective_offset - optimvals.fval;

            fprintf('\ntotal elapsed time: %s h:m:s\n', datestr(now-obj.t_comp0,13))  
            fprintf('velocity (y): %0.3f m/s\n', vy_jump)        
            fprintf('optimal params:\n')
            for idx_fn = 1:obj.n_fn
                fprintf('    %s:  %0.2f\n', obj.fn{idx_fn}, param_vec(idx_fn));
            end

            if obj.export_param.opt_param == 2            
                obj.exportOptParams(optimvals.iteration, nan, vy_jump, param_vec)
            end     

            stop = false;
        end %x


        function stop = fminPlotFcn(obj,~,optimvals,~)
            persistent f
            if isempty(f)
                f = figure('Name', 'fmincon progress'); clf
                hold on
            end
            figure(f);

            vy_jump = obj.objective_offset - optimvals.fval;
            plot(optimvals.iteration, vy_jump, '.b') % TODO: move to plot function? existing fcn available?
            drawnow
            stop = false;
        end %x


        function [stop,options,optchanged] = saOutputFcn(obj, options, optimvals, ~)
            % Output for simulatedannealbnd

            % create fmin-output-style optimvals struct
            optimvals_fmin.x = optimvals.bestx;
            optimvals_fmin.fval = optimvals.bestfval;
            optimvals_fmin.iteration = optimvals.iteration;

            obj.fminOutputFcn(optimvals.bestx, optimvals_fmin, nan);

            stop = false;
            optchanged = false;
        end %x


        function stop = saplotfCustom(obj,~,optimvalues,flag)
            % Based on 'saplotf'
            persistent thisTitle

            stop = false;
            optimvalues.fval = obj.objective_offset - optimvalues.fval;

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
        end %x


        function stop = saplotbestfCustom(obj,~,optimvalues,flag)
            % Based on 'saplotbestf'
            persistent thisTitle
            
            stop = false;
            optimvalues.bestfval = obj.objective_offset - optimvalues.bestfval;

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
        end %x


        function stop = swarmOutputFcn(obj, optimvals, ~)
            % Output for particleswarm

            % create fmin-output-style optimvals struct
            optimvals_fmin.x = optimvals.bestx;
            optimvals_fmin.fval = optimvals.bestfval;
            optimvals_fmin.iteration = optimvals.iteration;    

            obj.fminOutputFcn(optimvals.bestx, optimvals_fmin, nan);

            stop = false;
        end %x


        function stop = pswplotbestfCustom(obj,optimvalues,state)
            %Based on 'pswplotbestf'
            stop = false;
            bestfval = obj.objective_offset - optimvalues.bestfval;
            switch state
                case 'init'
                    plotBest = plot(optimvalues.iteration,bestfval, '.b');
                    set(plotBest,'Tag','psoplotbestf');
                    xlabel('Iteration','interp','none');
                    ylabel('Function value','interp','none')
                    title(sprintf('Best Function Value: %g',bestfval),'interp','none');
                case 'iter'
                    plotBest = findobj(get(gca,'Children'),'Tag','psoplotbestf');
                    newX = [get(plotBest,'Xdata') optimvalues.iteration];
                    newY = [get(plotBest,'Ydata') bestfval];
                    set(plotBest,'Xdata',newX, 'Ydata',newY);
                    set(get(gca,'Title'),'String',sprintf('Best Function Value: %g',bestfval));
                case 'done'
                    % No clean up tasks required for this plot function.        
            end    
        end %x
        
        
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


        function [state,options,optchanged] = gaOutputFcn(obj,options,state,~)
            % Output for genetic algorithm
                
            [bestfval, idx] = min(state.Score); % get best of population
            bestx = state.Population(idx,:);

            % create fmin-output-style optimvals struct
            optimvals_fmin.x = bestx;
            optimvals_fmin.fval = bestfval;
            optimvals_fmin.iteration = state.Generation;    
 
            obj.fminOutputFcn(bestx, optimvals_fmin, nan);

            optchanged = false;
        end %x

        
        function state = gaplotbestfCustom(obj,options,state,flag)
            % Based on 'gaplotbestf'

            if size(state.Score,2) > 1
                msg = getString(message('globaloptim:gaplotcommon:PlotFcnUnavailable','gaplotbestf'));
                title(msg,'interp','none');
                return;
            end

            score = obj.objective_offset - state.Score;
            switch flag
                case 'init'
                    hold on;
                    set(gca,'xlim',[0,options.MaxGenerations]);
                    xlabel('Generation','interp','none');
                    ylabel('Fitness value','interp','none');
                    plotBest = plot(state.Generation,max(score),'.k');
                    set(plotBest,'Tag','gaplotbestf');
                    plotMean = plot(state.Generation,meanf(score),'.b');
                    set(plotMean,'Tag','gaplotmean');
                    title(['Best: ',' Mean: '],'interp','none')
                case 'iter'
                    best = max(score);
                    m    = meanf(score);
                    plotBest = findobj(get(gca,'Children'),'Tag','gaplotbestf');
                    plotMean = findobj(get(gca,'Children'),'Tag','gaplotmean');
                    newX = [get(plotBest,'Xdata') state.Generation];
                    newY = [get(plotBest,'Ydata') best];
                    set(plotBest,'Xdata',newX, 'Ydata',newY);
                    newY = [get(plotMean,'Ydata') m];
                    set(plotMean,'Xdata',newX, 'Ydata',newY);
                    set(get(gca,'Title'),'String',sprintf('Best: %g Mean: %g',best,m));
                case 'done'
                    LegnD = legend('Best fitness','Mean fitness');
                    set(LegnD,'FontSize',8);
                    hold off;
            end

            %------------------------------------------------
            function m = meanf(x)
                nans = isnan(x);
                x(nans) = 0;
                n = sum(~nans);
                n(n==0) = NaN; % prevent divideByZero warnings
                % Sum up non-NaNs, and divide by the number of non-NaNs.
                m = sum(x) ./ n;
            end
        end %x



%         function param_vec = camMapVars(param_vec_disc, opt_param, opt_param_discrete)
%             % map cam indices to variables
% 
%             param_vec = param_vec_disc;
%             fn = sort(fieldnames(opt_param));
%             n_fn = numel(fn);
%             for idx_fn = 1:n_fn
%                  if any(strcmp(fn{idx_fn}, opt_param_discrete)) % if discretized param
%                     param_vec(idx_fn) = opt_param.(fn{idx_fn})(param_vec_disc(idx_fn)); 
%                  end
%             end    
%         end
        
    end
    
    
    methods (Static)
        
    end
    
    
end