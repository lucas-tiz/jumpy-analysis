function mdot_out = fminconTubeMassFlow(obj, p1, p2, l_tube, d_tube,...
    eps_tube, n_tube_seg, gas_props) 
    % Solve for mass flow through tube via fmincon optimization ([kg/s, kg/m^3])
    
    % conversions
    p1 = p1*1e-3; % (kPa to MPa)
    p2 = p2*1e-3; % (kPa to MPa)

    % calculations
    a_tube = pi*(d_tube/2)^2; % (m^2) tube cross-sectional area
    n_knot = n_tube_seg + 1;


    %% Linear constraints
    Aeq = zeros(2,n_knot + 1);
    beq = zeros(2,1);

    % boundary conditions
    Aeq(1,2) = 1; beq(1) = p1; % pres at beginning of tube = source pres
    Aeq(2,end) = 1; beq(2) = p2; % pres at end of tube = muscle pres

    A = []; b = [];


    %% Optimization
    % initial guess
    mdot_guess = 0.01; % (kg/s)
    p_guess = linspace(p1,p2,n_knot)'; % (MPa)

    x0 = [mdot_guess; p_guess];

    options = optimoptions('fmincon',...
        'Display', 'none',...
        'MaxFunctionEvaluations', 5e3*n_knot,...
        'MaxIterations', 400);%,...
%         'StepTolerance', 1e-9);

    [x,~] = fmincon(@(x)objFun(x,n_tube_seg), x0, A,b, Aeq,beq, [],[],...
        @(x)constFun(x,n_tube_seg,gas_props,l_tube,d_tube,a_tube,eps_tube), ...
        options);

    mdot = x(1);
    mdot_out = mdot(end);
    p = x(2:end);


    %% plot
%     linewidth = 1.5;
% 
%     f2 = figure(2); clf
% %     subplot(2,1,1)
%     hold on
%     grid on
%     ylim([0 80])
% %     plot(1:n_knot, p_guess*145.038 - 14.7, 'LineWidth', linewidth) % (MPa to psi)
% %     plot(1:n_knot, p*145.038 - 14.7, 'LineWidth', linewidth) % (MPa to psi)
% %     xlabel('Collocation Point')
% 
% %     plot(linspace(0, L_tube, n_knot), p_guess*145.038 - 14.7, 'LineWidth', linewidth) % (MPa to psi)
%     plot(linspace(0, l_tube, n_knot), p*145.038 - 14.7, 'LineWidth', linewidth) % (MPa to psi)
% 
% 
%     xlabel('Tube Length (m)')
%     ylabel('Gauge Pressure (psi)')
%     
% %     subplot(2,1,2)
% %     hold on
% %     grid on
% %     ylim([0 5e-2])
% %     plot(1:n_tube_seg, mdot, 'LineWidth', linewidth)
% %     xlabel('Segment')
% %     ylabel('Mass Flow Rate')


    %% Nonlinear constraints
    function [c,ceq] = constFun(x, n_tube_seg, gas_props, l_tube, d_tube, a_tube, eps_tube)

        mdot_con = x(1); % (kg/s) mass flow rate
        p_con = x(2:end); % (MPa) pressures
        dL_tube = l_tube/n_tube_seg; % (m) tube segment length

        % tube segment pressure constraints
        ceq = zeros(n_tube_seg,1);
        for k = 1:n_tube_seg
            %TODO: average pressure between nodes? or use inlet pressure?
            rho1 = interp1(gas_props.pres, gas_props.rho, ...
                (p_con(k)+p_con(k+1))/2, 'makima')*1000; % (g/mL to kg/m^3) 
                % density of air in tube segment (avg segment inlet & outlet pressures)
            mu1 = interp1(gas_props.pres, gas_props.mu, ...
                (p_con(k)+p_con(k+1))/2, 'makima'); % (Pa*s) viscosity of 
                % air in tube segment (avg segment inlet & outlet pressures)
                
%             rho1 = interp1(gas_props.pres, gas_props.rho, p_con(k), 'makima')*1000; % (g/mL to kg/m^3) density of air in tube segment (assume density at segment inlet pressure)
%             mu1 = interp1(gas_props.pres, gas_props.mu, p_con(k), 'makima'); % (Pa*s) viscosity of air in tube segment (assume density at segment inlet pressure)


            v1 = mdot_con/(rho1*a_tube); % (m/s) fluid average velocity in segment

            re = (rho1*v1*d_tube)/mu1; % fluid Reynolds number in segment
            f_D = (-1.8*log10(6.9/re + (eps_tube/(3.7*d_tube))^1.11))^(-2); % friction factor

            ceq(k) = p_con(k) - p_con(k+1) - (f_D*(dL_tube/d_tube)*(rho1*v1^2/2))*1e-6; % (MPa)            
        end
        
        c = []; % no inequality nonlinear constraints
    end


    %% Cost
    function cost = objFun(~, ~)
        cost = 0;
    end
end

