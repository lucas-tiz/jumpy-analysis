clear, clc


%% block powered by PAM "thrusters", one in +y direction, one in -y direction

% m = 1;
% g = 9.81;
% 
% x0 = [1,-1];
% u = [12,0];
% 
% sysDyn = @(t,x,u,m) ([x(2); (u(1) - u(2) - m*g)/m]);
% 
% t_span = [0 1];
% [t,x] = ode45(@(t,x) sysDyn(t,x,u,m), t_span, x0);
% 
% subplot(2,1,1)
% plot(t,x(:,1))
% grid on
% 
% subplot(2,1,2)
% plot(t,x(:,2))
% grid on





%% linear constraints
% Aeq = zeros(4,n_knot*4 + n_times);
% beq = zeros(4,1);
% 
% % boundary conditions
% Aeq(1,1) = 1; beq(1) = 0; % y0 = 0
% Aeq(2,n_knot) = 1; beq(2) = 1; % yf = 1
% Aeq(3, n_knot+1) = 1; beq(3) = 0; % vy0 = 0
% Aeq(4, n_knot+n_knot) = 1; beq(4) = 0; % vyf = 0
% 
% % -tf <= 0
% A1 = zeros(1,n_knot*4 + n_times);
% A1(1,4*n_knot+1) = -1;
% b1 = 0;
% % -t1 <=0
% A2 = zeros(1,n_knot*4 + n_times);
% A2(1,4*n_knot+2) = -1;
% b2 = 0;
% 
% % t1 - t2 <= 0
% A3 = zeros(1,n_knot*4 + n_times);
% A3(1,4*n_knot+2) = 1; 
% A3(1,4*n_knot+3) = -1;
% b3 = 0; 
% % t2 - t3 <= 0
% A4 = zeros(1,n_knot*4 + n_times);
% A4(1,4*n_knot+3) = 1; 
% A4(1,4*n_knot+4) = -1;
% b4 = 0; 
% % % t3 - t4 <= 0
% % A5 = zeros(1,n_knot*4 + n_times);
% % A5(1,4*n_knot+4) = 1; 
% % A5(1,4*n_knot+5) = -1;
% % b5 = 0; 
% % t4 - t5 <= 0
% A6 = zeros(1,n_knot*4 + n_times);
% A6(1,4*n_knot+5) = 1; 
% A6(1,4*n_knot+6) = -1;
% b6 = 0; 
% 
% % u1 <= 25
% A7 = zeros(n_knot,n_knot*4 + n_times);
% A7(:,(2*n_knot+1):(3*n_knot)) = eye(n_knot);
% b7 = 25*ones(n_knot,1);
% 
% % % -u2 <= 0
% % A3 = zeros(n_knot,n_knot*4 + n_times);
% % b3 = zeros(n_knot,1);
% % A3(:,(3*n_knot+1):(4*n_knot)) = -eye(n_knot);
% 
% % u_max = 30;
% % % u1 <= u_max
% % A4 = zeros(n_knot,n_knot*4 + n_times);
% % b4 = u_max*ones(n_knot,1);
% % A4(:,(2*n_knot+1):(3*n_knot)) = eye(n_knot);
% % 
% % % u2 <= u_max
% % A5 = zeros(n_knot,n_knot*4 + n_times);
% % b5 = u_max*ones(n_knot,1);
% % A5(:,(3*n_knot+1):(4*n_knot)) = eye(n_knot);
% 
% 
% A = [A1;A2;A3;A4;A6;A7];
% b = [b1;b2;b3;b4;b6;b7];


% all times >= 0
A1 = -eye(3,3);
b1 = zeros(3,1);

% t3 >= t1 + 0.67 --> -0.67 >= t1 - t3
A2 = [1 -1 0];
b2 = -0.67;

% tf >= t3;
A3 = [0 1 -1];
b3 = 0;


A = [A1;A2;A3];
b = [b1;b2;b3];

Aeq = [];
beq = [];


%% optimization
% initial guess
t1_guess = 0.0;
t2_guess = 0.5;
t3_guess = 0.94;
tf_guess = 1.36;

% x0 = [t1_guess; t2_guess; t3_guess; tf_guess];
x0 = [t1_guess; t3_guess; tf_guess];

options = optimoptions('fmincon', 'Display', 'iter-detailed',...
    'MaxFunctionEvaluations', 5e3,...
    'MaxIterations', 400,...
    'StepTolerance', 1e-12);

[x,fval] = fmincon(@(x)objFun(x), x0, A,b, Aeq,beq, [],[],...
    @(x)constFun(x), options);


%% results
t1 = x(1);
t2 = t1 + 0.67;
% t2 = x(2);
t3 = x(2);
tf = x(3);

[yf, vyf, t, y, vy, u] = sysShoot(t1,t2,t3,tf);


%% plot
linewidth = 1.5;

figure(1); clf
subplot(1,3,1)
hold on
grid on
plot(t, y, 'LineWidth', linewidth)
xlabel('Time (s)')
ylabel('Position')

subplot(1,3,2)
hold on
grid on
plot(t, vy, 'LineWidth', linewidth)
xlabel('Time (s)')
ylabel('Velocity')

subplot(1,3,3)
hold on
grid on
plot(t, u, 'LineWidth', linewidth)
xlabel('Time (s)')
ylabel('Input')


%% nonlinear constraints
function [c,ceq] = constFun(x)
    t1 = x(1);
    t2 = t1+0.67;
%     t2 = x(2);
    t3 = x(2);
    tf = x(3);

    c = [];
    ceq = zeros(2,1);

    [yf, vyf, ~,~,~,~] = sysShoot(t1,t2,t3,tf);

    ceq(1) = yf - 1;
    ceq(2) = vyf - 0;
%     ceq = ceq*100;
end


%%
function cost = objFun(x)
    t1 = x(1);
%     t2 = x(2);
    t3 = x(2);
    tf = x(3);

%     cost = sum(u1.*u1) + sum (u2.*u2);
    cost = tf^2;
%     cost = tf + (t2-t1);
%     cost = 0;
end


function f = logistic(x, L, x0, alpha)
    % logistic function: L = max value, x0 = midpoint, alpha = growth rate
    f = L./(1 + exp(-alpha*(x-x0)));
end



function [yf, vyf, t, y, vy, u] = sysShoot(t1,t2,t3,tf)

    % sim parameters
    dt = 0.001;
    t = 0:dt:tf;
    n_step = length(t);
    
    % input
    alpha_log = 500;
    p_coeffs = [1 1 1 0]*17;
    alpha_exp = 4;
    
    u_inf = logistic(t, 1, t1, alpha_log) - logistic(t, 1, t2, alpha_log);
    p_inf = polyval(p_coeffs,t-t1);

    u_seal = logistic(t, 1, t2, alpha_log) - logistic(t, 1, t3, alpha_log);
    p_seal = polyval(p_coeffs,t2-t1);

    u_def = logistic(t, 1, t3, alpha_log);
    p_def = exp(-alpha_exp*(t-t3))*p_seal;

    u = (u_inf.*p_inf + u_seal.*p_seal + u_def.*p_def);
    
    % simulation
    y = zeros(n_step,1);
    vy = zeros(n_step,1);
    for i = 2:length(t)
        y(i) = y(i-1) + dt* vy(i-1);        
        vy(i) = vy(i-1) + dt*u(i-1);
    end
    
    yf = y(end);
    vyf = vy(end);
end


