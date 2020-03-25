clear, clc


%%
t = 0:0.01:4;
% t = 2.9:0.00001:3.5;


t1 = 1;
t2 = 1.5;
t3 = 2;

alpha_log = 500; % 800
p_coeffs = [1 1 1 0]*10; % 17
alpha_exp = 6;

y_inf = logistic(t, 1, t1, alpha_log) - logistic(t, 1, t2, alpha_log);
p_inf = polyval(p_coeffs,t-t1);

y_seal = logistic(t, 1, t2, alpha_log) - logistic(t, 1, t3, alpha_log);
p_seal = polyval(p_coeffs,t2-t1);

y_def = logistic(t, 1, t3, alpha_log);
p_def = exp(-alpha_exp*(t-t3))*p_seal;


u = y_inf.*p_inf + y_seal*p_seal + y_def.*p_def;

linewidth = 1.5;
f = figure(1122); clf
subplot(2,1,1)
grid on
hold on
plot(t,y_inf, 'LineWidth', linewidth)
plot(t,y_seal, 'LineWidth', linewidth)
plot(t,y_def, 'LineWidth', linewidth)
% plot(t,y+y2, ':k', 'LineWidth', 3)
legend('Open valve', 'Seal valve', 'Vent valve')%, 'Interpreter', 'Latex')
xlabel('Time')%, 'Interpreter', 'Latex')
ylabel('Signal')%, 'Interpreter', 'Latex')


subplot(2,1,2)
grid on
hold on
% plot(t,y_inf, 'LineWidth', linewidth)
% plot(t,y_seal, 'LineWidth', linewidth)
% plot(t,y_def, 'LineWidth', linewidth)
plot(t,u, 'k', 'LineWidth', linewidth)
plot(t,p_inf, ':', 'LineWidth', 1.25)
plot(t,ones(size(t))*p_seal, ':', 'LineWidth', 1.25)
plot(t,p_def, ':', 'LineWidth', 1.25)
% plot(t,p_def)
% xlim([0 5])
ylim([-5 15])
legend('Combined', 'Inflate', 'Seal', 'Deflate')%, 'Interpreter', 'Latex')
xlabel('Time')%, 'Interpreter', 'Latex')
ylabel('Pressure')%, 'Interpreter', 'Latex')

pub_figureFormat(f, 'CMU Serif')
set(f, 'Position', [-853, 405, 450, 450])

filepath = 'C:\Users\Lucas\Dropbox (GaTech)\Research\Thesis\proposal\diagrams\logistic smoothing';
print(fullfile(filepath, 'steep plot2'), '-dpng', '-r600')


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



%% x = [y0:yf; vy0:vyf; u10:u1f; u20:u2f; tf]
global m g
m = 1;
g = 9.81;


n_seg = 100;
n_knot = n_seg + 1;
n_times = 6; % decision variable times (tf, t1:t5)


%% linear constraints
Aeq = zeros(4,n_knot*4 + n_times);
beq = zeros(4,1);

% boundary conditions
Aeq(1,1) = 1; beq(1) = 0; % y0 = 0
Aeq(2,n_knot) = 1; beq(2) = 1; % yf = 1
Aeq(3, n_knot+1) = 1; beq(3) = 0; % vy0 = 0
Aeq(4, n_knot+n_knot) = 1; beq(4) = 0; % vyf = 0

% -tf <= 0
A1 = zeros(1,n_knot*4 + n_times);
A1(1,4*n_knot+1) = -1;
b1 = 0;
% -t1 <=0
A2 = zeros(1,n_knot*4 + n_times);
A2(1,4*n_knot+2) = -1;
b2 = 0;

% t1 - t2 <= 0
A3 = zeros(1,n_knot*4 + n_times);
A3(1,4*n_knot+2) = 1; 
A3(1,4*n_knot+3) = -1;
b3 = 0; 
% t2 - t3 <= 0
A4 = zeros(1,n_knot*4 + n_times);
A4(1,4*n_knot+3) = 1; 
A4(1,4*n_knot+4) = -1;
b4 = 0; 
% % t3 - t4 <= 0
% A5 = zeros(1,n_knot*4 + n_times);
% A5(1,4*n_knot+4) = 1; 
% A5(1,4*n_knot+5) = -1;
% b5 = 0; 
% t4 - t5 <= 0
A6 = zeros(1,n_knot*4 + n_times);
A6(1,4*n_knot+5) = 1; 
A6(1,4*n_knot+6) = -1;
b6 = 0; 

% u1 <= 25
A7 = zeros(n_knot,n_knot*4 + n_times);
A7(:,(2*n_knot+1):(3*n_knot)) = eye(n_knot);
b7 = 25*ones(n_knot,1);

% % -u2 <= 0
% A3 = zeros(n_knot,n_knot*4 + n_times);
% b3 = zeros(n_knot,1);
% A3(:,(3*n_knot+1):(4*n_knot)) = -eye(n_knot);

% u_max = 30;
% % u1 <= u_max
% A4 = zeros(n_knot,n_knot*4 + n_times);
% b4 = u_max*ones(n_knot,1);
% A4(:,(2*n_knot+1):(3*n_knot)) = eye(n_knot);
% 
% % u2 <= u_max
% A5 = zeros(n_knot,n_knot*4 + n_times);
% b5 = u_max*ones(n_knot,1);
% A5(:,(3*n_knot+1):(4*n_knot)) = eye(n_knot);


A = [A1;A2;A3;A4;A6;A7];
b = [b1;b2;b3;b4;b6;b7];


%% optimization
% initial guess
y_guess = (linspace(0,1,n_knot))'; % linearly increasing in t
vy_guess = ones(n_seg+1, 1);
u1_guess = ones(n_seg+1,1);
u2_guess = zeros(n_seg+1,1);
tf_guess = 1;
t1_guess = 0.1;
t2_guess = 0.2;
t3_guess = 0.3;
t4_guess = 0.4;
t5_guess = 0.5;

x0 = [y_guess; vy_guess; u1_guess; u2_guess; tf_guess;...
      t1_guess; t2_guess; t3_guess; t4_guess; t5_guess];

options = optimoptions('fmincon', 'Display', 'iter-detailed',...
    'MaxFunctionEvaluations', 5e3*n_knot,...
    'MaxIterations', 400);%,...
%     'StepTolerance', 1e-9);

[x,fval] = fmincon(@(x)objFun(x,n_seg,n_knot), x0, A,b, Aeq,beq, [],[],...
    @(x)constFun(x,n_seg,n_knot), options);


%% results
y_inf = x(1:n_knot);
vy = x((n_knot+1):(2*n_knot));
u1 = x((2*n_knot+1):(3*n_knot));
u2 = x((3*n_knot+1):(4*n_knot));
tf = x(4*n_knot+1);
t1 = x(4*n_knot+2);
t2 = x(4*n_knot+3);
t3 = x(4*n_knot+4);
t4 = x(4*n_knot+5);
t5 = x(4*n_knot+6);

t = linspace(0,tf,n_knot);


% simulate
sysDyn = @(t,x,u) ([x(2); (u(1) - u(2) - m*g)/m]);

t_sim = [];
x_sim = [];
for k = 1:n_seg
    t_span = [t(k) t(k+1)];
    x0 = [y_inf(k); vy(k)];    
    u = [u1(k); u2(k)];
    
    [t45,x45] = ode45(@(t,x) sysDyn(t,x,u), t_span, x0);
    t_sim = [t_sim; t45];
    x_sim = [x_sim; x45];
end



%% plot
linewidth = 1.5;

f1 = figure(1); clf
subplot(1,3,1)
hold on
grid on
plot(t, y_inf, 'LineWidth', linewidth)
% plot(t_sim, x_sim(:,1), '--')
xlabel('Time (s)')
ylabel('Position')

subplot(1,3,2)
hold on
grid on
plot(t, vy, 'LineWidth', linewidth)
% plot(t_sim, x_sim(:,2), '--')
xlabel('Time (s)')
ylabel('Velocity')

subplot(1,3,3)
hold on
grid on
plot(t, u1, 'LineWidth', linewidth)
% plot(t, u2, 'LineWidth', linewidth)
xlabel('Time (s)')
ylabel('Thrust (pressure)')
% legend('u_1', 'u_2')
ylim([0 30])

pub_figureFormat(f1, 'CMU Serif')
f1.Position = [-1100, 425, 1025, 375];



%% nonlinear constraints
function [c,ceq] = constFun(x, n_seg, n_knot)
    global m g

    y = x(1:n_knot);
    vy = x((n_knot+1):(2*n_knot));
    u1 = x((2*n_knot+1):(3*n_knot));
    u2 = x((3*n_knot+1):(4*n_knot));
    tf = x(4*n_knot+1);
    t1 = x(4*n_knot+2);
    t2 = x(4*n_knot+3);
    t3 = x(4*n_knot+4);
    t4 = x(4*n_knot+5);
    t5 = x(4*n_knot+6);
    
    dt = tf/n_seg;
    t = 0:dt:tf;

    

    
    
    % control constraints (pressure)
    alpha_log = 500;
    p_coeffs = [1 1 1 0]*17;
    alpha_exp = 4;
    
    u_max = 20;
    
    ceq_u1 =  zeros(n_knot,1);
    ceq_u2 =  zeros(n_knot,1);
    for k = 1:n_knot
        u_inf = logistic(t(k), 1, t1, alpha_log) - logistic(t(k), 1, t2, alpha_log);
        p_inf = polyval(p_coeffs,t(k)-t1);

        u_seal = logistic(t(k), 1, t2, alpha_log) - logistic(t(k), 1, t3, alpha_log);
        p_seal = polyval(p_coeffs,t2-t1);

        u_def = logistic(t(k), 1, t3, alpha_log);
        p_def = exp(-alpha_exp*(t(k)-t3))*p_seal;


        ceq_u1(k) = u1(k) - (u_inf*p_inf + u_seal*p_seal + u_def*p_def);
    
        ceq_u2(k) = u2(k) - (1/(1 + exp(-alpha_log*(t(k) - t4))) - 1/(1 + exp(-alpha_log*(t(k) - t5))))*u_max;
    end
    
    
    
%     % control constraint (step)
%     u_max = 20;
%     alpha = 1000;
%     ceq_u1 =  zeros(n_knot,1);
%     ceq_u2 =  zeros(n_knot,1);
%     for k = 1:n_knot
% %         y = 1./(1 + exp(-10*(t - 4)));
%         ceq_u1(k) = u1(k) - (1/(1 + exp(-alpha*(t(k) - t1))) - 1/(1 + exp(-alpha*(t(k) - t3))))*u_max;
%         ceq_u2(k) = u2(k) - (1/(1 + exp(-alpha*(t(k) - t2))) - 1/(1 + exp(-alpha*(t(k) - t4))))*u_max;
%     end

    
%     % binary control constraint
%     u_on = 1;
%     ceq_u1 = zeros(n_knot,1);
%     ceq_u2 = zeros(n_knot,1);
%     for k = 1:n_knot
%         ceq_u1(k) = u1(k)*(u_on - u1(k));
%         ceq_u2(k) = u2(k)*(u_on - u2(k));
%     end
    
%     % control slope constraint
%     slope_lim = 50;
%     c_ub = zeros(n_seg,1);
%     c_lb = zeros(n_seg,1);
%     for k = 1:n_seg
%         c_ub(k) = (u1(k+1) - u1(k))/(dt) - slope_lim;
%         c_lb(k) = -(u1(k+1) - u1(k))/(dt) - slope_lim;
%     end
%     c = [c_ub; c_lb];


%     % control smoothness constraint
%     slope_lim = 1;
%     c1_ub = zeros(n_seg*2-1,1);
%     c1_lb = zeros(n_seg*2-1,1);
%     c2_ub = zeros(n_seg*2-1,1);
%     c2_lb = zeros(n_seg*2-1,1);
%     for k = 2:n_seg
%         s1_k = (u1(k) - u1(k-1))/(dt);
%         s1_kp1 = (u1(k+1) - u1(k))/(dt);
%         c1_ub(k-1) = (s1_kp1 - s1_k) - slope_lim;
%         c1_lb(k-1) = -(s1_kp1 - s1_k) - slope_lim;
%         
%         s2_k = (u2(k) - u2(k-1))/(dt);
%         s2_kp1 = (u2(k+1) - u2(k))/(dt);
%         c2_ub(k-1) = (s2_kp1 - s2_k) - slope_lim;
%         c2_lb(k-1) = -(s2_kp1 - s2_k) - slope_lim;
%     end
%     c = [c1_ub; c1_lb; c2_ub; c2_lb];

    c = [];

    
    % dynamics constraints
    ceq_pos = zeros(n_seg,1);
    ceq_vel = zeros(n_seg,1);
    for k = 1:n_seg
        ceq_pos(k) = y(k+1) - y(k) - (1/2)*dt*(vy(k) + vy(k+1));
        ceq_vel(k) = vy(k+1) - vy(k) - (1/2)*dt*((u1(k) - u2(k) - m*g)/m + (u1(k+1) - u2(k+1) - m*g)/m);
    end
    
    ceq = [ceq_u1; ceq_u2; ceq_pos; ceq_vel];
end


%%
function cost = objFun(x, n_seg, n_knot)
    y = x(1:n_knot);
    vy = x((n_knot+1):(2*n_knot));
    u1 = x((2*n_knot+1):(3*n_knot));
    u2 = x((3*n_knot+1):(4*n_knot));
    tf = x(4*n_knot+1);
    t1 = x(4*n_knot+2);
    t2 = x(4*n_knot+3);
    t3 = x(4*n_knot+4);
    t4 = x(4*n_knot+5);

%     cost = sum(u1.*u1) + sum (u2.*u2);
    cost = tf;
%     cost = sum(u1.*u1) + sum (u2.*u2) + 1*tf^2;
%     cost = 0;
end


function f = logistic(x, L, x0, alpha)
    % logistic function: L = max value, x0 = midpoint, alpha = growth rate
    f = L./(1 + exp(-alpha*(x-x0)));
end




