%
clear, clc
%%


tf = 1; % final time

n_seg = 5;
n_knot = n_seg+1;
h = tf/n_seg;


t_col = 0:h:tf; % time collocation vector

dt_fine = h/10;
t_fine = 0:dt_fine:tf; % fine time vector for trajectory interpolation


%% Initial guess
% x = [p0;p1;p2;p3;p4;p5; v0;v1;v2;v3;v4;v5; u0;u1;u2;u3;u4;u5]

p0 = (0:h:tf)'; % x(t) = t
v0 = ones(n_seg+1, 1);
u0 = zeros(n_seg+1,1);
% u0 = [h; tf-h];
% u0 = [0.3; 0.7];
y0 = [p0; v0; u0];


%% Equality constraints

% % initial and final constraints
% A1 = zeros(4,n_knot*2 + 2);
% A1(1,1) = 1; % p0
% A1(2,n_knot) = 1; % pN
% A1(3, n_knot+1) = 1; % v0
% A1(4, n_knot+n_knot) = 1; % vN
% 
% % dynamics constraints: position trapezoidal quadrature
% A21 = -1*eye(n_seg, n_knot) + [zeros(n_seg,1), 1*eye(n_seg, n_seg)];
% A22 = (-1/2*h)*eye(n_seg, n_knot) +...
%     [zeros(n_seg,1), (-1/2*h)*eye(n_seg, n_seg)];
% A23 = zeros(n_seg, 2);
% A2 = [A21, A22, A23];


% initial and final constraints
A1 = zeros(4,n_knot*3);
A1(1,1) = 1; % p0
A1(2,n_knot) = 1; % pN
A1(3, n_knot+1) = 1; % v0
A1(4, n_knot+n_knot) = 1; % vN

% dynamics constraints: position trapezoidal quadrature
A21 = -1*eye(n_seg, n_knot) + [zeros(n_seg,1), 1*eye(n_seg, n_seg)];
A22 = (-1/2*h)*eye(n_seg, n_knot) +...
    [zeros(n_seg,1), (-1/2*h)*eye(n_seg, n_seg)];
A23 = zeros(n_seg, n_knot);
A2 = [A21, A22, A23];


% dynamics constraints: velocity trapezoidal quadrature
% A31 = zeros(n_seg, n_knot);
% A32 = -1*eye(n_seg, n_knot) + [zeros(n_seg,1), 1*eye(n_seg, n_seg)];
% A33 = (-1/2*h)*eye(n_seg, n_knot) +...
%     [zeros(n_seg,1), (-1/2*h)*eye(n_seg, n_seg)];
% A3 = [A31, A32, A33];

% Aeq = [A1; A2; A3];
Aeq = [A1; A2];

% beq = [0; 1; 0; 0;  zeros(2*n_seg,1)];
beq = [0; 1; 0; 0;  zeros(n_seg,1)];


% equality constraints: Aeq*x = beq
% Aeq = [1, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0; % p0 = 0
%        0, 0, 0, 0, 0, 1,  0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0; % pN = 1
%        0, 0, 0, 0, 0, 0,  1, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0; % v0 = 0
%        0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 1,  0, 0, 0, 0, 0, 0; % vN = 0
%        
%        -1, 1, 0, 0, 0, 0,  -1/2*h, -1/2*h, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0; % dynamics: pos trap. integ.
%        0, -1, 1, 0, 0, 0,  0, -1/2*h, -1/2*h, 0, 0, 0,  0, 0, 0, 0, 0, 0;
%        0, 0, -1, 1, 0, 0,  0, 0, -1/2*h, -1/2*h, 0, 0,  0, 0, 0, 0, 0, 0;
%        0, 0, 0, -1, 1, 0,  0, 0, 0, -1/2*h, -1/2*h, 0,  0, 0, 0, 0, 0, 0;
%        0, 0, 0, 0, -1, 1,  0, 0, 0, 0, -1/2*h, -1/2*h,  0, 0, 0, 0, 0, 0;
% 
%        0, 0, 0, 0, 0, 0,  -1, 1, 0, 0, 0, 0,  -1/2*h, -1/2*h, 0, 0, 0, 0; % dynamics: vel trap. integ.
%        0, 0, 0, 0, 0, 0,  0, -1, 1, 0, 0, 0,  0, -1/2*h, -1/2*h, 0, 0, 0;
%        0, 0, 0, 0, 0, 0,  0, 0, -1, 1, 0, 0,  0, 0, -1/2*h, -1/2*h, 0, 0;
%        0, 0, 0, 0, 0, 0,  0, 0, 0, -1, 1, 0,  0, 0, 0, -1/2*h, -1/2*h, 0;
%        0, 0, 0, 0, 0, 0,  0, 0, 0, 0, -1, 1,  0, 0, 0, 0, -1/2*h, -1/2*h];


%% Inequality constraints
% control time constraints
% A = [zeros(1, n_knot*2),  1   0;
%      zeros(1, n_knot*2),  0,  1;
%      zeros(1, n_knot*2), -1,  0;
%      zeros(1, n_knot*2),  0, -1;];
% b = [1; 1; 0; 0];

A = [zeros(n_knot, n_knot*2), eye(n_knot);
     zeros(n_knot, n_knot*2), -eye(n_knot)];
b = [5*ones(n_knot,1); zeros(n_knot,1)+5];


%% Optimization
options = optimoptions('fmincon', 'Display', 'iter',...
    'MaxFunctionEvaluations', 1e5,...
    'MaxIterations', 5000);

[y,fval] = fmincon(@(y)objFun(y,n_seg,h,tf), y0, A,b, Aeq,beq, [],[],...
    @(y)constFun(y,n_seg,n_knot,h), options);


%%
p = y(1:(n_seg+1));
v = y((n_seg+1 + 1):(n_seg+1 + 1 + n_seg));
u = y((2*(n_seg+1) + 1):(2*(n_seg+1) + 1 + n_seg));
% u = y((end-1):end);

% u_interp(1) = u(1);
% for k = 1:(length(t_col) - 1)
%     t = (t_col(k)+dt_fine):dt_fine:(t_col(k+1));
%     tao = t - t_col(k);
%     u_interp_spline = u(k) + tao/h*(u(k+1) - u(k));
%     u_interp = [u_interp, u_interp_spline];
% end

% x_interp = [p(1); v(1)];
% for k = 1:(length(t_col) - 1)
%     t = (t_col(k)+dt_fine):dt_fine:(t_col(k+1));
%     tau = t - t_col(k);
%     
%     f_k = [v(k); u(k)];
%     f_kp1 = [v(k+1); u(k+1)];
%     x_interp_spline = [p(k); v(k)] + f_k.*tau + (tau.^2)./(2*h).*(f_kp1 - f_k);
%     
%     x_interp = [x_interp, x_interp_spline];
% end
    
linewidth = 1.5;

figure(1); clf
subplot(1,3,1)
hold on
grid on
plot(t_col, p, 'LineWidth', linewidth)
plot(t_fine, 3*t_fine.^2 - 2*t_fine.^3, '--', 'LineWidth', linewidth)
% plot(t_fine, x_interp(1,:))
xlabel('Time (s)')
ylabel('Position')
legend('opt','analytic')

subplot(1,3,2)
hold on
grid on
plot(t_col, v, 'LineWidth', linewidth)
plot(t_fine, 6*t_fine - 6*t_fine.^2, '--', 'LineWidth', linewidth)
% plot(t_fine, x_interp(2,:))
xlabel('Time (s)')
ylabel('Velocity')
legend('opt','analytic')

subplot(1,3,3)
hold on
grid on
% plot([0; u; tf], [1; 0; 0; -1], 'LineWidth', linewidth)
plot(t_col, u, 'LineWidth', linewidth)
% plot(t_fine, 6 - 12*t_fine, '--', 'LineWidth', linewidth)
% plot(t_fine, u_interp, 'LineWidth', linewidth)
xlabel('Time (s)')
ylabel('Input')


%%
t1 = 0.4;
t2 = 0.8;

y0 = zeros(n_knot,1);
y0(1) = 1;

fun = @(x)0;

options = optimoptions('fmincon', 'Display', 'iter-detailed',...
    'MaxFunctionEvaluations', 100*n_knot,...
    'MaxIterations', 400,...
    'StepTolerance', 1e-9);

[y,fval] = fmincon(fun, y0, [],[], [],[], [],[],...
    @(y)ctrlConstFun(y,t1,t2,n_seg,n_knot,h), options);


%%
function [c,ceq] = ctrlConstFun(y,t1,t2,n_seg,n_knot,h)

    u = y;
    t = (0:h:(h*n_seg))';
    
    c = (1/2 - u).*(t1 - t);
%          (1/2 - u
    ceq = u.*(u - 1);
%     ceq = [];
end


function [c,ceq] = constFun(y,n_seg,n_knot,h)

    % control 



    c = [];

%     p = y(1:n_knot);
    v = y((n_knot+1):(2*n_knot));
    u = y((2*n_knot+1):end);

%     t1= y(end-1);
%     t2 = y(end);
%     
%     u = zeros(1,n_knot);
%     for k = 1:n_knot
%     end
    
    ceq = zeros(1,n_seg);
    for k = 1:n_seg
       ceq(k) = v(k+1) - v(k) - 1/2*h*(u(k+1) + u(k));
    end

end


function cost = objFun(y,n_seg,h,tf)
    n_knot = n_seg+1;
%     u = y(n_knot*2+1:end);

%     cost = 0;
%     for i = 1:n_seg
%        cost = cost + 1/2*h*(u(i)^2 + u(i+1)^2);
%     end
    
    
%     p = y(1:n_knot);
    v = y((n_knot+1):(2*n_knot));
    t1= y(end-1);
    t2 = y(end);
    
%     cost = 0;
%     for k = 1:n_knot
%         t = (k-1)*h;
%         
%         if t <= t1
%             cost_inc = h*v(k);
%         elseif t >= t2
%             cost_inc = h*v(k);
%         else
%             cost_inc = 0;
%         end
%         
%         cost = cost + cost_inc;
%     end    

%     cost = t1^2 + (tf-t2)^2;
%     cost = t1 + (tf-t2);
%     cost = 0;
    
    
    
    n_knot = n_seg+1;
    u = y(n_knot*2+1:end);

    cost = 0;
    for i = 1:n_seg
       cost = cost + 1/2*h*(u(i)^2 + u(i+1)^2);
    end
end






