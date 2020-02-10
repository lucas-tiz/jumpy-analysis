%
clear, clc
%%


tf = 1;

n_seg = 3;
n_knot = n_seg+1;
h = tf/n_seg;


t_col = 0:h:tf; % time collocation vector

dt_fine = h/10;
t_fine = 0:dt_fine:tf; % fine time vector for trajectory interpolation


%%
% x = [p0;p1;p2;p3;p4;p5; v0;v1;v2;v3;v4;v5; u0;u1;u2;u3;u4;u5]

% initial guess
p0 = (0:h:tf)'; % x(t) = t
v0 = ones(n_seg+1, 1);
u0 = zeros(n_seg+1,1);
y0 = [p0; v0; u0];


%%
A1 = zeros(4,n_knot*3);
A1(1,1) = 1; % p0
A1(2,n_knot) = 1; % pN
A1(3, n_knot+1) = 1; % v0
A1(4, n_knot+n_knot) = 1; % vN

% A2 = zeros(4,n_knot*3);


A21 = -1*eye(n_seg, n_knot) + [zeros(n_seg,1), 1*eye(n_seg, n_seg)];
A22 = (-1/2*h)*eye(n_seg, n_knot) +...
    [zeros(n_seg,1), (-1/2*h)*eye(n_seg, n_seg)];
A23 = zeros(n_seg, n_knot);
A2 = [A21, A22, A23];

A31 = zeros(n_seg, n_knot);
A32 = -1*eye(n_seg, n_knot) + [zeros(n_seg,1), 1*eye(n_seg, n_seg)];
A33 = (-1/2*h)*eye(n_seg, n_knot) +...
    [zeros(n_seg,1), (-1/2*h)*eye(n_seg, n_seg)];
A3 = [A31, A32, A33];

Aeq = [A1; A2; A3];

% sum(sum(abs(A1-Aeq(1:4,:))))
% sum(sum(abs(A2-Aeq(5:9,:))))
% sum(sum(abs(A-Aeq)))


%%

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
   
beq = [0; 1; 0; 0;  zeros(2*n_seg,1)];



[y,fval] = fmincon(@(y)objFun(y,n_seg,h), y0, [],[], Aeq,beq);


%%
p = y(1:(n_seg+1));
v = y((n_seg+1 + 1):(n_seg+1 + 1 + n_seg));
u = y((2*(n_seg+1) + 1):(2*(n_seg+1) + 1 + n_seg));


% u_interp(1) = u(1);
% for k = 1:(length(t_col) - 1)
%     t = (t_col(k)+dt_fine):dt_fine:(t_col(k+1));
%     tao = t - t_col(k);
%     u_interp_spline = u(k) + tao/h*(u(k+1) - u(k));
%     u_interp = [u_interp, u_interp_spline];
% end

x_interp = [p(1); v(1)];
for k = 1:(length(t_col) - 1)
    t = (t_col(k)+dt_fine):dt_fine:(t_col(k+1));
    tau = t - t_col(k);
    
    f_k = [v(k); u(k)];
    f_kp1 = [v(k+1); u(k+1)];
    x_interp_spline = [p(k); v(k)] + f_k.*tau + (tau.^2)./(2*h).*(f_kp1 - f_k);
    
    x_interp = [x_interp, x_interp_spline];
end
    
linewidth = 1.5;

figure(1); clf
subplot(1,3,1)
hold on
grid on
plot(t_col, p, 'LineWidth', linewidth)
plot(t_fine, 3*t_fine.^2 - 2*t_fine.^3, '--', 'LineWidth', linewidth)
plot(t_fine, x_interp(1,:))
xlabel('Time (s)')
ylabel('Position')

subplot(1,3,2)
hold on
grid on
plot(t_col, v, 'LineWidth', linewidth)
plot(t_fine, 6*t_fine - 6*t_fine.^2, '--', 'LineWidth', linewidth)
plot(t_fine, x_interp(2,:))
xlabel('Time (s)')
ylabel('Velocity')

subplot(1,3,3)
hold on
grid on
plot(t_col, u, 'LineWidth', linewidth)
plot(t_fine, 6 - 12*t_fine, '--', 'LineWidth', linewidth)
% plot(t_fine, u_interp, 'LineWidth', linewidth)
xlabel('Time (s)')
ylabel('Input')


%%
function cost = objFun(y,n_seg,h)
    n_knot = n_seg+1;
    u = y(n_knot*2+1:end);

    cost = 0;
    for i = 1:n_seg
       cost = cost + 1/2*h*(u(i)^2 + u(i+1)^2);
        
    end
end






