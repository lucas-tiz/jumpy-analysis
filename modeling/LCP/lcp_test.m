%
clear, clc
addpath('LCPSolve')

lcp_analysis = 2;

% physical parameters
par.g = 9.81;
par.r = 0.5;
par.l = 1;
par.m = 1;
par.J = 1/12*par.m*par.l^2;
par.mu = 0.9; % friction coefficient

vid_export = 1;
vid_file = 'rod_1_contact';



%% Spatial model
robot.NB = 1 + 2; % extra 2 bodies for massless links
robot.parent = [0,1,2];
robot.jtype = {'px','py','r'};
robot.Xtree{1} = eye(3);
robot.Xtree{2} = eye(3);
robot.Xtree{3} = plnr( 0, [0 0]);
robot.gravity = [0;-9.81];

I = mcI(par.m, [0.5, 0], par.J);   
robot.I = {zeros(3), zeros(3), I};

robot.appearance.base = ...
  { 'box', [-0.2 -0.3 -0.2; 0.2 0.3 -0.07] };

for i = 1:robot.NB
  robot.appearance.body{i} = ...
    { 'box', [0 -0.07 -0.04; 1 0.07 0.04], ...
      'cyl', [0 0 -0.07; 0 0 0.07], 0.1 };
end



%%
dt = 0.001;
t_span = 0:dt:4;
n = length(t_span);

theta0 = deg2rad(60);
thetad0 = -1.5;

% velocity-based lcp
x0 = [0,3,theta0, par.r*sin(theta0)*thetad0,-par.r*cos(theta0)*thetad0,thetad0];
% x0 = [-0.5,1,theta0, 0,0,thetad0];
x_arr = zeros(n,6);
x_arr(1,:) = x0;
f_arr = zeros(n,3*2);

% position-based lcp
x_arr_pos = zeros(n,6);
x_arr_pos(1,:) = x0;
f_arr_pos = zeros(n,3*2);

% analytical (no  contact)
x0_an = [0,(2+par.r*sin(x0(3))),theta0, -par.r*sin(theta0)*thetad0,par.r*cos(theta0)*thetad0,thetad0];
x_an_arr = zeros(n,6);
x_an_arr(1,:) = x0_an;


%% Simulate
for i = 2:length(t_span)
    tau = zeros(3,1);
    

    %% velocity-based lcp
%     q = x_arr(i-1,1:3)';
%     qd = x_arr(i-1,4:6)';
% 
%     [x,f] = lcpVel2(robot, par, q, qd, tau, dt);
%     x_arr(i,:) = x;
%     f_arr(i,:) = f;
    
    
    %% position-based lcp
    q = x_arr_pos(i-1,1:3)';
    qd = x_arr_pos(i-1,4:6)';
%     [x,f] = lcpPos2(robot, par, q, qd, tau, dt);
    [x,f] = lcpPos4(robot, par, q, qd, tau, dt);
    x_arr_pos(i,:) = x;
    f_arr_pos(i,:) = f;
%     

    %% analytical (no contact)
    x_an = eomRod(x_an_arr(i-1,:),par);
    x_an_arr(i,:) = x_an_arr(i-1,:) + dt*x_an;
end
% [t,x] = ode45(@(t,x) eomRod(t,x,p), t_span, q0);


%% Plot
figure(433); clf
co = get(gca, 'ColorOrder');
% joint space positions
subplot(1,2,1)
title('Joint Space Positions')
hold on
grid on
% plot(t_span, x_an_arr(:,2) - par.r*sin(x_an_arr(:,3)), ':', 'LineWidth', 1.5)
plot(t_span, x_arr(:,2), 'Color', co(1,:))
plot(t_span, x_arr_pos(:,2), '--', 'Color', co(2,:), 'LineWidth', 1.5)
% plot(t_span, x_an_arr(:,3), ':', 'LineWidth', 1.5)
legend('y (LCP vel)', 'y (LCP pos)')
% joint space angle
subplot(1,2,2)
title('Joint Space Angle')
hold on
grid on
plot(t_span, x_arr(:,3), 'Color', co(1,:))
plot(t_span, x_arr_pos(:,3), '--', 'Color', co(2,:),'LineWidth', 1.5)
legend('LCP vel', 'LCP pos')
% % error
% subplot(2,1,2)
% hold on
% grid on
% % plot(t_span, (x_sp_arr(:,2) + p.r*sin(x_sp_arr(:,3))) - q_arr(:,3))
% plot(t_span, x_arr(:,2) - (x_an_arr(:,2)-par.r*sin(x_an_arr(:,3))))
% plot(t_span, x_arr(:,3) - x_an_arr(:,3), ':', 'LineWidth', 1.5)
% legend('y err', 'angle err')


fig_f = figure(434); clf
% normal force
subplot(2,1,1)
title('Normal Force')
ylim([-10 200])
hold on
grid on
plot(t_span, f_arr(:,1));
plot(t_span, f_arr_pos(:,1));
legend('LCP vel', 'LCP pos')
% tangent force
subplot(2,1,2)
title('Tangent Force')
ylim([-100 100])
hold on
grid on
% h_ft1 = plot(t_span, f_arr(:,2));
% h_ft2 = plot(t_span, f_arr(:,3));
plot(t_span, f_arr(:,2)-f_arr(:,3));
% plot(t_span, f_arr_pos(:,2)-f_arr_pos(:,3));
legend('LCP vel', 'LCP pos')


% % tip velocity
% figure(435); clf
% hold on
% grid on
%  plot(t_span, x_arr(:,4))
% plot(t_span, x_arr(:,5))
% vel2 = zeros(n,3);
% for i = 1:n
%     theta = x_arr(i,3);
%     J11 = [1 0 -par.l*sin(theta);
%            0 1 par.l*cos(theta);
%            0 0 1];
%     vel2(i,:) = J11*x_arr(i,1:3)';
% end
% 
% plot(t_span, vel2(:,1), '--', 'Color', co(1,:))
% plot(t_span, vel2(:,2), '--', 'Color', co(2,:))
% legend('x-vel', 'y-vel')


%% Animate
if vid_export == 1
    vid = VideoWriter(['anim/',vid_file],'MPEG-4');
    vid.Quality = 50;
    vid.FrameRate = 60;
    open(vid);
end

% showmotion(robot, t_span, x_sp_arr(:,1:3)')

fig_anim = figure(432); clf
% subplot(2,2,[1,3])
axis equal
ax_anim = gca;
hold on
grid on
xlim([-6 1])
ylim([-1 3])
plot([-10,10], [-0.01,-0.01], 'k', 'LineWidth', 1.5)

% plot trajectories
% plot(x_an_arr(:,1)-par.r*cos(x_an_arr(:,3)), x_an_arr(:,2)-par.r*sin(x_an_arr(:,3))) % tip 
% plot(x_an_arr(:,1), x_an_arr(:,2)) % center
% plot(x_arr(:,1), x_arr(:,2), '--', 'Color', co(1,:)) % tip
% plot(x_arr(:,1) + par.l*cos(x_arr(:,3)), x_arr(:,2) + par.l*sin(x_arr(:,3)),...
%     'Color', co(1,:)) % tip

% plot(x_arr_pos(:,1), x_arr_pos(:,2), '--', 'Color', co(2,:)) % tip
% plot(x_arr_pos(:,1) + par.l*cos(x_arr_pos(:,3)), x_arr_pos(:,2) + par.l*sin(x_arr_pos(:,3)),...
%     'Color', co(2,:)) % tip
% plot(1 + x_arr(:,1), x_arr(:,2), '--', 'Color', co(2,:)) % tip
% plot(1 + x_arr(:,1) + par.l*cos(x_arr(:,3)), x_arr(:,2) + par.l*sin(x_arr(:,3)),...
%     'Color', co(2,:)) % tip
% plot(x_arr(:,1)+par.r*cos(x_arr(:,3)) + 2, x_arr(:,2)+par.r*sin(x_arr(:,3))) % center

% subplot(2,2,2)
% title('Normal Force')
% ax_fn = gca;
% ylim([-10 200])
% hold on
% grid on
% % h_fn = plot(t_span, f_arr(:,1), 'Color', co(2,:));
% h_fn = plot(t_span, f_arr_pos(:,1), 'Color', co(2,:));
% plot(t_span, f_arr_pos(:,2), ':', 'Color', co(2,:));
% 
% subplot(2,2,4)
% title('Tangent Force')
% ax_ft = gca;
% ylim([-100 100])
% hold on
% grid on
% % h_ft = plot(t_span, f_arr(:,2)-f_arr(:,3), 'Color', co(2,:));
% h_ft = plot(t_span, f_arr_pos(:,3)-f_arr_pos(:,4), 'Color', co(2,:));
% plot(t_span, f_arr_pos(:,5)-f_arr_pos(:,6), ':', 'Color', co(2,:));


% % create rod (velocity-based LCP)
% h_rod_lcpv(1) = plot([0, par.l], [0,0], 'LineWidth', 1.5, 'Color', co(1,:));
% h_rod_lcpv(2) = plot(par.r, 0, '.', 'MarkerSize', 30, 'Color', co(1,:));
% obj_rod_lcpv = hgtransform('parent',ax_anim);
% set(h_rod_lcpv, 'parent', obj_rod_lcpv);

% create rod (position-based LCP)
h_rod_lcpp(1) = plot([0, par.l], [0,0], 'LineWidth', 2, 'Color', co(2,:));
% h_rod_lcpp(2) = plot(par.r, 0, '.', 'MarkerSize', 30, 'Color', co(2,:));
obj_rod_lcpp = hgtransform('parent',ax_anim);
set(h_rod_lcpp, 'parent', obj_rod_lcpp);

% create rod (analytical, no contact)
% h_rod(1) = plot([-par.r, par.r], [0,0], 'LineWidth', 1.5, 'Color', 'k');
% h_rod(2) = plot(0, 0, '.', 'MarkerSize', 30, 'Color', 'k');
% obj_rod = hgtransform('parent',ax_anim);
% set(h_rod, 'parent', obj_rod);

% % create plot points
% p_fn = plot(ax_fn,0,0,'.', 'Color', h_fn.Color, 'MarkerSize', 20);
% p_fn_obj = hgtransform('parent',ax_fn);
% set(p_fn,'parent',p_fn_obj);
% p_ft = plot(ax_fn,0,0,'.', 'Color', h_ft.Color, 'MarkerSize', 20);
% p_ft_obj = hgtransform('parent',ax_ft);
% set(p_ft,'parent',p_ft_obj);





hTime = annotation('textbox',[0.01 0.01 0.06 0.03],'String',0,'Color',[0,0,0]);
tic
inc = max(1, round(1/dt/100));
for i = 1:inc:n
    while toc < t_span(i); continue; end % loop until at time
    
    set(hTime,'string',sprintf('%0.2f',t_span(i))) % update time display
    
%     % move rod (velocity-based LCP)
%     T_rod_lcpv = makehgtform('zrotate',x_arr(i,3));
%     T_rod_lcpv(1,4) = (x_arr(i,1) ); % adjust x-location
%     T_rod_lcpv(2,4) = x_arr(i,2); % adjust y-location
%     set(obj_rod_lcpv,'matrix',T_rod_lcpv);
    
    % move rod (position-based LCP)
    T_rod_lcpp = makehgtform('zrotate',x_arr_pos(i,3));
    T_rod_lcpp(1,4) = (x_arr_pos(i,1)); % adjust x-location
    T_rod_lcpp(2,4) = x_arr_pos(i,2); % adjust y-location
    set(obj_rod_lcpp,'matrix',T_rod_lcpp);
    
%     % move rod (baseline)
%     T_rod = makehgtform('zrotate',x_an_arr(i,3));
%     T_rod(1,4) = x_an_arr(i,1) + par.r*cos(theta0) + 2; % adjust x-location
%     T_rod(2,4) = x_an_arr(i,2); % adjust y-location
%     set(obj_rod,'matrix',T_rod);
%     
%     % move plot points
%     T_p_fn = makehgtform('translate',[t_span(i),f_arr_pos(i,1),0]);
%     set(p_fn_obj,'matrix',T_p_fn);
%     T_p_ft = makehgtform('translate',[t_span(i),f_arr_pos(i,3)-f_arr_pos(i,4),0]);
%     set(p_ft_obj,'matrix',T_p_ft);


    drawnow
    if vid_export == 1
        writeVideo(vid, getframe(fig_anim));
    end
    pause(0.02)
end

if vid_export == 1
    close(vid);
end


%% Dynamics
function qd = eomRod(q,p)

    x = q(1);
    y = q(2);
    theta = q(3);
    xd = q(4);
    yd = q(5);
    thetad = q(6);
    
    qd = zeros(1,6);
    qd(1) = xd;
    qd(2) = yd;
    qd(3) = thetad;
    qd(4) = 0;
    qd(5) = -p.g + 0;%fn/p.m;
    qd(6) = 0;%fn*(l/2)*sin(theta)/p.J;

%     qd = zeros(1,6);
%     qd(1) - xd = 0;
%     qd(2) = 0;
%     qd(3) - yd = 0;
%     qd(4) - -p.g = 0;%fn/p.m;
%     qd(5) - thetad = 0;
%     qd(6) = 0;%fn*(l/2)*sin(theta)/p.J;

end


function [x,f] = lcpVel(robot, par, q, qd, tau, dt)
    % velocity-based LCP
    [H, C] = HandC(robot, q, qd); % H*qdd+C=tau 
    M = H; % mass
    tau_star = M*qd - dt*(C - tau);

%     J11 = [1 0 0; % Jacobian for single contact point
%            0 1 0;
%            0 0 0];
    theta = q(3);
    J11 = [1 0 -par.l*sin(theta);
           0 1 par.l*cos(theta);
           0 0 1];
    n1 = [0 1 0]'; % normal force direction at contact point
    N = J11'*n1;

    D1 = [1 0 0;
          -1 0 0]';
    B = J11'*D1;

    m = 3; % DOF of system
    p = 1; % number of contacts
    d = 2; % number of friction cone bases
    E = eye(p*d,p); % binary matrix

    A = [dt*N'*inv(M)*N, dt*N'*inv(M)*B, 0;
         dt*B'*inv(M)*N, dt*B'*inv(M)*B, E;
         par.mu, -E', 0];
    q_lcp = [N'*inv(M)*tau_star;
             B'*inv(M)*tau_star;
             0];

    [w,z] = LCPSolve(A,q_lcp); % w = A*z + q
    fn = z(1);
    fd = z(2:3);

    qd_next = inv(M)* (M*qd - dt*(C - tau) + dt*(J11'*fn*n1 + J11'*D1*fd));  
%     qd_next = qd_next - N'*qd_next;
    q_next = q' + dt*qd_next'; % Euler integration

%     y_next = q_next(2);
    y_next = q_next(2) + par.l*sin(q_next(3));
    if y_next > 0 % if above boundary, don't use LCP
%         fprintf('above\n')
        qdd = FDab(robot, q, qd, tau);        
        xd = [qd', qdd'];
        x = [q',qd'] + dt*xd;
        f = zeros(1,3);
    else
%         fprintf('0\n')
        x = [q_next, qd_next'];
        f = [fn, fd'];
    end

end


function [x,f] = lcpVel2(robot, par, q, qd, tau, dt)
    % velocity-based LCP

%     q_next_est = q + dt*qd;
    
    q_next_est = q + dt*qd;
    qd_next_est = qd;
    
    
%     y11 = q_next_est(2);
%     y12 = q_next_est(2) + par.l*sin(q_next_est(3));
%     p11 = 0;
%     p12 = 0;
%     if y11 <=0
%         p11 = 1;
%     end
%     if y12 <= 0
%         p12 = 1;
%     end
%     
    i = 0;
    err = 1;
    while err > 1e-9
        i = i + 1;
        
        [M, C] = HandC(robot, q, qd); % M*qdd+C=tau 
        tau_star = M*qd - dt*(C - tau);

        
        y11 = q_next_est(2);
        y12 = q_next_est(2) + par.l*sin(q_next_est(3));
        p11 = 0;
        p12 = 0;
        if y11 <=0
            p11 = 1;
        end
        if y12 <= 0
            p12 = 1;
        end
        
        J11 = [1 0 0; % Jacobian for single contact point
               0 1 0;
               0 0 1];
        theta = q_next_est(3);
        J12 = [1 0 -par.l*sin(theta);
              0 1 par.l*cos(theta);
              0 0 1];
        
        n1 = [0 1 0]'; % normal force direction at contact point

%         N = J11'*n1;
        N = [p11*J11'*n1, p12*J12'*n1];
%         N = [p12*J12'*n1];


        D1 = [1 0 0;
              -1 0 0]';
          
        B = [p11*J11'*D1, p12*J12'*D1];
%         B = [p12*J12'*D1];

        m = 3; % DOF of system
        p = 2; % number of contacts
        d = 2; % number of friction cone bases
        E = ones(d*p,1); %eye(p*d,p); % binary matrix

        A = [dt*N'*inv(M)*N, dt*N'*inv(M)*B, zeros(2,1);
             dt*B'*inv(M)*N, dt*B'*inv(M)*B, E;
             par.mu*ones(1,p), -E', 0];
        q_lcp = [N'*inv(M)*tau_star;
                 B'*inv(M)*tau_star;
                 0];

        [w,z] = LCPSolve(A,q_lcp); % w = A*z + q
        fn = z(1:2);
        fd = z(3:6);

        qd_next = inv(M)* (M*qd - dt*(C - tau) + dt*(N*fn + B*fd));  
        q_next = q + dt*qd_next; % Euler integration
        
        err = sum(abs(q_next - q_next_est));
        qd_next_est = qd_next;
        q_next_est = q_next;
        
        if i == 10e2
            fprintf('max reached, err: %0.6f \n', err)
            break
        end
    end
%     if i > 1
%         fprintf('%i\n', i)
%     end

% %     y_next = q_next(2);
%     y_next = q_next(2) + par.l*sin(q_next(3));
%     if y_next > 0 % if above boundary, don't use LCP
% %         fprintf('above\n')
%         qdd = FDab(robot, q, qd, tau);        
%         xd = [qd', qdd'];
%         x = [q',qd'] + dt*xd;
%         f = zeros(1,3);
%     else
% %         fprintf('0\n')
%         x = [q_next, qd_next'];
%         f = [fn, fd'];
%     end
    x = [q_next', qd_next'];
    f = [fn', fd'];

end


function [x,f] = lcpPos(robot, par, q, qd, tau, dt)
    % position-based LCP
    [H, C] = HandC(robot, q, qd); % H*qdd+C=tau 
    M = H; % mass
    tau_star = M*qd - dt*(C - tau);

    m = 3; % DOF of system
    p = 1; % number of contacts
    d = 2; % number of friction cone bases
    
    J11 = [1 0 0; % Jacobian for single contact point
           0 1 0;
           0 0 1];
%     theta = x_arr2(i-1,3);
%     J11 = [1 0 -par.l*sin(theta);
%           0 1 par.l*cos(theta);
%           0 0 1];
    n1 = [0 1 0]'; % normal force direction at contact point
    N = J11'*n1;

    D1 = [1 0 0;
          -1 0 0]';
    B = J11'*D1;

    E = ones(d,1); %eye(p*d,p); % binary matrix

%     A11 = dt^2*N'*inv(M)*N;
%     A12 = dt^2*N'*inv(M)*B;
%     A13 = 0;
%     A21 = dt*B'*inv(M)*N;
%     A22 = dt*B'*inv(M)*B;
%     A23 = E;
%     A31 = par.mu;
%     A32 = -E';
%     A33 = 0;

    A = [dt^2*N'*inv(M)*N, dt^2*N'*inv(M)*B, 0;
         dt*B'*inv(M)*N, dt*B'*inv(M)*B, E;
         par.mu, -E', 0];
    q_lcp = [N'*q + dt*N'*inv(M)*tau_star;
         B'*inv(M)*tau_star;
         0];

    [w,z] = LCPSolve(A,q_lcp); % w = A*z + q
    fn = z(1);
    fd = z(2:3);

    qd_next = inv(M)* (M*qd - dt*(C - tau) + dt*(N*fn + B*fd));  
    q_next = q' + dt*qd_next'; % Euler integration
    
    x = [q_next, qd_next'];
    f = [fn, fd'];
end


function [x,f] = lcpPos2(robot, par, q, qd, tau, dt)
    % position-based LCP
    [H, C] = HandC(robot, q, qd); % H*qdd+C=tau 
    M = H; % mass
    tau_star = M*qd - dt*(C - tau);

    m = 3; % DOF of system
    p = 1; % number of contacts
    d = 2; % number of friction cone bases
    

    

    q_next_est = q + dt*qd;
    err = 1;
    i = 0;
%     for i = 1:1
    while err > 1e-9
        i = i + 1;
        
%         theta = q_next_est(3);

        J11 = [1 0 0; % Jacobian for single contact point
               0 1 0;
               0 0 1];
%         J11 = [1 0 -par.l*sin(theta);
%               0 1 par.l*cos(theta);
%               0 0 1];
        n1 = [0 1 0]'; % normal force direction at contact point
        N = J11'*n1;

        D1 = [1 0 0;
              -1 0 0]';
        B = J11'*D1;

        E = ones(d,1); %eye(p*d,p); % binary matrix 

        A11 = 0;
        A12 = zeros(1,2);
        A13 = 0;
        A21 = dt*B'*(M\N);
        A22 = dt*B'*(M\B);
        A23 = E;
        A31 = par.mu;
        A32 = -E';
        A33 = 0;
        
        A = [A11, A12, A13;
             A21, A22, A23;
             A31, A32, A33];
         
        q_lcp = [q_next_est(2) + par.l*sin(q_next_est(3));
                 B'*(M\tau_star) - 0.0;
                 0];

%         A = [dt^2*N'*(M\N), dt^2*N'*(M\B), 0;
%              dt*B'*(M\N), dt*B'*(M\B), E;
%              par.mu, -E', 0];
%         q_lcp = [N'*q + dt*N'*(M\tau_star);
%              B'*(M\tau_star) - 0.0;
%              0];

        [w,z] = LCPSolve(A,q_lcp); % w = A*z + q
        fn = z(1);
        fd = z(2:3);

        qd_next = M\(M*qd - dt*(C - tau) + dt*(J11'*fn*n1 + J11'*D1*fd));
        q_next = q + dt*qd_next; % Euler integration
        
%         fprintf('q_next_est: %3.6f %3.6f %3.6f\n', q_next_est)
%         fprintf('q_next    : %3.6f %3.6f %3.6f\n', q_next)
%         fprintf('err: %3.6f %3.6f %3.6f\n', q_next-q_next_est)

        err = sum(abs(q_next - q_next_est));
%         fprintf('%i\n', i)
        
        q_next_est = q_next;
        
        if i == 10e2
            fprintf('max reached \n')
            break
        end
    end
%     fprintf('iter\n')
%     fprintf('\n')

%     y_next = q_next(2) + par.l*sin(q_next(3)) %DEBUG
    
    x = [q_next', qd_next'];
    f = [fn, fd'];
end


function [x,f] = lcpPos3(robot, par, q, qd, tau, dt)
    % position-based LCP
%     [H, C] = HandC(robot, q, qd); % H*qdd+C=tau 
%     M = H; % mass
%     tau_star = M*qd - dt*(C - tau);

    m = 3; % DOF of system
    p = 1; % number of contacts
    d = 2; % number of friction cone bases
    
    
    q_next_est = q + dt*qd;
%     qd_next = qd;
    err = 1;
    i = 0;
    while err > 1e-6
        i = i + 1;
        
        [M, C] = HandC(robot, q_next_est, qd); % M*qdd+C=tau 
        tau_star = M*qd - dt*(C - tau);

    
%         J11 = [1 0 0; % Jacobian for single contact point
%                0 1 0;
%                0 0 1];
        theta = q_next_est(3);
        J11 = [1 0 -par.l*sin(theta);
              0 1 par.l*cos(theta);
              0 0 1];
        n1 = [0 1 0]'; % normal force direction at contact point
        N = J11'*n1;

        D1 = [1 0 0;
              -1 0 0]';
        B = J11'*D1;

        E = ones(d,1); %eye(p*d,p); % binary matrix

    %     A11 = dt^2*N'*inv(M)*N;
    %     A12 = dt^2*N'*inv(M)*B;
    %     A13 = 0;
    %     A21 = dt*B'*inv(M)*N;
    %     A22 = dt*B'*inv(M)*B;
    %     A23 = E;
    %     A31 = par.mu;
    %     A32 = -E';
    %     A33 = 0;

        alpha0 = -par.l*sin(theta) + theta*par.l*cos(theta);
    
        A = [dt^2*N'*inv(M)*N, dt^2*N'*inv(M)*B, 0;
             dt*B'*inv(M)*N, dt*B'*inv(M)*B, E;
             par.mu, -E', 0];
        q_lcp = [N'*q + dt*N'*inv(M)*tau_star - alpha0;
                 B'*inv(M)*tau_star;
                 0];
%         A = [N'*inv(M)*N, N'*inv(M)*B, 0;
%              dt*B'*inv(M)*N, dt*B'*inv(M)*B, E;
%              par.mu, -E', 0];
%         q_lcp = [(N'*q + dt*N'*inv(M)*tau_star)/(dt^2);
%                  B'*inv(M)*tau_star;
%                  0];

        [w,z] = LCPSolve(A,q_lcp); % w = A*z + q
        fn = z(1);
        fd = z(2:3);

%         qd_next = inv(M)*(M*qd - dt*(C - tau) + dt*(N*fn + B*fd));  
        qd_next = qd + M\(-dt*(C - tau) + dt*(N*fn + B*fd));          
        q_next = q + dt*qd_next; % Euler integration
        
        err = sum(abs(q_next - q_next_est));
        q_next_est = q_next;
        
        
%         if i > 2
%             continue
%         end
%         
        if i == 10e2
            fprintf('max reached, err: %4.3f \n', err)
            break
        end
    end
    if i > 1
        fprintf('%i\n', i)
    end

    
    x = [q_next', qd_next'];
    f = [fn', fd'];
end


function [x,f] = lcpPos4(robot, par, q, qd, tau, dt)
    % position-based LCP

    m = 3; % DOF of system
    p = 2; % number of contacts
    d = 2; % number of friction cone bases    
    
    q_next_est = q + dt*qd;
    err = 1;
    iter = 0;
    while err > 1e-6
        iter = iter + 1;
        
        x = q_next_est(1);
        y = q_next_est(2);
        theta = q_next_est(3);
        
        [M, C] = HandC(robot, q_next_est, qd); % M*qdd+C=tau 
        tau_star = M*qd - dt*(C - tau);

    
        J11 = [1 0 0; % Jacobian for foot contact point
               0 1 0;
               0 0 1];
        J12 = [1 0 -par.l*sin(theta); % Jacobian for end of rod contact point
               0 1 par.l*cos(theta);
               0 0 1];
           
        n1 = [0 1 0]'; % normal force direction at contact point(s)
        N = [J11'*n1, J12'*n1*0];

        D1 = [1 0 0; % friction cone bases at contact point(s)
              -1 0 0]';
        B = [J11'*D1, J12'*D1*0];

        e = ones(d,1);
        E = zeros(p*d,p);
        row = 1;
        for col = 1:p
            E(row:row+d-1,col) = e;
            row = row + d;
        end
        
%         E = ones(d,1); %eye(p*d,p); % binary matrix

    %     A11 = dt^2*N'*inv(M)*N;
    %     A12 = dt^2*N'*inv(M)*B;
    %     A13 = 0;
    %     A21 = dt*B'*inv(M)*N;
    %     A22 = dt*B'*inv(M)*B;
    %     A23 = E;
    %     A31 = par.mu;
    %     A32 = -E';
    %     A33 = 0;

%         alpha0 = -par.l*sin(theta) + theta*par.l*cos(theta);
        f = [y; 0];
%              y + par.l*sin(theta)]; % contact constraint
        alpha0 = N'*q_next_est - f;
%         fprintf('alpha err: %0.9f\n', alpha0-alpha0_alt)
        
    
        A = [dt^2*N'*inv(M)*N, dt^2*N'*inv(M)*B, zeros(p,p);
             dt*B'*inv(M)*N, dt*B'*inv(M)*B, E;
             par.mu*eye(p), -E', zeros(p,p)];
        q_lcp = [N'*q + dt*N'*inv(M)*tau_star - alpha0;
                 B'*inv(M)*tau_star;
                 zeros(p,1)];


        [w,z] = LCPSolve(A,q_lcp); % w = A*z + q
        fn = z(1:p);
        fd = z(p+1:p+p*2);

%         qd_next = inv(M)*(M*qd - dt*(C - tau) + dt*(N*fn + B*fd));  
        qd_next = qd + M\(-dt*(C - tau) + dt*(N*fn + B*fd));          
        q_next = q + dt*qd_next; % Euler integration
        
        err = sum(abs(q_next - q_next_est));
        q_next_est = q_next;
        
        if iter == 10e2
            fprintf('max reached, err: %4.3f \n', err)
            break
        end
    end
%     if iter > 1
%         fprintf('%i\n', iter)
%     end

    
    x = [q_next', qd_next'];
    f = [fn', fd'];
end