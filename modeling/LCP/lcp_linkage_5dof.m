%
clear, clc
addpath('LCPSolve')
addpath('C:\Users\Lucas\Dropbox (GaTech)\Research\Hexapod\analysis\rbd_algorithms')

% physical parameters
% par.g = 9.81;
% par.r = 0.5;
% par.l = 1;
% par.m = 1;
% par.J = 1/12*par.m*par.l^2;
par.mu = 0.9; % friction coefficient

% animation
animation = 1;
anim_delay = 0.0;
vid_export = 0;       % animation export on
vid_file = 'blank'; % animation video file name


%% 2-DOF model
% robot.NB = 2 + 2; % extra 2 bodies for massless links
% robot.parent = [0,1,2,3];
% robot.jtype = {'px','py','r','r'};
% robot.Xtree{1} = eye(3);
% robot.Xtree{2} = eye(3);
% robot.Xtree{3} = plnr( 0, [0 0]);
% robot.Xtree{4} = plnr(0, [par.l,0]);
% robot.gravity = [0;-9.81];
% 
% I = mcI(par.m, [0.5, 0], par.J);   
% robot.I = {zeros(3), zeros(3), I, I};
% 
% robot.appearance.base = ...
%   { 'box', [-0.2 -0.3 -0.2; 0.2 0.3 -0.07] };
% 
% for i = 1:robot.NB
%   robot.appearance.body{i} = ...
%     { 'box', [0 -0.07 -0.04; 1 0.07 0.04], ...
%       'cyl', [0 0 -0.07; 0 0 0.07], 0.1 };
% end
% 
% % showmotion(robot)


%% Hexapod model
hexapod = struct;
hexapod = build_planar_hexapod(hexapod); % build planar hexapod robot
robot = build_planar_hexapod_aerial(hexapod);

% showmotion(robot)


%%
dt = 0.001;
t_vec = 0:dt:3;
n = length(t_vec);

x0 = [0, 0, deg2rad(-70), deg2rad(140), deg2rad(20), deg2rad(20), deg2rad(140)...
      0, 0, deg2rad(0),   deg2rad(0),   deg2rad(0),  deg2rad(0),  deg2rad(0)];

x_arr = zeros(n,14);
x_arr(1,:) = x0;
f_arr = zeros(n,3*2);


%% Simulate
for i = 2:length(t_vec)
%     tau = zeros(4,1);
    
    if t_vec(i) < 0.3
        tau = [0; 0; ...
               0; 0; 0; 0; 0];
    else
        tau = zeros(7,1);
    end
    
    q = x_arr(i-1,1:7)';
    qd = x_arr(i-1,8:14)';
    [x,f] = lcpPos(robot, par, q, qd, tau, dt);
    x_arr(i,:) = x;
    f_arr(i,:) = f;
    
    
%     xA0 = x(1);
%     yA0 = x(2);
%     theta1 = x(3);
%     theta2 = x(4);
%     theta3 = x(3);
%     theta4 = x(4);
%     theta5 = x(3);
%     
%     
%     xA0+robot.l(1).*cos(theta1)+robot.l(2).*cos(theta1+theta2)+...
%                 robot.l(3).*cos(theta1+theta2+theta3)+robot.l(4).*...
%                 cos(theta1+theta2+theta3+theta4)+robot.l(5).*...
%                 cos(theta1+theta2+theta3+theta4+theta5)

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
plot(t_vec, x_arr(:,1), 'Color', co(1,:), 'LineWidth', 1.5)
plot(t_vec, x_arr(:,2), 'Color', co(2,:), 'LineWidth', 1.5)
legend('x','y')
% joint space angle
subplot(1,2,2)
title('Joint Space Angle')
hold on
grid on
plot(t_vec, x_arr(:,3), 'Color', co(1,:))

fig_f = figure(434); clf
% normal force
subplot(2,1,1)
title('Normal Force')
ylim([-10 200])
hold on
grid on
plot(t_vec, f_arr(:,1));
plot(t_vec, f_arr(:,2));
legend('right foot', 'left foot')
% tangent force
subplot(2,1,2)
title('Tangent Force')
ylim([-100 100])
hold on
grid on
plot(t_vec, f_arr(:,3)-f_arr(:,4));
plot(t_vec, f_arr(:,5)-f_arr(:,6));
legend('right foot', 'left foot')


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
% showmotion(robot, t_span, x_sp_arr(:,1:3)')




fig_anim = figure(432); clf
% subplot(2,2,[1,3])
axis equal
ax_anim = gca;
hold on
grid on
xlim([-2 4])
ylim([-1 3])
plot([ax_anim.XLim(1), ax_anim.XLim(2)], [0,0], 'k')


if animation == 1
    if vid_export == 1
        vid = VideoWriter(['anim/',vid_file],'MPEG-4');
        vid.Quality = 50;
        vid.FrameRate = 60;
        open(vid);
        
        anim_2leg(hexapod,t_vec,x_arr,anim_delay, 432, vid);
        
        close(vid);
    else
        anim_2leg(hexapod,t_vec,x_arr,anim_delay, 432);
    end
    
end





% % plot trajectories
% % plot(x_an_arr(:,1)-par.r*cos(x_an_arr(:,3)), x_an_arr(:,2)-par.r*sin(x_an_arr(:,3))) % tip 
% % plot(x_an_arr(:,1), x_an_arr(:,2)) % center
% % plot(x_arr(:,1), x_arr(:,2), '--', 'Color', co(1,:)) % tip
% % plot(x_arr(:,1) + par.l*cos(x_arr(:,3)), x_arr(:,2) + par.l*sin(x_arr(:,3)),...
% %     'Color', co(1,:)) % tip
% 
% % plot(x_arr_pos(:,1), x_arr_pos(:,2), '--', 'Color', co(2,:)) % tip
% % plot(x_arr_pos(:,1) + par.l*cos(x_arr_pos(:,3)), x_arr_pos(:,2) + par.l*sin(x_arr_pos(:,3)),...
% %     'Color', co(2,:)) % tip
% % plot(1 + x_arr(:,1), x_arr(:,2), '--', 'Color', co(2,:)) % tip
% % plot(1 + x_arr(:,1) + par.l*cos(x_arr(:,3)), x_arr(:,2) + par.l*sin(x_arr(:,3)),...
% %     'Color', co(2,:)) % tip
% % plot(x_arr(:,1)+par.r*cos(x_arr(:,3)) + 2, x_arr(:,2)+par.r*sin(x_arr(:,3))) % center
% 
% subplot(2,2,2)
% title('Normal Force')
% ax_fn = gca;
% ylim([-10 200])
% hold on
% grid on
% % h_fn = plot(t_span, f_arr(:,1), 'Color', co(2,:));
% h_fn = plot(t_span, f_arr(:,1), 'Color', co(1,:));
% plot(t_span, f_arr(:,2), 'Color', co(2,:));
% 
% subplot(2,2,4)
% title('Tangent Force')
% ax_ft = gca;
% ylim([-100 100])
% hold on
% grid on
% h_ft = plot(t_span, f_arr(:,3)-f_arr(:,4), 'Color', co(2,:));
% plot(t_span, f_arr(:,5)-f_arr(:,6), ':', 'Color', co(2,:));
% 
% 
% 
% 
% % create link 1
% h_link1(1) = plot([0, par.l], [0,0], 'LineWidth', 1.5, 'Color', co(2,:));
% h_link1(2) = plot(par.r, 0, '.', 'MarkerSize', 30, 'Color', co(2,:));
% obj_link1 = hgtransform('parent',ax_anim);
% set(h_link1, 'parent', obj_link1);
% 
% % create link 2
% h_link2(1) = plot([0, par.l], [0,0], 'LineWidth', 1.5, 'Color', co(2,:));
% h_link2(2) = plot(par.r, 0, '.', 'MarkerSize', 30, 'Color', co(2,:));
% obj_link2 = hgtransform('parent',ax_anim);
% set(h_link2, 'parent', obj_link2);
% 
% 
% 
% 
% % create plot points
% p_fn = plot(ax_fn,0,0,'.', 'Color', h_fn.Color, 'MarkerSize', 20);
% p_fn_obj = hgtransform('parent',ax_fn);
% set(p_fn,'parent',p_fn_obj);
% p_ft = plot(ax_fn,0,0,'.', 'Color', h_ft.Color, 'MarkerSize', 20);
% p_ft_obj = hgtransform('parent',ax_ft);
% set(p_ft,'parent',p_ft_obj);
% 
% 
% vid_file = 'LCP_2link_asym';
% vid = VideoWriter(['anim/',vid_file],'MPEG-4');
% vid.Quality = 50;
% vid.FrameRate = 30;
% open(vid);
%         
% 
% 
% hTime = annotation('textbox',[0.01 0.01 0.06 0.03],'String',0,'Color',[0,0,0]);
% tic
% inc = max(1, round(1/dt/100));
% for i = 1:inc:n
%     while toc < t_span(i); continue; end % loop until at time
%     
%     set(hTime,'string',sprintf('%0.2f',t_span(i))) % update time display
%     
%     % coordinates
%     xA0 = x_arr(i,1);
%     yA0 = x_arr(i,2);
%     theta1 = x_arr(i,3);
%     theta2 = x_arr(i,4);
% 
%     % move link 1
%     T_link1 = makehgtform('zrotate',theta1);
%     T_link1(1,4) = xA0; % adjust x-location
%     T_link1(2,4) = yA0; % adjust y-location
%     set(obj_link1,'matrix',T_link1);
%     
%     % move link 2
%     T_link2 = makehgtform('zrotate',theta1+theta2);
%     T_link2(1,4) = xA0 + par.l*cos(theta1); % adjust x-location
%     T_link2(2,4) = yA0 + par.l*sin(theta1); % adjust y-location
%     set(obj_link2,'matrix',T_link2);
%     
%     % move plot points
%     T_p_fn = makehgtform('translate',[t_span(i),f_arr(i,1),0]);
%     set(p_fn_obj,'matrix',T_p_fn);
%     T_p_ft = makehgtform('translate',[t_span(i),f_arr(i,3)-f_arr(i,4),0]);
%     set(p_ft_obj,'matrix',T_p_ft);
% 
%     drawnow
% %     pause(0.02)
%     
%     writeVideo(vid, getframe(fig_anim));
% 
% end
% 
% close(vid);


%% Dynamics
function [x,f] = lcpPos(robot, par, q, qd, tau, dt)
    % position-based LCP
    m = 3; % DOF of system
    p = 2; % number of contacts
    d = 2; % number of friction cone bases    
    
    q_next_est = q + dt*qd; % kinematic estimate of next positions
    err = 1; % error between estimate and LCP solution
    iter = 0; % LCP iteration
    while err > 1e-6
        iter = iter + 1;
        
        xA0 = q_next_est(1);
        yA0 = q_next_est(2);
        theta1 = q_next_est(3);
        theta2 = q_next_est(4);
        theta3 = q_next_est(5);
        theta4 = q_next_est(6);
        theta5 = q_next_est(7);
        
        [M, C] = HandC(robot, q_next_est, qd); % M*qdd+C=tau 
        tau_star = M*qd - dt*(C - tau);

        J11 = [1 0 0 0 0 0 0; % Jacobian for foot contact point
               0 1 0 0 0 0 0;
               0 0 1 0 0 0 0];
%         J12 = [1, 0, -par.l*sin(theta1_est)-par.l*sin(theta1_est+theta2_est), -par.l*sin(theta1_est+theta2_est); % Jacobian for ee contact point
%                0, 1, par.l*cos(theta1_est)+par.l*cos(theta1_est+theta2_est),  par.l*cos(theta1_est+theta2_est);
%                0, 0, 1, 1];
        J12 = [1,0,(-1).*robot.l(1).*sin(theta1)+(-1).*robot.l(2).*sin(theta1+theta2)+(-1).* ...
            robot.l(3).*sin(theta1+theta2+theta3)+(-1).*robot.l(4).*sin(theta1+theta2+theta3+theta4)+( ...
            -1).*robot.l(4).*sin(theta1+theta2+theta3+theta4+theta5),(-1).*robot.l(2).*sin( ...
            theta1+theta2)+(-1).*robot.l(3).*sin(theta1+theta2+theta3)+(-1).*robot.l(4).* ...
            sin(theta1+theta2+theta3+theta4)+(-1).*robot.l(4).*sin(theta1+theta2+theta3+theta4+theta5),( ...
            -1).*robot.l(3).*sin(theta1+theta2+theta3)+(-1).*robot.l(4).*sin(theta1+theta2+theta3+theta4)+( ...
            -1).*robot.l(4).*sin(theta1+theta2+theta3+theta4+theta5),(-1).*robot.l(4).*sin( ...
            theta1+theta2+theta3+theta4)+(-1).*robot.l(4).*sin(theta1+theta2+theta3+theta4+theta5),( ...
            -1).*robot.l(4).*sin(theta1+theta2+theta3+theta4+theta5);0,1,robot.l(1).*cos( ...
            theta1)+robot.l(2).*cos(theta1+theta2)+robot.l(3).*cos(theta1+theta2+theta3)+robot.l(4).* ...
            cos(theta1+theta2+theta3+theta4)+robot.l(4).*cos(theta1+theta2+theta3+theta4+theta5),robot.l(2).* ...
            cos(theta1+theta2)+robot.l(3).*cos(theta1+theta2+theta3)+robot.l(4).*cos(theta1+theta2+theta3+theta4)+robot.l(4).* ...
            cos(theta1+theta2+theta3+theta4+theta5),robot.l(3).*cos(theta1+theta2+theta3)+robot.l(4).* ...
            cos(theta1+theta2+theta3+theta4)+robot.l(4).*cos(theta1+theta2+theta3+theta4+theta5),robot.l(4).* ...
            cos(theta1+theta2+theta3+theta4)+robot.l(4).*cos(theta1+theta2+theta3+theta4+theta5),robot.l(4).* ...
            cos(theta1+theta2+theta3+theta4+theta5);0,0,1,1,1,1,1];

           

        n1 = [1 0 0]'; % normal force direction at contact point(s)
        N = [J11'*n1, J12'*n1];

        D1 = [0 1  0; % friction cone bases at contact point(s)
              0 -1 0]';
        B = [J11'*D1, J12'*D1];

        e = ones(d,1);
        E = zeros(p*d,p);
        row = 1;
        for col = 1:p
            E(row:row+d-1,col) = e;
            row = row + d;
        end
        
        f = [xA0;
             xA0+robot.l(1).*cos(theta1)+robot.l(2).*cos(theta1+theta2)+...
                robot.l(3).*cos(theta1+theta2+theta3)+robot.l(4).*...
                cos(theta1+theta2+theta3+theta4)+robot.l(5).*...
                cos(theta1+theta2+theta3+theta4+theta5)];        
        alpha0 = N'*q_next_est - f;
        
        A = [dt^2*N'*(M\N), dt^2*N'*(M\B), zeros(p,p);
             dt*B'*(M\N), dt*B'*(M\B), E;
             par.mu*eye(p), -E', zeros(p,p)];
        q_lcp = [N'*q + dt*N'*(M\tau_star) - alpha0;
                 B'*(M\tau_star);
                 zeros(p,1)];

        [~,z] = LCPSolve(A,q_lcp); % w = A*z + q
        fn = z(1:p);
        fd = z(p+1:p+p*2);

        qd_next = qd + M\(-dt*(C - tau) + dt*(N*fn + B*fd));          
        q_next = q + dt*qd_next; % Euler integration
        
        err = sum(abs(q_next - q_next_est));
        q_next_est = q_next;
        
        if iter == 10e2
            fprintf('max reached, err: %4.3f \n', err)
            break
        end
    end
    
    x = [q_next', qd_next'];
    f = [fn', fd'];
end

