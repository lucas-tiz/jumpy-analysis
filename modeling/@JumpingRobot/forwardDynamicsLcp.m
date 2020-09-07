function [x,f,err_lim_bool] = forwardDynamicsLcp(obj, q, qd, tau)
    % Calculate forward dynamics using position-based LCP approach
    %TODO: in the future, generalize this for any robot (pass in contact
    %Jacobians, position constraint function, etc

    l = obj.config.morphology.l;
    mu = obj.config.model.mu;
    dt = obj.sim_param.dt;
        
    m = 3; % DOF of system
    p = 2; % number of contacts
    d = 2; % number of friction cone bases    
    
    q_next_est = q + dt*qd; % kinematic estimate of next positions
    err = 1; % error between estimate and LCP solution
    iter = 0; % LCP iteration
    while err > 1e-6 %TODO: determine error limit
        iter = iter + 1;
        
        xA0 = q_next_est(1);
        yA0 = q_next_est(2);
        theta1 = q_next_est(3);
        theta2 = q_next_est(4);
        theta3 = q_next_est(5);
        theta4 = q_next_est(6);
        theta5 = q_next_est(7);
        
        [M, C] = HandC(obj.spatialRobot, q_next_est, qd); % M(q)*qdd + C(q,qd) = tau 
        tau_star = M*qd - dt*(C - tau);

        J11 = [1 0 0 0 0 0 0; % Jacobian for foot contact point
               0 1 0 0 0 0 0;
               0 0 1 0 0 0 0];
        J12 = [1,0,(-1).*l(1).*sin(theta1)+(-1).*l(2).*sin(theta1+theta2)+(-1).* ...
            l(3).*sin(theta1+theta2+theta3)+(-1).*l(4).*sin(theta1+theta2+theta3+theta4)+( ...
            -1).*l(5).*sin(theta1+theta2+theta3+theta4+theta5),(-1).*l(2).*sin( ...
            theta1+theta2)+(-1).*l(3).*sin(theta1+theta2+theta3)+(-1).*l(4).* ...
            sin(theta1+theta2+theta3+theta4)+(-1).*l(5).*sin(theta1+theta2+theta3+theta4+theta5),( ...
            -1).*l(3).*sin(theta1+theta2+theta3)+(-1).*l(4).*sin(theta1+theta2+theta3+theta4)+( ...
            -1).*l(5).*sin(theta1+theta2+theta3+theta4+theta5),(-1).*l(4).*sin( ...
            theta1+theta2+theta3+theta4)+(-1).*l(5).*sin(theta1+theta2+theta3+theta4+theta5),( ...
            -1).*l(5).*sin(theta1+theta2+theta3+theta4+theta5);0,1,l(1).*cos( ...
            theta1)+l(2).*cos(theta1+theta2)+l(3).*cos(theta1+theta2+theta3)+l(4).* ...
            cos(theta1+theta2+theta3+theta4)+l(5).*cos(theta1+theta2+theta3+theta4+theta5),l(2).* ...
            cos(theta1+theta2)+l(3).*cos(theta1+theta2+theta3)+l(4).*cos(theta1+theta2+theta3+theta4)+l(5).* ...
            cos(theta1+theta2+theta3+theta4+theta5),l(3).*cos(theta1+theta2+theta3)+l(4).* ...
            cos(theta1+theta2+theta3+theta4)+l(5).*cos(theta1+theta2+theta3+theta4+theta5),l(4).* ...
            cos(theta1+theta2+theta3+theta4)+l(5).*cos(theta1+theta2+theta3+theta4+theta5),l(5).* ...
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
        
        pos_constraint = obj.footPos(q_next_est);%... % position constraints for robot feet
%             [xA0;
%              xA0+l(1).*cos(theta1)+l(2).*cos(theta1+theta2)+...
%                 l(3).*cos(theta1+theta2+theta3)+l(4).*...
%                 cos(theta1+theta2+theta3+theta4)+l(5).*...
%                 cos(theta1+theta2+theta3+theta4+theta5)];        
        alpha0 = N'*q_next_est - pos_constraint;        
%         A = [dt^2*N'*(M\N), dt^2*N'*(M\B), zeros(p,p);
%              dt*B'*(M\N), dt*B'*(M\B), E;
%              robot.mu*eye(p), -E', zeros(p,p)];
%         q_lcp = [N'*q + dt*N'*(M\tau_star) - alpha0;
%                  B'*(M\tau_star);
%                  zeros(p,1)];
        A = [N'*(M\N), N'*(M\B), zeros(p,p);
             B'*(M\N), B'*(M\B), 1/dt*E;
             mu*eye(p), -E', zeros(p,p)];
        q_lcp = [1/(dt^2)*(N'*q + dt*N'*(M\tau_star) - alpha0);
                 1/dt*(B'*(M\tau_star));
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
    
    err_lim_bool = 0;
    if err > 1
        err_lim_bool = 1;
    end
    
    x = [q_next', qd_next'];
    f = [fn', fd'];    
end

