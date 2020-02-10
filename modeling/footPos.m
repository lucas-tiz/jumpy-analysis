function foot_pos = footPos(robot, q)
    % calculate robot foot positions

    xA0 = q(1);
    theta1 = q(3);
    theta2 = q(4);
    theta3 = q(5);
    theta4 = q(6);
    theta5 = q(7);

    xA5 = xA0+robot.l(1).*cos(theta1)+robot.l(2).*cos(theta1+theta2)+...
        robot.l(3).*cos(theta1+theta2+theta3)+robot.l(4).*...
        cos(theta1+theta2+theta3+theta4)+robot.l(5).*...
        cos(theta1+theta2+theta3+theta4+theta5);     
           
    foot_pos = [xA0; xA5];
    
end