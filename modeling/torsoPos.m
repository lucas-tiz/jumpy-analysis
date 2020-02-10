function torso_pos = torsoPos(robot, q)
    % calculate robot foot positions
    
    xA0 = q(:,1);
    yA0 = q(:,2);
    theta1 = q(:,3); 
    theta2 = q(:,4);
    theta3 = q(:,5);
    x_torso = xA0 + robot.l(1)*cos(theta1) + robot.l(2)*cos(theta1+theta2)...
        + robot.l(3)*cos(theta1+theta2+theta3);
    y_torso = yA0 + robot.l(1)*sin(theta1) + robot.l(2)*sin(theta1+theta2)...
        + robot.l(3)*sin(theta1+theta2+theta3);  
           
    torso_pos = [x_torso, y_torso];
    
end