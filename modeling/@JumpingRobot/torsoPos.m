function torso_pos = torsoPos(obj, x)
    % calculate robot foot positions
    l = obj.config.morphology.l;

    xA0 = x(:,1);
    yA0 = x(:,2);
    theta1 = x(:,3); 
    theta2 = x(:,4);
    theta3 = x(:,5);
    
    x_torso = xA0 + l(1)*cos(theta1) + l(2)*cos(theta1+theta2)...
        + l(3)*cos(theta1+theta2+theta3);
    y_torso = yA0 + l(1)*sin(theta1) + l(2)*sin(theta1+theta2)...
        + l(3)*sin(theta1+theta2+theta3);  
           
    torso_pos = [x_torso, y_torso];
    
end