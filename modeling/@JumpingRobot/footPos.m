function foot_pos = footPos(obj, x)
    % calculate robot foot positions

    l = obj.config.morphology.l;

    xA0 = x(1);
    theta1 = x(3);
    theta2 = x(4);
    theta3 = x(5);
    theta4 = x(6);
    theta5 = x(7);

    xA5 = xA0+l(1).*cos(theta1)+l(2).*cos(theta1+theta2)+...
        l(3).*cos(theta1+theta2+theta3)+l(4).*...
        cos(theta1+theta2+theta3+theta4)+l(5).*...
        cos(theta1+theta2+theta3+theta4+theta5);     
           
    foot_pos = [xA0; xA5];
    
end