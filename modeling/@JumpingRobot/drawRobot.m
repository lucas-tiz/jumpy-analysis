function drawRobot(obj, t, varargin)
    markersize = 15;
    colors = get(gca, 'ColorOrder');
        
    l = obj.config.morphology.l; % robot link lengths
    idx = find(obj.sim_data.t >= t, 1);
        
    % set plot dimensions
    w = 2;
    h = 2.5;
    xlim([-w/2 w/2])
    ylim([0 h])
    pbaspect([1 h/w 1])
    ax = gca;
    hold on
    grid on
    set(ax, 'YGrid', 'on', 'XGrid', 'off')

    % draw floor
%     plot([-10 10], [0 0], 'k', 'linewidth', 1);

    % draw links
    if nargin > 2
        h_link1(1) = plot([0,0], [0,l(1)], 'LineWidth', 1.5, 'Color', varargin{1});
        h_link1(2) = plot(0, 0, '.', 'MarkerSize', markersize, 'Color', varargin{1});

        h_link2(1) = plot([0,0], [0,l(2)], 'LineWidth', 1.5, 'Color', varargin{1});
        h_link2(2) = plot(0, 0, '.', 'MarkerSize', markersize, 'Color', varargin{1});

        h_link3(1) = plot([0,0], [0,l(3)], 'LineWidth', 1.5, 'Color', varargin{1});
        h_link3(2) = plot(0, 0, '.', 'MarkerSize', markersize, 'Color', varargin{1});

        h_link4(1) = plot([0,0], [0,l(4)], 'LineWidth', 1.5, 'Color', varargin{1});
        h_link4(2) = plot(0, 0, '.', 'MarkerSize', markersize, 'Color', varargin{1});

        h_link5(1) = plot([0,0], [0,l(5)], 'LineWidth', 1.5, 'Color', varargin{1});
        h_link5(2) = plot(0, 0, '.', 'MarkerSize', markersize, 'Color', varargin{1});
        h_link5(3) = plot(0, l(5), '.', 'MarkerSize', markersize, 'Color', varargin{1});
    else
        % show time
        text(0.7, 0.9, sprintf('%0.2f s', obj.sim_data.t(idx)), 'Units', 'normalized',...
            'FontName', 'CMU Serif')
        
        h_link1(1) = plot([0,0], [0,l(1)], 'LineWidth', 1.5, 'Color', colors(1,:));
        h_link1(2) = plot(0, 0, '.', 'MarkerSize', markersize, 'Color', colors(2,:));

        h_link2(1) = plot([0,0], [0,l(2)], 'LineWidth', 1.5, 'Color', colors(1,:));
        h_link2(2) = plot(0, 0, '.', 'MarkerSize', markersize, 'Color', 'k');

        h_link3(1) = plot([0,0], [0,l(3)], 'LineWidth', 1.5, 'Color', colors(1,:));
        h_link3(2) = plot(0, 0, '.', 'MarkerSize', markersize, 'Color', 'k');

        h_link4(1) = plot([0,0], [0,l(4)], 'LineWidth', 1.5, 'Color', colors(1,:));
        h_link4(2) = plot(0, 0, '.', 'MarkerSize', markersize, 'Color', 'k');

        h_link5(1) = plot([0,0], [0,l(5)], 'LineWidth', 1.5, 'Color', colors(1,:));
        h_link5(2) = plot(0, 0, '.', 'MarkerSize', markersize, 'Color', 'k');
        h_link5(3) = plot(0, l(5), '.', 'MarkerSize', markersize, 'Color', colors(2,:));
    end

    % create transform objects and parent wheel/body to objects
    obj_link1 = hgtransform('parent',ax);
    set(h_link1, 'parent', obj_link1);

    obj_link2 = hgtransform('parent',ax);
    set(h_link2, 'parent', obj_link2);

    obj_link3 = hgtransform('parent',ax);
    set(h_link3, 'parent', obj_link3);

    obj_link4 = hgtransform('parent',ax);
    set(h_link4, 'parent', obj_link4);

    obj_link5 = hgtransform('parent',ax);
    set(h_link5, 'parent', obj_link5);

    
    % get states
    xA0 = -obj.sim_data.x(idx,2) + 0.55/2;
    yA0 = obj.sim_data.x(idx,1);
    theta1 = obj.sim_data.x(idx,3); % [radians] theta 1 angle
    theta2 = obj.sim_data.x(idx,4); % [radians] theta 2 angle
    theta3 = obj.sim_data.x(idx,5);
    theta4 = obj.sim_data.x(idx,6);
    theta5 = obj.sim_data.x(idx,7);

    % transform links
    T_link1 = makehgtform('zrotate',theta1);
    T_link1(1,4) = xA0; % adjust x-location
    T_link1(2,4) = yA0; % adjust y-location
    set(obj_link1,'matrix',T_link1);

    T_link2 = makehgtform('zrotate',theta1+theta2);
    T_link2(1,4) = xA0 - l(1)*sin(theta1); % adjust x-location
    T_link2(2,4) = yA0 + l(1)*cos(theta1); % adjust y-location
    set(obj_link2,'matrix',T_link2);

    T_link3 = makehgtform('zrotate',theta1+theta2+theta3);
    T_link3(1,4) = xA0 - l(1)*sin(theta1) - l(2)*sin(theta1+theta2); % adjust x-location
    T_link3(2,4) = yA0 + l(1)*cos(theta1) + l(2)*cos(theta1+theta2); % adjust y-location
    set(obj_link3,'matrix',T_link3);

    T_link4 = makehgtform('zrotate',theta1+theta2+theta3+theta4);
    T_link4(1,4) = xA0 - l(1)*sin(theta1) - l(2)*sin(theta1+theta2)...
        - l(3)*sin(theta1+theta2+theta3); % adjust x-location
    T_link4(2,4) = yA0 + l(1)*cos(theta1) + l(2)*cos(theta1+theta2)...
        + l(3)*cos(theta1+theta2+theta3);% adjust y-location
    set(obj_link4,'matrix',T_link4);

    T_link5 = makehgtform('zrotate',theta1+theta2+theta3+theta4+theta5);
    T_link5(1,4) = xA0 - l(1)*sin(theta1) - l(2)*sin(theta1+theta2)...
        - l(3)*sin(theta1+theta2+theta3) - l(4)*sin(theta1+theta2+theta3+theta4); % adjust x-location
    T_link5(2,4) = yA0 + l(1)*cos(theta1) + l(2)*cos(theta1+theta2)...
        + l(3)*cos(theta1+theta2+theta3) + l(4)*cos(theta1+theta2+theta3+theta4);% adjust y-location
    set(obj_link5,'matrix',T_link5);

    drawnow

end



