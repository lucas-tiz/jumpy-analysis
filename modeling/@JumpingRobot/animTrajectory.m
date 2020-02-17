function animTrajectory(obj, dt, anim_delay, fig_num, vid)

    if vid.export == 1
        video = VideoWriter(fullfile(vid.path,vid.file),'MPEG-4');
        video.Quality = vid.Quality;
        video.FrameRate = vid.FrameRate;
        open(video);
    end
        
    l = obj.config.morphology.l;

    % create figure
    f = figure(fig_num); clf
    hold on
    grid on

    % set plot dimensions
    w = 3;
    h = 3;
    xlim([-w w])
    ylim([-h h])
    pbaspect([1 h/w 1])
    ax = gca;

    % show time
    hTime = annotation('textbox',[0.01 0.01 0.06 0.03],'String',0,'Color',[0,0,0]);

    % draw floor
    plot([-10 10], [0 0], '--k', 'linewidth', 2);

    % draw links
    h_link1(1) = plot([0,0], [0,l(1)], 'LineWidth', 1.5);
    h_link1(2) = plot(0, 0, '.', 'MarkerSize', 30);

    h_link2(1) = plot([0,0], [0,l(2)], 'LineWidth', 1.5);
    h_link2(2) = plot(0, 0, '.', 'MarkerSize', 30);

    h_link3(1) = plot([0,0], [0,l(3)], 'LineWidth', 1.5);
    h_link3(2) = plot(0, 0, '.', 'MarkerSize', 30);

    h_link4(1) = plot([0,0], [0,l(4)], 'LineWidth', 1.5);
    h_link4(2) = plot(0, 0, '.', 'MarkerSize', 30);

    h_link5(1) = plot([0,0], [0,l(5)], 'LineWidth', 1.5);
    h_link5(2) = plot(0, 0, '.', 'MarkerSize', 30);
    h_link5(3) = plot(0, l(5), '.', 'MarkerSize', 30);

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


    pause(0.2)
    tic
    inc = max(1, round(1/dt/100));
    for i = 1:inc:length(obj.sim_data.t)        
        while toc < obj.sim_data.t(i); continue; end % loop until at time
        set(hTime,'string',sprintf('%0.2f',obj.sim_data.t(i))) % update time display

        % get states
        xA0 = -obj.sim_data.x(i,2);
        yA0 = obj.sim_data.x(i,1);
        theta1 = obj.sim_data.x(i,3); % [radians] theta 1 angle
        theta2 = obj.sim_data.x(i,4); % [radians] theta 2 angle
        theta3 = obj.sim_data.x(i,5);
        theta4 = obj.sim_data.x(i,6);
        theta5 = obj.sim_data.x(i,7);

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

        if anim_delay > 0
            pause(anim_delay)
        end

        if vid.export == 1
            writeVideo(video, getframe(f));
        end
    end

    if vid.export == 1
        close(video);
    end
end

