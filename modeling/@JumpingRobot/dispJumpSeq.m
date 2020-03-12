function dispJumpSeq(obj, t_seq, dim_subplot, fig_no)

    f = figure(fig_no); clf
    hold on
    

    n_frames = length(t_seq);
    for i = 1:n_frames
        subplot(dim_subplot(1),dim_subplot(2),i)
        ax = gca;
        box on
        
        pos = get(ax, 'Position');
        pos(2) = pos(2)*0.9;
        pos(3) = pos(3)*1.2;
        pos(4) = pos(4)*1.2;
        set(gca, 'Position', pos)
        ax.XTick = [];
        
        if i == 1
            ylabel('Height (m)')
        else
            ax.YTickLabel = {};
        end
        
        % draw past robot poses
        n_shadow = 1; % number of shadows
        c_shadow = 1; % shadow counter
        for j = max(1,(i-(n_shadow))):(i-1)
            idx_frame = find(obj.sim_data.t >= t_seq(j), 1);
            t_frame = obj.sim_data.t(idx_frame);
            obj.drawRobot(t_frame, [1,1,1]*0.98 - [1,1,1]*0.08*c_shadow/n_shadow);
            c_shadow = c_shadow + 1;
        end
        
        % draw robot pose
        idx_frame = find(obj.sim_data.t >= t_seq(i), 1);
        t_frame = obj.sim_data.t(idx_frame);
        obj.drawRobot(t_frame);
    end

    
    pub_figureFormat(f, 'CMU Serif');
end