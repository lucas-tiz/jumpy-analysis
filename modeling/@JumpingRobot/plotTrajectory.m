function plotTrajectory(obj)

    co = get(0, 'DefaultAxesColorOrder');

    % jump plot
    f11 = figure(11); clf
    subplot(2,1,1)
    title('Jump Position (Torso)')
    xlabel('Time (s)')
    ylabel('Distance (m)')
    hold on
    grid on
    plot(obj.sim_data.t, obj.sim_data.traj.pos_jump(:,1))
    plot(obj.sim_data.t, obj.sim_data.traj.pos_jump(:,2))
    plot(obj.sim_data.info_jump.t_max_height, obj.sim_data.info_jump.height, '.', 'MarkerSize', 12)
    ylims = get(gca, 'YLim');
    plot([obj.sim_data.info_aerial.t,obj.sim_data.info_aerial.t], [ylims(1), ylims(2)], '--k', 'HandleVisibility','off')
    legend('vertical', 'horizontal')

    subplot(2,1,2)
    title('Jump Velocity (COM)')
    xlabel('Time (s)')
    ylabel('Velocity (m/s)')
    hold on
    grid on
    plot(obj.sim_data.t, obj.sim_data.traj.v_com(:,1))
    plot(obj.sim_data.t, obj.sim_data.traj.v_com(:,2))
    legend('vertical', 'horizontal')

    % joint plots
    f12 = figure(12); clf
    subplot(2,1,1)
    title('Joint Angles')
    xlabel('Time (s)')
    ylabel('Angle (deg)')
    hold on
    grid on
    plot(obj.sim_data.t, rad2deg(obj.sim_data.x(:,4)), 'Color', co(1,:))
    plot(obj.sim_data.t, rad2deg(obj.sim_data.x(:,7)), ':', 'Color', co(1,:), 'LineWidth', 1.5)
    plot(obj.sim_data.t, rad2deg(obj.sim_data.x(:,5)), 'Color', co(2,:))
    plot(obj.sim_data.t, rad2deg(obj.sim_data.x(:,6)), ':', 'Color', co(2,:), 'LineWidth', 1.5)
    legend('\Theta_2 (knee right)','\Theta_5 (knee left)',...
        '\Theta_3 (hip right)','\Theta_4 (hip left)', 'Location', 'Best')
    ylims = get(gca, 'YLim');
    % ylim([-360 360])
    plot([obj.sim_data.info_aerial.t,obj.sim_data.info_aerial.t], [ylims(1), ylims(2)], '--k', 'HandleVisibility','off')

    subplot(2,1,2)
    title('Joint Angular Velocities')
    xlabel('Time (s)')
    ylabel('Angular Velocity (deg/s)')
    hold on
    grid on
    plot(obj.sim_data.t, rad2deg(obj.sim_data.x(:,11)), 'Color', co(1,:))
    plot(obj.sim_data.t, rad2deg(obj.sim_data.x(:,14)), ':', 'Color', co(1,:), 'LineWidth', 1.5)
    plot(obj.sim_data.t, rad2deg(obj.sim_data.x(:,12)), 'Color', co(2,:))
    plot(obj.sim_data.t, rad2deg(obj.sim_data.x(:,13)), ':', 'Color', co(2,:), 'LineWidth', 1.5)
    legend('\Theta_2d (knee right)','\Theta_5d (knee left)',...
        '\Theta_3d (hip right)','\Theta_4d (hip left)', 'Location', 'Best')
    ylims = get(gca, 'YLim');
    % ylim([-360 360])
    plot([obj.sim_data.info_aerial.t,obj.sim_data.info_aerial.t], [ylims(1), ylims(2)], '--k', 'HandleVisibility','off')

    % joint state plots -------------------------------------------------------
    f13 = figure(13); clf
    subplot(4,2,1)
    title('Knee Dynamics')
    hold on
    grid on
    plot(obj.sim_data.t, obj.sim_data.j_state.knee_right.linear_displace, 'Color', co(1,:))
    plot(obj.sim_data.t, obj.sim_data.j_state.knee_left.linear_displace, ':', 'Color', co(1,:), 'LineWidth', 1.5)
    plot(obj.sim_data.t, obj.sim_data.j_state.knee_right.muscle_contract, 'Color', co(2,:))
    plot(obj.sim_data.t, obj.sim_data.j_state.knee_left.muscle_contract, ':', 'Color', co(2,:), 'LineWidth', 1.5)
    plot(obj.sim_data.t, obj.sim_data.j_state.knee_right.tendon_stretch, 'Color', co(3,:))
    plot(obj.sim_data.t, obj.sim_data.j_state.knee_left.tendon_stretch, ':', 'Color', co(3,:), 'LineWidth', 1.5)
    xlabel('Time (s)')
    ylabel('Length (cm)')
    legend('Joint Disp 1', 'Joint Disp 4', 'Muscle Contract 1', 'Muscle Contract 4',...
        'Tendon Stretch 1', 'Tendon Stretch 4', 'Location', 'NorthEast')
    ylims = get(gca, 'YLim');
    plot([obj.sim_data.info_aerial.t,obj.sim_data.info_aerial.t], [ylims(1), ylims(2)], '--k', 'HandleVisibility','off')

    subplot(4,2,3)
    hold on
    grid on
    plot(obj.sim_data.t, obj.sim_data.j_state.knee_right.muscle_pressure, 'Color', co(1,:))
    plot(obj.sim_data.t, obj.sim_data.j_state.knee_left.muscle_pressure, ':', 'Color', co(1,:), 'LineWidth', 1.5)
    xlabel('Time (s)')
    ylabel('Pressure (kPa)')
    legend('Pressure 1', 'Pressure 4')
    ylims = get(gca, 'YLim');
    plot([obj.sim_data.info_aerial.t,obj.sim_data.info_aerial.t], [ylims(1), ylims(2)], '--k', 'HandleVisibility','off')

    subplot(4,2,5)
    hold on
    grid on
    plot(obj.sim_data.t, obj.sim_data.j_state.knee_right.mtu_force, 'Color', co(1,:))
    plot(obj.sim_data.t, obj.sim_data.j_state.knee_left.mtu_force, ':', 'Color', co(1,:), 'LineWidth', 1.5)
    xlabel('Time (s)')
    ylabel('Force (N)')
    legend('Force 1', 'Force 4')
    ylims = get(gca, 'YLim');
    plot([obj.sim_data.info_aerial.t,obj.sim_data.info_aerial.t], [ylims(1), ylims(2)], '--k', 'HandleVisibility','off')

    subplot(4,2,7)
    hold on
    grid on
    plot(obj.sim_data.t, obj.sim_data.j_state.knee_right.torque, 'Color', co(1,:))
    plot(obj.sim_data.t, obj.sim_data.j_state.knee_left.torque, ':', 'Color', co(1,:), 'LineWidth', 1.5)
    plot(obj.sim_data.t, obj.sim_data.tau(:,4), 'Color', co(2,:))
    plot(obj.sim_data.t, obj.sim_data.tau(:,7), ':', 'Color', co(2,:), 'LineWidth', 1.5)
    xlabel('Time (s)')
    ylabel('Torque (Nm)')
    legend('muscle r', 'muscle l', 'total r', 'total l')
    ylims = get(gca, 'YLim');
    % ylim([0 20]);
    plot([obj.sim_data.info_aerial.t,obj.sim_data.info_aerial.t], [ylims(1), ylims(2)], '--k', 'HandleVisibility','off')
    %
    subplot(4,2,2)
    title('Hip Dynamics')
    hold on
    grid on
    plot(obj.sim_data.t, obj.sim_data.j_state.hip_right.linear_displace, 'Color', co(1,:))
    plot(obj.sim_data.t, obj.sim_data.j_state.hip_left.linear_displace, ':', 'Color', co(1,:), 'LineWidth', 1.5)
    plot(obj.sim_data.t, obj.sim_data.j_state.hip_right.muscle_contract, 'Color', co(2,:))
    plot(obj.sim_data.t, obj.sim_data.j_state.hip_left.muscle_contract, ':', 'Color', co(2,:), 'LineWidth', 1.5)
    plot(obj.sim_data.t, obj.sim_data.j_state.hip_right.tendon_stretch, 'Color', co(3,:))
    plot(obj.sim_data.t, obj.sim_data.j_state.hip_left.tendon_stretch, ':', 'Color', co(3,:), 'LineWidth', 1.5)
    xlabel('Time (s)')
    ylabel('Length (cm)')
    legend('Joint Disp 1', 'Joint Disp 4', 'Muscle Contract 1', 'Muscle Contract 4',...
        'Tendon Stretch 1', 'Tendon Stretch 4', 'Location', 'NorthEast')
    ylims = get(gca, 'YLim');
    plot([obj.sim_data.info_aerial.t,obj.sim_data.info_aerial.t], [ylims(1), ylims(2)], '--k', 'HandleVisibility','off')

    subplot(4,2,4)
    hold on
    grid on
    plot(obj.sim_data.t, obj.sim_data.j_state.hip_right.muscle_pressure, 'Color', co(1,:))
    plot(obj.sim_data.t, obj.sim_data.j_state.hip_left.muscle_pressure, ':', 'Color', co(1,:), 'LineWidth', 1.5)
    xlabel('Time (s)')
    ylabel('Pressure (kPa)')
    legend('Pressure 1', 'Pressure 4')
    ylims = get(gca, 'YLim');
    plot([obj.sim_data.info_aerial.t,obj.sim_data.info_aerial.t], [ylims(1), ylims(2)], '--k', 'HandleVisibility','off')

    subplot(4,2,6)
    hold on
    grid on
    plot(obj.sim_data.t, obj.sim_data.j_state.hip_right.mtu_force, 'Color', co(1,:))
    plot(obj.sim_data.t, obj.sim_data.j_state.hip_left.mtu_force, ':', 'Color', co(1,:), 'LineWidth', 1.5)
    xlabel('Time (s)')
    ylabel('Force (N)')
    legend('Force 1', 'Force 4')
    ylims = get(gca, 'YLim');
    plot([obj.sim_data.info_aerial.t,obj.sim_data.info_aerial.t], [ylims(1), ylims(2)], '--k', 'HandleVisibility','off')

    subplot(4,2,8)
    hold on
    grid on
    plot(obj.sim_data.t, obj.sim_data.j_state.hip_right.torque, 'Color', co(1,:))
    plot(obj.sim_data.t, obj.sim_data.j_state.hip_left.torque, ':', 'Color', co(1,:), 'LineWidth', 1.5)
    plot(obj.sim_data.t, obj.sim_data.tau(:,5), 'Color', co(2,:))
    plot(obj.sim_data.t, obj.sim_data.tau(:,6), ':', 'Color', co(2,:), 'LineWidth', 1.5)
    xlabel('Time (s)')
    ylabel('Torque (Nm)')
    legend('muscle r', 'muscle l', 'total r', 'total l')
    ylims = get(gca, 'YLim');
    plot([obj.sim_data.info_aerial.t,obj.sim_data.info_aerial.t], [ylims(1), ylims(2)], '--k', 'HandleVisibility','off')


    % reaction force plots ----------------------------------------------------
    f14 = figure(14); clf
    subplot(2,1,1)
    title('Normal GRFs')
    hold on
    grid on
    ylim([0 50])
    h_r = plot(obj.sim_data.t, obj.sim_data.grf.r(:,1), 'Color', co(1,:));
    h_l = plot(obj.sim_data.t, obj.sim_data.grf.l(:,1), ':', 'Color', co(2,:), 'LineWidth', 1.5);
    ylims = get(gca, 'YLim');
    plot([obj.sim_data.info_aerial.t,obj.sim_data.info_aerial.t], [ylims(1), ylims(2)], '--k')
    legend([h_r, h_l], 'right foot', 'left foot')

    subplot(2,1,2)
    title('Tangential GRFs')
    hold on
    grid on
    ylim([-50 50])
    plot(obj.sim_data.t, obj.sim_data.grf.r(:,2), 'Color', co(1,:))
    plot(obj.sim_data.t, obj.sim_data.grf.l(:,2), ':', 'Color', co(2,:), 'LineWidth', 1.5)
    ylims = get(gca, 'YLim');
    plot([obj.sim_data.info_aerial.t,obj.sim_data.info_aerial.t], [ylims(1), ylims(2)], '--k')
    legend('right foot', 'left foot')
end

