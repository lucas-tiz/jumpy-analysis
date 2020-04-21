%
clear, clc


analysis = 1;

cam.d = 54; %30
cam.phi_range = [0, 180]; %[0, 180]
cam.beta_range = [-20, 200]; %[-20, 200]
cam.beta_inc = 10; %10

knee.rad0 = 6; %0
knee.slope = -1; %6
knee.theta_range = [180, 0]; %[180, 0]
knee.k_tendon = 250;

joint = MuscleJoint(knee, cam);


%% Plot cam-muscle geometry
analysis = 1
if analysis == 1
    beta = deg2rad(0); % (rad) joint angle
    
    geom = joint.calc_ema(beta, knee.rad0, knee.slope, cam.d, deg2rad(cam.phi_range)); 
    double(rad2deg(geom.phi))
    fprintf('r_tan: %4.2f, ema: %4.2f\n', geom.r_tan, geom.ema)
    
    plotCam(beta, cam, knee, geom)    
end


%% Animate cam-muscle geometry
analysis = 4
if analysis == 4
    theta_vec = deg2rad(180:-5:90);
    beta_vec = joint.convert_joint_angle_to_cam_angle(theta_vec);
    n_beta = length(beta_vec);

    ema_vec = zeros(1,n_beta);
    l_disp_vec = zeros(1,n_beta);
    phi_vec = zeros(1,n_beta);
    r_tan_vec = zeros(1,n_beta);
    
    ema_vec_interp = zeros(1,n_beta);
    l_disp_vec_interp = zeros(1,n_beta);
    
    r_beta_vec = knee.rad0 + knee.slope*(beta_vec); % radius at joint angle
    
    geom0 = joint.calc_ema(beta_vec(1), knee.rad0, knee.slope, cam.d, deg2rad(cam.phi_range));     
    for i = 1:length(beta_vec)
        beta = beta_vec(i); % (rad) joint angle

        geom = joint.calc_ema(beta, knee.rad0, knee.slope, cam.d, deg2rad(cam.phi_range)); 
        fprintf('phi: %4.2f\n', rad2deg(geom.phi))
        fprintf('r_tan: %4.4f, ema: %4.4f\n\n', geom.r_tan, geom.ema)

        plotCam(beta, cam, knee, geom) 
        
        ema_vec(i) = geom.ema;
        l_disp_vec(i) = geom.l_disp - geom0.l_disp;
        r_tan_vec(i) = geom.r_tan;
        phi_vec(i) = geom.phi;
        
        joint_state_struct = joint.calcJointState(theta_vec(1), theta_vec(i), 0, 240);
        ema_vec_interp(i) = joint_state_struct.ema;
        l_disp_vec_interp(i) = joint_state_struct.linear_displace;
    end
    
    % import yetong data
    yetong_data = csvread(['C:\Users\Lucas\Dropbox (GaTech)\Research\Hexapod\',...
        'analysis\cam profile geometry\yetong_cam_calcs.csv'], 1, 0);
    ydata.beta = yetong_data(:,1);
    ydata.phi = yetong_data(:,2);
    ydata.l_disp = yetong_data(:,3);
    ydata.ema = yetong_data(:,4);
    
    % plot
    linewidth = 1.5;
    f544 = figure(544); clf
    subplot(3,1,1)
    hold on
    grid on
    plot(rad2deg(beta_vec), r_beta_vec, 'LineWidth', linewidth)
    plot(rad2deg(beta_vec), r_tan_vec, ':', 'LineWidth', linewidth)
    plot(rad2deg(beta_vec), ema_vec, 'LineWidth', linewidth)
    plot(rad2deg(ydata.beta), ydata.ema, '--', 'LineWidth', linewidth)
    plot(rad2deg(beta_vec), ema_vec_interp, ':', 'LineWidth', linewidth)

%     plot(rad2deg([beta_vec(1),beta_vec(end)]),...
%         [ema_vec(1),ema_vec(end)], '--k')
%     plot(rad2deg(beta_vec_cor), ema_vec_cor)
%     plot(rad2deg([beta_vec_cor(1),beta_vec_cor(end)]),...
%         [ema_vec_cor(1),ema_vec_cor(end)], '--k')
    legend('radius @ beta', 'radius @ tangent', 'ema-Lucas',...
        'ema-Yetong', 'ema-interp', 'Location','Best')
    xlabel('Cam Angle \beta (deg)')
    ylabel('Length (mm)')

    subplot(3,1,2)
    hold on
    grid on
    plot(rad2deg(beta_vec), rad2deg(phi_vec), 'LineWidth', linewidth)
    plot(rad2deg(ydata.beta), rad2deg(ydata.phi), '--', 'LineWidth', linewidth)
    xlabel('Cam Angle \beta (deg)')
    ylabel('Phi (deg)')
    legend('Lucas', 'Yetong')

    subplot(3,1,3)
    hold on
    grid on
    plot(rad2deg(beta_vec), l_disp_vec, 'LineWidth', linewidth)
    plot(rad2deg(ydata.beta), ydata.l_disp, '--', 'LineWidth', linewidth)
    plot(rad2deg(beta_vec), l_disp_vec_interp, ':', 'LineWidth', linewidth)
    xlabel('Cam Angle \beta (deg)')
    ylabel('Joint Displacement (cm)')
    legend('Lucas', 'Yetong', 'Lucas-interp')
    
end



%% Plot moment arm versus beta
if analysis == 2
    beta_vec = param.beta_vec;
    n_beta = length(beta_vec);

    r_beta_vec = param.rad0 + param.slope*(beta_vec); % radius at joint angle
    x_beta_vec = zeros(1,n_beta); % point corresponding to beta in global frame
    y_beta_vec = r_beta_vec; % point corresponding to beta in global frame

    ema_vec = zeros(1,n_beta);
    phi_vec = zeros(1,n_beta);
    r_tan_vec = zeros(1,n_beta);

    tic
    parfor i = 1:n_beta
        beta = beta_vec(i);
%         [ema_vec(i),r_tan_vec(i),phi_vec(i),~] = calc_ema(beta,param);
        
        geom = joint.calc_ema(beta, knee.rad0, knee.slope, cam.d, cam.phi_range); 
        ema_vec(i) = geom.ema;
%         l_disp = geom.l_disp;
        r_tan_vec(i) = geom.r_tan;
        phi_vec(i) = geom.phi;
        
    end
    toc

    % plot
    f544 = figure(544); clf
    subplot(2,1,1)
    hold on
    grid on
    plot(rad2deg(beta_vec), r_beta_vec)
    plot(rad2deg(beta_vec), r_tan_vec, ':')
    plot(rad2deg(beta_vec), ema_vec)
%     plot(rad2deg([beta_vec(1),beta_vec(end)]),...
%         [ema_vec(1),ema_vec(end)], '--k')
%     plot(rad2deg(beta_vec_cor), ema_vec_cor)
%     plot(rad2deg([beta_vec_cor(1),beta_vec_cor(end)]),...
%         [ema_vec_cor(1),ema_vec_cor(end)], '--k')
    legend('radius @ beta', 'radius @ tangent', 'effective moment arm',...
        'Location','Best')
    xlabel('Joint Angle \theta (deg)')
    ylabel('Length (mm)')

    subplot(2,1,2)
    hold on
    grid on
    plot(rad2deg(beta_vec), phi_vec)
end


%%
if analysis == 3
    beta_vec = param.beta_vec;
    n_beta = length(beta_vec);
    
    if isfile(filename) % check data already exists
        load(filename)
        if ((cam_param_saved.d == param.d) &&... % check parameters are the same
            (isequal(cam_param_saved.phi_range, param.phi_range)) &&...
			(isequal(cam_param_saved.beta_vec, param.beta_vec)) == 1)
        
			cam_map = cam_map_saved;
        else
	    	cam_map = containers.Map;
        end
    else
        cam_map = containers.Map;
    end

    tic
    for idx_sweep = 1:n % loop over sweep vector
        param.rad0 = sweep_vec(idx_sweep,1); % (cm) cam radius at zero degrees
        param.slope = sweep_vec(idx_sweep,2); % (cm/rad) cam profile radius slope
        
        key = [num2str(param.rad0),',',num2str(param.slope)]; % map key
        if ~cam_map.isKey(key) % if key doesn't already exist, calculate data

	        ema_vec = zeros(1,n_beta);
	        phi_vec = zeros(1,n_beta);
	        r_tan_vec = zeros(1,n_beta);
	        parfor idx_beta = 1:n_beta
	            beta = beta_vec(idx_beta);
	            [ema_vec(idx_beta),r_tan_vec(idx_beta),phi_vec(idx_beta),~] = calc_ema(beta,param);
	        end
                
        	cam_map(key) = ema_vec;
        end
        
        fprintf('%i of %i, %4.1f elapsed\n', idx_sweep, n, toc)
    end
    
    cam_map_saved = cam_map;
    cam_param_saved = param;
%     save(filename, 'cam_map_saved', 'cam_param_saved') % save data
end




%% Functions
function plotCam(beta, cam, knee, geom)
    ema = geom.ema;
%     l_disp = geom.l_disp;
    r_tan = geom.r_tan;
    phi = geom.phi;
    psi = geom.psi;    

    alpha = pi/2 - psi; % alpha angle

    % calculate tangent point location
    xc_tan = r_tan*sin(phi); % tangent point location in cam frame
    yc_tan = r_tan*cos(phi); % tangent point location in cam frame
    x_tan = xc_tan*cos(beta) - yc_tan*sin(beta); % tangent point location in global frame
    y_tan = xc_tan*sin(beta) + yc_tan*cos(beta); % tangent point location in global frame

    % calculate cam radius at joint angle
    r_beta = knee.rad0 + knee.slope*(beta); % radius at joint angle
    x_beta = 0; % point corresponding to beta in global frame
    y_beta = r_beta; % point corresponding to beta in global frame

    % calculate cam profile
    phi_vec = deg2rad(0:0.1:180); % phi_vec parameterizes cam profile
    r_vec = knee.rad0 + knee.slope*phi_vec; % cam radius vector
    
    idx_left = (r_vec >= 0);
    phi_vec = phi_vec(idx_left);
    r_vec = r_vec(idx_left);
    
    xc_vec = r_vec.*sin(phi_vec); % cam profile in cam frame
    yc_vec = r_vec.*cos(phi_vec); % cam profile in cam frame
    x_vec = xc_vec*cos(beta) - yc_vec*sin(beta); % cam profile in global frame
    y_vec = xc_vec*sin(beta) + yc_vec*cos(beta); % cam profile in global frame

    % plot
    f = figure(543); clf
    f.Position = [139, 370, 990, 530];
    hold on
    grid on
    axis equal
%     set(gca,'visible','off')
    colors = get(gca, 'colororder');    
    xlim([-10 10])
    ylim([-10 10])
    lwidth_hardware = 2.5;
    lwidth_line = 1.25;

    % plot hardware
    h_cam = plot(x_vec,y_vec, 'LineWidth', lwidth_hardware,...
        'Color', colors(1,:)); % plot cam profile
    h_cam_edge = plot([x_vec(1),x_vec(end)], [y_vec(1),y_vec(end)], 'Color', h_cam.Color,...
        'LineWidth', lwidth_hardware); % plot cam edge
    patch([h_cam.XData], [h_cam.YData], h_cam.Color,...
        'EdgeColor', 'none', 'FaceAlpha', 0.1)
    h_link = plot([-cam.d,0], [0,0], 'k', 'LineWidth', lwidth_hardware); % plot link
    h_muscle = plot([-cam.d,x_tan], [0,y_tan], 'LineWidth', lwidth_hardware); % plot muscle-tendon unit
    
    % plot cam coordinate y-axis
    l_yc = 4;
    plot([0, l_yc*cos(beta+pi/2)], [0, l_yc*sin(beta+pi/2)], 'Color',...
        h_cam.Color, 'LineWidth', lwidth_line)

    % plot radial lines
%     h_rad_beta = plot([0,x_beta], [0,y_beta], 'LineWidth', 1.5); % plot radius corresponding to joint angle
    h_rad_tangent = plot([0,x_tan], [0,y_tan], 'LineWidth', lwidth_line,...
        'Color', colors(2,:)); % plot cam radius corresponding to tangent point
    h_ema = plot([0,-ema*cos(alpha)], [0, ema*sin(alpha)], 'LineWidth',...
        lwidth_line, 'Color', colors(5,:));
    plot(x_tan,y_tan,'.','MarkerSize',30, 'Color', colors(2,:))
    plot(0,0,'.k','MarkerSize',30)
%     legend([h_rad_beta,h_rad_tangent,h_ema],'radius @ beta','radius @ tangent',...
%         'effective moment arm','Location', 'NorthWest')

    % plot tangent point cam coordinates
    plot([0,xc_tan*cos(beta)], [0,xc_tan*sin(beta)], '--', 'Color', h_cam.Color)
    plot([xc_tan*cos(beta), xc_tan*cos(beta)+yc_tan*cos(beta+pi/2)],...
        [xc_tan*sin(beta), xc_tan*sin(beta)+yc_tan*sin(beta+pi/2)], '--','Color', h_cam.Color)
    

    
    % plot tangent point global coordinates
    plot([0,x_tan], [0,0], '--k')
    plot([x_tan, x_tan], [0, y_tan], '--k')
end



function [ema,r_tan,phi,psi] = calc_ema(beta,param)    
    % parameters
    d = param.d; % (cm) link length
    m = param.slope; % (cm/rad) cam profile radius slope
    r0 = param.rad0; % (cm) cam radius at zero degrees
    phi_range = param.phi_range; % (rad) cam angle range for which cam geometry exists
    phi_sym = param.phi_sym; % symbolic variable for solving
    
    % formulate slopes
    slope_cam = ( m*cos(beta-phi_sym) + (r0+m*phi_sym)*sin(beta-phi_sym) ) /...
        ( (r0+m*phi_sym)*cos(beta-phi_sym) - m*sin(beta-phi_sym) );
%     gamma = atan(m/(r0 + m*phi_sym));
%     slope_cam = tan(gamma + beta - phi_sym);
    
    slope_muscle = -( (r0+m*phi_sym)*cos(beta-phi_sym) ) /...
        (-d + (r0+m*phi_sym)*sin(beta-phi_sym) );
    
    % solve for tangent angle
    phi_solve = zeros(3,1);
    phi_solve(1) = max([-100,vpasolve(slope_cam == slope_muscle, phi_sym, deg2rad(0))]);
    phi_solve(2) = max([-100,vpasolve(slope_cam == slope_muscle, phi_sym, deg2rad(90))]);
    phi_solve(3) = max([-100,vpasolve(slope_cam == slope_muscle, phi_sym, deg2rad(180))]);
    
    % filter out negative tangent radii (not physically possible)
    r_tan_solve = r0 + m*phi_solve; % radius at tangent point
    idx = r_tan_solve >= 0;
    phi_solve = phi_solve(idx);

    % remove negative slopes
    slope_solve = -( (r0+m*phi_solve).*cos(beta-phi_solve) ) ./...
        (-d + (r0+m*phi_solve).*sin(beta-phi_solve) );
    idx = slope_solve >= 0;
    phi_solve = phi_solve(idx);
    slope_solve = slope_solve(idx);
    
    % filter by max slope
    [~,idx] = max(slope_solve);
    phi = phi_solve(idx); % cam parameterization angle at tangent point 
    slope = slope_solve(idx); % slope at tangent point

    % adjust for tangent points outside cam parameterization range
    if phi < phi_range(1)
        phi = phi_range(1);
        slope = -( (r0+m*phi)*cos(beta-phi) ) / (-d + (r0+m*phi)*sin(beta-phi) );
    elseif phi > phi_range(2)
        phi = phi_range(2);
        slope = -( (r0+m*phi)*cos(beta-phi) ) / (-d + (r0+m*phi)*sin(beta-phi) );
    end

    % calculate final values
    r_tan = r0 + m*phi; % radius at tangent point
    psi = atan(slope); % psi angle
    ema = d*sin(psi); % effective moment arm
    
    % if no physically possible values found
    if isempty(ema)
        ema = 0;
        r_tan = 0;
        phi = 0;
        psi = 0;
    end
end


