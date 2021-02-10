% calculate muscle volume vs length from water mass experimental data
clear, clc

%{
TODO:
- remove small inlet/outlet tube liquid volumes
 
%}


%% Water volume experiment
water_cont_vec = 0:1:7; % (cm) contraction
water_mass_vec = [0.409, 0.656, 0.885, 1.052, 1.165, 1.272, 1.330,...
    1.395]/9.80665; % (kg)
water_vol_vec = water_mass_vec/997*1e6; % (mL)

[fit2_water, gof2] = fit(water_cont_vec', water_vol_vec', 'poly2');
[fit3_water, gof3] = fit(water_cont_vec', water_vol_vec', 'poly3');
gof2
gof3


water_cont_vec2 = [0:1:6, 6.904]; % (cm)
water_mass_vec2 = [0.439, 0.266, 0.235, 0.167, 0.126, 0.100, 0.073, 0.047]; % (N) incremental
tmp = zeros(size(water_mass_vec2));
for i = 1:length(water_mass_vec2)
    tmp(i) = sum(water_mass_vec2(1:i));
end
water_mass_vec2 = tmp/9.80665 - 2.2/1000; % (kg) absolute
water_vol_vec2 = water_mass_vec2/997*1e6; % (mL)

[fit2_water2, gof22] = fit(water_cont_vec2', water_vol_vec2', 'poly2');
[fit3_water2, gof32] = fit(water_cont_vec2', water_vol_vec2', 'poly3');
gof22
gof32

% save('volume_curve_fit-poly3-water2.mat', 'fit3_water2')


%% Geometry volume calc
% muscle parameters
l_m_init = 21.2; % (cm) nominal muscle length %TODO: make more accurate by accounting for end cap volume
d_i = 1.6; % (cm) inner muscle diameter
d_o = 1.9; % (cm) outer muscle diameter
theta = 20; % (deg) nominal mesh angle %TODO: measure this

% muscle geometry calcs
c_i = pi*d_i; % (cm) inner circumference
c_o = pi*d_o; % (cm) outer circumference
w = l_m_init*tand(theta); % (cm) mesh fiber wrap length
musc_param.b = sqrt(w^2 + l_m_init^2); % (cm) length of helically-wound threads in muscle
musc_param.n = w/c_i; % number of thread turns around muscle


l_m = l_m_init - 0;
V_m = (musc_param.b^2*l_m - l_m^3)/(4*pi*musc_param.n^2) % (mL) muscle volume
pi*(d_i/2)^2*l_m % (mL)...


l_m_vec = l_m_init:-0.1:(l_m_init - 10);
geom_vol_vec = zeros(size(l_m_vec));
for i = 1:length(l_m_vec)
    l_m = l_m_vec(i);
    geom_vol_vec(i) = (musc_param.b^2*l_m - l_m^3)/(4*pi*musc_param.n^2); % (mL) muscle volume
end
geom_cont_vec = l_m_init - l_m_vec; % (cm) contraction

[V_m_max, idx_V_m_max] = max(geom_vol_vec); % get max
V_m_max
cont_max = geom_cont_vec(idx_V_m_max)

plot(geom_cont_vec, geom_vol_vec)
% set(gca, 'Xdir', 'reverse')
grid on



%%
colors = get(gca, 'colororder');
linewidth = 1.5;

f3 = figure(3); clf
hold on
grid on
% plot(water_cont_vec, water_vol_vec, '.', 'MarkerSize', 16, 'Color', colors(1,:))
% plot(geom_cont_vec, fit2_water(geom_cont_vec), '--', 'Color', colors(1,:), 'LineWidth', linewidth)
% plot(geom_cont_vec, fit3_water(geom_cont_vec), ':', 'Color', colors(1,:), 'LineWidth', linewidth)

plot(water_cont_vec2, water_vol_vec2, '.', 'MarkerSize', 16, 'Color', colors(2,:))
% plot(geom_cont_vec, fit2_water2(geom_cont_vec), '--', 'Color', colors(2,:), 'LineWidth', linewidth)
plot(geom_cont_vec, fit3_water2(geom_cont_vec), ':', 'Color', colors(2,:), 'LineWidth', linewidth)

% plot(geom_cont_vec, geom_vol_vec, 'Color', colors(5,:), 'LineWidth', linewidth)

xlabel('Contraction (cm)')
ylabel('Volume (mL)')
ylim([0 200])
% legend('Data', 'Poly2', 'Poly3', 'Geometric')
legend('Experimental Data', 'Polynomial Fit', 'Location', 'NorthWest')

pub_figureFormat(f3, 'CMU Serif')
print('musc_vol_fit', '-dpng', '-r300');



