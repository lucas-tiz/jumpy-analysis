%
clear, clc

% PA-imide
strength_yield = 185; % (MPa)
strain_yield = 0.08; % ()

% PFA
strength_yield = 29; % (MPa)
strain_yield = 0.85; % ()

% Teflon
strength_yield = 25; % (MPa)
strain_yield = 0.70; % ()

% % Polyethersulfone
% strength_yield = 84; % (MPa)
% strain_yield = 0.066; % ()

% area_cs = 10*10; % (mm^2)
area_cs = pi*((4/8)/2)^2*645.16; % (mm^2)
len = 10; % (mm)


%%
modulus = strength_yield/strain_yield
stiff = modulus*area_cs/len

disp_max = strain_yield*len % (mm)



%%


mod_100 = 50*6894.76; % (N/m^2)
area_cs = (3.4*20) /(1000^2); % (mm^2 to m^2)
len = 2 /100; % (cm to m)

stiff = mod_100*area_cs/len /100 % (N/m to N/cm)



%% antagonistic tendon
% [knee_r, hip_r, hip_l, knee_l]
% hexapod antagonistic spring stiffness = [5, 15, 15, 5]; % (Nm/rad)

r_pulley = 2/100; % (m)

k_8 = 0.95*9.81/(10/100); % (N/m)
k_14 = 0.63*9.81/(10/100); % (N/m)


k_8*r_pulley^2
k_14*r_pulley^2