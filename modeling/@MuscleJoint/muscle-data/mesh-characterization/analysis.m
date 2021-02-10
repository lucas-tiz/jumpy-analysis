% Mesh stiffness
% clear
% clc

folder = ['C:\Users\Lucas\Documents\git\jumpy-analysis\modeling\'...
    '@MuscleJoint\muscle-data\mesh-characterization\mesh 1.is_tens_RawData'];
file2 = 'Specimen_RawData_2.csv';
data2 = xlsread(fullfile(folder,file2));

ext_raw = data2(:,2) - data2(1,2);
force_raw = data2(:,3) + 2.5;

% adjust for zero force intercept
ext_lim_lin = [0.2, 0.4];
idx_lim_lin = (ext_raw >= ext_lim_lin(1)) & (ext_raw <= ext_lim_lin(2));
[cfit_lin,gof] = fit(ext_raw(idx_lim_lin), force_raw(idx_lim_lin), 'poly1'); gof
ext_raw = ext_raw + cfit_lin.p2/cfit_lin.p1; % shift extension data right

% fit quadratic
ext_lim_quad = [3.5, 6];
idx_lim = (ext_raw >= ext_lim_quad(1)) & (ext_raw <= ext_lim_quad(2));
[cfit,gof] = fit(ext_raw(idx_lim), force_raw(idx_lim), 'poly2'); gof

% create interp data
ext_quad_vec = (ext_lim_quad(1):0.1:50)';
ext_lim_interp = [0.8, ext_lim_quad(1)];
idx_interp = (ext_raw >= ext_lim_interp(1)) & (ext_raw < ext_lim_interp(2));
ext_interp = [-100;
              0;
              ext_raw(idx_interp);
              ext_quad_vec;
              ext_quad_vec(end)+1];
force_interp = [cfit_lin(-100 + cfit_lin.p2/cfit_lin.p1);
                0;
                force_raw(idx_interp); 
                cfit(ext_quad_vec); 
                interp1(ext_quad_vec(end-1:end), cfit(ext_quad_vec(end-1:end)),...
                    ext_quad_vec(end)+1, 'linear', 'extrap')];

% save data
ext = ext_interp/10; % (mm to cm)
force = force_interp;
% save('mesh_stiffness_data-20cm.mat', 'ext', 'force');
force = force_interp/2;
% save('mesh_stiffness_data-40cm.mat', 'ext', 'force');



% plot
ext_vec_lin = -1:0.1:1;
ext_vec = -20:0.1:100;
figure(1); clf
hold on
grid on
idx_raw = ((ext_raw >= 0.65) & (ext_raw <= 6));
plot(ext_raw(idx_raw)/10, force_raw(idx_raw), '.', 'MarkerSize', 10)
plot(ext_raw(idx_raw)/10, force_raw(idx_raw)/2, '.', 'MarkerSize', 10)
% plot(ext_raw/10, force_raw, '--', 'LineWidth', 1.5)

% plot(ext_interp/10, force_interp, '.', 'MarkerSize', 10)
plot(ext_vec/10, interp1(ext_interp, force_interp, ext_vec, 'makima')/2, '--', 'LineWidth', 1.5)



% plot(ext_vec, cfit_series(ext_vec), '--', 'LineWidth', 1.5)
% plot(ext_vec, cfit(ext_vec)*1/2, ':', 'LineWidth', 1.5)
% plot(ext_vec, 2*cfit(ext_vec), '--', 'LineWidth', 1.5)
xlim([-0.5, 1])
xlabel('Extension (cm)')
ylabel('Force (N)')
legend('Data', 'Interp')


%% Mesh 2
file = 'Specimen_RawData_2.csv';
data = xlsread(fullfile(folder,file));

ext_raw = data2(:,2) - data2(1,2);
force_raw = data2(:,3);% + 2.5;

plot(ext_raw/10, force_raw, '.', 'MarkerSize', 10)



%% Muscle stiffness
% clear
clc

folder = ['C:\Users\Lucas\Documents\git\jumpy-analysis\modeling\'...
    '@MuscleJoint\muscle-data\mesh-characterization\muscle 1.is_tens_RawData'];
file1 = 'Specimen_RawData_1.csv';
data1 = xlsread(fullfile(folder,file1));

ext1 = (data1(:,2) - data1(1,2))/10; % (mm to cm)
force1 = data1(:,3); % (N)


ext_lim_quad = [0, 2]; %[0.1, 5];
idx_lim1 = (ext1 >= ext_lim_quad(1)) & (ext1 <= ext_lim_quad(2));



[cfit,gof] = fit(ext1(idx_lim1), force1(idx_lim1), 'poly4');
gof
% save('muscle_stiffness.mat', 'cfit');

ext_vec = 0:0.1:3;
figure(1); clf
hold on
grid on
plot(ext1(idx_lim1), force1(idx_lim1), 'LineWidth', 1.5)
plot(ext_vec, cfit(ext_vec), '--', 'LineWidth', 1.5)
% xlim([0, 30])
xlabel('Extension (cm)')
ylabel('Force (N)')
legend('Raw data', 'Fit')

