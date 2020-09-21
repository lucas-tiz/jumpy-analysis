%
clear, clc


%% Parameters
% f.path = ['C:\Users\Lucas\Dropbox (GaTech)\Research\Hexapod\testing\',...
%     'force characterization'];
f.pre = 'clean_cut_1p25D_crimp_over_tube_sleeve';

repo_folder = 'jumpy-analysis';
current_path = pwd;
idx_repo = strfind(pwd, repo_folder);
repo_path = current_path(1:(idx_repo-1 + length(repo_folder)));

f.path = fullfile(repo_path, 'modeling', '@MuscleJoint', 'muscle-data',...
    'force-calibration');


% test setup
testArr =   [5 ,10,15,20,25,30,35,40,45,50;  % test pressures (psi)
             1 ,1 ,1 ,1 ,1 ,1 ,1 ,1 ,1 ,1 ]; % corresponding test numbers
         
% testArr =   [50;  % test pressures (psi)
%              1]; % corresponding test numbers

% mcu data parameters
timeStart = 20; % (s) mcu start time
% timeEnd = 47; % (s) end time

% exports
save_surffit = 0;

% plots
forceOptens3D.plotView = 2; % 1=3D, 2=len, 3=pres
forceOptSens3D.plot = 1;
forceOptSens3D.save = 0;


%% Process data
f.mcu = 'mcu data'; % microcontroller data folder
f.ins = 'instron data'; % instron data folder
dataCellArr = cell(1,length(testArr));
nPressures = size(testArr); % number of test pressures
nPressures = nPressures(2);
nPoints = 0; % initialize number of total data points



for i = 1:nPressures
    presTest = testArr(1,i); % test pressure
    numTest = testArr(2,i); % test number
   
    % read raw data
    fileMcu = dir(fullfile(f.path,f.mcu, [f.pre, '_',...
        num2str(presTest), 'psi_mcu_', num2str(numTest), '*.*']));
    mcu.raw = csvread(fullfile(fileMcu.folder,fileMcu.name),1);

    fileIns = dir(fullfile(f.path,f.ins, [num2str(presTest),...
        'psi', '*.*']));
    ins.raw = xlsread(fullfile(fileIns.folder,fileIns.name));
    
    % organize Instron data
    force = ins.raw(:,3); % (N)
    indTest = (force >= 1); % valid test force indices
    ins.force = force(indTest); % (N)
    ins.time = ins.raw(indTest,1); % (s)
    ins.len = -ins.raw(indTest,2)/10; % (cm)
   
    % organize valid MCU data
    timeRaw = mcu.raw(:,1); % (s) raw time data
    indRaw = 1:length(timeRaw); % raw time data indices
    trueTest = (timeRaw >= timeStart) & (timeRaw <= (timeStart+ins.time(end))); % test time logic array
    indTest = indRaw(trueTest); % test time indices
    timeTest = timeRaw(indTest); % (s) test times
    [~,indTimeTestUnique] = unique(timeTest); % indices of unique test times
    indTestUnique = indTest(indTimeTestUnique); % unique time indices
    
    mcu.time = timeRaw(indTestUnique) - timeStart; % (s) unique times
    mcu.pres = mcu.raw(indTestUnique,3)*6.89476; % (kPa) pressure data
%     mcu.optPres = mcu.raw(indTestUnique,10); % (V) optical pressure sensor data
%     mcu.optLen = mcu.raw(indTestUnique,9); % (V) optical length sensor data
    
    % interpolate MCU optical sensor data for Instron sampling times
    mcu.presInterp = interp1(mcu.time, mcu.pres, ins.time, 'linear', 'extrap');
%     mcu.optPresInterp = interp1(mcu.time, mcu.optPres, ins.time, 'linear', 'extrap');
%     mcu.optLenInterp = interp1(mcu.time, mcu.optLen, ins.time, 'linear', 'extrap');
    
%     %TODO: fake data - remove this:
%     ins.force = ins.force*3.75;
%     mcu.presInterp = mcu.presInterp*5;
    
    % record data
    dataCellArr{i} = {mcu, ins};
    nPoints = nPoints + length(ins.time); 
end

dataCellArr0 = [ {dataCellArr{1}}, dataCellArr ];
dataCellArr0{1}{1}.pres = dataCellArr0{1}{1}.pres*0;
dataCellArr0{1}{1}.presInterp = dataCellArr0{1}{1}.presInterp*0;

dataCellArr0{1}{2}.force = dataCellArr0{1}{2}.force*0;






%%
forceOptens3D.plotView = 1; % 1=3D, 2=len, 3=pres

if forceOptSens3D.plot == 1
    
    f7 = figure(7); clf
    hold on
    grid on
    xlim([-4, 12]) %xlim([0,8.0])
    ylim([0,500])
    zlim([0 700]) %zlim([0 700])
    set(f7, 'Units', 'Normalized');
    xlab = xlabel('Contraction (cm)');
    xlab.Units = 'Normalized';
    ylab = ylabel('Pressure (kPa)');
    ylab.Units = 'Normalized';
    zlabel('Contraction Force (N)')

    switch forceOptens3D.plotView
        case 1
            view(135,35) % (135, 45)
            rot = 29.5;
%             xlab.Rotation = rot;
%             xlab.Position = [0.816,0.073,0];    
%             ylab.Position = [0.170,0.079,0];
%             ylab.Rotation = -rot;
            figName = 'force_vs_cont_pres_3Dplot_';
            figPos = [-10 8 5 5];
        case 2
            view(0,0)
            xlim([0, 7])
            figName = 'force_vs_cont_flipped';
            figPos = [-10 8 4. 4.];
        case 3
            view(90,0)
            figName = 'force_vs_pres_';
            figPos = [-10 8 4. 4.];
        otherwise
            view(0,0)
    end
    
    % plot data
    legStr = cell(1, nPressures); % initialize legend strings cell array
    scatterHandle = zeros(1, nPressures);
    surfArr = zeros(nPoints,3); % initialize surface fit data array
    indData = 1; % initialize overall data index
    cont_final = zeros(nPressures, 1);
    for i = 1:nPressures
        presTest = testArr(1,i); % test pressure
        mcu = dataCellArr{i}{1}; % MCU data
        ins = dataCellArr{i}{2}; % Instron data
        scatterHandle(i) = scatter3(ins.len, mcu.presInterp, ins.force, 120, '.'); % plot points
        
        lenData = length(ins.len);
        surfArr(indData:(indData+lenData-1),1) = ins.len;
        surfArr(indData:(indData+lenData-1),2) = mcu.presInterp;
        surfArr(indData:(indData+lenData-1),3) = ins.force;
        indData = indData + lenData;
        
        cont_final(i) = ins.len(end);

        legStr{i} = [num2str(presTest), ' psi'];
        
%         csvwrite(fullfile(f.path,f.pre,'compiled data',[num2str(presTest),'psi.csv']),...
%             [ins.len, mcu.presInterp, ins.force])
    end
    minSat = 0.1;
    colorPlotLines(f7, 'standard') 
    
    % add saturation/value gradient to each set of data
    valMin = 0.6; % minimum saturation level
    ax = gca;
    for i = 1:nPressures
        ins = dataCellArr{i}{2}; % Instron data
        l = ax.Children(nPressures-i+1);
        col = l.CData;
        
        y = rgb2hsv(col); % convert RGB color to HSV
        cArr = repmat(y, length(l.XData), 1);
%         cArr(:,3) = valMin + (1-valMin)*cArr(:,3).*(ins.force/max(ins.force)); % change saturation based on force
        cArr = hsv2rgb(cArr); % convert back to RGB
%         cArr = flip(cArr);
        set(scatterHandle(i), 'CData', cArr) % set line colors
    end

    % create/plot surface fit
    [sf, gof] = fit([surfArr(:,1), surfArr(:,2)], surfArr(:,3), 'poly33',...
        'Lower',[0, -100*ones(1,9)]);
    
    % fit max contraction border line
    pres = [0; testArr(1,:)']*6.89476; % (kPa)
    cont0 = [0; cont_final];
    cont = zeros(size(pres));
    
    for i = 1:length(pres)
        options = optimset('Display','iter');
        cont(i) = fminsearch(@(c) surfaceBoundaryObjFun(c, pres(i), sf), cont0(i), options);
    end
%     plot3(cont,pres, zeros(size(cont)), 'Marker', '.', 'LineStyle', 'none',...
%         'MarkerSize', 30, 'Color', 'r')
    
%     max_cont_fit = fit(pres, cont, 'poly3');

    fo = fitoptions('Method','NonlinearLeastSquares',...
               'Lower',[-100, 0.01, 0.01, -100, -100],...
               'Upper',[100, 100, 100, 100, 100],...
               'StartPoint',[0,1,1,1,1]);
    ft = fittype('a + b*log(x+c) + d*x + e*x^2', 'options',fo);
    [cf, cgof] = fit(pres, cont, ft)
    
    cf_mod = cf;
    cf_mod.a = cf.a - 0.5;
    
    pres_eval = 0:1:pres(end);
%     plot3(cf(pres_eval), pres_eval, zeros(size(pres_eval)), 'LineWidth', 1.5)
    plot3(cf_mod(pres_eval), pres_eval, zeros(size(pres_eval)), 'LineWidth', 3,...
        'Color', 'r')


    
    

    if save_surffit == 1
        save(fullfile(pwd, f.pre, ['surface_fit_', f.pre, '_',...
            num2str(round(max(testArr(1,:)*6.89476))), 'kpa', '.mat']), 'sf'); % save surface fit
    end

    % shadows, surface, legend
    % legend
    legHandle = zeros(1, nPressures); % initialize legend strings cell array
    for i = 1:nPressures
        colors = get(scatterHandle(i), 'CData'); % set line colors
        legHandle(i) = plot3(3, 3, -1, '.', 'MarkerSize', 20); % plot points
        set(legHandle(i), 'Color', colors(floor(0.2*length(colors)),:))
    end
    hLeg = legend(legHandle, legStr, 'Location', 'best');
            
    switch forceOptens3D.plotView
        case 1
            % plot shadows
            for i = 1:nPressures
                mcu = dataCellArr{i}{1}; % MCU data
                ins = dataCellArr{i}{2}; % Instron data
                scatter3(ins.len, mcu.presInterp, zeros(size(ins.force))...
                    , 80, 0.9*[1, 1, 1], '.') % plot shadows
            end
            
            % plot surface
            dgc = 0.25;
            dgp = 25;
%             [xg, yg] = meshgrid(0:dgc:8, 0:dgp:350);
            [xg, yg] = meshgrid(-4:dgc:12, 0:dgp:500);
            zg = sf(xg,yg);
            colormap copper
            caxis([0 150])
            hSurf = surf(xg,yg,zg, 'EdgeColor','none','FaceAlpha',0.5);
%             hSurf = surf(xg,yg,zg, 'EdgeColor','k','FaceAlpha',0.15);

            % plot dividing line
            p_div = [0, 400];
            c_div = 6.5 + p_div/300;
%             plot3(c_div,p_div,zeros(size(p_div)), '-')
            
            hLeg = legend(legHandle, legStr, 'Location', 'northeast');
        case 2
            hLeg = legend(legHandle, legStr, 'Location', 'northeast');
        case 3
    end


    
    % print fit metrics
    fprintf('force fit metrics:  R^2: %3.3f, RMSE: %3.3f, SSE: %3.1f \n', gof.rsquare, gof.rmse,...
        gof.sse)

    % print coefficient values
    coeffs = coeffvalues(sf);
    for i = 1:(length(coeffs)-1)
       fprintf('%3.4f, ', coeffs(i)); 
    end
    fprintf('%3.4f\n', coeffs(end)); 
    
    % print coefficient names
    names = coeffnames(sf);
    for i = 1:(length(names)-1)
        fprintf('%s, ', names{i}(2:end))
    end
    fprintf('%s\n', names{end}(2:end))
end


forceOptSens3D.save = 0;
if forceOptSens3D.save == 1
    figPos = [-26 8 6 5];
    pub_figureFormat(f7, 'CMU Serif')
    f7.PaperUnits = 'inches';
    f7.PaperPosition = figPos;
    f7.Units = 'inches';
    f7.Position = figPos;
%     print(fullfile(f.path,f.pre,[figName, f.pre]), '-dpng', '-r600')
%     export_fig 'force_vs_length.png' -transparent -r600

end


% %%
% % clear, clc
% 
% % data
% x = (0:0.2:4)';
% y = x.^2 + randn(size(x))*0.0;
% 
% % prepend zeros
% x_pre = (-4:0.2:-0.2)';
% x_post = (4:0.2:8)';
% x = [x_pre; x; x_post];
% y = [zeros(size(x_pre)); y; y(end)*ones(size(x_post))];
% 
% % tahn fit
% ft1 = fittype('a*tanh(x+b) + c',...
%         'dependent',{'y'}, 'independent',{'x'},...
%         'coefficients',{'a','b','c'});
% [c1,gof1] = fit(x,y,ft1)
% y1 = c1(x);
% 
% % sigmoid fit
% ft2 = fittype('a*(1/(1+exp(-x+b))) + c',...
%         'dependent',{'y'}, 'independent',{'x'},...
%         'coefficients',{'a','b','c'});
% [c2,gof2] = fit(x,y,ft2)
% y2 = c2(x);
% 
% % plot 
% figure(1232); clf
% hold on
% grid on
% plot(x,y, '.')
% plot(x,y1)
% plot(x,y2, '--')
% 


%%




%%

if contraction < 0
    
else
    
end




%% Interp stuff
k_cont_neg = 500; % (N/cm)
x_elong = -1:1:-0.1;

x = [x_elong, 0:0.1:8.0]; % (cm) contractions
y = (0:5:50)*6.89476; % (kPa) pressures
[X,Y] = meshgrid(x,y); 
Z = zeros(size(X)); % (N) force

Xs = [];
Ys = [];
Zs = [];


% dataArr = [];
dataCell = {};

% add negative pressure data
cont = (0:0.1:10)';
pres = ones(size(cont))*-50;
force = zeros(size(cont));
% dataArr = [dataArr; cont, pres, force];
dataCell{1} = [cont, pres, force];

% add zero pressure data
cont = (0:0.1:10)';
pres = zeros(size(cont));
force = zeros(size(cont));
% dataArr = [dataArr; cont, pres, force];
dataCell{2} = [cont, pres, force];

% add test data
for i = (1:length(dataCellArr))
    mcu = dataCellArr{i}{1}; % MCU data
    ins = dataCellArr{i}{2}; % Instron data
    
    cont = ins.len;
    pres = mcu.presInterp;
    force = ins.force;
    
%     dataArr = [dataArr; cont, pres, force];
    dataCell{i+2} = [cont, pres, force];
end

% add negative and additional large contraction data
cont_neg = (-1:1:-0.1)'; % (cm)
k_cont_neg = 500; % (N/cm)
cont_add = 2; % (cm)

pres_nom = [-50, 0, testArr(1,:)*6.89476];

dataArr = [];
for i = 1:length(dataCell)
    cont = dataCell{i}(:,1);
    pres = dataCell{i}(:,2);
    force = dataCell{i}(:,3);
    
    
    cont = [cont_neg; cont; cont(end) + cont_add];
    pres = [pres_nom(i)*ones(size(cont_neg)); pres; pres_nom(i)];
    force = [force(1) - cont_neg*k_cont_neg; force; 0];
    
    dataArr = [dataArr; cont, pres, force];
end


f8 = figure(8); clf
hold on
grid on
xlabel('Contraction (cm)')
ylabel('Pressure (psi)')
zlabel('Force (N)')
xlim([-4,12]) %xlim([0,8.0])
zlim([-100 700]) %zlim([0 700])
view(135,35) % (135, 45)
% surf(X,Y,Z)
% scatter3(Xs,Ys,Zs)
scatter3(dataArr(:,1),dataArr(:,2),dataArr(:,3))


%% Interp stuff 2
k_elong = 200; % (N/cm)

% cont_lims = [-1,8]; % (cm)
cont_vec = (-1:0.5:8)'; % (cm)
pres_vec = ([-5, 0, testArr(1,:), 60]*6.89476)'; % (kPa)

[X,Y] = meshgrid(cont_vec, pres_vec);
Z = zeros(size(X)); % (N) force

f_outside = 0;

dataCell = {};
% add negative pressure data
cont = (0:1:4)';
pres = pres_vec(1)*ones(size(cont));
force = [0; ones(length(cont)-1,1)*f_outside];
dataCell{1} = [cont, pres, force];

% add zero pressure data
cont = (0:1:4)';
pres = pres_vec(2)*ones(size(cont));
force = [0; ones(length(cont)-1,1)*f_outside];
dataCell{2} = [cont, pres, force];

% add test data
for i = 1:length(dataCellArr)
    mcu = dataCellArr{i}{1}; % MCU data
    ins = dataCellArr{i}{2}; % Instron data
    
    cont = ins.len;
    pres = mcu.presInterp;
    force = ins.force;
    
    dataCell{i+2} = [cont, pres, force];
end

% add large pressure data (constant)
dataCell{i+3} = dataCell{i+2};

% add negative contraction and large contraction data
dataArr = [];
dataArrGrid = [];
for i = 1:length(dataCell)
    cont = dataCell{i}(:,1);
    pres = dataCell{i}(:,2);
    force = dataCell{i}(:,3);
    
    cont_pre = cont_vec(cont_vec < 0);
    cont_post = cont_vec(cont_vec > cont(end));
    
    cont = [cont_pre; cont; cont_post];
    if i > 2
        force = [force(1) - k_elong*cont_pre; force; 0; ones(length(cont_post)-1,1)*f_outside];
    else
        force = [force(1) - k_elong*cont_pre; force; ones(length(cont_post),1)*f_outside];
    end
    
    cont_grid = cont_vec;
    pres_grid = pres_vec(i)*ones(size(cont_vec));
    force_grid = interp1(cont, force, cont_vec, 'pchip');
    Z(i,:) = force_grid;
    
    dataArrGrid = [dataArrGrid; cont_grid, pres_grid, force_grid];
end

% fix contraction = 0 & pres = 0 data point
% dataArrGrid((dataArrGrid(:,1)) == 0 & (dataArrGrid(:,2)) == 0, 3) = 0;
% Z((X == 0) & (Y == 0)) = 0;

% save('force_makima_interp_data.mat', 'X','Y','Z')


% interp2 surface
[Xint,Yint] = meshgrid(-1:0.1:8, 0:10:500);

Zint = interp2(X, Y, Z, Xint, Yint, 'pchip');


f8 = figure(8); clf
hold on
grid on
xlabel('Contraction (cm)')
ylabel('Pressure (kPa)')
zlabel('Force (N)')
xlim([-2,12]) %xlim([0,8.0])
ylim([0 500])
zlim([0 1000]) %zlim([0 700])
view(135,35) % (135, 45)
% surf(X,Y,Z)
% scatter3(Xs,Ys,Zs)
% scatter3(dataArr(:,1),dataArr(:,2),dataArr(:,3))


colors = get(gca, 'ColorOrder');
scatter3(dataArrGrid(:,1),dataArrGrid(:,2),dataArrGrid(:,3), 200, '.',...
    'MarkerEdgeColor', colors(2,:))


colormap copper
caxis([0 150])
% surf(Xint,Yint,Zint, 'EdgeColor','none','FaceAlpha',0.5);


% show zero-force line
y_min = (0:1:500)';
x_min = zeros(length(y_min),1);
z_min = zeros(length(y_min),1);
for i = 1:length(y_min)
    y = y_min(i);
    x_vec = 0:0.01:8;
    z_vec = interp2(X, Y, Z, x_vec, y, 'makima');
    
    idx = find(z_vec<1.0, 1, 'first');
    x_min(i) = x_vec(idx);
    z_min(i) = z_vec(idx);
end
scatter3(x_min,y_min,z_min, '.g')


forceOptSens3D.save = 0;
if forceOptSens3D.save == 1
    figPos = [-26 2 6 5];
    pub_figureFormat(f8, 'CMU Serif')
    f8.PaperUnits = 'inches';
    f8.PaperPosition = figPos;
    f8.Units = 'inches';
    f8.Position = figPos;
%     print(fullfile(f.path,f.pre,[figName, f.pre]), '-dpng', '-r600')
%     export_fig 'force_interp_global.png' -transparent -r600

end


%% Interp stuff 3
k_elong = 200; % (N/cm)

cont_vec = (-1:0.5:8)'; % (cm)
pres_vec = ([-5, 0, testArr(1,:), 60]*6.89476)'; % (kPa)

[X,Y] = meshgrid(cont_vec, pres_vec);
Z = zeros(size(X)); % (N) force


dataCell = {};
% add negative pressure data
cont = 0; %(0:1:4)';
pres = pres_vec(1)*ones(size(cont));
force = zeros(length(cont),1);
dataCell{1} = [cont, pres, force];

% add zero pressure data
cont = 0; %(0:1:4)';
pres = pres_vec(2)*ones(size(cont));
force = zeros(length(cont),1);
dataCell{2} = [cont, pres, force];

% add test data
for i = 1:length(dataCellArr)
    mcu = dataCellArr{i}{1}; % MCU data
    ins = dataCellArr{i}{2}; % Instron data
    
    cont = ins.len;
    pres = mcu.presInterp;
    force = ins.force;
    
    dataCell{i+2} = [cont, pres, force];
end

% add large pressure data (constant)
dataCell{i+3} = dataCell{i+2};

% add negative contraction and large contraction data
dataArr = [];
dataArrGrid = [];
for i = 1:length(dataCell)
    cont = dataCell{i}(:,1);
    pres = dataCell{i}(:,2);
    force = dataCell{i}(:,3);
        
    cont_pre = cont_vec(cont_vec < 0);
    cont_post = cont_vec(cont_vec > cont(end));
    cont = [cont_pre; cont; cont_post];
    
    force = [force(1) - k_elong*cont_pre; force; min(-1, sf(cont_post,pres_vec(i)))];
%     force(force < 0) = 0; % remove negative forces

    cont_grid = cont_vec;
    pres_grid = pres_vec(i)*ones(size(cont_vec));
    force_grid = interp1(cont, force, cont_vec, 'makima');
    Z(i,:) = force_grid;
    
    dataArrGrid = [dataArrGrid; cont_grid, pres_grid, force_grid];
end

% save('force_makima_interp_data.mat', 'X','Y','Z')


% interp2 surface
[Xint,Yint] = meshgrid(-1:0.1:8, 0:10:500);

Zint = interp2(X, Y, Z, Xint, Yint, 'makima');


f8 = figure(8); clf
hold on
grid on
xlabel('Contraction (cm)')
ylabel('Pressure (kPa)')
zlabel('Force (N)')
xlim([-2,12]) %xlim([0,8.0])
ylim([0 500])
zlim([0 1000]) %zlim([0 700])
view(135,35) % (135, 45)
% surf(X,Y,Z)
% scatter3(Xs,Ys,Zs)
% scatter3(dataArr(:,1),dataArr(:,2),dataArr(:,3))


colors = get(gca, 'ColorOrder');
scatter3(dataArrGrid(:,1),dataArrGrid(:,2),dataArrGrid(:,3), 200, '.',...
    'MarkerEdgeColor', colors(2,:))


colormap copper
caxis([0 150])
surf(Xint,Yint,Zint, 'EdgeColor','none','FaceAlpha',0.5);


% show zero-force line
y_min = (0:1:500)';
x_min = zeros(length(y_min),1);
z_min = zeros(length(y_min),1);
for i = 1:length(y_min)
    y = y_min(i);
    x_vec = 0:0.01:8;
    z_vec = interp2(X, Y, Z, x_vec, y, 'makima');
    
    idx = find(z_vec<=0.1, 1, 'first');
    x_min(i) = x_vec(idx);
    z_min(i) = z_vec(idx);
end
scatter3(x_min,y_min,z_min, '.g')


forceOptSens3D.save = 0;
if forceOptSens3D.save == 1
    figPos = [-26 2 6 5];
    pub_figureFormat(f8, 'CMU Serif')
    f8.PaperUnits = 'inches';
    f8.PaperPosition = figPos;
    f8.Units = 'inches';
    f8.Position = figPos;
%     print(fullfile(f.path,f.pre,[figName, f.pre]), '-dpng', '-r600')
%     export_fig 'force_interp_global.png' -transparent -r600

end


%% Sigmoid stuff
% clear, clc
% folder_surf_fit = ['C:\Users\lucas\Dropbox (GaTech)\',...
%     'Research\Hexapod\testing\force characterization\',...
%     'clean_cut_1p25D_crimp_over_tube_sleeve'];
% file_surf_fit = ['surface_fit_clean_cut_1p25D_crimp_over_',...
%     'tube_sleeve_345kpa.mat'];
% surffit = load(fullfile(folder_surf_fit, file_surf_fit), 'sf'); 
% surffit = surffit.('sf');

% data
x_orig = dataCellArr{end}{2}.len;
y_orig = dataCellArr{end}{2}.force;

% prepend/append data
x_pre = linspace(-1,0,length(x_orig))';
x_post = linspace(x_orig(end), x_orig(end)+7, length(x_orig))';

x_pre_flat = linspace(-7,0,length(x_orig))';
x = [x_pre_flat; x_orig; x_post];
y_flat = [1.0*y_orig(1)*ones(size(x_pre_flat)); y_orig; zeros(size(x_post))];

x_pre_linear = linspace(-1,0,length(x_orig))';
x_linear = [x_pre_linear; x_orig; x_post];
y_linear = [y_orig(1) - 150.0*x_pre_linear; y_orig; zeros(size(x_post))];

x_fit = -10:0.1:14;


% linear fit
% [c1,gof1] = fit(x_orig,y_orig,'poly1');
% y1 = c1(x_orig);


% sigmoid fit 1
x_pre1 = linspace(-7,0,length(x_orig))';
x_post1 = linspace(x_orig(end), x_orig(end)+7, length(x_orig))';
x1 = [x_pre1; x_orig; x_post1];
y1 = [1.0*y_orig(1)*ones(size(x_pre1)); y_orig; zeros(size(x_post1))];

ft1 = fittype('a*(1/(1+exp(-x*b + c))) + d',...
        'dependent',{'y'}, 'independent',{'x'},...
        'coefficients',{'a','b','c','d'});
[c1,gof1] = fit(x1,y1,ft1, 'StartPoint',[-100,700,3.5,300])
y_fit1 = c1(x_fit);


% sigmoid fit 2
x_pre2 = linspace(-0.5,0,length(x_orig))';
x_post2 = linspace(x_orig(end), x_orig(end)+7, length(x_orig))';
x2 = [x_pre2; x_orig; x_post2];
y2 = [y_orig(1) - 150.0*x_pre2; y_orig; zeros(size(x_post2))];

ft2 = fittype('a*(1/(1+exp(-x*b + c))) + d',...
        'dependent',{'y'}, 'independent',{'x'},...
        'coefficients',{'a','b','c','d'});
[c3,gof3] = fit(x2,y2,ft2, 'StartPoint',[-100,700,3.5,300])
y_fit2 = c3(x_fit);

% % adjusted sigmoid
% c4 = c3
% c4.b = 10;
% y4 = c4(x);


% plot 
co = get(0, 'DefaultAxesColorOrder');
figure(1232); clf
hold on
grid on
plot(x_orig,y_orig, '.')
% plot(x_orig,y1, 'LineWidth', 1.5)
plot(x_fit,y_fit1, '--', 'LineWidth', 1.5)
plot(x_fit,y_fit2, '--', 'LineWidth', 1.5)
legend('50 psi test data', 'Sigmoid fit: constant @ 0 & max contraction',...
    'Sigmoid fit: linear @ 0, constant @ max contraction')
xlabel('Contraction Length (cm)')
ylabel('Contraction force (N)')


% gof1
% gof2
% gof3


%% Surface fit (manual)

contract_vec = [];
pres_vec = [];
force_vec = [];

% concatenate data
for i = 1:length(dataCellArr)
    mcu = dataCellArr{i}{1}; % MCU data
    ins = dataCellArr{i}{2}; % Instron data
    
    contract_vec = [contract_vec; ins.len];
    pres_vec = [pres_vec; mcu.presInterp]; %[pres_vec; ones(size(ins.len)*);
    force_vec = [force_vec; ins.force];
end

% fit
p0 = randn(9,1);
% p0 = [1.11; 2.22; -0.95; -0.45; -0.00; 0.17; 0.02; 0.00; -0.00];

options = optimset('Display','iter', 'MaxFunEvals',4000);
p = fminsearch(@(p) surfFitObjFun(p,contract_vec,pres_vec,force_vec), p0, options);
% p=p0;

% evaluate
dgc = 0.25;
dgp = 25;
[xg, yg] = meshgrid(-4:dgc:12, 0:dgp:350);
zg = p(1)*xg + p(2)*yg + p(3)*xg.^2 + p(4)*xg.*yg + p(5)*yg.^2 + p(6)*xg.^3 + ...
                  p(7)*xg.^2.*yg + p(8)*xg.*yg.^2 + p(9)*yg.^3;

f9 = figure(9); clf          
hold on
grid on
xlabel('Contraction (cm)')
ylabel('Pressure (kPa)')
zlabel('Force (N)')
xlim([-4,12]) %xlim([0,8.0])
zlim([0 700]) %zlim([0 700])
view(135,35) % (135, 45)
colormap copper
caxis([0 150])
surf(xg,yg,zg, 'EdgeColor','none','FaceAlpha',0.5);


%%

function cost = surfFitObjFun(p, contract,pres,force)

    x = contract;
    y = pres;

    cost = (p(1)*x + p(2)*y + p(3)*x.^2 + p(4)*x.*y + p(5)*y.^2 + p(6)*x.^3 + ...
                  p(7)*x.^2.*y + p(8)*x.*y.^2 + p(9)*y.^3 - force);
              
    cost = cost'*cost;

end


function cost = surfaceBoundaryObjFun(c, pressure, sf)
    cost = (sf(c,pressure))^2;
end




