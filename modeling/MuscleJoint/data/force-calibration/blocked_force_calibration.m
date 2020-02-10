%
clear, clc


%% Parameters
f.path = ['C:\Users\Lucas\Dropbox (GaTech)\Research\Thesis\analysis\muscle',...
    ' force analysis\force calibration data'];
f.dir = 'A1';

% test setup
if strcmp('A1', f.dir)
    testArr =   [5 ,10,15,20,25,30,35,40,45,50,55;  % test pressures
                 1 ,1 ,1 ,1 ,1 ,1 ,1 ,1 ,1 ,1 ,1 ]; % corresponding test numbers
end

% mcu data parameters
timeStart = 10; % (s) mcu start time
timeEnd = 47; % (s) end time

% plots
forceOptSens3D.plot = 1;
forceOptSens3D.save = 0;


%% Process data
f.mcu = 'mcu data'; % microcontroller data folder
f.ins = ['instron data\', f.dir, '.is_comp_RawData']; % instron data folder
dataArr = cell(1,length(testArr));
nPressures = length(testArr); % number of test pressures
nPoints = 0; % initialize number of total data points

for i = 1:nPressures
    presTest = testArr(1,i); % test pressure
    numTest = testArr(2,i); % test number
   
    % read raw data
    fileMcu = dir(fullfile(f.path,f.dir,f.mcu, [f.dir, '_',...
       num2str(presTest), 'kPa_mcu_', num2str(numTest), '*.*']));
    mcu.raw = csvread(fullfile(fileMcu.folder,fileMcu.name),1);

    fileIns = dir(fullfile(f.path,f.dir,f.ins, [num2str(presTest),...
        '_', num2str(numTest), '*.*']));
    ins.raw = xlsread(fullfile(fileIns.folder,fileIns.name));
    
    % organize Instron data
    force = -ins.raw(:,3); % (N)
    indTest = (force >= 1); % valid test force indices
    ins.force = force(indTest); % (N)
    ins.time = ins.raw(indTest,1); % (s)
    ins.len = ins.raw(indTest,2)/10; % (cm)
   
    % organize valid MCU data
    timeRaw = mcu.raw(:,1); % (s) raw time data
    indRaw = 1:length(timeRaw); % raw time data indices
    trueTest = (timeRaw >= timeStart) & (timeRaw <= (timeStart+ins.time(end))); % test time logic array
    indTest = indRaw(trueTest); % test time indices
    timeTest = timeRaw(indTest); % (s) test times
    [~,indTimeTestUnique] = unique(timeTest); % indices of unique test times
    indTestUnique = indTest(indTimeTestUnique); % unique time indices
    
    mcu.time = timeRaw(indTestUnique) - timeStart; % (s) unique times
    mcu.pres = mcu.raw(indTestUnique,4)*6.89476; % (kPa) pressure data
    mcu.optPres = mcu.raw(indTestUnique,10); % (V) optical pressure sensor data
    mcu.optLen = mcu.raw(indTestUnique,9); % (V) optical length sensor data
    
    % interpolate MCU optical sensor data for Instron sampling times
    mcu.presInterp = interp1(mcu.time, mcu.pres, ins.time, 'linear', 'extrap');
    mcu.optPresInterp = interp1(mcu.time, mcu.optPres, ins.time, 'linear', 'extrap');
    mcu.optLenInterp = interp1(mcu.time, mcu.optLen, ins.time, 'linear', 'extrap');
    mcu.presInterp = interp1(mcu.time, mcu.pres, ins.time, 'linear', 'extrap');
    
    %TODO: fake data - remove this:
    ins.force = ins.force*3.75;
    mcu.presInterp = mcu.presInterp*5;
    
    % record data
    dataArr{i} = {mcu, ins};
    nPoints = nPoints + length(ins.time);
end


%%
if forceOptSens3D.plot == 1
    plotView = 1; % 1=3D, 2=len, 3=pres
    forceOptSens3D.save = 0;
    
    f7 = figure(7); clf
    hold on
    grid on
    xlim([0,4.0])
    ylim([0,300])
    zlim([0 500])
    set(f7, 'Units', 'Normalized');
    xlab = xlabel('Contraction (cm)');
    xlab.Units = 'Normalized';
    ylab = ylabel('Pressure (kPa)');
    ylab.Units = 'Normalized';
    zlabel('Contraction Force (N)')

    switch plotView
        case 1
            view(135,35) % (135, 45)
            rot = 29.5;
            xlab.Rotation = rot;
            xlab.Position = [0.816,0.073,0];    
            ylab.Position = [0.170,0.079,0];
            ylab.Rotation = -rot;
            figName = 'force_vs_optSens_3Dplot_';
            figPos = [-10 8 5 5];
        case 2
            view(180,0)
%             xlim([0.5, 3])
            figName = 'force_vs_optSens_len_';
            figPos = [-10 8 4. 4.];
        case 3
            view(90,0)
            figName = 'force_vs_optSens_pres_';
            figPos = [-10 8 4. 4.];
        otherwise
            view(0,0)
    end
    
    % plot data
    legStr = cell(1, nPressures); % initialize legend strings cell array
    scatterHandle = zeros(1, nPressures);
    surfArr = zeros(nPoints,3); % initialize surface fit data array
    indData = 1; % initialize overall data index
    for i = 1:nPressures
        presTest = testArr(1,i); % test pressure
        mcu = dataArr{i}{1}; % MCU data
        ins = dataArr{i}{2}; % Instron data
        scatterHandle(i) = scatter3(ins.len, mcu.presInterp, ins.force, 120, '.'); % plot points
        
        lenData = length(mcu.optLenInterp);
        surfArr(indData:(indData+lenData-1),1) = ins.len;
        surfArr(indData:(indData+lenData-1),2) = mcu.presInterp;
        surfArr(indData:(indData+lenData-1),3) = ins.force;
        indData = indData + lenData;

        legStr{i} = [num2str(presTest), ' kPa'];
    end
    minSat = 0.1;
    colorPlotLines(f7, 'standard') 
    
    % add saturation/value gradient to each set of data
    valMin = 0.6; % minimum saturation level
    ax = gca;
    for i = 1:nPressures
        ins = dataArr{i}{2}; % Instron data
        l = ax.Children(nPressures-i+1);
        col = l.CData;
        
        c = rgb2hsv(col); % convert RGB color to HSV
        cArr = repmat(c, length(l.XData), 1);
%         cArr(:,3) = valMin + (1-valMin)*cArr(:,3).*(ins.force/max(ins.force)); % change saturation based on force
        cArr = hsv2rgb(cArr); % convert back to RGB
%         cArr = flip(cArr);
        set(scatterHandle(i), 'CData', cArr) % set line colors
    end

    % create/plot surface fit
    [sf, gof] = fit([surfArr(:,1), surfArr(:,2)], surfArr(:,3), 'poly33');
    save(fullfile(pwd, 'force calibration data', [f.dir, '_surface_fit.mat']), 'sf'); % save surface fit

    % shadows, surface, legend
    switch plotView
        case 1
            % plot shadows
            for i = 1:nPressures
                mcu = dataArr{i}{1}; % MCU data
                ins = dataArr{i}{2}; % Instron data
                scatter3(mcu.optLenInterp, mcu.optPresInterp, zeros(size(ins.force))...
                    , 80, 0.9*[1, 1, 1], '.') % plot shadows
            end
            
            % plot surface
            dg = 0.25;
            [xg, yg] = meshgrid(0:dg:40, 0:dg:300);
            zg = sf(xg,yg);
            colormap copper
            caxis([0 150])
            hSurf = surf(xg,yg,zg, 'EdgeColor','none','FaceAlpha',0.15);

            % legend
            legHandle = zeros(1, nPressures); % initialize legend strings cell array
            for i = 1:nPressures
                colors = get(scatterHandle(i), 'CData'); % set line colors
                legHandle(i) = plot3(3, 3, -1, '.', 'MarkerSize', 20); % plot points
                set(legHandle(i), 'Color', colors(floor(0.2*length(colors)),:))
            end
%             hLeg = legend(legHandle, legStr, 'Location', 'northeast');
            
        case 2
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


if forceOptSens3D.save == 1
%     figPos = [-26 8 6 6];
    pub_figureFormat(f7)
    f7.PaperUnits = 'inches';
    f7.PaperPosition = figPos;
    f7.Units = 'inches';
    f7.Position = figPos;
    print(fullfile(f.path,f.dir,[figName, f.dir]), '-dpng', '-r600')
end
