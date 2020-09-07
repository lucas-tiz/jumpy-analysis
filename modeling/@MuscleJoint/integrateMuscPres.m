function integrateMuscPres(obj, t, robot)
    % Integrate muscle pressure dynamics
    
%     % load muscle volume curvefit
%     if isempty(obj.musc_vol_curvefit) 
%         musc_vol_curvefit_path = fullfile(obj.repo_path,...
%             'modeling', '@MuscleJoint', 'muscle-data', 'volume',...
%             'volume_curve_fit-poly3-water2.mat');
%         curvefit = load(musc_vol_curvefit_path, 'fit3_water2');
%         obj.musc_vol_curvefit = curvefit.('fit3_water2');
%     end
%     
%     % load gas properties
%     if isempty(obj.gas_props)
%         gas_data_path = fullfile(obj.repo_path,...
%             'modeling', '@MuscleJoint', 'gas-data',...
%             [robot.config.pneumatic.gas, '.txt']);
%         fileId = fopen(gas_data_path);
%         (textscan(fileId, '%s',14,'Delimiter','\t\')); % header
%         rawData = textscan(fileId, '%f %f %f %f %f %f %f %f %f %f %f %f %f %s');
%         numData = cell2mat(rawData(:,1:13));
%         obj.gas_props.pres = numData(:,2); % (MPa)
%         obj.gas_props.rho = numData(:,3); % (g/mL)
%         obj.gas_props.mu = numData(:,12); % (Pa*s)
%         fclose(fileId);
%     end

%     persistent valve_state
%     if isempty(valve_state)
%         valve_state = 0;
%     end

    % update muscle pressure
    if (t >= 0) && (t <= 0.1) %(obj.state.muscle_pressure < robot.config.control.p_max) %...
%             && (valve_state ~= 2)
    % if muscle activated & below max pressure & valve not already cycled --> valve open
%         valve_state = 1;

        if (robot.state.p_source - obj.state.muscle_pressure <= (0.2*6.89476))
        % (psi to kPa) if pres diff is less than 1 psi
            %TODO: single segment sim?
            mdot = 0; % assume no flow
        else
            [pdot, mdot] = muscPresDyn(obj, robot);
            obj.state.muscle_pressure = obj.state.muscle_pressure + ...
                pdot*robot.sim_param.dt; % integrate muscle pressure
        end

    else % if valve is closed
        Rs = 287.058; % (J/(kg K))
        T = 298; % (K)
%         v_musc = obj.musc_vol_curvefit(obj.state.muscle_contract)*1e-6; % (mL to m^3) muscle vol from curvefit
        v_musc = obj.state.muscle_volume;

        obj.state.muscle_pressure = (obj.state.m_gas/ ...
            v_musc)*Rs*T/1000; % (Pa to kPa)  
              
        mdot = 0;
        
%         if valve_state == 1
%             valve_state = 2;
%         end
    end


    obj.state.mdot_gas = mdot;


    %% Muscle pressure dynamics
    function [pdot, mdot] = muscPresDyn(obj, robot)
        % Evaluate muscle pressure dynamics
    
        % parameters
        c_musc = obj.state.muscle_contract;
        cdot_musc = (obj.state.muscle_contract - ...
            obj.state.muscle_contract_prev)/robot.sim_param.dt;
        p_source = robot.state.p_source; % (kPa)
        p_musc = obj.state.muscle_pressure; % (kPa)
        l_tube = robot.config.pneumatic.l_tube_valve_musc*0.0254; % (in to m) tube length
        d_tube = robot.config.pneumatic.d_tube_valve_musc*0.0254; % (in to m) tube inner diameter
        eps_tube = robot.config.pneumatic.eps_tube; % (m) tube surface roughness
        n_tube_seg = robot.sim_param.n_tube_seg; % number of tube discretization segments
    
        % calculate muscle volume
%         V_musc = obj.musc_vol_curvefit(c_musc)*1e-6; % (mL to m^3) muscle vol from curvefit
%         V_musc = V_musc + l_tube*pi*(d_tube/2)^2; % (m^3) add tubing volume to muscle
        V_musc = obj.state.muscle_volume;
        Vdot_musc = differentiate(obj.musc_vol_curvefit, c_musc)...
            *cdot_musc*1e-6; %(mL/s to m^3/s)
        
        % calculate muscle air mass        
        rho_musc = interp1(obj.gas_props.pres, obj.gas_props.rho, ...
            p_musc*1e-3, 'makima')*1000; % (p: kPa to MPA) (rho: g/mL to kg/m^3) density of air in muscle
        m = rho_musc*V_musc; % (kg/m^3 * m^3 = kg) mass of air in muscle
        obj.state.m_gas = m; %TODO
        
        % solve for mass flow rate into muscle
%         mdot = obj.fminconTubeMassFlow(p_source, p_musc, l_tube, d_tube, ...
%             eps_tube, n_tube_seg, obj.gas_props); % ([kg/s, kg/m^3])
        mdot = obj.tubeMassFlowConstMu(p_source, p_musc);

        % evaluate pressure dynamics
        gamma = 1.0; %TODO: try 1.4
        pdot = gamma*p_musc*((mdot/m) - (Vdot_musc/V_musc));
        
    end

end

