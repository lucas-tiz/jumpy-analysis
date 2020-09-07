function getMuscPresFromCurvefit(obj, t_valve, robot)
    % Estimate transient pressure response during muscle inflation
    % using Instron force response curve

    if isempty(obj.musc_pres_curvefit) % create pressure-time curve fit
        pres_response_path = fullfile(obj.repo_path,...
            'modeling', '@MuscleJoint', 'muscle-data', 'pressure-response',...
            'response-clean_cut_1p25D_0p19inID_quickconnect_noslack-raw_data.csv');

        [t_vec,f_vec,~,f_steady] = get_step_data(pres_response_path,...
            [1 4], 0.1, 1.5);                
        pres_vec = f_vec*(robot.config.control.p_max/f_steady); % scale curve to p_max
%         obj.musc_pres_curvefit = fit(t_vec*3.5,pres_vec,'linearinterp'); %NOTE: 3.5 used to slow response down
        obj.musc_pres_curvefit = fit(t_vec*1.0,pres_vec*1.2,'linearinterp');
    end
    
    
    if t_valve < 0
        obj.state.muscle_pressure = (14.7)*6.89476; % (psi to kPa);
%     elseif t_valve > obj.musc_pres_curvefit.p.breaks(end) % if time is beyond transient
%         pres = obj.musc_pres_curvefit.p.coefs(end,2); % set pressure to max
    elseif t_valve > 0.1 %DEBUG: if valve open longer than specified time
        
        Rs = 287.058; % (J/(kg K))
        T = 298; % (K)
        obj.state.muscle_pressure = (obj.state.m_gas/(obj.state.muscle_volume))*Rs*T/1000; % (Pa to kPa)    
        obj.state.rho_gas = interp1(obj.gas_props.pres, obj.gas_props.rho, ...
            obj.state.muscle_pressure*1e-3, 'makima')*1000; % (p: kPa to MPA) (rho: g/mL to kg/m^3)
        obj.state.mdot_gas = 0;
    else
        obj.state.muscle_pressure = obj.musc_pres_curvefit(t_valve)...
            + (14.7)*6.89476; % (psi to kPa); % set abs pressure based on curve fit
        obj.state.rho_gas = interp1(obj.gas_props.pres, obj.gas_props.rho, ...
            obj.state.muscle_pressure*1e-3, 'makima')*1000; % (p: kPa to MPA) (rho: g/mL to kg/m^3)
        m_gas = obj.state.rho_gas*obj.state.muscle_volume; % (kg/m^3 * m^3 = kg)
        obj.state.mdot_gas = (m_gas - obj.state.m_gas)/robot.sim_param.dt;
        obj.state.m_gas = m_gas;
    end


    function [t_step,f_step,t_peak,f_steady] = get_step_data(filename,...
            t_baseline_range, f_thresh, t_steady)
        % Load and modify step data
        
        % align step response start curve with t = 0
        fid = fopen(filename, 'r');
        fgets(fid);
        fgets(fid);
        d = textscan(fid, '"%f","%f"', 'Delimiter', ',');
        fclose(fid);

        t = d{:,1};
        f = d{:,2};

        truth_baseline = (t>=t_baseline_range(1) & t<=t_baseline_range(2));
        f_baseline = f(truth_baseline);
        f_baseline_avg = mean(f_baseline);

        idx_step = find(f >= f_baseline_avg + f_thresh);
        t_step = t(idx_step) - t(idx_step(1));
        f_step = f(idx_step);
        f_step(1) = 0;

        idx_steady = find(t_step >= t_steady, 1);
        f_steady = f_step(idx_steady);
        idx_peak = find(f_step >= f_steady, 1);
        t_peak = t_step(idx_peak);
    end

end