function integrateMuscPres(obj, t, t_max, p_max, robot)
    % Iteratively integrate muscle pressure & update muscle contraction
    % together

%     if (obj.joint_param.rad0 < 6) && (t >= 0) %DEBUG
%         fprintf('pause\n')
%     end
    
    % loop: update muscle pressure, update muscle volume, iterate until
    % pressure convergence
    err = 1e3;
    i = 1;
    lim_iter = 50;
    while (err > 1e0) && (i < lim_iter)
    
        % update muscle pressure
        if (t >= 0) && (obj.state.valve == 0) % if muscle activated & valve not already cycled
            [pdot_musc, mdot_musc] = muscPresDyn(obj, robot, t); % (kPa, kg/s) % integrate pres dynamics

            p_musc = obj.state.p_musc + pdot_musc*robot.sim_param.dt; % integrate muscle pressure
            m_musc = (p_musc*obj.state.vol_musc)/...
                (obj.gas_props.Rs*obj.gas_props.T/1000); % (Pa to kPa)  
            
%             m_musc = obj.state.m_musc + mdot_musc*robot.sim_param.dt; % integrate muscle mass
%             p_musc = (m_musc/obj.state.vol_musc)*...
%                 obj.gas_props.Rs*obj.gas_props.T/1000; % (Pa to kPa)  

            rho_musc = interp1(obj.gas_props.pres, obj.gas_props.rho, ...
                    p_musc*1e-3, 'makima')*1000; % (p: kPa to MPA) (rho: g/mL to kg/m^3) air density

            % check if at max pressure or valve close time
            if (p_musc >= (p_max + 101.325)) || (t >= t_max)
                valve_state = 1;
            else
                valve_state = 0;
            end

        else % if valve not yet open or valve has been cycled (muscle sealed)
            mdot_musc = 0;
            m_musc = obj.state.m_musc;

            p_musc = (obj.state.m_musc/obj.state.vol_musc)*...
                obj.gas_props.Rs*obj.gas_props.T/1000; % (Pa to kPa)  

            pdot_musc = (p_musc - obj.state.p_musc)/robot.sim_param.dt;
            rho_musc = interp1(obj.gas_props.pres, obj.gas_props.rho, ...
                p_musc*1e-3, 'makima')*1000; % (p: kPa to MPA) (rho: g/mL to kg/m^3) air density            
            
            valve_state = obj.state.valve; % valve state remains the same
        end

        obj.updateMtu(p_musc) % calc muscle contraction/force
    
%         % calculate mean volume for next iteration
%         p_musc_next = (m_musc/obj.state.vol_musc)*...
%             obj.gas_props.Rs*obj.gas_props.T/1000; % (Pa to kPa) calc pres based on new vol
%         p_musc_mean = (p_musc + p_musc_next)/2; % (kPa) calc mean pres
%         obj.state.vol_musc = (m_musc/p_musc_mean)*...
%             obj.gas_props.Rs*obj.gas_props.T/1000; % (Pa to kPa) calc mean vol
%         err = abs(p_musc - p_musc_next);
        err = 0; %abs(p_musc - p_musc_next);
        
%         if i == (lim_iter-1) %DEBUG
%             fprintf('%f %i\n', t, i)
%         end
        i = i + 1;
    end
    
    % update muscle fluid state
    obj.state.pdot_musc = pdot_musc;
    obj.state.mdot_musc = mdot_musc;
    obj.state.p_musc = p_musc;
    obj.state.m_musc = m_musc;
    obj.state.rho_musc = rho_musc;
    obj.state.valve = valve_state;
    
    %NOTE: state is unchanged before t >= 0
end


% function integrateMuscPres(obj, t, robot)
%     % Integrate muscle pressure dynamics
% 
%     % update muscle pressure
% %     if (t >= 0) && (t <= 0.06) 
%     if (t >= 0) && (obj.state.valve == 0) 
%         % if muscle activated & valve not already cycled
%         [pdot, mdot] = muscPresDyn(obj, robot, t); % (kPa, kg/s) % integrate pres dynamics
%         
%         obj.state.pdot_musc = pdot;
%         obj.state.mdot_musc = mdot;
%         obj.state.p_musc = obj.state.p_musc + pdot*robot.sim_param.dt; % integrate muscle pressure
%         obj.state.m_musc = obj.state.m_musc + mdot*robot.sim_param.dt; % integrate muscle mass
%         
% %         obj.state.p_musc = (obj.state.m_musc/obj.state.vol_musc)*...
% %             obj.gas_props.Rs*obj.gas_props.T/1000; % (Pa to kPa)  
%         
% %         if (obj.joint_param.rad0 < 6)
% %             fprintf('pause\n')
% %         end
%         
%         obj.state.rho_musc = interp1(obj.gas_props.pres, obj.gas_props.rho, ...
%             obj.state.p_musc*1e-3, 'makima')*1000; % (p: kPa to MPA) (rho: g/mL to kg/m^3) air density
%         
%         % check if at control pressure
%         if obj.state.p_musc >= (robot.config.control.p_max + 101.325)
%             obj.state.valve = 1;
%         end
%         
%     elseif obj.state.valve == 1 % if valve has been cycled (muscle sealed)
%         obj.state.mdot_musc = 0;
%             
%         p_musc = (obj.state.m_musc/obj.state.vol_musc)*...
%             obj.gas_props.Rs*obj.gas_props.T/1000; % (Pa to kPa)  
%         
%         obj.state.pdot_musc = (p_musc - obj.state.p_musc)/robot.sim_param.dt;
%         obj.state.p_musc = p_musc;        
%         obj.state.rho_musc = interp1(obj.gas_props.pres, obj.gas_props.rho, ...
%             obj.state.p_musc*1e-3, 'makima')*1000; % (p: kPa to MPA) (rho: g/mL to kg/m^3) air density
%     end
% 
%     %NOTE: state is unchanged before t >= 0
%     
% end


    %% Muscle pressure dynamics
    function [pdot, mdot] = muscPresDyn(joint, robot, t)
        % Evaluate muscle pressure dynamics
        
        % calculate muscle volumetric rate of change
        cdot_musc = (joint.state.contract_musc - ...
            joint.state.contract_musc_prev)/robot.sim_param.dt;
        Vdot_musc = differentiate(joint.musc_vol_curvefit, joint.state.contract_musc)...
            *cdot_musc*1e-6; %(mL/s to m^3/s)
        
%         fprintf('%0.4f %e\n', t, Vdot_musc)
                
        % solve for mass flow rate into muscle
%         l_tube = robot.config.pneumatic.l_tube_valve_musc*0.0254; % (in to m) tube length
%         d_tube = robot.config.pneumatic.d_tube_valve_musc*0.0254; % (in to m) tube inner diameter
%         eps_tube = robot.config.pneumatic.eps_tube; % (m) tube surface roughness
%         n_tube_seg = robot.sim_param.n_tube_seg; % number of tube discretization segments
%         mdot = obj.fminconTubeMassFlow(robot.state.p_source, joint.state.p_musc,...
%             l_tube, d_tube, eps_tube, n_tube_seg, obj.gas_props); % ([kg/s, kg/m^3])
        mdot = joint.tubeMassFlowConstMu(robot.state.p_source, joint.state.p_musc); % (kg/s)

        % evaluate pressure dynamics
        gamma = 1.0; %TODO: try 1.4
        pdot = gamma*joint.state.p_musc*((mdot/joint.state.m_musc) - (Vdot_musc/joint.state.vol_musc));
%         fprintf('re: %f, im: %f\n\n', real(pdot), imag(pdot))
    end

    
    