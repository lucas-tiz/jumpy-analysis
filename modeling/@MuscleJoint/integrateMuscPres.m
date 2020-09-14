function integrateMuscPres(obj, t, robot)
    % Integrate muscle pressure dynamics

    % update muscle pressure
%     if (t >= 0) && (t <= 0.06) 
    if (t >= 0) && (obj.state.valve == 0) 
        % if muscle activated & valve not already cycled
        [pdot, mdot] = muscPresDyn(obj, robot); % (kPa, kg/s) % integrate pres dynamics
        
        obj.state.pdot_musc = pdot;
        obj.state.mdot_musc = mdot;
        obj.state.p_musc = obj.state.p_musc + pdot*robot.sim_param.dt; % integrate muscle pressure
        obj.state.m_musc = obj.state.m_musc + mdot*robot.sim_param.dt; % integrate muscle mass
        
%         if ~isreal(obj.state.p_musc)
%             fprintf('imag\n')
%         end
%         
        obj.state.rho_musc = interp1(obj.gas_props.pres, obj.gas_props.rho, ...
            obj.state.p_musc*1e-3, 'makima')*1000; % (p: kPa to MPA) (rho: g/mL to kg/m^3) air density
        
        % check if at control pressure
        if obj.state.p_musc >= (robot.config.control.p_max + 101.325)
            obj.state.valve = 1;
        end
        
    elseif obj.state.valve == 1 % if valve has been cycled (muscle sealed)
        obj.state.mdot_musc = 0;
            
        p_musc = (obj.state.m_musc/obj.state.vol_musc)*...
            obj.gas_props.Rs*obj.gas_props.T/1000; % (Pa to kPa)  
        
        obj.state.pdot_musc = (p_musc - obj.state.p_musc)/robot.sim_param.dt;
        obj.state.p_musc = p_musc;        
        obj.state.rho_musc = interp1(obj.gas_props.pres, obj.gas_props.rho, ...
            obj.state.p_musc*1e-3, 'makima')*1000; % (p: kPa to MPA) (rho: g/mL to kg/m^3) air density
    end

    %NOTE: state is unchanged before t >= 0
    
end


    %% Muscle pressure dynamics
    function [pdot, mdot] = muscPresDyn(joint, robot)
        % Evaluate muscle pressure dynamics
    
        % calculate muscle volumetric rate of change
        cdot_musc = (joint.state.contract_musc - ...
            joint.state.contract_musc_prev)/robot.sim_param.dt;
        Vdot_musc = differentiate(joint.musc_vol_curvefit, joint.state.contract_musc)...
            *cdot_musc*1e-6; %(mL/s to m^3/s)
                
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

    
    