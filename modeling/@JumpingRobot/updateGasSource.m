function updateGasSource(obj) 
    % Update source and muscle pressures
        
    mdot_source = 0; % reset mass flow rate out of source
    for idx_j = 1:4 % loop over index corresponding to actuated joint
        joint_fn = obj.config.model.joint_order{idx_j}; % joint name

        mdot_source = mdot_source + obj.joints.(joint_fn).state.mdot_gas; 
        % update total mass flow out of tank
    end
    
    % update source tank
    Rs = 287.058; % (J/(kg K))
    T = 298; % (K)
    obj.state.m_source = obj.state.m_source - mdot_source*obj.sim_param.dt; % (kg) source
    obj.state.p_source = (obj.state.m_source/ ...
        (obj.config.pneumatic.v_source*1e-3))*Rs*T/1000; % (Pa to kPa)    
    
end
