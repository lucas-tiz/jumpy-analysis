function mdot_tube = tubeMassFlowConstMu(obj, p1, p2) %, l_tube, d_tube, eps_tube, gas_props) 

    p1 = p1*1e3; % (kPa to Pa) source pressure 
    p2 = p2*1e3; % (kPa to Pa) muscle pressure
    
    L = obj.pneu_param.l_tube_valve_musc*0.0254; % (in to m) tube length
    D = obj.pneu_param.d_tube_valve_musc*0.0254; % (in to m) tube inner diameter
    eps = obj.pneu_param.eps_tube; % (m) tube surface roughness    
    mu = interp1(obj.gas_props.pres, obj.gas_props.mu, (p2+p1)/2*1e-6, ...
        'makima'); % (Pa*s) average viscosity of air in tube (pressure in MPa)
    k = obj.gas_props.k; % (Pa-m3/kg) Rs*T

    
    %%
    s = pi*(D/2)^2;

    
    %%
%     syms mdot
% 
%     fD = (-1.8*log10((6.9*mu*D)/(4*mdot) + (eps/(3.7*D))^1.11))^-2;
% 
%     eq = mdot == sqrt(D*k*(p1^2-p2^2)/(fD*L))*s;
% 
% %     tic
%     sol = vpasolve(eq, mdot, [0 0.1]);
%     mdot_tube = double(sol);
% %     toc

    %TODO: tune grid resolution or use better binary search
    mdot_vec = linspace(0,0.1,1000);
    fD = (-1.8*log10((6.9*mu*pi*D)./(4*mdot_vec) + (eps/(3.7*D)).^1.11)).^(-2);
    mdot_dw = sqrt((p1^2-p2^2)*(D*k*s^2)./(fD*L));
    [~,idx] = min((mdot_vec - mdot_dw).^2);
    mdot_tube = mdot_vec(idx);
     
end