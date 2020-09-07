function mdot_tube = tubeMassFlowConstMu(obj, p1, p2) %, l_tube, d_tube, eps_tube, gas_props) 

    p1 = p1*1e3; % (kPa to Pa) source pressure 
    p2 = p2*1e3; % (kPa to Pa) muscle pressure

    %TODO: bring these in from config............................
    L = 48*0.0254; % (in to m)
    D = 3/16*0.0254; % (in to m) diameter
    eps = 15e-6; % (m) tube surface roughness
    mu = 1.7719e-05; % (Pa-s)
    k = 1/8.7896e4; % (Pa-m3/kg) Rs*T


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