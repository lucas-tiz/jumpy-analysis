function [model, morphology] = buildSpatialRobot(morphology)
% create planar hexapod robot

    %% Parameters
    % get parameters from config struct and addto config struct
    l = morphology.l;
    r = morphology.l/2;
    m = morphology.m;
    r_cyl = morphology.r_cyl;
    morphology.r_ = r;

    % calculate inertias and add to config struct
    J(1) = 1/12*m(1)*(3*r_cyl^2 + l(1)^2); % (kg/m^2) 
    J(2) = 1/12*m(2)*(3*r_cyl^2 + l(2)^2); % (kg/m^2)
    J(3) = 1/12*m(3)*(3*r_cyl^2 + l(3)^2); % (kg/m^2)
    J(4) = 1/12*m(4)*(3*r_cyl^2 + l(4)^2); % (kg/m^2)
    J(5) = 1/12*m(5)*(3*r_cyl^2 + l(5)^2); % (kg/m^2)
    morphology.J_ = J;


    %% Spatial_v2 Model
    % parameters
    model.NB = 5+2; % number of bodies (+2 for x-,y- prismatic joints at right foot)
    model.jtype = {'px','py','r','r','r','r','r'};
    model.parent = [0,1,2,3,4,5,6];
    model.gravity = [-9.81;0];

    % build model
    for idx_b = 1:2
        model.Xtree{idx_b} = eye(3); % coordinate transform from parent body to predecessor frame
        model.I{idx_b} = zeros(3);
    end

    for idx_b = 3:model.NB % idx_b = index of body
        idx_l = idx_b - 2; % idx_l = index of physical robot link

        if idx_b == 3
            model.Xtree{idx_b} = eye(3);
        else
            model.Xtree{idx_b} = plnr(0, [l(idx_l-1),0]);
        end

        mass = m(idx_l);
        CoM = r(idx_l)*[1,0];
        Icm = J(idx_l);
        model.I{idx_b} = mcI(mass, CoM, Icm);    
    end

    % build model appearance
    model.appearance.base = {'box', [-0.2 -0.3 -0.2; 0.2 0.3 -0.06]};
    model.appearance.body = {{}, {}};

    p0 = -1;
    for idx_b = 3:model.NB
        idx_l = idx_b - 2; % idx_l = index of physical robot link

        p1 = model.parent(idx_b);
        tap = 1^(idx_b-1);
        if p1 == 0
            ptap = 1;
        else
        ptap = 1^(p1-1);
        end
        if ( p1 > p0 )
            model.appearance.body{idx_b} = ...
            {'cyl', [0 0 0; l(idx_l) 0 0]*tap, 0.05*tap, ...
            'cyl', [0 0 -0.07; 0 0 0.07]*ptap, 0.08*ptap };
            p0 = p1;
        else
            model.appearance.body{idx_l} = ...
            {'cyl', [0 0 0; 1 0 0]*tap, 0.05*tap };
        end
    end

end

