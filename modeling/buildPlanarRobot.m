function robot = buildPlanarRobot(robot)
% create planar hexapod robot

%% Parameters
% robot configuration
robot.l(1) = 0.55; % (m)
robot.r(1) = robot.l(1)/2; % (m)
robot.l(2) = 0.55; % (m)
robot.r(2) = robot.l(2)/2; % (m)
robot.l(3) = 0.55; % (m)
robot.r(3) = robot.l(3)/2; % (m)
robot.l(4) = 0.55; % (m)
robot.r(4) = robot.l(4)/2; % (m)
robot.l(5) = 0.55; % (m)
robot.r(5) = robot.l(5)/2; % (m)

% robot inertia
% robot.m = [0.31, (0.23+0.05), (0.44+2*0.05 +2), (0.23+0.05), 0.31]; % (kg)
robot.m = [0.31, (0.23+0.05), (0.44+2*0.05), (0.23+0.05), 0.31]; % (kg)
r_cyl = 0.02; % (m)
robot.J(1) = 1/12*robot.m(1)*(3*r_cyl^2 + robot.l(1)^2); % (kg/m^2) 
robot.J(2) = 1/12*robot.m(2)*(3*r_cyl^2 + robot.l(2)^2); % (kg/m^2)
robot.J(3) = 1/12*robot.m(3)*(3*r_cyl^2 + robot.l(3)^2); % (kg/m^2)
robot.J(4) = 1/12*robot.m(4)*(3*r_cyl^2 + robot.l(4)^2); % (kg/m^2)
robot.J(5) = 1/12*robot.m(5)*(3*r_cyl^2 + robot.l(5)^2); % (kg/m^2)

% robot input torques
robot.tau = [0; 0; 0; 0; 0]; % actual robot joints 1-5 (1 is unactuated)


%% Model
% parameters
robot.NB = 5+2; % number of bodies (+2 for x-,y- prismatic joints at right foot)
robot.jtype = {'px','py','r','r','r','r','r'};
robot.parent = [0,1,2,3,4,5,6];
robot.gravity = [-9.81;0];

% build model
for idx_b = 1:2
    robot.Xtree{idx_b} = eye(3); % coordinate transform from parent body to predecessor frame
    robot.I{idx_b} = zeros(3);
end
    
for idx_b = 3:robot.NB % idx_b = index of body
    idx_l = idx_b - 2; % idx_l = index of physical robot link

    if idx_b == 3
        robot.Xtree{idx_b} = eye(3);
    else
        robot.Xtree{idx_b} = plnr(0, [robot.l(idx_l-1),0]);
    end
    
    mass = robot.m(idx_l);
    CoM = robot.l(idx_l)*[0.5,0];
    Icm = robot.J(idx_l);
    robot.I{idx_b} = mcI(mass, CoM, Icm);    
end

% build model appearance
robot.appearance.base = {'box', [-0.2 -0.3 -0.2; 0.2 0.3 -0.06]};
robot.appearance.body = {{}, {}};

p0 = -1;
for idx_b = 3:robot.NB
    idx_l = idx_b - 2; % idx_l = index of physical robot link

    p1 = robot.parent(idx_b);
    tap = 1^(idx_b-1);
    if p1 == 0
        ptap = 1;
    else
    ptap = 1^(p1-1);
    end
    if ( p1 > p0 )
        robot.appearance.body{idx_b} = ...
        {'cyl', [0 0 0; robot.l(idx_l) 0 0]*tap, 0.05*tap, ...
        'cyl', [0 0 -0.07; 0 0 0.07]*ptap, 0.08*ptap };
        p0 = p1;
    else
        robot.appearance.body{idx_l} = ...
        {'cyl', [0 0 0; 1 0 0]*tap, 0.05*tap };
    end
end

end

