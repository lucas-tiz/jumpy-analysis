f11 = figure(11); clf

% plot range
[x,p] = meshgrid(-3:0.1:20, -20: 1: 400);

% parameters
p00 =      -17.39;
p10 =        1.11;
p01 =        2.22;
p20 =     -0.9486;
p11 =     -0.4481;
p02 =  -0.0003159;
p30 =      0.1745;
p21 =     0.01601;
p12 =   0.0001081;
p03 =  -7.703e-07;
surface = p00 + p10*x + p01*p + p20*x.^2 + p11*x.*p + p02*p.^2 + p30*x.^3 + p21*x.^2.*p + p12*x.*p.^2 + p03*p.^3;

% clip the zero values
surface(surface<0) = 0;
surface(x>8) = 0;
surface(x>6.5 & p<100) = 0;
surface(x>0 & p<0) = 0;

% use a slope for all x < 0
% for x<0: surface (x, p) = surface(0, p) - slope * x
slope = 200;
zero_index = find(x(1,:) == 0);
x0_col = surface(x==0);
x0 = repmat(x0_col, 1, zero_index);
surface(:, 1:zero_index) = x0 - slope * x(:,1:zero_index);
surf(x, p, surface)
xlabel('contraction');
ylabel('pressure');
zlabel('force');