clear, clc


global force_sf
sf_struct = load('surface_fit-clean_cut_1p25D_crimp_over_tube_sleeve_345kpa.mat', 'sf'); 
force_sf = sf_struct.('sf');

cont_vec = -1:0.1:10;
pres_vec = -50:10:700;

% cont_vec = 0:0.1:6;
% pres_vec = 0:10:400;

length(cont_vec)*length(pres_vec)

% interpolation
tic
for i = 1:length(cont_vec)
    for j = 1:length(pres_vec)
        cont = cont_vec(i);
        pres = pres_vec(j);
        interpForce(cont,pres);
    end
end
t1 = toc;

% surface fit
tic
for i = 1:length(cont_vec)
    for j = 1:length(pres_vec)
        cont = cont_vec(i);
        pres = pres_vec(j);
        force_sf(cont, pres);
    end
end
t2 = toc;

t1/t2


%% Test MTU force balance
clc

k_tendon = 50;
pres = 100; % (kPa)
disp = 0; % (cm)

psf = [force_sf.p30,...
   force_sf.p20, force_sf.p21,...
   force_sf.p10, force_sf.p11, force_sf.p12,...
   force_sf.p00, force_sf.p01, force_sf.p02, force_sf.p03];


cont_vec = 1;
pres_vec = -50:1:700;

% root finding alg
tic
for i = 1:length(cont_vec)
    for j = 1:length(pres_vec)
%         cont = cont_vec(i);
        pres = pres_vec(j);
        
        p = [psf(1),... % cont^3
             (psf(2) + psf(3)*pres),... % cont^2
             (psf(4) + psf(5)*pres + psf(6)*pres^2 - k_tendon),... % cont
             (psf(7) + psf(8)*pres + psf(9)*pres^2 + psf(10)*pres^3 - k_tendon*disp)]; % constant 
        r = roots(p);
        cont_musc = r(3);
    end
end
toc


% binary search
tic
cont_range = [-2 7];
for i = 1:length(cont_vec)
    for j = 1:length(pres_vec)
%         cont = cont_vec(i);
        pres = pres_vec(j);
        tol = 0.1; %TODO
        cont = binaryContSearch(cont_range, k_tendon, pres, tol);
    end
end
        toc


%% Functions



function cont_guess = binaryContSearch(cont_range, k_tendon, pres, tol)
    cont_guess = (cont_range(2) + cont_range(1))/2;

    force_muscle = interpForce(cont_guess,pres);
%     global force_sf
%     force_muscle = force_sf(cont_guess, pres);
    force_spring = cont_guess*k_tendon;

    if force_muscle >= (force_spring + tol/2)
        % search right side
        cont_range = [cont_guess cont_range(2)];
        cont_guess = binaryContSearch(cont_range, k_tendon, pres, tol);
    elseif (force_spring - tol/2) > force_muscle
        % search left side
        cont_range = [cont_range(1) cont_guess];
        cont_guess = binaryContSearch(cont_range, k_tendon, pres, tol);
    else
        return
    end
end













