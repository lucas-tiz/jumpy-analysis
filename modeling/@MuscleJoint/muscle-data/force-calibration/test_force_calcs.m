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

k_tendon = 500;
pres = 100; % (kPa)
disp = 0.5; % (cm)

psf = [force_sf.p30,...
   force_sf.p20, force_sf.p21,...
   force_sf.p10, force_sf.p11, force_sf.p12,...
   force_sf.p00, force_sf.p01, force_sf.p02, force_sf.p03];


cont_vec = 0;
pres_vec = 0; %-0:1:500;

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
        tol = 1; %TODO
        cont = binaryContSearch(cont_range, k_tendon, pres, disp, tol);
    end
end
toc


% simple search
tic
cont_guess = -1:0.01:8;
err_vec = zeros(size(pres_vec));
n = 1;
for i = 1:length(cont_vec)
    for j = 1:length(pres_vec)
        pres = pres_vec(j);
        
        force_muscle = interpForce(cont_guess,pres);
        force_spring = cont_guess*k_tendon + disp*k_tendon;
    
        [err,idx] = min((force_muscle - force_spring).^2);
        err_vec(n) = err;
        cont_musc_simple = cont_guess(idx);
        
        n = n + 1;
    end
end
toc
[val, idx] = max(err_vec);
sqrt(val)
sqrt(mean(err_vec))


figure(31); clf; hold on; grid on;
plot(cont_guess, interpForce(cont_guess,pres_vec(idx)))
plot(cont_guess, cont_guess*k_tendon + disp*k_tendon)
ylim([-10 700])



%%



%% Functions


function cont_guess = binaryContSearch(cont_range, k_tendon, pres, disp, tol)
    cont_guess = (cont_range(2) + cont_range(1))/2;

    force_muscle = interpForce(cont_guess,pres);
%     global force_sf
%     force_muscle = force_sf(cont_guess, pres);
    force_spring = cont_guess*k_tendon + disp*k_tendon;

    if force_muscle >= (force_spring + tol/2)
        % search right side
        cont_range = [cont_guess cont_range(2)];
        cont_guess = binaryContSearch(cont_range, k_tendon, pres, disp, tol);
    elseif (force_spring - tol/2) > force_muscle
        % search left side
        cont_range = [cont_range(1) cont_guess];
        cont_guess = binaryContSearch(cont_range, k_tendon, pres, disp, tol);
    else
        return
    end
end













