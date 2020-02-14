clear, 



sf_struct = load('surface_fit-clean_cut_1p25D_crimp_over_tube_sleeve_345kpa.mat', 'sf'); 
sf = sf_struct.('sf');

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
        sf(cont, pres);
    end
end
t2 = toc;

t1/t2

