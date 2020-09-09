function calculateCamData(obj, sweep_arr)
    % calculate effective moment arm for all cam geometries
    % not previously saved; sweep_vec = [radius0, slope]
    fprintf('calculating cam data...\n')
    d = obj.cam_param.d;
    phi_range = obj.cam_param.phi_range;

%     obj.load_cam_data(); % re-load cam data (for parallel i think) TODO: required if saving is an option?
    s = size(sweep_arr); % get sweep array size
    n_sweep = s(1); % get sweep array length (rows)
    beta_vec = obj.cam_param.beta_vec;
    n_beta = length(beta_vec);
    tic
    for idx_sweep = 1:n_sweep % loop over sweep vector
        cam_rad0_tmp = sweep_arr(idx_sweep,1); % (cm) cam radius at zero degrees
        cam_slope_tmp = sweep_arr(idx_sweep,2); % (cm/rad) cam profile radius slope
        key = obj.cam_map_key(cam_rad0_tmp, cam_slope_tmp);

        if ~obj.cam_map.isKey(key) % if key doesn't already exist, calculate data
            ema_vec = zeros(1,n_beta);
            linear_displace_vec = zeros(1,n_beta);

            calc_ema = @(beta,r0,m,d,phi_range)obj.calc_ema(beta,r0,m,d,phi_range); % create anonymous fcn for parfor
            parfor idx_beta = 1:n_beta % loop over joint angles TODO: change back to parfor
                beta = beta_vec(idx_beta);
                geom = calc_ema(beta,cam_rad0_tmp,cam_slope_tmp,d,phi_range); % calculate cam geometry
                ema_vec(idx_beta) = geom.ema; % add effective moment arm
                linear_displace_vec(idx_beta) = geom.l_disp; % add linear joint displacement                         
            end

            obj.cam_map(key) = {ema_vec, linear_displace_vec}; % update temp cam map

            fprintf('    updated %i of %i cam profiles, %4.1f elapsed\n',...
                idx_sweep, n_sweep, toc)
        end
    end
    fprintf('all cam profiles updated, %0.2f s elapsed\n\n', toc)

    %TODO: make saving an option?
    cam_map_saved = obj.cam_map; % create cam map save var
    cam_param_saved = obj.cam_param; % create param save var
    save(obj.cam_param.filename, 'cam_map_saved', 'cam_param_saved') % save cam map & params
end
