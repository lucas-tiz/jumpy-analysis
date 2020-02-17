function force = interpForce(contraction, pressure)

    persistent data

    if isempty(data)
        fprintf('loading...\n')
    	data = load('force_makima_interp_data.mat', 'X','Y','Z');
    end

    force = interp2(data.X, data.Y, data.Z, contraction, pressure, 'makima');

end


