function posterior_covariance_final = propagate_KF_path(path, obstacles)
    x = 0;      % State Vector
                %   x   =   [   x_pos;
                %               y_pos;
                %               north_range;
                %               west_range;
                %               south_range;
                %               east_range;
                %           ]
                %
    A = 0;      % State Transition Matrix
    H_k = 0;    % Measurement matrix at timestep k
    Q = 0;      % Process noise covariance
    R_k = 0;    % Measurement noise covariance at timestep k
    P_k1 = 0;   % Previous posterior covariance 
end

