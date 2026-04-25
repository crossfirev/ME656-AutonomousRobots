function KF_populated_path = propagate_KF_path(path, obstacles, cfg)
    %% Collect Measurement Data for full path
    sensor_rich_path = collect_sensor_data(path, obstacles, cfg);

    %% Collect KF variables for the full path
    KF_populated_path = compile_KF_vars(sensor_rich_path, cfg);
    plot_covariance_evolution(KF_populated_path)
end
function path = compile_KF_vars(path, cfg)
    function node = populate_state_vectors(node)
        x = node.state(1);
        y = node.state(2);

        if isempty(node.action)
            x_vel = 0;
            y_vel = 0;
        else
            x_vel = node.action(1);
            y_vel = node.action(2);
        end

        node.x = [
            x;
            y;
            x_vel;
            y_vel
        ];
    end
    
    nodes = sensor_rich_path.path;
    for k = 1 : nodes
        node = nodes(k);

        %% State Transition Matrix : A
        %       Constant-velocity model:
        %           x_{k+1}     = x_k     + x_vel_k * dt
        %           y_{k+1}     = y_k     + y_vel_k * dt
        %           x_vel_{k+1} = x_vel_k
        %           y_vel_{k+1} = y_vel_k
        dt = cfg.time_step;
        node.A = [
            1   0   dt  0;
            0   1   0   dt;
            0   0   1   0;
            0   0   0   1;
        ];
        %% Process noise covariance : Q
        node.Q = eye(4);

        %% State Vector : x
        if k ~= 1
            node = populate_state_vectors(node, nodes(k-1));
        else
            node.state_vector = [
                cfg.start_state(1);
                cfg.start_state(2);
                0;
                0;
            ];
        end

        %% Measurement matrix at timestep k : H_k
        if node.x_observability
            node.H(end+1, :) = [1, 0, 0, 0];
        end
        if node.y_observability
            node.H(end+1, :) = [0, 1, 0, 0];
        end
        node.H(end+1, :) = [0, 0, 1, 0]; 
        node.H(end+1, :) = [0, 0, 0, 1]; 

        %%  Measurement noise covariance at timestep k : R_k
        node.R = eye(size(node.H, 1));

        %% Previous Posterior covariance : P_(k-1)
        if k ~= 1
            node.P_prev = node.parent.P;
        else
            node.P_prev = eye(4);
        end

        A = node.A;
        Q = node.Q;
        H = node.H;
        R = node.R;
        P_predict = A * P_prev * A' + Q;
        node.P = inv(inv(P_predict) + transpose(H)*inv(R)*H);
    end
end
