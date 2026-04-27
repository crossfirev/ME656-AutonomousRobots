function KF_populated_path = propagate_KF_path(path, cfg)
    %% Copy selected RRT nodes into KF-specific path structs (Performance Step, pruning dead data)
    data_pruned_path = initialize_KF_path(path);

    %% Collect measurement data for full path
    sensor_rich_path = collect_sensor_data(data_pruned_path, cfg);

    %% Collect KF variables for the full path
    KF_populated_path = compile_KF_vars(sensor_rich_path, cfg);
end

function kf_path = initialize_KF_path(path)
    rrt_nodes = path.path;
    kf_path = path;

    empty_kf_node = struct( ...
        'state', [], ...
        'cost', [], ...
        'action', [], ...
        'x_observability', false, ...
        'y_observability', false, ...
        'x', [], ...
        'A', [], ...
        'Q', [], ...
        'H', [], ...
        'R', [], ...
        'P_prev', [], ...
        'P', []);

    kf_nodes = repmat(empty_kf_node, size(rrt_nodes));

    for k = 1 : numel(rrt_nodes)
        kf_nodes(k).state = rrt_nodes(k).state;
        kf_nodes(k).cost = rrt_nodes(k).cost;
        kf_nodes(k).action = rrt_nodes(k).action;
    end

    kf_path.path = kf_nodes;
end

function path = compile_KF_vars(path, cfg)
    nodes = path.path;

    for k = 1 : length(nodes)
        node = nodes(k);

        %% State transition matrix : A
        dt = cfg.time_step;
        node.A = [
            1   0   dt  0;
            0   1   0   dt;
            0   0   1   0;
            0   0   0   1;
        ];

        %% Process noise covariance : Q
        node.Q = eye(4);

        %% State vector : x & Prior Posterior covariance : P_(k-1)
        if k == 1
            node.x = [
                cfg.start_state(1);
                cfg.start_state(2);
                0;
                0;
            ];
            node.P_prev = eye(4);
        else
            node = populate_state_vector(node);
            node.P_prev = nodes(k - 1).P;
        end

        %% Measurement matrix : H_k
        node.H = build_measurement_matrix(node);

        %% Measurement noise covariance : R_k
        node.R = eye(size(node.H, 1));

        %% Posterior covariance update
        P_predict = node.A * node.P_prev * node.A' + node.Q;
        H = node.H;
        R = node.R;
        node.P = inv(inv(P_predict) + H' * (R \ H));

        nodes(k) = node;
    end

    path.path = nodes;
    path.final_uncertainty = trace(nodes(end).P);
end

function node = populate_state_vector(node)
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

function H = build_measurement_matrix(node)
    H = zeros(0, 4);

    if node.x_observability
        H(end + 1, :) = [1, 0, 0, 0];
    end
    if node.y_observability
        H(end + 1, :) = [0, 1, 0, 0];
    end

    % Odometry velocity measurements are always available.
    H(end + 1, :) = [0, 0, 1, 0];
    H(end + 1, :) = [0, 0, 0, 1];
end