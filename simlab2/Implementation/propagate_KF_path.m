function KF_populated_path = propagate_KF_path(path, cfg)
    %% Copy selected RRT nodes into KF-specific path structs (Performance Step, pruning dead data)
    data_pruned_path = initialize_KF_path(path);

    %% Collect measurement data for full path
    sensor_rich_path = collect_sensor_data(data_pruned_path, cfg);

    %% Collect KF variables for the full path
    KF_populated_path = compile_KF_vars(sensor_rich_path, cfg);

    %% Plot the error covariance evolution over the shortest path
    plot_covariance_evolution(KF_populated_path.path);
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
    path.posterior_covariance_final = nodes(end).P;
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

function plot_covariance_evolution(nodes)
    if isempty(nodes)
        return;
    end

    hold on;
    ellipse_color = [0.95, 0.65, 0.15];
    ellipse_line_width = 1.0;
    ellipse_num_points = 60;

    for k = 1 : length(nodes)
        if isempty(nodes(k).P)
            continue;
        end

        position_covariance = nodes(k).P(1:2, 1:2);
        position_covariance = (position_covariance + position_covariance') / 2;

        [eigenvectors, eigenvalues] = eig(position_covariance);
        eigenvalues = max(real(diag(eigenvalues)), 0);
        [major_variance, major_idx] = max(eigenvalues);
        minor_variance = min(eigenvalues);

        major_axis = sqrt(major_variance);
        minor_axis = sqrt(minor_variance);
        if major_axis < eps && minor_axis < eps
            continue;
        end

        major_direction = eigenvectors(:, major_idx);
        ellipse_angle = atan2(major_direction(2), major_direction(1));

        h = ellipse(major_axis, minor_axis, ellipse_angle, ...
            nodes(k).state(1), nodes(k).state(2), ellipse_color, ellipse_num_points);
        set(h, 'LineWidth', ellipse_line_width);
    end

    drawnow limitrate;
end
