%% run_particle_filter_path.m
% Author: Matthew Lepis
% Course: ME 656 - Autonomous Navigation for Mobile Robots
% Assignment: SimLab 2
%
% File: run_particle_filter_path.m
%
% Description:
%   Runs a particle filter along one selected RRT path and stores particle
%   distributions at several path nodes for extra-credit visualization.

function particle_filter_result = run_particle_filter_path(trial, cfg)
    nodes = trial.path;
    num_nodes = numel(nodes);
    num_particles = cfg.particle_filter_num_particles;
    num_snapshots = min(cfg.particle_filter_num_snapshots, num_nodes);
    snapshot_indices = unique(round(linspace(1, num_nodes, num_snapshots)));

    particles = initialize_particles(num_particles, cfg);
    snapshot_particles = zeros(num_particles, 4, numel(snapshot_indices));
    snapshot_cursor = 1;

    for k = 1 : num_nodes
        node = nodes(k);

        if k > 1
            particles = predict_particles(particles, node, nodes(k - 1), cfg);
        end

        particles = update_and_resample_particles(particles, node, cfg);

        if snapshot_cursor <= numel(snapshot_indices) && k == snapshot_indices(snapshot_cursor)
            snapshot_particles(:, :, snapshot_cursor) = particles;
            snapshot_cursor = snapshot_cursor + 1;
        end
    end

    particle_filter_result = struct();
    particle_filter_result.trial = trial;
    particle_filter_result.snapshot_indices = snapshot_indices;
    particle_filter_result.snapshot_particles = snapshot_particles;
    particle_filter_result.num_particles = num_particles;
end

function particles = initialize_particles(num_particles, cfg)
    initial_state = [
        cfg.start_state(1), ...
        cfg.start_state(2), ...
        0, ...
        0];

    particles = repmat(initial_state, num_particles, 1) + ...
        randn(num_particles, 4) * sqrt(cfg.particle_filter_initial_variance);
end

function particles = predict_particles(particles, node, previous_node, cfg)
    action = node.action;
    if isempty(action)
        action = node.state - previous_node.state;
    end

    planned_velocity = action / cfg.time_step;
    process_std = sqrt(cfg.particle_filter_process_variance);

    particles(:, 1:2) = particles(:, 1:2) + action + ...
        randn(size(particles, 1), 2) * process_std;
    particles(:, 3:4) = repmat(planned_velocity, size(particles, 1), 1) + ...
        randn(size(particles, 1), 2) * process_std;
end

function particles = update_and_resample_particles(particles, node, cfg)
    [measurement_matrix, measurement_vector] = build_measurement_model(node, cfg);

    predicted_measurements = particles * measurement_matrix';
    residuals = predicted_measurements - measurement_vector';
    sensor_variance = cfg.particle_filter_sensor_variance;

    log_weights = -0.5 * sum((residuals .^ 2) / sensor_variance, 2);
    log_weights = log_weights - max(log_weights);
    weights = exp(log_weights);
    weights = weights / sum(weights);

    particles = systematic_resample(particles, weights);
end

function [measurement_matrix, measurement_vector] = build_measurement_model(node, cfg)
    measurement_matrix = zeros(0, 4);
    measurement_vector = zeros(0, 1);

    if node.x_observability
        measurement_matrix(end + 1, :) = [1, 0, 0, 0];
        measurement_vector(end + 1, 1) = node.state(1);
    end

    if node.y_observability
        measurement_matrix(end + 1, :) = [0, 1, 0, 0];
        measurement_vector(end + 1, 1) = node.state(2);
    end

    planned_velocity = get_planned_velocity(node, cfg);
    measurement_matrix(end + 1, :) = [0, 0, 1, 0];
    measurement_vector(end + 1, 1) = planned_velocity(1);
    measurement_matrix(end + 1, :) = [0, 0, 0, 1];
    measurement_vector(end + 1, 1) = planned_velocity(2);
end

function planned_velocity = get_planned_velocity(node, cfg)
    if isempty(node.action)
        planned_velocity = [0, 0];
    else
        planned_velocity = node.action / cfg.time_step;
    end
end

function resampled_particles = systematic_resample(particles, weights)
    num_particles = size(particles, 1);
    positions = ((0 : num_particles - 1)' + rand()) / num_particles;
    cumulative_weights = cumsum(weights);

    resampled_indices = zeros(num_particles, 1);
    particle_idx = 1;
    weight_idx = 1;

    while particle_idx <= num_particles
        if positions(particle_idx) <= cumulative_weights(weight_idx)
            resampled_indices(particle_idx) = weight_idx;
            particle_idx = particle_idx + 1;
        else
            weight_idx = weight_idx + 1;
        end
    end

    resampled_particles = particles(resampled_indices, :);
end
