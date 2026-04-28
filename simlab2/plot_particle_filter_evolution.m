%% plot_particle_filter_evolution.m
% Author: Matthew Lepis
% Course: ME 656 - Autonomous Navigation for Mobile Robots
% Assignment: SimLab 2
%
% File: plot_particle_filter_evolution.m
%
% Description:
%   Plots particle filter distributions at several instants along one
%   selected path for the SimLab 2 extra-credit deliverable.

function plot_particle_filter_evolution(particle_filter_result, type, cfg)
    type = lower(char(type));
    trial = particle_filter_result.trial;
    path = trial.path;
    path_color = selection_color(type);

    figure;
    hold on;
    axis([cfg.x_bounds(1), cfg.x_bounds(2), cfg.y_bounds(1), cfg.y_bounds(2)]);
    axis equal;
    box on;

    plot_workspace(cfg);
    plot_path_line(path, path_color);
    plot_particle_snapshots(particle_filter_result);

    title(sprintf('Particle Filter Evolution: %s path', selection_label(type)));
    xlabel('x (m)');
    ylabel('y (m)');
    legend('Location', 'bestoutside');
    drawnow limitrate;
end

function plot_workspace(cfg)
    goal_region = cfg.goal_region;
    goal_x = [goal_region(1), goal_region(3), goal_region(5), goal_region(7)];
    goal_y = [goal_region(2), goal_region(4), goal_region(6), goal_region(8)];
    patch(goal_x, goal_y, [0.45, 1.00, 0.20], ...
        'EdgeColor', [0.20, 0.55, 0.10], ...
        'DisplayName', 'Goal region');

    for obs_idx = 1 : size(cfg.obstacles, 1)
        obstacle = cfg.obstacles(obs_idx, :);
        obstacle_x = [obstacle(1), obstacle(3), obstacle(5), obstacle(7)];
        obstacle_y = [obstacle(2), obstacle(4), obstacle(6), obstacle(8)];
        if obs_idx == 1
            display_name = 'Obstacles';
        else
            display_name = '';
        end
        patch(obstacle_x, obstacle_y, [0.10, 0.10, 0.90], ...
            'EdgeColor', [0.00, 0.00, 0.35], ...
            'DisplayName', display_name);
    end
end

function plot_path_line(path, path_color)
    path_states = zeros(numel(path), 2);
    for k = 1 : numel(path)
        path_states(k, :) = path(k).state;
    end

    plot(path_states(:, 1), path_states(:, 2), ...
        'Color', path_color, ...
        'LineWidth', 3, ...
        'DisplayName', 'Planned path');
end

function plot_particle_snapshots(particle_filter_result)
    snapshot_indices = particle_filter_result.snapshot_indices;
    snapshot_particles = particle_filter_result.snapshot_particles;
    snapshot_colors = parula(numel(snapshot_indices));

    for snapshot_idx = 1 : numel(snapshot_indices)
        particles = snapshot_particles(:, :, snapshot_idx);
        scatter(particles(:, 1), particles(:, 2), ...
            14, ...
            snapshot_colors(snapshot_idx, :), ...
            'filled', ...
            'MarkerFaceAlpha', 0.35, ...
            'MarkerEdgeAlpha', 0.35, ...
            'DisplayName', sprintf('Particles at node %d', snapshot_indices(snapshot_idx)));
    end
end

function label = selection_label(type)
    switch type
        case 'min'
            label = 'minimum terminal uncertainty';
        case 'max'
            label = 'maximum terminal uncertainty';
        case 'shortest'
            label = 'shortest';
        otherwise
            label = type;
    end
end
