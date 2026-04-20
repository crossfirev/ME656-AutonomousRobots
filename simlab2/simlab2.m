function simlab2()
%% SimLab 2 - Main Simulation Driver
% Author: Matthew Lepis
% Date: April 19, 2026
%
% Main simulation driver script for SimLab 2.
%
% TODO: Add description for this file before submission.
%

[start_state, goal_region, obstacles] = generate_obstacles("original");

%
% Robot Constraints
%
%   Robot is a point robot.
%   Robot has 4 laser range finders, one in each cardinal direction - with range: `beam_range`

beam_range = 4;

%
% RRT Constraints
%

growth_dist = 2; % meters

%
% Simulation Constraints
%

num_RRT_trials = 100;
x_bounds = [0, 100];
y_bounds = [0, 100];
plot_RRT_runs = true;
plot_RRT_modulo = 25;

function [x_pos_rand, y_pos_rand] = random_state()
    x_pos_rand = x_bounds(1) + (x_bounds(2) - x_bounds(1)) * rand();
    y_pos_rand = y_bounds(1) + (y_bounds(2) - y_bounds(1)) * rand();
end

function nearest_vertex = nearest_neighbor(x, tree)
    nearest_vertex = tree.vertices(1);
    for vertex_id = 1 : length(tree.vertices)
        if norm(tree.vertices(vertex_id).state - x.state) <= norm(nearest_vertex.state - x.state)
            nearest_vertex = tree.vertices(vertex_id);
        end
    end
end

function [status, x_new, u_new] = new_state(x, x_near)
    % check if `x` is valid (if it is inside an obstacle or not)
    % if valid:
    %   - status = 1
    %   - calculate the required translation/rotation required to get a linear path with length `growth_dist` from x_near toward x; set to `u_new`
    %   - make a new node and set far end of the path to `x_new`
    % if not valid:
    %   - status = 0
    %   - x_new = NaN
    %   - u_new = NaN    
    x_dir = x.state - x_near.state;
    x_dist = norm(x_dir);
    x_unit_dir = x_dir / x_dist;
    if (x_dist >= growth_dist)
        x_dist = growth_dist;
    end
    
    x_new = x_near.state + x_unit_dir * x_dist;
    x_new = TreeVertex(x_new(1), x_new(2));
    u_new = x_new.state - x_near.state;

    if collision_check_point(x_new.state(1), x_new.state(2), obstacles) == 0 && ...
        collision_check_segment(x_near.state(1), x_near.state(2), x_new.state(1), x_new.state(2), obstacles) == 0
        status = 1;
    else
        status = 0;
    end
end

function status = extend(tree, edge_color, plot_this_run)
    status = 0;
    while status ~= 1 
        [sample_x, sample_y] = random_state();

        x = TreeVertex(sample_x, sample_y);
        x_near = nearest_neighbor(x, tree);

        [success, x_new, u_new] = new_state(x, x_near);
        if success
            tree.add_vertex(x_new);
            tree.add_edge(x_near, x_new, u_new);
            if plot_this_run
                plot([x_near.state(1), x_new.state(1)], ...
                     [x_near.state(2), x_new.state(2)], ...
                     'Color', edge_color, 'LineWidth', 0.5);
                drawnow limitrate;
            end
            if collision_check_point(x_new.state(1), x_new.state(2), goal_region) == 1
                status = 1;  % Reached Goal
            else  % x_new == x
                status = 0;  % Advanced 
            end
        else
            status = -1;  % Trapped
        end
    end
end

function tree = build_RRT(x_pos_init, y_pos_init, edge_color, plot_this_run)
    tree = Tree(x_pos_init, y_pos_init);
    extend(tree, edge_color, plot_this_run);
end

function update_progress_bar(completed_runs, total_runs)
    persistent progress_background progress_fill progress_label
    
    progress_bar_position = [0.20 0.03 0.60 0.025];
    progress_label_position = [0.20 0.055 0.60 0.025];
    progress_fraction = completed_runs / total_runs;
    
    if isempty(progress_background) || ~isvalid(progress_background)
        progress_background = annotation('rectangle', progress_bar_position, ...
            'FaceColor', [0.90 0.90 0.90], 'EdgeColor', [0.25 0.25 0.25]);
        progress_fill = annotation('rectangle', ...
            [progress_bar_position(1), progress_bar_position(2), 0, progress_bar_position(4)], ...
            'FaceColor', [0.10 0.45 0.80], 'EdgeColor', 'none');
        progress_label = annotation('textbox', progress_label_position, ...
            'String', '', 'EdgeColor', 'none', ...
            'HorizontalAlignment', 'center', 'FontSize', 9);
    end
    
    progress_fill.Position = [progress_bar_position(1), progress_bar_position(2), ...
        progress_bar_position(3) * progress_fraction, progress_bar_position(4)];
    progress_label.String = sprintf('%d / %d RRT runs complete', completed_runs, total_runs);
    drawnow;
end


RRT_trials = {};
RRT_colors = parula(num_RRT_trials);
state_state = num2cell(start_state);
for trial = 1 : num_RRT_trials
    update_progress_bar(trial - 1, num_RRT_trials);
    title(sprintf('RRT run %d of %d', trial, num_RRT_trials));
    drawnow;
    plot_this_run = plot_RRT_runs && mod(trial, plot_RRT_modulo) == 0;
    RRT_trials{trial} = build_RRT(state_state{:}, RRT_colors(trial, :), plot_this_run);
    update_progress_bar(trial, num_RRT_trials);
end
title(sprintf('RRT runs complete: %d of %d', num_RRT_trials, num_RRT_trials));
end
