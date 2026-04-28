%% RRT.m
% Author: Matthew Lepis
% Course: ME 656 - Autonomous Navigation for Mobile Robots
% Assignment: SimLab 2
%
% File: RRT.m
%
% Description:
%   Runs repeated Rapidly-exploring Random Tree trials and stores each
%   recovered goal-reaching path with its total path length.

function RRT_trials = RRT(cfg)

function tree = build_RRT(x_pos_init, y_pos_init, edge_color, plot_this_run, cfg)
    tree = Tree(x_pos_init, y_pos_init, cfg);
    tree.extend(edge_color, plot_this_run);
end

function [path, path_length] = reconstruct_goal_path(tree)
    node = tree.vertices(end);  % Goal Node

    reverse_path = [TreeNode.empty(0, 1);];
    path_length = 0;

    while ~node.is_root()
        reverse_path(end+1, 1) = node;
        path_length = path_length + node.cost;
        node = node.parent;
    end
    reverse_path(end+1, 1) = node;
    path = clone_path_nodes(flip(reverse_path));

end

function path = clone_path_nodes(source_path)
    % Return lightweight path only nodes instead of handles into the full RRT.
    % Otherwise the path root's children can retain the entire tree for each
    % trial.
    path = [TreeNode.empty(0, 1);];

    for node_idx = 1 : numel(source_path)
        state = source_path(node_idx).state;
        path_node = TreeNode(state(1), state(2));
        path_node.idx = node_idx;
        path_node.cost = source_path(node_idx).cost;
        path_node.action = source_path(node_idx).action;

        if node_idx > 1
            path_node.parent = path(node_idx - 1);
        end

        path(end+1, 1) = path_node;
    end
end

function plot_reconstructed_path(path, path_length, path_color)
    path_line_width = 3.5;
    label_offset = [0.75, 0.75];

    if numel(path) < 2
        return;
    end

    for segment_idx = 1 : numel(path) - 1
        plot([path(segment_idx).state(1), path(segment_idx + 1).state(1)], ...
            [path(segment_idx).state(2), path(segment_idx + 1).state(2)], ...
            'Color', path_color, 'LineWidth', path_line_width);
    end

    goal_state = path(end).state;
    text(goal_state(1) + label_offset(1), goal_state(2) + label_offset(2), ...
        sprintf('L = %.2f m', path_length), ...
        'Color', path_color, 'FontSize', 11, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');

    drawnow limitrate;
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

RRT_trials = cell(1, cfg.num_RRT_trials);
RRT_colors = parula(cfg.num_RRT_trials);
state_state = num2cell(cfg.start_state);
for trial = 1 : cfg.num_RRT_trials
    %% PLOTTING helpers
    %
    update_progress_bar(trial - 1, cfg.num_RRT_trials);
    title(sprintf('RRT run %d of %d', trial, cfg.num_RRT_trials));
    drawnow;
    plot_this_run = cfg.plot_RRT_runs && mod(trial, cfg.plot_RRT_modulo) == 0;
    
    %% RRT
    %
    tree = build_RRT(state_state{:}, RRT_colors(trial, :), plot_this_run, cfg);
    [path, path_length] = reconstruct_goal_path(tree);

    if plot_this_run
        plot_reconstructed_path(path, path_length, RRT_colors(trial, :));
        if cfg.plot_RRT_runs_w_ellipses
            temp.path = path;
            propagate_KF_path(temp, cfg);
        end
    end

    trial_data = struct();
    trial_data.path = path;
    trial_data.path_length = path_length;

    RRT_trials{trial} = trial_data;


    %% PLOTTING helpers 
    %
    update_progress_bar(trial, cfg.num_RRT_trials);
end
title(sprintf('RRT runs complete: %d of %d', cfg.num_RRT_trials, cfg.num_RRT_trials));
end
