%% plot_path.m
% Author: Matthew Lepis
% Course: ME 656 - Autonomous Navigation for Mobile Robots
% Assignment: SimLab 2
%
% File: plot_path.m
%
% Description:
%   Plots a selected path and labels its selection type, path length, and
%   terminal uncertainty.

function plot_path(trial, type)
    type = lower(char(type));
    path = trial.path;
    path_length = trial.path_length;
    uncertainty = trial.final_uncertainty;
    path_color = selection_color(type);
    path_line_width = 4;
    label_offset = [0.75, -1.50];

    if numel(path) < 2
        return;
    end

    for segment_idx = 1 : numel(path) - 1
        plot([path(segment_idx).state(1), path(segment_idx + 1).state(1)], ...
            [path(segment_idx).state(2), path(segment_idx + 1).state(2)], ...
            'Color', path_color, 'LineWidth', path_line_width);
    end

    goal_state = path(end).state;
    switch type
        case 'max'
            text(goal_state(1) + label_offset(1), goal_state(2) + label_offset(2), ...
                sprintf('Maximal Terminal Uncertainty\n  L = %.2f m\n  Uncertainty = %.2f', path_length, uncertainty), ...
                'Color', path_color, 'FontSize', 11, 'FontWeight', 'bold', ...
                'HorizontalAlignment', 'left', 'VerticalAlignment', 'top');
        case 'min'
            text(goal_state(1) + label_offset(1), goal_state(2) + label_offset(2), ...
                sprintf('Minimal Terminal Uncertainty\n  L = %.2f m\n  Uncertainty = %.2f', path_length, uncertainty), ...
                'Color', path_color, 'FontSize', 11, 'FontWeight', 'bold', ...
                'HorizontalAlignment', 'left', 'VerticalAlignment', 'top');
        case 'shortest'
            text(goal_state(1) + label_offset(1), goal_state(2) + label_offset(2), ...
                sprintf('Shortest\n  L = %.2f m\n  Uncertainty = %.2f', path_length, uncertainty), ...
                'Color', path_color, 'FontSize', 11, 'FontWeight', 'bold', ...
                'HorizontalAlignment', 'left', 'VerticalAlignment', 'top');
        case 'best'
            if isfield(trial, 'selection_cost')
                label = sprintf('Best Tradeoff\n  L = %.2f m\n  Uncertainty = %.2f\n  Cost = %.3f', ...
                    path_length, uncertainty, trial.selection_cost);
            else
                label = sprintf('Best Tradeoff\n  L = %.2f m\n  Uncertainty = %.2f', ...
                    path_length, uncertainty);
            end
            text(goal_state(1) + label_offset(1), goal_state(2) + label_offset(2), ...
                label, ...
                'Color', path_color, 'FontSize', 11, 'FontWeight', 'bold', ...
                'HorizontalAlignment', 'left', 'VerticalAlignment', 'top');
    end
    drawnow limitrate;
end
