function plot_path(trial, type)
    path = trial.path;
    path_length = trial.path_length;
    uncertainty = trial.final_uncertainty;
    path_color = [0.85, 0.10, 0.10];
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
                sprintf('Maximal Terminal Uncertainty,\n  L = %.2f m\n  Uncertainty = %.2f', path_length, uncertainty), ...
                'Color', path_color, 'FontSize', 11, 'FontWeight', 'bold', ...
                'HorizontalAlignment', 'left', 'VerticalAlignment', 'top');
        case 'min'
            text(goal_state(1) + label_offset(1), goal_state(2) + label_offset(2), ...
                sprintf('Minimal Terminal Uncertainty,\n  L = %.2f m\n  Uncertainty = %.2f', path_length, uncertainty), ...
                'Color', path_color, 'FontSize', 11, 'FontWeight', 'bold', ...
                'HorizontalAlignment', 'left', 'VerticalAlignment', 'top');
        case 'shortest'
            text(goal_state(1) + label_offset(1), goal_state(2) + label_offset(2), ...
                sprintf('Shortest\n  L = %.2f m\n  Uncertainty = %.2f', path_length, uncertainty), ...
                'Color', path_color, 'FontSize', 11, 'FontWeight', 'bold', ...
                'HorizontalAlignment', 'left', 'VerticalAlignment', 'top');
    end
    drawnow limitrate;
end
