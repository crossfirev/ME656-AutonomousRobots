function shortest_path = find_shortest_path(RRT_trials)
    num_trials = numel(RRT_trials);
    if num_trials == 0
        error('shortest_path:NoTrials', 'RRT_trials must contain at least one trial.');
    end

    path_lengths = zeros(1, num_trials);
    for k = 1 : num_trials
        path_lengths(k) = RRT_trials{k}.path_length;
    end

    [shortest_path_length, shortest_path_idx] = min(path_lengths);

    shortest_path = RRT_trials{shortest_path_idx};
    shortest_path.path_length = shortest_path_length;

    plot_shortest_path(shortest_path.path, shortest_path.path_length);
    title(sprintf('Shortest path found on trial %d of %d', shortest_path_idx, num_trials));
end

function plot_shortest_path(path, path_length)
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
    text(goal_state(1) + label_offset(1), goal_state(2) + label_offset(2), ...
        sprintf('Shortest L = %.2f m', path_length), ...
        'Color', path_color, 'FontSize', 11, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'left', 'VerticalAlignment', 'top');

    drawnow limitrate;
end
