function plot_covariance_evolution(trial)
    nodes = trial.path;
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
