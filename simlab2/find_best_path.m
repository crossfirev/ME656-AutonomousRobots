%% find_best_path.m
% Author: Matthew Lepis
% Course: ME 656 - Autonomous Navigation for Mobile Robots
% Assignment: SimLab 2
%
% File: find_best_path.m
%
% Description:
%   Scores each RRT trial by a weighted combination of normalized path
%   length and terminal uncertainty, then returns the lowest-cost trial.

function [best_trial, best_trial_idx, best_cost] = find_best_path(RRT_trials, terminal_uncertainty, distance_weight, uncertainty_weight)
    num_trials = numel(RRT_trials);
    if num_trials == 0
        error('best_path:NoTrials', 'RRT_trials must contain at least one trial.');
    end

    if numel(terminal_uncertainty) ~= num_trials
        error('best_path:SizeMismatch', 'terminal_uncertainty must have one value per RRT trial.');
    end

    path_lengths = zeros(1, num_trials);
    for k = 1 : num_trials
        path_lengths(k) = RRT_trials{k}.path_length;
    end

    weights = normalize_weights(distance_weight, uncertainty_weight);
    normalized_lengths = normalize_to_unit_interval(path_lengths);
    normalized_uncertainty = normalize_to_unit_interval(terminal_uncertainty);

    combined_cost = weights(1) * normalized_lengths + weights(2) * normalized_uncertainty;
    [best_cost, best_trial_idx] = min(combined_cost);
    best_trial = RRT_trials{best_trial_idx};
end

function weights = normalize_weights(distance_weight, uncertainty_weight)
    weights = [distance_weight, uncertainty_weight];
    if any(weights < 0)
        error('best_path:InvalidWeights', 'Best-path weights must be nonnegative.');
    end

    weight_sum = sum(weights);
    if weight_sum <= 0
        error('best_path:InvalidWeights', 'At least one best-path weight must be positive.');
    end

    weights = weights / weight_sum;
end

function normalized_values = normalize_to_unit_interval(values)
    value_range = max(values) - min(values);
    if value_range < eps
        normalized_values = zeros(size(values));
    else
        normalized_values = (values - min(values)) / value_range;
    end
end
