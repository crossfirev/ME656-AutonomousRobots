%% find_shortest_path.m
% Author: Matthew Lepis
% Course: ME 656 - Autonomous Navigation for Mobile Robots
% Assignment: SimLab 2
%
% File: find_shortest_path.m
%
% Description:
%   Finds the minimum-length RRT trial and returns both the selected trial
%   and its trial index.

function [shortest_trial, shortest_path_idx] = find_shortest_path(RRT_trials)
    num_trials = numel(RRT_trials);
    if num_trials == 0
        error('shortest_path:NoTrials', 'RRT_trials must contain at least one trial.');
    end

    path_lengths = zeros(1, num_trials);
    for k = 1 : num_trials
        path_lengths(k) = RRT_trials{k}.path_length;
    end

    [shortest_path_length, shortest_path_idx] = min(path_lengths);

    shortest_trial = RRT_trials{shortest_path_idx};
end
