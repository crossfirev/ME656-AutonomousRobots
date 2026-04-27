%% Establish configuration constants & constraint
% "original"
% "maze"
% "spiral"
cfg = simulation_configuation("original", 100, 33);

%% Run 'Rapidly expanding Random Tree' (RRT) trials to find many valid paths to goal. 
RRT_trials = simlab2_RRT(cfg);
% RRT_trials{k}.path - holds the path array of all `TreeNodes` in `k` path.
% RRT_trials{k}.path_length - holds the length of path `k`.

%% Propagate the Posterior Error Covariance for all RRT paths
% RRT_trials{k}.KF_populated_path - holds the minimal path with all relevant KF attributes, specifically Posterior Error Covariance.
terminal_uncertainty = zeros(1, cfg.num_RRT_trials);
for k = 1 : numel(RRT_trials)
    RRT_trials{k}.KF_populated_path = propagate_KF_path(RRT_trials{k}, cfg);
    terminal_uncertainty(k) = RRT_trials{k}.KF_populated_path.final_uncertainty;
end

%% Find minimal terminal uncertainty path
[minimal_uncertainty, minimal_uncertainty_idx] = min(terminal_uncertainty);
minimal_trial = RRT_trials{minimal_uncertainty_idx};
KF_populated_minimal_trial = propagate_KF_path(minimal_trial, cfg);
plot_path(KF_populated_minimal_trial, "min");
plot_covariance_evolution(KF_populated_minimal_trial, "min");

%% Find maximal terminal uncertainty path
[maximal_uncertainty, maximal_uncertainty_idx] = max(terminal_uncertainty);
maximal_trial = RRT_trials{maximal_uncertainty_idx};
KF_populated_maximal_trial = propagate_KF_path(maximal_trial, cfg);
plot_path(KF_populated_maximal_trial, 'max');
plot_covariance_evolution(KF_populated_maximal_trial, 'max');

%% Find Shortest Path of the trials
[shortest_trial, shortest_trial_idx] = find_shortest_path(RRT_trials);
KF_populated_shortest_trial = propagate_KF_path(shortest_trial, cfg);
plot_path(KF_populated_shortest_trial, 'shortest');
plot_covariance_evolution(KF_populated_shortest_trial, 'shortest');

%% Add trial-selection summary text box
selection_summary = {
    sprintf('Shortest path found on trial %d of %d', shortest_trial_idx, cfg.num_RRT_trials)
    sprintf('Minimal uncertainty path found on trial %d of %d', minimal_uncertainty_idx, cfg.num_RRT_trials)
    sprintf('Maximal uncertainty path found on trial %d of %d', maximal_uncertainty_idx, cfg.num_RRT_trials)
};
annotation('textbox', [0.16 0.80 0.36 0.11], ...
    'String', selection_summary, ...
    'FitBoxToText', 'on', ...
    'BackgroundColor', [1 1 1], ...
    'EdgeColor', [0.25 0.25 0.25], ...
    'FontSize', 10, ...
    'FontWeight', 'bold');
