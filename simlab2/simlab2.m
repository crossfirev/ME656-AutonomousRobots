%% Establish configuration constants & constraints
cfg = simulation_configuation();

%% Run 'Rapidly expanding Random Tree' (RRT) trials to find many valid paths to goal. 
RRT_trials = simlab2_RRT(cfg);
% RRT_trials{k}.path - holds the path array of all TreeNodes in `k` path.
% RRT_trials{k}.path_length - holds the length of path `k`.

%% Find Shortest Path of the trials
best_path = feval('shortest_path', RRT_trials);
% `best_path.path` - holds the path array of all TreeNodes in the shortest path
% `best_path.path_length` - holds the length of the shortest path
