function sim_config = simulation_configuation()
[start_state, goal_region, obstacles] = generate_obstacles("original");

%
% Robot Constraints
%
%   Robot is a point robot.
%   Robot has 4 laser range finders, one in each cardinal direction - with range: `beam_range`

beam_range = 5;

%
% RRT Constraints
%

growth_dist = 2; % meters

%
% Simulation Constraints
%

num_RRT_trials = 1000;
x_bounds = [0, 100];
y_bounds = [0, 100];
plot_RRT_runs = true;
plot_RRT_modulo = 500;

%%
cfg.start_state = start_state;
cfg.goal_region = goal_region;
cfg.obstacles = obstacles;
%
cfg.beam_range = beam_range;
%
cfg.growth_dist = growth_dist;
%
cfg.num_RRT_trials = num_RRT_trials;
cfg.x_bounds = x_bounds;
cfg.y_bounds = y_bounds;
cfg.plot_RRT_runs = plot_RRT_runs;
cfg.plot_RRT_modulo = plot_RRT_modulo;
%
sim_config = cfg;
end