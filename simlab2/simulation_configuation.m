%% simulation_configuation.m
% Author: Matthew Lepis
% Course: ME 656 - Autonomous Navigation for Mobile Robots
% Assignment: SimLab 2
%
% File: simulation_configuation.m
%
% Description:
%   Builds the configuration struct containing workspace, RRT, sensor, and
%   Kalman filter parameters for the SimLab 2 simulation.

function sim_config = simulation_configuation(obstacle_type, num_RRT_trials, plot_RRT_modulo)
[start_state, goal_region, obstacles] = generate_obstacles(obstacle_type);

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
% KF Constraints
%

time_step = 1; % second

%
% Simulation Constraints
%
x_bounds = [0, 100];
y_bounds = [0, 100];
plot_RRT_runs = false;
plot_RRT_runs_w_ellipses = false;

%
% Best path scoring
%
best_path_distance_weight = 0.5;
best_path_uncertainty_weight = 0.5;

%
% Particle filter extra credit
%
particle_filter_num_particles = 100;
particle_filter_num_snapshots = 20;
particle_filter_initial_variance = 1;
particle_filter_process_variance = 1;
particle_filter_sensor_variance = 1;

%%
cfg.start_state = start_state;
cfg.goal_region = goal_region;
cfg.obstacles = obstacles;
%
cfg.beam_range = beam_range;
%
cfg.growth_dist = growth_dist;
%
cfg.time_step = time_step;
%
cfg.num_RRT_trials = num_RRT_trials;
cfg.x_bounds = x_bounds;
cfg.y_bounds = y_bounds;
cfg.plot_RRT_runs = plot_RRT_runs;
cfg.plot_RRT_runs_w_ellipses = plot_RRT_runs_w_ellipses; 
cfg.plot_RRT_modulo = plot_RRT_modulo;
cfg.best_path_distance_weight = best_path_distance_weight;
cfg.best_path_uncertainty_weight = best_path_uncertainty_weight;
cfg.particle_filter_num_particles = particle_filter_num_particles;
cfg.particle_filter_num_snapshots = particle_filter_num_snapshots;
cfg.particle_filter_initial_variance = particle_filter_initial_variance;
cfg.particle_filter_process_variance = particle_filter_process_variance;
cfg.particle_filter_sensor_variance = particle_filter_sensor_variance;
%
sim_config = cfg;
end
