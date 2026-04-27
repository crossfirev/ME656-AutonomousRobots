%% collect_sensor_data.m
% Author: Matthew Lepis
% Course: ME 656 - Autonomous Navigation for Mobile Robots
% Assignment: SimLab 2
%
% File: collect_sensor_data.m
%
% Description:
%   Determines whether each path node has x and/or y observability from
%   nearby range-beam intersections with mapped obstacles.

function path = collect_sensor_data(path, cfg)
    nodes = path.path;
    for k = 1 : length(nodes)
        % Collecting state_vector data
        [x_observable, y_observable] = obstacle_in_reach(nodes(k), cfg);
        nodes(k).x_observability = x_observable;
        nodes(k).y_observability = y_observable;
    end
    path.path = nodes;
end
function [x_observable, y_observable] = obstacle_in_reach(node, cfg)
    obstacles = cfg.obstacles;
    x_observable = 0;
    y_observable = 0;

    x_pos = node.state(1);
    y_pos = node.state(2);

    north_x = x_pos;
    north_y = y_pos + cfg.beam_range;

    west_x = x_pos + cfg.beam_range;
    west_y = y_pos;

    south_x = x_pos; 
    south_y = y_pos - cfg.beam_range;

    east_x = x_pos - cfg.beam_range; 
    east_y = y_pos;

    if collision_check_segment(x_pos, y_pos, north_x, north_y, obstacles) ...
        || collision_check_point(north_x, north_y, obstacles)
        y_observable = 1;
    end
    if collision_check_segment(x_pos, y_pos, east_x, east_y, obstacles)...
        || collision_check_point(east_x, east_y, obstacles)
        x_observable = 1;
    end
    if collision_check_segment(x_pos, y_pos,south_x, south_y, obstacles)...
        || collision_check_point(south_x, south_y, obstacles)
        y_observable = 1;
    end
    if collision_check_segment(x_pos, y_pos, west_x, west_y, obstacles)...
        || collision_check_point(west_x, west_y, obstacles)
        x_observable = 1;
    end
end
