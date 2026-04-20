function simlab2()
%% SimLab 2 - Main Simulation Driver
% Author: Matthew Lepis
% Date: April 19, 2026
%
% Main simulation driver script for SimLab 2.
%
% TODO: Add description for this file before submission.
%

[start_state, goal_region, obstacles] = generate_obstacles();

%
% Robot Constraints
%
%   Robot is a point robot.
%   Robot has 4 laser range finders, one in each cardinal direction - with range: `beam_range`

beam_range = 4;

%
% RRT Constraints
%

growth_dist = 2; % meters

%
% Simulation Constraints
%

num_RRT_trials = 100;
x_bounds = [0, 100];
y_bounds = [0, 100];

function [x_pos_rand, y_pos_rand] = random_state()
    x_pos_rand = x_bounds(1) + (x_bounds(2) - x_bounds(1)) * rand();
    y_pos_rand = y_bounds(1) + (y_bounds(2) - y_bounds(1)) * rand();
end

function nearest_vertex = nearest_neighbor(x, tree)
    nearest_vertex = tree.vertices(1);
    for vertex_id = 1 : length(tree.vertices)
        if norm(tree.vertices(vertex_id).state - x.state) <= norm(nearest_vertex.state - x.state)
            nearest_vertex = tree.vertices(vertex_id);
        end
    end
end

function [status, x_new, u_new] = new_state(x, x_near)
    % check if `x` is valid (if it is inside an obstacle or not)
    % if valid:
    %   - status = 1
    %   - calculate the required translation/rotation required to get a linear path with length `growth_dist` from x_near toward x; set to `u_new`
    %   - make a new node and set far end of the path to `x_new`
    % if not valid:
    %   - status = 0
    %   - x_new = NaN
    %   - u_new = NaN    
    x_dir = x.state - x_near.state;
    x_dist = norm(x_dir);
    x_unit_dir = x_dir / x_dist;
    if (x_dist >= growth_dist)
        x_dist = growth_dist;
    end
    
    x_new = x_near.state + x_unit_dir * x_dist;
    x_new = TreeVertex(x_new(1), x_new(2));
    u_new = x_new.state - x_near.state;

    if collision_check_point(x_new.state(1), x_new.state(2), obstacles) == 0 && ...
        collision_check_segment(x_near.state(1), x_near.state(2), x_new.state(1), x_new.state(2), obstacles) == 0
        status = 1;
    else
        status = 0;
    end
end

function status = extend(tree)
    status = 0;
    while status ~= 1 
        [sample_x, sample_y] = random_state();

        x = TreeVertex(sample_x, sample_y);
        x_near = nearest_neighbor(x, tree);

        [success, x_new, u_new] = new_state(x, x_near);
        if success
            tree.add_vertex(x_new);
            tree.add_edge(x_near, x_new, u_new);
            plot([x_near.state(1), x_new.state(1)], ...
                 [x_near.state(2), x_new.state(2)], ...
                 'Color', [0.35 0.35 0.35], 'LineWidth', 0.5);
            drawnow limitrate;
            if collision_check_point(x_new.state(1), x_new.state(2), goal_region) == 1
                status = 1;  % Reached Goal
            else  % x_new == x
                status = 0;  % Advanced 
            end
        else
            status = -1;  % Trapped
        end
    end
end

function tree = build_RRT(x_pos_init, y_pos_init)
    tree = Tree(x_pos_init, y_pos_init);
    extend(tree)
end


RRT_trials = {};
state_state = num2cell(start_state);
for trial = 1 : num_RRT_trials
    RRT_trials{trial} = build_RRT(state_state{:});
end
end
