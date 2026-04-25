classdef Tree < handle
    properties
        vertices
        c
    end

    methods
        function obj = Tree(x_pos_init, y_pos_init, cfg)
            root = TreeNode(x_pos_init, y_pos_init);
            root.idx = 1;

            obj.vertices = [root];
            obj.c = cfg;
        end

        function extend(obj, edge_color, plot_this_run)
            status = 0;
            while status ~= 1 
                x_sample = random_state(obj.c);
                x_near = nearest_neighbor(x_sample, obj);

                [success, x_new, u_new] = new_state(x_sample, x_near, obj.c);
                if success
                    obj.add_node(x_near, x_new, u_new);

                    if plot_this_run
                        plot([x_near.state(1), x_new.state(1)], ...
                            [x_near.state(2), x_new.state(2)], ...
                            'Color', edge_color, 'LineWidth', 0.5);
                        drawnow limitrate;
                    end

                    if collision_check_point(x_new.state(1), x_new.state(2), obj.c.goal_region) == 1
                        status = 1;  % Reached Goal
                    else  % x_new == x
                        status = 0;  % Advanced 
                    end
                else
                    status = -1;  % Trapped
                end
            end
        end
    end
    methods (Access = private)
        function obj = add_node(obj, x_near, x_new, u_new)
            x_new.idx = length(obj.vertices) + 1;
            x_new.set_edge(x_near, u_new);

            obj.vertices(end+1, :) = x_new;
        end
    end
end

function nearest_node = nearest_neighbor(x_state, tree)
    nearest_node = tree.vertices(1);
    nearest_distance_sq = sum((nearest_node.state - x_state) .^ 2);

    for node_id = 2 : length(tree.vertices)
        distance_sq = sum((tree.vertices(node_id).state - x_state) .^ 2);
        if distance_sq <= nearest_distance_sq
            nearest_node = tree.vertices(node_id);
            nearest_distance_sq = distance_sq;
        end
    end
end

function [status, x_new, u_new] = new_state(x_state, x_near, c)
    % check if `x` is valid (if it is inside an obstacle or not)
    % if valid:
    %   - status = 1
    %   - calculate the required translation/rotation required to get a linear path with length `c.growth_dist` from x_near toward x; set to `u_new`
    %   - make a new node and set far end of the path to `x_new`
    % if not valid:
    %   - status = 0
    %   - x_new = NaN
    %   - u_new = NaN

    x_dir = x_state - x_near.state;
    x_dist_sq = sum(x_dir .^ 2);

    % Check if the sampled node and the nearest neighbor are the same state.
    if x_dist_sq < eps^2
        status = 0;
        x_new = TreeNode.empty(0, 1);
        u_new = [0, 0];
        return;
    end

    x_dist = sqrt(x_dist_sq);
    x_unit_dir = x_dir / x_dist;
    if (x_dist >= c.growth_dist)
        x_dist = c.growth_dist;
    end
    
    x_new_state = x_near.state + x_unit_dir * x_dist;

    if collision_check_point(x_new_state(1), x_new_state(2), c.obstacles) == 0 && ...
        collision_check_segment(x_near.state(1), x_near.state(2), x_new_state(1), x_new_state(2), c.obstacles) == 0
        x_new = TreeNode(x_new_state(1), x_new_state(2));
        u_new = x_new_state - x_near.state;
        status = 1;
    else
        x_new = TreeNode.empty(0, 1);
        u_new = [0, 0];
        status = 0;
    end
end

function state = random_state(c)
    x_pos_rand = c.x_bounds(1) + (c.x_bounds(2) - c.x_bounds(1)) * rand();
    y_pos_rand = c.y_bounds(1) + (c.y_bounds(2) - c.y_bounds(1)) * rand();
    state = [x_pos_rand, y_pos_rand];
end