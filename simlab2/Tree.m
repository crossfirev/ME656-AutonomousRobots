classdef Tree < handle
    properties
        vertices
        c
    end

    methods
        function obj = Tree(x_pos_init, y_pos_init, cfg)
            root = TreeVertex(x_pos_init, y_pos_init);
            root.idx = 1;

            obj.vertices = [root];
            obj.c = cfg;
        end

        function extend(obj, edge_color, plot_this_run)
            status = 0;
            while status ~= 1 
                x = random_state(obj.c);
                x_near = nearest_neighbor(x, obj);

                [success, x_new, u_new] = new_state(x, x_near, obj.c);
                if success
                    obj.add_vertex(x_near, x_new, u_new);

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
        function obj = add_vertex(obj, x_near, x_new, u_new)
            x_new.idx = length(obj.vertices) + 1;
            x_new.set_edge(x_near, u_new);

            obj.vertices(end+1, :) = x_new;
        end
    end
end

function nearest_vertex = nearest_neighbor(x, tree)
    nearest_vertex = tree.vertices(1);
    for vertex_id = 1 : length(tree.vertices)
        if norm(tree.vertices(vertex_id).state - x.state) <= norm(nearest_vertex.state - x.state)
            nearest_vertex = tree.vertices(vertex_id);
        end
    end
end

function [status, x_new, u_new] = new_state(x, x_near, c)
    % check if `x` is valid (if it is inside an obstacle or not)
    % if valid:
    %   - status = 1
    %   - calculate the required translation/rotation required to get a linear path with length `c.growth_dist` from x_near toward x; set to `u_new`
    %   - make a new node and set far end of the path to `x_new`
    % if not valid:
    %   - status = 0
    %   - x_new = NaN
    %   - u_new = NaN

    x_dir = x.state - x_near.state;
    x_dist = norm(x_dir);

    % Check if the sampled vertex and the nearest neighbor are the same state.
    if x_dist < eps
        status = 0;
        x_new = TreeVertex.empty(0, 1);
        u_new = [0, 0];
        return;
    end

    x_unit_dir = x_dir / x_dist;
    if (x_dist >= c.growth_dist)
        x_dist = c.growth_dist;
    end
    
    x_new = x_near.state + x_unit_dir * x_dist;
    x_new = TreeVertex(x_new(1), x_new(2));
    u_new = x_new.state - x_near.state;

    if collision_check_point(x_new.state(1), x_new.state(2), c.obstacles) == 0 && ...
        collision_check_segment(x_near.state(1), x_near.state(2), x_new.state(1), x_new.state(2), c.obstacles) == 0
        status = 1;
    else
        status = 0;
    end
end

function vertex = random_state(c)
    x_pos_rand = c.x_bounds(1) + (c.x_bounds(2) - c.x_bounds(1)) * rand();
    y_pos_rand = c.y_bounds(1) + (c.y_bounds(2) - c.y_bounds(1)) * rand();
    vertex = TreeVertex(x_pos_rand, y_pos_rand);
end