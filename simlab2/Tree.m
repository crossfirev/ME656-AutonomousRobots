classdef Tree < handle
    properties
        vertices
        edges
    end

    methods
        function obj = Tree(x_pos_init, y_pos_init)
            obj.vertices = [TreeVertex(x_pos_init, y_pos_init)];
            obj.edges = [TreeEdge.empty(0, 1)];
        end
        function obj = add_vertex(obj, x_new)
            obj.vertices(end+1, :) = x_new;
        end
        function obj = add_edge(obj, x_near, x_new, u_new)
            obj.edges(end+1) = TreeEdge(x_near, x_new, u_new);
        end
    end

end