classdef TreeVertex < handle
    properties
        state
        idx
        parent
        children
        cost
        action
    end

    methods
        function obj = TreeVertex(x, y)
            obj.state = [x, y];
            obj.idx = [];
            obj.parent = [];
            obj.children = TreeVertex.empty(0, 1);
            obj.cost = 0;
            obj.action = [];
        end

        function set_edge(obj, parent_vertex, action)
            obj.parent = parent_vertex;
            obj.action = action;
            obj.cost = norm(parent_vertex.state - obj.state);
            parent_vertex.children(end+1, :) = obj;
        end

        function status = is_root(obj)
            status = isempty(obj.parent);
        end
    end
end
