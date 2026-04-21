classdef TreeNode < handle
    properties
        state
        idx
        parent
        children
        cost
        action
    end

    methods
        function obj = TreeNode(x, y)
            obj.state = [x, y];
            obj.idx = [];
            obj.parent = [];
            obj.children = TreeNode.empty(0, 1);
            obj.cost = 0;
            obj.action = [];
        end

        function set_edge(obj, parent_node, action)
            obj.parent = parent_node;
            obj.action = action;
            obj.cost = norm(parent_node.state - obj.state);
            parent_node.children(end+1, :) = obj;
        end

        function status = is_root(obj)
            status = isempty(obj.parent);
        end
    end
end
