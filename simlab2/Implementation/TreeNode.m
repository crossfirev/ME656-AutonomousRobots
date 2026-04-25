classdef TreeNode < handle
    properties
        % RRT properties
        state
        idx
        parent
        children
        cost
        action

        % KF properties
        x_observability
        y_observability
        x
        A
        Q
        H
        R
        P_prev
        P
    end

    methods
        function obj = TreeNode(x, y)
            obj.state = [x, y];
            obj.idx = [];
            obj.parent = [];
            obj.children = TreeNode.empty(0, 1);
            obj.cost = 0;
            obj.action = [];

            obj.x_observability = [];
            obj.y_observability = [];
            obj.x = [];     % state_vector
            obj.A = [];     % state_transition_matrix
            obj.Q = [];     % process_noise_covariance
            obj.H = [];     % measurement_matrix
            obj.R = [];     % measurement_noise_covariance
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
