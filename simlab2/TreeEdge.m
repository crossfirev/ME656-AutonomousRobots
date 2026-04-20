classdef TreeEdge < handle
    properties
        % State
        parent
        child
        cost
        action
    end
    methods
        function obj = TreeEdge(x_near, x_new, u_new)
            obj.parent = x_near;
            obj.child = x_new;
            obj.cost = norm(x_near.state - x_new.state);
            obj.action = u_new;
        end
    end
end
