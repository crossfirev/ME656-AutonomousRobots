classdef TreeVertex < handle
    properties
        % State
        state
    end
    methods
        function obj = TreeVertex(x, y)
            obj.state = [x, y];
        end
    end
end
