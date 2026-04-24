function path = collect_measurement_data(path, obstacles)
    nodes = path.path;
    for k = 1 : length(nodes)
        state = nodes(k).state;
    end
end
function [north, west, south, east] = in_range_obstacle(node, obstacles)
    
end