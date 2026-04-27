%% selection_color.m
% Author: Matthew Lepis
% Course: ME 656 - Autonomous Navigation for Mobile Robots
% Assignment: SimLab 2
%
% File: selection_color.m
%
% Description:
%   Returns consistent RGB colors for selected-path plot categories,
%   including minimum uncertainty, maximum uncertainty, shortest, and best.

function color = selection_color(selection_type)
selection_type = lower(char(selection_type));

switch selection_type
    case 'min'
        color = [0.00, 0.55, 0.55];
    case 'max'
        color = [0.55, 0.25, 0.70];
    case 'shortest'
        color = [0.85, 0.33, 0.10];
    case 'best'
        color = [0.10, 0.45, 0.85];
    otherwise
        color = [0.20, 0.20, 0.20];
end
end
