function [start_state, goal_region, obstacles] = generate_obstacles(obstacle_layout)
if nargin < 1
    obstacle_layout = 'original';
end

obstacle_layout = lower(char(obstacle_layout));

close all;
figure(1); hold on;
axis([0 100 0 100]);
box on;

% Define and plot the start state for the planning problem
%               x  y
start_state = [ 5 50];
plot(start_state(1),start_state(2),'.r','MarkerSize',20);

% Define the default goal region for the planning problem.
%              x1 y1  x2  y2   x3  y3  x4  y4
goal_region = [90  0 100   0  100 100  90 100];

% Define the locations of the obstacles.
%             x1 y1 x2 y2 x3 y3 x4 y4
switch obstacle_layout
    case 'original'
        obstacles = [  5 10 15 10 15 20  5 20; % obstacle 1
                      10 40 20 40 20 50 10 50; % obstacle 2
                      20 70 30 70 30 80 20 80; % ...etc...
                      30 20 40 20 40 30 30 30; 
                      40 50 50 50 50 60 40 60;
                      50  5 60  5 60 15 50 15;
                      55 80 65 80 65 90 55 90;
                      60 40 70 40 70 50 60 50;
                      70 20 80 20 80 30 70 30
                      75 65 85 65 85 75 75 75 ];
    case 'maze'
        obstacles = [ 15  0 18  0 18 70 15 70;
                      30 30 33 30 33 100 30 100;
                      45  0 48  0 48 65 45 65;
                      60 35 63 35 63 100 60 100;
                      75  0 78  0 78 60 75 60;
                      18 70 28 70 28 73 18 73;
                      33 27 43 27 43 30 33 30;
                      48 65 58 65 58 68 48 68;
                      63 32 73 32 73 35 63 35;
                      78 60 88 60 88 63 78 63 ];
    case 'spiral'
        goal_region = [40 52 54 52 54 54 40 54];
        obstacles = [ 18 18 85 18 85 21 18 21;
                      82 18 85 18 85 82 82 82;
                      18 79 85 79 85 82 18 82;
                      18 34 21 34 21 82 18 82;
                      18 34 72 34 72 37 18 37;
                      69 34 72 34 72 68 69 68;
                      34 65 72 65 72 68 34 68;
                      34 48 37 48 37 68 34 68;
                      34 48 58 48 58 51 34 51;
                      55 48 58 48 58 58 55 58;
                      46 55 58 55 58 58 46 58 ];
    otherwise
        error('Unknown obstacle layout "%s". Use "original", "maze", or "spiral".', obstacle_layout);
end

goal_x = [goal_region(1) goal_region(3) goal_region(5) goal_region(7)];
goal_y = [goal_region(2) goal_region(4) goal_region(6) goal_region(8)];
patch(goal_x,goal_y,'green');

num_obstacles = size(obstacles,1);

for i_obs = 1:num_obstacles
    obs_x = [obstacles(i_obs,1) obstacles(i_obs,3) obstacles(i_obs,5) obstacles(i_obs,7)];
    obs_y = [obstacles(i_obs,2) obstacles(i_obs,4) obstacles(i_obs,6) obstacles(i_obs,8)];
    patch(obs_x,obs_y,'blue');
end

end
