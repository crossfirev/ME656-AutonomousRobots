length_hallway = 10;  % m
sensor_range = 0.5;  % m

landmarks = [2; 5; 8];  % m
num_landmarks = length(landmarks);  % 3

stdev_odometry = 0.1; % m/s
stdev_range = 0.01; % m

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Robot ground-truth trajectory for the hallway traversal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

delta_t = 0.1; % time is discretized with a time-step of 0.1 seconds
v_robot = 0.1; % robot travels at a constant speed of 0.1 m/s

t_terminal = length_hallway/v_robot; % time that we reach the end of hall

t_vector = 0:delta_t:t_terminal; % time vector for robot's trajectory
x_vector = 0:v_robot*delta_t:length_hallway; % true robot position over time
v_vector = ones(1,length(t_vector)).*v_robot;  % true robot velocity over time

num_states = length(x_vector); % number of discrete-time pose states