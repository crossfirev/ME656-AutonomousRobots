length_hallway = 10;  % m
sensor_range = 0.5;  % m

landmarks = [2; 5; 8];  % m

num_landmarks = length(landmarks);  % 3

stdev_odometry = 0.1; % m/s
stdev_range = 0.01; % m

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Robot's "Ground Truth" Trajectory when it travels the hallway 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

delta_t = 0.1; % time is discretized with a time-step of 0.1 seconds
v_robot = 0.1; % robot travels at a constant speed of 0.1 m/s

t_terminal = length_hallway/v_robot; % time that we reach the end of hall

t_vector = 0:delta_t:t_terminal; % time vector for robot's trajectory
x_vector = 0:v_robot*delta_t:length_hallway; % robot's true position in time
v_vector = ones(1,length(t_vector)).*v_robot;  % robot's true velocity in time

num_states = length(x_vector); % number of discrete-time states in trajectory

%{

%
%       Ax = b
%
%  `A`          : measurement/design matrix
%  `x`          : unknown state vector we are solving for
%                   Contains robot position at every time-step; (And doesn't include robot velocities)                    
%
%                      x = [x_0, x_1, x_2, ..., x_K]^T
%
%  `b`          : measurement vector built from noisy simulated measurements

%
%       A(k^odom, :)x = x_k - x_(k-1)
%
%  `k_odom`     : the row index in the stacked measurment system corresponding to the `k`-th odometry measurement
%  `x_k`        : robot position state at time-step `k`
%  `x_(k-1)`    : robot position state at previous time-step
%  `k`          : the discrete time index

%
%       b(k^odom) = (x_k_dot + υ^odom_k) * Δt
%
%  `b_k_odom`   : the scalar entry (matrix element) of `b` for that odometry measurement
%  `z^odom_k`   : the noisy odometry velocity measurement at time-step `k`
%  `Δt`         : the discrete time-step (0.1s)
%  `x_k_dot`    : true robot velocity at time-step `k`; = 0.1 m/s
%  `υ^odom_k`   : additive Gaussian odometry noise at time-step `k`


%
% So...
%

%
%            A(k^odom, :)x = b(k^odom)
%
%                          |
%                          |
%                          \/
%
%            x_k - x_(k-1) = (x_k_dot + υ^odom_k) * Δt
%
%            x_k - x_(k-1) = ((0.1) + υ^odom_k) * (0.1)
%
%       b_vector(1) is the first MEASUREMENT row, and it encodes the special
%       initial-position constraint x_0 = 0.
%
%       State x_0 absolutely exists in the math model.
%       MATLAB just stores x_0 in column/index 1, because MATLAB is allergic to 0.
%
%            so:
%               x_1 - x_0 = ((0.1) + υ^odom_1) * (0.1); and x_0 = 0
%               x_1 - 0   = ((0.1) + υ^odom_1) * (0.1)
%               x_1       = ((0.1) + υ^odom_1) * (0.1)
%               x_2 - x_1 = ((0.1) + υ^odom_2) * (0.1)
%               x_3 - x_2 = ((0.1) + υ^odom_3) * (0.1)
%                              ...
%               x_K - x_(K-1) = ((0.1) + υ^odom_K) * (0.1)
%}

%% Deliverable 1: Batch Least-Squares SLAM with Odometry-Only Measurements
num_of_trials = 1000;

abs_error = zeros(num_of_trials, num_states);
for trial = 1:num_of_trials
    % Construct the noisy corrupted measurement `b` vector, with the special uncorrupted initial condition.
    b_vector = zeros(num_states, 1);
    b_vector(1) = 0;
    for k = 2:num_states
        meas_noise = stdev_odometry*randn();
        b_vector(k) = (v_vector(k) + meas_noise) * delta_t;
    end

    % A matrix for the odometry-only case
    A_matrix = zeros(num_states, num_states);  % n = number of measurements, m = number of unknown states
    A_matrix(1,1) = 1;  % Special initial-condition row: x_0 = 0
    for k = 2:num_states
        A_matrix(k, k-1) = -1;
        A_matrix(k, k) = 1;
    end

    % Estimate the states by solving A*x = b for x
    x_estimates = A_matrix \ b_vector;


    % Store this trial's absolute error across all states in row `trial`
    abs_error(trial, :) = abs(x_estimates - x_vector(:)).';
end
mean_abs_error = mean(abs_error, 1);
figure;
plot(t_vector, mean_abs_error, 'LineWidth', 1.8)
xlabel('Time (s)')
ylabel('Mean Absolute Error (m)')
title('Deliverable 1: Odometry-Only Batch Least-Squares Mean Absolute Error')
grid on

%{
%%% Deliverable 1: Supplemental Plots
%
%   Plot Noisy measured displacement over time.
%
% figure;
% plot(t_vector(2:end), b_vector(2:end))
% ylabel('Measured displacement per step (m)')
% xlabel('Time (s)')
% title('Noisy odometry displacement measurements')
% grid on

% odom_measurement_mean = mean(b_vector(2:end)); % Should be ~0.01
% odom_measurement_std = std(b_vector(2:end));    % Should be ~0.01 


%
%   Plot Truth vs Estimate, Robot Position
%
% figure;
% plot(t_vector, x_vector, 'LineWidth', 4)
% hold on
% plot(t_vector, x_estimates, ':', 'LineWidth', 2)
% xlabel('Time (s)')
% ylabel('Position (m)')
% title('True vs Estimated Robot Position (Odometry Only)')
% legend('True Position', 'Estimated Position')
% grid on


%
%   Plot Single Trial Absolute Position Error over Time
%
% figure;
% plot(t_vector, abs_error(1, :), 'LineWidth', 1.2)
% xlabel('Time (s)')
% ylabel('Absolute Position Error (m)')
% title('Absolute Position Error Over Time')
% grid on
%}
