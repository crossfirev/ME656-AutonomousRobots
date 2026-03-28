function sim1
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

% Batch least-squares state:
%   [x_0, x_1, ..., x_K]^T
%
% Odometry rows encode consecutive pose differences:
%   x_k - x_(k-1) = z_odom(k)
% where z_odom(k) = (v_k + noise) * delta_t.
%
% Row 1 is the special initial-condition constraint x_0 = 0.

%% Deliverable 1: Batch Least-Squares SLAM with Odometry-Only Measurements
num_of_trials = 1000;

% Build the matching linear measurement matrix.
A_matrix = zeros(num_states, num_states);
A_matrix(1,1) = 1;  % Special initial-condition row: x_0 = 0
for k = 2:num_states
    A_matrix(k, k-1) = -1;
    A_matrix(k, k) = 1;
end

abs_error = zeros(num_of_trials, num_states);
for trial = 1:num_of_trials
    % Build the measurement vector with the initial condition and odometry.
    b_vector = zeros(num_states, 1);
    b_vector(1) = 0;
    for k = 2:num_states
        meas_noise = stdev_odometry * randn();
        b_vector(k) = (v_vector(k) + meas_noise) * delta_t;
    end

    % Solve for the full robot trajectory in one batch.
    x_estimates = A_matrix \ b_vector;

    % Store this trial's absolute pose error over time.
    abs_error(trial, :) = abs(x_estimates - x_vector(:)).';
end

mean_state_abs_error = mean(abs_error, 1);
figure;
plot(t_vector, mean_state_abs_error, 'LineWidth', 1.8)
xlabel('Time (s)')
ylabel('Mean Absolute Robot Position Error (m)')
title('Deliverable 1: Mean Absolute Robot Position Error (Batch Least-Squares, Odometry Only)')
ylim([0, 0.35])
grid on

%{
%%% Deliverable 1: Supplemental Plots

% Plot noisy odometry displacement measurements for one trial.
%
% figure;
% plot(t_vector(2:end), b_vector(2:end))
% ylabel('Measured displacement per step (m)')
% xlabel('Time (s)')
% title('Noisy odometry displacement measurements')
% ylim([0, 0.35])
grid on

% odom_measurement_mean = mean(b_vector(2:end)); % Should be ~0.01
% odom_measurement_std = std(b_vector(2:end));   % Should be ~0.01

% Plot true vs estimated robot position for one trial.
%
% figure;
% plot(t_vector, x_vector, 'LineWidth', 4)
% hold on
% plot(t_vector, x_estimates, ':', 'LineWidth', 2)
% xlabel('Time (s)')
% ylabel('Position (m)')
% title('True vs Estimated Robot Position (Odometry Only)')
% legend('True Position', 'Estimated Position')
% ylim([0, 0.35])
grid on

% Plot single-trial absolute position error over time.
%
% figure;
% plot(t_vector, abs_error(1, :), 'LineWidth', 1.2)
% xlabel('Time (s)')
% ylabel('Absolute Position Error (m)')
% title('Absolute Position Error Over Time')
% ylim([0, 0.35])
grid on
%}
end
