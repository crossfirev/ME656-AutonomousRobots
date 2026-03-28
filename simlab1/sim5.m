function sim5
length_hallway = 10;  % m
sensor_range = 0.5;  % m

landmarks = [2; 5; 8];  % m
num_landmarks = length(landmarks);  % 3

stdev_odometry = 0.1; % m/s
stdev_range = 0.01; % m

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Robot ground-truth trajectory for the back-and-forth hallway traversal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
delta_t = 0.1; % time is discretized with a time-step of 0.1 seconds
v_robot = 0.1; % robot travels toward the end of the hallway at 0.1 m/s
v_robot_rev = -0.1; % robot returns to the origin at -0.1 m/s

t_terminal = length_hallway / v_robot; % time when the robot first reaches the end of hall
t_terminal_rev = -length_hallway / v_robot_rev; % time needed to return to the origin
t_terminal_total = t_terminal + t_terminal_rev;

t_vector = 0:delta_t:t_terminal; % forward-leg time vector
t_vector_rev = t_terminal+delta_t:delta_t:t_terminal_total; % return-leg time vector
t_vector_total = [t_vector, t_vector_rev];

x_vector = 0:v_robot*delta_t:length_hallway; % forward-leg ground-truth positions
x_vector_rev = length_hallway+v_robot_rev*delta_t:v_robot_rev*delta_t:0; % return-leg ground-truth positions
x_vector_total = [x_vector, x_vector_rev];

v_vector = ones(1, length(t_vector)) .* v_robot; % forward-leg velocity history
v_vector_rev = ones(1, length(t_vector_rev)) .* v_robot_rev; % return-leg velocity history
v_vector_total = [v_vector, v_vector_rev];

num_states = length(x_vector_total); % number of discrete-time pose states

% Batch least-squares state:
%   [x_0, x_1, ..., x_K, l_1, l_2, l_3]^T
%
% Odometry rows encode consecutive pose differences:
%   x_k - x_(k-1) = z_odom(k)
% where z_odom(k) = (v_k + noise) * delta_t.
%
% Landmark rows encode signed robot-to-landmark displacement:
%   l_j - x_k = z_range(k,j)
% This keeps the landmark measurement model linear in the unknown states.
%
% Deliverable 5 extends Deliverable 2 by sending the robot to the end of
% the hallway and then back to the origin. Re-observing the same landmarks
% on the return trip creates loop-closure constraints in the batch system.
%
% Row 1 is the special initial-condition constraint x_0 = 0.

function measurement = odometryMeasurement(time_step)
    % Simulate one noisy odometry displacement measurement.
    odo_noise = stdev_odometry * randn();
    measurement = (v_vector_total(time_step) + odo_noise) * delta_t;
end

function in_range = inRange(time_step, true_landmark_pose)
    % Return whether the robot can sense this landmark at the current time-step.
    if abs(true_landmark_pose - x_vector_total(time_step)) <= sensor_range
        in_range = true;
    else
        in_range = false;
    end
end

function measurements = sensorMeasurements(time_step)
    % Simulate signed landmark measurements for all landmarks currently in range.
    measurements = zeros(num_landmarks, 1);
    for landmark = 1:num_landmarks
        true_landmark_pose = landmarks(landmark);

        if inRange(time_step, true_landmark_pose)
            true_robot_pose = x_vector_total(time_step);
            range_sensor_noise = stdev_range * randn();

            % Signed displacement keeps the batch system linear.
            noisy_range_to_landmark = true_landmark_pose - true_robot_pose + range_sensor_noise;
            measurements(landmark) = noisy_range_to_landmark;
        else
            % Placeholder row; removed before solving.
            measurements(landmark) = nan;
        end
    end
end

function b_vector = bStorageHandling(b_vector, time_step)
    % Landmark measurements for one time-step occupy a contiguous block
    % after the odometry rows.
    idx_shift = num_states + (time_step - 1) * num_landmarks;

    sensor_measurements = sensorMeasurements(time_step);
    for landmark = 1:num_landmarks
        b_vector(idx_shift + landmark) = sensor_measurements(landmark);
    end
end

function A_matrix = AStorageHandling(A_matrix, time_step)
    % Each landmark row encodes l_j - x_k = z_range(k,j).
    idx_shift = num_states + (time_step - 1) * num_landmarks;

    for landmark = 1:num_landmarks
        row = idx_shift + landmark;
        A_matrix(row, time_step) = -1;
        A_matrix(row, num_states + landmark) = 1;
    end
end

%% Deliverable 5: Batch Least-Squares SLAM with Back-and-Forth Landmark Reobservations
num_of_trials = 1000;

% Build the matching linear measurement matrix.
A_matrix = zeros(num_states + num_landmarks * num_states, num_states + num_landmarks);
A_matrix(1, 1) = 1;  % Special initial-condition row: x_0 = 0
A_matrix = AStorageHandling(A_matrix, 1);

for k = 2:num_states
    A_matrix(k, k-1) = -1;
    A_matrix(k, k) = 1;
    A_matrix = AStorageHandling(A_matrix, k);
end

t0 = tic;
h = waitbar(0, 'Starting...');
cleanupObj = onCleanup(@() close(h));

state_abs_error = zeros(num_of_trials, num_states + num_landmarks);
landmark_estimate_history = zeros(num_of_trials, num_landmarks);
for trial = 1:num_of_trials
    % Build the measurement vector: initial condition, odometry, then
    % in-range landmark observations across the full there-and-back motion.
    num_pose_landmark_states = num_states + num_landmarks * num_states;
    b_vector = zeros(num_pose_landmark_states, 1);

    b_vector(1) = 0;
    b_vector = bStorageHandling(b_vector, 1);

    for time_step = 2:num_states
        b_vector(time_step) = odometryMeasurement(time_step);
        b_vector = bStorageHandling(b_vector, time_step);
    end

    % Remove placeholder rows for landmarks that were out of range.
    valid_rows = ~isnan(b_vector);
    b_valid = b_vector(valid_rows);
    if trial == 1
        A_valid = A_matrix(valid_rows, :);
    end

    % Solve for all robot poses and landmark locations in one batch.
    state_estimates = A_valid \ b_valid;
    landmark_estimates = state_estimates(num_states + 1:end);
    landmark_estimate_history(trial, :) = landmark_estimates.';

    state_truth = [x_vector_total(:); landmarks(:)];
    state_abs_error(trial, :) = abs(state_estimates - state_truth).';

    if mod(trial, 25) == 0 || trial == 1 || trial == num_of_trials
        elapsed = toc(t0);
        eta = elapsed * (num_of_trials - trial) / trial;
        msg = sprintf('Progress: %d/%d | ETA: %.1f s', trial, num_of_trials, eta);
        waitbar(trial / num_of_trials, h, msg);
    end
end

mean_state_abs_error = mean(state_abs_error, 1);
mean_landmark_estimates = mean(landmark_estimate_history, 1).';
mean_pose_abs_error = mean_state_abs_error(1:num_states);
mean_landmark_abs_error = mean_state_abs_error(num_states + 1:end);

figure;
plot(t_vector_total, mean_pose_abs_error, 'LineWidth', 1.2)
xlabel('Time (s)')
ylabel('Mean Absolute Robot Position Error (m)')
title('Deliverable 5: Mean Absolute Robot Position Error (Batch Least-Squares, Back-and-Forth Loop Closures)')
grid on

disp(table((1:num_landmarks).', landmarks, mean_landmark_estimates, mean_landmark_abs_error(:), ...
    'VariableNames', {'Landmark', 'Truth', 'MeanEstimate', 'MeanAbsError'}))
end
