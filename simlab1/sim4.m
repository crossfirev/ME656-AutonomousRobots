function sim4
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

% Kalman filter state:
%   [v_k, x_k, l_1, l_2, l_3]^T
%
% Process model:
%   X_(k+1) = A * X_k + w_k
% where the robot keeps a constant velocity and the landmarks remain static.
%
% Deliverable 4 uses both odometry and landmark updates:
%   z_k = H_k * X_k + n_k
% The measurement model changes size with time because the robot always
% receives one odometry measurement, and may also receive one or more
% landmark range measurements whenever landmarks fall within sensor range.
%
% Odometry is modeled as a direct noisy measurement of the velocity state.
% Landmark range measurements are modeled as signed displacement:
%   l_j - x_k + noise
% which keeps the measurement model linear in the unknown states.
%
% The initial covariance reflects:
%   var(v_0) = 1
%   var(x_0) = 0.0001
%   var(l_j) = 1

Q_11 = 1e-8;

velocity_variance = 1;
position_variance = 1e-4;
landmark_1_variance = 1;
landmark_2_variance = 1;
landmark_3_variance = 1;

P_0 = ...
    [
    velocity_variance, 0, 0, 0, 0;
    0, position_variance, 0, 0, 0;
    0, 0, landmark_1_variance, 0, 0;
    0, 0, 0, landmark_2_variance, 0;
    0, 0, 0, 0, landmark_3_variance
    ];

x_0 = [ v_robot;    0;  0;  0;  0; ];

A = [ 1, 0, 0, 0, 0;    delta_t, 1, 0, 0, 0;    0, 0, 1, 0, 0;      0, 0, 0, 1, 0;      0, 0, 0, 0, 1 ];

Q = [ Q_11, 0, 0, 0, 0; 0, 0, 0, 0, 0;          0, 0, 0, 0, 0;      0, 0, 0, 0, 0;      0, 0, 0, 0, 0 ];

    function in_range = inRange(time_step, true_landmark_pose)
        % Return whether the robot can sense this landmark at the current time-step.
        if abs(true_landmark_pose - x_vector(time_step)) <= sensor_range
            in_range = true;
        else
            in_range = false;
        end
    end

    function landmarks_in_range = whichInRange(time_step)
        % Collect the indices of all landmarks that are visible right now.
        landmarks_in_range = [];
        for landmark = 1:num_landmarks
            true_landmark_pose = landmarks(landmark);
  
            if inRange(time_step, true_landmark_pose)
                landmarks_in_range = [landmarks_in_range, landmark];
            end       
        end
    end

    function H = genH(time_step)
        % Build the time-varying measurement matrix.
        % Row 1 is odometry, and each additional row encodes:
        %   l_j - x_k = z_range(k,j)
        in_range_landmarks = whichInRange(time_step);

        H = [1, 0, 0, 0, 0];
        for landmark = in_range_landmarks
            row = zeros(1, 5);
            row(2) = -1;
            row(landmark + 2) = 1;   % landmark 1 -> col 3, 2 -> col 4, 3 -> col 5
            H = [H; row];
        end
    end

    function R = genR(time_step)
        % Build the time-varying diagonal sensor covariance matrix.
        % Odometry is always present; each visible landmark adds one
        % independent range variance on the diagonal.
        in_range_landmarks = whichInRange(time_step);
        
        variances = stdev_odometry^2;
        for landmark = in_range_landmarks
            variances = [variances, stdev_range^2];
        end
        R = diag(variances);
    end

    function [x_k_predict, P_k_predict] = predict(x_k_last, P_k_last)
        % Time update: propagate the state estimate and its covariance.
        x_k_predict = A * x_k_last;
        P_k_predict = A * P_k_last * A' + Q;
    end

    function [x_k, P_k] = correct(x_k_predict, P_k_predict, z_k, H, R)
        % Measurement update with the current odometry and landmark readings.
        H_T = transpose(H);

        K_k = P_k_predict * H_T / (H * P_k_predict * H_T + R);

        x_k = x_k_predict + K_k * (z_k - H * x_k_predict);
        P_k = (eye(5,5) - K_k * H) * P_k_predict;
    end

    function velocity_meas = odometryMeasurement(time_step)
        % Simulate one noisy odometry velocity measurement.
        odo_noise = stdev_odometry * randn();
        velocity_meas = (v_vector(time_step) + odo_noise);
    end

    function [landmark1_meas, landmark2_meas, landmark3_meas] = landmarkMeasurements(time_step)
        % Simulate signed range measurements for any landmarks currently in range.
        measurements = nan(1, num_landmarks);
        for landmark = 1:num_landmarks
            true_landmark_pose = landmarks(landmark);

            if inRange(time_step, true_landmark_pose)
                true_robot_pose = x_vector(time_step);
                range_sensor_noise = stdev_range * randn();

                % Signed displacement keeps the Kalman measurement model linear.
                noisy_range_to_landmark = true_landmark_pose - true_robot_pose + range_sensor_noise;
                measurements(landmark) = noisy_range_to_landmark;
            end
        end
        tmp = num2cell(measurements);
        [landmark1_meas, landmark2_meas, landmark3_meas] = tmp{:};
    end

    function z_k = gatherMeasurements(time_step)
        % Build the measurement vector for one time-step.
        % Deliverable 4 uses odometry and landmark sensing.
        velocity_meas = odometryMeasurement(time_step);
        position_meas = nan;  % No direct position measurement
        [landmark1_meas, landmark2_meas, landmark3_meas] = landmarkMeasurements(time_step);

        z_k = [
            velocity_meas;
            position_meas;
            landmark1_meas;
            landmark2_meas;
            landmark3_meas;
            ];
        valid_rows = ~isnan(z_k);
        z_k = z_k(valid_rows);
    end

%% Deliverable 4: Kalman Filter with Odometry + Landmark Measurements
num_of_trials = 1000;

std_position_error = zeros(1, num_states);
estimated_position = zeros(num_of_trials, num_states);
for trial = 1:num_of_trials
    % k = 1, aka t=0
    x_k_last = x_0;
    P_k_last = P_0;
    estimated_position(trial, 1) = x_0(2);
    if trial == 1
        % P evolution is deterministic here, so one trial is sufficient for the sigma curve.
        std_position_error(1) = sqrt(P_0(2,2));
    end
    for k = 2:num_states
        % Step 1: Predict
        [x_k_predict, P_k_predict] = predict(x_k_last, P_k_last);

        % Step 2: Correct using the available measurements at this time-step.
        z_k = gatherMeasurements(k);
        H = genH(k);
        R = genR(k);
        [x_k, P_k] = correct(x_k_predict, P_k_predict, z_k, H, R);

        x_k_last = x_k;
        P_k_last = P_k;
        estimated_position(trial, k) = x_k(2);
        if trial == 1
            % P evolution is deterministic here, so one trial is sufficient for the sigma curve.
            std_position_error(k) = sqrt(P_k(2,2));
        end
    end
end

% Compare each trial's estimated trajectory against the ground-truth hallway motion.
trial_abs_position_error = abs(estimated_position - x_vector).';
mean_trial_abs_position_error = mean(trial_abs_position_error, 2);

% Plot the Monte Carlo mean absolute error and the covariance-predicted
% position standard deviation on the same axes.
figure; 
plot(t_vector, mean_trial_abs_position_error, 'LineWidth', 1.2)
hold on
plot(t_vector, std_position_error, '--', 'LineWidth', 1.2)
xlabel('Time (s)')
ylabel('Error / Predicted Standard Deviation (m)')
title('Kalman Filter with Landmark Sensing: Mean Absolute Error and Predicted Position Std Dev')
legend('Mean Absolute Position Error', 'Predicted Position Std Dev', 'Location', 'northwest')
grid on
end
