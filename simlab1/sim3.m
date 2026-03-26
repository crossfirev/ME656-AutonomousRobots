function sim3
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

%
%%%
%

% Plant : X(k+1) = A * X_k
%
% X_k = [^x_dk, ^x_k, ^l_1k, ^l_2k, ^l_3k]
%
% A = [   1,    0, 0, 0, 0;
%      delta_t, 1, 0, 0, 0;
%         0,    0, 1, 0, 0;
%         0,    0, 0, 1, 0;
%         0,    0, 0, 0, 1;]
%
% P_0 = [ 1,    0  , 0, 0, 0;
%         0, 0.0001, 0, 0, 0;
%         0,    0  , 1, 0, 0;
%         0,    0  , 0, 1, 0;
%         0,    0  , 0, 0, 1;
%
%
%]
%
%

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

    function H = genH()
        H = [1, 0, 0, 0, 0];
    end

    function R = genR()
        R = stdev_odometry^2;
    end

    function [x_k_predict, P_k_predict] = predict(x_k_last, P_k_last)
        x_k_predict = A * x_k_last;
        P_k_predict = A * P_k_last * transpose(A) + Q;
    end

    function [x_k, P_k] = correct(x_k_predict, P_k_predict, z_k, H, R)
        H_T = transpose(H);

        K_k = P_k_predict * H_T / (H * P_k_predict * H_T + R);

        x_k = x_k_predict + K_k * (z_k - H * x_k_predict);
        P_k = (eye(5,5) - K_k * H) * P_k_predict;
    end

    function velocity_meas = odometryMeasurement(time_step)
        odo_noise = stdev_odometry * randn();
        velocity_meas = (v_vector(time_step) + odo_noise);
    end

    function z_k = gatherMeasurements(time_step)
        velocity_meas = odometryMeasurement(time_step);
        position_meas = nan;  % Odometry only, no position
        landmark1_meas = nan;  % Odometry only, no range sensing to Landmark 1
        landmark2_meas = nan;  % Odometry only, no range sensing to Landmark 2
        landmark3_meas = nan;  % Odometry only, no range sensing to Landmark 3

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

num_of_trials = 1000;

std_position_error = zeros(1, num_states);
estimated_position = zeros(num_of_trials, num_states);
for trial = 1:num_of_trials
    % k = 1, aka t=0
    x_k_last = x_0;
    P_k_last = P_0;
    estimated_position(trial, 1) = x_0(2);
    if trial == 1
        std_position_error(1) = sqrt(P_0(2,2));
    end
    for k = 2:num_states
        %  Step 1: Predict
        [x_k_predict, P_k_predict] = predict(x_k_last, P_k_last);

        %  Step 2: Correct
        z_k = gatherMeasurements(k);
        H = genH();
        R = genR();
        [x_k, P_k] = correct(x_k_predict, P_k_predict, z_k, H, R);

        x_k_last = x_k;
        P_k_last = P_k;
        estimated_position(trial, k) = x_k(2);
        if trial == 1
            std_position_error(k) = sqrt(P_k(2,2));
        end
    end
end
trial_abs_position_error = abs(estimated_position - x_vector).';
mean_trial_abs_position_error = mean(trial_abs_position_error, 2);

figure; 
plot(t_vector, mean_trial_abs_position_error, 'LineWidth', 1.2)
hold on
plot(t_vector, std_position_error, '--', 'LineWidth', 1.2)
xlabel('Time (s)')
ylabel('Error / Predicted Standard Deviation (m)')
title('Odometry-Only Kalman Filter: Mean Absolute Error and Predicted Position Std Dev')
legend('Mean Absolute Position Error', 'Predicted Position Std Dev', 'Location', 'northwest')
grid on
end
