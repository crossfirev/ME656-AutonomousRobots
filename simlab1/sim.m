%{
% Author: Matthew Lepis
% Course: ME 656 - Autonomous Navigation for Mobile Robots
% Assignment: SimLab 1
% File: sim.m
%
% This driver script runs the simulations for:
%   sim1 -> Deliverable 1: odometry-only batch least-squares SLAM
%   sim2 -> Deliverable 2: batch least-squares SLAM with landmark sensing
%   sim3 -> Deliverable 3: odometry-only Kalman filter SLAM
%}

% Run Deliverable 1. This script estimates the robot trajectory using only
% odometry measurements and plots mean absolute pose error over time.
sim1

% Run Deliverable 2. This script adds landmark observations to the batch
% least-squares system and shows how landmarks reduce trajectory drift.
sim2

%
% Deliverable 2 shows that adding landmark observations slows the growth of
% localization error compared with Deliverable 1, which relied on odometry
% alone. In the odometry-only case, error tends to accumulate over time
% because each noisy motion measurement builds on the previous one without
% any external correction. With landmark measurements included, the robot
% periodically receives additional information that anchors its estimated
% position to fixed features in the hallway, which helps correct drift and
% reduces overall mean absolute error.
%
% As a result, the Deliverable 2 error curve should grow more slowly and
% remain lower than the Deliverable 1 curve, especially when the robot is
% within sensing range of landmarks. This demonstrates the core benefit of
% SLAM: map features do not just get estimated, they also improve
% localization accuracy by constraining the robot's trajectory.

% Run Deliverable 3. This script switches from batch least-squares to an
% incremental Kalman filter with odometry-only sensing and plots both the
% mean absolute position error and the filter-predicted position standard
% deviation over time.
sim3

%
% Deliverable 3 shows the same odometry-only drift behavior as Deliverable 1,
% but now through the lens of recursive state estimation. Because the Kalman
% filter only receives noisy velocity measurements and no landmark updates,
% its position estimate becomes progressively less certain as the robot moves
% down the hallway. That growing uncertainty appears both in the empirical
% mean absolute error curve and in the covariance-based standard deviation.
%
% Comparing these two curves is useful because it checks whether the filter's
% internal uncertainty estimate is consistent with what actually happens over
% many trials. In the odometry-only case, both curves should increase with
% time, reflecting the fact that motion noise accumulates when there is no
% external position correction.
