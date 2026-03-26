%{
% Author: Matthew Lepis
% Course: ME 656 - Autonomous Navigation for Mobile Robots
% Assignment: SimLab 1
% File: sim.m
%
% This driver script runs the batch least-squares simulations for:
%   sim1 -> Deliverable 1: odometry-only batch least-squares SLAM
%   sim2 -> Deliverable 2: batch least-squares SLAM with landmark sensing
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

sim3