% Turn off pagination:
more off;

% clear all variables and close all windows
clear;
close all;

% Make tools available
addpath('tools');

% Read landmarks. The true landmark positions are not given to the robot
disp("Reading landmark positions")
landmarks = read_world('../data/world.dat');

% Read sensor readings, i.e. odometry and range-bearing sensor
disp("Reading sensor data")
sensor_readings = read_data('../data/sensor_data.dat');

% toogle the visualization type
showGui = true;  % show a window while the algorithm runs
% showGui = false; % plot to files instead

M = 0;
landmark_map = [];
xhat = zeros(3, 1);
Qr = [0.05 0
      0 0.05];
% initialize diagonal of pose covariance with small, nonzero values
% since we have a good estimate of the starting pose
Pr = zeros(2 * M + 3);
P = Pr;
Rr = [0.1,         0;
      0,         0.01];
% initialize landmark variances to large values since we have
% no prior information on the locations of the landmarks


motion = [];

for k = 1:size(sensor_readings.timestep, 2)

    % Predict Step
    [xhat_pred, P_pred, M, landmark_map] = prediction_step(sensor_readings.timestep(k).odometry, sensor_readings.timestep(k).sensor, xhat, P, M, Qr, Rr, landmark_map);
    
    % Update Step
    [xhat, P] = correction_step(xhat_pred, P_pred, sensor_readings.timestep(k).sensor, landmark_map, Rr);

    motion = plot_state(xhat, P, landmarks, landmark_map, sensor_readings.timestep(k).sensor, showGui, motion);
    
end
disp(P);
disp(xhat);

