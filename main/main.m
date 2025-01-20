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

N = size(landmarks,2);

% Matrizes de incertezas do sistema
LambdaHat = 0;
%m = eye(3+2*N,3+2*N); %[0; 0.0198];
% mudar para eye
%Ef = eye(3+2*N, 3+2*N); % [0 5];
%Eg = zeros(3, 3+2*N); %[0 0];
%-------------------------------------------------------------------
lm = false(1,N);
X = zeros(2 * N + 3, 1);
XHat = X;
P = [zeros(3), zeros(3, 2 * N); zeros(3, 2 * N)', zeros(2 * N)];

% initialize diagonal of pose covariance with small, nonzero values
% since we have a good estimate of the starting pose
P = zeros(2 * N + 3, 2 * N + 3);
P(1:3,1:3) = 0.001;

% initialize landmark variances to large values since we have
% no prior information on the locations of the landmarks
for i = 4:N*2+3
    P(i,i) = 10;
end

motion = [];

for k = 1:size(sensor_readings.timestep, 2)

    [X, P, Q, Gx, Fk] = prediction_step(sensor_readings.timestep(k).odometry, X, P, N);
    
    [X, P, lm, H, R] = correction_step(X, P, sensor_readings.timestep(k).sensor, N, lm);
    
    %if (k == 1)
    %    LambdaHat = (1 + 0.5) * norm(m' * H' * inv(R) * H * m);
        %invLambdaHat = inv(LambdaHat);
    %end

    %[x, p] = robusto(x, p, Q, gk, Fk, H, R, m, LambdaHat, Ef, Eg, N);

    motion = plot_state(X, P, landmarks, lm, sensor_readings.timestep(k).sensor, showGui, motion);
end

% for system tuning
%motion = plot_state(X, P, landmarks, k, showGui, motion);
%hold on;
%plot()

