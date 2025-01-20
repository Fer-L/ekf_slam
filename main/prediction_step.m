function [X_pred, P_pred, Q, Gx, Fk] = prediction_step(sensor_data, X, P, N) %alterar o que retorna
addpath('tools');

% Motion noise
Q = [0.1, 0, 0; 
     0, 0.1, 0; 
     0, 0, 0.1/10];

Gx = [eye(3), zeros(3, 2 * N)];
X_pred = X + Gx' * [(sensor_data.t * cos(X(3)+sensor_data.r1)); (sensor_data.t * sin(X(3) + sensor_data.r1)); (sensor_data.r1 + sensor_data.r2)];
Fk = eye(size(X,1)) + Gx' * [0 0 (sensor_data.t * -sin(X(3)+sensor_data.r1)); 0 0 (sensor_data.t * cos(X(3) + sensor_data.r1)); 0 0 0] * Gx; 
P_pred = Fk * P * Fk' + Gx' * Q * Gx;
end