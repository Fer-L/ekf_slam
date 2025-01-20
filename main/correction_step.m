function [X, P, lm, H, R] = correction_step(X_pred, P_pred, sensor_data, N, lm)
addpath('tools');

Z = zeros(size(sensor_data, 2) * 2, 1);
Z_pred = zeros(size(sensor_data, 2) * 2, 1);
H = [];

for i = 1:size(sensor_data, 2)
    %j = sensor_data(i).id
    lm_id = sensor_data(i).id;
    range = sensor_data(i).range;
    phi = sensor_data(i).bearing;

    if(lm(lm_id)==false)
        X_pred(2 * lm_id + 2:2 * lm_id + 3) = [X_pred(1); X_pred(2)] + [range * cos(phi + X_pred(3)); range * sin(phi + X_pred(3))];
		lm(lm_id) = true;
    end  
    Z(i*2-1:i*2) = [range; phi];
    delta = [X_pred(2 * lm_id + 2) - X_pred(1); X_pred(2 * lm_id + 3) - X_pred(2)];
    q = delta' * delta;
    Z_pred(i*2-1:i*2) = [sqrt(q); atan2(delta(2), delta(1)) - X_pred(3)];
    
    Gxj = [eye(3) zeros(3,(2*lm_id-2)) zeros(3,2) zeros(3,(2*N-2*lm_id));
            zeros(2,3) zeros(2,(2*lm_id -2)) eye(2) zeros(2,(2*N-2*lm_id))];
    h = 1/q * [-sqrt(q)*delta(1) -sqrt(q)*delta(2) 0 sqrt(q)*delta(1) sqrt(q)*delta(2);
        delta(2) -delta(1) -q -delta(2) delta(1)] * Gxj;
    H = [H; h];
end
    R = eye(size(sensor_data, 2) * 2) * 0.01;
    K = P_pred * H' * inv(H * P_pred * H' + R);
    X_pred = X_pred + K * normalize_all_bearings(Z - Z_pred);
    X_pred(3) = normalize_angle(X_pred(3));
    P_pred = (eye(size(X_pred, 1)) - K * H) * P_pred;

    X = X_pred;
    P = P_pred;
end
