function [xhat_upd, P_upd] = correction_step(xhat_pred, P_pred, sensor_data, landmark_map, Rr)
    addpath('tools');
  N_obs = numel(sensor_data);
  n = length(xhat_pred);

  H = zeros(2*N_obs, n);
  z_pred = zeros(2*N_obs, 1);
  z_med  = zeros(2*N_obs, 1);

  xr  = xhat_pred(1);
  yr  = xhat_pred(2);
  thr = xhat_pred(3);
%%
  for k = 1:N_obs
    id  = sensor_data(k).id;
    idx = find(landmark_map == id, 1);

    xm = xhat_pred(3 + 2*(idx-1) + 1);
    ym = xhat_pred(3 + 2*(idx-1) + 2);

    dx = xm - xr;
    dy = ym - yr;
    r_pred = sqrt(dx^2 + dy^2);
    phi_pred = normalize_angle(atan2(dy, dx) - thr);
%%
    z_pred(2*k-1:2*k) = [r_pred; phi_pred];
    z_med(2*k-1:2*k) = [sensor_data(k).range; sensor_data(k).bearing];
%%
    H_r = [ 
      -dx/r_pred,    -dy/r_pred,     0;
       dy/r_pred^2,  -dx/r_pred^2,  -1
    ];
    H_m = [
       dx/r_pred,       dy/r_pred;
      -dy/r_pred^2,    dx/r_pred^2
    ];
%%
    Hi = zeros(2, n);
    Hi(:,1:3) = H_r;
    cols = 3 + 2*(idx-1) + (1:2);
    Hi(:, cols) = H_m;

    H(2*k-1:2*k, :) = Hi;
  end
  Rbig = kron(eye(N_obs), Rr);

  K = (P_pred * H') / (H * P_pred * H' + Rbig);

  nu = z_med - z_pred;

  xhat_upd = xhat_pred + K * nu;
  xhat_upd(3) = normalize_angle(xhat_upd(3));
  P_upd = (eye(n) - K*H) * P_pred;
end