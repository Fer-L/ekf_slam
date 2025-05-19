function [xhat_pred, P_pred, M, landmark_map] = prediction_step(odom_data, sensor_data, xhat, Pr, M, Qr, Rr, landmark_map)
    addpath('tools');
    
    r1 = odom_data.r1; t = odom_data.t; r2 = odom_data.r2;
    xr = xhat(1); yr = xhat(2); thr = xhat(3);
    
    xrpred = xr + (t * cos(thr + r1));
    yrpred = yr + (t * sin(thr + r1));
    thrpred = normalize_angle(thr + (r1 + r2));
    
    xhat_pred = [ xrpred; yrpred; thrpred; xhat(4:end) ];
    
    Fr = [ 1, 0, -t*sin(thr + r1);
           0, 1,  t*cos(thr + r1);
           0, 0,  1 ];
    F  = blkdiag(Fr, eye(2*M));

    Gr  = [cos(thr+r1), 0;
           sin(thr+r1), 0;
           0,          1];
    
    G = [Gr; zeros(2*M, 2)];

    P_pred = F * Pr * F.' + G * Qr * G.';
    
    for k = 1:size(sensor_data, 2)
        id = sensor_data(k).id;
    
        if ~ismember(id, landmark_map)
            M = M + 1; 

            rr = sensor_data(k).range;
            phi = sensor_data(k).bearing;
    
            pos_m = [xrpred + rr*cos(thrpred+phi);
                     yrpred  + rr*sin(thrpred+phi) ];
            
            xhat_pred = [xhat_pred; pos_m];
            
            landmark_map(end+1) = id;

            G_mxk = [1, 0, -rr*sin(thrpred+phi);
                     0, 1,  rr*cos(thrpred+phi)];
    
            G_mzk = [ cos(thrpred+phi), -rr*sin(thrpred+phi);
                    sin(thrpred+phi),  rr*cos(thrpred+phi) ]; 
            
            Ym = [eye(size(P_pred,1)), zeros(size(P_pred,1),2);
                [G_mxk, zeros(2, size(P_pred,1)-3)], G_mzk ];
    
            P_pred = Ym * blkdiag(P_pred, Rr) * Ym';
        end  
    end
end