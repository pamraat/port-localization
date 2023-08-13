function [mu_gps, sigma_gps, mu_depth, sigma_depth] = ...
    testEKF(mu_prev, u_prev, sigma_prev, R, z_gps, Q_GPS, z_depth, Q_depth, map, sensor_pos, n_rs_rays)
% testEKF: Performs one step of the extended Kalman Filter, outputs the belief given previous belief
%
%   INPUTS
%       mu_prev           previous vector of pose state (mu(t-1))
%       u_prev           previous command [d; phi]
%       sigma_prev        previous covariance matrix
%       R            state model noise covariance matrix
%       z_gps        current gps measurement vector
%       Q_GPS        GPS measurement noise covariance matrix
%       z_depth      depth measurement vector
%       Q_depth      Realsense depth measurement noise covariance matrix
%       map          map of the environment
%       sensor_pos   sensor position in the robot frame [x y]
%       n_rs_rays    number of evenly distributed realsense depth rays
%       (27...-27) degrees
%
%   OUTPUTS
%       mu_gps      current estimate of vector of pose state (gps)
%       sigma_gps   current covariance matrix (gps)
%       mu_depth    current estimate of vector of pose state (depth)
%       sigma_depth current covariance matrix (depth)
%
%   Cornell University
%   Autonomous Mobile Robots
%   Parmar, Pamraat

% estimate the state with GPS data using EKF

% estimate the state with depth data using EKF
    g = @(x, u) integrateOdom(x, u(1), u(2));
    G = @(x, u) GjacDiffDrive(x, u);
    h = @(x) hGPS(x);
    H = @(x) HjacGPS(x);
    [mu_gps, sigma_gps] = EKF(mu_prev, sigma_prev, u_prev, z_gps, R, Q_GPS, g, G, h, H);
    
    angles = linspace(27, -27, n_rs_rays)'*pi/180;
    h = @(x) depthPredict(x, map, sensor_pos, angles);
    H = @(x) HjacDepth(x, map, sensor_pos, n_rs_rays);
    [mu_depth, sigma_depth] = EKF(mu_prev, sigma_prev, u_prev, z_depth, R, Q_depth, g, G, h, H);    
end