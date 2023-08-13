function H = HjacGPS(x)
% HjacGPS: output the jacobian of the GPS measurement. Returns the H matrix
%
%   INPUTS
%       x            3-by-1 vector of pose
%
%   OUTPUTS
%       H            jacobian matrix of GPS measurement
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   Parmar, Pamraat
    H = eye(length(x));   
end