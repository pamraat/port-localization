function[xyR] = global2robot(pose,xyG)
% GLOBAL2ROBOT: transform a 2D point in global coordinates into robot
% coordinates (assumes planar world).
% 
%   XYR = GLOBAL2ROBOT(POSE,XYG) returns the 2D point in robot coordinates
%   corresponding to a 2D point in global coordinates.
% 
%   INPUTS
%       pose    robot's current pose [x y theta]  (1-by-3)
%       xyG     2D point in global coordinates (1-by-2)
% 
%   OUTPUTS
%       xyR     2D point in robot coordinates (1-by-2)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   PARMAR, PAMRAAT

R_GR = [cos(pose(3)) sin(pose(3)) 0
        -sin(pose(3)) cos(pose(3)) 0
        0 0 1];
Tr_GR = [1 0 -pose(1)
        0 1 -pose(2)
        0 0 1];

T_RG = R_GR*Tr_GR;
xyR = T_RG*[xyG'; 1];
xyR(3) = [];
xyR = xyR';