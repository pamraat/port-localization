function [finalPose] = integrateOdom(initPose,d,phi)
% integrateOdom: Calculate the robot pose in the initial frame based on the
% odometry
% 
% [finalPose] = integrateOdom(initPose,dis,phi) returns a 3-by-N matrix of the
% robot pose in the initial frame, consisting of x, y, and theta.

%   INPUTS
%       initPose    robot's initial pose [x y theta]  (3-by-1)
%       d     distance vectors returned by DistanceSensorRoomba (1-by-N)
%       phi     angle vectors returned by AngleSensorRoomba (1-by-N)

% 
%   OUTPUTS
%       finalPose     The final pose of the robot in the initial frame
%       (3-by-N)

%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Homework #2
finalPose(:, 1) = initPose;
for i = 1:length(d)
    xyR = [d(i)/phi(i)*sin(phi(i)), d(i)/phi(i)*(1 - cos(phi(i)))];
    if(phi(i) == 0)
        xyR = [d(i), 0];
    end
    finalPose([1 2], i + 1) = robot2global(finalPose(:, i)', xyR)';
    finalPose(3, i + 1) = finalPose(3, i) + phi(i);
end
finalPose(:, 1) = [];