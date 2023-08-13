function[depth] = depthPredict(robotPose,map,sensorOrigin,angles)
% DEPTHPREDICT: predict the depth measurements for a robot given its pose
% and the map
%
%   DEPTH = DEPTHPREDICT(ROBOTPOSE,MAP,SENSORORIGIN,ANGLES) returns
%   the expected depth measurements for a robot 
%
%   INPUTS
%       robotPose   	3-by-1 pose vector in global coordinates (x,y,theta)
%       map         	N-by-4 matrix containing the coordinates of walls in
%                   	the environment: [x1, y1, x2, y2]
%       sensorOrigin	origin of the sensor-fixed frame in the robot frame: [x y]
%       angles      	K-by-1 vector of the angular orientation of the range
%                   	sensor(s) in the sensor-fixed frame, where 0 points
%                   	forward. All sensors are located at the origin of
%                   	the sensor-fixed frame.
%
%   OUTPUTS
%       depth       	K-by-1 vector of depths (meters)
%
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 2

eps = 1000;
sensorPose = robot2global(robotPose', sensorOrigin); sensorPose(3) = robotPose(3);
depth = -1*ones(length(angles), 1);

for k = 1:length(angles)
    for i = 1:size(map, 1)
        [isect,x,y] = intersectPoint(map(i, 1), map(i, 2), map(i, 3), map(i, 4), sensorPose(1), sensorPose(2), sensorPose(1) + eps*cos(sensorPose(3) + angles(k)), sensorPose(2) + eps*sin(sensorPose(3) + angles(k)));
        if isect == 1
            xyS= global2robot(sensorPose, [x y]);
            d(i) = xyS(1);
            if d(i) == 0 depth(k) = 0; break; end
            if any(d > 0) depth(k) = min(d(d > 0)); end
        end
    end
    d = [];
end
depth(depth == -1) = NaN;