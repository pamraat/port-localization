function[dataStore] = motionControl(Robot,maxTime)
% TURNINPLACE: simple example program to use with iRobot Create (or simulator).
% Reads data from sensors, makes the robot turn in place and saves a datalog.
% 
%   dataStore = TURNINPLACE(Robot,maxTime) runs 
% 
%   INPUTStype
%       Robot       Port configurations and robot name (get from running CreatePiInit)
%       maxTime     max time to run program (in seconds)
% 
%   OUTPUTS
%       dataStore   struct containing logged data

% 
%   NOTE: Assume differential-drive robot whose wheels turn at a constant 
%         rate between sensor readings.
% 
%   Cornell University
%   MAE 5180: Autonomous Mobile Robots
%
% 	Modified: Liran 2023


% Set unspecified inputs
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 100;
end

try 
    % When running with the real robot, we need to define the appropriate 
    % ports. This will fail when NOT connected to a physical robot 
    CreatePort=Robot.CreatePort;
catch
    % If not real robot, then we are using the simulator object
    CreatePort = Robot;
end

% declare dataStore as a global variable so it can be accessed from the
% workspace even if the program is stopped
global dataStore;

% initialize datalog struct (customize according to needs)
dataStore = struct('truthPose', [],...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'beacon', [], ...
                   'deadReck', [], ...
                   'gps', []);

% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;

SetFwdVelAngVelCreate(Robot, 0, 0);
wheel2Center = 0.13;
maxV = 0.5;

% Input following value for 'choice' variable
% 1 for EKF with GPS
% 2 for EKF with Depth
% 3 for PF with depth
choice = 3;

R = 0.01*eye(3);
% R = 0.001*eye(3);
g = @(x, u) integrateOdom(x, u(1), u(2));
G = @(x, u) GjacDiffDrive(x, u);
switch choice
    case 1
        h = @(x) hGPS(x);
        H = @(x) HjacGPS(x);
        measDim = 3;
    case 2
        sensor_pos = [0 0.08];
        load 'cornerMap.mat'
        map = cornerMap;
        angles = linspace(27, -27, 9)'*pi/180;
        h = @(x) depthPredict(x, map, sensor_pos, angles);
        H = @(x) HjacDepth(x, map, sensor_pos, length(angles));
        measDim = length(angles);
    case 3
        nParticles = 20;
        sensor_pos = [0 0.08];
        load 'cornerMap.mat'
        map = cornerMap;
        angles = linspace(27, -27, 9)'*pi/180;
        h = @(x) depthPredict(x, map, sensor_pos, angles);
        measDim = length(angles);
    otherwise
        disp('Set choice variable as 1, 2 or 3.')
        return
end
Q = 0.001*eye(measDim);
% Q = 0.01*eye(measDim);

% Set angular velocity
fwdVel = 0.3;
angVel = 0.1;
timrev = -1;
timturn = -1;
tic

%% Initialization

[px, py, pt] = OverheadLocalizationCreate(Robot);
if(choice == 1 || choice == 2)
    dataStore.ekfMu = [px py pt];
%     dataStore.ekfMu = [-2, -1.5, pi/2];

    dataStore.ekfSigma = [2, 0, 0;
                         0, 2, 0;
                         0, 0, 0.1];    
%     dataStore.ekfSigma = [4, 0, 0;
%                          0, 4, 0;
%                          0, 0, 0.02];
else
    dataStore.particles = [5*rand(1, nParticles) - 5;
                          10*rand(1, nParticles) - 5;
                          0.4*rand(1, nParticles) - 0.2
                          ones(1, nParticles)/nParticles];
end

while toc < maxTime
    
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);

    deadReckpose = integrateOdom(dataStore.truthPose(end, 2:end)', dataStore.odometry(end, 2), dataStore.odometry(end, 3))';
    dataStore.deadReck = [dataStore.deadReck; ...
                          deadReckpose];

    dataStore.gps = [dataStore.gps; ...
                    awgn(dataStore.truthPose(end, 2:end), 40)];
    
    % CONTROL FUNCTION (send robot commands)
    if(dataStore.bump(end,2) == 1 || dataStore.bump(end,3) == 1 || dataStore.bump(end,7) == 1)
        timrev = dataStore.bump(end,1);
    end

    if(choice == 1 || choice == 2)
        if(choice == 1)
            z = dataStore.gps(end, :)';
        else
            z = dataStore.rsdepth(end,3:end)';
        end
        [ekfMu, ekfSigma] = EKF(dataStore.ekfMu(end, :)',...
            dataStore.ekfSigma(end-2:end, :), dataStore.odometry(end, 2:3)',...
            z, R, Q, g, G, h, H);
        dataStore.ekfMu = [dataStore.ekfMu; ...
                              ekfMu'];
        dataStore.ekfSigma = [dataStore.ekfSigma; ...
                              ekfSigma];
    else
        z = dataStore.rsdepth(end,3:end)';
        [X, w] = PF(dataStore.particles(end - 3:end - 1, :),...
            dataStore.odometry(end, 2:3)', z, R, Q, g, h);
        dataStore.particles = [dataStore.particles;
                                X; w];
    end
    
    if(timrev ~= -1)
        if(dataStore.bump(end,1) < timrev + 0.25/abs(fwdVel))
            fwdVelnew = -fwdVel;
            angVelnew = 0;
        else
            timrev = -1;
            fwdVelnew = 0;
            angVelnew = -1;
            timturn = dataStore.bump(end,1);
        end
    elseif(timturn ~= -1)
        if(dataStore.bump(end,1) < timturn + pi/3)
            fwdVelnew = 0;
            angVelnew = -0.5;
        else
            fwdVelnew = fwdVel;
            angVelnew = angVel;
            timturn = -0.5;
        end
    else
        fwdVelnew = fwdVel;
        angVelnew = angVel;
    end
    [cmdV,cmdW] = limitCmds(fwdVelnew,angVelnew,maxV,wheel2Center);
    
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(Robot, 0,0);
    else
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
    end
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(Robot, 0,0);