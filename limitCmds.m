function[cmdV,cmdW] = limitCmds(fwdVel,angVel,maxV,wheel2Center)
% LIMITCMDS: scale forward and angular velocity commands to avoid
% saturating motors.
% 
%   [CMDV,CMDW] = LIMITCMDS(FWDVEL,ANGVEL,MAXV,WHEEL2CENTER) returns the 
%   forward and angular velocity commands to drive a differential-drive 
%   robot whose wheel motors saturate at +/- maxV.
% 
%   INPUTS
%       fwdVel      desired forward velocity (m/s)
%       angVel      desired angular velocity (rad/s)
%       maxV        maximum motor velocity (assumes symmetric saturation)
%       wheel2Center distance from the wheels to the center of the robot(in meters)
% 
%   OUTPUTS
%       cmdV        scaled forward velocity command (m/s)
%       cmdW        scaled angular velocity command (rad/s)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   PARMAR, PAMRAAT
sV = sign(fwdVel); sW = sign(angVel);
fwdVel = abs(fwdVel); angVel = abs(angVel);

V_test = angVel*wheel2Center + fwdVel;
B=[fwdVel;angVel];
if(V_test > maxV)
    A = [1 wheel2Center; angVel -fwdVel];
    B = A\[maxV; 0];
end
cmdV = sV*B(1); cmdW = sW*B(2);