function G = GjacDiffDrive(x, u)
% GjacDiffDrive: output the jacobian of the dynamics. Returns the G matrix
%
%   INPUTS
%       x            3-by-1 vector of pose
%       u            2-by-1 vector [d, phi]'
%
%   OUTPUTS
%       G            Jacobian matrix partial(g)/partial(x)
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   Last, First 
    R = [-sin(x(3)) -cos(x(3))
        cos(x(3)) -sin(x(3))];
    xR = [sin(u(2))*u(1)/u(2); (1 - cos(u(2)))*u(1)/u(2)];
    if u(2) == 0 xR = [u(1); 0]; end
    G = [1 0 0;
        0 1 0;
        (R*xR)' 1]'; 
end