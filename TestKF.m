function [gridProb, muFinal, sigmaFinal] = TestFunHW3(map, measurements, gridSize, mu0, sigma0)
% Test function for Homework 3.  
% This function checks the student's grid localization and stationary KF implementation.
% 
%   INPUTS
%       map       	    N-by-4 matrix containing the coordinates of walls in
%                   	the environment: [x1, y1, x2, y2]
%       measurements	M-by-4 matrix containing the range measurements 
%                       in the global North, East, South and West directions
%       gridSize 	    Number of cells in the X dimension x number of 
%                       cells in the Y dimension     1x2 [n m]
%       mu0             The initial expected position for Stationary KF 2x1
%       sigma0          initial variance of the position estimate
%
%   OUTPUTS
%       gridProb      	n x m matrix representing the probability of the
%                       robot being in a certain grid cell
%       muFinal      	final expected position for Stationary KF 2x1
%       sigmaFinal      final variance of the position estimate

%~~~~~~~~~~~~~~~~~~~~~
% Grid localization
%~~~~~~~~~~~~~~~~~~~~~

n = gridSize(1); m = gridsize(2);
pdf = gridLocalizationStationary(n, m, map, measurements(1, :));
for i = 2:size(measurements, 1)
    pdf = gridLocalizationStationary(n, m, map, measurements(i, :), pdf);
end
gridProb = pdf';


%~~~~~~~~~~~~~~~~~~~~~
% Stationary KF
%~~~~~~~~~~~~~~~~~~~~~

[mu, sigma] = KFStationary(mu0, sigma0, map, measurements(1, :));
for i = 2:size(measurements, 1)
    [mu, sigma] = KFStationary(mu, sigma, map, measurements(i, :));
end
muFinal = mu; sigmaFinal = sigma;

end