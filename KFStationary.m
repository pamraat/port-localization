function [munew, sigmanew] = KFStationary(mu, sigma, map, Meas)
if nargin < 4 || isempty(Meas)
    munew = mu; sigmanew = sigma;
    return
end
dirXInd = [0 1 0 -1]; dirYInd = [1 0 -1 0];
dist = NaN*ones(size(map, 1), 4);
for j = 1:size(map, 1)
    for k = 1:4
        [isect, x, y] = intersectPoint(map(j, 1), map(j, 2), map(j, 3), map(j, 4), mu(1), mu(2), mu(1) + 1000*dirXInd(k), mu(2) + 1000*dirYInd(k));
        if(isect)
            dist(j, k) = norm([x, y] - [mu(1), mu(2)]);
        end
    end
end
expMeas = min(dist);
expMeas(isnan(expMeas)) = 0;
C = [0 -1; -1 0; 0 1; 1 0];
C = C(~isnan(Meas), :); expMeas = expMeas(~isnan(Meas)); Meas = Meas(~isnan(Meas));
diaQ = [0.1^2, 0.3^2, 0.1^2, 0.3^2];
Q = diag(diaQ(~isnan(Meas)));
Kt = sigma*C'*(C*sigma*C' + Q)^-1;
munew = mu + Kt*(Meas - expMeas)';
sigmanew = (eye(length(mu)) - Kt*C)*sigma;