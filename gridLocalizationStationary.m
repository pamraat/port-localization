function [pdfnew, XGrid, YGrid] = gridLocalizationStationary(n, m, map, Meas, pdf)

[x_min, x_max] = bounds([map(:, 1); map(:, 3)]);
[y_min, y_max] = bounds([map(:, 2); map(:, 4)]);
lx = (x_max - x_min)/n; ly = (y_max - y_min)/m;
XGrid = repmat((x_min + lx/2):lx:(x_max - lx/2), 1, m);
YGrid = repelem((y_min + ly/2):ly:(y_max - ly/2), 1, n);
dirXInd = [0 1 0 -1]; dirYInd = [1 0 -1 0];
expMeas = zeros(n*m, 4);

for i = 1:n*m
    dist = NaN*ones(size(map, 1), 4);
    for j = 1:size(map, 1)
        for k = 1:4
            [isect, x, y] = intersectPoint(map(j, 1), map(j, 2), map(j, 3), map(j, 4), XGrid(i), YGrid(i), XGrid(i) + 3*dirXInd(k), YGrid(i) + 3*dirYInd(k));
            if(isect)
                dist(j, k) = norm([x, y] - [XGrid(i), YGrid(i)]);
            end
        end
    end
    expMeas(i, :) = min(dist);
end

if nargin < 5 || isempty(pdf) 
    pdf = 1/(m*n - length(expMeas(sum(expMeas == 0, 2)>0)))*ones(n, m);
    pdf(mod(find(sum(expMeas == 0, 2)>0), n), floor(find(sum(expMeas == 0, 2)>0)/m) + 1) = 0;
end
if isempty(Meas)
    pdfnew = pdf;
    return
end

expMeas(isnan(expMeas)) = 5; Meas(isnan(Meas)) = 5;
pdfnew = reshape(mvnpdf(expMeas, Meas, diag([0.1^2, 0.3^2, 0.1^2, 0.3^2])), n, m).*pdf;
pdfnew = pdfnew/sum(pdfnew,"all");
