function H = HjacDepth(x, map, sensor_pos, K)
    eps = [1e-3 1e-3 1e-8];
    angles = linspace(27, -27, K)'*pi/180;
    for i=1:3
        a = zeros(3, 1); a(i)= eps(i);
        H(:, i) = (depthPredict(x + a, map, sensor_pos, angles) - depthPredict(x, map, sensor_pos, angles))/eps(i);
    end
end