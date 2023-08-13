clear

%% hGPS
% 
% x = [0; 0; 0];
% Loc = hGPS(x);
% disp('GPS measurement:')
% disp(Loc)


%% GjacDiffDrive
% 
% x = [0;0;0];
% u = [1; 0];
% G = GjacDiffDrive(x, u);
% disp('The Jacobian at x:')
% disp(G);

%% HjacGPS
% 
% x = rand(3,1);
% H = HjacGPS(x);
% disp('Jacobian of GPS measurements:');
% disp(H);

%% HjacDepth

% K = 9;
% sensor_pos = [0.13 0];
% map = [-4 4 4 4];
% robotPose = [0; 0; pi/2];
% H = HjacDepth(robotPose, map, sensor_pos, K);
% disp('Depth Jacobian:');
% disp(H);

%% testEKF

% % you can validate with this code or with your own parameters.
% map = [-1 -1 -1 1; -1 1 1 1; 1 1 1 -1; 1 -1 -1 -1];
% mu_prev = [0; 0; pi/2]; u_prev = [0.0; 0.0]; 
% sigma_prev = [0.1 0 0; 0 0.1 0; 0 0 0.1];
% z_gps = [0.0; 0.0; 0.0];
% z_depth = ones(9,1);
% R =  0.1*eye(3);
% Q_gps = 0.1*eye(3); 
% n_rs_rays = 9;
% Q_rsense = 0.1*eye(n_rs_rays); %rsense meas.
% sensor_pos = [0.13 0];
% 
% 
% [mu_gps, sigma_gps, mu_depth, sigma_depth] = ...
%     testEKF(mu_prev, u_prev, sigma_prev, R, z_gps, Q_gps, z_depth, Q_rsense, map, sensor_pos, n_rs_rays);
% disp('mu_gps = '); disp(mu_gps);
% disp('mu_depth = '); disp(mu_depth);

%% testPF

% map = [-1000 1 1000 1];
% u = [1; 0];
% n_rs_rays = 9;
% z = 2*ones(n_rs_rays, 1);
% angles = linspace(27, -27, n_rs_rays)*pi/180';
% sensor_pos = [0.13 0];
% nParticles = 30;
% X0 = [2*rand(1, nParticles) - 1;
%     4*rand(1, nParticles) - 3;
%     pi/2*ones(1, nParticles)];
% R = 0.01*eye(3);
% Q = 0.01*eye(n_rs_rays);
% h = @(x) depthPredict(x, map, sensor_pos, angles);
% g = @(x, u) integrateOdom(x, u(1), u(2));
% X = PF(X0, u, z, R, Q, g, h);
% 
% hold on
% scatter(X0(1, :), X0(2, :), 'o', 'filled', 'black');
% scatter(X(1, :), X(2, :), 'd', 'filled', 'red');
% for i = 1:size(map, 1)
%     plot([map(i, 1) map(i, 3)], [map(i, 2) map(i, 4)], LineWidth=1, Color='black');
% end
% title("Particle Filter Update", "Interpreter","tex");
% legend('\chi_0', '\chi', "Interpreter","tex", 'Location', 'best')
% xlabel("X (m)");
% ylabel("Y (m)");
% xlim([-1.2 1.2]);
% ylim([-3.2 1.2])
% fontsize(gca,14,"points");
% set(gcf, 'Position',  [400, 150, 600, 500]);

%% motionControl EKFDepth

% global dataStore
% 
% load 'cornerMap.mat'
% map = cornerMap;
% hold on
% p1 = plot(dataStore.deadReck(:, 1), dataStore.deadReck(:, 2), 'Linewidth', 1, 'Color', 'blue', 'DisplayName', 'Dead Reckoning');
% p2 = plot(dataStore.truthPose(:, 2), dataStore.truthPose(:, 3), 'Linewidth', 1, 'Color', 'green', 'DisplayName', 'O/H localization');
% p3 = plot(dataStore.ekfMu(:, 1), dataStore.ekfMu(:, 2), 'red',  'Linewidth', 1, 'DisplayName', 'EKF');
% for i = 1:size(map, 1)
%     plot([map(i, 1) map(i, 3)], [map(i, 2) map(i, 4)], LineWidth=1, Color='black');
% end
% for i = 1:200:size(dataStore.ekfMu, 1)
%     p4 = plotCovEllipse(dataStore.ekfMu(i, 1:2)', dataStore.ekfSigma(3*(i - 1) + 1 : 3*(i - 1) + 2, 1:2), 1, [{'color'},{'m'}, {'DisplayName'}, {'1-Sigma'}]);
% end
% legend([p1 p2 p3 p4],"Interpreter","tex", 'Location', 'best');
% hold on
% title("Robot Trajectory", "Interpreter","tex");
% xlabel("X (m)");
% ylabel("Y (m)");
% fontsize(gca,14,"points");
% set(gcf, 'Position',  [400, 150, 600, 500]);

%% motionControl PF
% 
% global dataStore
% 
% load 'cornerMap.mat'
% map = cornerMap;
% figure(1)
% hold on
% p1 = scatter(dataStore.particles(1, :), dataStore.particles(2, :), 'o', 'filled', 'black', 'DisplayName', 'Initial Particles');
% p2 = plot(dataStore.deadReck(:, 1), dataStore.deadReck(:, 2), 'Linewidth', 1, 'DisplayName', 'Dead Reckoning');
% p3 = plot(dataStore.truthPose(:, 2), dataStore.truthPose(:, 3), 'DisplayName', 'O/H Localization');
% for i = 1:100:length(dataStore.particles(:, 1))/4
%     [~, maxind] = max(dataStore.particles(4*(i - 1) + 4, :));
%     p4 = scatter(dataStore.particles(4*(i - 1) + 1, maxind), dataStore.particles(4*(i - 1) + 2, maxind), 'd', 'filled', 'red', 'DisplayName', 'Best Particle');
% end
% p5 = scatter(dataStore.particles(end - 3, :), dataStore.particles(end - 2, :), 's', 'filled', 'blue', 'DisplayName', 'Final Particles');
% for i = 1:size(map, 1)
%     plot([map(i, 1) map(i, 3)], [map(i, 2) map(i, 4)], LineWidth=1, Color='black');
% end
% legend([p1 p2 p3 p4 p5],"Interpreter","tex", 'Location', 'best');
% title("Robot Trajectory", "Interpreter","tex");
% xlabel("X (m)");
% ylabel("Y (m)");
% fontsize(gca,14,"points");
% set(gcf, 'Position',  [400, 150, 600, 500]);
% 
% figure(2)
% hold on
% for i = 1:length(dataStore.particles(:, 1))/4
%     [~, maxind] = max(dataStore.particles(4*(i - 1) + 4, :));
%     point(i, :) = [dataStore.particles(4*(i - 1) + 1, maxind) dataStore.particles(4*(i - 1) + 2, maxind)];
%     point1(i, :) = [dataStore.particles(4*(i - 1) + 1, floor(length(dataStore.particles(1, :))/5)), dataStore.particles(4*(i - 1) + 2, floor(length(dataStore.particles(1, :))/5))];
%     point2(i, :) = [dataStore.particles(4*(i - 1) + 1, floor(length(dataStore.particles(1, :))*2/20)), dataStore.particles(4*(i - 1) + 2, floor(length(dataStore.particles(1, :))*2/20))];
%     point3(i, :) = [dataStore.particles(4*(i - 1) + 1, floor(length(dataStore.particles(1, :))*3/20)), dataStore.particles(4*(i - 1) + 2, floor(length(dataStore.particles(1, :))*3/20))];
%     point4(i, :) = [dataStore.particles(4*(i - 1) + 1, floor(length(dataStore.particles(1, :))*4/20)), dataStore.particles(4*(i - 1) + 2, floor(length(dataStore.particles(1, :))*4/20))];
%     point5(i, :) = [dataStore.particles(4*(i - 1) + 1, floor(length(dataStore.particles(1, :))/4)), dataStore.particles(4*(i - 1) + 2, floor(length(dataStore.particles(1, :))/4))];
% end
% a1 = plot(point(:, 1), point(:, 2), 'Linewidth', 1, 'DisplayName', 'Best');
% a2 = plot(point1(:, 1), point1(:, 2), 'Linewidth', 1, 'DisplayName', 'Particle-1');
% a3 = plot(point2(:, 1), point2(:, 2), 'Linewidth', 1, 'DisplayName', 'Particle-2');
% a4 = plot(point3(:, 1), point3(:, 2), 'Linewidth', 1, 'DisplayName', 'Particle-3');
% a5 = plot(point4(:, 1), point4(:, 2), 'Linewidth', 1, 'DisplayName', 'Particle-4');
% a6 = plot(point5(:, 1), point5(:, 2), 'Linewidth', 1, 'DisplayName', 'Particle-5');
% for i = 1:size(map, 1)
%     plot([map(i, 1) map(i, 3)], [map(i, 2) map(i, 4)], LineWidth=1, Color='black');
% end
% legend([a1 a2 a3 a4 a5 a6],"Interpreter","tex", 'Location', 'best');
% title(" Particle Trajectory", "Interpreter","tex");
% xlabel("X (m)");
% ylabel("Y (m)");
% fontsize(gca,14,"points");
% set(gcf, 'Position',  [400, 150, 600, 500]);

%% motionControl PF

global dataStore

load 'cornerMap.mat'
map = cornerMap;
hold on
p1 = scatter(dataStore.particles(1, :), dataStore.particles(2, :), 'o', 'filled', 'black', 'DisplayName', 'Initial Particles');
p2 = plot(dataStore.deadReck(:, 1), dataStore.deadReck(:, 2), 'Linewidth', 1, 'DisplayName', 'Dead Reckoning');
p3 = plot(dataStore.truthPose(:, 2), dataStore.truthPose(:, 3), 'DisplayName', 'O/H Localization');
for i = 1:100:length(dataStore.particles(:, 1))/4
    [~, modeind] = mode(dataStore.particles(4*(i - 1) + 1, :));
    p4 = scatter(dataStore.particles(4*(i - 1) + 1, modeind), dataStore.particles(4*(i - 1) + 2, modeind), 'd', 'filled', 'red', 'DisplayName', 'Best Particle');
end
p5 = scatter(dataStore.particles(end - 3, :), dataStore.particles(end - 2, :), 's', 'filled', 'blue', 'DisplayName', 'Final Particles');
for i = 1:size(map, 1)
    plot([map(i, 1) map(i, 3)], [map(i, 2) map(i, 4)], LineWidth=1, Color='black');
end
legend([p1 p2 p3 p4 p5],"Interpreter","tex", 'Location', 'best');
title("Robot Trajectory", "Interpreter","tex");
xlabel("X (m)");
ylabel("Y (m)");
fontsize(gca,14,"points");
set(gcf, 'Position',  [400, 150, 600, 500]);