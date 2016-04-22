% Import Positioning Data
data = importdata('../uav_physics_log_horiz.txt');
t=data(1:100:end,1);
r=data(1:100:end,2:4);
n=data(1:100:end,5:7);

% Plot figure
figure(1);
clf;
grid on;
hold on;
title('UAV Flight Path');
lx = max(max(abs(r(:))));
xlim([-lx lx])
ylim([-lx lx])
zlim([0 lx*1.1])
gx = [-lx -lx lx lx];
gy = [-lx lx lx -lx];
patch(gx,gy,[0 0 0 0],'green');
view([45 45]);
qplot = quiver3(r(:,1),r(:,2),r(:,3),n(:,1),n(:,2),n(:,3), 0.5);
animatePlot3(r(:,1),r(:,2),r(:,3), 'cFigure', 1, 'blockSize', inf, 'Frequency', 10);
hold off;