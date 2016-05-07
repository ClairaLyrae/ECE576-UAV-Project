% Import Positioning Data
data = importdata('../uav_physics_log.txt');
t=data(1:100:end,1);
r=data(1:100:end,2:4);
v=data(1:100:end,5:7);
att=data(1:100:end,8:10);
attr=data(1:100:end,11:13);
n=data(1:100:end,14:16);
f=data(1:100:end,17:19);

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
quiver3(r(:,1),r(:,2),r(:,3),n(:,1),n(:,2),n(:,3), 0.5, 'b');
quiver3(r(:,1),r(:,2),r(:,3),f(:,1),f(:,2),f(:,3), 0.5, 'r');
animatePlot3(r(:,1),r(:,2),r(:,3), 'cFigure', 1, 'blockSize', inf, 'Frequency', 10);
hold off;

