
data = importdata('../uav_physics_log.txt');
t=data(:,1);
r=data(:,2:4);
plot3(r(:,1),r(:,2),r(:,3));
grid on;
zlim([0 inf]);
title('UAV Flight Path');

lx = xlim;
ly = ylim;

gx = [lx(1) lx(1) lx(2) lx(2)];
gy = [ly(1) ly(2) ly(2) ly(1)];
patch(gx,gy,[0 0 0 0],'green')