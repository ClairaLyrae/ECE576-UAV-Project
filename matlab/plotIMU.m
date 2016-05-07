% Import Positioning Data
data = importdata('../uav_physics_log.txt');
t=data(1:100:end,1);
r=data(1:100:end,2:4);
v=data(1:100:end,5:7);
att=data(1:100:end,8:10);
attr=data(1:100:end,11:13);
n=data(1:100:end,14:16);
f=data(1:100:end,17:19);

% Plot positions
figure(2);
clf;
subplot(2,1,1);
grid on;
hold on;
title('UAV Flight Position');
xlabel('Time');
ylabel('Position (m)');
plot(t, r(:,1), '-b');
plot(t, r(:,2), '-g');
plot(t, r(:,3), '-r');
legend('X', 'Y', 'Z');
hold off;
subplot(2,1,2);
grid on;
hold on;
title('UAV Flight Velocities');
xlabel('Time');
ylabel('Velocity (m/s)');
plot(t, v(:,1), '-b');
plot(t, v(:,2), '-g');
plot(t, v(:,3), '-r');
legend('X', 'Y', 'Z');
hold off;
print('uav_pos_plot', '-dpng');


% Plot attitude
figure(3);
clf;
subplot(2,1,1);
grid on;
hold on;
title('UAV Flight Attitude');
xlabel('Time');
ylabel('Attitude (deg)');
plot(t, att(:,1)*180/pi, '-b');
plot(t, att(:,2)*180/pi, '-g');
plot(t, att(:,3)*180/pi, '-r');
legend('Roll', 'Pitch', 'Yaw');
hold off;
subplot(2,1,2);
grid on;
hold on;
title('UAV Flight Attitude Rate');
xlabel('Time');
ylabel('Attitude Rate (deg/s)');
plot(t, attr(:,1)*180/pi, '-b');
plot(t, attr(:,2)*180/pi, '-g');
plot(t, attr(:,3)*180/pi, '-r');
legend('Roll', 'Pitch', 'Yaw');
hold off;
print('uav_att_plot', '-dpng');
