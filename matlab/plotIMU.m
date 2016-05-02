% Import Positioning Data
data = importdata('../uav_physics_log.txt');
t=data(1:100:end,1);
r=data(1:100:end,2:4);
n=data(1:100:end,5:7);
f=data(1:100:end,8:10);
att=data(1:100:end,11:13);
attr=data(1:100:end,14:16);

% Plot positions
figure(2);
clf;
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


% Plot attitude
figure(3);
clf;
grid on;
hold on;
title('UAV Flight Attitude');
xlabel('Time');
ylabel('Attitude (deg)');
plot(t, att(:,1)*180/pi, '-b');
plot(t, att(:,2)*180/pi, '-g');
plot(t, att(:,3)*180/pi, '-r');
ylim([-180 180]);
legend('Roll', 'Pitch', 'Yaw');
hold off;

% Plot attitude rate
figure(4);
clf;
grid on;
hold on;
title('UAV Flight Attitude Rate');
xlabel('Time');
ylabel('Attitude (deg/s)');
plot(t, attr(:,1)*180/pi, '-b');
plot(t, attr(:,2)*180/pi, '-g');
plot(t, attr(:,3)*180/pi, '-r');
ylim([-180 180]);
legend('Roll', 'Pitch', 'Yaw');
hold off;
