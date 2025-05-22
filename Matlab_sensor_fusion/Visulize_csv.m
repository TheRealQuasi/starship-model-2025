
% Load sensor data
data = readmatrix("all_sensor_shake_data.csv");
data2 = readmatrix("Madgwick_output.csv");

% Extract and convert units
acc = data(:,1:3) / 5460 * 9.81;     % m/s²
gyro = deg2rad(data(:,4:6))/131.072;         % rad/s
dt = 0.005;
N = size(data,1)-1;

% Initialize
pitch_comp = zeros(N, 1);
roll_comp = zeros(N, 1);
roll_madg = data2(:,1);
pitch_madg = data2(:,2);
yaw_madg = data2(:,3);

alpha = 0.98;

% Initialize
q = [1, 0, 0, 0];  % Initial quaternion
beta = 0.04;

% Initial estimate from accelerometer
pitch_comp(1) = atan2(acc(1,2), sqrt(acc(1,1)^2 + acc(1,3)^2));
roll_comp(1) = atan2(-acc(1,1), acc(1,3));

for i = 2:N
    acc_pitch = atan2(acc(i,2), sqrt(acc(i,1)^2 + acc(i,3)^2));
    acc_roll  = atan2(-acc(i,1), acc(i,3));

    gyro_pitch = pitch_comp(i-1) + gyro(i,2) * dt;
    gyro_roll  = roll_comp(i-1)  + gyro(i,1) * dt;

    pitch_comp(i) = alpha * gyro_pitch + (1 - alpha) * acc_pitch;
    roll_comp(i)  = alpha * gyro_roll  + (1 - alpha) * acc_roll;
end

t = (0:N-1) * dt;

figure;
subplot(2,1,1);
plot(t, rad2deg(pitch_comp), 'b-', t, rad2deg(roll_madg), 'r');
legend('Complementary', 'Madgwick');
xlabel('Time (s)'); ylabel('Pitch (deg)');
title('Pitch Comparison');

subplot(2,1,2);
plot(t, rad2deg(roll_comp), 'b-', t, rad2deg(pitch_madg), 'r');
legend('Complementary', 'Madgwick');
xlabel('Time (s)'); ylabel('Roll (deg)');
title('Roll Comparison')

%% Animate orientation
viewerComp = HelperOrientationViewer('Title', 'Complementary Filter');
viewerComp.Title = 'Complementary Filter';
viewerMadg = HelperOrientationViewer('Title', 'Madgwick Filter');
viewerMadg.Title = 'Madgwick Filter';

for i = 1:10:length(t)
    % --- Complementary quaternion ---
    qComp = quaternion([0, pitch_comp(i), -roll_comp(i)], 'euler', 'ZYX', 'frame');
    viewerComp(qComp);

    % --- Madgwick quaternion ---
    qMadg = quaternion([yaw_madg(i), roll_madg(i), -pitch_madg(i)], 'euler', 'ZYX', 'frame');
    viewerMadg(qMadg);

    drawnow limitrate;
    pause(0.01);
end
