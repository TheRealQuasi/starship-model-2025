clear; clc;

%% Load Data (Assuming CSV with columns: time, accX, accY, accZ, gyroX, gyroY, gyroZ)
data = readmatrix('all_sensor_shake_data.csv');

vx = data(:,7);
acc = data(:,1:3);
gyro = deg2rad(data(:,4:6)/131.072);

dt = 0.01;  % Assuming uniform sampling
time = (0:length(vx)-1)*dt;

%% Constants and Initialization
g = 9.81;  % Gravity (m/s^2)
alpha = 0.98;  % Complementary filter weight
num_samples = length(time);

% Initialize orientation angles (roll, pitch)
roll = zeros(num_samples,1);
pitch = zeros(num_samples,1);
pitch_raw = zeros(num_samples,1);
roll_raw = zeros(num_samples,1);

for i = 2:num_samples

    theta_acc = atan2(acc(i,2), sqrt(acc(i,1).^2 + acc(i,3).^2));  % Pitch from accelerometer

    pitch_raw(i) = theta_acc;

    phi_acc = atan2(-acc(i,1), acc(i,3));  % Roll from accelerometer

    roll_raw(i) = phi_acc;

    % Integrate gyro rates
    roll_gyro = roll(i-1) + gyro(i,1) * dt;
    pitch_gyro = pitch(i-1) + gyro(i,2) * dt;
    
    % Complementary filter
    roll(i) = alpha * roll_gyro + (1 - alpha) * phi_acc;
    pitch(i) = alpha * pitch_gyro + (1 - alpha) * theta_acc;
end

plot(time, rad2deg(pitch_raw), 'r--'); hold on;
plot(time, rad2deg(pitch), 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Pitch (degrees)');
legend('Pitch Accelerometer', 'Filtered (Complementary)');
title('Pitch Estimation using Complementary Filter');
grid on;

%% Initialize orientation viewer
%viewer = HelperOrientationViewer;
%viewer.Title = 'Drone Orientation (Pitch & Roll)';

%% Animate orientation
%for i = 1:10:length(time)  % You can adjust the step size (e.g., 1 or 5)
    % Create quaternion from Euler angles (roll, pitch, yaw)
    % We only use roll & pitch for now; yaw is 0 as it’s not updated in this example
    %fused = quaternion([0, rad2deg(pitch(i)), -rad2deg(roll(i))], 'eulerd', 'ZYX', 'frame');

    % Update the viewer with the quaternion
    %viewer(fused);  % Update the viewer with the new orientation

    % Force the viewer to update and render
    %drawnow;

    % Pause to slow down the animation (adjust the value to control speed)
    %pause(0.00001);
%end
