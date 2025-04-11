clear; clc;

%% Load Data (Assuming CSV with columns: time, accX, accY, accZ, gyroX, gyroY, gyroZ)
data = readmatrix('imu_data_extracted.csv');

time = data(:,1);
acc = data(:,2:4);
gyro = data(:,5:7);

%% Constants and Initialization
dt = mean(diff(time));  % Assuming uniform sampling
g = 9.81;  % Gravity (m/s^2)
alpha = 0.98;  % Complementary filter weight
num_samples = length(time);

% Convert gyro from deg/s to rad/s
gyro = deg2rad(gyro);

% Initialize orientation angles (roll, pitch)
roll = zeros(num_samples,1);
pitch = zeros(num_samples,1);

theta_acc = atan2(acc(:,2), sqrt(acc(:,1).^2 + acc(:,3).^2));  % Pitch from accelerometer
phi_acc = atan2(-acc(:,1), acc(:,3));  % Roll from accelerometer

for i = 2:num_samples
    % Integrate gyro rates
    roll_gyro = roll(i-1) + gyro(i,1) * dt;
    pitch_gyro = pitch(i-1) + gyro(i,2) * dt;
    
    % Complementary filter
    roll(i) = alpha * roll_gyro + (1 - alpha) * phi_acc(i);
    pitch(i) = alpha * pitch_gyro + (1 - alpha) * theta_acc(i);
end

%% Initialize orientation viewer
viewer = HelperOrientationViewer;
viewer.Title = 'Drone Orientation (Pitch & Roll)';
% Optionally, set frame type (e.g., 'ENU') if supported
%viewer.Frame = 'ENU';  % Uncomment if frame is supported

%% Animate orientation
for i = 1:10:length(time)  % You can adjust the step size (e.g., 1 or 5)
    % Create quaternion from Euler angles (roll, pitch, yaw)
    % We only use roll & pitch for now; yaw is 0 as it’s not updated in this example
    q = quaternion([rad2deg(roll(i)), rad2deg(pitch(i)), 0], 'eulerd', 'ZYX', 'frame');

    % Update the viewer with the quaternion
    viewer(q);  % Update the viewer with the new orientation

    % Force the viewer to update and render
    drawnow;

    % Pause to slow down the animation (adjust the value to control speed)
    pause(0.3);
end
