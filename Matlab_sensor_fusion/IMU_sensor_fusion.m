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

%% Plot Cube in 3D Space
figure;
axis([-1 1 -1 1 -1 1]);
grid on;
hold on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('IMU Cube Orientation');

% Define cube vertices
cube_vertices = [-0.5 -0.5 -0.5; 0.5 -0.5 -0.5; 0.5 0.5 -0.5; -0.5 0.5 -0.5;
                 -0.5 -0.5  0.5; 0.5 -0.5  0.5; 0.5 0.5  0.5; -0.5 0.5  0.5];
cube_faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];

cube_patch = patch('Vertices', cube_vertices, 'Faces', cube_faces, 'FaceColor', 'cyan');


view(3);

for i = 1:num_samples
    % Create rotation matrix
    Rx = [1 0 0; 0 cos(roll(i)) -sin(roll(i)); 0 sin(roll(i)) cos(roll(i))];
    Ry = [cos(pitch(i)) 0 sin(pitch(i)); 0 1 0; -sin(pitch(i)) 0 cos(pitch(i))];
    R = Ry * Rx;
    
    % Apply rotation
    rotated_vertices = (R * cube_vertices')';
    
    % Update cube position
    set(cube_patch, 'Vertices', rotated_vertices);
    drawnow;

    pause(0.1);  % <-- Delay of 0.1 seconds between plot updates
end
