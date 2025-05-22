clear; close all; clc;

% Load data from CSV
data = readmatrix('all_sensor_shake_data.csv');
% Extract sensor data
ax = data(:,1)/ 5460 * 9.81; ay = data(:,2)/ 5460 * 9.81; az = data(:,3)/ 5460 * 9.81;
acc = data(:,1:3) / 5460 * 9.81;     % m/s²
gyro = deg2rad(data(:,4:6));
flow_x_raw = data(:,7);  % Store raw optical flow
flow_y_raw = data(:,8);  % Store raw optical flow
lidar = data(:,9)/100.0;
% Settings
dt = 0.01; % sample time (s)
N = length(ax);
% State vector: [x; x_dot; y; y_dot; z; z_dot]
x = zeros(6, 1);
% Initial state covariance
P = eye(6) * 0.1;
% Process noise (tune these)
Q = diag([0.001 0.01 0.001 0.01 0.0001 0.01]);
% Measurement noise (for [x, y, z] position and velocity)
R = diag([0.1 0.1 0.1 0.1 1]); % Increased noise for integrated flow, and lidar
% Measurement matrix (measuring position and velocity)
H = [1 0 0 0 0 0;
     0 0 0 0 0 0; %remove pos vel
     0 0 1 0 0 0;
     0 0 0 0 0 0; %remove pos vel
     0 0 0 0 1 0];
% Store results
X_hist = zeros(6, N);
q = [1, 0, 0, 0];  % Initial quaternion
beta = 0.01;
roll = zeros(N, 1);
pitch = zeros(N, 1);
yaw = zeros(N, 1);

% Optical flow integration and scaling
flow_scale = 0.001; %  adjust based on your sensor's characteristics!
integrated_flow_x = 0;
integrated_flow_y = 0;

for k = 1:N
    [q, roll(k), pitch(k), yaw(k)] = Madgwick_function(q, gyro(k,:), acc(k,:), beta, dt);
    % --------- Predict Step ---------
    A = [1 dt 0  0 0  0;
         0  1 0  0 0  0;
         0  0 1 dt 0  0;
         0  0 0  1 0  0;
         0  0 0  0 1 dt;
         0  0 0  0 0  1];
    
    B = [0.5*dt^2 0          0;
         dt        0          0;
         0         0.5*dt^2 0;
         0         dt        0;
         0         0         0.5*dt^2;
         0         0         dt];
    u = [ax(k); ay(k); az(k)];
    x = A * x + B * u;
    P = A * P * A' + Q;
    % --------- Measurement Update ---------
    % Integrate optical flow to get displacement (velocity)
    integrated_flow_x = integrated_flow_x + flow_x_raw(k) * flow_scale * dt; % Scale and integrate
    integrated_flow_y = integrated_flow_y + flow_y_raw(k) * flow_scale * dt;
    

    % Use integrated flow as a velocity measurement
    z = [integrated_flow_x; 0; integrated_flow_y; 0 ; lidar(k)];  %  measurement vector

    H = [0 1 0 0 0 0;  % Measure x_dot (velocity)
         1 0 0 0 0 0;  % Also measure x position
         0 0 0 1 0 0;  % Measure y_dot (velocity)
         0 0 1 0 0 0;  % Also measure y position
         0 0 0 0 1 0];       % Measure z (position)

    R = diag([0.2 0.2 0.2 0.2 1]); % Increased noise for integrated flow, and lidar
    
    y = z - H * x;
    S = H * P * H' + R;
    K = P * H' / S;
    x = x + K * y;
    P = (eye(6) - K * H) * P;
    % Store results
    X_hist(:, k) = x;
end

% Extract and display results (optional)
estimated_x = X_hist(1,:);
estimated_x_dot = X_hist(2,:);
estimated_y = X_hist(3,:);
estimated_y_dot = X_hist(4,:);
estimated_z = X_hist(5,:);

%  You can plot these to visualize the results
figure;
subplot(3,1,1);
plot(estimated_x);
title('Estimated X Position');
subplot(3,1,2);
plot(estimated_y);
title('Estimated Y Position');
subplot(3,1,3);
plot(estimated_z);
title('Estimated Z Position');

figure;
subplot(2,1,1);
plot(estimated_x_dot);
title('Estimated X Velocity');
subplot(2,1,2);
plot(estimated_y_dot);
title('Estimated Y Velocity');


% --- Setup figure and viewer ---
figure('Name', 'Drone Orientation and Position', 'NumberTitle', 'off');
axis equal;
grid on;
view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');
axis auto;
%xlim([-0.1, 0.6]);
%ylim([-0.1, 0.6]);
%zlim([-0.1, 0.5]);
hold on;

% Initialize empty handles
% Initial rotation matrix
R0 = eul2rotm([yaw(1), -roll(1), pitch(1)], 'ZYX');

origin = [X_hist(1,1), X_hist(3,1), X_hist(5,1)];
L = 0.02;

hX = quiver3(origin(1), origin(2), origin(3), R0(1,1)*L, R0(2,1)*L, R0(3,1)*L, 'r', 'LineWidth', 2);
hY = quiver3(origin(1), origin(2), origin(3), R0(1,2)*L, R0(2,2)*L, R0(3,2)*L, 'g', 'LineWidth', 2);
hZ = quiver3(origin(1), origin(2), origin(3), R0(1,3)*L, R0(2,3)*L, R0(3,3)*L, 'b', 'LineWidth', 2);

hPath = plot3(NaN, NaN, NaN, '.k', 'MarkerSize', 2);

% Path arrays
pathX = []; pathY = []; pathZ = [];

for i = 1:5:N
    % --- Orientation quaternion for the viewer ---
    %q = quaternion([0, pitch(i), -roll(i)], 'euler', 'ZYX', 'frame');
    %viewer(q);

    % Rotation matrix for arrows
    R = eul2rotm([yaw(i), -roll(i), pitch(i)], 'ZYX');
    origin = [X_hist(1,i), X_hist(3,i), X_hist(5,i)];

    % Delete and recreate arrows only if handles are invalid
    if ~isgraphics(hX)
        hX = quiver3(0,0,0,0,0,0,'r','LineWidth',2);
    end
    if ~isgraphics(hY)
        hY = quiver3(0,0,0,0,0,0,'g','LineWidth',2);
    end
    if ~isgraphics(hZ)
        hZ = quiver3(0,0,0,0,0,0,'b','LineWidth',2);
    end

    % Update arrows
    set(hX, 'XData', origin(1), 'YData', origin(2), 'ZData', origin(3), ...
            'UData', R(1,1)*L, 'VData', R(2,1)*L, 'WData', R(3,1)*L);
    set(hY, 'XData', origin(1), 'YData', origin(2), 'ZData', origin(3), ...
            'UData', R(1,2)*L, 'VData', R(2,2)*L, 'WData', R(3,2)*L);
    set(hZ, 'XData', origin(1), 'YData', origin(2), 'ZData', origin(3), ...
            'UData', R(1,3)*L, 'VData', R(2,3)*L, 'WData', R(3,3)*L);

    % Update path
    pathX(end+1) = origin(1);
    pathY(end+1) = origin(2);
    pathZ(end+1) = origin(3);
    set(hPath, 'XData', pathX, 'YData', pathY, 'ZData', pathZ);

    drawnow limitrate;
    pause(0.001);
end
