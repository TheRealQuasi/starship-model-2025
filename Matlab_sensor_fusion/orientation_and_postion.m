% Load sensor data
data = readmatrix("all_sensor_shake_data.csv");
dt = 0.0025;

% Extract and convert units
acc = data(:,1:3) / 5460 * 9.81;     % m/s²
gyro = deg2rad(data(:,4:6));         % rad/s
flow_x = data(:,7);                  % Optical flow in X (pixels)
flow_y = data(:,8);                  % Optical flow in Y (pixels)
lidar_z = data(:,9);                 % Lidar in meters

% Optical flow scaling
flow_scale = 0.001;  % meters/pixel (adjust to your setup)

N = length(lidar_z);
t = (0:N-1) * dt;
g = 9.81;

% Filter constants
alpha = 0.98;
beta = 0.9;       % IMU vs lidar
gamma = 0.8;      % IMU vs optical flow
alpha_acc = 0.5;

% State variables
roll = zeros(N,1);
pitch = zeros(N,1);
z_pos = zeros(N,1);
z_vel = zeros(N,1);
x_vel = zeros(N,1);
y_vel = zeros(N,1);
x_pos = zeros(N,1);
y_pos = zeros(N,1);

% Orientation initialization
pitch(1) = atan2(acc(1,2), sqrt(acc(1,1)^2 + acc(1,3)^2));
roll(1)  = atan2(-acc(1,1), acc(1,3));

for i = 2:N
    % --- Orientation ---
    acc_roll  = atan2(-acc(i,1), acc(i,3));
    acc_pitch = atan2(acc(i,2), sqrt(acc(i,1)^2 + acc(i,3)^2));

    roll_gyro  = roll(i-1) + gyro(i,1) * dt;
    pitch_gyro = pitch(i-1) + gyro(i,2) * dt;

    roll(i)  = alpha * roll_gyro  + (1 - alpha) * acc_roll;
    pitch(i) = alpha * pitch_gyro + (1 - alpha) * acc_pitch;

    % --- Gravity removal (body -> world) ---
    R_x = [1, 0, 0;
           0, cos(roll(i)), -sin(roll(i));
           0, sin(roll(i)),  cos(roll(i))];

    R_y = [cos(pitch(i)), 0, sin(pitch(i));
           0, 1, 0;
          -sin(pitch(i)), 0, cos(pitch(i))];

    R = R_x * R_y;

    % Gravity in world coordinates
    g_world = [0; 0; g];

    g_body = R * g_world;
    
    acc_nogravity = acc(i,:) - g_body';

    % --- Low-pass filter ---
    if i == 2
        acc_filtered = acc_nogravity;
    else
        acc_filtered = alpha_acc * acc_filtered + (1 - alpha_acc) * acc_nogravity;
    end

    % --- Z velocity from lidar and acc ---
    v_acc_z = z_vel(i-1) + acc_filtered(3) * dt;
    v_lidar_z = (lidar_z(i) - lidar_z(i-1)) / dt;
    z_vel(i) = beta * v_acc_z + (1 - beta) * v_lidar_z;
    z_pos(i) = z_pos(i-1) + z_vel(i) * dt;
    if z_pos(i) < 0
        z_pos(i) = 0;
    end

    % --- X/Y velocity ---
    x_vel_imu = x_vel(i-1) + acc_filtered(1) * dt;
    y_vel_imu = y_vel(i-1) + acc_filtered(2) * dt;

    % Optical flow velocity (m/s)
    v_of_x = flow_x(i) * flow_scale / dt;
    v_of_y = flow_y(i) * flow_scale / dt;

    x_vel(i) = gamma * x_vel_imu + (1 - gamma) * v_of_x;
    y_vel(i) = gamma * y_vel_imu + (1 - gamma) * v_of_y;

    % --- Position integration ---
    x_pos(i) = x_pos(i-1) + x_vel(i) * dt;
    y_pos(i) = y_pos(i-1) + y_vel(i) * dt;
end

%figure;
%subplot(2,1,1);
%plot(t, acc(:,1), '--', t, flow_x, ':', t, x_pos, 'k-', 'LineWidth', 1.5);
%legend('IMU-only', 'Optical Flow', 'Filtered');
%xlabel('Time (s)'); ylabel('X (m)'); title('X Position');

%subplot(2,1,2);
%plot(t, acc(:,2), '--', t, flow_y, ':', t, y_pos, 'k-', 'LineWidth', 1.5);
%legend('IMU-only', 'Optical Flow', 'Filtered');
%xlabel('Time (s)'); ylabel('Y (m)'); title('Y Position');

%subplot(3,1,3);
%plot(t, acc(:,3), '--', t, lidar_z, ':', t, z_pos, 'k-', 'LineWidth', 1.5);
%legend('IMU-only', 'Lidar', 'Filtered');
%xlabel('Time (s)'); ylabel('Z (m)'); title('Z Position');


% --- Visualization ---
prev_x = NaN; prev_y = NaN; prev_z = NaN;
viewer = HelperOrientationViewer('Title', 'Drone Orientation and Position');

% --- At the top, initialize arrow handles ---
hX = []; hY = []; hZ = [];

% --- Inside the animation loop ---
for i = 1:10:N
    q = quaternion([0, pitch(i), -roll(i)], 'euler', 'ZYX', 'frame');
    viewer(q);

    % Delete old arrows if they exist
    if isgraphics(hX); delete(hX); end
    if isgraphics(hY); delete(hY); end
    if isgraphics(hZ); delete(hZ); end

    % Rotation matrix from body to world (ZYX convention)
    R = eul2rotm([0, -pitch(i), roll(i)], 'ZYX');

    % Origin of the quiver arrows (drone's position)
    origin = [x_pos(i), y_pos(i), z_pos(i)];

    % Length of the axis arrows
    L = 0.05;

    % Plot rotated body axes at current position
    hX = quiver3(origin(1), origin(2), origin(3), ...
                 R(1,1)*L, R(2,1)*L, R(3,1)*L, 'r', 'LineWidth', 2);
    hY = quiver3(origin(1), origin(2), origin(3), ...
                 R(1,2)*L, R(2,2)*L, R(3,2)*L, 'g', 'LineWidth', 2);
    hZ = quiver3(origin(1), origin(2), origin(3), ...
                 R(1,3)*L, R(2,3)*L, R(3,3)*L, 'b', 'LineWidth', 2);

    % Trace the path with points
    plot3(x_pos(i), y_pos(i), z_pos(i), 'k.', 'MarkerSize', 1);
    hold on;

    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('3D Position & Orientation');
    axis equal; grid on; view(3);
    drawnow;
end
