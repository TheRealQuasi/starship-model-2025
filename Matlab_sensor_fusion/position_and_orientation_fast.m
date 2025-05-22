% Load sensor data
data = readmatrix("all_sensor_shake_data.csv");
dt = 0.0025;

% Extract and convert units
acc = data(:,1:3) / 5460 * 9.81;     % m/s²
gyro = deg2rad(data(:,4:6));         % rad/s
flow_x = data(:,7);                  % Optical flow in X (pixels)
flow_y = data(:,8);                  % Optical flow in Y (pixels)
lidar_z = data(:,9)/100;             % Lidar in meters

% Optical flow scaling
flow_scale = 0.001;  % meters/pixel (adjust to your setup)

N = length(lidar_z);
t = (0:N-1) * dt;
g = 9.81;

% Filter constants
alpha = 0.98;
beta = 0.5;       % IMU vs lidar
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

function velocity = flowToVelocity(flow_px_per_frame, height_m, fps, scale_per_meter)
    if nargin < 3
        fps = 121.0;
    end
    if nargin < 4
        scale_per_meter = 21.9; % pixels per meter at 1 meter height
    end

    % Convert pixel displacement to meters/second
    scale = scale_per_meter / height_m;
    velocity = (flow_px_per_frame / scale) * fps;
end


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
    z_pos(i) = lidar_z(i);
    if z_pos(i) < 0
        z_pos(i) = 0;
    end

    % --- X/Y velocity ---
    x_vel_imu = x_vel(i-1) + acc_filtered(1) * dt;
    y_vel_imu = y_vel(i-1) + acc_filtered(2) * dt;

    % Optical flow velocity (m/s)
    v_of_x = flowToVelocity(flow_x(i), lidar_z(i));
    v_of_y = flowToVelocity(flow_y(i), lidar_z(i));
    v_of_x = v_of_x * flow_scale / dt; % flow_x * flow_scale
    v_of_y = v_of_y * flow_scale / dt; % * flow_y * flow_scale

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


% --- Orientation viewer ---
%viewer = HelperOrientationViewer('Title', 'Drone Orientation and Position');

% --- Setup figure and viewer ---
figure('Name', 'Drone Orientation and Position', 'NumberTitle', 'off');
axis equal;
grid on;
view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');
%xlim([-0.1, 0.6]);
%ylim([-0.1, 0.6]);
%zlim([-0.1, 0.5]);
axis auto;
hold on;

% Initialize empty handles
% Initial rotation matrix
R0 = eul2rotm([0, -pitch(1), roll(1)], 'ZYX');

origin = [x_pos(1), y_pos(1), z_pos(1)];
L = 0.4;

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
    R = eul2rotm([0, -pitch(i), roll(i)], 'ZYX');
    origin = [x_pos(i), y_pos(i), z_pos(i)];

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


