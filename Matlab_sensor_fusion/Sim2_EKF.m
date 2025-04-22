% In MATLAB

data = readmatrix('structured_sensor_data.csv');   % Read entire file

dt = 0.01;  % 1/100 Hz = 0.01 seconds
time = (0:size(data,1)-1)' * dt;
acc  = data(:,[1,10,11]);
gyro = data(:,2:4);
lidar = data(:,8);
vx_of = data(:,6);
vy_of = data(:,7);

mex('EKF_wrapper.cpp', 'EKF_C.cpp');

n = length(time);

pos_history = zeros(n, 3);
ori_history = zeros(n, 2);  % Roll and Pitch

for i = 1:n-1
    dt = time(i+1) - time(i);

    % Predict step with IMU
    EKF_wrapper('predict', acc(i,:)', gyro(i,:)', dt);

    % Lidar update
    EKF_wrapper('updateLidar', lidar(i));

    % Optical flow update
    EKF_wrapper('updateOpticalFlow', vx_of(i), vy_of(i));

    % Save estimated states
    pos = EKF_wrapper('getPosition');
    ori = EKF_wrapper('getOrientation');

    pos_history(i,:) = pos';
    ori_history(i,:) = ori';
end

% Initialize the figure
figure('NumberTitle', 'off', 'Name', 'Drone Flight');
hold on;
grid on;
axis equal;
axis([-10, 10, -10, 10, -10, 10]);
xlabel('X');
ylabel('Y');
zlabel('Z');
view(3);

% Create the trajectory line
traj = plot3(NaN, NaN, NaN, 'r-', 'LineWidth', 2);

% Create a simple 3D drone marker (an arrow)
droneMarker = quiver3(0, 0, 0, 1, 0, 0, 2, 'b', 'LineWidth', 3, 'MaxHeadSize', 1);

% Initialize arrays to store trajectory
X = [];
Y = [];
Z = [];

% Loop through data
for i = 1:length(time)
    % Current position
    x = pos_history(i, 1);
    y = pos_history(i, 2);
    z = pos_history(i, 3);

    X(end+1) = x;
    Y(end+1) = y;
    Z(end+1) = z;
    
    % Update trajectory
    set(traj, 'XData', X, 'YData', Y, 'ZData', Z);

    % Get current orientation (roll, pitch)
    roll = ori_history(i, 1);
    pitch = ori_history(i, 2);
    
    % Convert to quaternion
    q = quaternion([rad2deg(roll), rad2deg(pitch), 0], 'eulerd', 'ZYX', 'frame');

    % Convert quaternion to rotation matrix
    R = rotmat(q, 'frame');

    % Update drone marker orientation and position
    dir = R * [1; 0; 0]; % Drone's "front" direction
    set(droneMarker, 'XData', x, 'YData', y, 'ZData', z, ...
                     'UData', dir(1), 'VData', dir(2), 'WData', dir(3));

    % Smooth camera following
    campos([x+5, y+5, z+5]);
    camtarget([x, y, z]);
    
    drawnow;
    pause(0.01);
end
