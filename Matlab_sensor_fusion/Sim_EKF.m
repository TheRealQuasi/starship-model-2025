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

% Create the orientation viewer object
viewer = HelperOrientationViewer;

% Initialize the previous position for the line
prev_x = NaN;
prev_y = NaN;
prev_z = NaN;

% Loop through the data to update position and orientation
for i = 1:length(time)-1
    % Get the current orientation (roll, pitch)
    roll = ori_history(i, 1);
    pitch = ori_history(i, 2);
    
    % Update the orientation viewer
    % We only use roll & pitch for now; yaw is 0 as it’s not updated in this example
    q = quaternion([rad2deg(roll), rad2deg(pitch), 0], 'eulerd', 'ZYX', 'frame');

    % Update the viewer with the quaternion
    viewer(q);  % Update the viewer with the new orientation

    % Get the current position (x, y, z)
    x = pos_history(i, 1);
    y = pos_history(i, 2);
    z = pos_history(i, 3);
    
    % If not the first iteration, plot a line between the previous position and the current position
    if ~isnan(prev_x)
        plot3([prev_x, x], [prev_y, y], [prev_z, z], 'o-', 'LineWidth', 2);  % Draw the line segment
        hold on;
        axis equal;
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
        grid on;
    end
    
    % Update the previous position for the next iteration
    prev_x = x;
    prev_y = y;
    prev_z = z;

    drawnow;
    
    % Pause to create the animation effect (100 Hz update)
    pause(0.01);
end
