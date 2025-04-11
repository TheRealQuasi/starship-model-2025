s = serialport("/dev/tty.usbmodem135745001", 115200);
configureTerminator(s, "LF");
flush(s);

N = 100; % number of samples
accX = zeros(1, N);
accY = zeros(1, N);
accZ = zeros(1, N);
gyroX = zeros(1, N);
gyroY = zeros(1, N);
gyroZ = zeros(1, N);

for i = 1:N
    line = readline(s);
    
    if startsWith(line, ">Ax:")
        accX(i) = str2double(extractAfter(line, "Ax:"));
    elseif startsWith(line, ">Ay:")
        accY(i) = str2double(extractAfter(line, "Ay:"));
    elseif startsWith(line, ">Az:")
        accZ(i) = str2double(extractAfter(line, "Az:"));
    end

    if startsWith(line, ">gyroX:")
        gyroX(i) = str2double(extractAfter(line, "gyroX:"));
    elseif startsWith(line, ">gyroY:")
        gyroY(i) = str2double(extractAfter(line, "gyroY:"));
    elseif startsWith(line, ">gyroZ:")
        gyroZ(i) = str2double(extractAfter(line, "gyroZ:"));
    end
end


%% Real-Time Data Update (using serial or simulated data)

% Simulate IMU data (Replace this with your actual data)
acc_data = [accX, accY, accZ];  % Placeholder for accelerometer data (accX, accY, accZ)
gyro_data = [gyroX, gyroY, gyroZ]; % Placeholder for gyroscope data (gyroX, gyroY, gyroZ)

% Normalize Accelerometer Data (For simplicity, assume it's in m/s^2)
acc_data = acc_data / norm(acc_data);

% Convert Gyroscope Data to Rotation (Here we'll just simulate)
% In a real application, use a proper rotation update based on gyro readings
gyro_angle = norm(gyro_data) * 0.01;  % Simulate a rotation angle based on gyro magnitude
gyro_axis = gyro_data / norm(gyro_data);  % Axis of rotation (normalize)

% Update Rotation Matrix Using Gyroscope (simple example)
R = axang2rotm([gyro_axis, gyro_angle]);  % Update rotation matrix

% Apply Rotation to Cube Vertices
rotated_vertices = (R * cube_vertices')';  % Apply the rotation

% Update Cube Position and Orientation
cube_patch.Vertices = rotated_vertices;

% Optional: You can plot accelerometer data as points (for visual reference)
plot3(acc_data(1), acc_data(2), acc_data(3), 'ro', 'MarkerFaceColor','r'); 

% Pause for a short time to simulate real-time update (e.g., 0.05 seconds)
pause(0.05);  % Adjust for your real-time rate


clear s;