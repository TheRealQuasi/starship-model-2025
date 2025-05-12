
% Load sensor data
data = readmatrix("all_sensor_shake_data.csv");

% Extract and convert units
acc = data(:,1:3) / 5460 * 9.81;     % m/s²
gyro = deg2rad(data(:,4:6));         % rad/s
dt = 0.005;
N = size(data,1);

% Initialize
pitch_comp = zeros(N, 1);
roll_comp = zeros(N, 1);
roll_madg = zeros(N, 1);
pitch_madg = zeros(N, 1);
yaw_madg = zeros(N, 1);

alpha = 0.98;

% Initialize
q = [1, 0, 0, 0];  % Initial quaternion
beta = 0.04;

% Initial estimate from accelerometer
pitch_comp(1) = atan2(acc(1,2), sqrt(acc(1,1)^2 + acc(1,3)^2));
roll_comp(1) = atan2(-acc(1,1), acc(1,3));

function [q, roll, pitch, yaw] = madgwick6DOF(q, gyro, acc, beta, dt)
    % Inputs:
    % q    - previous quaternion [q0, q1, q2, q3]
    % gyro - [gx, gy, gz] in rad/s
    % acc  - [ax, ay, az] in m/s^2
    % beta - Madgwick filter gain (e.g., 0.04)
    % dt   - time step in seconds
    
    % Extract previous quaternion values
    q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
    gx = gyro(1); gy = gyro(2); gz = gyro(3);
    ax = acc(1);  ay = acc(2);  az = acc(3);
    
    % Normalise accelerometer measurement
    if norm([ax, ay, az]) == 0
        return;
    end
    normAcc = 1 / norm([ax, ay, az]);
    ax = ax * normAcc;
    ay = ay * normAcc;
    az = az * normAcc;
    
    % Auxiliary variables
    d2q0 = 2.0 * q0; d2q1 = 2.0 * q1; d2q2 = 2.0 * q2; d2q3 = 2.0 * q3;
    d4q0 = 4.0 * q0; d4q1 = 4.0 * q1; d4q2 = 4.0 * q2;
    d8q1 = 8.0 * q1; d8q2 = 8.0 * q2;
    q0q0 = q0 * q0; q1q1 = q1 * q1; q2q2 = q2 * q2; q3q3 = q3 * q3;
    
    % Gradient descent algorithm corrective step
    s0 = d4q0*q2q2 + d2q2*ax + d4q0*q1q1 - d2q1*ay;
    s1 = d4q1*q3q3 - d2q3*ax + 4.0*q0q0*q1 - d2q0*ay - d4q1 + d8q1*q1q1 + d8q1*q2q2 + d4q1*az;
    s2 = 4.0*q0q0*q2 + d2q0*ax + d4q2*q3q3 - d2q3*ay - d4q2 + d8q2*q1q1 + d8q2*q2q2 + d4q2*az;
    s3 = 4.0*q1q1*q3 - d2q1*ax + 4.0*q2q2*q3 - d2q2*ay;
    normS = 1 / norm([s0, s1, s2, s3]);
    s0 = s0 * normS; s1 = s1 * normS; s2 = s2 * normS; s3 = s3 * normS;
    
    % Rate of change of quaternion from gyroscope
    qDot1 = 0.5 * (-q1*gx - q2*gy - q3*gz);
    qDot2 = 0.5 * ( q0*gx + q2*gz - q3*gy);
    qDot3 = 0.5 * ( q0*gy - q1*gz + q3*gx);
    qDot4 = 0.5 * ( q0*gz + q1*gy - q2*gx);
    
    % Apply feedback step
    qDot1 = qDot1 - beta * s0;
    qDot2 = qDot2 - beta * s1;
    qDot3 = qDot3 - beta * s2;
    qDot4 = qDot4 - beta * s3;
    
    % Integrate to yield quaternion
    q0 = q0 + qDot1 * dt;
    q1 = q1 + qDot2 * dt;
    q2 = q2 + qDot3 * dt;
    q3 = q3 + qDot4 * dt;
    
    % Normalize quaternion
    normQ = 1 / norm([q0, q1, q2, q3]);
    q = [q0, q1, q2, q3] * normQ;
    
    % Convert to Euler angles
    roll  = atan2(q0*q1 + q2*q3, 0.5 - q1^2 - q2^2);
    pitch = -asin(max(-1, min(1, 2*(q1*q3 - q0*q2))));  % Clamped
    yaw   = -atan2(q1*q2 + q0*q3, 0.5 - q2^2 - q3^2);
end

for i = 2:N
    acc_pitch = atan2(acc(i,2), sqrt(acc(i,1)^2 + acc(i,3)^2));
    acc_roll  = atan2(-acc(i,1), acc(i,3));

    gyro_pitch = pitch_comp(i-1) + gyro(i,2) * dt;
    gyro_roll  = roll_comp(i-1)  + gyro(i,1) * dt;

    pitch_comp(i) = alpha * gyro_pitch + (1 - alpha) * acc_pitch;
    roll_comp(i)  = alpha * gyro_roll  + (1 - alpha) * acc_roll;

    [q, pitch_madg(i), roll_madg(i), yaw_madg(i)] = madgwick6DOF(q, gyro(i,:), acc(i,:), beta, dt);
end

%fuse = imufilter('SampleRate', 1/dt);
%quat = fuse(gyro, acc);
%eul = quat2eul(quat);  % [yaw, pitch, roll] in radians

%pitch_madg = eul(:,2);
%roll_madg = eul(:,3);

t = (0:N-1) * dt;

figure;
subplot(2,1,1);
plot(t, rad2deg(pitch_comp), 'b-', t, rad2deg(pitch_madg), 'r');
legend('Complementary', 'Madgwick');
xlabel('Time (s)'); ylabel('Pitch (deg)');
title('Pitch Comparison');

subplot(2,1,2);
plot(t, rad2deg(roll_comp), 'b-', t, rad2deg(roll_madg), 'r');
legend('Complementary', 'Madgwick');
xlabel('Time (s)'); ylabel('Roll (deg)');
title('Roll Comparison');

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
    qMadg = quaternion([yaw_madg(i), pitch_madg(i), -roll_madg(i)], 'euler', 'ZYX', 'frame');
    viewerMadg(qMadg);

    drawnow limitrate;
    pause(0.01);
end

