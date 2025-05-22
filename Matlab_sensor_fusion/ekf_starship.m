clear; close all; clc;

%% 1. Load your sensor data (assuming columns: ax, ay, az, gx, gy, gz, vx, vy, z)
data = readmatrix('all_sensor_shake_data.csv');

ax = data(:,1)/ 5460 * 9.81;
ay = data(:,2)/ 5460 * 9.81;
az = data(:,3)/ 5460 * 9.81;
gx = deg2rad(data(:,4)); % Convert to rad/s if necessary / 131.072
gy = deg2rad(data(:,5));
gz = deg2rad(data(:,6)); % Not used here but loaded
vx_meas = data(:,7) * 0.01;
vy_meas = data(:,8) * 0.01;
z_meas = data(:,9) /100.0;

dt = 0.01; % Sample time (adjust according to your data!)

%% 2. State vector: 
% [x; vx; y; vy; z; vz; pitch; roll; pitchRate; rollRate]
n = 10; % state size

% Initial state estimate
x_est = zeros(n,1);

% Initial covariance estimate
P = eye(n)*0.1;

% State transition matrix A (from your image)
A = zeros(10,10);
A(1,2) = 1;
A(2,8) = 9.81/2; % g/2
A(3,4) = 1;
A(5,10) = 9.81/2; % g/2
A(5,6) = 1;
A(7,8) = 1;
A(9,10) = 1;

% Control matrix B
m = 2.5; % mass
Iy = 0.226;
Ix = 0.215;
h1 = 0.456;
h2 = 0.331;

B = zeros(10,4);
B(2,1) = 1/m;
B(2,2) = 1/m;
B(5,4) = 1/m;
B(8,3) = m*9.81*h1/(2*Iy);
B(10,4) = m*9.81*h2/(2*Ix);

% Measurement matrix H
% We measure: [vx; vy; pitchRate; rollRate; z]
H = zeros(5,10);
H(1,2) = 1; % vx
H(2,4) = 1; % vy
H(3,9) = 1; % pitchRate
H(4,10) = 1; % rollRate
H(5,7) = 1; % z position

%% 3. Noise matrices
Q = eye(n) * 0.1;  % Process noise covariance
R = diag([2 1 6 5 1]); % Measurement noise covariance (tune these)

%% 4. Pre-allocate
N = length(vx_meas);
x_est_hist = zeros(n,N); 

%% 5. EKF loop
for k = 1:N
    % 5.1 Prediction
    x_pred = x_est + dt*(A*x_est); % Euler integration
    P_pred = A*P*A' + Q;

    % 5.2 Measurement update
    z = [vx_meas(k); vy_meas(k); gy(k); gx(k); z_meas(k)];

    y = z - H*x_pred;              % Innovation
    S = H*P_pred*H' + R;           % Innovation covariance
    K = P_pred*H'/S;               % Kalman gain

    x_est = x_pred + K*y;          % State update
    P = (eye(n) - K*H)*P_pred;     % Covariance update

    % 5.3 Store history
    x_est_hist(:,k) = x_est;

end

%% Complementary filter
alpha = 0.98;
pitch_hist = zeros(N,1);
roll_hist = zeros(N,1);

gx = deg2rad(data(:,4)/ 131.072); % Convert to rad/s if necessary
gy = deg2rad(data(:,5)/ 131.072);
gz = deg2rad(data(:,6)/ 131.072); % Not used here but loaded

for k = 2:N
    theta_acc = atan2(ay(k), sqrt(ax(k).^2 + az(k).^2));  % Pitch from accelerometer
    phi_acc = atan2(-ax(k), az(k));  % Roll from accelerometer

    % Integrate gyro rates
    roll_gyro = roll_hist(k-1) + gx(k) * dt;
    pitch_gyro = pitch_hist(k-1) + gy(k) * dt;
    
    % Complementary filter
    roll_hist(k) = alpha * roll_gyro + (1 - alpha) * phi_acc;
    pitch_hist(k) = alpha * pitch_gyro + (1 - alpha) * theta_acc;
end

%% 6. Plot results
time = (0:N-1)*dt;

%x_est_hist = [x, vx, y, vy, ...]

figure;
subplot(5,1,1);
plot(time, vx_meas, 'r--', time, x_est_hist(2,:), 'b');
ylabel('vx [m/s]');
legend('Measured','Estimated');
grid on;

subplot(5,1,2);
plot(time, vy_meas, 'r--', time, x_est_hist(4,:), 'b');
ylabel('vy [m/s]');
legend('Measured','Estimated');
grid on;

subplot(5,1,3);
plot(time, z_meas, 'r--', time, x_est_hist(7,:), 'b');
xlabel('Time [s]');
ylabel('z [m]');
legend('Measured','Estimated');
grid on;

subplot(5,1,4);
plot(time, pitch_hist, 'r--', time, x_est_hist(10,:), 'b');
xlabel('Time [s]');
ylabel('pitch [rad/s]');
legend('Measured','Estimated');
grid on;

subplot(5,1,5);
plot(time, -roll_hist, 'r--', time, -x_est_hist(9,:), 'b');
xlabel('Time [s]');
ylabel('roll [rad/s]');
legend('Measured','Estimated');
grid on;

sgtitle('EKF State Estimation');


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
R0 = eul2rotm([0, -x_est_hist(10,1), x_est_hist(9,1)], 'ZYX');

origin = [x_est_hist(1,1), x_est_hist(3,1), x_est_hist(7,1)];
L = 0.1;

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
    R = eul2rotm([0, -x_est_hist(10,i), x_est_hist(9,i)], 'ZYX');
    origin = [x_est_hist(1,i), x_est_hist(3,i), x_est_hist(7,i)];

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
    pause(0.01);
end

%{
%% Animate orientation
viewerComp = HelperOrientationViewer('Title', 'Complementary Filter');
viewerComp.Title = 'Complementary Filter';
viewerMadg = HelperOrientationViewer('Title', 'Madgwick Filter');
viewerMadg.Title = 'Madgwick Filter';

% --- Real-Time Loop ---

for i = 1:N
    % --- Complementary quaternion ---
    qComp = quaternion([0, pitch_hist(i), -roll_hist(i)], 'euler', 'ZYX', 'frame');
    viewerComp(qComp);

    % --- Madgwick quaternion ---
    qMadg = quaternion([0, x_est_hist(9,i), -x_est_hist(10,i)], 'euler', 'ZYX', 'frame');
    viewerMadg(qMadg);

    drawnow limitrate nocallbacks;
    %pause(0.001);
end
%}

