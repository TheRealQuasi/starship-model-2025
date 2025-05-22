% Load sensor data
data = readmatrix("all_sensor_shake_data.csv");
dt = 0.0025;

% Preprocessing
acc = data(:,1:3) / 5460 * 9.81;
gyro = deg2rad(data(:,4:6));
flow_x = data(:,7);
flow_y = data(:,8);
lidar_z = data(:,9) / 100.0;
flow_scale = 0.001;

N = length(lidar_z);
t = (0:N-1) * dt;
g = 9.81;

alpha = 0.98;
alpha_acc = 0.5;

%% --- Beta Test (Vary beta, fix gamma) ---
beta_values = [0.05, 0.8];
gamma_fixed = 0.8;
colors = lines(length(beta_values));

figure;
hold on;
for b = 1:length(beta_values)
    beta = beta_values(b);

    % Reset state
    roll = zeros(N,1); pitch = zeros(N,1);
    z_pos = zeros(N,1); z_vel = zeros(N,1);
    x_vel = zeros(N,1); y_vel = zeros(N,1);
    x_pos = zeros(N,1); y_pos = zeros(N,1);
    acc_filtered = acc(1,:);

    pitch(1) = atan2(acc(1,2), sqrt(acc(1,1)^2 + acc(1,3)^2));
    roll(1)  = atan2(-acc(1,1), acc(1,3));

    for i = 2:N
        acc_roll  = atan2(-acc(i,1), acc(i,3));
        acc_pitch = atan2(acc(i,2), sqrt(acc(i,1)^2 + acc(i,3)^2));

        roll_gyro  = roll(i-1) + gyro(i,1) * dt;
        pitch_gyro = pitch(i-1) + gyro(i,2) * dt;

        roll(i)  = alpha * roll_gyro  + (1 - alpha) * acc_roll;
        pitch(i) = alpha * pitch_gyro + (1 - alpha) * acc_pitch;

        R = eul2rotm([0, -pitch(i), roll(i)], 'ZYX');
        g_body = R * [0; 0; g];
        acc_nogravity = acc(i,:) - g_body';
        acc_filtered = alpha_acc * acc_filtered + (1 - alpha_acc) * acc_nogravity;

        v_acc_z = z_vel(i-1) + acc_filtered(3) * dt;
        v_lidar_z = (lidar_z(i) - lidar_z(i-1)) / dt;
        z_vel(i) = beta * v_acc_z + (1 - beta) * v_lidar_z;
        z_pos(i) = z_pos(i-1) + z_vel(i) * dt;
        z_pos(i) = max(z_pos(i), 0);

        x_vel_imu = x_vel(i-1) + acc_filtered(1) * dt;
        y_vel_imu = y_vel(i-1) + acc_filtered(2) * dt;
        v_of_x = flow_x(i) * flow_scale / dt;
        v_of_y = flow_y(i) * flow_scale / dt;
        x_vel(i) = gamma_fixed * x_vel_imu + (1 - gamma_fixed) * v_of_x;
        y_vel(i) = gamma_fixed * y_vel_imu + (1 - gamma_fixed) * v_of_y;
        x_pos(i) = x_pos(i-1) + x_vel(i) * dt;
        y_pos(i) = y_pos(i-1) + y_vel(i) * dt;
    end

    plot3(x_pos, y_pos, z_pos, 'Color', colors(b,:), 'LineWidth', 1.5);
    legend_entries_beta{b} = sprintf('\\beta = %.1f', beta);
end
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Trajectory Comparison for Varying \beta');
legend(legend_entries_beta, 'Location', 'bestoutside');
grid on;
axis tight; 
view(3);

%% --- Gamma Test (Vary gamma, fix beta) ---
gamma_values = [0.2, 0.8];
beta_fixed = 0.05;
colors = lines(length(gamma_values));

figure;
hold on;
for g = 1:length(gamma_values)
    gamma = gamma_values(g);

    % Reset state
    roll = zeros(N,1); pitch = zeros(N,1);
    z_pos = zeros(N,1); z_vel = zeros(N,1);
    x_vel = zeros(N,1); y_vel = zeros(N,1);
    x_pos = zeros(N,1); y_pos = zeros(N,1);
    acc_filtered = acc(1,:);

    pitch(1) = atan2(acc(1,2), sqrt(acc(1,1)^2 + acc(1,3)^2));
    roll(1)  = atan2(-acc(1,1), acc(1,3));

    for i = 2:N
        acc_roll  = atan2(-acc(i,1), acc(i,3));
        acc_pitch = atan2(acc(i,2), sqrt(acc(i,1)^2 + acc(i,3)^2));

        roll_gyro  = roll(i-1) + gyro(i,1) * dt;
        pitch_gyro = pitch(i-1) + gyro(i,2) * dt;

        roll(i)  = alpha * roll_gyro  + (1 - alpha) * acc_roll;
        pitch(i) = alpha * pitch_gyro + (1 - alpha) * acc_pitch;

        R = eul2rotm([0, -pitch(i), roll(i)], 'ZYX');
        g_body = R * [0; 0; g];
        acc_nogravity = acc(i,:) - g_body';
        acc_filtered = alpha_acc * acc_filtered + (1 - alpha_acc) * acc_nogravity;

        v_acc_z = z_vel(i-1) + acc_filtered(3) * dt;
        v_lidar_z = (lidar_z(i) - lidar_z(i-1)) / dt;
        z_vel(i) = beta_fixed * v_acc_z + (1 - beta_fixed) * v_lidar_z;
        z_pos(i) = z_pos(i-1) + z_vel(i) * dt;
        z_pos(i) = max(z_pos(i), 0);

        x_vel_imu = x_vel(i-1) + acc_filtered(1) * dt;
        y_vel_imu = y_vel(i-1) + acc_filtered(2) * dt;
        v_of_x = flow_x(i) * flow_scale / dt;
        v_of_y = flow_y(i) * flow_scale / dt;
        x_vel(i) = gamma * x_vel_imu + (1 - gamma) * v_of_x;
        y_vel(i) = gamma * y_vel_imu + (1 - gamma) * v_of_y;
        x_pos(i) = x_pos(i-1) + x_vel(i) * dt;
        y_pos(i) = y_pos(i-1) + y_vel(i) * dt;
    end

    plot3(x_pos, y_pos, z_pos, 'Color', colors(g,:), 'LineWidth', 1.5);
    legend_entries_gamma{g} = sprintf('\\gamma = %.1f', gamma);
end
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Trajectory Comparison for Varying \gamma');
legend(legend_entries_gamma, 'Location', 'bestoutside');
grid on; 
axis tight; 
view(3);
