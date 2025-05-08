
data = readmatrix("data2_log_all.csv");
T = readtable("data2_log_all.csv", 'TextType', 'string');

timestamps = datetime(T{:,1}, 'InputFormat', 'yyyy-MM-dd''T''HH:mm:ss.SSSSSS');

rate = data(:, 2);  % Assuming the second column is the rate

% Plotting rate over time
figure;
scatter(timestamps, rate, 10, 'filled');  % 10 is marker size, adjust as needed
xlabel('Time (s)');
ylabel('Rate');
title('Sensor rate over time');
grid on;
