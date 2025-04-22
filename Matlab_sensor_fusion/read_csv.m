data = readmatrix('structured_sensor_data.csv');   % Read entire file

dt = 0.01;  % 1/100 Hz = 0.01 seconds
time = (0:size(data,1)-1)' * dt;
acc  = data(:,[1,10,11]);
gyro = data(:,2:4);
lidar = data(:,8);
vx_of = data(:,6);
vy_of = data(:,7);
