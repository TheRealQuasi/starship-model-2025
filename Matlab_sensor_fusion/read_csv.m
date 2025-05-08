data = readtable('all_sensor_shake_data.csv');   % Read entire file

data.Properties.VariableNames = {'Ax', 'Ay', 'Az', 'Gx', 'Gy', 'Gz', 'Flow_X', 'Flow_Y', 'LiDar'};

writetable(data, 'all_sensor_shake_data.csv');