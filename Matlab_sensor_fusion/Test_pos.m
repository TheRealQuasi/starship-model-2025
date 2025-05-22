% --- CSV-based Drone Orientation and Position Plot ---
clear; clc;

% Load data from CSV
data = readmatrix('circle_test.csv');  % Format: [pitch, roll, x, y, z]

% Check data shape
if size(data, 2) < 5
    error('CSV must have at least 5 columns: pitch, roll, x, y, z');
end

pitch = data(:,2);  % radians
roll  = data(:,3);  % radians
x     = data(:,4)/10.0;
y     = data(:,5)/10.0;
z     = data(:,6);
N     = size(data,1);

% --- Setup Figure ---
set(0, 'DefaultFigureWindowStyle', 'normal');
figure('Name', 'Drone Orientation and Position (From CSV)', 'NumberTitle', 'off');
axis equal;
grid on;
view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');
axis auto;
hold on;

% Length of orientation axes
L = 0.4;

% Initial Rotation
R = eul2rotm([0, -pitch(1), roll(1)], 'ZYX');
origin = [x(1), y(1), z(1)];

% Initialize arrows
hX = quiver3(origin(1), origin(2), origin(3), R(1,1)*L, R(2,1)*L, R(3,1)*L, 'r', 'LineWidth', 2);
hY = quiver3(origin(1), origin(2), origin(3), R(1,2)*L, R(2,2)*L, R(3,2)*L, 'g', 'LineWidth', 2);
hZ = quiver3(origin(1), origin(2), origin(3), R(1,3)*L, R(2,3)*L, R(3,3)*L, 'b', 'LineWidth', 2);
hPath = plot3(NaN, NaN, NaN, '.k', 'MarkerSize', 2);

% Store path
pathX = []; pathY = []; pathZ = [];

% --- Animate ---
for i = 1:5:N
    if any(isnan([pitch(i), roll(i), x(i), y(i), z(i)]))
        continue;
    end

    R = eul2rotm([0, -pitch(i), roll(i)], 'ZYX');
    origin = [x(i), y(i), z(i)];

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
