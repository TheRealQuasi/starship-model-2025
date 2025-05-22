% --- Serial Setup ---
clear s;
port = "/dev/tty.usbmodem135745001";  % Adjust as needed
baud = 115200;
s = serialport(port, baud);
configureTerminator(s, "LF");
flush(s);

% --- Setup figure and viewer ---
% Add this before creating your figure
set(0, 'DefaultFigureWindowStyle', 'normal');

figure('Name', 'Drone Orientation and Position (Real-Time)', 'NumberTitle', 'off');
axis equal;
grid on;
view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');
%xlim([-0.1, 0.6]);
%ylim([-0.1, 0.6]);
%zlim([-0.1, 0.5]);
axis auto;
hold on;

% Length of orientation axes
L = 0.4;

% Initial read to get a valid line
valid = false;
while ~valid
    line = readline(s);
    values = str2double(split(strtrim(line), ","));
    if numel(values) == 5 && all(~isnan(values))
        valid = true;
    end
end

% Initialize pose
pitch = values(1);
roll  = values(2);
x     = values(3);
y     = values(4);
z     = values(5);

% Initial rotation
R = eul2rotm([0, -pitch, roll], 'ZYX');
origin = [x, y, z];

% Initialize quivers and path
hX = quiver3(origin(1), origin(2), origin(3), R(1,1)*L, R(2,1)*L, R(3,1)*L, 'r', 'LineWidth', 2);
hY = quiver3(origin(1), origin(2), origin(3), R(1,2)*L, R(2,2)*L, R(3,2)*L, 'g', 'LineWidth', 2);
hZ = quiver3(origin(1), origin(2), origin(3), R(1,3)*L, R(2,3)*L, R(3,3)*L, 'b', 'LineWidth', 2);
hPath = plot3(NaN, NaN, NaN, '.k', 'MarkerSize', 2);

% Path data
pathX = []; pathY = []; pathZ = [];

% --- Real-Time Loop ---
while true
    try
        line = readline(s);
        values = str2double(split(strtrim(line), ","));
        if numel(values) ~= 5 || any(isnan(values))
            continue; % Skip malformed line
        end

        %disp(values);
        
        pitch = values(1);
        roll  = values(2);
        x     = values(3);
        y     = values(4);
        z     = values(5);

        if any(isnan([pitch, roll, x, y, z]))
            continue;  % Skip invalid data
        end

        if all(R(:) == 0) || norm(R(:,1)) == 0
            warning('Rotation matrix is zero or invalid at time %.2f', t(i));
            continue;
        end

        R = eul2rotm([0, -pitch, roll], 'ZYX');
        origin = [x, y, z];

        % Update quiver arrows
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

        drawnow limitrate nocallbacks;
        %pause(0.01);

    catch ME
        warning("Plotting error: %s", '%s', ME.message);
    end
end

