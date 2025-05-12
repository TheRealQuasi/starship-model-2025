% --- Serial Setup ---
clear s;
port = "/dev/tty.usbmodem135745001";  % Adjust as needed
baud = 115200;
s = serialport(port, baud);
configureTerminator(s, "LF");
flush(s);


%% Animate orientation
viewerComp = HelperOrientationViewer('Title', 'Complementary Filter');
viewerComp.Title = 'Complementary Filter';
viewerMadg = HelperOrientationViewer('Title', 'Madgwick Filter');
viewerMadg.Title = 'Madgwick Filter';

% --- Real-Time Loop ---
while true
    try
        line = readline(s);
        values = str2double(split(strtrim(line), ","));
        if numel(values) ~= 5 || any(isnan(values))
            %disp(values);
            continue; % Skip malformed line
        end

        disp(values);
        
        pitch_madg = values(1);
        roll_madg  = values(2);
        yaw_madg = values(3);
        pitch_comp = values(4);
        roll_comp = values(5);

        % --- Complementary quaternion ---
        qComp = quaternion([0, pitch_comp, -roll_comp], 'euler', 'ZYX', 'frame');
        viewerComp(qComp);
    
        % --- Madgwick quaternion ---
        qMadg = quaternion([yaw_madg, pitch_madg, -roll_madg], 'euler', 'ZYX', 'frame');
        viewerMadg(qMadg);
    
        drawnow limitrate nocallbacks;
        %pause(0.001);

    catch ME
        warning("Plotting error: %s", '%s', ME.message);
    end
end



