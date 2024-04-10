% Create serial port object
arduinoObj = serialport("COM9",500000);

% Start terminator
configureTerminator(arduinoObj,"CR/LF");

% Flush serialport
flush(arduinoObj);


arduinoObj.UserData = struct("Roll",[],"Time",1);

% Function that reads 1000 roll-values from the serial port
function readRoll(src, ~)

    % Read the ASCII data from the serialport object.
    data = readline(src);
    
    % Convert the string data to numeric type and save it in the UserData
    % property of the serialport object.
    src.UserData.Data(end+1) = str2double(data);
    
    % Update the Count value of the serialport object.
    src.UserData.Count = src.UserData.Count + 1;
    
    % If 1001 data points have been collected from the Arduino, switch off the
    % callbacks and plot the data.
    if src.UserData.Count > 1001
        configureCallback(src, "off");
        plot(src.UserData.Data(2:end));
    end
end
