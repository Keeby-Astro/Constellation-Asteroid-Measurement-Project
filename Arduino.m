clc; close all; clear;

% Open serial port with the correct baud rate
s = serialport("COM3", 115200);

% Create arrays to store data
data_w = [];
data_x = [];
data_y = [];
data_z = [];
timestamps = [];

% Create a single figure for all subplots
figure;
ax_w = subplot(2, 2, 1);
hLine_w = animatedline(ax_w);
title('Quaternion w-component');
ylabel('w-value');

ax_x = subplot(2, 2, 2);
hLine_x = animatedline(ax_x);
title('Quaternion x-component');
ylabel('x-value');

ax_y = subplot(2, 2, 3);
hLine_y = animatedline(ax_y);
title('Quaternion y-component');
ylabel('y-value');

ax_z = subplot(2, 2, 4);
hLine_z = animatedline(ax_z);
title('Quaternion z-component');
ylabel('z-value');

% Initialize a counter
i = 1;

% Read and plot live data
while true % loop until the figure is closed
    % Read data from serial port
    line = readline(s);
    if line == ""
        continue;
    end
    
    % Parse data
    parsedData = str2double(strsplit(line, ','));
    
    % Check for invalid data
    if any(isnan(parsedData))
        fprintf('Invalid data received: %s\n', line);
        continue;
    end

    % Record timestamp
    timestamps(i) = i - 1;
    
    % Append new data points to arrays
    addpoints(hLine_w, timestamps(i), parsedData(1));
    addpoints(hLine_x, timestamps(i), parsedData(2));
    addpoints(hLine_y, timestamps(i), parsedData(3));
    addpoints(hLine_z, timestamps(i), parsedData(4));
    
    drawnow;
    
    % Increment counter
    i = i + 1;
end

% Close serial port
fclose(s);
delete(s);