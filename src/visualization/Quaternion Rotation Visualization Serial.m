% Quaternion Rotation Visualizer with Real-Time Serial Data

% Function to visualize the rotation of a set of axes using quaternion rotation
function quaternion_rotation_visualizer()
    % Create the visualization figure and axes
    fig = figure('Name', 'Quaternion Rotation Visualizer', 'Units', 'normalized', 'Position', [0 0 1 0.9], 'Color', [0.1 0.1 0.1]);
    ax = axes('Parent', fig, 'DataAspectRatio', [1 1 1], 'Box', 'off', 'Color', [0 0 0]);
    hold(ax, 'on');
    grid(ax, 'on');
    xlabel(ax, 'X');
    ylabel(ax, 'Y');
    zlabel(ax, 'Z');
    sgtitle('Quaternion Rotation Visualizer', 'Color', [1 1 1]);
    view(3);
    axis(ax, 'vis3d');
    set(ax, 'Visible', 'off');

    % Plot the unit sphere for reference
    [X, Y, Z] = sphere;
    surf(ax, X, Y, Z, 'FaceAlpha', 0.2, 'EdgeColor', 'none');

    % Plot initial axes
    rotated_axes_i = quiver3(0, 0, 0, 1, 0, 0, 'r', 'LineWidth', 2, 'AutoScale', 'off');
    rotated_axes_j = quiver3(0, 0, 0, 0, 1, 0, 'g', 'LineWidth', 2, 'AutoScale', 'off');
    rotated_axes_k = quiver3(0, 0, 0, 0, 0, 1, 'b', 'LineWidth', 2, 'AutoScale', 'off');

    % Setup serial port
    s = serialport("COM3", 115200);
    configureTerminator(s, 'LF');
    flush(s);

    % Read and process data from serial port
    while isvalid(fig)  % Continue until the figure is closed
        line = readline(s);
        if line == ""
            continue;
        end
        
        % Parse the received data as quaternion values
        parsedData = str2double(strsplit(line, ','));
        if length(parsedData) >= 4 && ~any(isnan(parsedData))
            quaternion = parsedData(1:4) / norm(parsedData(1:4));
            guidata(fig, quaternion);
            updateAxes(quaternion);
            drawnow;
        else
            fprintf('Invalid data received: %s\n', line);
        end
    end

    % Close serial port on exit
    delete(s);
    clear s;

    % Function to update the rotated axes based on the quaternion
    function updateAxes(quaternion)
        i_rot = quatrotate(quaternion, [1 0 0]); % Rotate the initial i-axis
        j_rot = quatrotate(quaternion, [0 1 0]); % Rotate the initial j-axis
        k_rot = quatrotate(quaternion, [0 0 1]); % Rotate the initial k-axis
        set(rotated_axes_i, 'UData', i_rot(1), 'VData', i_rot(2), 'WData', i_rot(3)); % Update the rotated i-axis
        set(rotated_axes_j, 'UData', j_rot(1), 'VData', j_rot(2), 'WData', j_rot(3)); % Update the rotated j-axis
        set(rotated_axes_k, 'UData', k_rot(1), 'VData', k_rot(2), 'WData', k_rot(3)); % Update the rotated k-axis
    end
end