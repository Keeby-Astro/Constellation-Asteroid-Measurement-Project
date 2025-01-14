% Quaternion Visualization with Real-Time Data

%{
This example demonstrates how to create a quaternion rotation visualizer in MATLAB using the `quiver3`
function to display the rotated axes based on a quaternion input. The visualization includes a unit sphere
for reference and sliders to control the quaternion components and view angles. The quaternion rotation
is updated in real-time based on the slider input.

Purpose:
    - Visualize quaternion rotation in 3D space
    - Update the rotated axes based on the quaternion input
    - Provide interactive controls to adjust the quaternion components and view angles

Input:
    - Quaternion components (q0, q1, q2, q3)

Output:
    - Real-time visualization of quaternion rotation
    - Rotated axes based on the quaternion input
    - Quaternion components display
%}

function quaternion_rotation_visualizer()
    % Create a figure and axis for visualization
    fig = figure('Name', 'Quaternion Rotation Visualizer', 'Units', 'normalized', 'Position', [0 0 1 0.9], 'Color', [0.1 0.1 0.1]);
    ax = axes('Parent', fig, 'DataAspectRatio', [1 1 1], 'Box', 'off', 'Color', [0 0 0]);
    hold(ax, 'on');
    grid(ax, 'on');
    xlabel(ax, 'X');
    ylabel(ax, 'Y');
    zlabel(ax, 'Z');
    sgtitle('                 Quaternion Rotation Visualizer', 'Color', [1 1 1])  % Set title color to white
    view(3);
    % Remove the default axes box
    axis(ax, 'vis3d');
    set(ax, 'Visible', 'off');

    % Plot unit sphere for reference
    [X, Y, Z] = sphere;
    surf(ax, X, Y, Z, 'FaceAlpha', 0.2, 'EdgeColor', 'none');

    % Plot initial axes
    quiver3(0, 0, 0, 1, 0, 0, 'k', 'LineWidth', 2, 'AutoScale', 'off');
    quiver3(0, 0, 0, 0, 1, 0, 'k', 'LineWidth', 2, 'AutoScale', 'off');
    quiver3(0, 0, 0, 0, 0, 1, 'k', 'LineWidth', 2, 'AutoScale', 'off');

    % Define handles for rotated axes
    rotated_axes_i = quiver3(0, 0, 0, 1, 0, 0, 'r', 'LineWidth', 2, 'AutoScale', 'off');
    rotated_axes_j = quiver3(0, 0, 0, 0, 1, 0, 'g', 'LineWidth', 2, 'AutoScale', 'off');
    rotated_axes_k = quiver3(0, 0, 0, 0, 0, 1, 'b', 'LineWidth', 2, 'AutoScale', 'off');

    % Initialize quaternion and store in the figure's data
    quaternion = [1, 0, 0, 0]; % Unit quaternion
    guidata(fig, quaternion);

    % Setup slider controls and labels
    [slider_q0, slider_q1, slider_q2, slider_q3, az_slider, el_slider] = setupControls();

    % Real-time Quaternion Display
    quaternion_display = uicontrol('Style', 'text', 'Position', [150 105 200 30], ...
                                   'String', sprintf('Quaternion: [1 0 0 0]'), 'ForegroundColor', [1 1 1], 'BackgroundColor', [0.5 0.5 0.5]);

    % Callback function to update rotation based on slider input
    function updateRotation(f, component, value, isReset)
        quaternion = guidata(f); % Retrieve the quaternion from the figure's data
        quaternion(component) = value;

        % If the reset flag is not set, normalize the quaternion to maintain it as a unit quaternion
        if ~isReset
            % Normalize the quaternion to maintain it as a unit quaternion
            quaternion = quaternion / norm(quaternion);
            % Update the slider positions to reflect the new quaternion values
            set(slider_q0, 'Value', quaternion(1));
            set(slider_q1, 'Value', quaternion(2));
            set(slider_q2, 'Value', quaternion(3));
            set(slider_q3, 'Value', quaternion(4));
        end

        guidata(f, quaternion); % Store the updated quaternion back into the figure's data

        % Update the display
        set(quaternion_display, 'String', sprintf('Quaternion: [%.2f %.2f %.2f %.2f]', quaternion));

        % Update the rotation of the axes using the quaternion
        updateAxes(quaternion);
    end

    % Callback function to update the view based on slider input
    function updateView()
        azimuth = get(az_slider, 'Value');
        elevation = get(el_slider, 'Value');
        view(ax, [azimuth, elevation]);
    end

    % Update the rotated axes based on the quaternion
    function updateAxes(quaternion)
        i_rot = quatrotate(quaternion, [1 0 0]);
        j_rot = quatrotate(quaternion, [0 1 0]);
        k_rot = quatrotate(quaternion, [0 0 1]);
        set(rotated_axes_i, 'UData', i_rot(1), 'VData', i_rot(2), 'WData', i_rot(3));
        set(rotated_axes_j, 'UData', j_rot(1), 'VData', j_rot(2), 'WData', j_rot(3));
        set(rotated_axes_k, 'UData', k_rot(1), 'VData', k_rot(2), 'WData', k_rot(3));
    end

    % Setup the controls for quaternion components and view angles
    function [s0, s1, s2, s3, az, el] = setupControls()
        
    % Quaternion component sliders
    % Labels for the sliders
    uicontrol('Style', 'text', 'Position', [10 72 956 15], 'String', 'Scalar q_0',     'ForegroundColor', [1 1 1], 'BackgroundColor', [0.1 0.1 0.1]);
    uicontrol('Style', 'text', 'Position', [10 52 968 15], 'String', 'Vector q_1 (i)', 'ForegroundColor', [1 1 1], 'BackgroundColor', [0.1 0.1 0.1]);
    uicontrol('Style', 'text', 'Position', [10 32 968 15], 'String', 'Vector q_2 (j)', 'ForegroundColor', [1 1 1], 'BackgroundColor', [0.1 0.1 0.1]);
    uicontrol('Style', 'text', 'Position', [10 12 972 15], 'String', 'Vector q_3 (k)', 'ForegroundColor', [1 1 1], 'BackgroundColor', [0.1 0.1 0.1]);

    % Sliders for adjusting the quaternion components
    s0 = uicontrol('Style', 'slider', 'Min', -1, 'Max', 1, 'Value', 1, ...
                   'Position', [10 70 450 20], 'BackgroundColor', [0.8 0.8 0.8], ...
                   'Callback', @(src, event) updateRotation(fig, 1, src.Value, false));
    s1 = uicontrol('Style', 'slider', 'Min', -1, 'Max', 1, 'Value', 0, ...
                   'Position', [10 50 450 20], 'BackgroundColor', [1 0.5 0.5], ...
                   'Callback', @(src, event) updateRotation(fig, 2, src.Value, false));
    s2 = uicontrol('Style', 'slider', 'Min', -1, 'Max', 1, 'Value', 0, ...
                   'Position', [10 30 450 20], 'BackgroundColor', [0.5 1 0.5], ...
                   'Callback', @(src, event) updateRotation(fig, 3, src.Value, false));
    s3 = uicontrol('Style', 'slider', 'Min', -1, 'Max', 1, 'Value', 0, ...
                   'Position', [10 10 450 20], 'BackgroundColor', [0.5 0.5 1], ...
                   'Callback', @(src, event) updateRotation(fig, 4, src.Value, false));

    % Azimuth slider
    az = uicontrol('Style', 'slider', 'Min', -180, 'Max', 180, 'Value', -45, ...
                   'Position', [600 50 300 20], 'BackgroundColor', [0.6 0.6 0.6], ...
                   'Callback', @(src, event) updateView());

    % Elevation slider
    el = uicontrol('Style', 'slider', 'Min', -90, 'Max', 90, 'Value', 30, ...
                   'Position', [600 10 300 20], 'BackgroundColor', [0.6 0.6 0.6], ...
                   'Callback', @(src, event) updateView());
    end
end