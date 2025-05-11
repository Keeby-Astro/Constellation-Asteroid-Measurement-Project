% Quaternion Visualizer Selector

% This script creates a simple GUI to select between two different quaternion visualizers.
%   The user can select between Visualizer 1 and Visualizer 2 by clicking the corresponding button.
%   The selected visualizer will then be launched. This script is intended to be used as a starting point
%   for creating a more complex GUI that allows the user to select between multiple visualizers or other tools.

function switch_visualizer()
    % Close any existing figures to prevent overlap and confusion
    close all;

    % Create the main GUI figure
    mainFig = figure('Name', 'Quaternion Visualizer Selector', 'Position', [100, 100, 300, 200]);

    % Button for Visualizer 1
    uicontrol('Style', 'pushbutton', 'String', 'Visualizer 1', ...
              'Position', [50 120 200 50], 'Callback', @startVisualizer1);

    % Button for Visualizer 2
    uicontrol('Style', 'pushbutton', 'String', 'Visualizer 2', ...
              'Position', [50 50 200 50], 'Callback', @startVisualizer2);

    % Function to start Visualizer 1
    function startVisualizer1(src, event)
        fprintf('Starting Visualizer 1\n');
        quaternion_rotation_visualizer1(); % Assumes this function exists
    end

    % Function to start Visualizer 2
    function startVisualizer2(src, event)
        fprintf('Starting Visualizer 2\n');
        quaternion_rotation_visualizer2(); % Assumes this function exists
    end
end