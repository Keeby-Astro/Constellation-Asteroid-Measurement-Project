function createfigure(XData1, YData1, YData2, YData3, Parent1)
% CREATEFIGURE Visualizes IMU rotation data and controller output.
% Inputs:
%   XData1: Time data for the x-axis
%   YData1: Gain data for the first quaternion (q1)
%   YData2: Gain data for the second quaternion (q2)
%   YData3: Gain data for the third quaternion (q3)
%   Parent1: Parent object for text annotation (e.g., figure or axes)

% Validate inputs
if nargin < 5
    error('All five inputs (XData1, YData1, YData2, YData3, Parent1) are required.');
end

% Ensure data vectors are the same size
if ~isequal(length(XData1), length(YData1), length(YData2), length(YData3))
    error('XData1, YData1, YData2, and YData3 must have the same length.');
end

% Figure Setup
figure('WindowState', 'maximized', 'Tag', 'ScopePrintToFigure', ...
       'Color', [1 1 1]);

% Axes Setup
axes1 = axes('Tag', 'DisplayAxes1_RealMag');
hold(axes1, 'on');
colororder(axes1, [0 0.447 0.741; 0.851 0.325 0.098; 0.929 0.694 0.125]);
grid(axes1, 'on');

% Plot Data
line(XData1, YData1, 'DisplayName', 'Gain q1', 'LineWidth', 3, 'Color', [1 1 0.1]);
line(XData1, YData2, 'DisplayName', 'Gain q2', 'LineWidth', 3, 'Color', [0.075 0.624 1]);
line(XData1, YData3, 'DisplayName', 'Gain q3', 'LineWidth', 3, 'Color', [1 0.412 0.161]);

% Labels, Title, and Legend
xlabel('Time (seconds)', 'FontName', 'Georgia', 'FontSize', 18);
ylabel('Gain', 'FontName', 'Georgia', 'FontSize', 18);
title('Controller Output', 'FontSize', 20, 'Interpreter', 'none');
legend('show', 'FontSize', 14, 'EdgeColor', [0 0 0]);

% Axis Limits (Optional: Uncomment to set explicit limits)
% xlim([0 max(XData1)]);
% ylim([min([YData1 YData2 YData3]) max([YData1 YData2 YData3])]);

% Text Annotation (e.g., display additional info)
if isvalid(Parent1)
    text('Parent', Parent1, 'Units', 'pixels', 'VerticalAlignment', 'bottom', ...
         'FontSize', 8, 'Position', [10 10 0], 'String', 'Time Offset: 0', ...
         'Visible', 'on', 'Tag', 'TimeOffsetStatus');
else
    warning('Parent1 is invalid. Skipping text annotation.');
end

hold(axes1, 'off');
end