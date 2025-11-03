%% 2-Link Planar Robot Manipulator Animation
% Robot explores positive X-axis with elbow-up and elbow-down configurations

clear; close all; clc;

% Robot parameters
L1 = 35.35;  % Length of link 1
L2 = 35.35;  % Length of link 2
total_reach = L1 + L2;

% Workspace parameters
workspace_limit = 50;
robot_base = [0, 0];

% Target points along positive X-axis (every 5 units)
min_reach = L1 - L2;  % Minimum reachable distance
max_reach = L1 + L2;  % Maximum reachable distance
x_targets = 5:5:max_reach;  % Points at 5, 10, 15, ..., up to max reach
y_targets = zeros(size(x_targets));  % All on X-axis

num_points = length(x_targets);

% Initialize figure
fig = figure('Position', [100, 100, 1200, 600]);
set(fig, 'Color', 'w');

% Animation parameters
steps_per_segment = 15;  % Number of interpolation steps between points
pause_time = 0.01;  % Pause between frames for smooth animation

% Storage for previous configurations
prev_q1_up = 0;
prev_q2_up = 0;
prev_q1_down = 0;
prev_q2_down = 0;

%% Main animation loop
for pt = 1:num_points
    % Current target
    x_target = x_targets(pt);
    y_target = y_targets(pt);
    
    % Check if point is reachable
    dist = sqrt(x_target^2 + y_target^2);
    
    if dist < min_reach || dist > max_reach
        fprintf('Point (%0.1f, %0.1f) is not reachable. Skipping...\n', x_target, y_target);
        continue;
    end
    
    % Inverse Kinematics - Elbow UP configuration
    [q1_up, q2_up] = inverse_kinematics(x_target, y_target, L1, L2, 'up');
    
    % Inverse Kinematics - Elbow DOWN configuration
    [q1_down, q2_down] = inverse_kinematics(x_target, y_target, L1, L2, 'down');
    
    % Interpolate from previous to current configuration
    if pt == 1
        % First point - start from home position
        q1_up_traj = linspace(0, q1_up, steps_per_segment);
        q2_up_traj = linspace(0, q2_up, steps_per_segment);
        q1_down_traj = linspace(0, q1_down, steps_per_segment);
        q2_down_traj = linspace(0, q2_down, steps_per_segment);
    else
        % Interpolate from previous configuration
        q1_up_traj = linspace(prev_q1_up, q1_up, steps_per_segment);
        q2_up_traj = linspace(prev_q2_up, q2_up, steps_per_segment);
        q1_down_traj = linspace(prev_q1_down, q1_down, steps_per_segment);
        q2_down_traj = linspace(prev_q2_down, q2_down, steps_per_segment);
    end
    
    % Animate interpolation
    for step = 1:steps_per_segment
        clf;
        
        % Create two subplots for elbow-up and elbow-down
        % Elbow UP subplot
        subplot(1, 2, 1);
        draw_robot_config(q1_up_traj(step), q2_up_traj(step), L1, L2, ...
                         workspace_limit, x_targets, y_targets, pt, 'Elbow DOWN');
        
        % Elbow DOWN subplot
        subplot(1, 2, 2);
        draw_robot_config(q1_down_traj(step), q2_down_traj(step), L1, L2, ...
                         workspace_limit, x_targets, y_targets, pt, 'Elbow UP');
        
        drawnow;
        pause(pause_time);
    end
    
    % Hold at final position for better visualization
    pause(0.2);
    
    % Update previous configuration
    prev_q1_up = q1_up;
    prev_q2_up = q2_up;
    prev_q1_down = q1_down;
    prev_q2_down = q2_down;
end

fprintf('Animation complete!\n');

%% Function: Inverse Kinematics
function [theta1, theta2] = inverse_kinematics(x, y, L1, L2, elbow_config)
    % Calculate joint angles for a 2-link planar manipulator
    
    % Distance to target
    D = sqrt(x^2 + y^2);
    
    % Check reachability
    if D > (L1 + L2) || D < abs(L1 - L2)
        error('Target is not reachable');
    end
    
    % Calculate theta2 using law of cosines
    cos_theta2 = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2);
    cos_theta2 = max(-1, min(1, cos_theta2));  % Clamp to valid range
    
    if strcmp(elbow_config, 'up')
        theta2 = acos(cos_theta2);  % Positive angle (elbow up)
    else
        theta2 = -acos(cos_theta2);  % Negative angle (elbow down)
    end
    
    % Calculate theta1
    k1 = L1 + L2 * cos(theta2);
    k2 = L2 * sin(theta2);
    theta1 = atan2(y, x) - atan2(k2, k1);
end

%% Function: Draw Robot Configuration
function draw_robot_config(theta1, theta2, L1, L2, workspace_limit, ...
                          x_targets, y_targets, current_pt, config_name)
    % Forward kinematics
    x1 = L1 * cos(theta1);
    y1 = L1 * sin(theta1);
    x2 = x1 + L2 * cos(theta1 + theta2);
    y2 = y1 + L2 * sin(theta1 + theta2);
    
    % Draw workspace boundary
    rectangle('Position', [-workspace_limit, -workspace_limit, ...
              2*workspace_limit, 2*workspace_limit], ...
              'EdgeColor', [0.7, 0.7, 0.7], 'LineWidth', 1.5, 'LineStyle', '--');
    hold on;
    
    % Draw reachable workspace circle
    theta_circle = linspace(0, 2*pi, 100);
    max_reach = L1 + L2;
    plot(max_reach * cos(theta_circle), max_reach * sin(theta_circle), ...
         'Color', [0.9, 0.9, 0.9], 'LineWidth', 1);
    
    % Draw all target points
    plot(x_targets, y_targets, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', [0.5, 0.8, 0.5]);
    
    % Highlight current target
    if current_pt <= length(x_targets)
        plot(x_targets(current_pt), y_targets(current_pt), 'ro', ...
             'MarkerSize', 12, 'MarkerFaceColor', 'g', 'LineWidth', 2);
    end
    
    % Draw robot links
    plot([0, x1], [0, y1], 'b-', 'LineWidth', 8);  % Link 1
    plot([x1, x2], [y1, y2], 'r-', 'LineWidth', 8);  % Link 2
    
    % Draw joints
    plot(0, 0, 'ko', 'MarkerSize', 12, 'MarkerFaceColor', 'k');  % Base
    plot(x1, y1, 'ko', 'MarkerSize', 12, 'MarkerFaceColor', 'g');  % Joint 1
    plot(x2, y2, 'ko', 'MarkerSize', 12, 'MarkerFaceColor', 'r');  % End-effector
    
    % Draw positive X-axis
    plot([0, workspace_limit], [0, 0], 'k--', 'LineWidth', 1);
    
    % Settings
    axis equal;
    xlim([-workspace_limit, workspace_limit]);
    ylim([-workspace_limit, workspace_limit]);
    grid on;
    xlabel('X (units)', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Y (units)', 'FontSize', 12, 'FontWeight', 'bold');
    title(sprintf('%s Configuration', config_name), 'FontSize', 14, 'FontWeight', 'bold');
    
    % Display joint angles
    theta1_deg = rad2deg(theta1);
    theta2_deg = rad2deg(theta2);
    text(-45, 45, sprintf('θ₁ = %.1f°', theta1_deg), 'FontSize', 11, 'FontWeight', 'bold');
    text(-45, 40, sprintf('θ₂ = %.1f°', theta2_deg), 'FontSize', 11, 'FontWeight', 'bold');
    text(-45, 35, sprintf('Target: (%.1f, %.1f)', x_targets(current_pt), y_targets(current_pt)), ...
         'FontSize', 10);
    
    hold off;
end