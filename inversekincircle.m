clc; clear; close all;

%% --- Robot Parameters ---
l1 = 1.0;
l2 = 0.8;

%% --- Circle Parameters ---
cx = 1.0;        % Circle center x
cy = 0.3;        % Circle center y
r  = 0.2;        % Circle radius

% Make sure the circle is reachable
% Rule: every point must satisfy: |l1-l2| < dist < l1+l2

%% --- Generate Circle Points ---
n_points = 100;                          % Number of points along circle
angles   = linspace(0, 2*pi, n_points); % Parametric angle 0 to 360

x_circle = cx + r * cos(angles);        % x coordinates of circle
y_circle = cy + r * sin(angles);        % y coordinates of circle

%% --- Solve IK for Every Point on Circle ---
theta1_traj = zeros(1, n_points);   % Store all theta1 solutions
theta2_traj = zeros(1, n_points);   % Store all theta2 solutions

% Initial guess
theta_current = [0.5; 0.5];

for i = 1:n_points
    
    xg = x_circle(i);
    yg = y_circle(i);
    
    % IK equations for this point
    ik_eq = @(theta) [ ...
        xg - l1*cos(theta(1)) - l2*cos(theta(1)+theta(2)); ...
        yg - l1*sin(theta(1)) - l2*sin(theta(1)+theta(2))  ...
    ];
    
    % Solve - use previous solution as initial guess (ensures continuity)
    options = optimoptions('fsolve', 'Display', 'off', 'TolFun', 1e-10);
    [sol, ~, exitflag] = fsolve(ik_eq, theta_current, options);
    
    if exitflag > 0
        theta1_traj(i) = sol(1);
        theta2_traj(i) = sol(2);
        theta_current  = sol;   % Use this as guess for next point (smooth path)
    else
        warning('IK failed at point %d', i);
    end
end

%% --- Animate the Robot Drawing the Circle ---
figure;
hold on; grid on; axis equal;
title('2-DOF Robot Drawing a Circle');
xlabel('X (m)'); ylabel('Y (m)');

% Set axis limits with some padding
axis([0 2 -0.5 1.5]);

% Draw the target circle in grey (reference)
plot(x_circle, y_circle, '--', 'Color', [0.7 0.7 0.7], 'LineWidth', 1.5);

% Initialize plot handles
h_link1   = plot([0 0], [0 0], 'b-o', 'LineWidth', 3, 'MarkerSize', 8, 'MarkerFaceColor', 'b');
h_link2   = plot([0 0], [0 0], 'r-o', 'LineWidth', 3, 'MarkerSize', 8, 'MarkerFaceColor', 'r');
h_trace   = plot(NaN, NaN, 'g-', 'LineWidth', 2);  % Traced path
h_tip     = plot(NaN, NaN, 'g.', 'MarkerSize', 10);

legend('Target Circle', 'Link 1', 'Link 2', 'Traced Path');

trace_x = [];
trace_y = [];

for i = 1:n_points
    
    t1 = theta1_traj(i);
    t2 = theta2_traj(i);
    
    % Joint positions
    P0 = [0, 0];
    P1 = [l1*cos(t1), l1*sin(t1)];
    P2 = [l1*cos(t1) + l2*cos(t1+t2), ...
          l1*sin(t1) + l2*sin(t1+t2)];
    
    % Update robot links
    set(h_link1, 'XData', [P0(1) P1(1)], 'YData', [P0(2) P1(2)]);
    set(h_link2, 'XData', [P1(1) P2(1)], 'YData', [P1(2) P2(2)]);
    
    % Update traced path
    trace_x(end+1) = P2(1);
    trace_y(end+1) = P2(2);
    set(h_trace, 'XData', trace_x, 'YData', trace_y);
    
    drawnow;
    pause(0.03);  % Control animation speed (lower = faster)
end

%% --- Plot Joint Angle Trajectories ---
figure;
subplot(2,1,1);
plot(rad2deg(theta1_traj), 'b-', 'LineWidth', 2);
ylabel('Theta1 (deg)'); xlabel('Point index');
title('Joint Angle Trajectories'); grid on;

subplot(2,1,2);
plot(rad2deg(theta2_traj), 'r-', 'LineWidth', 2);
ylabel('Theta2 (deg)'); xlabel('Point index');
grid on;
