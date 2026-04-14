clc; clear; close all;

% --- Robot Parameters ---
l1 = 1.0;   % Link 1 length (meters)
l2 = 0.8;   % Link 2 length (meters)

% --- Desired End-Effector Position ---
x_g = 1.2;  % Target x
y_g = 0.6;  % Target y

% --- Define the equation system (from notes) ---
% f1 = x_g - l1*cos(t1) - l2*cos(t1+t2) = 0
% f2 = y_g - l1*sin(t1) - l2*sin(t1+t2) = 0

ik_equations = @(theta) [ ...
    x_g - l1*cos(theta(1)) - l2*cos(theta(1) + theta(2)); ...
    y_g - l1*sin(theta(1)) - l2*sin(theta(1) + theta(2))  ...
];

% --- Initial Guess for [theta1, theta2] ---
theta0 = [1.0; 0.5];  % radians

% --- Solve using fsolve ---
options = optimoptions('fsolve', 'Display', 'iter', 'TolFun', 1e-10);
[theta_sol, fval, exitflag] = fsolve(ik_equations, theta0, options);

theta1 = theta_sol(1);
theta2 = theta_sol(2);

% --- Display Results ---
fprintf('\n=== IK Solution ===\n');
fprintf('theta1 = %.4f rad  (%.2f deg)\n', theta1, rad2deg(theta1));
fprintf('theta2 = %.4f rad  (%.2f deg)\n', theta2, rad2deg(theta2));
fprintf('Exit flag: %d (1 = converged)\n', exitflag);

% --- Verify using FK ---
x_check = l1*cos(theta1) + l2*cos(theta1 + theta2);
y_check = l1*sin(theta1) + l2*sin(theta1 + theta2);
fprintf('\n=== FK Verification ===\n');
fprintf('Target:    (%.4f, %.4f)\n', x_g, y_g);
fprintf('Achieved:  (%.4f, %.4f)\n', x_check, y_check);

% --- Plot the Robot Arm ---
figure; hold on; grid on; axis equal;
title('2-DOF Planar Robot - IK Solution');
xlabel('X (m)'); ylabel('Y (m)');

% Joint positions
P0 = [0, 0];                                         % Base
P1 = [l1*cos(theta1), l1*sin(theta1)];               % Elbow
P2 = [x_check, y_check];                             % End-effector

% Draw links
plot([P0(1) P1(1)], [P0(2) P1(2)], 'b-o', 'LineWidth', 3, 'MarkerSize', 8);
plot([P1(1) P2(1)], [P1(2) P2(2)], 'r-o', 'LineWidth', 3, 'MarkerSize', 8);

% Draw target
plot(x_g, y_g, 'g*', 'MarkerSize', 15, 'LineWidth', 2);

legend('Link 1', 'Link 2', 'Target');
