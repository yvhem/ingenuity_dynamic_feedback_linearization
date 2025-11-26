clear; clc; close all;

%% physical params
params.m = 1.8;     % mass [kg]
params.g = 3.69;    % Mars gravity [m/s^2]

% inertia matrix
params.I = diag([0.02, 0.02, 0.03]); 
params.invI = inv(params.I);

% linear drag coefficients
params.A_trans = diag([0.05, 0.05, 0.1]);
params.A_rot   = diag([0.01, 0.01, 0.05]);

%% sim setup
t_span = [0 120];

% initial state
xi0 = zeros(12,1);
xi0(3) = 0.5; % start at 0.5m altitude

% control input
hover_thrust = params.m * params.g;  % thrust to counteract gravity
pitch_torque = 0.005;                % small torque for forward motion [Nm]
%pitch_torque = 0;

% test: time-varying control input
%u_control = @(t) [0; 0; 0; 0]; % test with zero input
u_control = @(t) [hover_thrust; 0; pitch_torque; 0]; 

% run sim
[t, xi] = ode45(@(t,x) ingenuity_dynamics(t, x, u_control(t), params), t_span, xi0);

%% visualization
% 3D traj
subplot(2,2,1);
plot3(xi(:,1), xi(:,2), xi(:,3), 'b-', 'LineWidth', 2);
hold on;
plot3(xi(1,1), xi(1,2), xi(1,3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(xi(end,1), xi(end,2), xi(end,3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
title('3D trajectory'); xlabel('x [m]'); ylabel('y [m]'); zlabel('altitude [m]');
grid on; axis equal;

% Altitude
subplot(2,2,2); 
plot(t, xi(:,3), 'LineWidth', 2); 
title('altitude (z)'); xlabel('time [s]'); ylabel('altitude [m]'); grid on;

% Euler Angles
subplot(2,2,3);
plot(t, rad2deg(xi(:,7:9))); 
legend('\phi (roll)', '\theta (pitch)', '\psi (yaw)'); 
title('Euler angles'); xlabel('time [s]'); ylabel('angle [deg]'); grid on;

% Body-frame Linear Velocities
subplot(2,2,4);
plot(t, xi(:,4:6)); 
legend('V^b_x', 'V^b_y', 'V^b_z'); 
title('body-frame velocities'); xlabel('time [s]'); ylabel('velocity [m/s]'); grid on;