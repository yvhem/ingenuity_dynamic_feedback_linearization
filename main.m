clear; clc; close all;

%% Physical params
params.m = 1.8;      % mass [kg]
params.g = 3.69;     % Mars gravity [m/s^2]
params.I = diag([0.02, 0.02, 0.03]); 
params.invI = inv(params.I);
params.A_trans = diag([0.05, 0.05, 0.1]); % Drag coefficients
params.A_rot   = diag([0.01, 0.01, 0.05]);

%% Disturbance

% Random magnitude between -3N and +3N for X, Y, Z
params.gust_force_inertial = (rand(3,1) - 0.5) * 6; 

% Define when the gust happens
params.gust_start_time = 10.0; 
params.gust_end_time = 20.0;

%% Controller gains
params.kp = [27, 36, 18, 4]; % [k0, k1, k2, k3]

params.kpsi = [4, 4]; % [k0, k1]


%% Simulation setup
t_span = [0 30];
trajectory_type = 'box'; % Options: 'box', 'helix'

% Initial state
% xi = [P(3); V(3); Euler(3); Omega(3); F_T(1); F_T_dot(1)]
xi0 = zeros(14,1);
xi0(3) = 0.0; % Start on ground
xi0(13) = params.m * params.g; % Initial thrust parabola

% Run simulation
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
[t, xi] = ode15s(@(t,x) ingenuity_closed_loop(t, x, params, trajectory_type), t_span, xi0, options);

%% Visualization
% Control inputs
F_T_hist = xi(:, 13);

% Calculate reference values for plotting
ref_vals = [];
for i = 1:length(t)
    ti = t(i); 
    r = traj_utils(ti, trajectory_type); 
    ref_vals = [ref_vals; r.pos'];
end

% Define wind timing
t_start = params.gust_start_time;
t_end   = params.gust_end_time;
x_patch = [t_start, t_end, t_end, t_start];

figure('Name', 'Ingenuity Simulation Results', 'Color', 'w');

% 3D trajectory
subplot(2,2,1);
plot3(xi(:,1), xi(:,2), xi(:,3), 'b-', 'LineWidth', 2); hold on;
plot3(ref_vals(:,1), ref_vals(:,2), ref_vals(:,3), 'k--', 'LineWidth', 1);
plot3(xi(1,1), xi(1,2), xi(1,3), 'go', 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
plot3(xi(end,1), xi(end,2), xi(end,3), 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'End');
title('3D Trajectory vs Reference'); 
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
grid on; axis equal; 
legend('Actual', 'Reference', 'Start', 'End', 'Location', 'best');


% Altitude tracking
subplot(2,2,2); 
plot(t, xi(:,3), 'b', 'LineWidth', 2); hold on;
plot(t, ref_vals(:,3), 'k--');

% Shaded region
ylim_2 = ylim;
y_patch_2 = [ylim_2(1), ylim_2(1), ylim_2(2), ylim_2(2)];
patch(x_patch, y_patch_2, 'k', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');

title('Altitude Tracking'); xlabel('t [s]'); ylabel('z [m]'); grid on; 
legend('Actual', 'Ref', 'Location', 'best');


% Euler angles
subplot(2,2,3);
plot(t, rad2deg(xi(:,7:9)), 'LineWidth', 1.5); hold on;

% Shaded region
ylim_3 = ylim; 
y_patch_3 = [ylim_3(1), ylim_3(1), ylim_3(2), ylim_3(2)];
patch(x_patch, y_patch_3, 'k', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');

legend('\phi', '\theta', '\psi', 'Location', 'best'); 
title('Attitude (Euler Angles)'); xlabel('t [s]'); ylabel('Angle [deg]'); grid on;


% Thrust input
subplot(2,2,4);
plot(t, F_T_hist, 'LineWidth', 2); hold on;
yline(params.m * params.g, 'k--', 'Hover Force');

% Shaded Region
ylim_4 = ylim; 
y_patch_4 = [ylim_4(1), ylim_4(1), ylim_4(2), ylim_4(2)];
patch(x_patch, y_patch_4, 'k', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');

title('Control Input: Thrust (F_T)'); xlabel('t [s]'); ylabel('Force [N]'); grid on;