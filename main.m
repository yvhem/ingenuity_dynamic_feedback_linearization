clear; clc; close all;

%% Physical params
params.m = 1.8;      % mass [kg]
params.g = 3.69;     % Mars gravity [m/s^2]
params.I = diag([0.02, 0.02, 0.03]); 
params.invI = inv(params.I);
params.A_trans = diag([0.05, 0.05, 0.1]); % Drag coefficients
params.A_rot   = diag([0.01, 0.01, 0.05]);

%% Controller gains
params.kp = [27, 36, 18, 4]; % [k0, k1, k2, k3]

params.kpsi = [4, 4]; % [k0, k1]


%% Simulation setup
t_span = [0 30];
trajectory_type = 'helix'; % Options: 'box', 'helix'

% Initial state
% xi = [P(3); V(3); Euler(3); Omega(3); F_T(1); F_T_dot(1)]
xi0 = zeros(14,1);
xi0(3) = 0.0; % Start on ground
xi0(13) = params.m * params.g; % Initial thrust parabola

% Run simulation
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
[t, xi] = ode15s(@(t,x) ingenuity_closed_loop(t, x, params, trajectory_type), t_span, xi0, options);

%% Visualization
% Extract control inputs from state history
F_T_hist = xi(:, 13);

% Calculate reference values for plotting
ref_vals = [];
for i = 1:length(t)
    ti = t(i); 
    r = traj_utils(ti, trajectory_type); 
    ref_vals = [ref_vals; r.pos'];
end

% Plotting
figure('Name', 'Ingenuity Simulation Results', 'Color', 'w');

% 3D Trajectory
subplot(2,2,1);
plot3(xi(:,1), xi(:,2), xi(:,3), 'b-', 'LineWidth', 2); hold on;
plot3(ref_vals(:,1), ref_vals(:,2), ref_vals(:,3), 'k--', 'LineWidth', 1);
plot3(xi(1,1), xi(1,2), xi(1,3), 'go', 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
plot3(xi(end,1), xi(end,2), xi(end,3), 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'End');
title('3D Trajectory vs Reference'); 
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
grid on; axis equal; legend('Actual', 'Reference');

% Altitude tracking
subplot(2,2,2); 
plot(t, xi(:,3), 'b', 'LineWidth', 2); hold on;
plot(t, ref_vals(:,3), 'k--');
title('Altitude Tracking'); xlabel('t [s]'); ylabel('z [m]'); grid on; legend('Actual', 'Ref');

% Euler angles
subplot(2,2,3);
plot(t, rad2deg(xi(:,7:9))); 
legend('\phi', '\theta', '\psi'); 
title('Attitude (Euler Angles)'); xlabel('t [s]'); ylabel('Angle [deg]'); grid on;

% Thrust input
subplot(2,2,4);
plot(t, F_T_hist, 'LineWidth', 2);
yline(params.m * params.g, 'k--', 'Hover Force');
title('Control Input: Thrust (F_T)'); xlabel('t [s]'); ylabel('Force [N]'); grid on;