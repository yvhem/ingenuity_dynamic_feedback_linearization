clear; clc; close all;

%% parameters
params.m = 1.8;      % mass [kg]
params.g = 3.69;     % Mars gravity [m/s^2]
params.I = diag([0.02, 0.02, 0.03]); 
params.invI = inv(params.I);

% drag coefficients
params.A_trans = diag([0.05, 0.05, 0.1]);
params.A_rot   = diag([0.01, 0.01, 0.05]);

% controller gains
params.kp = [16, 32, 24, 8];
params.kpsi = [4, 4];

% thrust-to-weight ratio (130-160%)
params.TWR = 1.45; 

%% disturbance
% wind gust with random magnitude +/- 3N on x,y,z
params.gust_force = (rand(3,1) - 0.5) * 3; 
params.gust_start = 10.0; 
params.gust_end = 20.0;

%% sim setup
t_span = [0 30];
trajectory_type = 'helix'; % 'box', 'helix'

% initial state
xi0 = zeros(14,1);
% start on the ground (z=0) compensating for gravity (F_T=mg)
xi0(3) = 0.0;
xi0(13) = params.m * params.g;

% run sim
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
[t, xi] = ode15s(@(t,x) ingenuity_closed_loop(t, x, params, trajectory_type), t_span, xi0, options);

%% visualization
F_T_hist = xi(:, 13);

ref_vals = [];
for i = 1:length(t)
    ti = t(i); 
    r = traj_utils(ti, trajectory_type); 
    ref_vals = [ref_vals; r.pos'];
end

% disturbance timing
t_start = params.gust_start;
t_end   = params.gust_end;
x_patch = [t_start, t_end, t_end, t_start];

figure('Name', 'Simulation results', 'Color', 'w');

% 3D trajectory
subplot(2,2,1);
plot3(xi(:,1), xi(:,2), xi(:,3), 'b-', 'LineWidth', 2); hold on;
plot3(ref_vals(:,1), ref_vals(:,2), ref_vals(:,3), 'k--', 'LineWidth', 1);
plot3(xi(1,1), xi(1,2), xi(1,3), 'go', 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
plot3(xi(end,1), xi(end,2), xi(end,3), 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'End');
title('Trajectory tracking'); 
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
grid on; axis equal; 
legend('actual', 'reference', 'start', 'end', 'Location', 'best');

% altitude
subplot(2,2,2); 
plot(t, xi(:,3), 'b', 'LineWidth', 2); hold on;
plot(t, ref_vals(:,3), 'k--');
title('Altitude tracking'); xlabel('t [s]'); ylabel('z [m]'); grid on; 
legend('actual', 'ref', 'Location', 'best');
% disturbance region
ylim_2 = ylim;
y_patch_2 = [ylim_2(1), ylim_2(1), ylim_2(2), ylim_2(2)];
patch(x_patch, y_patch_2, 'k', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');

% Euler angles
subplot(2,2,3);
plot(t, rad2deg(xi(:,7:9)), 'LineWidth', 1.5); hold on;
legend('\phi', '\theta', '\psi', 'Location', 'best'); 
title('Attitude'); xlabel('t [s]'); ylabel('angle [deg]'); grid on;
% disturbance region
ylim_3 = ylim; 
y_patch_3 = [ylim_3(1), ylim_3(1), ylim_3(2), ylim_3(2)];
patch(x_patch, y_patch_3, 'k', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');

% thrust
subplot(2,2,4);
F_max = params.TWR * params.m * params.g;
F_min = 0.3 * params.m * params.g;
F_T_plot = max(F_min, min(F_max, F_T_hist));
plot(t, F_T_plot, 'LineWidth', 2); hold on;
plot(t, F_T_hist, 'r:', 'LineWidth', 1, 'DisplayName', 'commanded');
yline(params.m * params.g, 'k--', 'DisplayName', 'hovering');
title('Thrust'); xlabel('t [s]'); ylabel('force [N]'); grid on;
legend('applied', 'commanded', 'hovering', 'Location', 'best');
% disturbance region
ylim_4 = ylim; 
y_patch_4 = [ylim_4(1), ylim_4(1), ylim_4(2), ylim_4(2)];
patch(x_patch, y_patch_4, 'k', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');

%% performance metrics
error_norms = zeros(length(t), 1);
for k=1:length(t)
    ref = traj_utils(t(k), trajectory_type);
    error_norms(k) = norm(ref.pos - xi(k, 1:3)');
end

% metric 1: global tracking accuracy
rmse_pos = sqrt(mean(error_norms.^2));
max_error = max(error_norms);

% metric 2: robustness (wind rejection)
idx_gust = t >= params.gust_start & t <= params.gust_end;
if any(idx_gust)
    max_gust_error = max(error_norms(idx_gust));
else
    max_gust_error = 0;
end

% metric 3: control effort: % of time the motors were saturated
is_saturated = (F_T_hist >= (F_max - 1e-3)) | (F_T_hist <= (F_min + 1e-3));
saturation_pct = (sum(is_saturated) / length(t)) * 100;

% display results
fprintf('RMSE position:\t\t%.4f [m]\n', rmse_pos);
fprintf('MAE:\t\t\t%.4f [m]\n', max_error);
fprintf('Max error during gust:\t%.4f [m]\n', max_gust_error);
fprintf('Actuator saturation:\t%.2f %% of flight time\n', saturation_pct);