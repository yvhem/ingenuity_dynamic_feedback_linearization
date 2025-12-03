clear; clc; close all;

% Parameters to config by user

wind_enabled = true;            % true = wind on, false = wind off
wind_random = true;            % false = deterministic intensity
wind_direction = 'xyz';           % 'x', 'y', 'z', 'xy', 'xz', 'yz', 'xyz'
wind_intensity = 2;            % force magnitude [N] (negative = against axis)
wind_start = 0.0;              % start time [s]
wind_end = 30.0;                % end time [s]

traj_config.type = 'box';     % 'box', 'helix', 'eight'
traj_config.yaw_mode = 'constant';      % 'spinning', 'constant'

t_span = [0 30];                % [start, end] time [s]

%% Fixed params
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

if wind_enabled
    % Parse wind direction
    wind_vec = [0; 0; 0];
    if wind_random
        if contains(lower(wind_direction), 'x')
            wind_vec(1) = (rand() - 0.5) * 2 * wind_intensity;
        end
        if contains(lower(wind_direction), 'y')
            wind_vec(2) = (rand() - 0.5) * 2 * wind_intensity;
        end
        if contains(lower(wind_direction), 'z')
            wind_vec(3) = (rand() - 0.5) * 2 * wind_intensity;
        end
    else
        % Deterministic wind
        if contains(lower(wind_direction), 'x')
            wind_vec(1) = wind_intensity;
        end
        if contains(lower(wind_direction), 'y')
            wind_vec(2) = wind_intensity;
        end
        if contains(lower(wind_direction), 'z')
            wind_vec(3) = wind_intensity;
        end
    end
    params.gust_force = wind_vec;
    params.gust_start = wind_start;
    params.gust_end = wind_end;
else
    params.gust_force = [0; 0; 0];
    params.gust_start = inf;
    params.gust_end = inf;
end

xi0 = zeros(14,1);
xi0(3) = 0.0;  % start on the ground (z=0)
xi0(13) = params.m * params.g;  % initial thrust

%% run sim
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
[t, xi] = ode15s(@(t,x) ingenuity_closed_loop(t, x, params, traj_config), t_span, xi0, options);

%% visualization
F_T_hist = xi(:, 13);
ref_vals = zeros(length(t), 3);
for i = 1:length(t)
    r = traj_utils(t(i), traj_config.type, traj_config.yaw_mode); 
    ref_vals(i,:) = r.pos';
end

% dir setup
base_dir = fullfile('plots', traj_config.type);
if wind_enabled
    sub_dir = 'wind';
    wind_str = wind_direction;
else
    sub_dir = 'no_wind';
    wind_str = 'none';
end
save_dir = fullfile(base_dir, sub_dir);
if ~exist(save_dir, 'dir')
    mkdir(save_dir); 
end
suffix = sprintf('%s_%s', wind_str, traj_config.yaw_mode);

% disturbance timing
t_start = params.gust_start;
t_end   = params.gust_end;
x_patch = [t_start, t_end, t_end, t_start];

% limits for thrust plot
F_max = params.TWR * params.m * params.g;
F_min = 0.3 * params.m * params.g;
F_T_plot = max(F_min, min(F_max, F_T_hist));

%% summary plot
f_sum = figure('Name', 'Summary', 'Color', 'w', 'Position', [100, 100, 1400, 900]);

% 3D trajectory
subplot(2,2,1);
plot3(xi(:,1), xi(:,2), xi(:,3), 'b-', 'LineWidth', 2); hold on;
plot3(ref_vals(:,1), ref_vals(:,2), ref_vals(:,3), 'k--', 'LineWidth', 1);
plot3(xi(1,1), xi(1,2), xi(1,3), 'go', 'MarkerFaceColor', 'g', 'DisplayName', 'start');
plot3(xi(end,1), xi(end,2), xi(end,3), 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'end');
title('Trajectory tracking', 'FontSize', 16); 
xlabel('x [m]', 'FontSize', 16); ylabel('y [m]', 'FontSize', 16); zlabel('z [m]', 'FontSize', 16);
grid on; axis equal; 
legend('actual', 'reference', 'start', 'end', 'Location', 'southeast', 'FontSize', 16);
set(gca, 'FontSize', 16); 

% altitude tracking
subplot(2,2,2); 
plot(t, xi(:,3), 'b', 'LineWidth', 2); hold on;
plot(t, ref_vals(:,3), 'k--');
title('Altitude tracking', 'FontSize', 16); xlabel('t [s]', 'FontSize', 16); ylabel('z [m]', 'FontSize', 16); grid on; 
legend('actual', 'ref', 'Location', 'best', 'FontSize', 16);
ylim_2 = ylim;
y_patch_2 = [ylim_2(1), ylim_2(1), ylim_2(2), ylim_2(2)];
patch(x_patch, y_patch_2, 'k', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
set(gca, 'FontSize', 16);

% attitude
subplot(2,2,3);
plot(t, rad2deg(xi(:,7:9)), 'LineWidth', 1.5); hold on;
legend('\phi', '\theta', '\psi', 'Location', 'southeast', 'FontSize', 16); 
title('Attitude', 'FontSize', 16); xlabel('t [s]', 'FontSize', 16); ylabel('angle [deg]', 'FontSize', 16); grid on;
ylim_3 = ylim; 
y_patch_3 = [ylim_3(1), ylim_3(1), ylim_3(2), ylim_3(2)];
patch(x_patch, y_patch_3, 'k', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
set(gca, 'FontSize', 16);

% thrust
subplot(2,2,4);
plot(t, F_T_plot, 'LineWidth', 2); hold on;
plot(t, F_T_hist, 'r:', 'LineWidth', 1, 'DisplayName', 'commanded');
yline(params.m * params.g, 'k--', 'DisplayName', 'hovering');
title('Thrust', 'FontSize', 16); xlabel('t [s]', 'FontSize', 16); ylabel('force [N]', 'FontSize', 16); grid on;
legend('applied', 'commanded', 'hovering', 'Location', 'best', 'FontSize', 16);
ylim_4 = ylim; 
y_patch_4 = [ylim_4(1), ylim_4(1), ylim_4(2), ylim_4(2)];
patch(x_patch, y_patch_4, 'k', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
set(gca, 'FontSize', 16);

saveas(f_sum, fullfile(save_dir, ['summary_' suffix '.png']));

%% individual big plots
% 3D path
f_path = figure('Name', '3D path', 'Color', 'w', 'Visible', 'off'); % Hidden
plot3(xi(:,1), xi(:,2), xi(:,3), 'b-', 'LineWidth', 2); hold on;
plot3(ref_vals(:,1), ref_vals(:,2), ref_vals(:,3), 'k--', 'LineWidth', 1.5);
plot3(xi(1,1), xi(1,2), xi(1,3), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
plot3(xi(end,1), xi(end,2), xi(end,3), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
title('Trajectory tracking'); xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]'); 
grid on; axis equal; legend('actual', 'reference', 'start', 'end');
set(gca, 'FontSize', 14); % Bigger font for individual plots
saveas(f_path, fullfile(save_dir, ['path_3d_' suffix '.png']));

% altitude
f_alt = figure('Name', 'Altitude', 'Color', 'w', 'Visible', 'off'); % Hidden
plot(t, xi(:,3), 'b', 'LineWidth', 2); hold on;
plot(t, ref_vals(:,3), 'k--', 'LineWidth', 1.5);
title('Altitude tracking'); xlabel('t [s]'); ylabel('z [m]'); grid on; legend('actual', 'ref');
yl = ylim; patch(x_patch, [yl(1) yl(1) yl(2) yl(2)], 'k', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
set(gca, 'FontSize', 14);
saveas(f_alt, fullfile(save_dir, ['altitude_' suffix '.png']));

% attitude
f_att = figure('Name', 'Attitude', 'Color', 'w', 'Visible', 'off'); % Hidden
plot(t, rad2deg(xi(:,7:9)), 'LineWidth', 1.5); hold on;
title('Attitude'); xlabel('t [s]'); ylabel('angle [deg]'); grid on; legend('\phi', '\theta', '\psi');
yl = ylim; patch(x_patch, [yl(1) yl(1) yl(2) yl(2)], 'k', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
set(gca, 'FontSize', 14);
saveas(f_att, fullfile(save_dir, ['attitude_' suffix '.png']));

% thrust
f_thr = figure('Name', 'Thrust', 'Color', 'w', 'Visible', 'off'); % Hidden
plot(t, F_T_plot, 'LineWidth', 2); hold on;
plot(t, F_T_hist, 'r:', 'LineWidth', 1.5);
yline(params.m * params.g, 'k--', 'DisplayName', 'hovering');
title('Thrust'); xlabel('t [s]'); ylabel('force [N]'); grid on; legend('applied', 'commanded', 'hovering');
yl = ylim; patch(x_patch, [yl(1) yl(1) yl(2) yl(2)], 'k', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
set(gca, 'FontSize', 14);
saveas(f_thr, fullfile(save_dir, ['thrust_' suffix '.png']));

%% plots of the state
% XY components
f_xy = figure('Name', 'XY components', 'Color', 'w', 'Visible', 'off'); % Hidden
subplot(2,1,1);
plot(t, xi(:,1), 'b', 'LineWidth', 1.5); hold on; plot(t, ref_vals(:,1), 'k--');
ylabel('x [m]'); title('X position'); grid on; legend('actual', 'ref');
yl = ylim; patch(x_patch, [yl(1) yl(1) yl(2) yl(2)], 'k', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
set(gca, 'FontSize', 14);

subplot(2,1,2);
plot(t, xi(:,2), 'b', 'LineWidth', 1.5); hold on; plot(t, ref_vals(:,2), 'k--');
ylabel('y [m]'); title('Y position'); xlabel('t [s]'); grid on;
yl = ylim; patch(x_patch, [yl(1) yl(1) yl(2) yl(2)], 'k', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
set(gca, 'FontSize', 14);
saveas(f_xy, fullfile(save_dir, ['xy_components_' suffix '.png']));

% Linear velocities
f_vel = figure('Name', 'Linear velocities', 'Color', 'w', 'Visible', 'off'); % Hidden
plot(t, xi(:, 4:6), 'LineWidth', 1.5); hold on;
title('Body linear velocities'); xlabel('t [s]'); ylabel('velocity [m/s]'); grid on; legend('u', 'v', 'w');
yl = ylim; patch(x_patch, [yl(1) yl(1) yl(2) yl(2)], 'k', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
set(gca, 'FontSize', 14);
saveas(f_vel, fullfile(save_dir, ['velocities_' suffix '.png']));

% Angular rates
f_rates = figure('Name', 'Angular rates', 'Color', 'w', 'Visible', 'off'); % Hidden
plot(t, rad2deg(xi(:, 10:12)), 'LineWidth', 1.5); hold on;
title('Body angular rates'); xlabel('t [s]'); ylabel('rate [deg/s]'); grid on; legend('p', 'q', 'r');
yl = ylim; patch(x_patch, [yl(1) yl(1) yl(2) yl(2)], 'k', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
set(gca, 'FontSize', 14);
saveas(f_rates, fullfile(save_dir, ['rates_' suffix '.png']));

fprintf('Plots saved to: %s\n', save_dir);

%% performance metrics
error_norms = zeros(length(t), 1);
for k=1:length(t)
    ref = traj_utils(t(k), traj_config.type, traj_config.yaw_mode);
    error_norms(k) = norm(ref.pos - xi(k, 1:3)');
end

% metric 1: robustness (wind rejection)
idx_gust = t >= params.gust_start & t <= params.gust_end;
if any(idx_gust)
    max_gust_error = max(error_norms(idx_gust));
else
    max_gust_error = NaN;
end

% metric 2: control effort: % of time the motors were saturated
is_saturated = (F_T_hist >= (F_max - 1e-3)) | (F_T_hist <= (F_min + 1e-3));
saturation_pct = (sum(is_saturated) / length(t)) * 100;

fprintf('Max error during gust:\t%.4f [m]\n', max_gust_error);
fprintf('Actuator saturation:\t%.2f %% of flight time\n', saturation_pct);

%% animations
animation_2d(xi, t, traj_config.type, params, 60);
animation_3d(xi, t, traj_config.type, params, 60);