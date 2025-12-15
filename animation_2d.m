function animation_2d(xi, t, trajectory_type, params, target_fps, filename, show_window)
    % inputs
    if nargin < 7, show_window = true; end % Default: Show the animation
    if nargin < 6, filename = []; end      % Default: Don't save video
    if nargin < 5 || isempty(target_fps), target_fps = 30; end
    
    % if no filename is provided, we show the window
    if isempty(filename) && ~show_window
        warning('No filename provided. Forcing window to be visible.');
        show_window = true;
    end
    %% Data preparation
    fps = target_fps;
    dt_frame = 1 / fps; 
    t_uniform = t(1) : dt_frame : t(end);
    
    % Interpolate data
    xi_uni = interp1(t, xi, t_uniform, 'linear');
    
    F_T_hist = xi(:, 13);
    euler_deg = rad2deg(xi(:,7:9));
    
    F_max = params.TWR * params.m * params.g;
    F_min = 0.3 * params.m * params.g;
    F_T_plot = max(F_min, min(F_max, F_T_hist)); 
    
    F_T_plot_uni = interp1(t, F_T_plot, t_uniform, 'linear');
    F_T_hist_uni = interp1(t, F_T_hist, t_uniform, 'linear');
    euler_uni    = interp1(t, euler_deg, t_uniform, 'linear');
    
    %% Wind and vector
    wind_enabled = false;
    if exist('params', 'var') && isfield(params, 'gust_force')
        wind_vec = params.gust_force;
        t_gust_start = params.gust_start;
        t_gust_end = params.gust_end;
        if norm(wind_vec) > 0, wind_enabled = true; end
    end
    x_patch = [t_gust_start, t_gust_end, t_gust_end, t_gust_start];
    
    %% Reference trajectory
    ref_vals = [];
    if exist('traj_utils', 'file')
        for i = 1:length(t)
            r = traj_utils(t(i), trajectory_type);
            ref_vals = [ref_vals; r.pos'];
        end
    end
    
    all_points = xi(:, 1:3);
    if ~isempty(ref_vals), all_points = [all_points; ref_vals]; end
    
    x_lim = [min(all_points(:,1)), max(all_points(:,1))];
    y_lim = [min(all_points(:,2)), max(all_points(:,2))];
    z_lim = [min(all_points(:,3)), max(all_points(:,3))];
    z_min = z_lim(1); z_max = z_lim(2);
    f_all = [F_T_hist; F_T_plot];
    
    %% Figure Setup
    if show_window, vis_str = 'on'; else, vis_str = 'off'; end
    
    fig = figure('Name', 'Simulation Animation', 'Color', 'w', ...
                 'Position', [100, 100, 1400, 900], 'Visible', vis_str);
    
    % Subplot 1: Trajectory
    ax1 = subplot(2,2,1); hold on; grid on; axis equal; view(3);
    title('Trajectory tracking'); xlabel('x'); ylabel('y'); zlabel('z');
    xlim(x_lim); ylim(y_lim); zlim(z_lim);
    if ~isempty(ref_vals)
        plot3(ref_vals(:,1), ref_vals(:,2), ref_vals(:,3), 'k--', 'LineWidth', 1, 'DisplayName', 'Reference');
    end
    plot3(xi(1,1), xi(1,2), xi(1,3), 'go', 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
    plot3(xi(end,1), xi(end,2), xi(end,3), 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'End');
    plot3(NaN,NaN,NaN, 'b-', 'LineWidth', 2, 'DisplayName', 'Actual'); 
    h_traj_anim = animatedline('Color', 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
    h_drone = plot3(NaN, NaN, NaN, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6, 'HandleVisibility', 'off'); 
    h_wind = quiver3(0,0,0, 0,0,0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'DisplayName', 'Wind', 'AutoScale', 'off');
    h_thrust = quiver3(0,0,0, 0,0,0, 'b', 'LineWidth', 1.5, 'MaxHeadSize', 0.5, 'DisplayName', 'Thrust', 'AutoScale', 'off');
    legend(ax1, 'Location', 'best');
    
    % Subplot 2: Altitude
    ax2 = subplot(2,2,2); hold on; grid on;
    title('Altitude'); xlabel('t'); ylabel('z');
    xlim([t(1), t(end)]); ylim([z_min, z_max]);
    patch(x_patch, [z_min, z_min, z_max, z_max], 'k', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    if ~isempty(ref_vals)
        plot(t, ref_vals(:,3), 'k--', 'DisplayName', 'Ref');
    end
    plot(NaN,NaN, 'b', 'LineWidth', 2, 'DisplayName', 'Actual'); 
    h_alt_anim = animatedline('Color', 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
    legend(ax2, 'Location', 'southeast');
    
    % Subplot 3: Attitude
    ax3 = subplot(2,2,3); hold on; grid on;
    title('Attitude'); xlabel('t'); ylabel('deg');
    e_min = min(euler_deg, [], 'all')-5; e_max = max(euler_deg, [], 'all')+5;
    xlim([t(1), t(end)]); ylim([e_min, e_max]);
    patch(x_patch, [e_min, e_min, e_max, e_max], 'k', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    plot(NaN,NaN, 'Color', [0 0.4470 0.7410], 'LineWidth', 1.5, 'DisplayName', '\phi');
    plot(NaN,NaN, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5, 'DisplayName', '\theta');
    plot(NaN,NaN, 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 1.5, 'DisplayName', '\psi');
    h_phi_anim = animatedline('Color', [0 0.4470 0.7410], 'LineWidth', 1.5, 'HandleVisibility', 'off');
    h_theta_anim = animatedline('Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5, 'HandleVisibility', 'off');
    h_psi_anim = animatedline('Color', [0.9290 0.6940 0.1250], 'LineWidth', 1.5, 'HandleVisibility', 'off');
    legend(ax3, 'Location', 'best');
    
    % Subplot 4: Thrust
    ax4 = subplot(2,2,4); hold on; grid on;
    title('Thrust'); xlabel('t'); ylabel('N');
    f_min_g = min(f_all)-0.5; f_max_g = max(f_all)+0.5;
    xlim([t(1), t(end)]); ylim([f_min_g, f_max_g]);
    patch(x_patch, [f_min_g, f_min_g, f_max_g, f_max_g], 'k', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    yline(params.m * params.g, 'k--', 'DisplayName', 'Hovering');
    plot(NaN,NaN, 'b-', 'LineWidth', 2, 'DisplayName', 'Applied');
    plot(NaN,NaN, 'r:', 'LineWidth', 1, 'DisplayName', 'Commanded');
    h_thrust_app_anim = animatedline('Color', 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
    h_thrust_cmd_anim = animatedline('Color', 'r', 'LineStyle', ':', 'LineWidth', 1, 'HandleVisibility', 'off');
    legend(ax4, 'Location', 'northeast');
    
    %% Video Writer Setup
    save_video = false;
    v_writer = [];
    if ~isempty(filename)
        save_video = true;
        fprintf('Video recording enabled: %s\n', filename);
        v_writer = VideoWriter(filename, 'MPEG-4'); 
        v_writer.FrameRate = target_fps;
        v_writer.Quality = 95; 
        open(v_writer);
    end
    
    %% Animation loop
    disp('Starting Animation...');
    start_time = tic;
    
    % progress bar
    h_wait = waitbar(0, 'Initializing 2D Plots...', 'Name', 'Rendering Animation', ...
        'CreateCancelBtn', 'setappdata(gcbf,''canceling'',1)');
    setappdata(h_wait, 'canceling', 0);
    
    total_frames = length(t_uniform);
    
    for k = 1:total_frames
        if show_window && ~isvalid(fig), break; end 
        if getappdata(h_wait, 'canceling')
            fprintf('Animation cancelled by user.\n');
            break;
        end
        
        current_time = t_uniform(k);
        
        % update plots
        addpoints(h_traj_anim, xi_uni(k,1), xi_uni(k,2), xi_uni(k,3));
        set(h_drone, 'XData', xi_uni(k,1), 'YData', xi_uni(k,2), 'ZData', xi_uni(k,3));
        
        pos = xi_uni(k, 1:3);
        roll = xi_uni(k, 7); pitch = xi_uni(k, 8); yaw = xi_uni(k, 9);
        
        if wind_enabled && current_time >= t_gust_start && current_time <= t_gust_end
             set(h_wind, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3), ...
                'UData', wind_vec(1), 'VData', wind_vec(2), 'WData', wind_vec(3));
        else
             set(h_wind, 'UData', 0, 'VData', 0, 'WData', 0);
        end
        
        R_z = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
        R_y = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
        R_x = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
        R = R_z * R_y * R_x;
        thrust_dir = R * [0;0;1];
        scale = 0.5;
        set(h_thrust, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3), ...
            'UData', thrust_dir(1)*scale, 'VData', thrust_dir(2)*scale, 'WData', thrust_dir(3)*scale);
        
        addpoints(h_alt_anim, current_time, xi_uni(k,3));
        addpoints(h_phi_anim, current_time, euler_uni(k,1));
        addpoints(h_theta_anim, current_time, euler_uni(k,2));
        addpoints(h_psi_anim, current_time, euler_uni(k,3));
        addpoints(h_thrust_app_anim, current_time, F_T_plot_uni(k));
        addpoints(h_thrust_cmd_anim, current_time, F_T_hist_uni(k));
        
        % update progress bar
        if mod(k, 5) == 0
            waitbar(k/total_frames, h_wait, ...
                sprintf('Rendering: %.0f%% (Time: %.1fs / %.1fs)', ...
                (k/total_frames)*100, current_time, t_uniform(end)));
        end
        
        % rendering logic
        if save_video
            drawnow;
            frame = getframe(fig);
            writeVideo(v_writer, frame);
        else
            drawnow limitrate;
            elapsed = toc(start_time);
            expected = k * dt_frame;
            if elapsed < expected
                pause(expected - elapsed);
            end
        end
    end
    
    delete(h_wait);
    
    if save_video
        close(v_writer);
        fprintf('Video saved: %s\n', filename);
    end
    
    if ~show_window && isvalid(fig)
        close(fig);
    end
    fprintf('Done.\n');
end