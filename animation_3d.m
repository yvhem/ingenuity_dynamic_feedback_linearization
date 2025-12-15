function animation_3d(xi, t, trajectory_type, params, target_fps, filename, show_window)
    % inputs
    if nargin < 7, show_window = true; end 
    if nargin < 6, filename = []; end      
    if nargin < 5, target_fps = 30; end 
    if nargin < 3 || isempty(trajectory_type)
        trajectory_type = "box";
    end
    
    % if no filename is provided, we show the window
    if isempty(filename) && ~show_window
        warning('No filename provided. Forcing window to be visible.');
        show_window = true;
    end
    
    %% Wind and vector logic
    wind_enabled = false;
    if exist('params', 'var') && isfield(params, 'gust_force')
        wind_vec = params.gust_force;
        t_gust_start = params.gust_start;
        t_gust_end = params.gust_end;
        if norm(wind_vec) > 0
            wind_enabled = true;
        end
    end
    
    %% Reference trajectory
    ref_vals = [];
    if exist('traj_utils', 'file')
        for i = 1:length(t)
            r = traj_utils(t(i), trajectory_type);
            ref_vals = [ref_vals; r.pos'];
        end
    end
    
    % Scene bounds
    all_points = xi(:, 1:3);
    if ~isempty(ref_vals)
        all_points = [all_points; ref_vals];
    end
    
    min_bounds = min(all_points);
    max_bounds = max(all_points);
    scene_center = (min_bounds + max_bounds) / 2;
    scene_span = max(max_bounds - min_bounds) / 2 * 1.5; 
    fprintf('Starting Animation setup...\n');
    
    %% Figure
    if show_window, vis_str = 'on'; else, vis_str = 'off'; end

    anim_fig = figure('Name', 'Ingenuity Flight 3D', 'Color', 'w', ...
                      'Visible', vis_str);
    set(anim_fig, 'Position', [100, 100, 1280, 720]); 
    ax = axes(anim_fig);
    axis equal; grid on;
    view(30, 20);
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    hold(ax, 'on');
    
    % Adding Lighting
    cam_light = camlight('headlight'); 
    lighting gouraud;
    
    % Loading URDF
    if exist('mhs', 'dir'), addpath(genpath('mhs')); end
    robot = [];
    try
        robot = importrobot('mhs/MHS.urdf'); 
        robot.DataFormat = 'row';
    catch
        warning('URDF failed to load. Creating placeholder.');
        robot = rigidBodyTree;
        body = rigidBody('fuselage');
        addBody(robot, body, 'base');
    end
    
    hg_robot = hgtransform('Parent', ax);
    
    objs_before = allchild(ax);
    show(robot, 'Parent', ax, 'Frames', 'off', 'Visuals', 'on', 'Collisions', 'off');
    objs_after = allchild(ax);
    robot_parts = setdiff(objs_after, objs_before);
    set(robot_parts, 'Parent', hg_robot);
    
    all_patches = findobj(robot_parts, 'Type', 'Patch');
    for i = 1:length(all_patches)
        p = all_patches(i);
        p.EdgeColor = 'none'; p.FaceLighting = 'gouraud';
        p.FaceColor = [0.7, 0.7, 0.7]; 
        if contains(p.DisplayName, 'MainBody'), material(p, 'shiny'); 
        elseif contains(p.DisplayName, 'Blades'), p.FaceColor = [0.1 0.1 0.1]; material(p, 'dull');
        elseif contains(p.DisplayName, 'Leg'), p.FaceColor = [0.1 0.1 0.1]; end
    end
    
    geom_fix = makehgtform('xrotate', pi/2); 
    set(hg_robot, 'Matrix', geom_fix);
    
    if ~isempty(ref_vals)
        plot3(ref_vals(:,1), ref_vals(:,2), ref_vals(:,3),'r--', 'Parent', ax, 'LineWidth', 1.5, 'DisplayName', 'Reference');
    end
    plot3(xi(:,1), xi(:,2), xi(:,3), 'b:', 'Parent', ax, 'LineWidth', 1.5, 'DisplayName', 'Actual');
    h_wind = quiver3(0,0,0, 0,0,0, 'r', 'LineWidth', 3, 'MaxHeadSize', 0.5,'DisplayName', 'Wind Gust', 'AutoScale', 'off');
    h_thrust = quiver3(0,0,0, 0,0,0, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5,'DisplayName', 'Thrust Dir', 'AutoScale', 'off');
    legend([findobj(ax, 'DisplayName', 'Reference'), findobj(ax, 'DisplayName', 'Actual'), h_wind, h_thrust],'Location', 'northeast');
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
    fps = target_fps;
    dt_frame = 1 / fps; 
    t_uniform = t(1) : dt_frame : t(end);
    xi_uniform = interp1(t, xi, t_uniform, 'linear');
    
    start_zoom = 0.8; zoom_delay = 1.0; zoom_duration = 3.0;    
    
    disp("3D Animation started...")
    
    % --- progress bar ---
    h_wait = waitbar(0, 'Initializing...', 'Name', 'Rendering Animation', ...
        'CreateCancelBtn', 'setappdata(gcbf,''canceling'',1)');
    setappdata(h_wait, 'canceling', 0);
    
    start_time = tic;
    total_frames = length(t_uniform);
    
    for i = 1:total_frames
        if show_window && ~isvalid(anim_fig), break; end 
        if getappdata(h_wait, 'canceling')
            fprintf('Animation cancelled by user.\n');
            break;
        end
        
        current_time = t_uniform(i);
        
        % State
        pos = xi_uniform(i, 1:3);
        roll = xi_uniform(i, 7); pitch = xi_uniform(i, 8); yaw = xi_uniform(i, 9);
        
        % Update Transform
        R_z = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
        R_y = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
        R_x = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
        R = R_z * R_y * R_x;
        
        M = makehgtform('translate', pos) * ...
            makehgtform('zrotate', yaw) * ...
            makehgtform('yrotate', pitch) * ...
            makehgtform('xrotate', roll) * ...
            geom_fix;   
        set(hg_robot, 'Matrix', M);
        
        camlight(cam_light, 'headlight');
        
        if wind_enabled && current_time >= t_gust_start && current_time <= t_gust_end
            set(h_wind, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3), ...
                        'UData', wind_vec(1), 'VData', wind_vec(2), 'WData', wind_vec(3));
        else
            set(h_wind, 'UData', 0, 'VData', 0, 'WData', 0);
        end
        
        thrust_dir = R * [0; 0; 1]; 
        scale = 0.5;
        set(h_thrust, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3),'UData', thrust_dir(1)*scale, 'VData', thrust_dir(2)*scale, 'WData', thrust_dir(3)*scale);
        
        % Camera Logic
        if current_time < zoom_delay
            target_center = pos; target_span = start_zoom;
        elseif current_time < (zoom_delay + zoom_duration)
            rel_time = current_time - zoom_delay;
            alpha = rel_time / zoom_duration;
            target_center = (1 - alpha) * pos + alpha * scene_center;
            target_span = (1 - alpha) * start_zoom + alpha * scene_span;
        else
            target_center = scene_center; target_span = scene_span;
        end
        xlim([target_center(1) - target_span, target_center(1) + target_span]);
        ylim([target_center(2) - target_span, target_center(2) + target_span]);
        zlim([target_center(3) - target_span, target_center(3) + target_span]);
        
        title(sprintf('Time: %.2fs | Alt: %.1fm', current_time, pos(3)));
        
        % update progress bar
        if mod(i, 5) == 0
            waitbar(i/total_frames, h_wait, ...
                sprintf('Rendering: %.0f%% (Time: %.1fs / %.1fs)', ...
                (i/total_frames)*100, current_time, t_uniform(end)));
        end
        
        % save logic
        if save_video
            drawnow;
            frame = getframe(anim_fig);
            writeVideo(v_writer, frame);
        else
            drawnow limitrate;
            elapsed = toc(start_time);
            expected = (i - 1) * dt_frame;
            if elapsed < expected
                pause(expected - elapsed);
            end
        end
    end
    
    delete(h_wait);
    
    if save_video
        close(v_writer);
        fprintf('Video saved successfully to %s\n', filename);
    end
    
    if ~show_window && isvalid(anim_fig)
        close(anim_fig);
    end
    
    fprintf('Animation ended.\n');
end