function ref = traj_utils(t, type)
    % Generates reference trajectories.
    
    if nargin < 2
        type = 'none';
    end
    
    switch lower(type)
        case 'box'
            ref = box_traj(t);
        case 'eight'
            ref = eight_traj(t);
        otherwise
            ref = helix_traj(t);
    end
end

%% Box trajectory
function ref = box_traj(t)
    % Settings of the trajectory
    side_length = 10.0;
    altitude = 5.0;
    side_time = 5.0;
    
    % 4 corners of the square
    % P0(0,0,Z) -> P1(L,0,Z) -> P2(L,L,Z) -> P3(0,L,Z) -> P4(0,0,Z)
    % all the corners are at the same altitude
    corners = [0, 0, altitude;
               side_length, 0, altitude;
               side_length, side_length, altitude;
               0, side_length, altitude;
               0, 0, altitude]';
               
    % Cap time to end of trajectory
    t_total = 4 * side_time;
    if t >= t_total
        % Hover at start point
        ref.pos = corners(:, end);
        ref.vel = [0;0;0];
        ref.acc = [0;0;0];
        ref.jerk = [0;0;0];
        ref.snap = [0;0;0];
        ref.psi = 0; ref.psi_dot = 0; ref.psi_ddot = 0;
        return;
    end
    
    % Current side index
    side_idx = floor(t / side_time) + 1;
    
    % Local time tau normalized to [0, 1] for the current size
    t_local = mod(t, side_time);
    tau = t_local / side_time;
    
    % Quintic polynomial scaling
    s      = 10*tau^3 - 15*tau^4 + 6*tau^5;
    s_d1   = 30*tau^2 - 60*tau^3 + 30*tau^4;      
    s_d2   = 60*tau   - 180*tau^2 + 120*tau^3;    
    s_d3   = 60       - 360*tau   + 360*tau^2;    
    s_d4   =          - 360       + 720*tau;      
    
    % Chain rule scaling factors
    scale_v = 1 / side_time;
    scale_a = 1 / side_time^2;
    scale_j = 1 / side_time^3;
    scale_s = 1 / side_time^4;
    
    % Set state
    P_start = corners(:, side_idx);
    P_end   = corners(:, side_idx + 1);
    Dist    = P_end - P_start;
    
    ref.pos  = P_start + Dist * s;
    ref.vel  = Dist * (s_d1 * scale_v);
    ref.acc  = Dist * (s_d2 * scale_a);
    ref.jerk = Dist * (s_d3 * scale_j);
    ref.snap = Dist * (s_d4 * scale_s);
    
    % Yaw
    % Setting to 0 means we keep alway the same yaw angle
    % we could modifiy this to face the direction of travel
    ref.psi = 0; 
    ref.psi_dot = 0; 
    ref.psi_ddot = 0;
end

%% Helix trajectory
function ref = helix_traj(t)
    % Settings of the trajectory 
    radius = 2.0; 
    omega_ref = 0.5; 
    z_vel = 0.2;

    % Positions
    x = radius * cos(omega_ref * t);
    y = radius * sin(omega_ref * t);
    z = min(z_vel * t, 5); 

    % Velocities
    vx = -radius * omega_ref * sin(omega_ref * t);
    vy =  radius * omega_ref * cos(omega_ref * t);
    vz = (t < 25) * z_vel;

    % Accelerations
    ax = -radius * omega_ref^2 * cos(omega_ref * t);
    ay = -radius * omega_ref^2 * sin(omega_ref * t);
    az = 0;

    % Jerk
    jx =  radius * omega_ref^3 * sin(omega_ref * t);
    jy = -radius * omega_ref^3 * cos(omega_ref * t);
    jz = 0;

    % Snap
    sx =  radius * omega_ref^4 * cos(omega_ref * t);
    sy =  radius * omega_ref^4 * sin(omega_ref * t);
    sz = 0;

    % Set state
    ref.pos = [x; y; z]; ref.vel = [vx; vy; vz]; ref.acc = [ax; ay; az];
    ref.jerk = [jx; jy; jz]; ref.snap = [sx; sy; sz];

    % Set yaw
    % Also here as in the box trajcetory yaw is not of interest
    ref.psi = 0; ref.psi_dot = 0; ref.psi_ddot = 0;
end

%% 8-shape trajectory
function ref = eight_traj(t)
    % Settings of the trajectory
    scale = 3.0;          % size of the figure eight
    omega_ref = 0.4;      % angular frequency
    altitude = 5.0;       % constant altitude
    z_rise_time = 0;      % time to reach altitude
    
    % Parametric equations for lemniscate of Gerono
    % x(t) = a * cos(omega*t)
    % y(t) = a * sin(omega*t) * cos(omega*t) = (a/2) * sin(2*omega*t)
    
    wt = omega_ref * t;
    
    % Positions
    x = scale * cos(wt);
    y = 0.5 * scale * sin(2 * wt);
    z = min(altitude, (altitude / z_rise_time) * t);
    
    % Velocities
    vx = -scale * omega_ref * sin(wt);
    vy = scale * omega_ref * cos(2 * wt);
    vz = 0;               % set this for z_rise_time != 0
    
    % Accelerations
    ax = -scale * omega_ref^2 * cos(wt);
    ay = -2 * scale * omega_ref^2 * sin(2 * wt);
    az = 0;
    
    % Jerk
    jx = scale * omega_ref^3 * sin(wt);
    jy = -4 * scale * omega_ref^3 * cos(2 * wt);
    jz = 0;
    
    % Snap
    sx = scale * omega_ref^4 * cos(wt);
    sy = 8 * scale * omega_ref^4 * sin(2 * wt);
    sz = 0;
    
    % Set state
    ref.pos = [x; y; z];
    ref.vel = [vx; vy; vz];
    ref.acc = [ax; ay; az];
    ref.jerk = [jx; jy; jz];
    ref.snap = [sx; sy; sz];
    
    % Set yaw
    ref.psi = 0;
    ref.psi_dot = 0;
    ref.psi_ddot = 0;
end