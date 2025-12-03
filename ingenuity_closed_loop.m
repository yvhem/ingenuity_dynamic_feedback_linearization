function dxi_ext = ingenuity_closed_loop(t, xi_ext, p, traj_config)
    % extended state
    P = xi_ext(1:3);
    V_b = xi_ext(4:6);
    Theta = xi_ext(7:9); % [phi; theta; psi]
    phi = Theta(1); theta = Theta(2); psi = Theta(3);
    omega = xi_ext(10:12);
    
    % controller state
    F_T = xi_ext(13);
    F_T_dot = xi_ext(14);

    % reference trajectory
    ref = traj_utils(t, traj_config.type, traj_config.yaw_mode);
    
    % rotation matrix from body to inertial frame
    cph = cos(phi); sph = sin(phi);
    cth = cos(theta); sth = sin(theta);
    cps = cos(psi); sps = sin(psi);    
    R = [cps*cth, cps*sth*sph - sps*cph, cps*sth*cph + sps*sph;
         sps*cth, sps*sth*sph + cps*cph, sps*sth*cph - cps*sph;
         -sth,    cth*sph,                cth*cph];

    % W -> Euler rates
    W_mat = [1, sph*sth/cth, cph*sth/cth;
             0, cph,         -sph;
             0, sph/cth,     cph/cth];
    Theta_dot_current = W_mat * omega;
    phi_dot = Theta_dot_current(1); 
    W_row3 = W_mat(3, :);
    
    % forces in body frame
    F_thrust_b = [0; 0; F_T];
    F_drag_b = -p.A_trans * V_b;
    F_gravity_b = R' * [0; 0; -p.m * p.g];
    
    F_tot_b = F_thrust_b + F_drag_b + F_gravity_b;
    V_b_dot_current = (1/p.m) * F_tot_b - cross(omega, V_b);
    
    %% DFL: P channel
    % velocity
    P_dot = R * V_b;
    
    % acceleration
    P_ddot = (1/p.m) * (R * [0;0;F_T] + R*F_drag_b) + [0;0;-p.g];
    
    % jerk
    e3 = [0;0;1];
    omega_skew = [0 -omega(3) omega(2); omega(3) 0 -omega(1); -omega(2) omega(1) 0];
    F_drag_dot_b = -p.A_trans * V_b_dot_current;
    term_thrust_jerk = (1/p.m) * (F_T_dot * R * e3 + F_T * R * omega_skew * e3);
    term_drag_jerk   = (1/p.m) * R * (cross(omega, F_drag_b) + F_drag_dot_b);
    P_dddot = term_thrust_jerk + term_drag_jerk; 

    % P errors
    e_p = ref.pos - P;
    e_v = ref.vel - P_dot;
    e_a = ref.acc - P_ddot;
    e_j = ref.jerk - P_dddot;
    
    % P virtual input
    nu_pos = ref.snap + p.kp(4)*e_j + p.kp(3)*e_a + p.kp(2)*e_v + p.kp(1)*e_p;
    
    %% DFL: yaw channel
    psi_dot = W_row3 * omega;

    % errors
    e_psi = psi - ref.psi;
    e_psi_dot = psi_dot - ref.psi_dot;
    
    % yaw virtual input
    nu_psi = ref.psi_ddot - (p.kpsi(2)*e_psi_dot + p.kpsi(1)*e_psi);
    
    % stack v_P and v_psi
    nu = [nu_pos; nu_psi];

    %% DFL: decoupling matrix J
    J11 = (1/p.m) * R * e3;
    e3_skew = [0 -1 0; 1 0 0; 0 0 0];
    J12 = -(F_T/p.m) * R * e3_skew * p.invI;
    J21 = 0;
    J22 = W_row3 * p.invI;
    
    J_mat = [J11, J12; 
             J21, J22];
             
    %% DFL: drift l
    % yaw drift
    q = omega(2); r = omega(3);
    term1 = (q*cph - r*sph)*phi_dot/cth;
    term2 = (q*sph + r*cph)*(sth/cth^2)*(q*cph - r*sph); 
    W_dot_omega = term1 + term2; 
    
    tau_drag = -p.A_rot * omega;
    gyro = cross(omega, p.I * omega);
    omega_dot_drift = p.invI * (tau_drag - gyro);
    
    l_psi = W_dot_omega + W_row3 * omega_dot_drift;
    
    % P drift
    % analytically deriving the full Snap drift for drag terms is complex
    % => we ignore it assuming that the change in drag is negligible
    
    term_coriolis = (2 * F_T_dot / p.m) * R * omega_skew * e3;
    term_centrifugal = (F_T / p.m) * R * (omega_skew * omega_skew) * e3;
    
    omega_dot_drift_skew = [0 -omega_dot_drift(3) omega_dot_drift(2); omega_dot_drift(3) 0 -omega_dot_drift(1); -omega_dot_drift(2) omega_dot_drift(1) 0];
    term_rot_coupling = (F_T / p.m) * R * omega_dot_drift_skew * e3;
    
    l_pos = term_coriolis + term_centrifugal + term_rot_coupling;
    
    % stack l_P and l_psi
    l_vec = [l_pos; l_psi];
    
    %% DFL: solve
    % check J singular (F_T=0)
    if abs(F_T) < 1e-3
        v = zeros(4,1); 
    else
        v = J_mat \ (nu - l_vec);
    end
    
    F_T_ddot_cmd = v(1);
    tau_cmd = v(2:4);
    
    % physical limits
    max_thrust = p.TWR * p.m * p.g;
    min_thrust = 0.3 * p.m * p.g; 
    max_torque = 0.05;
    
    F_T_saturated = max(min_thrust, min(max_thrust, F_T));
    tau_saturated = max(-max_torque, min(max_torque, tau_cmd));
    
    %% plant dynamics integration with saturated inputs
    if isfield(p, 'gust_force')
        F_wind_i = p.gust_force;
        t_start = p.gust_start;
        t_end   = p.gust_end;
    else
        F_wind_i = [0;0;0];
        t_start = 9999; t_end = 9999;
    end

    % Apply gust only within the specific time window
    if t >= t_start && t <= t_end
        F_gust_b = R' * F_wind_i; % Rotate inertial wind to body frame
    else
        F_gust_b = [0; 0; 0];
    end

    F_thrust_b_sat = [0; 0; F_T_saturated];
    F_tot_b_sat = F_thrust_b_sat + F_drag_b + F_gravity_b + F_gust_b;
    
    tau_tot_b = tau_saturated + tau_drag;
    
    V_b_dot_final = (1/p.m) * F_tot_b_sat - cross(omega, V_b);
    omega_dot_final = p.invI * (tau_tot_b - gyro);
    
    dxi_ext = [P_dot; V_b_dot_final; Theta_dot_current; omega_dot_final; F_T_dot; F_T_ddot_cmd];
end