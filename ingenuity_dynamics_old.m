function dxi = ingenuity_dynamics(t, xi, u, p)
    % unpack state
    V_b = xi(4:6);
    phi = xi(7); theta = xi(8); psi = xi(9);
    omega = xi(10:12);
    
    % body to inertial rotation matrix
    cph = cos(phi); sph = sin(phi);
    cth = cos(theta); sth = sin(theta);
    cps = cos(psi); sps = sin(psi);
    
    R = [cps*cth, cps*sth*sph - sps*cph, cps*sth*cph + sps*sph;
         sps*cth, sps*sth*sph + cps*cph, sps*sth*cph - cps*sph;
         -sth,    cth*sph,               cth*cph];
     
    % unpack inputs
    F_mag = u(1); % |F_m|
    alpha = u(2);
    beta  = u(3);
    
    % thrust vector
    tan_gamma_sq = tan(alpha)^2 + tan(beta)^2;
    cos_gamma = sqrt(1 / (1 + tan_gamma_sq)); 

    T_vec = [ -tan(alpha) * cos_gamma;
               tan(beta)  * cos_gamma;
              -cos_gamma ];
          
    % forces in body frame
    F_rotor_b = T_vec * F_mag;
    F_drag = -p.A_trans * V_b;
    F_tot_b = F_rotor_b + F_drag;

    % torques
    tau_thrust = cross(p.d_cm, F_rotor_b);
    tau_reaction = [0; 0; u(4)];    % reaction torque (yaw)
    tau_drag = -p.A_rot * omega;
    tau_tot_b = tau_thrust + tau_reaction + tau_drag;
    
    % equations of motion
    F_g_i = [0; 0; p.m * p.g]; 
    P_dot = R * V_b;

    % translational
    term1 = F_tot_b;                % body forces
    term2 = R' * F_g_i;             % inertial gravity rotated to body
    coriolis = cross(omega, V_b);   % kinematic coupling
    V_b_dot = (1/p.m)*(term1 + term2) - coriolis;

    % rotational
    gyroscopic = cross(omega, p.I * omega);
    omega_dot = p.invI * (-gyroscopic + tau_tot_b);
    
    % kinematics (Euler rates)
    W = [1, sph*tan(theta), cph*tan(theta);
         0, cph,           -sph;
         0, sph/cth,        cph/cth];
    Theta_dot = W * omega;

    % pack derivatives
    dxi = [P_dot; V_b_dot; Theta_dot; omega_dot];
end