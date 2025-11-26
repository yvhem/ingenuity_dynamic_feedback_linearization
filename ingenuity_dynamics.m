function dxi = ingenuity_dynamics(t, xi, u, p)
    % state xi: [P(3), V_body(3); Euler(3); omega(3)]
    % input u: [F_mag; tau_phi; tau_theta; tau_psi]

    % unpack state
    V_b = xi(4:6);
    phi = xi(7); theta = xi(8); psi = xi(9);
    omega = xi(10:12);
    
    % unpack inputs
    F_mag = u(1); % |F_m|
    tau_phi = u(2);
    tau_theta  = u(3);
    tau_psi = u(4);
    
    % body to inertial rotation matrix
    cph = cos(phi); sph = sin(phi);
    cth = cos(theta); sth = sin(theta);
    cps = cos(psi); sps = sin(psi);
    
    R = [cps*cth, cps*sth*sph - sps*cph, cps*sth*cph + sps*sph;
         sps*cth, sps*sth*sph + cps*cph, sps*sth*cph - cps*sph;
         -sth,    cth*sph,               cth*cph];
          
    % forces in body frame
    F_thrust_b = [0; 0; F_mag];
    F_drag_b = -p.A_trans * V_b;
    F_gravity_b = R' * [0; 0; -p.m * p.g];
    F_tot_b = F_thrust_b + F_drag_b + F_gravity_b;

    % torques
    tau_control_b = [tau_phi; tau_theta; tau_psi];
    tau_drag_b = -p.A_rot * omega;
    tau_tot_b = tau_control_b + tau_drag_b;
    
    % equations of motion
    % translational (Newton's law in a rotating frame)
    V_b_dot = (1/p.m) * F_tot_b - cross(omega, V_b);

    % rotational (Euler's equation)
    gyroscopic_torque = cross(omega, p.I * omega);
    omega_dot = p.invI * (tau_tot_b - gyroscopic_torque);
    
    % kinematics
    P_dot = R * V_b;

    W = [1, sph*tan(theta), cph*tan(theta);
         0, cph,           -sph;
         0, sph/cth,        cph/cth];
    Theta_dot = W * omega;

    % pack derivatives
    dxi = [P_dot; V_b_dot; Theta_dot; omega_dot];
end
