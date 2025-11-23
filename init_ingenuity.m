clear; clc; close all;

% physical params
params.m = 1.8;     % mass [kg]
params.g = 3.69;    % Mars gravity [m/s^2]

% inertia matrix
params.I = diag([0.02, 0.02, 0.03]); 
params.invI = inv(params.I);

% rotor hub location relative to CoM
% z-down frame, so negative z is up
params.d_cm = [0; 0; -0.3]; 

% aerodynamic params
params.rho = 0.017;         % Mars air density [kg/m^3]
params.R_rotor = 0.6;       % rotor radius [m]

% linear drag coefficients
params.A_trans = diag([0.05, 0.05, 0.1]);
params.A_rot   = diag([0.01, 0.01, 0.05]);

% sim setup
t_span = [0 5]; 
% state: [P(3); V_body(3); Euler(3); Omega(3)]
xi0 = zeros(12,1);
xi0(3) = -0.5; % start 0.5m height

% control inputs u = [F_mag; alpha; beta; tau_yaw]
% hover thrust approximation
u_hover = params.m * params.g;

% test input: hover thrust + 5 deg pitch
u_control = @(t) [u_hover; deg2rad(5); 0; 0]; 

% run sim
[t, xi] = ode45(@(t,x) ingenuity_dynamics(t, x, u_control(t), params), t_span, xi0);

% visualization
figure(1); 
plot(t, -xi(:,3), 'LineWidth', 2); 
title('Altitude (-Z)'); grid on;

figure(2);
plot(t, xi(:,4:6)); 
legend('u','v','w'); title('Body Velocities'); grid on;