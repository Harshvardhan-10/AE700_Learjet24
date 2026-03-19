% compute_autopilot_gains.m
%   Designs all autopilot gains using B&M Chapter 6 methodology.
%   Gains are derived analytically from transfer function coefficients.
%
%   Design bandwidth hierarchy (inner to outer, each ~5x slower):
%     pitch:    omega_n_theta  (~4.8 rad/s)
%     roll:     omega_n_phi    (~2.85 rad/s)
%     altitude: omega_n_h      (~0.5 rad/s)  << pitch
%     throttle: omega_n_Vt     (~0.5 rad/s)
%     course:   omega_n_chi    (~0.57 rad/s) << roll
%     sideslip: omega_n_beta   (~0.1 rad/s)


addpath('../chap3')
addpath('../chap4')
addpath('../chap5')
load('transfer_function_coef.mat')
addpath('../parameters')
simulation_parameters
load('trim_results.mat')

% -----------------------------------------------------------------------
% Basic autopilot parameters
% -----------------------------------------------------------------------
AP.gravity = 9.81;
AP.sigma   = 0.05;          % dirty-derivative filter (not used when rates measured directly)
AP.Va0     = Va_trim;       % trim airspeed
AP.Ts      = SIM.ts_simulation;

% -----------------------------------------------------------------------
% Control surface and tracking error limits
%   Project specifies: surface deflection bound = 40 degrees
% -----------------------------------------------------------------------
AP.delta_a_max  = 40 * pi/180;   % [rad]
AP.delta_e_max  = 40 * pi/180;
AP.delta_r_max  = 40 * pi/180;
AP.phi_c_max    = 30 * pi/180;   % max commanded roll  [rad]
AP.theta_c_max  = 30 * pi/180;   % max commanded pitch [rad]

e_phi_max   = 20 * pi/180;       % max roll error for gain design  [rad]
e_theta_max = 15 * pi/180;       % max pitch error for gain design [rad]

% -----------------------------------------------------------------------
% ROLL LOOP  (inner lateral)
%   Plant: phi/delta_a = a_phi2 / (s^2 + a_phi1*s)
%   Controller: PD  delta_a = kp*(phi_c-phi) + kd*(0-p)
%   Design: place CL poles at desired omega_n_phi, zeta_phi = 0.707
%
%   CL char poly: s^2 + (a_phi1 + a_phi2*kd)*s + a_phi2*kp = 0
%   => omega_n_phi^2 = a_phi2*kp  (needs kp*a_phi2 > 0)
%   => kp = sign(a_phi2) * delta_a_max / e_phi_max
% -----------------------------------------------------------------------
zeta_phi    = 0.707;
omega_n_phi = sqrt(abs(a_phi2) * AP.delta_a_max / e_phi_max);

AP.roll_kp  = sign(a_phi2) * AP.delta_a_max / e_phi_max;
AP.roll_kd  = (2*zeta_phi*omega_n_phi - a_phi1) / a_phi2;

fprintf('=== Roll Loop ===\n');
fprintf('  omega_n_phi = %.4f rad/s\n', omega_n_phi);
fprintf('  roll_kp = %.4f\n', AP.roll_kp);
fprintf('  roll_kd = %.4f\n', AP.roll_kd);
% Verify CL poles
a_cl = [1, a_phi1 + a_phi2*AP.roll_kd, a_phi2*AP.roll_kp];
fprintf('  CL poles: '); disp(roots(a_cl)')

% -----------------------------------------------------------------------
% COURSE LOOP  (outer lateral)
%   Plant: chi/phi = g/Va/s  (integrator)
%   Controller: PI  phi_c = kp*(chi_c-chi) + ki*int(chi_c-chi)
%   Design: omega_n_chi = omega_n_phi/5 for bandwidth separation
%
%   CL char poly: s^2 + kp*(g/Va)*s + ki*(g/Va) = 0
%   => kp = 2*zeta_chi*omega_n_chi*Va/g
%   => ki = omega_n_chi^2 * Va/g
% -----------------------------------------------------------------------
zeta_chi    = 1.0;    % overdamped to avoid oscillation in heading
omega_n_chi = omega_n_phi / 5.0;

AP.course_kp = 2 * zeta_chi * omega_n_chi * Va_trim / AP.gravity;
AP.course_ki = omega_n_chi^2 * Va_trim / AP.gravity;

fprintf('\n=== Course Loop ===\n');
fprintf('  omega_n_chi = %.4f rad/s\n', omega_n_chi);
fprintf('  course_kp = %.4f\n', AP.course_kp);
fprintf('  course_ki = %.4f\n', AP.course_ki);

% -----------------------------------------------------------------------
% SIDESLIP LOOP  (rudder, coordinates turns)
%   Plant: beta/delta_r = a_beta2 / (s + a_beta1)
%   Controller: PI  delta_r = kp*(0-beta) + ki*int(0-beta)
%
%   CL char poly: s^2 + (a_beta1 + kp*a_beta2)*s + ki*a_beta2 = 0
%   => ki  = omega_n_beta^2 / a_beta2
%   => kp  = (2*zeta_beta*omega_n_beta - a_beta1) / a_beta2
% -----------------------------------------------------------------------
zeta_beta    = 0.707;
omega_n_beta = 0.1;    % well below Dutch roll to avoid interaction

AP.sideslip_kp = (2*zeta_beta*omega_n_beta - a_beta1) / a_beta2;
AP.sideslip_ki = omega_n_beta^2 / a_beta2;

fprintf('\n=== Sideslip Loop ===\n');
fprintf('  omega_n_beta = %.4f rad/s\n', omega_n_beta);
fprintf('  sideslip_kp = %.4f\n', AP.sideslip_kp);
fprintf('  sideslip_ki = %.4f\n', AP.sideslip_ki);

% -----------------------------------------------------------------------
% YAW DAMPER  (improves Dutch roll damping)
%   Dutch roll: zeta_DR = 0.079 (very lightly damped — yaw damper essential)
%   Simple P controller on yaw rate: delta_r = -kp_yd * r
%   tau_r: washout filter time constant (removes steady-state rudder)
% -----------------------------------------------------------------------
AP.yaw_damper_tau_r = 0.05;
AP.yaw_damper_kp    = 0.5;

fprintf('\n=== Yaw Damper ===\n');
fprintf('  tau_r = %.4f s\n', AP.yaw_damper_tau_r);
fprintf('  yaw_damper_kp = %.4f\n', AP.yaw_damper_kp);

% -----------------------------------------------------------------------
% PITCH LOOP  (inner longitudinal)
%   Plant: theta/delta_e = a_theta3 / (s^2 + a_theta1*s + a_theta2)
%   Controller: PD  delta_e = kp*(theta_c-theta) + kd*(0-q)
%
%   CL char poly: s^2 + (a_theta1 + a_theta3*kd)*s + (a_theta2 + a_theta3*kp)
%   => omega_n_theta^2 = a_theta2 + a_theta3*kp
%   => kp = (omega_n_theta^2 - a_theta2) / a_theta3
%   => kd = (2*zeta_theta*omega_n_theta - a_theta1) / a_theta3
% -----------------------------------------------------------------------
zeta_theta    = 0.707;
omega_n_theta = sqrt(abs(a_theta3) * AP.delta_e_max / e_theta_max);

AP.pitch_kp = (omega_n_theta^2 - a_theta2) / a_theta3;
AP.pitch_kd = (2*zeta_theta*omega_n_theta - a_theta1) / a_theta3;

% Closed-loop DC gain of theta/theta_c (used in altitude loop)
K_theta_DC = AP.pitch_kp * a_theta3 / omega_n_theta^2;

fprintf('\n=== Pitch Loop ===\n');
fprintf('  omega_n_theta = %.4f rad/s\n', omega_n_theta);
fprintf('  pitch_kp = %.4f\n', AP.pitch_kp);
fprintf('  pitch_kd = %.4f\n', AP.pitch_kd);
fprintf('  K_theta_DC (CL) = %.4f\n', K_theta_DC);
% Verify CL poles
a_cl_th = [1, a_theta1 + a_theta3*AP.pitch_kd, a_theta2 + a_theta3*AP.pitch_kp];
fprintf('  CL poles: '); disp(roots(a_cl_th)')

% -----------------------------------------------------------------------
% ALTITUDE LOOP  (outer longitudinal)
%   Effective plant (with pitch loop closed): h/theta_c = K_theta_DC*Va/s
%   Controller: PI  theta_c = kp*(h_c-h) + ki*int(h_c-h)
%
%   CL char poly: s^2 + kp*K_DC*Va*s + ki*K_DC*Va = 0
%   => kp = 2*zeta_h*omega_n_h / (K_theta_DC * Va)
%   => ki = omega_n_h^2 / (K_theta_DC * Va)
%
%   altitude_zone: if |h_c - h| > zone, pitch controls altitude aggressively
% -----------------------------------------------------------------------
zeta_h    = 0.9;
omega_n_h = omega_n_theta / 10.0;   % well below pitch bandwidth

AP.altitude_kp   = 2*zeta_h*omega_n_h / (K_theta_DC * Va_trim);
AP.altitude_ki   = omega_n_h^2 / (K_theta_DC * Va_trim);
AP.altitude_zone = 30;    % [m] zone for altitude-hold switching

fprintf('\n=== Altitude Loop ===\n');
fprintf('  omega_n_h = %.4f rad/s\n', omega_n_h);
fprintf('  altitude_kp = %.6f\n', AP.altitude_kp);
fprintf('  altitude_ki = %.6f\n', AP.altitude_ki);
fprintf('  altitude_zone = %.1f m\n', AP.altitude_zone);

% -----------------------------------------------------------------------
% AIRSPEED LOOP (throttle)
%   Plant: Va/delta_t = a_V2 / (s + a_V1)
%   Controller: PI  delta_t = kp*(Va_c-Va) + ki*int(Va_c-Va)
%
%   CL char poly: s^2 + (a_V1 + kp*a_V2)*s + ki*a_V2 = 0
%   => kp = (2*zeta_Vt*omega_n_Vt - a_V1) / a_V2
%   => ki = omega_n_Vt^2 / a_V2
% -----------------------------------------------------------------------
zeta_Vt    = 0.9;
omega_n_Vt = omega_n_theta / 10.0;   % same bandwidth as altitude

AP.airspeed_throttle_kp = (2*zeta_Vt*omega_n_Vt - a_V1) / a_V2;
AP.airspeed_throttle_ki = omega_n_Vt^2 / a_V2;

% Store trim throttle for integrator initialisation
AP.delta_t_trim = u_trim(4);

fprintf('\n=== Airspeed/Throttle Loop ===\n');
fprintf('  omega_n_Vt = %.4f rad/s\n', omega_n_Vt);
fprintf('  airspeed_throttle_kp = %.4f\n', AP.airspeed_throttle_kp);
fprintf('  airspeed_throttle_ki = %.4f\n', AP.airspeed_throttle_ki);

% -----------------------------------------------------------------------
% Save
% -----------------------------------------------------------------------
save('autopilot_gains.mat', 'AP');
fprintf('\nSaved to autopilot_gains.mat\n');