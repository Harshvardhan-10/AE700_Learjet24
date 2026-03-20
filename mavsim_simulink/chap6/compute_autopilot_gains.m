% compute_autopilot_gains.m
%   Designs all autopilot gains using B&M Chapter 6 methodology.
%   FIX: Surface deflection limit = 40 deg (per project spec, not 20 deg).
%   NEW: Adds airspeed_pitch_kp and airspeed_pitch_ki for Va control via pitch.

addpath('../chap3')
addpath('../chap4')
addpath('../chap5')
load('transfer_function_coef.mat')
addpath('../parameters')
simulation_parameters
load('trim_results.mat')

AP.gravity = 9.81;
AP.sigma   = 0.05;
AP.Va0     = Va_trim;
AP.Ts      = SIM.ts_simulation;

% -----------------------------------------------------------------------
% Control surface limits
%   Project spec: bound = 40 degrees
% -----------------------------------------------------------------------
AP.delta_a_max = 40 * pi/180;   % FIX: project requires 40 deg (was 20)
AP.delta_e_max = 40 * pi/180;
AP.delta_r_max = 40 * pi/180;
AP.phi_c_max   = 45 * pi/180;   % max commanded roll [rad]
AP.theta_c_max = 30 * pi/180;   % max commanded pitch [rad]

e_phi_max   = 20 * pi/180;      % max roll  tracking error for gain design
e_theta_max = 20 * pi/180;      % max pitch tracking error for gain design

% -----------------------------------------------------------------------
% ROLL LOOP
%   Plant: phi/delta_a = a_phi2 / (s^2 + a_phi1*s)
%   CL: s^2 + (a_phi1 + a_phi2*kd)*s + a_phi2*kp = 0
% -----------------------------------------------------------------------
zeta_phi    = 0.707;
omega_n_phi = sqrt(abs(a_phi2) * AP.delta_a_max / e_phi_max);

AP.roll_kp  = sign(a_phi2) * AP.delta_a_max / e_phi_max;
AP.roll_kd  = (2*zeta_phi*omega_n_phi - a_phi1) / a_phi2;

fprintf('=== Roll Loop ===\n');
fprintf('  omega_n_phi = %.4f rad/s\n', omega_n_phi);
fprintf('  roll_kp = %.4f\n', AP.roll_kp);
fprintf('  roll_kd = %.4f\n', AP.roll_kd);
a_cl_phi = [1, a_phi1+a_phi2*AP.roll_kd, a_phi2*AP.roll_kp];
fprintf('  CL poles: '); disp(roots(a_cl_phi)')
r = roots(a_cl_phi);
if real(r(1)) >= 0
    warning('Roll CL has unstable pole!');
end

% -----------------------------------------------------------------------
% COURSE LOOP
%   Plant (chi/phi): g/Va/s
%   omega_n_chi = omega_n_phi/10  (bandwidth separation)
% -----------------------------------------------------------------------
zeta_chi    = 1.0;
omega_n_chi = omega_n_phi / 10.0;

AP.course_kp = 2 * zeta_chi * omega_n_chi * Va_trim / AP.gravity;
AP.course_ki = omega_n_chi^2 * Va_trim / AP.gravity;

fprintf('\n=== Course Loop ===\n');
fprintf('  omega_n_chi = %.4f rad/s\n', omega_n_chi);
fprintf('  course_kp = %.4f\n', AP.course_kp);
fprintf('  course_ki = %.4f\n', AP.course_ki);

% -----------------------------------------------------------------------
% SIDESLIP LOOP
% -----------------------------------------------------------------------
zeta_beta    = 0.707;
omega_n_beta = 0.1;

AP.sideslip_kp = (2*zeta_beta*omega_n_beta - a_beta1) / a_beta2;
AP.sideslip_ki = omega_n_beta^2 / a_beta2;

fprintf('\n=== Sideslip Loop ===\n');
fprintf('  sideslip_kp = %.4f\n', AP.sideslip_kp);
fprintf('  sideslip_ki = %.4f\n', AP.sideslip_ki);

% -----------------------------------------------------------------------
% YAW DAMPER
% -----------------------------------------------------------------------
AP.yaw_damper_tau_r = 0.5;
AP.yaw_damper_kp    = 2;

fprintf('\n=== Yaw Damper ===\n');
fprintf('  tau_r = %.4f\n', AP.yaw_damper_tau_r);
fprintf('  kp    = %.4f\n', AP.yaw_damper_kp);

% -----------------------------------------------------------------------
% PITCH LOOP
%   CL: s^2 + (a_theta1 + a_theta3*kd)*s + (a_theta2 + a_theta3*kp) = 0
% -----------------------------------------------------------------------
zeta_theta    = 0.707;
omega_n_theta = sqrt(abs(a_theta3) * AP.delta_e_max / e_theta_max);

AP.pitch_kp = (omega_n_theta^2 - a_theta2) / a_theta3;
AP.pitch_kd = (2*zeta_theta*omega_n_theta - a_theta1) / a_theta3;

K_theta_DC = AP.pitch_kp * a_theta3 / omega_n_theta^2;

fprintf('\n=== Pitch Loop ===\n');
fprintf('  omega_n_theta = %.4f rad/s\n', omega_n_theta);
fprintf('  pitch_kp = %.4f\n', AP.pitch_kp);
fprintf('  pitch_kd = %.4f\n', AP.pitch_kd);
fprintf('  K_theta_DC = %.4f\n', K_theta_DC);
a_cl_th = [1, a_theta1+a_theta3*AP.pitch_kd, a_theta2+a_theta3*AP.pitch_kp];
fprintf('  CL poles: '); disp(roots(a_cl_th)')

% -----------------------------------------------------------------------
% ALTITUDE LOOP
%   omega_n_h = omega_n_theta/10
% -----------------------------------------------------------------------
zeta_h    = 0.9;
omega_n_h = omega_n_theta / 10.0;

AP.altitude_kp   = 2*zeta_h*omega_n_h / (K_theta_DC * Va_trim);
AP.altitude_ki   = omega_n_h^2 / (K_theta_DC * Va_trim);
AP.altitude_zone = 30;

fprintf('\n=== Altitude Loop ===\n');
fprintf('  omega_n_h = %.4f rad/s\n', omega_n_h);
fprintf('  altitude_kp = %.6f\n', AP.altitude_kp);
fprintf('  altitude_ki = %.6f\n', AP.altitude_ki);

% -----------------------------------------------------------------------
% AIRSPEED WITH THROTTLE
%   Plant: Va/delta_t = a_V2/(s + a_V1)
% -----------------------------------------------------------------------
zeta_Vt    = 0.9;
omega_n_Vt = omega_n_theta / 10.0;

AP.airspeed_throttle_kp = (2*zeta_Vt*omega_n_Vt - a_V1) / a_V2;
AP.airspeed_throttle_ki = omega_n_Vt^2 / a_V2;
AP.delta_t_trim         = u_trim(4);

fprintf('\n=== Airspeed/Throttle Loop ===\n');
fprintf('  omega_n_Vt = %.4f rad/s\n', omega_n_Vt);
fprintf('  airspeed_throttle_kp = %.4f\n', AP.airspeed_throttle_kp);
fprintf('  airspeed_throttle_ki = %.4f\n', AP.airspeed_throttle_ki);

% -----------------------------------------------------------------------
% AIRSPEED WITH PITCH  (NEW - required by project Section 5)
%   Plant: Va/theta_c = T_Va_theta = -a_V3/(s + a_V1)
%   Note: plant has NEGATIVE sign (pitching up slows the aircraft).
%   So kp and ki are negative, which pitches down when Va is too low.
%
%   CL char poly: s^2 + (a_V1 + kp*(-a_V3))*s + ki*(-a_V3) = 0
%   => kp = (2*zeta*omega_n - a_V1) / (-a_V3)
%   => ki = omega_n^2 / (-a_V3)
% -----------------------------------------------------------------------
zeta_Va_pitch    = 0.9;
omega_n_Va_pitch = omega_n_theta / 10.0;   % same bandwidth as throttle loop

AP.airspeed_pitch_kp = (2*zeta_Va_pitch*omega_n_Va_pitch - a_V1) / (-a_V3);
AP.airspeed_pitch_ki = omega_n_Va_pitch^2 / (-a_V3);

fprintf('\n=== Airspeed/Pitch Loop (NEW) ===\n');
fprintf('  omega_n_Va_pitch = %.4f rad/s\n', omega_n_Va_pitch);
fprintf('  airspeed_pitch_kp = %.4f  (negative = pitch down when slow)\n', AP.airspeed_pitch_kp);
fprintf('  airspeed_pitch_ki = %.4f\n', AP.airspeed_pitch_ki);
% Verify CL stability
a_cl_Vp = [1, a_V1 + AP.airspeed_pitch_kp*(-a_V3), AP.airspeed_pitch_ki*(-a_V3)];
fprintf('  CL poles: '); disp(roots(a_cl_Vp)')
if any(real(roots(a_cl_Vp)) >= 0)
    warning('Airspeed/pitch CL has unstable pole — check signs of a_V3.');
end

fprintf('\n=== Closed-loop Poles Summary ===\n');

loops = {
    'Roll',              [1, a_phi1+a_phi2*AP.roll_kd,                    a_phi2*AP.roll_kp];
    'Course',            [1, AP.course_kp*(AP.gravity/Va_trim),            AP.course_ki*(AP.gravity/Va_trim)];
    'Pitch',             [1, a_theta1+a_theta3*AP.pitch_kd,                a_theta2+a_theta3*AP.pitch_kp];
    'Altitude',          [1, AP.altitude_kp*K_theta_DC*Va_trim,            AP.altitude_ki*K_theta_DC*Va_trim];
    'Airspeed/Throttle', [1, a_V1+AP.airspeed_throttle_kp*a_V2,            AP.airspeed_throttle_ki*a_V2];
    'Airspeed/Pitch',    [1, a_V1+AP.airspeed_pitch_kp*(-a_V3),            AP.airspeed_pitch_ki*(-a_V3)];
};

for k = 1:size(loops,1)
    p = roots(loops{k,2});
    if all(real(p) < 0), status = 'stable'; else, status = 'UNSTABLE'; end
    fprintf('  %-20s : ', loops{k,1});
    for j = 1:length(p)
        fprintf('%+.4f%+.4fi   ', real(p(j)), imag(p(j)));
    end
    fprintf('[%s]\n', status);
end

% -----------------------------------------------------------------------
% Save
% -----------------------------------------------------------------------
save('autopilot_gains.mat', 'AP');
fprintf('\nSaved autopilot_gains.mat\n');

% Bandwidth summary
fprintf('\n=== Bandwidth Summary ===\n');
fprintf('  omega_n_phi   = %.4f rad/s  (roll — fastest)\n',   omega_n_phi);
fprintf('  omega_n_theta = %.4f rad/s  (pitch)\n',             omega_n_theta);
fprintf('  omega_n_chi   = %.4f rad/s  (course  = phi/10)\n',  omega_n_chi);
fprintf('  omega_n_Vt    = %.4f rad/s  (airspeed/throttle)\n', omega_n_Vt);
fprintf('  omega_n_h     = %.4f rad/s  (altitude = theta/10)\n', omega_n_h);

% Restore MAV ICs
MAV.pn0=x_trim(1); MAV.pe0=x_trim(2); MAV.pd0=x_trim(3);
MAV.u0=x_trim(4);  MAV.v0=x_trim(5);  MAV.w0=x_trim(6);
MAV.phi0=x_trim(7); MAV.theta0=x_trim(8); MAV.psi0=x_trim(9);
MAV.p0=x_trim(10); MAV.q0=x_trim(11); MAV.r0=x_trim(12);