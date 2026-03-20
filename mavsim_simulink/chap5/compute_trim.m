% compute_trim.m
%   Computes trim conditions for STRAIGHT WINGS-LEVEL FLIGHT using
%   Simulink's trim() function with mavsim_trim_2023b.slx.
%
%   Aircraft: Learjet 24  (aerosonde_parameters.m)
%
%   Trim constraints (straight level flight):
%     phi   = 0        (wings level)
%     beta  = 0        (no sideslip)
%     p=q=r = 0        (no body rates)
%     gamma = 0        (level, no climb)
%     R     = Inf      (straight, no turn)
%
%   Free variables solved by trim():
%     delta_e, delta_t  (elevator and throttle)
%     u, w              (body velocities -> alpha)
%
%   Held fixed:
%     delta_a = 0,  delta_r = 0   (iu = [2; 3])
%     Va = desired airspeed        (iy = [1])
%     beta = 0                     (iy = [3])
%
%   Run order:
%     1. compute_trim.m          <- this file
%     2. compute_ss_model.m
%     3. compute_tf_model.m
%     4. compute_autopilot_gains.m  (in chap6/)

addpath('../parameters')
addpath('../tools')
addpath('../chap3')
addpath('../chap4')
aerosonde_parameters      % loads MAV struct

% -----------------------------------------------------------------------
% USER SETTINGS — change these if you want a different trim condition
% -----------------------------------------------------------------------
gamma = 0 * pi/180;   % flight-path angle [rad]  (0 = level flight)
Va    = 170;           % desired airspeed   [m/s]  (Learjet 24 cruise)

% n     = 1.2;    % load factor
n = 1;

% ADD THESE derived quantities:
g        = MAV.gravity;
phi_trim = acos(1/n);                    % bank angle from load factor

if abs(phi_trim) < 1e-6    % essentially zero
    R = Inf;
else
    R = Va^2 / (g * tan(phi_trim));
end

psi_dot = Va * cos(gamma) / R;   % = g*tan(phi)/Va

% Body rates for coordinated level turn
p_trim  = 0;                          % level turn, gamma=0
q_trim  = psi_dot * sin(phi_trim);
r_trim  = psi_dot * cos(phi_trim);

% 1. Set initial conditions (State vector guess)
% x = [pn; pe; pd; u; v; w; phi; theta; psi; p; q; r]
%x0 = [0; 0; -200; Va; 0; 0; 0; gamma; 0; 0; 0; 0];     %straight flight
x0 = [0; 0; MAV.pd0; Va; 0; 0; phi_trim; gamma; 0; p_trim; q_trim; r_trim];

% specify which states to hold equal to the initial conditions
ix = [];

% 2. Specify initial inputs guess
% u = [delta_e; delta_a; delta_r; delta_t]
u0 = [
    0;   % 1 - delta_e
    0;   % 2 - delta_a
    0;   % 3 - delta_r
    0.5; % 4 - delta_t
];
% specify which inputs to hold constant (force aileron and rudder to 0 for straight flight)
iu = [2; 3];   %straight flight
% iu = [3];       %taking a turn

% -----------------------------------------------------------------------
% 3. Output constraints
%    Outputs from mavsim_trim: y = [Va; alpha; beta]
%
%    Hold Va = desired airspeed, and beta = 0
% -----------------------------------------------------------------------
y0 = [Va; 0; 0];
iy = [1; 3];    % fix Va (index 1) and beta (index 3)

% -----------------------------------------------------------------------
% 4. Derivative constraints
%    dx = [pn_dot; pe_dot; pd_dot; u_dot; v_dot; w_dot;
%           phi_dot; theta_dot; psi_dot; p_dot; q_dot; r_dot]
%
%    Straight level flight: all rates are zero except
%      pd_dot = -Va*sin(gamma)  (= 0 for gamma=0, but kept general)
%      psi_dot = 0              (no turn)
% -----------------------------------------------------------------------

% CHANGE dx0:
dx0 = [0; 0; -Va*sin(gamma); 0; 0; 0; 0; 0; psi_dot; 0; 0; 0];
%                                                ^^^^^^^ was 0 for straight flight

%dx0 = [0; 0; -Va*sin(gamma); 0; 0; 0; 0; 0; 0; 0; 0; 0];       %straight flight

if R ~= Inf
    dx0(9) = Va*cos(gamma)/R; % 9 - psidot for orbiting
end 

% specify which derivatives to hold constant in trim algorithm
% Holding pd_dot through r_dot constant
idx = [3; 4; 5; 6; 7; 8; 9; 10; 11; 12];

% 5. Compute trim conditions
fprintf('Running Simulink trim calculation for mavsim_trim...\n');
[x_trim, u_trim, y_trim, dx_trim] = trim('mavsim_trim', x0, u0, y0, ix, iu, iy, dx0, idx);

% [x_trim, u_trim, y_trim, dx_trim] = trim('mavsim_trim_2023b', ...
                                          % x0, u0, y0, ix, iu, iy, dx0, idx);

% -----------------------------------------------------------------------
% 6. Convergence check
%    Residual of held derivatives should be near zero
% -----------------------------------------------------------------------
trim_error = norm(dx_trim(3:end) - dx0(3:end));
fprintf('\nTrim derivative residual norm: %.4e\n', trim_error);
if trim_error > 1e-4
    warning(['Trim may not have converged (residual = %.4e). ' ...
             'Try adjusting Va or initial guess x0.'], trim_error);
end

% -----------------------------------------------------------------------
% 7. Extract and display results
% -----------------------------------------------------------------------
Va_trim    = y_trim(1);
alpha_trim = y_trim(2);
beta_trim  = y_trim(3);
phi_trim   = x_trim(7);
theta_trim = x_trim(8);
psi_trim   = x_trim(9);

delta_e_trim = u_trim(1);
delta_a_trim = u_trim(2);
delta_r_trim = u_trim(3);
delta_t_trim = u_trim(4);

fprintf('\n========================================\n');
fprintf(' Trim Results  Va=%.1f m/s  gamma=%.1f deg\n', Va, gamma*180/pi);
fprintf('========================================\n');
fprintf('Va      = %.4f m/s\n',  Va_trim);
fprintf('alpha   = %.4f deg\n',  alpha_trim  * 180/pi);
fprintf('beta    = %.4f deg\n',  beta_trim   * 180/pi);
fprintf('phi     = %.4f deg\n',  phi_trim    * 180/pi);   % should be ~0
fprintf('theta   = %.4f deg\n',  theta_trim  * 180/pi);
fprintf('delta_e = %.4f deg\n',  delta_e_trim * 180/pi);
fprintf('delta_a = %.4f deg\n',  delta_a_trim * 180/pi);  % should be 0
fprintf('delta_r = %.4f deg\n',  delta_r_trim * 180/pi);  % should be 0
fprintf('delta_t = %.4f\n',      delta_t_trim);
fprintf('----------------------------------------\n');
fprintf('Sanity checks:\n');
fprintf('  phi   should be ~0 deg  -> %.4f deg\n', phi_trim*180/pi);
fprintf('  beta  should be ~0 deg  -> %.4f deg\n', beta_trim*180/pi);
fprintf('  p,q,r should be ~0      -> [%.2e, %.2e, %.2e]\n', ...
        x_trim(10), x_trim(11), x_trim(12));
fprintf('========================================\n');

% -----------------------------------------------------------------------
% 8. Update MAV initial conditions to trim state
%    (so the simulation starts already in trim)
% -----------------------------------------------------------------------
MAV.pn0    = x_trim(1);
MAV.pe0    = x_trim(2);
MAV.pd0    = x_trim(3);
MAV.u0     = x_trim(4);
MAV.v0     = x_trim(5);
MAV.w0     = x_trim(6);
MAV.phi0   = x_trim(7);
MAV.theta0 = x_trim(8);
MAV.psi0   = x_trim(9);
MAV.p0     = x_trim(10);
MAV.q0     = x_trim(11);
MAV.r0     = x_trim(12);

% -----------------------------------------------------------------------
% 9. Save
% -----------------------------------------------------------------------
save('trim_results.mat', 'x_trim', 'u_trim', 'y_trim', ...
     'Va_trim', 'alpha_trim', 'theta_trim', 'phi_trim', 'psi_trim');
fprintf('Saved trim_results.mat\n');