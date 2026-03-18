% % compute_trim.m
% %   Computes trim conditions using fsolve().
% %   Works with Euler-based mav_dynamics (12 states).
% %
% %   Fixed trim (wings-level straight flight):
% %     phi=0, beta=0, p=q=r=0, v=0, delta_a=0, delta_r=0
% %   Solved by fsolve: [u, w, delta_e, delta_t]
% 
% addpath('../parameters')
% addpath('../tools')
% addpath('../chap3')
% addpath('../chap4')
% aerosonde_parameters
% 
% gamma  = 0 * pi/180;   % flight-path angle [rad]
% Va_des = 80;          % desired airspeed [m/s]
% 
%     function residual = trim_equations(z, Va_des, gamma, MAV)
%         u = z(1);  w = z(2);  delta_e = z(3);  delta_t = z(4);
%         phi = 0;  q_nd = 0;  alphadot_nd = 0;
% 
%         Va    = sqrt(u^2 + w^2);
%         alpha = atan2(w, u);
%         theta = alpha + gamma;
% 
%         CL = MAV.CL_0 + MAV.CL_alpha*alpha + MAV.CL_adot*alphadot_nd ...
%            + MAV.CL_q*q_nd + MAV.CL_de*delta_e;
%         CD = MAV.CD_0 + MAV.CD_alpha*alpha + MAV.CD_de*delta_e;
%         Cm = MAV.Cm_0 + MAV.Cm_alpha*alpha + MAV.Cm_adot*alphadot_nd ...
%            + MAV.Cm_q*q_nd + MAV.Cm_de*delta_e;
% 
%         qbar = 0.5 * MAV.rho * Va^2;
%         ca = cos(alpha);  sa = sin(alpha);
%         F_lift = qbar * MAV.S_wing * CL;
%         F_drag = qbar * MAV.S_wing * CD;
% 
%         fx = -MAV.mass*MAV.gravity*sin(theta) ...
%              + (-ca*F_drag + sa*F_lift) + delta_t*MAV.T_max;
%         fz =  MAV.mass*MAV.gravity*cos(theta)*cos(phi) ...
%              + (-sa*F_drag - ca*F_lift);
%         My = qbar * MAV.S_wing * MAV.c * Cm;
% 
%         residual = [fx/MAV.mass; fz/MAV.mass; My/MAV.Jy; Va-Va_des];
%     end
% 
% z0 = [Va_des; 0; 0; 0.3];
% options = optimoptions('fsolve','Display','iter', ...
%     'TolFun',1e-12,'TolX',1e-12,'MaxFunEvals',2000,'MaxIter',500);
% 
% [z_sol, fval, exitflag] = fsolve(@(z) trim_equations(z,Va_des,gamma,MAV), z0, options);
% 
% fprintf('\nfsolve residual norm: %.4e\n', norm(fval));
% if exitflag <= 0, warning('fsolve did not converge (exitflag=%d).', exitflag); end
% 
% u_sol       = z_sol(1);
% w_sol       = z_sol(2);
% delta_e_sol = z_sol(3);
% delta_t_sol = max(0, min(1, z_sol(4)));
% 
% Va_trim    = sqrt(u_sol^2 + w_sol^2);
% alpha_trim = atan2(w_sol, u_sol);
% theta_trim = alpha_trim + gamma;
% phi_trim   = 0;
% beta_trim  = 0;
% psi_trim   = 0;
% 
% % 12-element Euler state vector (no quaternion)
% x_trim = [0; 0; -200; u_sol; 0; w_sol;
%           phi_trim; theta_trim; psi_trim; 0; 0; 0];
% 
% u_trim = [delta_e_sol; 0; 0; delta_t_sol];
% y_trim = [Va_trim; alpha_trim; beta_trim];
% 
% % Update MAV initial conditions
% MAV.pn0 = 0;   MAV.pe0 = 0;   MAV.pd0 = -200;
% MAV.u0  = u_sol;  MAV.v0 = 0;  MAV.w0 = w_sol;
% MAV.phi0 = phi_trim;  MAV.theta0 = theta_trim;  MAV.psi0 = psi_trim;
% MAV.p0 = 0;  MAV.q0 = 0;  MAV.r0 = 0;
% 
% fprintf('\n--- Trim Results  Va=%.1f m/s  gamma=%.1f deg ---\n', Va_des, gamma*180/pi);
% fprintf('Va      = %.4f m/s\n',  Va_trim);
% fprintf('alpha   = %.4f deg\n',  alpha_trim  * 180/pi);
% fprintf('beta    = %.4f deg\n',  beta_trim   * 180/pi);
% fprintf('phi     = %.4f deg\n',  phi_trim    * 180/pi);
% fprintf('theta   = %.4f deg\n',  theta_trim  * 180/pi);
% fprintf('delta_e = %.4f deg\n',  delta_e_sol * 180/pi);
% fprintf('delta_a = %.4f deg\n',  0.0);
% fprintf('delta_r = %.4f deg\n',  0.0);
% fprintf('delta_t = %.4f\n',      delta_t_sol);
% 
% save('trim_results.mat', 'x_trim','u_trim','y_trim', ...
%      'Va_trim','alpha_trim','theta_trim','phi_trim','psi_trim');
% fprintf('Saved to trim_results.mat\n');

% compute_trim.m
%   Computes trim conditions using Simulink's trim() function.
%   Works with Euler-based mav_dynamics (12 states) via 'mavsim_trim.slx'.
%
%   Fixed trim (wings-level straight flight):
%     phi=0, beta=0, p=q=r=0, v=0, delta_a=0, delta_r=0

addpath('../parameters')
addpath('../tools')
addpath('../chap3')
addpath('../chap4')
aerosonde_parameters

% Trim definition parameters
gamma = 0 * pi/180;   % desired flight path angle [rad]
Va    = 170;           % desired airspeed [m/s]
%R     = Inf;          % desired radius (m) - use Inf for straight flight

n     = 1.2;    % load factor

% ADD THESE derived quantities:
g        = MAV.gravity;
phi_trim = acos(1/n);                    % bank angle from load factor
R        = Va^2 / (g * tan(phi_trim));   % turn radius [m], taking a turn


% 1. Set initial conditions (State vector guess)
% x = [pn; pe; pd; u; v; w; phi; theta; psi; p; q; r]
%x0 = [0; 0; -200; Va; 0; 0; 0; gamma; 0; 0; 0; 0];     %straight flight
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
%iu = [2; 3];   %straight flight
iu = [3];       %taking a turn

% 3. Define constant outputs
% y = [Va; alpha; beta] from the selector block in the SLX model
y0 = [
    Va;  % 1 - Va
    0;   % 2 - alpha
    0;   % 3 - beta
];
% specify which outputs to hold constant (hold Va and beta)
iy = [1; 3];

% 4. Define constant derivatives
% dx = [pn_dot; pe_dot; pd_dot; u_dot; v_dot; w_dot; phi_dot; theta_dot; psi_dot; p_dot; q_dot; r_dot]

% Turn rate
psi_dot = Va * cos(gamma) / R;   % = g*tan(phi)/Va

% Body rates for coordinated level turn
p_trim  = 0;                          % level turn, gamma=0
q_trim  = psi_dot * sin(phi_trim);
r_trim  = psi_dot * cos(phi_trim);

% CHANGE dx0:
dx0 = [0; 0; -Va*sin(gamma); 0; 0; 0; 0; 0; psi_dot; 0; 0; 0];
%                                                ^^^^^^^ was 0 for straight flight

% Also update x0 body rates to match:
x0 = [0; 0; -200; Va; 0; 0; phi_trim; gamma; 0; p_trim; q_trim; r_trim];    %taking a turn

%dx0 = [0; 0; -Va*sin(gamma); 0; 0; 0; 0; 0; 0; 0; 0; 0];       %straight flight

if R ~= Inf
    dx0(9) = Va*cos(gamma)/R; % 9 - psidot for orbiting
end 

% specify which derivatives to hold constant in trim algorithm
% Holding pd_dot through r_dot constant
idx = [3; 4; 5; 6; 7; 8; 9; 10; 11; 12];

% 5. Compute trim conditions
fprintf('Running Simulink trim calculation for mavsim_trim...\n');
[x_trim, u_trim, y_trim, dx_trim] = trim('mavsim_trim_2023b', x0, u0, y0, ix, iu, iy, dx0, idx);

% Check to make sure that the linearization worked (norm should be very small)
trim_error = norm(dx_trim(3:end) - dx0(3:end));
fprintf('\nTrim derivative error norm: %.4e\n', trim_error);
if trim_error > 1e-4
    warning('Trim algorithm may not have converged properly. Check parameters or initial guesses.');
end

% 6. Extract trim variables for display and saving
u_sol       = x_trim(4);
w_sol       = x_trim(6);
delta_e_sol = u_trim(1);
delta_a_sol = u_trim(2);
delta_r_sol = u_trim(3);
delta_t_sol = u_trim(4);

Va_trim    = y_trim(1);
alpha_trim = y_trim(2);
beta_trim  = y_trim(3);
phi_trim   = x_trim(7);
theta_trim = x_trim(8);
psi_trim   = x_trim(9);

% 7. Update MAV initial conditions
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

% 8. Display Results
fprintf('\n--- Trim Results  Va=%.1f m/s  gamma=%.1f deg ---\n', Va, gamma*180/pi);
fprintf('Va      = %.4f m/s\n', Va_trim);
fprintf('alpha   = %.4f deg\n', alpha_trim * 180/pi);
fprintf('beta    = %.4f deg\n', beta_trim  * 180/pi);
fprintf('phi     = %.4f deg\n', phi_trim   * 180/pi);
fprintf('theta   = %.4f deg\n', theta_trim * 180/pi);
fprintf('delta_e = %.4f deg\n', delta_e_sol * 180/pi);
fprintf('delta_a = %.4f deg\n', delta_a_sol * 180/pi);
fprintf('delta_r = %.4f deg\n', delta_r_sol * 180/pi);
fprintf('delta_t = %.4f\n',     delta_t_sol);

% 9. Save to mat file
save('trim_results.mat', 'x_trim', 'u_trim', 'y_trim', ...
     'Va_trim', 'alpha_trim', 'theta_trim', 'phi_trim', 'psi_trim');
fprintf('Saved to trim_results.mat\n');