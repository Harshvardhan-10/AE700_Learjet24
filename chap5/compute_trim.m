% compute_trim.m
%   Computes trim conditions for the Learjet 24.
%   SECTION 4(iv): Straight wings-level flight  with alpha = 5-7 deg
%   SECTION 4(v):  Constant coordinated turn    with n = 1.2
%
%   Run order:
%     1. compute_trim.m
%     2. compute_ss_model.m
%     3. compute_tf_model.m
%     4. compute_autopilot_gains.m

addpath('../parameters')
addpath('../tools')
addpath('../chap3')
addpath('../chap4')
aerosonde_parameters

% =========================================================================
% CASE 1 : Straight-and-level flight
%   Va = 80 m/s  ->  alpha ≈ 5.5 deg  (satisfies 5-7 deg requirement)
%   gamma = 0, R = Inf, n = 1
% =========================================================================
fprintf('\n======================================================\n');
fprintf(' CASE 1 : Straight-and-level  Va=80 m/s  gamma=0\n');
fprintf('======================================================\n');

gamma = 0;
Va    = 80;       
n     = 1;        % load factor (wings-level)

g        = MAV.gravity;
phi_trim = acos(1/n);      % = 0 for n=1
R        = Inf;
psi_dot  = 0;
p_trim   = 0;  q_trim = 0;  r_trim = 0;

x0 = [0; 0; MAV.pd0; Va; 0; 0; phi_trim; gamma; 0; p_trim; q_trim; r_trim];
ix = [];

u0 = [0; 0; 0; 0.5];
iu = [2; 3];               % hold delta_a = 0, delta_r = 0 for straight flight

y0 = [Va; 0; 0];
iy = [1; 3];               % fix Va and beta=0 for straight flight

dx0 = [0; 0; -Va*sin(gamma); 0; 0; 0; 0; 0; psi_dot; 0; 0; 0];
idx = [3; 4; 5; 6; 7; 8; 9; 10; 11; 12];

fprintf('Running trim (straight flight)...\n');
[x_trim, u_trim, y_trim, dx_trim] = trim('mavsim_trim', x0, u0, y0, ix, iu, iy, dx0, idx);

trim_error = norm(dx_trim(3:end) - dx0(3:end));
fprintf('Trim residual: %.4e\n', trim_error);
if trim_error > 1e-4
    warning('Trim may not have converged (residual = %.4e).', trim_error);
end

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

% Check alpha requirement
if alpha_trim*180/pi < 5 || alpha_trim*180/pi > 7
    warning('alpha = %.2f deg is outside 5-7 deg. Adjust Va to get alpha between 5-7 deg.', ...
            alpha_trim*180/pi);
end

% Save straight-level trim
save('trim_results.mat', 'x_trim','u_trim','y_trim', ...
     'Va_trim','alpha_trim','theta_trim','phi_trim','psi_trim');

% Update MAV ICs
MAV.pn0=x_trim(1); MAV.pe0=x_trim(2); MAV.pd0=x_trim(3);
MAV.u0=x_trim(4);  MAV.v0=x_trim(5);  MAV.w0=x_trim(6);
MAV.phi0=x_trim(7); MAV.theta0=x_trim(8); MAV.psi0=x_trim(9);
MAV.p0=x_trim(10); MAV.q0=x_trim(11); MAV.r0=x_trim(12);

fprintf('\nSaved trim_results.mat (straight-level)\n');

% =========================================================================
% CASE 2 : Coordinated constant-altitude turn
%   n = 1.2, target CL = 0.7-1.0
%   At Va=80 m/s, n=1.2: CL = 2*n*m*g/(rho*S*Va^2) ≈ 0.83 (in range)
%   phi_trim = acos(1/1.2) = 33.56 deg
% =========================================================================
fprintf('\n======================================================\n');
fprintf(' CASE 2 : Coordinated turn  n=1.2  Va=80 m/s\n');
fprintf('======================================================\n');

aerosonde_parameters          % reload clean MAV struct

n_turn   = 1.2;
Va_turn  = 80;
gamma_t  = 0;

phi_turn = acos(1/n_turn);              % ≈ 33.56 deg
R_turn   = Va_turn^2 / (g * tan(phi_turn));
psi_dot_t = Va_turn * cos(gamma_t) / R_turn;
q_turn   = psi_dot_t * sin(phi_turn);
r_turn   = psi_dot_t * cos(phi_turn);

% Verify CL is in required range
rho = MAV.rho;  S = MAV.S_wing;  m = MAV.mass;
CL_check = 2*n_turn*m*g / (rho * Va_turn^2 * S);
fprintf('Expected CL at trim = %.4f  (must be 0.7-1.0)\n', CL_check);

x0t = [0; 0; MAV.pd0; Va_turn; 0; 0; phi_turn; gamma_t; 0; 0; q_turn; r_turn];
u0t = [0; 0; 0; 0.5];
iut = [3];            % only hold delta_r = 0 (delta_a is free for the bank)
y0t = [Va_turn; 0; 0];
iyt = [1; 3];

dx0t = [0; 0; -Va_turn*sin(gamma_t); 0; 0; 0; 0; 0; psi_dot_t; 0; 0; 0];
idxt = [3; 4; 5; 6; 7; 8; 9; 10; 11; 12];

fprintf('Running trim (turn)...\n');
[x_trim_t, u_trim_t, y_trim_t, dx_trim_t] = trim('mavsim_trim', x0t, u0t, y0t, [], iut, iyt, dx0t, idxt);

trim_error_t = norm(dx_trim_t(3:end) - dx0t(3:end));
fprintf('Turn trim residual: %.4e\n', trim_error_t);

fprintf('\n--- Turn trim results ---\n');
fprintf('Va      = %.4f m/s\n',  y_trim_t(1));
fprintf('alpha   = %.4f deg\n',  y_trim_t(2)*180/pi);
fprintf('beta    = %.4f deg  (must be ~0)\n', y_trim_t(3)*180/pi);
fprintf('phi     = %.4f deg  (expected %.2f)\n', x_trim_t(7)*180/pi, phi_turn*180/pi);
fprintf('theta   = %.4f deg\n',  x_trim_t(8)*180/pi);
fprintf('delta_e = %.4f deg\n',  u_trim_t(1)*180/pi);
fprintf('delta_a = %.4f deg\n',  u_trim_t(2)*180/pi);
fprintf('delta_t = %.4f\n',      u_trim_t(4));
fprintf('Only psi should change in simulation (check this manually).\n');

save('trim_results_turn.mat', 'x_trim_t','u_trim_t','y_trim_t');
fprintf('\nSaved trim_results_turn.mat\n');

% =========================================================================
% CASE 3 : Gamma study — run for multiple climb angles (Sec 4(iv))
%   Verifies that altitude rate = Va * sin(gamma)
% =========================================================================
fprintf('\n======================================================\n');
fprintf(' CASE 3 : Gamma study  (climb rate verification)\n');
fprintf('======================================================\n');

gamma_list = [-5, -3, 0, 3, 5] * pi/180;
fprintf('%-8s  %-12s  %-14s  %-14s\n', 'gamma(deg)', 'Va(m/s)', 'h_dot_theory', 'delta_e(deg)');
fprintf('%-8s  %-12s  %-14s  %-14s\n', '----------', '--------', '------------', '-----------');

for k = 1:length(gamma_list)
    aerosonde_parameters
    gk = gamma_list(k);
    x0k = [0; 0; MAV.pd0; Va; 0; 0; 0; gk; 0; 0; 0; 0];
    u0k = [0; 0; 0; 0.5];
    y0k = [Va; 0; 0];
    dx0k = [0; 0; -Va*sin(gk); 0; 0; 0; 0; 0; 0; 0; 0; 0];
    [xk,uk,yk,~] = trim('mavsim_trim', x0k, u0k, y0k, [], [2;3], [1;3], dx0k, [3;4;5;6;7;8;9;10;11;12]);
    fprintf('%-8.1f  %-12.4f  %-14.4f  %-14.4f\n', gk*180/pi, yk(1), yk(1)*sin(gk), uk(1)*180/pi);
end
fprintf('\nClimb rate h_dot should equal Va*sin(gamma).\n');