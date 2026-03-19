% compute_ss_model.m
%   Linearises mavsim_trim about the trim condition.
%   With Euler-based mav_dynamics (12 states), linmod returns A as 12x12
%   directly — no quaternion-to-Euler transformation needed.
%
%   State ordering from mdlOutputs (= state ordering):
%     1=pn  2=pe  3=pd  4=u  5=v  6=w
%     7=phi  8=theta  9=psi  10=p  11=q  12=r
%
%   Inputs: [delta_e(1), delta_a(2), delta_r(3), delta_t(4)]
%   Outputs (from mavsim_trim): [Va(1), alpha(2), beta(3)]

addpath('../parameters')
addpath('../tools')
aerosonde_parameters
load('trim_results.mat')

[A, B, C, D] = linmod('mavsim_trim', x_trim, u_trim);

fprintf('linmod sizes: A=%dx%d  B=%dx%d\n', size(A,1),size(A,2),size(B,1),size(B,2));
% Expected: A=12x12, B=12x4

% -----------------------------------------------------------------------
% Lateral state-space
%   States: [v(5), p(10), r(12), phi(7), psi(9)]
%   Inputs: [delta_a(2), delta_r(3)]
% -----------------------------------------------------------------------
lat_idx = [5, 10, 12, 7, 9];
A_lat = A(lat_idx, lat_idx);
B_lat = B(lat_idx, [2, 3]);

% -----------------------------------------------------------------------
% Longitudinal state-space
%   States: [u(4), w(6), q(11), theta(8), pd(3)]
%   Inputs: [delta_e(1), delta_t(4)]
% -----------------------------------------------------------------------
lon_idx = [4, 6, 11, 8, 3];
A_lon = A(lon_idx, lon_idx);
B_lon = B(lon_idx, [1, 4]);

% -----------------------------------------------------------------------
% Display
% -----------------------------------------------------------------------
disp('=== Lateral  [v, p, r, phi, psi] ===')
disp('A_lat ='); disp(A_lat)
disp('B_lat ='); disp(B_lat)
fprintf('Lateral eigenvalues:\n'); disp(eig(A_lat))

disp('=== Longitudinal  [u, w, q, theta, pd] ===')
disp('A_lon ='); disp(A_lon)
disp('B_lon ='); disp(B_lon)
fprintf('Longitudinal eigenvalues:\n'); disp(eig(A_lon))

% -----------------------------------------------------------------------
% Save
% -----------------------------------------------------------------------
save('ss_models.mat', 'A_lat','B_lat','A_lon','B_lon', ...
     'Va_trim','alpha_trim','theta_trim');
fprintf('Saved to ss_models.mat\n');

% Restore MAV ICs from trim after aerosonde_parameters reset them
MAV.pn0 = x_trim(1);  MAV.pe0 = x_trim(2);  MAV.pd0 = x_trim(3);
MAV.u0  = x_trim(4);  MAV.v0  = x_trim(5);  MAV.w0  = x_trim(6);
MAV.phi0= x_trim(7);  MAV.theta0=x_trim(8); MAV.psi0= x_trim(9);
MAV.p0  = x_trim(10); MAV.q0  = x_trim(11); MAV.r0  = x_trim(12);