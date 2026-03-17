% compute_tf_model.m
%   Derives transfer function models from linearised state-space.
%
%   Key fix: a_theta2 is NOT A_lon(3,4) (which is zero because Cm has no
%   direct theta dependence). Instead, a_theta2 comes from the short-period
%   approximation (B&M Eq. 5.22):
%       a_theta2 = -M_alpha/Jy = -M_w * Va_trim = -A_lon(3,2) * Va_trim
%   This captures gravity's indirect effect through the alpha-theta
%   relationship at trim.

addpath('../parameters')
addpath('../tools')
aerosonde_parameters
load('trim_results.mat')
load('ss_models.mat')

% =======================================================================
% LATERAL coefficients
% A_lat states = [v(1), p(2), r(3), phi(4), psi(5)]
% Inputs       = [delta_a(1), delta_r(2)]
% =======================================================================

% Roll:  phi/delta_a = a_phi2 / (s^2 + a_phi1*s)
a_phi1 = -A_lat(2,2);       % -L_p
a_phi2 =  B_lat(2,1);       %  L_da

% Sideslip:  beta/delta_r = a_beta2 / (s + a_beta1)
a_beta1 = -A_lat(1,1);              % -Y_v  [1/s]
a_beta2 =  B_lat(1,2) / Va_trim;    %  Y_dr/Va

% =======================================================================
% LONGITUDINAL coefficients
% A_lon states = [u(1), w(2), q(3), theta(4), pd(5)]
% Inputs       = [delta_e(1), delta_t(2)]
% =======================================================================

% Pitch:  theta/delta_e = a_theta3 / (s^2 + a_theta1*s + a_theta2)
a_theta1 = -A_lon(3,3);     % -M_q

% a_theta2 = M_alpha/Jy expressed through state-space.
% A_lon(3,2) = dqdot/dw = M_w.  M_alpha = M_w * Va_trim (since alpha=w/Va)
% B&M Eq 5.22: a_theta2 = -A_lon(3,2)*Va_trim
a_theta2 = -A_lon(3,2) * Va_trim;   % CORRECT: uses M_w * Va

a_theta3 =  B_lon(3,1);     %  M_de

K_theta_DC = -a_theta3 / a_theta2;

% Airspeed
a_V1 = -A_lon(1,1);                     % -X_u
a_V2 =  B_lon(1,2);                     %  X_dt
a_V3 =  MAV.gravity * cos(theta_trim);  %  g*cos(theta_trim)

% =======================================================================
% Transfer functions
% =======================================================================
T_phi_delta_a   = tf([a_phi2],              [1, a_phi1, 0]);
T_chi_phi       = tf([MAV.gravity/Va_trim], [1, 0]);
T_theta_delta_e = tf([a_theta3],            [1, a_theta1, a_theta2]);
T_h_theta       = tf([Va_trim],             [1, 0]);
T_h_Va          = tf([theta_trim],          [1, 0]);
T_Va_delta_t    = tf([a_V2],                [1, a_V1]);
T_Va_theta      = tf([-a_V3],               [1, a_V1]);
T_v_delta_r     = tf([a_beta2],             [1, a_beta1]);

% =======================================================================
% Display
% =======================================================================
fprintf('\n=== Transfer Function Coefficients ===\n');
fprintf('Lateral:\n');
fprintf('  a_phi1   = %8.4f   a_phi2   = %8.4f\n', a_phi1, a_phi2);
fprintf('  a_beta1  = %8.4f   a_beta2  = %8.4f\n', a_beta1, a_beta2);
fprintf('Longitudinal:\n');
fprintf('  a_theta1 = %8.4f\n', a_theta1);
fprintf('  a_theta2 = %8.4f  (from M_w*Va = %.4f * %.4f)\n', ...
        a_theta2, A_lon(3,2), Va_trim);
fprintf('  a_theta3 = %8.4f\n', a_theta3);
fprintf('  K_theta_DC = %.4f\n', K_theta_DC);
fprintf('  a_V1 = %8.4f   a_V2 = %8.4f   a_V3 = %8.4f\n', a_V1, a_V2, a_V3);
fprintf('  a_beta1 = %8.4f   a_beta2 = %8.4f\n', a_beta1, a_beta2);

% Sanity checks
if a_phi1  <= 0, warning('a_phi1 should be > 0 (roll damping).'); end
if a_theta1<= 0, warning('a_theta1 should be > 0 (pitch damping).'); end
if a_theta2<= 0, warning('a_theta2 should be > 0 (pitch stiffness).'); end
if a_V1    <= 0, warning('a_V1 should be > 0 (speed damping).'); end

% =======================================================================
% Save
% =======================================================================
save('transfer_function_coef.mat', ...
    'a_phi1','a_phi2','a_beta1','a_beta2', ...
    'a_theta1','a_theta2','a_theta3','K_theta_DC', ...
    'a_V1','a_V2','a_V3', ...
    'Va_trim','alpha_trim','theta_trim', ...
    'T_phi_delta_a','T_chi_phi','T_theta_delta_e', ...
    'T_h_theta','T_h_Va','T_Va_delta_t','T_Va_theta','T_v_delta_r');
fprintf('Saved to transfer_function_coef.mat\n');