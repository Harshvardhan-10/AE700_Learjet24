% generate_report_plots.m  v2 — FIXED (no improper TFs)
%   Root cause of error: TF_da_c = feedback(C_phi, G_phi) is improper
%   because C_phi contains a pure 's' term (derivative).
%
%   Fix: compute control inputs using u = C * S * r, where S = 1 - CL.
%   C*S is always proper because the 's' in C cancels with the extra
%   integrator/pole in S when the loop contains an integrator, and for
%   PD controllers the sensitivity rolls off fast enough.
%   Where that still causes issues, split into P and D parts explicitly:
%     delta_a = kp*(phi_c - phi) + kd*(0 - p)
%             = kp*S*phi_c       - kd*s*CL*phi_c
%   Both terms are proper (s*CL has relative degree >= 0 for this plant).
%
%   Prerequisites (run once in order):
%     1. compute_trim.m         -> trim_results.mat
%     2. compute_ss_model.m     -> ss_models.mat
%     3. compute_tf_model.m     -> transfer_function_coef.mat
%     4. compute_autopilot_gains.m -> autopilot_gains.mat

clear; clc;
addpath('../chap5')
addpath('../chap6')
addpath('../parameters')
load('transfer_function_coef.mat');
load('autopilot_gains.mat');

FS  = 12;
LW  = 1.8;
r2d = 180/pi;
t   = (0 : AP.Ts : 200)';

t_dist      = 30;    % disturbance step time [s]
D_mag       = 0.2;   % disturbance magnitude [rad] (project spec)
phi_c_deg   = 20;    % roll command step
chi_c_deg   = 45;    % course command step
theta_c_deg = 5;     % pitch command step
delta_h     = 20;    % altitude step [m]
delta_Va    = 5;     % airspeed step [m/s]

fprintf('Building closed-loop transfer functions...\n');
s = tf('s');

% =========================================================================
% LATERAL
% =========================================================================
G_phi   = tf(a_phi2, [1, a_phi1, 0]);
kp_phi  = AP.roll_kp;
kd_phi  = AP.roll_kd;

% CL: phi/phi_c
CL_phi  = minreal(feedback(G_phi * (kp_phi + kd_phi*s), 1));
S_phi   = minreal(1 - CL_phi);   % sensitivity: e_phi / phi_c

% Control input delta_a / phi_c
%   delta_a = kp*(phi_c-phi) + kd*(0-p)
%           = kp*S*phi_c - kd*s*CL*phi_c        [both proper]
TF_da_cmd   = minreal(kp_phi*S_phi - kd_phi*s*CL_phi);

% phi and delta_a due to disturbance at delta_a input
%   phi_d    = G*S * d
%   delta_a_d= -kp*G*S + kd*s*G*S = -(kp - kd*s)*G*S
TF_phi_dist = minreal(G_phi * S_phi);
TF_da_dist  = minreal(-(kp_phi - kd_phi*s) * TF_phi_dist);

% Course loop
G_chi_phi   = tf(AP.gravity/Va_trim, [1,0]);
G_course    = minreal(G_chi_phi * CL_phi);
C_chi       = AP.course_kp + AP.course_ki/s;
CL_chi      = minreal(feedback(G_course * C_chi, 1));
S_chi       = minreal(1 - CL_chi);
% phi_c commanded by course PI: phi_c = C_chi * S_chi * chi_c  [proper: PI*S]
TF_phic_chi = minreal(C_chi * S_chi);
TF_da_chi   = minreal(TF_da_cmd * TF_phic_chi);

% Sideslip (2nd order CL, used for IC response)
G_beta  = tf(a_beta2, [1, a_beta1]);
C_beta  = AP.sideslip_kp + AP.sideslip_ki/s;
CL_beta = minreal(feedback(G_beta * C_beta, 1));

% =========================================================================
% LONGITUDINAL
% =========================================================================
G_theta  = tf(a_theta3, [1, a_theta1, a_theta2]);
kp_th    = AP.pitch_kp;
kd_th    = AP.pitch_kd;

CL_theta = minreal(feedback(G_theta*(kp_th + kd_th*s), 1));
S_theta  = minreal(1 - CL_theta);
% delta_e / theta_c: same split as roll
TF_de_cmd = minreal(kp_th*S_theta - kd_th*s*CL_theta);

K_DC = dcgain(CL_theta);

% Altitude
G_h     = minreal(tf(Va_trim,[1,0]) * CL_theta);
C_h     = AP.altitude_kp + AP.altitude_ki/s;
CL_h    = minreal(feedback(G_h * C_h, 1));
S_h     = minreal(1 - CL_h);
TF_tc_h = minreal(C_h * S_h);          % theta_c / h_c
TF_de_h = minreal(TF_de_cmd * TF_tc_h);

% Airspeed / throttle
G_Va_dt   = tf(a_V2, [1, a_V1]);
C_Va_dt   = AP.airspeed_throttle_kp + AP.airspeed_throttle_ki/s;
CL_Va_dt  = minreal(feedback(G_Va_dt * C_Va_dt, 1));
S_Va_dt   = minreal(1 - CL_Va_dt);
TF_dt_cmd = minreal(C_Va_dt * S_Va_dt);   % delta_t / Va_c  [PI*S, proper]

% Airspeed / pitch  (plant has negative sign)
G_Va_th  = tf(-a_V3, [1, a_V1]);
C_Va_th  = AP.airspeed_pitch_kp + AP.airspeed_pitch_ki/s;
CL_Va_th = minreal(feedback(G_Va_th * C_Va_th, 1));
S_Va_th  = minreal(1 - CL_Va_th);
TF_tc_Va = minreal(C_Va_th * S_Va_th);    % theta_c / Va_c
TF_de_Va = minreal(TF_de_cmd * TF_tc_Va);

% =========================================================================
% Stability check
% =========================================================================
sys_list  = {CL_phi, CL_chi, CL_beta, CL_theta, CL_h, CL_Va_dt, CL_Va_th};
sys_names = {'phi','chi','beta','theta','h','Va_throttle','Va_pitch'};
fprintf('Stability check:\n');
for k = 1:length(sys_list)
    p = pole(sys_list{k});
    if any(real(p) >= 0)
        fprintf('  %-15s : UNSTABLE\n', sys_names{k});
    else
        fprintf('  %-15s : stable  (slowest: %.4f)\n', sys_names{k}, max(real(p)));
    end
end

% =========================================================================
% Properness guard
% =========================================================================
check_tfs   = {TF_da_cmd, TF_da_dist, TF_phi_dist, TF_da_chi, ...
               TF_de_cmd, TF_dt_cmd,  TF_tc_h,     TF_de_h, ...
               TF_tc_Va,  TF_de_Va};
check_names = {'TF_da_cmd','TF_da_dist','TF_phi_dist','TF_da_chi', ...
               'TF_de_cmd','TF_dt_cmd', 'TF_tc_h',   'TF_de_h', ...
               'TF_tc_Va', 'TF_de_Va'};
for k = 1:length(check_tfs)
    [num,den] = tfdata(check_tfs{k},'v');
    if length(num) > length(den)
        error('Improper TF: %s  — check sign conventions.', check_names{k});
    end
end
fprintf('All TFs proper. Running simulations...\n\n');

% =========================================================================
% Inputs
% =========================================================================
phi_step   = phi_c_deg   * pi/180 * ones(size(t));
chi_step   = chi_c_deg   * pi/180 * ones(size(t));
theta_step = theta_c_deg * pi/180 * ones(size(t));
h_step     = delta_h     * ones(size(t));
Va_step    = delta_Va    * ones(size(t));
dist_step  = zeros(size(t));
dist_step(t >= t_dist) = D_mag;

% =========================================================================
% State responses
% =========================================================================
phi_resp   = lsim(CL_phi,   phi_step,  t);
chi_resp   = lsim(CL_chi,   chi_step,  t);
theta_resp = lsim(CL_theta, theta_step, t);
h_resp     = lsim(CL_h,     h_step,    t);
Va_dt_resp = lsim(CL_Va_dt, Va_step,   t);
Va_th_resp = lsim(CL_Va_th, Va_step,   t);

% Roll with disturbance (superposition)
phi_dist   = lsim(CL_phi,       phi_step,  t) + ...
             lsim(TF_phi_dist,  dist_step, t);

% Sideslip: initial condition response from beta0 = 5 deg
beta0   = 5 * pi/180;
[~,~,X] = lsim(CL_beta, zeros(size(t)), t);   % get state dimension
n_st    = size(X,2);
x0_beta = zeros(1, n_st);
x0_beta(1) = beta0;                            % set first state to beta0
beta_ic = lsim(CL_beta, zeros(size(t)), t, x0_beta);

% =========================================================================
% Control inputs
% =========================================================================
da_roll   = lsim(TF_da_cmd,  phi_step,   t);
da_dist_r = lsim(TF_da_cmd,  phi_step,   t) + lsim(TF_da_dist, dist_step, t);
da_course = lsim(TF_da_chi,  chi_step,   t);
de_pitch  = lsim(TF_de_cmd,  theta_step, t);
de_alt    = lsim(TF_de_h,    h_step,     t);
dt_Va     = AP.delta_t_trim + lsim(TF_dt_cmd, Va_step, t);
tc_Va_p   = lsim(TF_tc_Va,   Va_step,    t);
de_Va_p   = lsim(TF_de_Va,   Va_step,    t);

fprintf('All lsim complete.\n\n');

% =========================================================================
% Plotting helpers
% =========================================================================
function tracked(t, actual, cmd_scalar, ylbl, albl, clbl, ttl, fs, lw)
    plot(t, actual, 'b-',  'LineWidth', lw); hold on
    plot(t, cmd_scalar*ones(size(t)), 'r--', 'LineWidth', lw*0.8);
    xlabel('Time (s)','FontSize',fs); ylabel(ylbl,'FontSize',fs);
    title(ttl,'FontSize',fs);
    legend(albl, clbl, 'Location','best','FontSize',fs-1);
    grid on; set(gca,'FontSize',fs);
end

function ctrl_plot(t, u, ylbl, ttl, fs, lw, lim_val)
    plot(t, u, 'k-', 'LineWidth', lw); hold on
    if nargin > 6 && ~isempty(lim_val)
        yline( lim_val,'r--','LineWidth',0.8);
        yline(-lim_val,'r--','LineWidth',0.8);
    end
    xlabel('Time (s)','FontSize',fs); ylabel(ylbl,'FontSize',fs);
    title(ttl,'FontSize',fs);
    grid on; set(gca,'FontSize',fs);
end

W = 800; H = 500;
da_lim = AP.delta_a_max*r2d;
de_lim = AP.delta_e_max*r2d;

% =========================================================================
% FIG 1 — Roll, no disturbance
% =========================================================================
figure(1); clf; set(gcf,'Position',[50 500 W H]);
tracked(t, phi_resp*r2d, phi_c_deg, '\phi (deg)', ...
    'Actual \phi', sprintf('\\phi_c = %d°', phi_c_deg), ...
    sprintf('Fig 1 — Roll response, no disturbance  (\\phi_c = %d°)', phi_c_deg), FS, LW);
saveas(gcf,'fig1_roll_no_disturbance.png');
fprintf('Saved fig1_roll_no_disturbance.png\n');

% =========================================================================
% FIG 2 — Roll, with disturbance
% =========================================================================
figure(2); clf; set(gcf,'Position',[50 500 W H]);
plot(t, phi_dist*r2d,           'b-',  'LineWidth', LW); hold on
plot(t, phi_c_deg*ones(size(t)),'r--', 'LineWidth', LW*0.8);
xline(t_dist, 'g:', 'LineWidth', 1.2);
xlabel('Time (s)','FontSize',FS); ylabel('\phi (deg)','FontSize',FS);
title(sprintf('Fig 2 — Roll response WITH disturbance  (step %.1f at t = %d s)', D_mag, t_dist),'FontSize',FS);
legend('Actual \phi', sprintf('\\phi_c = %d°',phi_c_deg), 'Disturbance onset', ...
       'Location','best','FontSize',FS-1);
grid on; set(gca,'FontSize',FS);
saveas(gcf,'fig2_roll_with_disturbance.png');
fprintf('Saved fig2_roll_with_disturbance.png\n');

% =========================================================================
% FIG 3 — Course
% =========================================================================
figure(3); clf; set(gcf,'Position',[50 500 W H]);
tracked(t, chi_resp*r2d, chi_c_deg, '\chi (deg)', ...
    'Actual \chi', sprintf('\\chi_c = %d°', chi_c_deg), ...
    sprintf('Fig 3 — Course angle response  (\\chi_c = %d°)', chi_c_deg), FS, LW);
saveas(gcf,'fig3_course.png');
fprintf('Saved fig3_course.png\n');

% =========================================================================
% FIG 4 — Sideslip (IC response)
% =========================================================================
figure(4); clf; set(gcf,'Position',[50 500 W H]);
plot(t, beta_ic*r2d,        'b-',  'LineWidth', LW); hold on
plot(t, zeros(size(t)),     'r--', 'LineWidth', LW*0.8);
xlabel('Time (s)','FontSize',FS); ylabel('\beta (deg)','FontSize',FS);
title(sprintf('Fig 4 — Sideslip \\beta: disturbance rejection  (\\beta_0 = %d°, \\beta_c = 0)', round(beta0*r2d)),'FontSize',FS);
legend('Actual \beta','\beta_c = 0','Location','best','FontSize',FS-1);
grid on; set(gca,'FontSize',FS);
saveas(gcf,'fig4_sideslip.png');
fprintf('Saved fig4_sideslip.png\n');

% =========================================================================
% FIG 5 — Lateral control inputs
% =========================================================================
figure(5); clf; set(gcf,'Position',[50 500 W 600]);
subplot(2,1,1)
ctrl_plot(t, da_roll*r2d, '\delta_a (deg)', ...
    sprintf('Aileron — roll step (\\phi_c = %d°)', phi_c_deg), FS, LW, da_lim);
subplot(2,1,2)
ctrl_plot(t, da_course*r2d, '\delta_a (deg)', ...
    sprintf('Aileron — course step (\\chi_c = %d°)', chi_c_deg), FS, LW, da_lim);
sgtitle('Fig 5 — Lateral control inputs (\delta_a)','FontSize',FS+1);
saveas(gcf,'fig5_lateral_inputs.png');
fprintf('Saved fig5_lateral_inputs.png\n');

% =========================================================================
% FIG 6 — Pitch
% =========================================================================
figure(6); clf; set(gcf,'Position',[50 500 W H]);
tracked(t, theta_resp*r2d, theta_c_deg, '\theta (deg)', ...
    'Actual \theta', sprintf('\\theta_c = %d°', theta_c_deg), ...
    sprintf('Fig 6 — Pitch response  (\\theta_c = %d°)', theta_c_deg), FS, LW);
saveas(gcf,'fig6_pitch.png');
fprintf('Saved fig6_pitch.png\n');

% =========================================================================
% FIG 7 — Altitude
% =========================================================================
figure(7); clf; set(gcf,'Position',[50 500 W H]);
tracked(t, h_resp, delta_h, 'h (m)', ...
    'Actual h', sprintf('h_c = trim + %d m', delta_h), ...
    sprintf('Fig 7 — Altitude response  (\\Deltah = %d m)', delta_h), FS, LW);
saveas(gcf,'fig7_altitude.png');
fprintf('Saved fig7_altitude.png\n');

% =========================================================================
% FIG 8 — Airspeed via throttle
% =========================================================================
figure(8); clf; set(gcf,'Position',[50 500 W 600]);
subplot(2,1,1)
tracked(t, Va_trim+Va_dt_resp, Va_trim+delta_Va, 'V_a (m/s)', ...
    'Actual V_a', sprintf('V_{a,c} = %.0f m/s', Va_trim+delta_Va), ...
    sprintf('Fig 8 — Airspeed via throttle  (\\DeltaV_a = %d m/s)', delta_Va), FS, LW);
subplot(2,1,2)
ctrl_plot(t, dt_Va, '\delta_t', 'Throttle command', FS, LW, []);
yline(1,'r--','LineWidth',0.8,'Label','+limit');
yline(0,'r--','LineWidth',0.8,'Label','-limit');
set(gca,'FontSize',FS);
saveas(gcf,'fig8_airspeed_throttle.png');
fprintf('Saved fig8_airspeed_throttle.png\n');

% =========================================================================
% FIG 9 — Airspeed via pitch
% =========================================================================
figure(9); clf; set(gcf,'Position',[50 500 W 600]);
subplot(2,1,1)
tracked(t, Va_trim+Va_th_resp, Va_trim+delta_Va, 'V_a (m/s)', ...
    'Actual V_a', sprintf('V_{a,c} = %.0f m/s', Va_trim+delta_Va), ...
    sprintf('Fig 9 — Airspeed via pitch  (\\DeltaV_a = %d m/s, \\delta_t fixed at trim)', delta_Va), FS, LW);
subplot(2,1,2)
ctrl_plot(t, tc_Va_p*r2d, '\theta_c (deg)', ...
    'Pitch command from airspeed/pitch controller', FS, LW, []);
set(gca,'FontSize',FS);
saveas(gcf,'fig9_airspeed_pitch.png');
fprintf('Saved fig9_airspeed_pitch.png\n');

% =========================================================================
% FIG 10 — Longitudinal control inputs
% =========================================================================
figure(10); clf; set(gcf,'Position',[50 500 W 600]);
subplot(2,1,1)
ctrl_plot(t, de_pitch*r2d, '\delta_e (deg)', ...
    sprintf('Elevator — pitch step (\\theta_c = %d°)', theta_c_deg), FS, LW, de_lim);
subplot(2,1,2)
ctrl_plot(t, de_alt*r2d, '\delta_e (deg)', ...
    sprintf('Elevator — altitude step (\\Deltah = %d m)', delta_h), FS, LW, de_lim);
sgtitle('Fig 10 — Longitudinal control inputs (\delta_e)','FontSize',FS+1);
saveas(gcf,'fig10_longitudinal_inputs.png');
fprintf('Saved fig10_longitudinal_inputs.png\n');

fprintf('\nAll 10 figures saved.\n');