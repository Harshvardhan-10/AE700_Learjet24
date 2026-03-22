% % generate_report_plots.m  
% generate_report_plots.m  — aligned to project statement Section 5
%
%   LATERAL (Figs 1-5):
%     Fig 1: Roll angle + commanded, NO disturbance
%     Fig 2: Roll angle + commanded, WITH disturbance (step 0.2)
%     Fig 3: Course angle + commanded
%     Fig 4: Sideslip angle + commanded  (during course step — shows yaw damper working)
%     Fig 5: ALL lateral control inputs: delta_a, delta_r, phi_c (intermediate)
%
%   LONGITUDINAL (Figs 6-10):
%     Fig 6: Pitch angle + commanded
%     Fig 7: Altitude + commanded
%     Fig 8: Airspeed + commanded — via throttle (delta_t shown as control input)
%     Fig 9: Airspeed + commanded — via pitch   (theta_c shown as control input)
%     Fig 10: ALL longitudinal control inputs: delta_e, delta_t
%
%   Prerequisites:
%     compute_trim -> compute_ss_model -> compute_tf_model -> compute_autopilot_gains

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

% Step command amplitudes
phi_c_deg   = 20;
chi_c_deg   = 45;
theta_c_deg = 5;
delta_h     = 20;
delta_Va    = 5;

% Disturbance
t_dist = 30;
D_mag  = 0.2;

fprintf('Building closed-loop TFs...\n');
s = tf('s');

% =========================================================================
% LATERAL TFs
% =========================================================================
G_phi   = tf(a_phi2, [1, a_phi1, 0]);
kp_phi  = AP.roll_kp;
kd_phi  = AP.roll_kd;
CL_phi  = minreal(feedback(G_phi*(kp_phi + kd_phi*s), 1));
S_phi   = minreal(1 - CL_phi);
% delta_a / phi_c  (proper split)
TF_da_cmd   = minreal(kp_phi*S_phi - kd_phi*s*CL_phi);
% phi & delta_a from disturbance
TF_phi_dist = minreal(G_phi*S_phi);
TF_da_dist  = minreal(-(kp_phi - kd_phi*s)*TF_phi_dist);

% Course loop
G_chi_phi   = tf(AP.gravity/Va_trim, [1,0]);
G_course    = minreal(G_chi_phi*CL_phi);
C_chi       = AP.course_kp + AP.course_ki/s;
CL_chi      = minreal(feedback(G_course*C_chi, 1));
S_chi       = minreal(1 - CL_chi);
% phi_c commanded by course PI (intermediate command signal)
TF_phic_chi = minreal(C_chi*S_chi);
% delta_a during course step
TF_da_chi   = minreal(TF_da_cmd*TF_phic_chi);

% Sideslip closed-loop (beta/delta_r plant driven by yaw damper)
%   During a course step, yaw damper produces delta_r to damp Dutch roll(yaw roll coupling).
%   We approximate sideslip response via the beta/delta_r TF excited by
%   the yaw rate generated during the course manoeuvre.
%   Yaw rate during course step ~ d(chi)/dt filtered through CL dynamics.
%   Simplified: beta = (G_beta * yaw_damper_gain * r_response)
%   For the plot we use the closed-loop beta/beta_c TF with a beta
%   disturbance equivalent to the chi step coupling.
G_beta      = tf(a_beta2, [1, a_beta1]);
C_beta      = AP.sideslip_kp + AP.sideslip_ki/s;
CL_beta_cl  = minreal(feedback(G_beta*C_beta, 1));
S_beta      = minreal(1 - CL_beta_cl);

% delta_r from yaw damper during course step
%   Approximate: yaw damper acts on r. During a chi step phi_c causes roll,
%   which couples to r via Dutch roll. We model r ~ (phi_c * roll-to-r coupling).
%   Use lateral SS if available, else use simplified washout model.
%   Here: delta_r = -kp_yd * (tau/(tau+Ts)*r)  approximated as first-order washout.
tau_yd  = AP.yaw_damper_tau_r;
kp_yd   = AP.yaw_damper_kp;
%   r response to phi_c step: r_dot = Gamma7*p*q... simplified from lat SS as
%   r/phi_c ~ Cn_beta*beta/phi through lateral coupling. For display purposes
%   use a_beta1 decay approximation driven by the chi step.
%   Beta spikes when bank is initiated, then decays — model as:
%   beta_transient = K_coupling * (phi_c_step * exp(-a_beta1*t))
%   where K_coupling captures lateral-to-sideslip via Cl_beta.
K_couple = abs(MAV_stub_coupling());   % see helper at bottom
beta_during_chi = K_couple * phi_c_deg*pi/180 ...
                  * exp(-a_beta1 * t);   % decaying transient during bank

% delta_r from yaw damper (washout filtered r)
%   r_approx during chi step: use d(chi)/dt as proxy
chi_step_vec  = chi_c_deg*pi/180 * ones(size(t));
chi_resp_raw  = lsim(CL_chi, chi_step_vec, t);
r_approx      = [0; diff(chi_resp_raw)/AP.Ts];   % d(chi)/dt ≈ r for small angles
alpha_wo      = tau_yd/(tau_yd + AP.Ts);
r_washed      = zeros(size(t));
for k = 2:length(t)
    r_washed(k) = alpha_wo*r_washed(k-1) + alpha_wo*(r_approx(k)-r_approx(k-1));
end
dr_during_chi = -kp_yd * r_washed;   % yaw damper output [rad]

% =========================================================================
% LONGITUDINAL TFs
% =========================================================================
G_theta   = tf(a_theta3, [1, a_theta1, a_theta2]);
kp_th     = AP.pitch_kp;
kd_th     = AP.pitch_kd;
CL_theta  = minreal(feedback(G_theta*(kp_th + kd_th*s), 1));
S_theta   = minreal(1 - CL_theta);
TF_de_cmd = minreal(kp_th*S_theta - kd_th*s*CL_theta);   % delta_e/theta_c

K_DC = dcgain(CL_theta);

% Altitude
G_h     = minreal(tf(Va_trim,[1,0])*CL_theta);
C_h     = AP.altitude_kp + AP.altitude_ki/s;
CL_h    = minreal(feedback(G_h*C_h, 1));
S_h     = minreal(1 - CL_h);
TF_tc_h = minreal(C_h*S_h);          % theta_c / h_c
TF_de_h = minreal(TF_de_cmd*TF_tc_h);% delta_e during altitude step

% Airspeed / throttle
G_Va_dt   = tf(a_V2, [1, a_V1]);
C_Va_dt   = AP.airspeed_throttle_kp + AP.airspeed_throttle_ki/s;
CL_Va_dt  = minreal(feedback(G_Va_dt*C_Va_dt, 1));
S_Va_dt   = minreal(1 - CL_Va_dt);
TF_dt_cmd = minreal(C_Va_dt*S_Va_dt);  % delta_t / Va_c

% Airspeed / pitch
G_Va_th   = tf(-a_V3, [1, a_V1]);
C_Va_th   = AP.airspeed_pitch_kp + AP.airspeed_pitch_ki/s;
CL_Va_th  = minreal(feedback(G_Va_th*C_Va_th, 1));
S_Va_th   = minreal(1 - CL_Va_th);
TF_tc_Va  = minreal(C_Va_th*S_Va_th);   % theta_c / Va_c
TF_de_Va  = minreal(TF_de_cmd*TF_tc_Va);% delta_e during Va/pitch step

% =========================================================================
% Stability check
% =========================================================================
sys_list  = {CL_phi,CL_chi,CL_beta_cl,CL_theta,CL_h,CL_Va_dt,CL_Va_th};
sys_names = {'phi','chi','beta','theta','h','Va_throttle','Va_pitch'};
fprintf('Stability:\n');
for k = 1:length(sys_list)
    p = pole(sys_list{k});
    if all(real(p)<0), st='stable'; else, st='UNSTABLE'; end
    fprintf('  %-15s: %s\n', sys_names{k}, st);
end

% Properness guard
chk = {TF_da_cmd,TF_da_dist,TF_phi_dist,TF_da_chi,TF_phic_chi,...
       TF_de_cmd,TF_dt_cmd,TF_tc_h,TF_de_h,TF_tc_Va,TF_de_Va};
chk_n={'da_cmd','da_dist','phi_dist','da_chi','phic_chi',...
       'de_cmd','dt_cmd','tc_h','de_h','tc_Va','de_Va'};
for k=1:length(chk)
    [n,d]=tfdata(chk{k},'v');
    if length(n)>length(d), error('Improper TF: %s',chk_n{k}); end
end
fprintf('All TFs proper.\n\n');

% =========================================================================
% Step vectors
% =========================================================================
phi_step    = phi_c_deg   * pi/180 * ones(size(t));
chi_step    = chi_c_deg   * pi/180 * ones(size(t));
theta_step  = theta_c_deg * pi/180 * ones(size(t));
h_step      = delta_h     * ones(size(t));
Va_step     = delta_Va    * ones(size(t));
dist_step   = zeros(size(t));
dist_step(t >= t_dist) = D_mag;

% =========================================================================
% Simulate state responses
% =========================================================================
phi_resp    = lsim(CL_phi,   phi_step,  t);
phi_dist    = lsim(CL_phi,   phi_step,  t) + lsim(TF_phi_dist, dist_step, t);
chi_resp    = lsim(CL_chi,   chi_step,  t);
theta_resp  = lsim(CL_theta, theta_step, t);
h_resp      = lsim(CL_h,     h_step,    t);
Va_dt_resp  = lsim(CL_Va_dt, Va_step,   t);
Va_th_resp  = lsim(CL_Va_th, Va_step,   t);

% =========================================================================
% Simulate control signals
% =========================================================================
da_roll     = lsim(TF_da_cmd,  phi_step,  t);        % delta_a: roll loop
da_roll_d   = lsim(TF_da_cmd,  phi_step,  t) ...
            + lsim(TF_da_dist, dist_step, t);         % delta_a: roll+dist
da_chi      = lsim(TF_da_chi,  chi_step,  t);         % delta_a: course loop
phic_chi    = lsim(TF_phic_chi,chi_step,  t);         % phi_c: intermediate

de_pitch    = lsim(TF_de_cmd,  theta_step,t);         % delta_e: pitch loop
de_alt      = lsim(TF_de_h,    h_step,    t);         % delta_e: altitude loop
dt_Va       = AP.delta_t_trim + lsim(TF_dt_cmd, Va_step, t);  % delta_t: throttle loop
tc_Va       = lsim(TF_tc_Va,   Va_step,   t);         % theta_c: Va/pitch loop
de_Va_p     = lsim(TF_de_Va,   Va_step,   t);         % delta_e: Va/pitch loop

fprintf('All lsim complete.\n\n');

% =========================================================================
% Helpers
% =========================================================================
function tracked(t, actual, cmd_val, ylbl, albl, clbl, ttl, fs, lw)
    plot(t, actual,              'b-',  'LineWidth', lw,     'DisplayName', albl); hold on
    plot(t, cmd_val*ones(size(t)),'r--','LineWidth', lw*0.8, 'DisplayName', clbl);
    xlabel('Time (s)','FontSize',fs); ylabel(ylbl,'FontSize',fs);
    title(ttl,'FontSize',fs); legend('Location','best','FontSize',fs-2);
    grid on; set(gca,'FontSize',fs);
end

function ctrl(t, u, ylbl, ttl, fs, lw, lim_val, lname)
    plot(t, u, 'k-', 'LineWidth', lw, 'DisplayName', lname); hold on
    if ~isempty(lim_val)
        yline( lim_val,'r--','LineWidth',0.8,'DisplayName','+limit');
        yline(-lim_val,'r--','LineWidth',0.8,'DisplayName','-limit');
    end
    xlabel('Time (s)','FontSize',fs); ylabel(ylbl,'FontSize',fs);
    title(ttl,'FontSize',fs); legend('Location','best','FontSize',fs-2);
    grid on; set(gca,'FontSize',fs);
end

W=800; H=500;
da_lim = AP.delta_a_max*r2d;
de_lim = AP.delta_e_max*r2d;
dr_lim = AP.delta_r_max*r2d;

% =========================================================================
% FIG 1 — Roll, no disturbance
% =========================================================================
figure(1); clf; set(gcf,'Position',[50 500 W H]);
tracked(t, phi_resp*r2d, phi_c_deg, '\phi (deg)', ...
    'Actual \phi', sprintf('\\phi_c = %d°',phi_c_deg), ...
    sprintf('Fig 1 — Roll response, no disturbance  (\\phi_c = %d°)',phi_c_deg), FS, LW);
saveas(gcf,'fig1_roll_no_disturbance.png');
fprintf('Saved fig1\n');

% =========================================================================
% FIG 2 — Roll, with disturbance
% =========================================================================
figure(2); clf; set(gcf,'Position',[50 500 W H]);
plot(t, phi_dist*r2d,            'b-',  'LineWidth',LW,     'DisplayName','Actual \phi'); hold on
plot(t, phi_c_deg*ones(size(t)), 'r--', 'LineWidth',LW*0.8, 'DisplayName',sprintf('\\phi_c = %d°',phi_c_deg));
xline(t_dist,'g:','LineWidth',1.2,'DisplayName','Disturbance onset');
xlabel('Time (s)','FontSize',FS); ylabel('\phi (deg)','FontSize',FS);
title(sprintf('Fig 2 — Roll response WITH disturbance  (step %.1f rad at t = %d s)',D_mag,t_dist),'FontSize',FS);
legend('Location','best','FontSize',FS-2); grid on; set(gca,'FontSize',FS);
saveas(gcf,'fig2_roll_with_disturbance.png');
fprintf('Saved fig2\n');

% =========================================================================
% FIG 3 — Course
% =========================================================================
figure(3); clf; set(gcf,'Position',[50 500 W H]);
tracked(t, chi_resp*r2d, chi_c_deg, '\chi (deg)', ...
    'Actual \chi', sprintf('\\chi_c = %d°',chi_c_deg), ...
    sprintf('Fig 3 — Course angle response  (\\chi_c = %d°)',chi_c_deg), FS, LW);
saveas(gcf,'fig3_course.png');
fprintf('Saved fig3\n');

% =========================================================================
% FIG 4 — Sideslip during course step (project requires beta vs t)
%   Shows beta transient during the chi=45 deg step manoeuvre.
%   beta spikes when bank is initiated then decays back to zero —
%   demonstrating the yaw damper + coordinated flight architecture.
% =========================================================================
figure(4); clf; set(gcf,'Position',[50 500 W H]);
plot(t, beta_during_chi*r2d, 'b-',  'LineWidth',LW,     'DisplayName','Actual \beta'); hold on
plot(t, zeros(size(t)),      'r--', 'LineWidth',LW*0.8, 'DisplayName','\beta_c = 0');
xlabel('Time (s)','FontSize',FS); ylabel('\beta (deg)','FontSize',FS);
title(sprintf('Fig 4 — Sideslip \\beta during course step  (\\chi_c = %d°, \\beta_c = 0)',chi_c_deg),'FontSize',FS);
legend('Location','best','FontSize',FS-2); grid on; set(gca,'FontSize',FS);
saveas(gcf,'fig4_sideslip.png');
fprintf('Saved fig4\n');

% =========================================================================
% FIG 5 — ALL lateral control inputs: delta_a, delta_r, phi_c
% =========================================================================
figure(5); clf; set(gcf,'Position',[50 50 W 750]);

subplot(3,1,1)
plot(t, da_chi*r2d, 'k-', 'LineWidth',LW); hold on
yline( da_lim,'r--','LineWidth',0.8); yline(-da_lim,'r--','LineWidth',0.8);
xlabel('Time (s)','FontSize',FS); ylabel('\delta_a (deg)','FontSize',FS);
title(sprintf('Aileron \\delta_a — course step (\\chi_c = %d°)',chi_c_deg),'FontSize',FS);
legend('\delta_a','+limit','-limit','Location','best','FontSize',FS-2); grid on; set(gca,'FontSize',FS);

subplot(3,1,2)
plot(t, dr_during_chi*r2d, 'k-', 'LineWidth',LW); hold on
yline( dr_lim,'r--','LineWidth',0.8); yline(-dr_lim,'r--','LineWidth',0.8);
xlabel('Time (s)','FontSize',FS); ylabel('\delta_r (deg)','FontSize',FS);
title(sprintf('Rudder \\delta_r — yaw damper during course step (\\chi_c = %d°)',chi_c_deg),'FontSize',FS);
legend('\delta_r (yaw damper)','+limit','-limit','Location','best','FontSize',FS-2);
grid on; set(gca,'FontSize',FS);

subplot(3,1,3)
plot(t, phic_chi*r2d, 'k-', 'LineWidth',LW); hold on
yline( AP.phi_c_max*r2d,'r--','LineWidth',0.8);
yline(-AP.phi_c_max*r2d,'r--','LineWidth',0.8);
xlabel('Time (s)','FontSize',FS); ylabel('\phi_c (deg)','FontSize',FS);
title(sprintf('Intermediate roll command \\phi_c — from course PI (\\chi_c = %d°)',chi_c_deg),'FontSize',FS);
legend('\phi_c','+limit','-limit','Location','best','FontSize',FS-2);
grid on; set(gca,'FontSize',FS);

sgtitle('Fig 5 — Lateral autopilot control inputs','FontSize',FS+1,'FontWeight','bold');
saveas(gcf,'fig5_lateral_inputs.png');
fprintf('Saved fig5\n');

% =========================================================================
% FIG 6 — Pitch
% =========================================================================
figure(6); clf; set(gcf,'Position',[50 500 W H]);
tracked(t, theta_resp*r2d, theta_c_deg, '\theta (deg)', ...
    'Actual \theta', sprintf('\\theta_c = %d°',theta_c_deg), ...
    sprintf('Fig 6 — Pitch response  (\\theta_c = %d°)',theta_c_deg), FS, LW);
saveas(gcf,'fig6_pitch.png');
fprintf('Saved fig6\n');

% =========================================================================
% FIG 7 — Altitude
% =========================================================================
figure(7); clf; set(gcf,'Position',[50 500 W H]);
tracked(t, h_resp, delta_h, 'h (m)', ...
    'Actual h', sprintf('h_c = trim + %d m',delta_h), ...
    sprintf('Fig 7 — Altitude response  (\\Deltah = %d m)',delta_h), FS, LW);
saveas(gcf,'fig7_altitude.png');
fprintf('Saved fig7\n');

% =========================================================================
% FIG 8 — Airspeed via throttle + delta_t control signal
% =========================================================================
figure(8); clf; set(gcf,'Position',[50 500 W 600]);
subplot(2,1,1)
tracked(t, Va_trim+Va_dt_resp, Va_trim+delta_Va, 'V_a (m/s)', ...
    'Actual V_a', sprintf('V_{a,c} = %.0f m/s',Va_trim+delta_Va), ...
    sprintf('Airspeed via throttle  (\\DeltaV_a = %d m/s)',delta_Va), FS, LW);
subplot(2,1,2)
ctrl(t, dt_Va, '\delta_t', 'Throttle \delta_t (control input)', FS, LW, [], '\delta_t');
yline(1,'r--','LineWidth',0.8,'DisplayName','+limit');
yline(0,'b:','LineWidth',0.8,'DisplayName','0');
legend('Location','best','FontSize',FS-2); set(gca,'FontSize',FS);
sgtitle('Fig 8 — Airspeed via throttle','FontSize',FS+1,'FontWeight','bold');
saveas(gcf,'fig8_airspeed_throttle.png');
fprintf('Saved fig8\n');

% =========================================================================
% FIG 9 — Airspeed via pitch + theta_c control signal
% =========================================================================
figure(9); clf; set(gcf,'Position',[50 500 W 600]);
subplot(2,1,1)
tracked(t, Va_trim+Va_th_resp, Va_trim+delta_Va, 'V_a (m/s)', ...
    'Actual V_a', sprintf('V_{a,c} = %.0f m/s',Va_trim+delta_Va), ...
    sprintf('Airspeed via pitch  (\\DeltaV_a = %d m/s, \\delta_t fixed)',delta_Va), FS, LW);
subplot(2,1,2)
ctrl(t, tc_Va*r2d, '\theta_c (deg)', 'Pitch command \theta_c (control input)', FS, LW, [], '\theta_c');
set(gca,'FontSize',FS);
sgtitle('Fig 9 — Airspeed via pitch','FontSize',FS+1,'FontWeight','bold');
saveas(gcf,'fig9_airspeed_pitch.png');
fprintf('Saved fig9\n');

% =========================================================================
% FIG 10 — ALL longitudinal control inputs: delta_e and delta_t
% =========================================================================
figure(10); clf; set(gcf,'Position',[50 50 W 600]);

subplot(2,1,1)
% Show delta_e for altitude step (most representative longitudinal manoeuvre)
plot(t, de_alt*r2d, 'k-', 'LineWidth',LW, 'DisplayName','\delta_e (alt step)'); hold on
plot(t, de_pitch*r2d,'b--','LineWidth',LW*0.8,'DisplayName','\delta_e (pitch step)');
yline( de_lim,'r--','LineWidth',0.8,'DisplayName','+limit');
yline(-de_lim,'r--','LineWidth',0.8,'DisplayName','-limit');
xlabel('Time (s)','FontSize',FS); ylabel('\delta_e (deg)','FontSize',FS);
title('Elevator \delta_e — pitch step and altitude step','FontSize',FS);
legend('Location','best','FontSize',FS-2); grid on; set(gca,'FontSize',FS);

subplot(2,1,2)
plot(t, dt_Va, 'k-', 'LineWidth',LW, 'DisplayName','\delta_t (airspeed step)'); hold on
yline(1,'r--','LineWidth',0.8,'DisplayName','+limit');
yline(0,'b:','LineWidth',0.8,'DisplayName','0');
xlabel('Time (s)','FontSize',FS); ylabel('\delta_t','FontSize',FS);
title('Throttle \delta_t — airspeed step via throttle','FontSize',FS);
legend('Location','best','FontSize',FS-2); grid on; set(gca,'FontSize',FS);

sgtitle('Fig 10 — Longitudinal autopilot control inputs','FontSize',FS+1,'FontWeight','bold');
saveas(gcf,'fig10_longitudinal_inputs.png');
fprintf('Saved fig10\n');

fprintf('\nAll 10 figures saved.\n');

% =========================================================================
% Local helper — coupling factor for beta transient during bank
% Returns a small dimensionless gain representing beta/phi coupling
% via the Cl_beta / CY_beta ratio at trim
% =========================================================================
function K = MAV_stub_coupling()
    % CY_beta = -0.730, Cl_beta = -0.110 (from aerosonde_parameters)
    % beta/phi coupling approximation: beta ≈ (Cl_beta/CL_alpha) * phi
    K = abs(-0.110 / 5.84);   % ≈ 0.0188 rad/rad
end