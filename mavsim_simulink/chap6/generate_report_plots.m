% generate_report_plots.m
% generates all 10 
clear; clc;
addpath('../chap5')
addpath('../chap6')
addpath('../parameters')
load('autopilot_sim_results.mat');
load('autopilot_gains.mat');

FS  = 12; LW  = 1.8;
W=800; H=500;
da_lim = AP.delta_a_max*r2d;
de_lim = AP.delta_e_max*r2d;
dr_lim = AP.delta_r_max*r2d;

function tracked(t, actual, cmd_val, ylbl, albl, clbl, ttl, fs, lw)
    plot(t, actual, 'b-', 'LineWidth',lw, 'DisplayName',albl); hold on
    plot(t, cmd_val*ones(size(t)), 'r--', 'LineWidth',lw*0.8, 'DisplayName',clbl);
    xlabel('Time (s)','FontSize',fs); ylabel(ylbl,'FontSize',fs);
    title(ttl,'FontSize',fs); legend('Location','best','FontSize',fs-2);
    grid on; set(gca,'FontSize',fs);
end

function ctrl(t, u, ylbl, ttl, fs, lw, lim_val, lname)
    plot(t, u, 'k-', 'LineWidth',lw, 'DisplayName',lname); hold on
    if ~isempty(lim_val)
        yline( lim_val,'r--','LineWidth',0.8,'DisplayName','+limit');
        yline(-lim_val,'r--','LineWidth',0.8,'DisplayName','-limit');
    end
    xlabel('Time (s)','FontSize',fs); ylabel(ylbl,'FontSize',fs);
    title(ttl,'FontSize',fs); legend('Location','best','FontSize',fs-2);
    grid on; set(gca,'FontSize',fs);
end

% FIG 1 — Roll angle
figure(1); clf; set(gcf,'Position',[50 500 W H]);
plot(T1, phi1, 'b-', 'LineWidth',LW, 'DisplayName','Actual \phi'); hold on
plot(T1, phic1, 'r--', 'LineWidth',LW*0.8, 'DisplayName','\phi_c (from course PI)');
xlabel('Time (s)','FontSize',FS); ylabel('\phi (deg)','FontSize',FS);
title('Roll angle response, no disturbance  (\chi_c = 45°)','FontSize',FS);
legend('Location','best','FontSize',FS-2); grid on; set(gca,'FontSize',FS);
saveas(gcf,'fig1_roll_no_disturbance.png');

% FIG 2 — Roll with disturbance
figure(2); clf; set(gcf,'Position',[50 500 W H]);
plot(T2, phi2, 'b-', 'LineWidth',LW, 'DisplayName','Actual \phi'); hold on
plot(T2, phic2, 'r--', 'LineWidth',LW*0.8, 'DisplayName','\phi_c');
xline(30, 'g:', 'LineWidth',1.2, 'DisplayName','Disturbance onset');
xlabel('Time (s)','FontSize',FS); ylabel('\phi (deg)','FontSize',FS);
title('Roll angle WITH disturbance  (step 0.2 rad at t = 30 s)','FontSize',FS);
legend('Location','best','FontSize',FS-2); grid on; set(gca,'FontSize',FS);
saveas(gcf,'fig2_roll_with_disturbance.png');

% FIG 3 — Course
figure(3); clf; set(gcf,'Position',[50 500 W H]);
tracked(T1, chi1, 45, '\chi (deg)', 'Actual \chi', '\chi_c = 45°', ...
    'Course angle response  (\chi_c = 45°)', FS, LW);
saveas(gcf,'fig3_course.png');

% FIG 4 — Sideslip
figure(4); clf; set(gcf,'Position',[50 500 W H]);
tracked(T1, beta1, 0, '\beta (deg)', 'Actual \beta', '\beta_c = 0', ...
    'Sideslip \beta during course step  (\chi_c = 45°, \beta_c = 0)', FS, LW);
saveas(gcf,'fig4_sideslip.png');

% FIG 5 — Lateral Inputs
figure(5); clf; set(gcf,'Position',[50 50 W 750]);
subplot(3,1,1); ctrl(T1, da1, '\delta_a (deg)', 'Aileron \delta_a — course step (\chi_c = 45°)', FS, LW, da_lim, '\delta_a');
subplot(3,1,2); ctrl(T1, dr1, '\delta_r (deg)', 'Rudder \delta_r — yaw damper', FS, LW, dr_lim, '\delta_r');
subplot(3,1,3); ctrl(T1, phic1, '\phi_c (deg)', 'Roll command \phi_c — from course PI', FS, LW, AP.phi_c_max*r2d, '\phi_c');
sgtitle('Lateral autopilot control inputs','FontSize',FS+1,'FontWeight','bold');
saveas(gcf,'fig5_lateral_inputs.png');

% FIG 6 — Pitch (Using T5 / Sim 5 6-DOF Data)
figure(6); clf; set(gcf,'Position',[50 500 W H]);
tracked(T5, theta5, 5, '\theta (deg)', 'Actual \theta', '\theta_c = 5°', ...
    'Pitch response  (\theta_c = 5°)', FS, LW);
saveas(gcf,'fig6_pitch.png');

% FIG 7 — Altitude (Using T3 / Sim 3 6-DOF Data)
figure(7); clf; set(gcf,'Position',[50 500 W H]);
tracked(T3, h3, h_trim+20, 'h (m)', 'Actual h', 'h_c = trim + 20 m', ...
    'Altitude response  (\Deltah = 20 m)', FS, LW);
saveas(gcf,'fig7_altitude.png');

% FIG 8 — Airspeed via Throttle (Using T4 / Sim 4 6-DOF Data)
figure(8); clf; set(gcf,'Position',[50 500 W 600]);
subplot(2,1,1); tracked(T4, Va4, Va_trim+5, 'V_a (m/s)', 'Actual V_a', sprintf('V_{a,c} = %.0f m/s', Va_trim+5), ...
    'Airspeed via throttle  (\DeltaV_a = 5 m/s)', FS, LW);
subplot(2,1,2); ctrl(T4, dt4, '\delta_t', 'Throttle \delta_t', FS, LW, [], '\delta_t');
yline(1,'r--','LineWidth',0.8,'DisplayName','+limit'); yline(0,'b:','LineWidth',0.8,'DisplayName','0');
legend('Location','best','FontSize',FS-2); set(gca,'FontSize',FS);
sgtitle('Airspeed via throttle','FontSize',FS+1,'FontWeight','bold');
saveas(gcf,'fig8_airspeed_throttle.png');

% FIG 9 — Airspeed via Pitch (Using T6 / Sim 6 6-DOF Data)
figure(9); clf; set(gcf,'Position',[50 500 W 600]);
subplot(2,1,1); tracked(T6, Va6, Va_trim+5, 'V_a (m/s)', 'Actual V_a', sprintf('V_{a,c} = %.0f m/s', Va_trim+5), ...
    'Airspeed via pitch  (\DeltaV_a = 5 m/s, \delta_t fixed)', FS, LW);
subplot(2,1,2); ctrl(T6, thetac6, '\theta_c (deg)', 'Pitch command \theta_c from airspeed/pitch controller', FS, LW, [], '\theta_c');
set(gca,'FontSize',FS);
sgtitle('Airspeed via pitch','FontSize',FS+1,'FontWeight','bold');
saveas(gcf,'fig9_airspeed_pitch.png');

% FIG 10 — Longitudinal Inputs
figure(10); clf; set(gcf,'Position',[50 50 W 600]);
subplot(2,1,1)
plot(T3, de3,   'k-',  'LineWidth',LW,     'DisplayName','\delta_e (alt step)'); hold on
plot(T5, de5, 'b--', 'LineWidth',LW*0.8, 'DisplayName','\delta_e (pitch step)');
yline( de_lim,'r--','LineWidth',0.8,'DisplayName','+limit');
yline(-de_lim,'r--','LineWidth',0.8,'DisplayName','-limit');
xlabel('Time (s)','FontSize',FS); ylabel('\delta_e (deg)','FontSize',FS);
title('Elevator \delta_e','FontSize',FS);
legend('Location','best','FontSize',FS-2); grid on; set(gca,'FontSize',FS);

subplot(2,1,2)
plot(T4, dt4, 'k-', 'LineWidth',LW, 'DisplayName','\delta_t (airspeed step)'); hold on
yline(1,'r--','LineWidth',0.8,'DisplayName','+limit');
yline(0,'b:','LineWidth',0.8,'DisplayName','0');
xlabel('Time (s)','FontSize',FS); ylabel('\delta_t','FontSize',FS);
title('Throttle \delta_t','FontSize',FS);
legend('Location','best','FontSize',FS-2); grid on; set(gca,'FontSize',FS);
sgtitle('Longitudinal autopilot control inputs','FontSize',FS+1,'FontWeight','bold');
saveas(gcf,'fig10_longitudinal_inputs.png');

fprintf('\nAll 10 figures saved from pure 6-DOF simulation.\n');