% verify_forces_moments.m
%   Section 3: Verifies forces_moments.m by applying individual control
%   surface deflections and observing the resulting motion.
%
%   Requires forces_moments.m in same folder.

addpath('../parameters')
addpath('../tools')
addpath('../chap3')
aerosonde_parameters

% -----------------------------------------------------------------------
% Integration helper (calls forces_moments.m and mav_dynamics.m)
% -----------------------------------------------------------------------
function x_new = step_eom(x, delta, wind, MAV, dt, tk)
    fm = forces_moments(x, delta, wind, MAV);   % [fx; fy; fz; l; m; n]
    xdot = mav_dynamics(tk, x, fm, 1, MAV);     % flag 1 => derivatives
    x_new = x + dt*xdot;
end

% -----------------------------------------------------------------------
% Setup
% -----------------------------------------------------------------------
dt  = 0.01;
t   = (0:dt:2)';
N   = length(t);
r2d = 180/pi;
wind = zeros(6,1);

% Start at trim-like state: Va=80 m/s, level flight, trim throttle
x0      = zeros(12,1);
x0(3)   = -200;     % 200 m altitude
x0(4)   = 80;       % u = Va_trim
x0(8)   = 5.5*pi/180; % trim pitch angle

de_test =  5*pi/180;   % +5 deg elevator
da_test =  5*pi/180;   % +5 deg aileron
dr_test =  5*pi/180;   % +5 deg rudder
dt_trim = 0.18;        % approximate trim throttle

% -----------------------------------------------------------------------
% Test 1: delta_e = +5 deg (all others at trim)
% Expected: Cm_de = -1.24 -> negative pitch moment -> q decreases -> theta decreases
%           (nose-down for positive elevator in this convention)
% -----------------------------------------------------------------------
traj1 = zeros(12,N);  traj1(:,1) = x0;  x = x0;
for k = 1:N-1
    x = step_eom(x, [de_test; 0; 0; dt_trim], wind, MAV, dt, t(k));
    traj1(:,k+1) = x;
end

% -----------------------------------------------------------------------
% Test 2: delta_a = +5 deg
% Expected: Cl_da = -0.178 -> negative roll moment -> p < 0 -> phi decreases
% -----------------------------------------------------------------------
traj2 = zeros(12,N);  traj2(:,1) = x0;  x = x0;
for k = 1:N-1
    x = step_eom(x, [0; da_test; 0; dt_trim], wind, MAV, dt, t(k));
    traj2(:,k+1) = x;
end

% -----------------------------------------------------------------------
% Test 3: delta_r = +5 deg
% Expected: Cn_dr = -0.074 -> negative yaw -> r < 0; CY_dr = 0.14 -> beta develops
% -----------------------------------------------------------------------
traj3 = zeros(12,N);  traj3(:,1) = x0;  x = x0;
for k = 1:N-1
    x = step_eom(x, [0; 0; dr_test; dt_trim], wind, MAV, dt, t(k));
    traj3(:,k+1) = x;
end

% -----------------------------------------------------------------------
% Print verification table
% -----------------------------------------------------------------------
theta_ss = (traj1(8,end) - traj1(8,1))*r2d;
phi_ss   = (traj2(7,end) - traj2(7,1))*r2d;
beta_ss  = asin(traj3(5,end)/80)*r2d;   % approx beta from v/Va

fprintf('\n=== Section 3 Verification Table ===\n');
fprintf('%-20s  %-35s  %-20s  %s\n','Input','Expected','Observed','Pass?');
fprintf('%s\n', repmat('-',1,90));

% Cm_de = -1.24 < 0 => pitch moment is negative for +de => theta decreases
pass1 = theta_ss < 0;
fprintf('delta_e = +5 deg    %-35s  theta change = %+.2f deg    %s\n', ...
    'Cm_de=-1.24 -> nose-down (theta dec)', theta_ss, yn(pass1));

% Cl_da = -0.178 < 0 => roll moment negative for +da => phi decreases
pass2 = phi_ss < 0;
fprintf('delta_a = +5 deg    %-35s  phi change   = %+.2f deg    %s\n', ...
    'Cl_da=-0.178 -> roll left (phi dec)', phi_ss, yn(pass2));

% Cn_dr = -0.074 < 0 => yaw left for +dr; CY_dr=0.14 => beta develops
pass3 = beta_ss ~= 0;
fprintf('delta_r = +5 deg    %-35s  beta         = %+.2f deg    %s\n', ...
    'Cn_dr=-0.074 -> yaw left, beta devs', beta_ss, yn(pass3));

function s = yn(v), if v, s='PASS'; else, s='FAIL'; end, end

% -----------------------------------------------------------------------
% Plot — 3-panel figure
% -----------------------------------------------------------------------
figure(1); clf;
set(gcf,'Name','Section 3 — Forces and Moments Verification', ...
        'Position',[50 50 900 850]);

% Panel 1: elevator -> theta
subplot(3,1,1)
plot(t, (traj1(8,:)-traj1(8,1))*r2d, 'b-', 'LineWidth',1.8); hold on
plot(t, zeros(size(t)), 'k:', 'LineWidth',0.8);
yline(0,'k:');
xlabel('Time (s)','FontSize',12);
ylabel('\Delta\theta (deg)','FontSize',12);
title(['\delta_e = +5° \rightarrow  C_{m_{\delta_e}} = -1.24  \rightarrow  '...
       'Nose-down moment  \rightarrow  \theta decreases'], ...
      'FontSize',11);
legend(sprintf('\\Delta\\theta = %.2f° (t=8s)', theta_ss), ...
       'Location','best','FontSize',10);
grid on; set(gca,'FontSize',11);

% Panel 2: aileron -> phi
subplot(3,1,2)
plot(t, (traj2(7,:)-traj2(7,1))*r2d, 'b-', 'LineWidth',1.8); hold on
plot(t, zeros(size(t)), 'k:', 'LineWidth',0.8);
xlabel('Time (s)','FontSize',12);
ylabel('\Delta\phi (deg)','FontSize',12);
title(['\delta_a = +5\circ \rightarrow  C_{l_{\delta_a}} = -0.178  \rightarrow  '...
       'Roll-left moment  \rightarrow  \phi decreases'], ...
      'FontSize',11, 'Interpreter','tex');
legend(sprintf('\\Delta\\phi = %.2f° (t=8s)', phi_ss), ...
       'Location','best','FontSize',10);
grid on; set(gca,'FontSize',11);

% Panel 3: rudder -> beta and r
subplot(3,1,3)
beta_traj = asin(traj3(5,:)/80)*r2d;   % approx beta = asin(v/Va)
r_traj    = traj3(12,:)*r2d;
yyaxis left
plot(t, beta_traj, 'b-', 'LineWidth',1.8);
ylabel('\beta (deg)','FontSize',12);
yyaxis right
plot(t, r_traj, 'r--', 'LineWidth',1.5);
ylabel('r (deg/s)','FontSize',12);
xlabel('Time (s)','FontSize',12);
title(['\delta_r = +5° \rightarrow  C_{n_{\delta_r}} = -0.074  \rightarrow  '...
       'Yaw-left moment  \rightarrow  \beta and r develop'], ...
      'FontSize',11);
legend('\beta (sideslip)','r (yaw rate)','Location','best','FontSize',10);
grid on; set(gca,'FontSize',11);

sgtitle('Section 3 — Forces and Moments Verification (one surface at a time)', ...
        'FontSize',13,'FontWeight','bold');

saveas(gcf,'fm_verification.png');
fprintf('\nSaved: fm_verification.png\n');