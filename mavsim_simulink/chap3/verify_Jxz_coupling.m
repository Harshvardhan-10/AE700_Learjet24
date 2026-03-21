% verify_jxz_coupling.m
%   Section 2(iii): Verifies roll-yaw coupling caused by Jxz.
%   Uses aerosonde MAV parameters and integrates using mav_dynamics.m.
%
%   Test: apply a pure roll moment (l > 0, m = 0, n = 0).
%   Expected:
%     Jxz = 0 → rdot = Gamma4*ell = 0 → yaw rate r stays zero (no coupling)
%     Jxz ≠ 0 → rdot = Gamma4*ell ≠ 0 → r grows (coupling exists)

addpath('../parameters')
aerosonde_parameters

% Applied moments [N*m]
l_applied = 1000;   % pure roll moment
m_applied = 0;
n_applied = 0;

t_end = 5;
dt    = 0.01;
t     = 0:dt:t_end;

% Force/moment vector expected by mav_dynamics: [fx; fy; fz; l; m; n]
uu = [0; 0; 0; l_applied; m_applied; n_applied];

% Build two MAV cases that differ only by Jxz and dependent Gamma terms
MAV_no_coupling = set_Jxz_and_gammas(MAV, 0);
MAV_with_coupling = set_Jxz_and_gammas(MAV, MAV.Jxz);

% Initial state at rest [pn pe pd u v w phi theta psi p q r]'
x0 = zeros(12,1);

% Integrate with shared dynamics model
X0 = integrate_with_mav_dynamics(MAV_no_coupling, x0, uu, t, dt);
X1 = integrate_with_mav_dynamics(MAV_with_coupling, x0, uu, t, dt);

% Extract rotational rates [p; q; r]
pqr0 = X0(10:12, :);
pqr1 = X1(10:12, :);

% -----------------------------------------------------------------------
% Plot
% -----------------------------------------------------------------------
figure('Name','Jxz Coupling Verification','Position',[100 100 900 600]);

subplot(2,2,1)
plot(t, pqr0(1,:)*180/pi, 'b-', 'LineWidth', 1.5); grid on
xlabel('Time (s)'); ylabel('Roll rate p (deg/s)');
title('Roll rate - J_{xz} = 0'); set(gca,'FontSize',12)

subplot(2,2,2)
plot(t, pqr0(3,:)*180/pi, 'r-', 'LineWidth', 1.5); grid on
xlabel('Time (s)'); ylabel('Yaw rate r (deg/s)');
title('Yaw rate - J_{xz} = 0 (should be near zero)'); set(gca,'FontSize',12)

subplot(2,2,3)
plot(t, pqr1(1,:)*180/pi, 'b-', 'LineWidth', 1.5); grid on
xlabel('Time (s)'); ylabel('Roll rate p (deg/s)');
title(sprintf('Roll rate - J_{xz} = %.4f kg*m^2', MAV.Jxz)); set(gca,'FontSize',12)

subplot(2,2,4)
plot(t, pqr1(3,:)*180/pi, 'r-', 'LineWidth', 1.5); grid on
xlabel('Time (s)'); ylabel('Yaw rate r (deg/s)');
title(sprintf('Yaw rate - J_{xz} = %.4f kg*m^2 (coupling)', MAV.Jxz)); set(gca,'FontSize',12)

sgtitle(sprintf('Section 2(iii): Gyroscopic coupling - applied roll moment = %d N*m', l_applied), ...
        'FontSize', 13);

saveas(gcf, 'jxz_coupling_verification.png')
fprintf('Saved: jxz_coupling_verification.png\n');
fprintf('\nConclusion:\n');
fprintf('  Jxz=0      -> max |r| = %.4f deg/s (no coupling)\n', ...
        max(abs(pqr0(3,:)))*180/pi);
fprintf('  Jxz=%.4f -> max |r| = %.4f deg/s (coupling exists)\n', ...
        MAV.Jxz, max(abs(pqr1(3,:)))*180/pi);

function X = integrate_with_mav_dynamics(MAVcase, x0, uu, t, dt)
    N = numel(t);
    X = zeros(12, N);
    X(:,1) = x0;

    for k = 1:N-1
        xdot = mav_dynamics(t(k), X(:,k), uu, 1, MAVcase);  % flag 1: derivatives
        X(:,k+1) = X(:,k) + dt*xdot;
    end
end

function MAVout = set_Jxz_and_gammas(MAVin, Jxz_new)
    MAVout = MAVin;
    MAVout.Jxz = Jxz_new;

    G = MAVout.Jx*MAVout.Jz - MAVout.Jxz^2;
    MAVout.Gamma  = G;
    MAVout.Gamma1 = (MAVout.Jxz*(MAVout.Jx - MAVout.Jy + MAVout.Jz)) / G;
    MAVout.Gamma2 = (MAVout.Jz*(MAVout.Jz - MAVout.Jy) + MAVout.Jxz^2) / G;
    MAVout.Gamma3 = MAVout.Jz / G;
    MAVout.Gamma4 = MAVout.Jxz / G;
    MAVout.Gamma5 = (MAVout.Jz - MAVout.Jx) / MAVout.Jy;
    MAVout.Gamma6 = MAVout.Jxz / MAVout.Jy;
    MAVout.Gamma7 = ((MAVout.Jx - MAVout.Jy)*MAVout.Jx + MAVout.Jxz^2) / G;
    MAVout.Gamma8 = MAVout.Jx / G;
end