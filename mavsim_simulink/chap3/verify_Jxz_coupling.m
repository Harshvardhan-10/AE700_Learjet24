% verify_jxz_coupling.m
%   Section 2(iii): Verifies gyroscopic coupling between roll and yaw
%   due to Jxz.  Uses direct numerical integration of rotational EOM
%   (no Simulink required).
%
%   Test: apply a pure roll moment (ell > 0, m = 0, n = 0).
%   Expected:
%     Jxz = 0 → rdot = Gamma4*ell = 0 → yaw rate r stays zero (no coupling)
%     Jxz ≠ 0 → rdot = Gamma4*ell ≠ 0 → r grows (coupling exists)

addpath('../parameters')
aerosonde_parameters

% Applied moments [N·m]
ell_applied = 1000;   % pure roll moment
m_applied   = 0;
n_applied   = 0;

t_end = 5;            % short simulation — just long enough to see coupling
dt    = 0.01;
t     = 0 : dt : t_end;
N     = length(t);

% -----------------------------------------------------------------------
% Helper: Gamma parameters from inertia values
% -----------------------------------------------------------------------
function G = compute_gammas(Jx, Jy, Jz, Jxz)
    G.Jx  = Jx;  G.Jy = Jy;  G.Jz = Jz;  G.Jxz = Jxz;
    G.Gam  = Jx*Jz - Jxz^2;
    G.G1   = (Jxz*(Jx - Jy + Jz)) / G.Gam;
    G.G2   = (Jz*(Jz - Jy) + Jxz^2) / G.Gam;
    G.G3   = Jz  / G.Gam;
    G.G4   = Jxz / G.Gam;
    G.G5   = (Jz - Jx) / Jy;
    G.G6   = Jxz / Jy;
    G.G7   = ((Jx - Jy)*Jx + Jxz^2) / G.Gam;
    G.G8   = Jx  / G.Gam;
end

% -----------------------------------------------------------------------
% Euler integration of [p, q, r]
% -----------------------------------------------------------------------
function pqr = integrate_rotation(G, ell, m, n, t)
    dt = t(2) - t(1);
    N  = length(t);
    pqr = zeros(3, N);
    p = 0;  q = 0;  r = 0;
    for k = 1:N
        pqr(:,k) = [p; q; r];
        pdot = G.G1*p*q - G.G2*q*r + G.G3*ell + G.G4*n;
        qdot = G.G5*p*r - G.G6*(p^2 - r^2)    + m/G.Jy;
        rdot = G.G7*p*q - G.G1*q*r + G.G4*ell + G.G8*n;
        p = p + dt*pdot;
        q = q + dt*qdot;
        r = r + dt*rdot;
    end
end

% Case 1: Jxz = 0 (no coupling expected)
G0 = compute_gammas(MAV.Jx, MAV.Jy, MAV.Jz, 0);
pqr0 = integrate_rotation(G0, ell_applied, m_applied, n_applied, t);

% Case 2: Jxz = original value (coupling expected)
G1 = compute_gammas(MAV.Jx, MAV.Jy, MAV.Jz, MAV.Jxz);
pqr1 = integrate_rotation(G1, ell_applied, m_applied, n_applied, t);

% -----------------------------------------------------------------------
% Plot
% -----------------------------------------------------------------------
figure('Name','Jxz Coupling Verification','Position',[100 100 900 600]);

subplot(2,2,1)
plot(t, pqr0(1,:)*180/pi, 'b-', 'LineWidth', 1.5); grid on
xlabel('Time (s)'); ylabel('Roll rate p (deg/s)'); 
title('Roll rate — J_{xz} = 0'); set(gca,'FontSize',12)

subplot(2,2,2)
plot(t, pqr0(3,:)*180/pi, 'r-', 'LineWidth', 1.5); grid on
xlabel('Time (s)'); ylabel('Yaw rate r (deg/s)');
title('Yaw rate — J_{xz} = 0  (should be zero)'); set(gca,'FontSize',12)

subplot(2,2,3)
plot(t, pqr1(1,:)*180/pi, 'b-', 'LineWidth', 1.5); grid on
xlabel('Time (s)'); ylabel('Roll rate p (deg/s)');
title(sprintf('Roll rate — J_{xz} = %.0f kg·m²', MAV.Jxz)); set(gca,'FontSize',12)

subplot(2,2,4)
plot(t, pqr1(3,:)*180/pi, 'r-', 'LineWidth', 1.5); grid on
xlabel('Time (s)'); ylabel('Yaw rate r (deg/s)');
title(sprintf('Yaw rate — J_{xz} = %.0f kg·m² (coupling!)', MAV.Jxz)); set(gca,'FontSize',12)

sgtitle(sprintf('Section 2(iii): Gyroscopic coupling — applied roll moment = %d N·m', ell_applied), 'FontSize', 13);

saveas(gcf, 'jxz_coupling_verification.png')
fprintf('Saved: jxz_coupling_verification.png\n');
fprintf('\nConclusion:\n');
fprintf('  Jxz=0   -> r stays at %.4f deg/s (no coupling, as expected)\n', ...
        max(abs(pqr0(3,:)))*180/pi);
fprintf('  Jxz=%.0f -> r reaches %.4f deg/s (coupling exists)\n', ...
        MAV.Jxz, max(abs(pqr1(3,:)))*180/pi);