% verify_eom.m
%   Section 2(ii): Verifies EOM by applying one nonzero force/moment
%   at a time and checking the resulting motion is physically correct.
%
%   Uses direct numerical integration of mav_dynamics.m with aerosonde MAV parameters.
%   Saves a 6-panel figure for the report.
%
%   Run from chap3/ after aerosonde_parameters has been loaded.

addpath('../parameters')
aerosonde_parameters

dt   = 0.01;
t    = (0 : dt : 5)';
N    = length(t);

% -----------------------------------------------------------------------
% Expected behaviour for each test
% -----------------------------------------------------------------------
% Test 1: fx > 0  -> u increases (accelerates forward)
% Test 2: fy > 0  -> v increases (sideslip to right)
% Test 3: fz > 0  -> w increases (nose drops, downward body velocity)
% Test 4: ell > 0 -> p increases (rolls right)
% Test 5: m > 0   -> q increases (pitches up)
% Test 6: n > 0   -> r increases (yaws right)

tests = {
    struct('name','f_x = 5000 N (forward thrust)',  'input',[5000;0;0;0;0;0], ...
           'watch',4,  'ylabel','u  (m/s)',          'expect','u increases (accelerates forward)');
    struct('name','f_y = 2000 N (side force)',       'input',[0;2000;0;0;0;0], ...
           'watch',5,  'ylabel','v  (m/s)',          'expect','v increases (drifts right)');
    struct('name','f_z = 3000 N (downward force)',   'input',[0;0;3000;0;0;0], ...
           'watch',6,  'ylabel','w  (m/s)',          'expect','w increases (nose drops)');
    struct('name','l = 5000 N·m (roll moment)',   'input',[0;0;0;5000;0;0], ...
           'watch',10, 'ylabel','p  (deg/s)',        'expect','p increases (rolls right)');
    struct('name','m = 5000 N·m (pitch moment)',     'input',[0;0;0;0;5000;0], ...
           'watch',11, 'ylabel','q  (deg/s)',        'expect','q increases (pitches up)');
    struct('name','n = 5000 N·m (yaw moment)',       'input',[0;0;0;0;0;5000], ...
           'watch',12, 'ylabel','r  (deg/s)',        'expect','r increases (yaws right)');
};

% -----------------------------------------------------------------------
% Run all 6 tests
% -----------------------------------------------------------------------
fprintf('Running EOM verification tests...\n\n');
r2d = 180/pi;

figure(10); clf;
set(gcf, 'Name', 'EOM Verification', 'Position', [50 50 1200 800]);

for k = 1:6
    tst = tests{k};
    traj = integrate(tst, MAV, t, dt);

    % Convert angular rates to deg/s for display
    idx = tst.watch;
    y   = traj(idx, :)';
    if idx >= 10   % angular rate — convert to deg/s
        y = y * r2d;
    end

    % Check first and last finite value
    y_valid = y(~isnan(y));
    if ~isempty(y_valid)
        delta = y_valid(end) - y_valid(1);
        sign_ok = (delta > 0);
        status = {'✗ UNEXPECTED','✓ CORRECT'};
        fprintf('Test %d: %-35s  delta = %+.3f  %s\n', ...
                k, tst.name, delta, status{sign_ok+1});
        fprintf('         Expected: %s\n\n', tst.expect);
    end

    subplot(2,3,k);
    plot(t, y, 'b-', 'LineWidth', 1.8);
    xlabel('Time (s)', 'FontSize', 10);
    ylabel(tst.ylabel, 'FontSize', 10);
    title(tst.name, 'FontSize', 10, 'Interpreter', 'tex');
    grid on;
    set(gca, 'FontSize', 10);

    % Annotate expected direction
    ylims = ylim;
    text(0.05*t(end), ylims(1)+0.85*(ylims(2)-ylims(1)), ...
         ['Expected: ' tst.expect], ...
         'FontSize', 8, 'Color', [0.1 0.5 0.1], 'Interpreter', 'none');
end

sgtitle('Section 2(ii) — EOM Verification: One Force/Moment at a Time', ...
        'FontSize', 13, 'FontWeight', 'bold');

saveas(gcf, 'eom_verification.png');
fprintf('Saved: eom_verification.png\n');

function traj = integrate(test, MAV, t, dt)
    N = length(t);
    x = zeros(12, N);
    % Start at rest for force tests
    x(4,1) = 0;   % start from rest to make motion obvious
    for k = 1:N-1
        xdot = mav_dynamics(t(k), x(:,k), test.input, 1, MAV);  % flag 1 = derivatives
        x(:,k+1) = x(:,k) + dt*xdot;
        if abs(x(8,k+1)) > 1.4, x(:,k+1:end) = NaN; break; end
    end
    traj = x;
end
