% run_autopilot_plots.m
%   Generates all 9 required Section 5 autopilot report plots.
%
%   ONE-TIME SETUP in Simulink (do this before running):
%   -------------------------------------------------------
%   You need 3 "To Workspace" blocks added manually in mavsim_chap6.slx:
%
%     Block 1: connected to the output wire of the MAV block (states)
%              Variable name = x_out,        Format = Array
%
%     Block 2: connected to output port 1 of Autopilot block (delta)
%              Variable name = delta_out,    Format = Array
%
%     Block 3: connected to output port 2 of Autopilot block (x_command)
%              Variable name = xcommand_out, Format = Array
%
%   To add a To Workspace block:
%     - In Simulink, go to Library Browser -> Sinks -> To Workspace
%     - Drag it onto the canvas, connect it to the wire, set variable name
%
%   Also make sure in Model Settings (Ctrl+E) -> Data Import/Export:
%     "Time" is ticked, variable name = tout
%
%   After setup, just run:  >> run_autopilot_plots
% -----------------------------------------------------------------------

if ~exist('AP','var') || ~exist('Va_trim','var')
    error('Run compute_autopilot_gains.m first to load AP and Va_trim.');
end

mdl = 'mavsim_chap6';
load_system(mdl);

% -----------------------------------------------------------------------
%  SIMULATION 1: Lateral — no disturbance
%  Course step: 0 -> 30 deg at t=5s
% -----------------------------------------------------------------------
fprintf('=== Sim 1/3: Lateral, no disturbance ===\n');
set_steps(mdl, Va_trim, Va_trim, 100, 100, 30, 0);
sim(mdl);
check_workspace();
lat_nd = extract_signals(tout);
fprintf('Done.\n');

% -----------------------------------------------------------------------
%  SIMULATION 2: Lateral — with disturbance (0.2 at t=10s)
% -----------------------------------------------------------------------
fprintf('=== Sim 2/3: Lateral, with disturbance ===\n');
set_steps(mdl, Va_trim, Va_trim, 100, 100, 30, 0.2);
sim(mdl);
lat_d = extract_signals(tout);
fprintf('Done.\n');

% -----------------------------------------------------------------------
%  SIMULATION 3: Longitudinal
%  Altitude: 100->150m,  Airspeed: Va_trim -> Va_trim+10 m/s,  at t=5s
% -----------------------------------------------------------------------
fprintf('=== Sim 3/3: Longitudinal ===\n');
set_steps(mdl, Va_trim, Va_trim+10, 100, 150, 0, 0);
sim(mdl);
lon = extract_signals(tout);
fprintf('Done.\n');

% -----------------------------------------------------------------------
%  Plot all 9 figures
% -----------------------------------------------------------------------
make_all_plots(lat_nd, lat_d, lon, AP);
fprintf('\nFigures 1-9 generated.\n');


% =======================================================================
%  HELPER FUNCTIONS
% =======================================================================

function set_steps(mdl, Va_i, Va_f, h_i, h_f, chi_f_deg, dist_mag)
% Sets step block values by searching block names.
% Prints a warning (not an error) if a block is not found —
% in that case set it manually in Simulink.
    blocks = find_system(mdl, 'BlockType', 'Step');
    found  = struct('va',0,'h',0,'chi',0,'dist',0);

    for k = 1:numel(blocks)
        nm = lower(get_param(blocks{k}, 'Name'));

        if contains(nm,'airspeed') || (contains(nm,'va') && ~contains(nm,'mav'))
            set_param(blocks{k},'Before',num2str(Va_i),'After',num2str(Va_f),'Time','5');
            found.va = 1;

        elseif contains(nm,'altitude') || contains(nm,'h_c') || strcmp(nm,'h')
            set_param(blocks{k},'Before',num2str(h_i),'After',num2str(h_f),'Time','5');
            found.h = 1;

        elseif contains(nm,'course') || contains(nm,'chi')
            set_param(blocks{k},'Before','0','After',num2str(chi_f_deg),'Time','5');
            found.chi = 1;

        elseif contains(nm,'dist') || contains(nm,'disturb')
            set_param(blocks{k},'Before','0','After',num2str(dist_mag),'Time','10');
            found.dist = 1;
        end
    end

    % Report anything not found so user knows to set manually
    if ~found.va,   fprintf('  [!] Airspeed step block not found — set Va manually.\n'); end
    if ~found.h,    fprintf('  [!] Altitude step block not found — set h manually.\n');  end
    if ~found.chi,  fprintf('  [!] Course step block not found — set chi manually.\n'); end
    if ~found.dist, fprintf('  [!] Disturbance block not found — dist=%.2f ignored.\n', dist_mag); end
end


function check_workspace()
% Tells the user clearly if To Workspace blocks are missing
    needed = {'x_out','delta_out','xcommand_out','tout'};
    missing = {};
    for k = 1:numel(needed)
        if ~evalin('base', sprintf('exist(''%s'',''var'')', needed{k}))
            missing{end+1} = needed{k}; %#ok<AGROW>
        end
    end
    if ~isempty(missing)
        error([...
            'Missing workspace variables after simulation: %s\n\n' ...
            'You need To Workspace blocks in mavsim_chap6.slx:\n' ...
            '  x_out       <- wire from MAV output (12 states)\n' ...
            '  delta_out   <- wire from Autopilot port 1 (4 controls)\n' ...
            '  xcommand_out<- wire from Autopilot port 2 (12 commands)\n\n' ...
            'Drag "To Workspace" from Library Browser -> Sinks,\n' ...
            'connect to each wire, set Variable Name and Format=Array.\n' ...
            'Then re-run this script.'], strjoin(missing, ', '));
    end
end


function s = extract_signals(tout)
% Reads x_out, delta_out, xcommand_out from base workspace.
    x        = evalin('base','x_out');
    delta    = evalin('base','delta_out');
    xcommand = evalin('base','xcommand_out');

    % Trim to shortest length (Simulink off-by-one protection)
    n = min([numel(tout), size(x,1), size(delta,1), size(xcommand,1)]);
    s.t        = tout(1:n);
    x          = x(1:n,:);
    delta      = delta(1:n,:);
    xcommand   = xcommand(1:n,:);

    % States  [pn pe pd u v w phi theta psi p q r]
    s.h     = -x(:,3);
    Va_mag  = max(sqrt(x(:,4).^2 + x(:,5).^2 + x(:,6).^2), 0.1);
    s.Va    = Va_mag;
    s.phi   = x(:,7);
    s.theta = x(:,8);
    s.psi   = x(:,9);
    s.chi   = x(:,9);                                   % chi ≈ psi (no estimator)
    s.beta  = asin(min(max(x(:,5)./Va_mag,-1),1));      % sideslip

    % Controls  [delta_e  delta_a  delta_r  delta_t]
    s.delta_e = delta(:,1);
    s.delta_a = delta(:,2);
    s.delta_r = delta(:,3);
    s.delta_t = delta(:,4);

    % Commands from autopilot x_command:
    % [0; 0; h_c; Va_c; 0; 0; phi_c; theta_c; chi_c; 0; 0; 0]
    s.h_c     = xcommand(:,3);
    s.Va_c    = xcommand(:,4);
    s.phi_c   = xcommand(:,7);
    s.theta_c = xcommand(:,8);
    s.chi_c   = xcommand(:,9);
end


function make_all_plots(lat_nd, lat_d, lon, AP)

    r2d = 180/pi;
    set(0,'DefaultAxesFontSize',12,'DefaultLineLineWidth',1.8);

    blue   = [0.00 0.45 0.70];
    red    = [0.85 0.33 0.10];
    green  = [0.47 0.67 0.19];
    purple = [0.49 0.18 0.56];

    % ------ Fig 1: Roll, no disturbance ----------------------------------
    figure(1); clf;
    plot(lat_nd.t, lat_nd.phi*r2d,   '-',  'Color',blue,  'DisplayName','\phi (actual)');
    hold on;
    plot(lat_nd.t, lat_nd.phi_c*r2d, '--', 'Color',red,   'DisplayName','\phi_c (command)');
    grid on; legend('Location','best');
    xlabel('Time (s)'); ylabel('Roll angle (deg)');
    title('Roll angle tracking — no disturbance');

    % ------ Fig 2: Roll, with disturbance --------------------------------
    figure(2); clf;
    plot(lat_d.t, lat_d.phi*r2d,   '-',  'Color',blue,  'DisplayName','\phi (actual)');
    hold on;
    plot(lat_d.t, lat_d.phi_c*r2d, '--', 'Color',red,   'DisplayName','\phi_c (command)');
    xline(10,'k:','Disturbance at t=10 s','LabelVerticalAlignment','bottom');
    grid on; legend('Location','best');
    xlabel('Time (s)'); ylabel('Roll angle (deg)');
    title('Roll angle tracking — disturbance = 0.2 step at t = 10 s');

    % ------ Fig 3: Course angle ------------------------------------------
    figure(3); clf;
    plot(lat_nd.t, lat_nd.chi*r2d,   '-',  'Color',blue, 'DisplayName','\chi (actual)');
    hold on;
    plot(lat_nd.t, lat_nd.chi_c*r2d, '--', 'Color',red,  'DisplayName','\chi_c (command)');
    grid on; legend('Location','best');
    xlabel('Time (s)'); ylabel('Course angle (deg)');
    title('Course angle tracking');

    % ------ Fig 4: Sideslip ----------------------------------------------
    figure(4); clf;
    plot(lat_nd.t, lat_nd.beta*r2d, '-', 'Color',blue, 'DisplayName','\beta (actual)');
    hold on;
    yline(0,'--','Color',red,'DisplayName','\beta_c = 0');
    grid on; legend('Location','best');
    xlabel('Time (s)'); ylabel('Sideslip angle (deg)');
    title('Sideslip angle (\beta_c = 0)');

    % ------ Fig 5: Lateral control inputs --------------------------------
    figure(5); clf;
    subplot(2,1,1);
    plot(lat_nd.t, lat_nd.delta_a*r2d, '-',  'Color',blue,  'DisplayName','\delta_a no dist');
    hold on;
    plot(lat_d.t,  lat_d.delta_a*r2d,  '--', 'Color',green, 'DisplayName','\delta_a with dist');
    yline( AP.delta_a_max*r2d,'k:','Sat +'); yline(-AP.delta_a_max*r2d,'k:','Sat −');
    grid on; legend('Location','best');
    ylabel('\delta_a (deg)'); title('Lateral control inputs');

    subplot(2,1,2);
    plot(lat_nd.t, lat_nd.delta_r*r2d, '-',  'Color',blue,  'DisplayName','\delta_r no dist');
    hold on;
    plot(lat_d.t,  lat_d.delta_r*r2d,  '--', 'Color',green, 'DisplayName','\delta_r with dist');
    yline( AP.delta_r_max*r2d,'k:','Sat +'); yline(-AP.delta_r_max*r2d,'k:','Sat −');
    grid on; legend('Location','best');
    xlabel('Time (s)'); ylabel('\delta_r (deg)');

    % ------ Fig 6: Pitch angle -------------------------------------------
    figure(6); clf;
    plot(lon.t, lon.theta*r2d,   '-',  'Color',blue, 'DisplayName','\theta (actual)');
    hold on;
    plot(lon.t, lon.theta_c*r2d, '--', 'Color',red,  'DisplayName','\theta_c (command)');
    grid on; legend('Location','best');
    xlabel('Time (s)'); ylabel('Pitch angle (deg)');
    title('Pitch angle tracking');

    % ------ Fig 7: Altitude ----------------------------------------------
    figure(7); clf;
    plot(lon.t, lon.h,   '-',  'Color',blue, 'DisplayName','h (actual)');
    hold on;
    plot(lon.t, lon.h_c, '--', 'Color',red,  'DisplayName','h_c (command)');
    grid on; legend('Location','best');
    xlabel('Time (s)'); ylabel('Altitude (m)');
    title('Altitude tracking');

    % ------ Fig 8: Airspeed via throttle ---------------------------------
    figure(8); clf;
    yyaxis left;
    plot(lon.t, lon.Va,   '-',  'Color',blue,   'DisplayName','V_a (actual)');
    hold on;
    plot(lon.t, lon.Va_c, '--', 'Color',red,    'DisplayName','V_{a,c} (command)');
    ylabel('Airspeed (m/s)');
    yyaxis right;
    plot(lon.t, lon.delta_t, '-', 'Color',purple, 'DisplayName','\delta_t (throttle)');
    ylabel('Throttle \delta_t');
    xlabel('Time (s)'); title('Airspeed tracking — throttle control');
    legend('Location','best'); grid on;

    % ------ Fig 9: Airspeed via pitch command ----------------------------
    figure(9); clf;
    yyaxis left;
    plot(lon.t, lon.Va,   '-',  'Color',blue,   'DisplayName','V_a (actual)');
    hold on;
    plot(lon.t, lon.Va_c, '--', 'Color',red,    'DisplayName','V_{a,c} (command)');
    ylabel('Airspeed (m/s)');
    yyaxis right;
    plot(lon.t, lon.theta_c*r2d, '-', 'Color',purple, 'DisplayName','\theta_c (pitch cmd)');
    ylabel('Pitch command \theta_c (deg)');
    xlabel('Time (s)'); title('Airspeed response via pitch command');
    legend('Location','best'); grid on;

end