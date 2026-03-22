function plotMAVStateVariables(uu)
% plotMAVStateVariables.m
%   Real-time Simulink plotting callback.
%   Two figures:
%     Figure 2 — state variables (4 x 3 grid)
%     Figure 3 — course + control inputs (5 x 1)


    % ----------------------------------------------------------------
    % Unpack inputs
    % ----------------------------------------------------------------
    pn        = uu(1);
    pe        = uu(2);
    h         = -uu(3);
    phi       = 180/pi * uu(7);
    theta     = 180/pi * uu(8);
    psi       = 180/pi * uu(9);
    p         = 180/pi * uu(10);
    q_body    = 180/pi * uu(11);
    r_body    = 180/pi * uu(12);
    Va        = uu(13);
    alpha     = 180/pi * uu(14);
    beta      = 180/pi * uu(15);
    wn        = uu(16);
    we        = uu(17);
    t         = uu(54);

    pn_c      = uu(19);   pe_c    = uu(20);   h_c      = uu(21);
    Va_c      = uu(22);   alpha_c = 180/pi*uu(23); beta_c = 180/pi*uu(24);
    phi_c     = 180/pi*uu(25);  theta_c = 180/pi*uu(26); chi_c = 180/pi*uu(27);
    p_c       = 180/pi*uu(28);  q_c     = 180/pi*uu(29); r_c   = 180/pi*uu(30);

    pn_hat    = uu(31);   pe_hat  = uu(32);   h_hat    = uu(33);
    Va_hat    = uu(34);   alpha_hat = 180/pi*uu(35); beta_hat = 180/pi*uu(36);
    phi_hat   = 180/pi*uu(37);  theta_hat = 180/pi*uu(38); chi_hat = 180/pi*uu(39);
    p_hat     = 180/pi*uu(40);  q_hat   = 180/pi*uu(41); r_hat   = 180/pi*uu(42);

    delta_e   = 180/pi * uu(50);
    delta_a   = 180/pi * uu(51);
    delta_r   = 180/pi * uu(52);
    delta_t   = uu(53);

    % Course angle
    chi = 180/pi * atan2(Va*sin(uu(9)) + we, Va*cos(uu(9)) + wn);

    % ----------------------------------------------------------------
    % Persistent handles
    % ----------------------------------------------------------------
    persistent fig_states fig_ctrl
    persistent h_pn h_pe h_h
    persistent h_phi h_theta h_psi
    persistent h_p h_q h_r
    persistent h_Va h_alpha h_beta
    persistent h_chi h_de h_da h_dr h_dt

    % ----------------------------------------------------------------
    % On t==0: wipe everything and rebuild from scratch
    % This prevents stale x/y data from a previous simulation run
    % poisoning the axis ranges of the new run.
    % ----------------------------------------------------------------
    if t == 0
        % Clear all persistents so figures get rebuilt fresh
        fig_states = []; fig_ctrl = [];
        h_pn=[]; h_pe=[]; h_h=[];
        h_phi=[]; h_theta=[]; h_psi=[];
        h_p=[]; h_q=[]; h_r=[];
        h_Va=[]; h_alpha=[]; h_beta=[];
        h_chi=[]; h_de=[]; h_da=[]; h_dr=[]; h_dt=[];
    end

    % ----------------------------------------------------------------
    % Build figures on first call (t==0 OR figure was closed)
    % ----------------------------------------------------------------
    if isempty(fig_states) || ~isvalid(fig_states)

        % ---- Figure 2: States --------------------------------------
        fig_states = figure(2);
        clf(fig_states);
        set(fig_states, 'Name',     'MAV State Variables', ...
                        'Position', [20, 40, 1200, 860]);

        % Row 1 — Position
        subplot(4,3,1);  hold on;
        h_pn  = mk3(t, pn,    pn_hat,  pn_c,   'p_n (m)');

        subplot(4,3,2);  hold on;
        h_pe  = mk3(t, pe,    pe_hat,  pe_c,   'p_e (m)');

        subplot(4,3,3);  hold on;
        h_h   = mk3(t, h,     h_hat,   h_c,    'h (m)');

        % Row 2 — Attitude
        subplot(4,3,4);  hold on;
        h_phi   = mk3(t, phi,   phi_hat,   phi_c,   '\phi (deg)');

        subplot(4,3,5);  hold on;
        h_theta = mk3(t, theta, theta_hat, theta_c, '\theta (deg)');

        subplot(4,3,6);  hold on;
        h_psi   = mk1(t, psi, '\psi (deg)');

        % Row 3 — Body rates
        subplot(4,3,7);  hold on;
        h_p = mk3(t, p,      p_hat,  p_c, 'p (deg/s)');

        subplot(4,3,8);  hold on;
        h_q = mk3(t, q_body, q_hat,  q_c, 'q (deg/s)');

        subplot(4,3,9);  hold on;
        h_r = mk3(t, r_body, r_hat,  r_c, 'r (deg/s)');

        % Row 4 — Aerodynamic
        subplot(4,3,10);  hold on;
        h_Va    = mk3(t, Va,    Va_hat,    Va_c,    'V_a (m/s)');

        subplot(4,3,11);  hold on;
        h_alpha = mk3(t, alpha, alpha_hat, alpha_c, '\alpha (deg)');

        subplot(4,3,12);  hold on;
        h_beta  = mk3(t, beta,  beta_hat,  beta_c,  '\beta (deg)');

        % ---- Figure 3: Course + Controls ---------------------------
        fig_ctrl = figure(3);
        clf(fig_ctrl);
        set(fig_ctrl, 'Name',     'Control Inputs', ...
                      'Position', [1240, 40, 680, 860]);

        subplot(5,1,1);  hold on;
        h_chi = mk3(t, chi, chi_hat, chi_c, '\chi (deg)');

        subplot(5,1,2);  hold on;
        h_de  = mk1(t, delta_e, '\delta_e (deg)');

        subplot(5,1,3);  hold on;
        h_da  = mk1(t, delta_a, '\delta_a (deg)');

        subplot(5,1,4);  hold on;
        h_dr  = mk1(t, delta_r, '\delta_r (deg)');

        subplot(5,1,5);  hold on;
        h_dt  = mk1(t, delta_t, '\delta_t');

    % ----------------------------------------------------------------
    % Subsequent calls — append one point to each line
    % ----------------------------------------------------------------
    else
        app3(h_pn,    t, pn,    pn_hat,  pn_c);
        app3(h_pe,    t, pe,    pe_hat,  pe_c);
        app3(h_h,     t, h,     h_hat,   h_c);

        app3(h_phi,   t, phi,   phi_hat,   phi_c);
        app3(h_theta, t, theta, theta_hat, theta_c);
        app1(h_psi,   t, psi);

        app3(h_p,     t, p,      p_hat,  p_c);
        app3(h_q,     t, q_body, q_hat,  q_c);
        app3(h_r,     t, r_body, r_hat,  r_c);

        app3(h_Va,    t, Va,    Va_hat,    Va_c);
        app3(h_alpha, t, alpha, alpha_hat, alpha_c);
        app3(h_beta,  t, beta,  beta_hat,  beta_c);

        app3(h_chi,   t, chi,     chi_hat,  chi_c);
        app1(h_de,    t, delta_e);
        app1(h_da,    t, delta_a);
        app1(h_dr,    t, delta_r);
        app1(h_dt,    t, delta_t);

        drawnow limitrate
    end
end


% ========================================================================
%  INIT HELPERS  — create lines and label the subplot
% ========================================================================

function hdl = mk3(t, y, yhat, yd, label)
% Three lines: actual (blue solid), estimated (green dashed), commanded (red dash-dot)
    hdl(1) = plot(t, y,    'b-',  'LineWidth', 1.4);
    hdl(2) = plot(t, yhat, 'g--', 'LineWidth', 1.0);
    hdl(3) = plot(t, yd,   'r-.', 'LineWidth', 1.0);
    finish_ax(label);
    legend('actual','est.','cmd','Location','best','FontSize',8,'Box','off');
end

function hdl = mk1(t, y, label)
% Single line (psi, control surfaces)
    hdl = plot(t, y, 'b-', 'LineWidth', 1.4);
    finish_ax(label);
end

function finish_ax(label)
    ylabel(label, 'FontSize', 11, 'Rotation', 0, ...
           'HorizontalAlignment', 'right', 'VerticalAlignment', 'middle');
    xlabel('t (s)', 'FontSize', 10);
    grid on;
    set(gca, 'FontSize', 10);
    % Let MATLAB auto-scale both axes from the actual data — no manual limits
    axis auto;
end


% ========================================================================
%  UPDATE HELPERS  — append one data point
% ========================================================================

function app3(hdl, t, y, yhat, yd)
    app1(hdl(1), t, y);
    app1(hdl(2), t, yhat);
    app1(hdl(3), t, yd);
end

function app1(h, t, y)
    set(h, 'XData', [get(h,'XData'), t], ...
           'YData', [get(h,'YData'), y]);
end