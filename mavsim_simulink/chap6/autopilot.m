function y = autopilot(uu, AP)
    % ----------------------------------------------------------------
    % Unpack inputs
    % ----------------------------------------------------------------
    NN = 0;
    pn    = uu(1+NN);   pe   = uu(2+NN);   h   = uu(3+NN);
    Va    = uu(4+NN);   alpha= uu(5+NN);   beta= uu(6+NN);
    phi   = uu(7+NN);   theta= uu(8+NN);   chi = uu(9+NN);
    p     = uu(10+NN);  q    = uu(11+NN);  r   = uu(12+NN);
    Vg    = uu(13+NN);  wn   = uu(14+NN);  we  = uu(15+NN);
    psi   = uu(16+NN);
    bx    = uu(17+NN);  by   = uu(18+NN);  bz  = uu(19+NN);
    NN    = NN + 19;
    Va_c  = uu(1+NN);   h_c  = uu(2+NN);   chi_c = uu(3+NN);
    NN    = NN + 3;
    t     = uu(1+NN);
    
    % Check for longitudinal mode flag (default to 1 if not provided)
    if length(uu) >= 24
        long_mode = uu(24);
    else
        long_mode = 1;
    end

    % ----------------------------------------------------------------
    % LATERAL AUTOPILOT
    % ----------------------------------------------------------------
    chi_ref = wrap(chi_c, chi);

    if t == 0
        phi_c   = phi;
        delta_r = yaw_damper(r, 1, AP);
        course_with_roll(chi_ref, chi, 1, AP);
    else
        phi_c   = course_with_roll(chi_ref, chi, 0, AP);
        delta_r = yaw_damper(r, 0, AP);
    end

    delta_a = roll_with_aileron(phi_c, phi, p, AP);

    % ----------------------------------------------------------------
    % LONGITUDINAL AUTOPILOT MODES
    % ----------------------------------------------------------------
    h_ref = sat(h_c, h + AP.altitude_zone, h - AP.altitude_zone);

    if long_mode == 1 
        % MODE 1: Standard (Alt -> Pitch, Va -> Throttle)
        if t == 0
            delta_t = airspeed_with_throttle(Va_c, Va, 1, AP);
            theta_c = altitude_with_pitch(h_ref, h, 1, AP);
            airspeed_with_pitch(Va_c, Va, 1, AP); % init
        else
            delta_t = airspeed_with_throttle(Va_c, Va, 0, AP);
            theta_c = altitude_with_pitch(h_ref, h, 0, AP);
        end
        
    elseif long_mode == 2 
        % MODE 2: Pure Pitch Step (Bypass alt loop, h_c carries theta_c)
        theta_c = h_c; 
        if t == 0
            delta_t = airspeed_with_throttle(Va_c, Va, 1, AP);
        else
            delta_t = airspeed_with_throttle(Va_c, Va, 0, AP);
        end
        
    elseif long_mode == 3 
        % MODE 3: Airspeed via Pitch (Throttle fixed at trim)
        delta_t = AP.delta_t_trim;
        if t == 0
            theta_c = airspeed_with_pitch(Va_c, Va, 1, AP);
        else
            theta_c = airspeed_with_pitch(Va_c, Va, 0, AP);
        end
    end

    % Inner pitch loop always runs
    delta_e = pitch_with_elevator(theta_c, theta, q, AP);
    delta_t = sat(delta_t, 1, 0);

    % ----------------------------------------------------------------
    % Outputs (Appended theta_c and phi_c so we can plot them easily)
    % ----------------------------------------------------------------
    delta     = [delta_e; delta_a; delta_r; delta_t];
    x_command = [0; 0; h_c; Va_c; 0; 0; phi_c; theta_c; chi_c; 0; 0; 0];
    y = [delta; x_command];
end

% =====================================================================
%  LATERAL FUNCTIONS
% =====================================================================
function phi_c_sat = course_with_roll(chi_c, chi, flag, AP)
    persistent integrator_chi
    if isempty(integrator_chi) || flag == 1
        integrator_chi = 0;
    end
    e_chi = chi_c - chi;
    integrator_chi = integrator_chi + AP.Ts * e_chi;
    phi_c = AP.course_kp * e_chi + AP.course_ki * integrator_chi;
    phi_c_sat = sat(phi_c, AP.phi_c_max, -AP.phi_c_max);
    if abs(phi_c_sat) < abs(phi_c)
        integrator_chi = integrator_chi - AP.Ts * e_chi;
    end
end

function delta_a = roll_with_aileron(phi_c, phi, p, AP)
    e_phi   = phi_c - phi;
    delta_a = sat(AP.roll_kp * e_phi + AP.roll_kd * (0 - p), AP.delta_a_max, -AP.delta_a_max);
end

function delta_r = yaw_damper(r, flag, AP)
    persistent r_prev r_washed
    if isempty(r_prev) || flag == 1
        r_prev   = r;
        r_washed = 0;
    end
    tau      = AP.yaw_damper_tau_r;
    alpha_wo = tau / (tau + AP.Ts);
    r_washed = alpha_wo * r_washed + alpha_wo * (r - r_prev);
    r_prev   = r;
    delta_r  = sat(-AP.yaw_damper_kp * r_washed, AP.delta_r_max, -AP.delta_r_max);
end

% =====================================================================
%  LONGITUDINAL FUNCTIONS
% =====================================================================
function delta_e = pitch_with_elevator(theta_c, theta, q, AP)
    e_theta = theta_c - theta;
    delta_e = sat(AP.pitch_kp * e_theta + AP.pitch_kd * (0 - q), AP.delta_e_max, -AP.delta_e_max);
end

function delta_t_sat = airspeed_with_throttle(Va_c, Va, flag, AP)
    persistent integrator_Va
    if isempty(integrator_Va) || flag == 1
        if AP.airspeed_throttle_ki > 0
            integrator_Va = AP.delta_t_trim / AP.airspeed_throttle_ki;
        else
            integrator_Va = 0;
        end
    end
    e_Va = Va_c - Va;
    integrator_Va = integrator_Va + AP.Ts * e_Va;
    delta_t = AP.airspeed_throttle_kp * e_Va + AP.airspeed_throttle_ki * integrator_Va;
    delta_t_sat = sat(delta_t, 1, 0);
    if abs(delta_t_sat) < abs(delta_t)
        integrator_Va = integrator_Va - AP.Ts * e_Va;
    end
end

function theta_c_sat = altitude_with_pitch(h_c, h, flag, AP)
    persistent integrator_h
    if isempty(integrator_h) || flag == 1
        integrator_h = 0;
    end
    e_h = h_c - h;
    integrator_h = integrator_h + AP.Ts * e_h;
    theta_c = AP.altitude_kp * e_h + AP.altitude_ki * integrator_h;
    theta_c_sat = sat(theta_c, AP.theta_c_max, -AP.theta_c_max);
    if abs(theta_c_sat) < abs(theta_c)
        integrator_h = integrator_h - AP.Ts * e_h;
    end
end

function theta_c_sat = airspeed_with_pitch(Va_c, Va, flag, AP)
    persistent integrator_Va_pitch
    if isempty(integrator_Va_pitch) || flag == 1
        integrator_Va_pitch = 0;
    end
    e_Va = Va_c - Va;
    integrator_Va_pitch = integrator_Va_pitch + AP.Ts * e_Va;
    theta_c = AP.airspeed_pitch_kp * e_Va + AP.airspeed_pitch_ki * integrator_Va_pitch;
    theta_c_sat = sat(theta_c, AP.theta_c_max, -AP.theta_c_max);
    if abs(theta_c_sat) < abs(theta_c)
        integrator_Va_pitch = integrator_Va_pitch - AP.Ts * e_Va;
    end
end

% =====================================================================
%  UTILITY FUNCTIONS
% =====================================================================
function out = sat(in, up_limit, low_limit)
    if in > up_limit, out = up_limit;
    elseif in < low_limit, out = low_limit;
    else, out = in; end
end

function chi_c_wrapped = wrap(chi_c, chi)
    chi_c_wrapped = chi_c;
    while (chi_c_wrapped - chi) >  pi,  chi_c_wrapped = chi_c_wrapped - 2*pi; end
    while (chi_c_wrapped - chi) < -pi,  chi_c_wrapped = chi_c_wrapped + 2*pi; end
end