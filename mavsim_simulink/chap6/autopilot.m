function y = autopilot(uu, AP)
% autopilot.m
%   Full lateral and longitudinal autopilot for mavsim.
%
%   LATERAL LOOPS (inner to outer):
%     1. roll_with_aileron:   phi  -> delta_a  (PD, uses measured p)
%     2. course_with_roll:    chi  -> phi_c    (PI)
%     3. yaw_damper:          r    -> delta_r  (P washout)
%
%   LONGITUDINAL LOOPS (inner to outer):
%     1. pitch_with_elevator:   theta -> delta_e  (PD, uses measured q)
%     2. altitude_with_pitch:   h     -> theta_c  (PI)
%     3. airspeed_with_throttle: Va   -> delta_t  (PI)

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
    phi_c_ff = 0;       % no feedforward roll command
    t     = uu(1+NN);

    % ----------------------------------------------------------------
    % LATERAL AUTOPILOT
    % ----------------------------------------------------------------
    chi_ref = wrap(chi_c, chi);

    if t == 0
        % Initialise all lateral integrators, hold current angles
        phi_c   = phi;
        delta_r = yaw_damper(r, 1, AP);
        course_with_roll(chi_ref, chi, 1, AP);   % initialise integrator
    else
        phi_c   = course_with_roll(chi_ref, chi, 0, AP);
        delta_r = yaw_damper(r, 0, AP);
    end

    % Inner roll loop — always computed
    delta_a = roll_with_aileron(phi_c + phi_c_ff, phi, p, AP);

    % ----------------------------------------------------------------
    % LONGITUDINAL AUTOPILOT
    %   h_ref: saturated altitude command — limits how fast altitude
    %   is commanded to change (prevents abrupt pitch commands)
    % ----------------------------------------------------------------
    h_ref = sat(h_c, h + AP.altitude_zone, h - AP.altitude_zone);

    if t == 0
        % Initialise longitudinal integrators
        % Pre-load throttle integrator with trim value so output starts
        % at delta_t_trim rather than zero (prevents initial dive)
        delta_t = airspeed_with_throttle(Va_c, Va, 1, AP);
        theta_c = altitude_with_pitch(h_ref, h, 1, AP);
    else
        delta_t = airspeed_with_throttle(Va_c, Va, 0, AP);
        theta_c = altitude_with_pitch(h_ref, h, 0, AP);
    end

    % Inner pitch loop — always computed
    delta_e = pitch_with_elevator(theta_c, theta, q, AP);

    % Clamp throttle to physical range [0, 1]
    delta_t = sat(delta_t, 1, 0);

    % ----------------------------------------------------------------
    % Outputs
    % ----------------------------------------------------------------
    delta     = [delta_e; delta_a; delta_r; delta_t];
    x_command = [0; 0; h_c; Va_c; 0; 0; phi_c; theta_c; chi_c; 0; 0; 0];
    y = [delta; x_command];
end


% =====================================================================
%  LATERAL FUNCTIONS
% =====================================================================

function phi_c_sat = course_with_roll(chi_c, chi, flag, AP)
% PI controller: chi -> phi_c
% Plant: chi_dot/phi = g/Va  (integrator in plant -> P gives zero SS error,
%        but PI improves disturbance rejection and robustness)
    persistent integrator_chi
    if isempty(integrator_chi) || flag == 1
        integrator_chi = 0;
    end
    e_chi = chi_c - chi;
    % Trapezoidal integration with anti-windup
    integrator_chi = integrator_chi + AP.Ts * e_chi;
    phi_c = AP.course_kp * e_chi + AP.course_ki * integrator_chi;
    phi_c_sat = sat(phi_c, AP.phi_c_max, -AP.phi_c_max);
    % Anti-windup: if saturated, remove last integration step
    if abs(phi_c_sat) < abs(phi_c)
        integrator_chi = integrator_chi - AP.Ts * e_chi;
    end
end


function delta_a = roll_with_aileron(phi_c, phi, p, AP)
% PD controller: phi -> delta_a
% Uses measured roll rate p directly (no dirty derivative needed)
% CL: s^2 + (a_phi1 + a_phi2*kd)*s + a_phi2*kp = 0
    e_phi   = phi_c - phi;
    delta_a = sat(AP.roll_kp * e_phi + AP.roll_kd * (0 - p), ...
                  AP.delta_a_max, -AP.delta_a_max);
end


function delta_r = yaw_damper(r, flag, AP)
% Washout yaw damper: improves Dutch roll damping
% H(s) = tau_r*s / (tau_r*s + 1) applied to r, then scaled by kp
% Discrete: r_washed(k) = tau/(tau+Ts)*r_washed(k-1) + tau/(tau+Ts)*(r(k)-r(k-1))
    persistent r_prev r_washed
    if isempty(r_prev) || flag == 1
        r_prev   = r;
        r_washed = 0;
    end
    tau = AP.yaw_damper_tau_r;
    Ts  = AP.Ts;
    alpha_wo = tau / (tau + Ts);
    r_washed = alpha_wo * r_washed + alpha_wo * (r - r_prev);
    r_prev   = r;
    delta_r  = sat(-AP.yaw_damper_kp * r_washed, ...
                    AP.delta_r_max, -AP.delta_r_max);
end


function delta_r_out = sideslip_with_rudder(beta_in, flag, AP)
% PI controller: beta -> delta_r  (for coordinated turns, beta_c = 0)
% Not used in main loop (yaw_damper handles delta_r) but available
% for standalone sideslip regulation tests
    persistent integrator_beta
    if isempty(integrator_beta) || flag == 1
        integrator_beta = 0;
    end
    e_beta = 0 - beta_in;
    integrator_beta = integrator_beta + AP.Ts * e_beta;
    out = AP.sideslip_kp * e_beta + AP.sideslip_ki * integrator_beta;
    delta_r_out = sat(out, AP.delta_r_max, -AP.delta_r_max);
    if abs(delta_r_out) < abs(out)
        integrator_beta = integrator_beta - AP.Ts * e_beta;
    end
end


% =====================================================================
%  LONGITUDINAL FUNCTIONS
% =====================================================================

function delta_e = pitch_with_elevator(theta_c, theta, q, AP)
% PD controller: theta -> delta_e
% Uses measured pitch rate q directly (no dirty derivative needed)
% CL: s^2 + (a_theta1+a_theta3*kd)*s + (a_theta2+a_theta3*kp) = 0
    e_theta = theta_c - theta;
    delta_e = sat(AP.pitch_kp * e_theta + AP.pitch_kd * (0 - q), ...
                  AP.delta_e_max, -AP.delta_e_max);
end


function delta_t_sat = airspeed_with_throttle(Va_c, Va, flag, AP)
% PI controller: Va -> delta_t
% Plant: Va/delta_t = a_V2/(s + a_V1)
% Integrator pre-loaded with trim throttle to prevent initial dive
    persistent integrator_Va
    if isempty(integrator_Va) || flag == 1
        % Pre-load integrator so initial output ≈ delta_t_trim
        % delta_t = kp*0 + ki*I = delta_t_trim => I = delta_t_trim/ki
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
    % Anti-windup
    if abs(delta_t_sat) < abs(delta_t)
        integrator_Va = integrator_Va - AP.Ts * e_Va;
    end
end


function theta_c_sat = altitude_with_pitch(h_c, h, flag, AP)
% PI controller: h -> theta_c
% Effective plant (with pitch loop closed): h/theta_c = K_theta_DC*Va/s
    persistent integrator_h
    if isempty(integrator_h) || flag == 1
        integrator_h = 0;
    end
    e_h = h_c - h;
    integrator_h = integrator_h + AP.Ts * e_h;
    theta_c = AP.altitude_kp * e_h + AP.altitude_ki * integrator_h;
    theta_c_sat = sat(theta_c, AP.theta_c_max, -AP.theta_c_max);
    % Anti-windup
    if abs(theta_c_sat) < abs(theta_c)
        integrator_h = integrator_h - AP.Ts * e_h;
    end
end


% =====================================================================
%  UTILITY FUNCTIONS
% =====================================================================

function out = sat(in, up_limit, low_limit)
% Saturation function
    if in > up_limit
        out = up_limit;
    elseif in < low_limit
        out = low_limit;
    else
        out = in;
    end
end


function chi_c_wrapped = wrap(chi_c, chi)
% Wrap chi_c so that the error chi_c - chi is in (-pi, pi)
    chi_c_wrapped = chi_c;
    while (chi_c_wrapped - chi) >  pi,  chi_c_wrapped = chi_c_wrapped - 2*pi; end
    while (chi_c_wrapped - chi) < -pi,  chi_c_wrapped = chi_c_wrapped + 2*pi; end
end