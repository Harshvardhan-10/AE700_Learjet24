% forces_moments.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%

function out = forces_moments(x, delta, wind, P)

    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    
     % ----------------------------------------------------------------
    % Rotation matrix: inertial -> body  (transpose of body->inertial)
    % ----------------------------------------------------------------
    R_bi = Euler2Rotation(phi, theta, psi)';   % 3x3
 
    % Steady wind rotated into body frame
    w_body_steady = R_bi * [w_ns; w_es; w_ds];
    u_ws = w_body_steady(1);
    v_ws = w_body_steady(2);
    w_ws = w_body_steady(3);
 
    % Total wind in body frame
    u_w = u_ws + u_wg;
    v_w = v_ws + v_wg;
    w_w = w_ws + w_wg;
 
    % Wind in inertial frame (for output)
    w_n = w_ns;
    w_e = w_es;
    w_d = w_ds;
 
    % ----------------------------------------------------------------
    % Air-relative velocity and aerodynamic angles
    % ----------------------------------------------------------------
    u_r = u - u_w;
    v_r = v - v_w;
    w_r = w - w_w;
 
    Va = sqrt(u_r^2 + v_r^2 + w_r^2);
 
    if Va < 0.1   % avoid divide-by-zero at rest
        alpha = 0;
        beta  = 0;
    else
        alpha = atan2(w_r, u_r);
        beta  = asin(v_r / Va);
    end
 
    % ----------------------------------------------------------------
    % Non-dimensional coefficients  (Beard & McLain Ch.4 / Roskam)
    % All derivatives already in /rad from aircraft.dat
    % ----------------------------------------------------------------
    b    = P.b;       % wing span [m]
    c    = P.c;       % mean chord [m]
    S    = P.S_wing;  % wing area [m^2]
    rho  = P.rho;     % air density [kg/m^3]
    mass = P.mass;    % [kg]
    g    = P.gravity; % [m/s^2]
 
    % Common factor
    qbar = 0.5 * rho * Va^2;   % dynamic pressure [Pa]
 
    % Non-dimensional angular rates (avoid /0 at Va=0)
    if Va > 0.1
        p_nd = p * b / (2*Va);
        q_nd = q * c / (2*Va);
        r_nd = r * b / (2*Va);
    else
        p_nd = 0;  q_nd = 0;  r_nd = 0;
    end
 
    if Va > 0.1
        alphadot = q - (p*cos(alpha) + r*sin(alpha))*tan(beta);
        alphadot_nd = alphadot * P.c / (2*Va);
    else
        alphadot_nd = 0;
    end
    
    % ---- Lift coefficient ----
    CL = P.CL_0 + P.CL_alpha*alpha + P.CL_adot*alphadot_nd + P.CL_q*q_nd + P.CL_de*delta_e;
 
    % ---- Drag coefficient ----
    CD = P.CD_0 + P.CD_alpha*alpha + P.CD_de*delta_e;
 
    % ---- Pitching moment coefficient ----
    Cm = P.Cm_0 + P.Cm_alpha*alpha + P.Cm_adot*alphadot_nd + P.Cm_q*q_nd + P.Cm_de*delta_e;
 
    % ---- Side-force coefficient ----
    CY = P.CY_beta*beta + P.CY_p*p_nd + P.CY_r*r_nd ...
       + P.CY_da*delta_a + P.CY_dr*delta_r;
 
    % ---- Rolling moment coefficient ----
    Cell = P.Cl_beta*beta + P.Cl_p*p_nd + P.Cl_r*r_nd ...
         + P.Cl_da*delta_a + P.Cl_dr*delta_r;
 
    % ---- Yawing moment coefficient ----
    Cn = P.Cn_beta*beta + P.Cn_p*p_nd + P.Cn_r*r_nd ...
       + P.Cn_da*delta_a + P.Cn_dr*delta_r;
 
    % ----------------------------------------------------------------
    % Aerodynamic forces in stability (wind) axes -> rotate to body
    % Lift acts perpendicular to Va, Drag along -Va (in stability axes)
    %   F_x_stab = -D*cos(0) - L*sin(0)  ->  body frame via alpha
    % ----------------------------------------------------------------
    ca = cos(alpha);
    sa = sin(alpha);
 
    % Drag and Lift magnitudes
    F_drag = qbar * S * CD;
    F_lift = qbar * S * CL;
 
    % Forces in body frame
    fx_aero = -ca*F_drag + sa*F_lift;
    fz_aero = -sa*F_drag - ca*F_lift;
    fy_aero =  qbar * S * CY;
 
    % ----------------------------------------------------------------
    % Gravity force in body frame
    % ----------------------------------------------------------------
    fx_grav = -mass * g * sin(theta);
    fy_grav =  mass * g * cos(theta) * sin(phi);
    fz_grav =  mass * g * cos(theta) * cos(phi);
 
    % ----------------------------------------------------------------
    % Propulsion thrust (body x-axis)
    % Learjet: simpleSingle with T_max = 2950 lbf -> 13120.5 N per engine
    % aircraft.dat lists 2950 lb for one engine; plane has 2 engines
    % T_max total = 2 * 2950 * 4.44822 = 26241 N
    % Throttle maps linearly: T = delta_t * T_max
    % ----------------------------------------------------------------
    fx_prop = delta_t * P.T_max;
    fy_prop = 0;
    fz_prop = 0;
 
    % ----------------------------------------------------------------
    % Total forces [N]
    % ----------------------------------------------------------------
    Force(1) = fx_grav + fx_aero + fx_prop;
    Force(2) = fy_grav + fy_aero + fy_prop;
    Force(3) = fz_grav + fz_aero + fz_prop;
 
    % ----------------------------------------------------------------
    % Aerodynamic moments [N·m]
    % ----------------------------------------------------------------
    Torque(1) = qbar * S * b * Cell;          % roll  (l)
    Torque(2) = qbar * S * c * Cm;            % pitch (m)
    Torque(3) = qbar * S * b * Cn;            % yaw   (n)
 
    out = [Force'; Torque'; Va; alpha; beta; w_n; w_e; w_d];
end



