% compute_trim.m
%   Computes trim conditions using fsolve().
%   Works with Euler-based mav_dynamics (12 states).
%
%   Fixed trim (wings-level straight flight):
%     phi=0, beta=0, p=q=r=0, v=0, delta_a=0, delta_r=0
%   Solved by fsolve: [u, w, delta_e, delta_t]

addpath('../parameters')
addpath('../tools')
addpath('../chap3')
addpath('../chap4')
aerosonde_parameters

gamma  = 0 * pi/180;   % flight-path angle [rad]
Va_des = 80;          % desired airspeed [m/s]

    function residual = trim_equations(z, Va_des, gamma, MAV)
        u = z(1);  w = z(2);  delta_e = z(3);  delta_t = z(4);
        phi = 0;  q_nd = 0;  alphadot_nd = 0;

        Va    = sqrt(u^2 + w^2);
        alpha = atan2(w, u);
        theta = alpha + gamma;

        CL = MAV.CL_0 + MAV.CL_alpha*alpha + MAV.CL_adot*alphadot_nd ...
           + MAV.CL_q*q_nd + MAV.CL_de*delta_e;
        CD = MAV.CD_0 + MAV.CD_alpha*alpha + MAV.CD_de*delta_e;
        Cm = MAV.Cm_0 + MAV.Cm_alpha*alpha + MAV.Cm_adot*alphadot_nd ...
           + MAV.Cm_q*q_nd + MAV.Cm_de*delta_e;

        qbar = 0.5 * MAV.rho * Va^2;
        ca = cos(alpha);  sa = sin(alpha);
        F_lift = qbar * MAV.S_wing * CL;
        F_drag = qbar * MAV.S_wing * CD;

        fx = -MAV.mass*MAV.gravity*sin(theta) ...
             + (-ca*F_drag + sa*F_lift) + delta_t*MAV.T_max;
        fz =  MAV.mass*MAV.gravity*cos(theta)*cos(phi) ...
             + (-sa*F_drag - ca*F_lift);
        My = qbar * MAV.S_wing * MAV.c * Cm;

        residual = [fx/MAV.mass; fz/MAV.mass; My/MAV.Jy; Va-Va_des];
    end

z0 = [Va_des; 0; 0; 0.3];
options = optimoptions('fsolve','Display','iter', ...
    'TolFun',1e-12,'TolX',1e-12,'MaxFunEvals',2000,'MaxIter',500);

[z_sol, fval, exitflag] = fsolve(@(z) trim_equations(z,Va_des,gamma,MAV), z0, options);

fprintf('\nfsolve residual norm: %.4e\n', norm(fval));
if exitflag <= 0, warning('fsolve did not converge (exitflag=%d).', exitflag); end

u_sol       = z_sol(1);
w_sol       = z_sol(2);
delta_e_sol = z_sol(3);
delta_t_sol = max(0, min(1, z_sol(4)));

Va_trim    = sqrt(u_sol^2 + w_sol^2);
alpha_trim = atan2(w_sol, u_sol);
theta_trim = alpha_trim + gamma;
phi_trim   = 0;
beta_trim  = 0;
psi_trim   = 0;

% 12-element Euler state vector (no quaternion)
x_trim = [0; 0; -200; u_sol; 0; w_sol;
          phi_trim; theta_trim; psi_trim; 0; 0; 0];

u_trim = [delta_e_sol; 0; 0; delta_t_sol];
y_trim = [Va_trim; alpha_trim; beta_trim];

% Update MAV initial conditions
MAV.pn0 = 0;   MAV.pe0 = 0;   MAV.pd0 = -200;
MAV.u0  = u_sol;  MAV.v0 = 0;  MAV.w0 = w_sol;
MAV.phi0 = phi_trim;  MAV.theta0 = theta_trim;  MAV.psi0 = psi_trim;
MAV.p0 = 0;  MAV.q0 = 0;  MAV.r0 = 0;

fprintf('\n--- Trim Results  Va=%.1f m/s  gamma=%.1f deg ---\n', Va_des, gamma*180/pi);
fprintf('Va      = %.4f m/s\n',  Va_trim);
fprintf('alpha   = %.4f deg\n',  alpha_trim  * 180/pi);
fprintf('beta    = %.4f deg\n',  beta_trim   * 180/pi);
fprintf('phi     = %.4f deg\n',  phi_trim    * 180/pi);
fprintf('theta   = %.4f deg\n',  theta_trim  * 180/pi);
fprintf('delta_e = %.4f deg\n',  delta_e_sol * 180/pi);
fprintf('delta_a = %.4f deg\n',  0.0);
fprintf('delta_r = %.4f deg\n',  0.0);
fprintf('delta_t = %.4f\n',      delta_t_sol);

save('trim_results.mat', 'x_trim','u_trim','y_trim', ...
     'Va_trim','alpha_trim','theta_trim','phi_trim','psi_trim');
fprintf('Saved to trim_results.mat\n');