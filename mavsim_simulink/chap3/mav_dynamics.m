function [sys,x0,str,ts,simStateCompliance] = mav_dynamics(t,x,u,flag,MAV)
% mav_dynamics.m
%   Level-1 MATLAB S-function implementing the 6-DOF MAV equations of
%   motion using 12 Euler states (replaces quaternion formulation).
%
%   States (12):
%     x(1)  = pn      inertial North position [m]
%     x(2)  = pe      inertial East position  [m]
%     x(3)  = pd      inertial Down position  [m]
%     x(4)  = u       body-x velocity [m/s]
%     x(5)  = v       body-y velocity [m/s]
%     x(6)  = w       body-z velocity [m/s]
%     x(7)  = phi     roll  angle [rad]
%     x(8)  = theta   pitch angle [rad]
%     x(9)  = psi     yaw   angle [rad]
%     x(10) = p       roll  rate [rad/s]
%     x(11) = q       pitch rate [rad/s]
%     x(12) = r       yaw   rate [rad/s]
%
%   Inputs (6):
%     u(1)  = fx      total force  along body x [N]
%     u(2)  = fy      total force  along body y [N]
%     u(3)  = fz      total force  along body z [N]
%     u(4)  = ell     total moment about body x [N·m]
%     u(5)  = m       total moment about body y [N·m]
%     u(6)  = n       total moment about body z [N·m]
%
%   Outputs (12):  same ordering as states (Euler directly, no conversion)

switch flag
    case 0,  [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes(MAV);
    case 1,  sys = mdlDerivatives(t,x,u,MAV);
    case 2,  sys = [];
    case 3,  sys = mdlOutputs(t,x,u);
    case 4,  sys = mdlGetTimeOfNextVarHit(t,x,u);
    case 9,  sys = [];
    otherwise, error('Unhandled flag: %d', flag);
end
end

% =========================================================================
function [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes(MAV)

sizes = simsizes;
sizes.NumContStates  = 12;   % 12 Euler states (was 13 quaternion)
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);

% Initial conditions from MAV struct
x0 = [MAV.pn0;
      MAV.pe0;
      MAV.pd0;
      MAV.u0;
      MAV.v0;
      MAV.w0;
      MAV.phi0;
      MAV.theta0;
      MAV.psi0;
      MAV.p0;
      MAV.q0;
      MAV.r0];

str = [];
ts  = [0 0];
simStateCompliance = 'UnknownSimState';
end

% =========================================================================
function sys = mdlDerivatives(t, x, uu, MAV)

% ---- Unpack states ----
pn    = x(1);   pe   = x(2);   pd    = x(3);
u     = x(4);   v    = x(5);   w     = x(6);
phi   = x(7);   theta= x(8);   psi   = x(9);
p     = x(10);  q    = x(11);  r     = x(12);

% ---- Unpack inputs ----
fx  = uu(1);  fy = uu(2);  fz = uu(3);
ell = uu(4);  m  = uu(5);  n  = uu(6);

mass = MAV.mass;

% ---- Position kinematics  (B&M Eq. 3.14) ----
% Rotation matrix body -> inertial  R_bi = Euler2Rotation(phi,theta,psi)
cphi = cos(phi);   sphi = sin(phi);
cth  = cos(theta); sth  = sin(theta);
cpsi = cos(psi);   spsi = sin(psi);

R = [cth*cpsi,  sphi*sth*cpsi - cphi*spsi,  cphi*sth*cpsi + sphi*spsi;
     cth*spsi,  sphi*sth*spsi + cphi*cpsi,  cphi*sth*spsi - sphi*cpsi;
    -sth,       sphi*cth,                   cphi*cth];

pos_dot = R * [u; v; w];
pndot = pos_dot(1);
pedot = pos_dot(2);
pddot = pos_dot(3);

% ---- Translational dynamics  (B&M Eq. 3.15) ----
udot = r*v - q*w + fx/mass;
vdot = p*w - r*u + fy/mass;
wdot = q*u - p*v + fz/mass;

% ---- Euler angle kinematics  (B&M Eq. 3.14) ----
% Singularity at theta = ±90 deg (gimbal lock) — not an issue for this aircraft
tth  = tan(theta);
sth_safe = cth;   % cos(theta) used as denominator — safe away from ±90 deg

phidot   = p + (q*sphi + r*cphi) * tth;
thetadot = q*cphi - r*sphi;
psidot   = (q*sphi + r*cphi) / cth;

% ---- Rotational dynamics  (B&M Eq. 3.17) ----
% Uses pre-computed Gamma parameters from aerosonde_parameters.m
pdot = MAV.Gamma1*p*q - MAV.Gamma2*q*r + MAV.Gamma3*ell + MAV.Gamma4*n;
qdot = MAV.Gamma5*p*r - MAV.Gamma6*(p^2 - r^2)          + m/MAV.Jy;
rdot = MAV.Gamma7*p*q - MAV.Gamma1*q*r + MAV.Gamma4*ell + MAV.Gamma8*n;

sys = [pndot; pedot; pddot; udot; vdot; wdot;
       phidot; thetadot; psidot; pdot; qdot; rdot];
end

% =========================================================================
function sys = mdlOutputs(t, x, u)
% Outputs are the 12 states directly — no conversion needed
sys = x;
end

% =========================================================================
function sys = mdlGetTimeOfNextVarHit(t, x, u)
sys = t + 1;
end