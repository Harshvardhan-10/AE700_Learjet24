% learjet24_parameters.m
%   Aircraft parameters for the Learjet 24 (max weight cruise configuration)
%   All values converted to SI units.
%
%   Source: Roskam, Airplane Flight Dynamics and Automatic Flight Controls,
%           Part I, DARcorporation, 1995, pp.522-524  (via aircraft.dat)
%
%   Convention: aerosonde field names (MAV.xxx) to match mav_dynamics.m
%               and the mavsim_simulink template code.

addpath('../tools');

% -----------------------------------------------------------------------
% Unit conversion factors
% -----------------------------------------------------------------------
ft2m           = 0.3048;
ft2_2_m2       = ft2m^2;
slugft2_2_kgm2 = 1.35582;
lb2N           = 4.44822;

% -----------------------------------------------------------------------
% Initial conditions
% -----------------------------------------------------------------------
MAV.pn0    =    0;      % initial North position [m]
MAV.pe0    =    0;      % initial East position  [m]
MAV.pd0    = -100;      % initial Down position  [m]  (100 m altitude)
MAV.u0     =   80;      % initial body-x velocity [m/s]  (~155 kt cruise)
MAV.v0     =    0;
MAV.w0     =    0;
MAV.phi0   =    0;      % initial roll  [rad]
MAV.theta0 =    0;      % initial pitch [rad]
MAV.psi0   =    0;      % initial yaw   [rad]

e = Euler2Quaternion(MAV.phi0, MAV.theta0, MAV.psi0);
MAV.e0 = e(1);
MAV.e1 = e(2);
MAV.e2 = e(3);
MAV.e3 = e(4);

MAV.p0 = 0;
MAV.q0 = 0;
MAV.r0 = 0;

% -----------------------------------------------------------------------
% Physical parameters
% -----------------------------------------------------------------------
MAV.gravity = 9.81;                              % [m/s^2]

% Mass:  13000 lb -> kg
MAV.mass  = 13000 * lb2N / MAV.gravity;         % = 5897.0 kg

% Moments of inertia:  slug-ft^2 -> kg-m^2
MAV.Jx  = 28000 * slugft2_2_kgm2;              % = 37962.96 kg-m^2
MAV.Jy  = 18800 * slugft2_2_kgm2;              % = 25489.42 kg-m^2
MAV.Jz  = 47000 * slugft2_2_kgm2;              % = 63723.54 kg-m^2
MAV.Jxz =  1300 * slugft2_2_kgm2;              % =  1762.57 kg-m^2

% Derived Gamma parameters (used in mav_dynamics.m)
MAV.Gamma  = MAV.Jx*MAV.Jz - MAV.Jxz^2;
MAV.Gamma1 = (MAV.Jxz*(MAV.Jx - MAV.Jy + MAV.Jz)) / MAV.Gamma;
MAV.Gamma2 = (MAV.Jz*(MAV.Jz - MAV.Jy) + MAV.Jxz^2) / MAV.Gamma;
MAV.Gamma3 = MAV.Jz  / MAV.Gamma;
MAV.Gamma4 = MAV.Jxz / MAV.Gamma;
MAV.Gamma5 = (MAV.Jz - MAV.Jx) / MAV.Jy;
MAV.Gamma6 = MAV.Jxz / MAV.Jy;
MAV.Gamma7 = ((MAV.Jx - MAV.Jy)*MAV.Jx + MAV.Jxz^2) / MAV.Gamma;
MAV.Gamma8 = MAV.Jx  / MAV.Gamma;

% -----------------------------------------------------------------------
% Wing geometry:  ft -> m,  ft^2 -> m^2
% -----------------------------------------------------------------------
MAV.b      = 34.0  * ft2m;                      % wing span   [m]
MAV.c      =  7.0  * ft2m;                      % mean chord  [m]
MAV.S_wing = 230.0 * ft2_2_m2;                  % wing area   [m^2]
MAV.AR     = MAV.b^2 / MAV.S_wing;              % aspect ratio

% -----------------------------------------------------------------------
% Atmosphere (ISA sea level)
% -----------------------------------------------------------------------
MAV.rho = 1.225;                                 % air density [kg/m^3]

% -----------------------------------------------------------------------
% Propulsion
%   aircraft.dat: simpleSingle 2950 lb (one engine)
%   Learjet 24 has TWO engines -> total static thrust = 2 x 2950 lb
% -----------------------------------------------------------------------
MAV.T_max = 2 * 2950 * lb2N;                    % = 26241 N total thrust

% -----------------------------------------------------------------------
% Aerodynamic coefficients (all /rad, dimensionless — no conversion needed)
% -----------------------------------------------------------------------

% Lift
MAV.CL_0     =  0.130;
MAV.CL_alpha =  5.840;
MAV.CL_adot  =  2.2;
MAV.CL_q     =  4.7;
MAV.CL_de    =  0.46;

% Drag
MAV.CD_0     =  0.0216;
MAV.CD_alpha =  0.300;
MAV.CD_de    =  0.0;

% Pitching moment
MAV.Cm_0     =  0.05;
MAV.Cm_alpha = -0.64;
MAV.Cm_adot  = -6.7;
MAV.Cm_q     = -15.5;
MAV.Cm_de    = -1.24;

% Side force
MAV.CY_beta  = -0.730;
MAV.CY_p     =  0.0;
MAV.CY_r     =  0.400;
MAV.CY_da    =  0.0;
MAV.CY_dr    =  0.140;

% Rolling moment
MAV.Cl_beta  = -0.110;
MAV.Cl_p     = -0.450;
MAV.Cl_r     =  0.160;
MAV.Cl_da    = -0.178;
MAV.Cl_dr    =  0.019;

% Yawing moment
MAV.Cn_beta  =  0.127;
MAV.Cn_p     = -0.008;
MAV.Cn_r     = -0.200;
MAV.Cn_da    =  0.020;
MAV.Cn_dr    = -0.074;

% Control surface limits [rad]
MAV.de_max   = 20 * pi/180;
MAV.da_max   = 20 * pi/180;
MAV.dr_max   = 20 * pi/180;

% CG vertical offset
MAV.Dz_cg   = 3.5 * ft2m;                       % [m]