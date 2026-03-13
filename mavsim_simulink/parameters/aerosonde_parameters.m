% learjet24_parameters.m
% Parameters for the Learjet 24 (max weight cruise configuration)
% Source: Roskam, J., Airplane Flight Dynamics and Automatic Flight Controls, 1995
% Units      : ALL quantities stored in SI (m, kg, N, rad, kg·m²)
%              Values shown as  <dat-file value> * <conversion factor>
%              so every number is traceable back to aircraft.dat

addpath('../tools');

%% ---- Unit conversion factors -------------------------------------------
ft2m           = 0.3048;          % 1 ft        = 0.3048 m
ft2_2_m2       = ft2m^2;          % 1 ft²       = 0.0929 m²
ft3_2_m3       = ft2m^3;          % 1 ft³       = 0.0283 m³
slugft2_2_kgm2 = 1.35582;         % 1 slug·ft²  = 1.35582 kg·m²
slug2kg        = 14.5939;         % 1 slug      = 14.5939 kg
lb2N           = 4.44822;         % 1 lbf       = 4.44822 N
fts2ms         = ft2m;            % 1 ft/s      = 0.3048 m/s

%% ---- Initial conditions ------------------------------------------------
MAV.pn0    = 0;                          % North position      [m]
MAV.pe0    = 0;                          % East  position      [m]
MAV.pd0    = -1000  * ft2m;              % Down  position      [m]  (1000 ft altitude)
MAV.u0     =   250  * fts2ms;            % body x-velocity     [m/s]  (~76.2 m/s ≈ 148 kt)
MAV.v0     = 0;                          % body y-velocity     [m/s]
MAV.w0     = 0;                          % body z-velocity     [m/s]
MAV.phi0   = 0;                          % roll                [rad]
MAV.theta0 = 0.08;                       % pitch               [rad]  (~4.6°, approx trim)
MAV.psi0   = 0;                          % yaw                 [rad]

% Convert Euler angles to quaternion for the s-function
e = Euler2Quaternion(MAV.phi0, MAV.theta0, MAV.psi0);
MAV.e0 = e(1);
MAV.e1 = e(2);
MAV.e2 = e(3);
MAV.e3 = e(4);

MAV.p0 = 0;                              % roll  rate          [rad/s]
MAV.q0 = 0;                              % pitch rate          [rad/s]
MAV.r0 = 0;                              % yaw   rate          [rad/s]

%% ---- Physical parameters -----------------------------------------------
MAV.gravity = 9.81;                      % [m/s²]

% dat file: Weight = 13000 lb  →  mass = W/g
% Convert weight to Newtons first, then divide by SI gravity
MAV.mass = (13000 * lb2N) / MAV.gravity; % [kg]

% Moments of inertia  (dat file units: slug·ft²)
MAV.Jx  = 28000 * slugft2_2_kgm2;       % [kg·m²]   I_xx = 28000 slug·ft²
MAV.Jy  = 18800 * slugft2_2_kgm2;       % [kg·m²]   I_yy = 18800 slug·ft²
MAV.Jz  = 47000 * slugft2_2_kgm2;       % [kg·m²]   I_zz = 47000 slug·ft²
MAV.Jxz =  1300 * slugft2_2_kgm2;       % [kg·m²]   I_xz =  1300 slug·ft²

%% ---- Wing geometry  (dat file units: ft and ft²) -----------------------
MAV.S_wing = 230.0 * ft2_2_m2;          % [m²]   Sw   = 230  ft²
MAV.b      =  34.0 * ft2m;              % [m]    bw   = 34   ft
MAV.c      =   7.0 * ft2m;              % [m]    cbar = 7    ft
MAV.AR     = MAV.b^2 / MAV.S_wing;      % [-]    aspect ratio (≈ 5.02, unit-invariant)

%% ---- Atmosphere  (sea-level standard for initial testing) --------------
% dat file: rho not listed; aerosonde used 0.002377 slug/ft³ (SL standard)
% 0.002377 slug/ft³  ×  (14.5939 kg/slug)  /  (0.3048 m/ft)³  =  1.225 kg/m³
MAV.rho = 0.002377 * (slug2kg / ft3_2_m3);   % [kg/m³]  sea-level standard
% For cruise at ~25,000 ft: use MAV.rho = 0.549  [kg/m³]

%% ---- Engine  (dat file: simpleSingle 2950 lb) --------------------------
MAV.T_max = 2950 * lb2N;                % [N]

%% ---- Aerodynamic coefficients ------------------------------------------
% All aero coefficients are dimensionless or per-radian.
% NO unit conversion is needed — they are copied directly from aircraft.dat.

% --- Lift ---
MAV.C_L_0         =  0.130;             % [-]        CLo
MAV.C_L_alpha     =  5.840;             % [/rad]     CL_a
MAV.C_L_q         =  4.7;               % [/rad]     CL_q
MAV.C_L_alpha_dot =  2.2;               % [/rad]     CL_adot
MAV.C_L_delta_e   =  0.46;              % [/rad]     CL_de

% --- Drag ---
MAV.C_D_0         =  0.0216;            % [-]        CDo
MAV.C_D_alpha     =  0.300;             % [/rad]     CD_a
MAV.C_D_delta_e   =  0.0;               % [/rad]     CD_de
% CDK not listed in dat file; standard estimate: 1/(pi * e * AR), assume e = 0.8
MAV.e_oswald      =  0.80;              % [-]        Oswald efficiency (assumed)
MAV.C_D_K         =  1 / (pi * MAV.e_oswald * MAV.AR);   % [-]

% --- Pitching moment ---
MAV.C_m_0         =  0.05;              % [-]        Cmo
MAV.C_m_alpha     = -0.64;              % [/rad]     Cm_a
MAV.C_m_q         = -15.5;              % [/rad]     Cm_q
MAV.C_m_alpha_dot = -6.7;               % [/rad]     Cm_adot
MAV.C_m_delta_e   = -1.24;              % [/rad]     Cm_de

% --- Side force ---
MAV.C_Y_beta      = -0.730;             % [/rad]     CY_beta
MAV.C_Y_p         =  0.0;               % [/rad]     CY_p
MAV.C_Y_r         =  0.400;             % [/rad]     CY_r
MAV.C_Y_delta_a   =  0.0;               % [/rad]     CY_da  (sign reversed per dat)
MAV.C_Y_delta_r   =  0.140;             % [/rad]     CY_dr

% --- Rolling moment ---
MAV.C_ell_beta    = -0.110;             % [/rad]     Cl_beta
MAV.C_ell_p       = -0.450;             % [/rad]     Cl_p
MAV.C_ell_r       =  0.160;             % [/rad]     Cl_r
MAV.C_ell_delta_a = -0.178;             % [/rad]     Cl_da  (sign reversed per dat)
MAV.C_ell_delta_r =  0.019;             % [/rad]     Cl_dr

% --- Yawing moment ---
MAV.C_n_beta      =  0.127;             % [/rad]     Cn_beta
MAV.C_n_p         = -0.008;             % [/rad]     Cn_p
MAV.C_n_r         = -0.200;             % [/rad]     Cn_r
MAV.C_n_delta_a   =  0.020;             % [/rad]     Cn_da  (sign reversed per dat)
MAV.C_n_delta_r   = -0.074;             % [/rad]     Cn_dr

%% ---- Control surface limits  (dat file: deg) ---------------------------
MAV.delta_e_max = 20 * (pi/180);        % [rad]   controlSurface de 20 20
MAV.delta_a_max = 20 * (pi/180);        % [rad]   controlSurface da 20 20
MAV.delta_r_max = 20 * (pi/180);        % [rad]   controlSurface dr 20 20