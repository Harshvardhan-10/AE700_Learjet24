%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%
% Aircraft:
% Learjet 24
% Max weight cruise configuration
%
% File name:
% learjet24_v1.m
%
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%
% Authors:
%
% Michael Selig
% http://www.uiuc.edu/~m-selig
% 2000/02/10   initial creation using
%              "Twin jet-engine business jet" data from:
%              Roskam, J., Airplane Flight Dynamics and Automatic Flight
%              Controls, Part I, DARcorporation, Lawrence, KS, 1995,
%              pg 522-524
% 2002/02/24   added gear
%
% Converted to SI units (m, kg, N, s, rad) by MATLAB script
%
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%
% Conversion factors used:
%   1 ft        = 0.3048 m
%   1 ft^2      = 0.092903 m^2
%   1 lb (force)= 4.44822 N
%   1 lb (mass) = 0.453592 kg
%   1 slug      = 14.5939 kg
%   1 slug*ft^2 = 1.35582 kg*m^2
%   1 lbf/ft/s  = 14.5939 N/(m/s)   (damping: lbs/ft/sec -> N*s/m, same as kg/s)
%   1 lbf/ft    = 14.5939 N/m        (stiffness: lbs/ft -> N/m)
%
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

%% --- Initial Conditions ---

Dz_cg = 3.5 * 0.3048;          % CG height offset [m]  (was 3.5 ft)

%% --- Geometry ---

bw   = 34.0  * 0.3048;          % Wing span [m]          (was 34 ft)
cbar =  7.0  * 0.3048;          % Mean aerodynamic chord [m] (was 7 ft)
Sw   = 230.0 * 0.092903;        % Wing reference area [m^2]  (was 230 ft^2)

%% --- Control Surface Deflection Limits ---
% [negative limit, positive limit] in degrees -> radians

de_lim = [-20, 20] * pi/180;    % Elevator deflection limits [rad]
da_lim = [-20, 20] * pi/180;    % Aileron deflection limits [rad]
dr_lim = [-20, 20] * pi/180;    % Rudder deflection limits [rad]

%% --- Mass & Inertia ---

Weight = 13000 * 4.44822;       % Weight [N]             (was 13000 lb)
mass   = 13000 * 0.453592;      % Mass [kg]              (derived from weight in lbm)

I_xx =  28000 * 1.35582;        % Roll moment of inertia [kg*m^2]  (was 28000 slug*ft^2)
I_yy =  18800 * 1.35582;        % Pitch moment of inertia [kg*m^2] (was 18800 slug*ft^2)
I_zz =  47000 * 1.35582;        % Yaw moment of inertia [kg*m^2]   (was 47000 slug*ft^2)
I_xz =   1300 * 1.35582;        % Cross product of inertia [kg*m^2](was 1300 slug*ft^2)

%% --- Engine ---

T_max = 2950 * 4.44822;         % Max static thrust per engine [N]  (was 2950 lb)
% Note: simpleSingle model; two engines of 2950 lb static thrust each

%% --- Lift Coefficients ---
% Dimensionless coefficients; per-radian derivatives remain in [/rad]

CLo    =  0.130;                % Lift coeff at zero alpha []
CL_a   =  5.840;                % dCL/d(alpha) [/rad]
CL_adot=  2.2;                  % dCL/d(alphadot * cbar/(2V)) [/rad]
CL_q   =  4.7;                  % dCL/d(q * cbar/(2V)) [/rad]
CL_de  =  0.46;                 % dCL/d(delta_e) [/rad]

%% --- Drag Coefficients ---

CDo    =  0.0216;               % Zero-lift drag coeff []
CD_a   =  0.300;                % dCD/d(alpha) [/rad]
CD_de  =  0.0;                  % dCD/d(delta_e) [/rad]

%% --- Pitching Moment Coefficients ---

Cmo    =  0.05;                 % Pitching moment coeff at zero alpha []
Cm_a   = -0.64;                 % dCm/d(alpha) [/rad]
Cm_adot= -6.7;                  % dCm/d(alphadot * cbar/(2V)) [/rad]
Cm_q   = -15.5;                 % dCm/d(q * cbar/(2V)) [/rad]
Cm_de  = -1.24;                 % dCm/d(delta_e) [/rad]

%% --- Side Force Coefficients ---

CY_beta= -0.730;                % dCY/d(beta) [/rad]
CY_p   =  0.0;                  % dCY/d(p * bw/(2V)) [/rad]
CY_r   =  0.400;                % dCY/d(r * bw/(2V)) [/rad]
CY_da  =  0.0;                  % dCY/d(delta_a) [/rad]   sign reversed in original
CY_dr  =  0.140;                % dCY/d(delta_r) [/rad]

%% --- Roll Moment Coefficients ---

Cl_beta= -0.110;                % dCl/d(beta) [/rad]
Cl_p   = -0.450;                % dCl/d(p * bw/(2V)) [/rad]
Cl_r   =  0.160;                % dCl/d(r * bw/(2V)) [/rad]
Cl_da  = -0.178;                % dCl/d(delta_a) [/rad]   sign reversed in original
Cl_dr  =  0.019;                % dCl/d(delta_r) [/rad]

%% --- Yaw Moment Coefficients ---

Cn_beta=  0.127;                % dCn/d(beta) [/rad]
Cn_p   = -0.008;                % dCn/d(p * bw/(2V)) [/rad]
Cn_r   = -0.200;                % dCn/d(r * bw/(2V)) [/rad]
Cn_da  =  0.020;                % dCn/d(delta_a) [/rad]   sign reversed in original
Cn_dr  = -0.074;                % dCn/d(delta_r) [/rad]

%% --- Landing Gear ---
% Positions: 0 = nose, 1 = right main, 2 = left main
% Offsets are from CG; x positive forward, y positive right, z positive down

% Damping conversion: lbs/ft/sec -> N*s/m  (1 lbf/(ft/s) = 14.5939 N/(m/s))
% Stiffness conversion: lbs/ft -> N/m      (1 lbf/ft     = 14.5939 N/m)

% -- Nose Gear (position 0) --
Dx_gear(1) =  5.5 * 0.3048;    % x-offset from CG [m]
Dy_gear(1) =  0.0 * 0.3048;    % y-offset from CG [m]
Dz_gear(1) =  4.5 * 0.3048;    % z-offset from CG [m]
cgear(1)   = 5000 * 14.5939;   % Damping [N*s/m]
kgear(1)   = 15000 * 14.5939;  % Spring stiffness [N/m]
muGear(1)  = 0.01;             % Rolling friction coefficient []
strutLength(1) = -0.5 * 0.3048;% Strut travel [m]

% -- Right Main Gear (position 1) --
Dx_gear(2) = -1.9 * 0.3048;    % x-offset from CG [m]
Dy_gear(2) =  3.0 * 0.3048;    % y-offset from CG [m]
Dz_gear(2) =  5.4 * 0.3048;    % z-offset from CG [m]
cgear(2)   = 5000 * 14.5939;   % Damping [N*s/m]
kgear(2)   = 15000 * 14.5939;  % Spring stiffness [N/m]
muGear(2)  = 0.01;             % Rolling friction coefficient []
strutLength(2) = 2.5 * 0.3048; % Strut travel [m]

% -- Left Main Gear (position 2) --
Dx_gear(3) = -1.9 * 0.3048;    % x-offset from CG [m]
Dy_gear(3) = -3.0 * 0.3048;    % y-offset from CG [m]
Dz_gear(3) =  5.4 * 0.3048;    % z-offset from CG [m]
cgear(3)   = 5000 * 14.5939;   % Damping [N*s/m]
kgear(3)   = 15000 * 14.5939;  % Spring stiffness [N/m]
muGear(3)  = 0.01;             % Rolling friction coefficient []
strutLength(3) = 2.5 * 0.3048; % Strut travel [m]

MAV.I_xx = I_xx;
MAV.I_yy = I_yy;
MAV.I_zz = I_zz;
MAV.I_xz = I_xz;

% end of file