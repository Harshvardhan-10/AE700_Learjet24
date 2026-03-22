function drawAircraft(uu)
% drawAircraft.m
%   Draws a 3-D aircraft body that follows the MAV position and attitude.

    % ----------------------------------------------------------------
    % Unpack inputs
    % ----------------------------------------------------------------
    pn    = uu(1);    % inertial North  [m]
    pe    = uu(2);    % inertial East   [m]
    pd    = uu(3);    % inertial Down   [m]
    phi   = uu(7);    % roll  [rad]
    theta = uu(8);    % pitch [rad]
    psi   = uu(9);    % yaw   [rad]
    t     = uu(13);   % time  [s]

    % ----------------------------------------------------------------
    % Persistent variables
    % ----------------------------------------------------------------
    persistent aircraft_handle
    persistent Vertices Faces facecolors

    % ----------------------------------------------------------------
    % First call OR figure was closed — initialise everything
    % ----------------------------------------------------------------
    if t == 0 || isempty(aircraft_handle) || ~isvalid(aircraft_handle)

        figure(1), clf

        [Vertices, Faces, facecolors] = defineAircraftBody();

        aircraft_handle = drawBody(Vertices, Faces, facecolors, ...
                                   pn, pe, pd, phi, theta, psi, []);

        title('Learjet 24 — 3D Animation')
        xlabel('East (m)')
        ylabel('North (m)')
        zlabel('Altitude (m)')
        view(52, 24)
        axis equal
        grid on
        hold on

        % Set axis centred on initial position with generous margin
        S = 200;   % half-width of view box [m]
        h_alt = -pd;
        axis([pe-S, pe+S, pn-S, pn+S, h_alt-S, h_alt+S]);

    % ----------------------------------------------------------------
    % Subsequent calls — just move the aircraft
    % ----------------------------------------------------------------
    else
        drawBody(Vertices, Faces, facecolors, ...
                 pn, pe, pd, phi, theta, psi, aircraft_handle);

        % Keep view centred on aircraft
        S = 200;
        h_alt = -pd;
        axis([pe-S, pe+S, pn-S, pn+S, h_alt-S, h_alt+S]);
    end
end


% ======================================================================
%  drawBody  — rotate, translate, NED->XYZ, then patch
% ======================================================================
function handle = drawBody(V, F, colors, pn, pe, pd, phi, theta, psi, handle)

    V = rotate(V',    phi, theta, psi)';   % rotate about CG
    V = translate(V', pn,  pe,   pd)';     % move to world position
    
    % NED -> MATLAB XYZ:  X=East, Y=North, Z=Up
    R_ned2xyz = [0, 1, 0;
                 1, 0, 0;
                 0, 0,-1];
    V = V * R_ned2xyz';

    if isempty(handle)
        handle = patch('Vertices', V, 'Faces', F, ...
                       'FaceVertexCData', colors, ...
                       'FaceColor', 'flat', ...
                       'EdgeColor', 'none');
    else
        set(handle, 'Vertices', V, 'Faces', F);
        drawnow limitrate
    end
end


% ======================================================================
%  rotate  — ZYX body rotation
% ======================================================================
function XYZ = rotate(XYZ, phi, theta, psi)
    R_roll  = [1,         0,          0;
               0,  cos(phi),  -sin(phi);
               0,  sin(phi),   cos(phi)];

    R_pitch = [ cos(theta), 0, sin(theta);
                0,          1,          0;
               -sin(theta), 0, cos(theta)];

    R_yaw   = [cos(psi), -sin(psi), 0;
               sin(psi),  cos(psi), 0;
               0,              0,   1];

    R = R_yaw * R_pitch * R_roll;
    XYZ = R * XYZ;
end


% ======================================================================
%  translate
% ======================================================================
function XYZ = translate(XYZ, pn, pe, pd)
    XYZ = XYZ + repmat([pn; pe; pd], 1, size(XYZ, 2));
end


% ======================================================================
%  defineAircraftBody  — Learjet-style fuselage + wings + tail
%  All dimensions in metres, centred at CG
% ======================================================================
function [V, F, colors] = defineAircraftBody()

    % ---- Scale (approximate Learjet 24 proportions) ----
    %  Real span ~10.8 m, length ~14.5 m  → use ~1/14 scale
    sc = 14;   % scale factor — increase to make aircraft larger on screen

    fuse_l1    =  7 / sc;    % nose tip to widest point
    fuse_l2    =  4 / sc;    % widest-point to wing LE (body width section)
    fuse_l3    = 15 / sc;    % widest-point to tail tip
    fuse_w     =  2 / sc;    % fuselage half-width/height
    wing_l     =  6 / sc;    % wing chord
    wing_w     = 20 / sc;    % wing semi-span * 2  (full span)
    tailwing_w = 10 / sc;    % horizontal stabiliser span
    tailwing_l =  3 / sc;    % horizontal stabiliser chord
    tail_h     =  3 / sc;    % vertical stabiliser height

    % ---- Vertices ----
    V = [...
        fuse_l1,             0,              0;...   %  1 nose tip
        fuse_l2,            -fuse_w/2,      -fuse_w/2;... %  2
        fuse_l2,             fuse_w/2,      -fuse_w/2;... %  3
        fuse_l2,             fuse_w/2,       fuse_w/2;... %  4
        fuse_l2,            -fuse_w/2,       fuse_w/2;... %  5
       -fuse_l3,             0,              0;...   %  6 tail tip
        0,                   wing_w/2,       0;...   %  7 wing root TE starboard
       -wing_l,              wing_w/2,       0;...   %  8 wing tip  TE starboard
       -wing_l,             -wing_w/2,       0;...   %  9 wing tip  TE port
        0,                  -wing_w/2,       0;...   % 10 wing root TE port
       -fuse_l3+tailwing_l,  tailwing_w/2,  0;...   % 11 H-stab LE starboard
       -fuse_l3,             tailwing_w/2,  0;...   % 12 H-stab TE starboard
       -fuse_l3,            -tailwing_w/2,  0;...   % 13 H-stab TE port
       -fuse_l3+tailwing_l, -tailwing_w/2,  0;...   % 14 H-stab LE port
       -fuse_l3+tailwing_l,  0,             0;...   % 15 V-stab base LE
       -fuse_l3+tailwing_l,  0,            -tail_h;...% 16 V-stab tip LE
       -fuse_l3,             0,            -tail_h;...% 17 V-stab tip TE
    ];

    % ---- Faces ----
    F = [...
        1,  2,  3,  1;...   % nose top
        1,  3,  4,  1;...   % nose left
        1,  4,  5,  1;...   % nose bottom
        1,  5,  2,  1;...   % nose right
        2,  3,  6,  2;...   % fuselage top
        3,  4,  6,  3;...   % fuselage left
        4,  5,  6,  4;...   % fuselage bottom
        5,  2,  6,  5;...   % fuselage right
        7,  8,  9, 10;...   % main wing
       11, 12, 13, 14;...   % horizontal stabiliser
        6, 15, 17, 17;...   % vertical stabiliser
    ];

    % ---- Colours ----
    yellow  = [1.0, 1.0, 0.0];
    blue    = [0.2, 0.4, 0.8];
    red     = [0.9, 0.1, 0.1];
    green   = [0.1, 0.7, 0.2];

    colors = [...
        yellow;...   % nose top
        yellow;...   % nose left
        yellow;...   % nose bottom
        yellow;...   % nose right
        blue;...     % fuselage top
        blue;...     % fuselage left
        blue;...     % fuselage bottom
        blue;...     % fuselage right
        green;...    % main wing
        green;...    % horizontal stabiliser
        red;...      % vertical stabiliser
    ];

    V = sc * V;   % rescale back to metres
end