function [sys,x0,str,ts,simStateCompliance] = mav_dynamics(t,x,u,flag,MAV)

switch flag

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(MAV);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1
    sys=mdlDerivatives(t,x,u,MAV);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3
    sys=mdlOutputs(t,x);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

end

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(MAV)

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 13;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [...
    MAV.pn0;...
    MAV.pe0;...
    MAV.pd0;...
    MAV.u0;...
    MAV.v0;...
    MAV.w0;...
    MAV.e0;...
    MAV.e1;...
    MAV.e2;...
    MAV.e3;...
    MAV.p0;...
    MAV.q0;...
    MAV.r0;...
    ];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

end

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,uu, MAV)

    mass = MAV.mass;
    I_xx = MAV.Jx;
    I_yy = MAV.Jy;
    I_zz = MAV.Jz;
    I_xz = MAV.Jxz;

    pn    = x(1);
    pe    = x(2);
    pd    = x(3);
    u     = x(4);
    v     = x(5);
    w     = x(6);
    e0    = x(7);
    e1    = x(8);
    e2    = x(9);
    e3    = x(10);
    p     = x(11);
    q     = x(12);
    r     = x(13);
    fx    = uu(1);
    fy    = uu(2);
    fz    = uu(3);
    ell   = uu(4);
    m     = uu(5);
    n     = uu(6);

    R = Quaternion2Rotation([e0; e1; e2; e3]);

    posdot = R * [u; v; w];

    pndot = posdot(1);
    pedot = posdot(2);
    pddot = posdot(3);
    
    udot = r*v - q*w + fx/mass;
    vdot = p*w - r*u + fy/mass;
    wdot = q*u - p*v + fz/mass;
       
    Omega = [ 0   -p   -q   -r;
          p    0    r   -q;
          q   -r    0    p;
          r    q   -p    0 ];

    quat_dot = 0.5 * Omega * [e0; e1; e2; e3];

    e0dot = quat_dot(1);
    e1dot = quat_dot(2);
    e2dot = quat_dot(3);
    e3dot = quat_dot(4);

    Gamma  = I_xx*I_zz - I_xz^2;

    Gamma1 = (I_xz*(I_xx - I_yy + I_zz))/Gamma;
    Gamma2 = (I_zz*(I_zz - I_yy) + I_xz^2)/Gamma;
    Gamma3 = I_zz/Gamma;
    Gamma4 = I_xz/Gamma;
    Gamma5 = (I_zz - I_xx)/I_yy;
    Gamma6 = I_xz/I_yy;
    Gamma7 = ((I_xx - I_yy)*I_xx + I_xz^2)/Gamma;
    Gamma8 = I_xx/Gamma;
        
    pdot = Gamma1*p*q - Gamma2*q*r + Gamma3*ell + Gamma4*n;

    qdot = Gamma5*p*r - Gamma6*(p^2 - r^2) + m/I_yy;

    rdot = Gamma7*p*q - Gamma1*q*r + Gamma4*ell + Gamma8*n;
        

sys = [pndot; pedot; pddot; udot; vdot; wdot; e0dot; e1dot; e2dot; e3dot; pdot; qdot; rdot];

end

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

end

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x)
    pn = x(1);
    pe = x(2);
    pd = x(3);
    u  = x(4);
    v  = x(5);
    w  = x(6);
    e0 = x(7);
    e1 = x(8);
    e2 = x(9);
    e3 = x(10);
    p  = x(11);
    q  = x(12);
    r  = x(13);

    % Use Quaternion2Euler from tools/ folder
    [phi, theta, psi] = Quaternion2Euler([e0; e1; e2; e3]);

    y = [pn; pe; pd; u; v; w; phi; theta; psi; p; q; r];
    sys = y;

end

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

end

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

end