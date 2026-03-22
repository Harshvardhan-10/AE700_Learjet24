% % run_autopilot_sim.m
% %   Simulates the autopilot pipeline in MATLAB using the same blocks
% %   as the Simulink model:
% %       forces_moments.m  ->  mav_dynamics.m  ->  autopilot.m
% %
% %   Reference: Beard & McLain, Ch. 3-6
% %   Run from chap6/ after all .mat files are generated.

addpath('../parameters')
addpath('../tools')
addpath('../chap3')   
addpath('../chap4')   
aerosonde_parameters
load('trim_results.mat')
load('autopilot_gains.mat')

dt  = AP.Ts;
r2d = 180/pi;
wind = zeros(6,1);

function uu = make_uu(x, Va, alpha, beta, Va_c, h_c, chi_c, t, long_mode)
    pn=x(1); pe=x(2); pd=x(3);
    phi=x(7); theta=x(8); psi=x(9);
    p=x(10); q=x(11); r=x(12);
    h   = -pd;
    chi = atan2(Va*sin(psi), Va*cos(psi));  
    Vg  = Va;
    uu  = [pn;pe;h;Va;alpha;beta;phi;theta;chi;p;q;r;Vg;0;0;psi;0;0;0; ...
           Va_c; h_c; chi_c; t; long_mode]; % Added long_mode at the end
end

function [x_new, Va, alpha, beta] = step(x, delta, wind, MAV, dt, t)
    fm    = forces_moments(x, delta, wind, MAV);
    u_dyn = fm(1:6);                     
    Va    = fm(7); alpha = fm(8); beta  = fm(9);
    xdot  = mav_dynamics(t, x, u_dyn, 1, MAV);
    x_new = x + dt * xdot;
end

function [T, X, DELTA, CMDS, VA, ALPHA, BETA] = run_sim(x0, Va_c, h_c, chi_c, ...
                                                    t_end, dt, wind, MAV, AP, mode)
    N = round(t_end/dt) + 1;
    T = (0:dt:(N-1)*dt)';
    X = zeros(12,N);  DELTA = zeros(4,N); CMDS = zeros(12,N);
    VA = zeros(1,N);  ALPHA = zeros(1,N);  BETA = zeros(1,N);
    x = x0;  X(:,1) = x;

    fm = forces_moments(x,[0;0;0;AP.delta_t_trim],wind,MAV);
    Va=fm(7); al=fm(8); be=fm(9);

    for k = 1:N-1
        VA(k)=Va; ALPHA(k)=al; BETA(k)=be;
        uu    = make_uu(x,Va,al,be,Va_c,h_c,chi_c,T(k), mode);
        y     = autopilot(uu, AP);
        DELTA(:,k) = y(1:4);
        CMDS(:,k)  = y(5:16);
        [x, Va, al, be] = step(x, y(1:4), wind, MAV, dt, T(k));
        X(:,k+1) = x;
    end
    VA(N)=Va; ALPHA(N)=al; BETA(N)=be;
    DELTA(:,N) = DELTA(:,N-1);
    CMDS(:,N)  = CMDS(:,N-1);
end

h_trim = -x_trim(3);

% Sim 1: Course step 45 deg (Mode 1)
fprintf('Sim 1: course step 45 deg...\n');
clear autopilot
[T1,X1,D1,C1,VA1,~,BE1] = run_sim(x_trim, Va_trim, h_trim, deg2rad(45), 200, dt, wind, MAV, AP, 1);

% Sim 2: Course step with aileron disturbance at t=30s (Mode 1)
fprintf('Sim 2: course step with disturbance at t=30s...\n');
clear autopilot
N2 = round(200/dt)+1; T2 = (0:dt:(N2-1)*dt)';
X2 = zeros(12,N2); D2 = zeros(4,N2); C2 = zeros(12,N2);
x  = x_trim;
fm = forces_moments(x,[0;0;0;AP.delta_t_trim],wind,MAV);
Va=fm(7); al=fm(8); be=fm(9);
for k = 1:N2-1
    uu = make_uu(x,Va,al,be,Va_trim,h_trim,deg2rad(45),T2(k), 1);
    y  = autopilot(uu,AP);
    delta = y(1:4); C2(:,k) = y(5:16);
    if T2(k) >= 30, delta(2) = delta(2) + 0.2; end % Disturbance
    D2(:,k) = delta;
    [x,Va,al,be] = step(x,delta,wind,MAV,dt,T2(k));
    X2(:,k+1) = x;
end
D2(:,end) = D2(:,end-1); C2(:,end) = C2(:,end-1);

% Sim 3: Altitude step +20 m (Mode 1)
fprintf('Sim 3: altitude step +20 m...\n');
clear autopilot
[T3,X3,D3,~,~,~,~] = run_sim(x_trim, Va_trim, h_trim+20, 0, 200, dt, wind, MAV, AP, 1);

% Sim 4: Airspeed via throttle +5 m/s (Mode 1)
fprintf('Sim 4: airspeed via throttle +5 m/s...\n');
clear autopilot
[T4,X4,D4,~,VA4,~,~] = run_sim(x_trim, Va_trim+5, h_trim, 0, 200, dt, wind, MAV, AP, 1);

% Sim 5: Pitch step 5 deg (Mode 2)
fprintf('Sim 5: pure pitch step +5 deg...\n');
clear autopilot
[T5,X5,D5,C5,~,~,~] = run_sim(x_trim, Va_trim, deg2rad(5), 0, 200, dt, wind, MAV, AP, 2);

% Sim 6: Airspeed via pitch +5 m/s (Mode 3)
fprintf('Sim 6: airspeed via pitch +5 m/s...\n');
clear autopilot
[T6,X6,D6,C6,VA6,~,~] = run_sim(x_trim, Va_trim+5, h_trim, 0, 200, dt, wind, MAV, AP, 3);

% -----------------------------------------------------------------------
% Extract ALL 6-DOF signals for plotting
% -----------------------------------------------------------------------
chi1  = atan2(VA1.*sin(X1(9,:)), VA1.*cos(X1(9,:))) * r2d;
phi1  = X1(7,:) * r2d;
beta1 = BE1 * r2d;
da1   = D1(2,:) * r2d;
dr1   = D1(3,:) * r2d;
phic1 = C1(7,:) * r2d; % Actual commanded roll

phi2  = X2(7,:) * r2d;
da2   = D2(2,:) * r2d;
phic2 = C2(7,:) * r2d;

h3    = -X3(3,:);
de3   = D3(1,:) * r2d;

Va4   = VA4;
dt4   = D4(4,:);

theta5 = X5(8,:) * r2d;
thetac5 = C5(8,:) * r2d;
de5    = D5(1,:) * r2d;

Va6   = VA6;
thetac6 = C6(8,:) * r2d;

save('autopilot_sim_results.mat', ...
     'T1','chi1','phi1','beta1','da1','dr1','phic1', ...
     'T2','phi2','da2','phic2', ...
     'T3','h3','de3', ...
     'T4','Va4','dt4', ...
     'T5','theta5','thetac5','de5', ...
     'T6','Va6','thetac6', ...
     'h_trim','Va_trim','r2d','AP');
fprintf('Saved autopilot_sim_results.mat\n');