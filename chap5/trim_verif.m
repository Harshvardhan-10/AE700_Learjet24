% verify_turn_trim.m
% Run from chap6/ — requires trim_results_turn.mat and mavsim_chap5.slx
addpath('../parameters')
addpath('../tools')
addpath('../chap3')
addpath('../chap4')
aerosonde_parameters
load('trim_results_turn.mat')  % loads x_trim_t, u_trim_t
load('trim_results.mat')  % loads x_trim, u_trim

dt    = 0.02;
t_end = 60;
t     = (0:dt:t_end)';
N     = length(t);
wind  = zeros(6,1);

% Integrate using turn trim state and controls
x = x_trim_t;
X = zeros(12,N);  X(:,1) = x;
for k = 1:N-1
    fm   = forces_moments(x, u_trim_t, wind, MAV);
    xdot = mav_dynamics(t(k), x, fm(1:6), 1, MAV);
    x    = x + dt*xdot;
    X(:,k+1) = x;
end

r2d = 180/pi;
labels = {'p_n (m)','p_e (m)','p_d (m)','u (m/s)','v (m/s)','w (m/s)',...
          '\phi (deg)','\theta (deg)','\psi (deg)','p (deg/s)','q (deg/s)','r (deg/s)'};
scale  = [1 1 1 1 1 1 r2d r2d r2d r2d r2d r2d];

figure(1); clf; set(gcf,'Position',[50 50 1300 800]);
for k = 1:12
    subplot(3,4,k)
    plot(t, X(k,:)*scale(k), 'b-', 'LineWidth',1.5)
    ylabel(labels{k},'FontSize',9)
    xlabel('t (s)','FontSize',9)
    grid on; set(gca,'FontSize',9)
    if k == 9
        title('\psi — should change (turn)','FontSize',8,'Color','r')
    elseif k == 1 || k == 2
        title('changes during turn','FontSize',8,'Color',[0 0.5 0])
    else
        title('should stay constant','FontSize',8,'Color',[0 0.5 0])
    end
end
sgtitle('Turn Trim Verification (n=1.2) — only \psi should drift','FontSize',12,'FontWeight','bold')
saveas(gcf,'turn_trim_verification.png')
fprintf('Saved turn_trim_verification.png\n')

x = x_trim;
X = zeros(12,N);  X(:,1) = x;
for k = 1:N-1
    fm   = forces_moments(x, u_trim, wind, MAV);
    xdot = mav_dynamics(t(k), x, fm(1:6), 1, MAV);
    x    = x + dt*xdot;
    X(:,k+1) = x;
end

r2d = 180/pi;
labels = {'p_n (m)','p_e (m)','p_d (m)','u (m/s)','v (m/s)','w (m/s)',...
          '\phi (deg)','\theta (deg)','\psi (deg)','p (deg/s)','q (deg/s)','r (deg/s)'};
scale  = [1 1 1 1 1 1 r2d r2d r2d r2d r2d r2d];

figure(1); clf; set(gcf,'Position',[50 50 1300 800]);
for k = 1:12
    subplot(3,4,k)
    plot(t, X(k,:)*scale(k), 'b-', 'LineWidth',1.5)
    ylabel(labels{k},'FontSize',9)
    xlabel('t (s)','FontSize',9)
    grid on; set(gca,'FontSize',9)
    if k == 1 || k == 2
        title('changes during flight','FontSize',8,'Color',[0 0.5 0])
    else
        title('should stay constant','FontSize',8,'Color',[0 0.5 0])
    end
end
sgtitle('Trim Verification','FontSize',12,'FontWeight','bold')
saveas(gcf,'trim_verification.png')
fprintf('Saved trim_verification.png\n')