close all
clear all
clc


%%
L = 2;
B = 0.2500;
T = 0.1341;

%% 11 FIT

% 0.2572;    % 0 - 0.5   knots
% 0.0349;    % 0 - 2     deg/s
max_sway = 0.2572/2;                           % Fitting range sway
max_yaw = 0.0349/2;                          % Fitting range yaw
npoints = 31;                          % Fitting resolution

v_list =  -max_sway : 2*max_sway/(npoints-1)  : max_sway;
r_list =  -max_yaw : 2*max_yaw/(npoints-1)  : max_yaw;


swaydamp = [];
yawdamp = [];

for v = v_list
    for r = r_list
        nu_r = [0 v 0 0 0 r]';
        cf = crossFlowDrag(L,B,T,nu_r);
        
        swaydamp = [swaydamp [v r cf(2)]'];
        yawdamp = [yawdamp [v r cf(6)]'];
    end
end

swayfit11 = fit(swaydamp(1:2,:)',swaydamp(3,:)',"poly11");
yawfit11 = fit(yawdamp(1:2,:)',yawdamp(3,:)','poly11');

% figure('Name','SwayDamingFitting','DefaultAxesFontSize',20)
% plot(swayfit11,swaydamp(1:2,:)',swaydamp(3,:)'); hold on
% 
% figure('Name','YawDamingFitting','DefaultAxesFontSize',20)
% plot(yawfit11,yawdamp(1:2,:)',yawdamp(3,:)'); hold on

%% 22 FIT

% 0.2572;    % 0 - 0.5   knots
% 0.0349;    % 0 - 2     deg/s
max_sway = 0.2572;                           % Fitting range sway
max_yaw = 0.0349;                          % Fitting range yaw
npoints = 31;                          % Fitting resolution

v_list =  0 : max_sway/(npoints-1)  : max_sway;
r_list =  0 : max_yaw/(npoints-1)  : max_yaw;


swaydamp = [];
yawdamp = [];

for v = v_list
    for r = r_list
        nu_r = [0 v 0 0 0 r]';
        cf = crossFlowDrag(L,B,T,nu_r);
        
        swaydamp = [swaydamp [v r cf(2)]'];
        yawdamp = [yawdamp [v r cf(6)]'];
    end
end
swayfit22 = fit(swaydamp(1:2,:)',swaydamp(3,:)',"poly22");
yawfit22 = fit(yawdamp(1:2,:)',yawdamp(3,:)','poly22');

% figure('Name','SwayDamingFitting','DefaultAxesFontSize',20)
% plot(swayfit22,swaydamp(1:2,:)',swaydamp(3,:)');
% 
% figure('Name','YawDamingFitting','DefaultAxesFontSize',20)
% plot(yawfit22,yawdamp(1:2,:)',yawdamp(3,:)');


%%
syms v r real
tau_cf11 =[0 v 0 0 0 r]'; % Trick to make tau_cf symbolic
tau_cf22 =[0 v 0 0 0 r]'; % Trick to make tau_cf symbolic


tau_cf11(2) = coeffvalues(swayfit11) * [1 v r]';
tau_cf11(6) = coeffvalues(yawfit11) * [1 v r]';


tau_cf22(2) = coeffvalues(swayfit22) * [1 v r abs(v)*v abs(v)*r abs(r)*r]';
tau_cf22(6) = coeffvalues(yawfit22) * [1 v r abs(v)*v abs(v)*r abs(r)*r]';


latexeq("\bm{\tau}_{cf}",tau_cf11)
latexeq("\bm{\tau}_{cf}",tau_cf22)

%%
max_sway = max_sway;                 % Test range sway
max_yaw = max_yaw;                   % Test range yaw
npoints = 21;                          % Test resolution

v_list = -max_sway  : 2*max_sway/(npoints-1)  : max_sway;
r_list = -max_yaw : 2*max_yaw/(npoints-1) : max_yaw;


%%
% swaydamp = [];
% yawdamp = [];
% 
% for v = v_list
%     for r = r_list
%         nu_r = [0 v 0 0 0 r]';
% 
%         cf = crossFlowDrag(L,B,T,nu_r);     % Real
%         tau11 = eval(tau_cf11);                 % Fitted
%         tau22 = eval(tau_cf22);                 % Fitted
% 
%         swaydamp = [swaydamp [v r cf(2) tau11(2) tau22(2)]'];
%         yawdamp = [yawdamp [v r cf(6) tau11(6) tau22(6)]'];
%     end
% end
% 
% 
% figure('Name','YawDampingAll','DefaultAxesFontSize',20)
% plot3(yawdamp(1,:),yawdamp(2,:),yawdamp(3:end,:),'o','Linewidth',3)
% legend('real','fit 11', 'fit 22')
% 
% figure('Name','SwayDampingAll','DefaultAxesFontSize',20)
% plot3(swaydamp(1,:),swaydamp(2,:),swaydamp(3:end,:),'o','Linewidth',3)
% legend('real','fit 11', 'fit 22')


%%
swaydamp = [];
yawdamp = [];
for vt = 1:length(v_list)
    for rt = 1:length(r_list)
        v = v_list(vt);
        r = r_list(rt);
        nu_r = [0 v 0 0 0 r]';
        
        cf = crossFlowDrag(L,B,T,nu_r);         % Real
        tau11 = eval(tau_cf11);                 % Fitted
        tau22 = eval(tau_cf22);                 % Fitted
        
        swaydamp = [swaydamp [v r cf(2)]'];
        swaydampfit(rt,vt,1) = tau11(2);
        swaydampfit(rt,vt,2) = tau22(2);
        
        yawdamp = [yawdamp [v r cf(6)]'];
        yawdampfit(rt,vt,1) = tau11(6);
        yawdampfit(rt,vt,2) = tau22(6);
    end
end



figure('Name','SwayDampingAll','DefaultAxesFontSize',20)
    plot3(swaydamp(1,:),swaydamp(2,:),swaydamp(3,:),'o','MarkerSize',6,'MarkerFaceColor','blue','MarkerEdgeColor','white'); hold on
    surf(v_list,r_list,swaydampfit(:,:,1));
    surf(v_list,r_list,swaydampfit(:,:,2)); hold off
    colorbar; grid on
    xlabel('v [m/s]'); ylabel('r [rad/s]'); zlabel('drag')

figure('Name','YawDampingAll','DefaultAxesFontSize',20)
    plot3(yawdamp(1,:),yawdamp(2,:),yawdamp(3,:),'o','MarkerSize',6,'MarkerFaceColor','blue','MarkerEdgeColor','white'); hold on
    surf(v_list,r_list,yawdampfit(:,:,1));
    surf(v_list,r_list,yawdampfit(:,:,2)); hold off
    colorbar; grid on
    xlabel('v [m/s]'); ylabel('r [rad/s]'); zlabel('drag')





%%
% range = 1000
% mid = ceil(npoints^2/2)
% rangeidx = [mid-range:mid+range]
% figure('Name','YawDampingSmall','DefaultAxesFontSize',20)
% plot3(yawdamp(1,rangeidx),yawdamp(2,rangeidx),yawdamp(3:4,rangeidx),'o')
% figure('Name','SwayDampingSmall','DefaultAxesFontSize',20)
% plot3(swaydamp(1,rangeidx),swaydamp(2,rangeidx),swaydamp(3:4,rangeidx),'o')

