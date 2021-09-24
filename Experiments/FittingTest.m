close all
clear all
clc
%%
L = 2;
B = 0.2500;
T = 0.1341;

% 0.2572;    % 0 - 0.5   knots
% 0.0349;    % 0 - 2     deg/s
max_sway = 0.2572;                           % Fitting range sway
max_yaw = 0.0349;                          % Fitting range yaw
npoints = 20;                          % Fitting resolution

v_list =  0 : max_sway/npoints  : max_sway;
r_list =  0 : max_yaw/npoints : max_yaw;


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
swayfit = fit(swaydamp(1:2,:)',swaydamp(3,:)',"poly22");
figure('Name','SwayDamingFitting','DefaultAxesFontSize',20)
plot(swayfit,swaydamp(1:2,:)',swaydamp(3,:)')

yawfit = fit(yawdamp(1:2,:)',yawdamp(3,:)','poly22');
figure('Name','YawDamingFitting','DefaultAxesFontSize',20)
plot(yawfit,yawdamp(1:2,:)',yawdamp(3,:)')


%%
syms v r real
tau_cf =[0 v 0 0 0 r]'; % Trick to make tau_cf symbolic

tau_cf(2) = coeffvalues(swayfit) * [1 v r abs(v)*v abs(v)*r abs(r)*r]';
tau_cf(6) = coeffvalues(yawfit) * [1 v r abs(v)*v abs(v)*r abs(r)*r]';



%%
max_sway = 10
max_yaw = 5

v_list = -max_sway  : 2*max_sway/npoints  : max_sway;   
r_list = -max_yaw : 2*max_yaw/npoints : max_yaw;  

swaydamp = [];
yawdamp = [];
for v = v_list
    for r = r_list
        nu_r = [0 v 0 0 0 r]';
        cf = crossFlowDrag(L,B,T,nu_r);
        
        tau = eval(tau_cf);
        
        swaydamp = [swaydamp [v r cf(2) tau(2)]'];
        yawdamp = [yawdamp [v r cf(6) tau(6)]'];
    end
end

%%
figure('Name','YawDampingAll','DefaultAxesFontSize',20)
plot3(yawdamp(1,:),yawdamp(2,:),yawdamp(3:4,:),'o')
figure('Name','SwayDampingAll','DefaultAxesFontSize',20)
plot3(swaydamp(1,:),swaydamp(2,:),swaydamp(3:4,:),'o')


%%
% range = 1000
% mid = ceil(npoints^2/2)
% rangeidx = [mid-range:mid+range]
% figure('Name','YawDampingSmall','DefaultAxesFontSize',20)
% plot3(yawdamp(1,rangeidx),yawdamp(2,rangeidx),yawdamp(3:4,rangeidx),'o')
% figure('Name','SwayDampingSmall','DefaultAxesFontSize',20)
% plot3(swaydamp(1,rangeidx),swaydamp(2,rangeidx),swaydamp(3:4,rangeidx),'o')

