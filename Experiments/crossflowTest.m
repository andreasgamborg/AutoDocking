close all
clear all
clc
%%
L = 2;
B = 0.2500;
T = 0.1341;

v_list = 0:0.01: 0.2572;    % 0 - 0.5   knots
r_list = 0:0.002: 0.0349;    % 0 - 2     deg/s

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

% figure
% plot3(yawdamp(1,:),yawdamp(2,:),yawdamp(3,:),'.')
% figure
% plot3(swaydamp(1,:),swaydamp(2,:),swaydamp(3,:),'o')
% figure
% plot3(swaydamp(1,:),swaydamp(2,:),[yawdamp(3,:);swaydamp(3,:)],'.')





swayfit = fit(swaydamp(1:2,:)',swaydamp(3,:)','poly11');
figure('Name','SwayDaming','DefaultAxesFontSize',26)
plot(swayfit,swaydamp(1:2,:)',swaydamp(3,:)')

yawfit = fit(yawdamp(1:2,:)',yawdamp(3,:)','poly11');
figure('Name','YawDaming','DefaultAxesFontSize',26)
plot(yawfit,yawdamp(1:2,:)',yawdamp(3,:)')

swaycoef = coeffvalues(swayfit);
yawcoef = coeffvalues(yawfit);

%%
syms v r real

if (length(swaycoef) == 3)
    swaydamp = swaycoef * [1 v r]';
    yawdamp = yawcoef * [1 v r]';
end
if (length(swaycoef) == 6)
    swaydamp = swaycoef * [1 v r v^2 v*r r^2]';
    yawdamp = yawcoef * [1 v r v^2 v*r r^2]';
end

S.tau_cf = [ 0 swaydamp 0 0 0 yawdamp]';

%%
save('Experiments/LinearCFdamp.mat','-struct','S')





