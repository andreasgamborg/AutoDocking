
clear all
clc

%%
% path = 'Rute søkort.png';
% pixelsPerMeter = 3;
% background = imread(path);
% [N,M,~] = size(background);
% 
% close all
% fig = figure('Name','Track','DefaultAxesFontSize',20,'OuterPosition', [100           100        1720        880]);
% imagesc([-M/pixelsPerMeter M/pixelsPerMeter], [-N/pixelsPerMeter N/pixelsPerMeter], background); hold on
% axis equal
% 
% P = ginput(10);
% P = P';

%% Sine curve

x = 0:1:200;
y = 20*sin(0.05*x);

P = [x;y];
%%
close all
figure 
plot(P(1,:),P(2,:),'o-')
axis equal

%%
save('Course/sinecurve.mat','P');


%%
legSize = 50;
leg1 = [0:legSize ; ones(1,legSize+1)*0];
leg2 = [ones(1,legSize)*legSize; 1:legSize];
leg3 = [legSize-1:-1:0; ones(1,legSize)*legSize];
leg4 = [ones(1,legSize)*0; legSize-1:-1:0];

P = [leg1 leg2 leg3 leg4];

close all
figure 
plot(P(1,:),P(2,:),'o-')
axis equal

save('Course/square.mat','P');


%%
r = 30;

w  = 0:0.05:2*pi;
x = r* cos(w)-r;
y = r* sin(w);



P = [x;y];

close all
figure 
plot(P(1,:),P(2,:),'o-')
axis equal

save('Course/circle.mat','P');

%% Cos + Finish

x = 0:200;
y = 20*cos(2*pi*1/100*x)-20;

Pc = [x;y];

Pf = Pc(:,end)+[1:20;zeros(1,20)];

P = [Pc Pf];
close all
figure 
plot(P(1,:),P(2,:),'o-')
axis equal

%%
save('Course/CosFin.mat','P');



%% 8-loop
close all
x = 0:0.5:10;
nl = length(x);
line1 = [x;x; -pi/4*ones(1,nl)];
x = -10:0.5:0;
nl = length(x);
line3 = [x;x; -pi/4*ones(1,nl)];
x = -10:0.5:10;
nl = length(x);
line2 = [x;-x; -3*pi/4*ones(1,nl)];
line2 = flip(line2,2);



r = sqrt(200);
w  = 3*pi/4:-0.05:-3*pi/4+0.05;
x = r* cos(w);
y = r* sin(w);
arc1 = [x;y;wrapToPi(-w+pi/2)]+[20;0;0];
w  = pi/4:0.05:7*pi/4-0.05;
x = r* cos(w);
y = r* sin(w);
arc2 = [x;y;wrapToPi(-(w+pi/2))]+[-20;0;0];


figure
plot(line1(1,:),line1(2,:),'ro'), hold on;
plot(line2(1,:),line2(2,:),'ro')
plot(line3(1,:),line3(2,:),'go')
plot(arc1(1,:),arc1(2,:),'bo')
plot(arc2(1,:),arc2(2,:),'bo')
axis equal

P = [line1 arc1 line2 arc2 line3];


figure
plot(P(3,:))
figure
plot(P(1,:),P(2,:))

save('Course/Figure8.mat','P');



