
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





