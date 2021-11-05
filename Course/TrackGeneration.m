
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

%%

x = 0:5:200;
y = 20*sin(0.05*x);

P = [x;y];
%%
close all
figure 
plot(P(1,:),P(2,:))
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


