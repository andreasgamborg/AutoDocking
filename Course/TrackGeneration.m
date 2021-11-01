
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