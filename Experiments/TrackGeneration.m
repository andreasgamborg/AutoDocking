
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

x = 0:2:100;
y = 10*sin(0.1*x);

P = [x;y];
%%

figure 
plot(P(1,:),P(2,:))
axis equal