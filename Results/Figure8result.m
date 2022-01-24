%% Clean
close all
clear all
clc

%%
load('Figure8-mild.mat');   mild = Efinal;
load('Figure8-fair.mat');   fair = Efinal;
load('Figure8-harsh.mat');  harsh = Efinal;
load('Figure8-extreme.mat');extreme = Efinal;


E = [mild fair harsh extreme];

title = 'cummulativ figure8 error';
names = "$"+["E_p","E_\psi"]+"$";
niceplot(1:4,E(1:2,:), names, title, ["-o"], ["", ""], 'southeast');
xticklabels({'mild' 'fair' 'harsh' 'extreme'})

%%
load('Docking-mild.mat');   mild = Efinal;
load('Docking-fair.mat');   fair = Efinal;
load('Docking-harsh.mat');  harsh = Efinal;
load('Docking-extreme.mat');extreme = Efinal;


E = [mild fair harsh extreme];

title = 'cummulativ docking error';
names = "$"+["E_p","E_\psi"]+"$";
niceplot(1:4,E(1:2,:), names, title, ["-o"], ["", ""], 'southwest');
xticklabels({'mild' 'fair' 'harsh' 'extreme'})

