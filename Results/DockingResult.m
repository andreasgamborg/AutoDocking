%% Clean
close all
clear all
clc


load('Docking-mild.mat');   mild = Efinal;
load('Docking-fair.mat');   fair = Efinal;
load('Docking-harsh.mat');  harsh = Efinal;
load('Docking-extreme.mat');extreme = Efinal;


E = [mild fair harsh extreme];

title = 'cummulativ error';
names = "$"+["E_p","E_\psi"]+"$";
niceplot(1:4,E(1:2,:), names, title, ["-o"], ["", ""], 'southeast');
xticklabels({'mild' 'fair' 'harsh' 'extreme'})
