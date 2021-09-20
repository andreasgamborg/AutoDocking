
clc

L = 2;
B = 0.2500;
T = 0.1341;

r = 1;

cfList = [];
for v = 0:0.1:3

nu_r = [0 v 0 0 0 r]';

cf = crossFlowDrag(L,B,T,nu_r);
cfList = [cfList cf];
end
cfList