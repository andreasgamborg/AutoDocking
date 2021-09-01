%% Clean
close all
clear all
clc

%% Init
Ts = 1/50;
N = 3000;
t = 0; % Start time
T = [];

% Init State
F = Ferry3;
O = Otter6;
%% Manuel Thruster Values
% Idle
%      thrust_seq = makeref([0; 0; 0; 0], N);
% Slow
%      thrust_seq = makeref([10; 10; 0; 0], N);   
% Full - Break - Full
%      thrust_seq = makeref([30 0 30; 30 0 30; 0 0 0 ; 0 0 0], N/3);
% S-Line
%     thrust_seq = makeref([30 30 20 30; 30 20 30 30; 0 0 0 0; 0 0 0 0], N/4);
% Large circle
    thrust_seq = makeref([30; 25; 0; 0], N);
% Port Thruster half rotation
%     thrust_seq = makeref([20 20 20 20 20; 0 0 0 0 0 ; 0 pi/4 pi/2 3*pi/4 pi; 0 0 0 0 0], N/5);
% Starbord Thruster half rotation
    %thrust_seq = makeref([0 0 0 0 0; 20 20 20 20 20 ; 0 0 0 0 0; 0 -pi/4 -pi/2 -3*pi/4 -pi], N/5);
% Slalom by use of thruster angle
%     a = 12;
%     thrust_seq = makeref([20 20 20 20 20; 20 20 20 20 20 ; 0 -pi/a pi/a -pi/a pi/a; 0 -pi/a pi/a -pi/a pi/a], N/5);
    
%% Main Loop
Vessel = O
disp('Running Simulation...')
for i = 1:N
    Vessel.setProp(thrust_seq(1:2,i), thrust_seq(3:4,i));
    Vessel.step(Ts);
    T = [T t];
    t = t+Ts;
end
disp('Simulation done!')

History = Vessel.History;
Plots
