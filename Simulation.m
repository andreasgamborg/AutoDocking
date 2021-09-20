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
%% Vessel
vesselsList = {'Otter 6DOF','Otter 3DOF','Ferry 3DOF'};
vesselObj = {Otter6, Otter3, Ferry3};

[vidx,tf] = listdlg('ListString',vesselsList);
if(tf), Vessel = vesselObj{vidx};   else, return; end
clear vidx tf vesselsList vesselObj
%% Current
if (0)                          % ON-OFF
    Vessel.Current.v = 0.2;
    Vessel.Current.beta = 3*pi/2;
end
%% Manuel Thruster Values

thrustList = ["Idle","Forward","Turn","Full - Break - Full","S-Curve","Large circle","Port Thruster half rotation",...
    "Starbord Thruster half rotation","Slalom by use of thruster angle"];
[midx,tf] = listdlg('ListString',thrustList);
if(tf==0), return; end
switch midx
    case 1  % Idle
        thrust_seq = makeref([0; 0; 0; 0], N);
    case 2  % Forward
        a = 30;
        thrust_seq = makeref([a; a; 0; 0], N);
    case 3  % Turn
        thrust_seq = makeref([20 0 20; 20 20 20; 0 0 0; 0 0 0], N/3);
    case 4  % Full - Break - Full
        thrust_seq = makeref([30 0 30; 30 0 30; 0 0 0 ; 0 0 0], N/3);
    case 5  % S-Line
        thrust_seq = makeref([30 30 20 30; 30 20 30 30; 0 0 0 0; 0 0 0 0], N/4);
    case 6  % Large circle
        thrust_seq = makeref([25; 20; 0; 0], N);
    case 7  % Port Thruster half rotation
        thrust_seq = makeref([20 20 20 20 20; 0 0 0 0 0 ; 0 pi/4 pi/2 3*pi/4 pi; 0 0 0 0 0], N/5);
    case 8  % Starbord Thruster half rotation
        thrust_seq = makeref([0 0 0 0 0; 20 20 20 20 20 ; 0 0 0 0 0; 0 -pi/4 -pi/2 -3*pi/4 -pi], N/5);
    case 9  % Slalom by use of thruster angle
        a = 12;
        thrust_seq = makeref([20 20 20 20 20; 20 20 20 20 20 ; 0 -pi/a pi/a -pi/a pi/a; 0 -pi/a pi/a -pi/a pi/a], N/5);
    otherwise
        disp("Not valid manuver")
        thrust_seq = makeref([0; 0; 0; 0], N);
end

%% Main Loop
disp('Running Simulation...')
for i = 1:N
    Vessel.setProp(thrust_seq(1:2,i), thrust_seq(3:4,i));
    Vessel.step(Ts);
    T = [T t];
    t = t+Ts;
end
disp('Simulation done!')

Vessel.plot(T)
% History = Vessel.History;
% Plots
