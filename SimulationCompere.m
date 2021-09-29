%% Clean
close all
clear all
clc

%% Init
Ts = 1/50;
N = 6000;
t = 0; % Start time
T = [];

% Init State
%% Vessel

O3lin = Otter3('Models/Primitive/otter3mtrx_lin.mat');
O3 = Otter3('Models/Primitive/otter3mtrx.mat');
O6 = Otter6;

Model = O3lin;
%% Main Loop
nu = [0 0 0]';
tau = [0 0 0]';
A = Model.A
B = Model.B
K = eye(3);
%K = place(A,B,[-2 -2 -2]')
K = place(A,B,eig(A)*1.1)
r = [10 0 0.5]';

disp('Running Simulation...')
for i = 1:N
    nu = O6.State([1 2 6]);
    tau = -K*nu + r;
    
    tau = [tau(1:2); 0; 0; 0; tau(3)];
    
    Model.Thrust = tau;
    O6.Thrust = tau;

    
    Model.step(Ts);    
    O6.step(Ts);

    T = [T t];
    t = t+Ts;
end
disp('Simulation done!')

Model.plot(T)
O6.plot(T)



