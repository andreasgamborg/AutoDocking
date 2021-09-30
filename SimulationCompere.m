%% Clean
close all
clear all
clc

%% Init
Ts = 1/50;
N = 6000;
t = 0; % Start time
T = [];

% Vessel
Model = 'Models/Primitive/otter3mtrx_lin.mat'

load(Model);
%load('Models/Primitive/otter3mtrx.mat');

C = eye(3);
C(1,1) = toKnots(1);            % knot per m/s
C(2,2) = toKnots(1);            % knot per m/s
C(3,3) = 60*180/pi;             % deg/min per rad/s

O6 = Otter6;
O3 = Otter3(Model);


%% Control
nu = [0 0 0]';
tau = [0 0 0]';
Q = diag([1 10 100]);
R = eye(3);

[K,P,E] = lqr(A,B,Q,R);
%K = eye(3);
%K = place(A,B,[-2 -2 -2]')
%K = place(A,B,eig(A)*1.1)

r = [1 0 20]';

% scale ref
Ak = A-B*K;
Cref = -C*inv(Ak)*B;
r = inv(Cref)*r;
% r(2) = 0;
%% Main Loop
disp('Running Simulation...')
for i = 1:N
    nu = O6.State([1 2 6]);
    tau = -K*nu + r;
    
    tau = [tau(1:2); 0; 0; 0; tau(3)];
    
    O3.Thrust = tau;
    O6.Thrust = tau;

    O3.step(Ts);
    O6.step(Ts);

    T = [T t];
    t = t+Ts;
end
disp('Simulation done!')

%O6.plot(T)



