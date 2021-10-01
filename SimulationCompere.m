%% Clean
close all
clear all
clc

%% Init
Ts = 1/100;
N = 6000;
t = 0; % Start time
T = [];

% Vessel
Model = 'Models/Primitive/otter3mtrx_lin.mat'

load(Model);
%load('Models/Primitive/otter3mtrx.mat');
% 
% C(1,1) = toKnots(1);            % knot per m/s
% C(3,3) = 60*180/pi;             % deg/min per rad/s

O6 = Otter6;
O3 = Otter3(Model);


%% Control
nu = [0 0 0]';
tau = [0 0 0]';
Q = diag([1 0 10000]);
R = diag([1 10000 1]);

[K,P,E] = lqr(A,B,Q,R);
%K = eye(3);
%K = place(A,B,[-2 -2 -2]')
%K = place(A,B,eig(A)*1.1)

r = [[3 0 0]' [1 0 480]'];
rt = 1;
% scale ref
Ak = A-B*K;
Cref = -C*inv(Ak)*B;
r = inv(Cref)*r;
% r(2) = 0;
%% Main Loop
disp('Running Simulation...')
for i = 1:N
    
    if(i==N/4), rt = rt+1; end
    
    nu = O6.State([1 2 6]);
    tau = -K*nu + r(:,rt);
    
    tau = [tau(1:2); 0; 0; 0; tau(3)];
    
%     tau(2)=0;
        
    O3.Thrust = tau;
    O6.Thrust = tau;

    O3.step(Ts);
    O6.step(Ts);

    T = [T t];
    t = t+Ts;
end
disp('Simulation done!')

O6.plot(T)
% O6.getMeasurement()


