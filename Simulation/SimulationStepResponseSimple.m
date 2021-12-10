%% Clean
close all
clear all
clc

%% Init
Ts = 1/100;
N = 1000;
t = 0; % Start time
T = [];

%% Init
nu = zeros(3,1);
eta = zeros(3,1);
tau = [0 0 0]';

%% Main Loop
disp('Running Simulation...')
for it = 1:N
    
    % Input
        if(it == 1/Ts)
            tau = [0 1 0]';
        end
        
    % Rotation
        psi = eta(3);
        R = [   cos(psi)   -sin(psi)    0
            sin(psi)    cos(psi)    0
            0                  0    1   ];
        
    % Model step
        [M,C,D] = sysmtrx3DOF(nu);
        dnu = M\(tau-C*nu-D*nu);
        deta = R*nu;

        nu = nu + Ts*dnu;
        eta = eta + Ts*deta;
    
    % Save
        History.tau_r(:,it) = tau;
        History.tau_a(:,it) = tau;

        History.nu(:,it) = nu;
        History.eta(:,it) = eta;
    
    % Time update
        T = [T t];
        t = t+Ts;
end
disp('Simulation done!')

clear u v r

%% Plotting
close all

title = 'Position';
names = "$"+["x", "y", "\psi"]+"$";
niceplot(T,History.eta, names, title, ["-"], ["time [s]", "m - rad"], 'north');
title = 'Velocities';
names = "$"+["u", "v", "r"]+"$";
niceplot(T,History.nu, names, title, ["-"], ["time [s]", "m/s - rad/s"], 'south');

