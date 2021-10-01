%% Clean
close all
clear all
clc

%% Init
Ts = 1/50;
N = 6000;
t = 0; % Start time
T = [];

%% Vessel
load('Models/Primitive/otter3mtrx_lin.mat')

[F,G] = c2d(A,B,Ts);
clear A B
% 
L = dlqe(F,G,C,eye(3),eye(3));
K = dlqr(F,G,diag([1 1 1]),diag([1 1 1]));

%% Init State
nu = [0 0 0]';
eta = [0 0 0]';
tau = [0 0 0]';
x = [nu; tau];
nuhat = [0 0 0]';

r = [10 0 1]';

nu_c = [0 0 0]';

%% Main Loop 3
disp('Running Simulation...')
for i = 1:N
    
    y = C*nu + randn(3,1);
    nuhat = nuhat + L*(y - C*nuhat);
    
    tau = -K*nuhat + r;
    
    nu = F*nu+G*tau; 
    
    % Euler integration for position
    etadot = Rot(eta(3)) * nu + nu_c;
    eta = eta + Ts * etadot;
    eta(3) = wrapToPi(eta(3));

    % Save
    History.nu(:,i) = nu;
    History.tau(:,i) = tau;
    History.eta(:,i) = eta;
    History.nuhat(:,i) = nuhat;
    
    % Time update
    nuhat = F*nuhat + G*tau; % Observer
    T = [T t];
    t = t+Ts;
end
disp('Simulation done!')


%% Plot
title = 'Course';
niceplot(History.eta(1,:),History.eta(2,:), [], title, ["--"], ["x [m]", "y [m]"], 'north');
axis equal
grid
set(gca, 'YDir','reverse');

color = 'g-';
for i = [1:1000:N, N]
    
    vessel = vesselplot(History.eta(3,i),[0;0]);
    
    if i == N
        color = 'r-';
    end
    
    plot(vessel(1,:)+History.eta(1,i), vessel(2,:)+History.eta(2,i), color, 'LineWidth', 2);
    
    color = 'b-';
end


title = 'Input';
names = ["$\tau_u$ surge", "$\tau_v$ sway", "$\tau_r$ yaw"];
niceplot(T,History.tau(1:3,:), names, title, ["--"], ["time [s]", "[N] [Nm]"], 'northwest');

title = 'Linear Velocities';
names = ["$u$ ", "$v$ ","$\hat{u}$ ", "$\hat{v}$ "];
niceplot(T,toKnots([History.nu(1:2,:);History.nuhat(1:2,:)],'m/s'), names, title, ["-","--"], ["time [s]", "[knot]"], 'northeast');

title = 'Orientation';
names = ["$\psi$ yaw"];
niceplot(T,rad2deg(History.eta(3,:)), names, title, ["--"], ["time [s]", ""], 'south');
yticks(-180:30:180)
tl = [180:30:359 0:30:180]+"°"; tl(7) = "N"; tl(10) = "E"; tl([1,13]) = "S"; tl(4) = "W";
yticklabels(tl)


title = 'Angular Velocities';
names = ["$r$ ","$\hat{r}$ "];
niceplot(T,rad2deg([History.nu(3,:);History.nuhat(3,:)]*60), names, title, ["--"], ["time [s]", "[deg/min]"], 'southeast');
ytickformat('%.0f°')


%% Functions
function R = Rot(psi)
R = [   cos(psi)   -sin(psi)    0
    sin(psi)    cos(psi)    0
    0           0           1 ];
end

