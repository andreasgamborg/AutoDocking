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
load('Models/Primitive/otter3mtrx_lin.mat')
% load('Models/Primitive/otter6mtrx.mat')
% A6 = -(MRB + MA)\(CRB + CA + D);
% B6 = inv(MRB + MA);
clear MRB MA CRB CA D
%% Main Loop 3
nu = [0 0 0]';
tau = [0 0 0]';
x = [nu; tau];

K = eye(3);
K = place(A,B,[-2 -2 -2]')
%K = place(A,B,eig(A)*1.1)
r = [10 0 0.5]';

nu_c = [0 0 0]';

disp('Running Simulation...')
for i = 1:N
    nu = x(1:3);
    eta = x(4:6);
    psi = eta(3);
    
    % Save
    History.nu(:,i) = nu;
    History.tau(:,i) = tau;
    History.eta(:,i) = eta;
    %
    tau = -K*nu + r;
    %
    nudot = A*nu+B*tau;
    etadot = Rot(psi) * nu + nu_c;
    
    xdot = [nudot; etadot];
    
    % Euler integration
    x = x + Ts * xdot;
    x(6) = wrapToPi(x(6));
    % Time update
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
niceplot(T,toKnots(History.tau(1:3,:),'m/s'), names, title, ["--"], ["time [s]", "[N] [Nm]"], 'northwest');

title = 'Linear Velocities';
names = ["$u$ surge", "$v$ sway"];
niceplot(T,toKnots(History.nu(1:2,:),'m/s'), names, title, ["--"], ["time [s]", "[knot]"], 'northeast');

title = 'Orientation';
names = ["$\psi$ yaw"];
niceplot(T,rad2deg(History.eta(3,:)), names, title, ["--"], ["time [s]", ""], 'south');
yticks(-180:30:180)
tl = [180:30:359 0:30:180]+"°"; tl(7) = "N"; tl(10) = "E"; tl([1,13]) = "S"; tl(4) = "W";
yticklabels(tl)


title = 'Angular Velocities';
names = ["$r$ yaw"];
niceplot(T,rad2deg(History.nu(3,:)*60), names, title, ["--"], ["time [s]", "[deg/min]"], 'southeast');
ytickformat('%.0f°')


%% Functions
function R = Rot(psi)
R = [   cos(psi)   -sin(psi)    0
    sin(psi)    cos(psi)    0
    0           0           1 ];
end

