%% Clean
close all
clear all
clc

%% Init
Ts = 1/50;
N = 10000;
t = 0; % Start time
T = [];

%% Real Vessel
state(12,1) = pi/2;
O6 = Otter6(state);
O6.UseProppeller = true;
%% Model
Model = 'Models/Primitive/otter3mtrx_lin.mat'
load(Model);

%% Controller
Q = diag([1000 1000 10000]);    % Stateerror cost
R = diag([0.1 10 1]);           % Input cost
[K,P,E] = lqr(A,B,Q,R);

%% Observer
Cm = eye(3);                    % Measurement matrix
Q = diag([1000 1000 1000]);     % Process noise
R = diag([0.1 0.1 1]);          % Measurement noise
[L,P,E] = lqe(A,B,Cm,Q,R);

%% Reference
r = [100 0 0]';
r = [[3 0 0]' [1 0 110]' [0 -1 0]'];
r = [[2 0 0]' [1 0 110]' [2 0 0]'];
%r = [[3 0 0]' [5 0 0]' [3 0 0]'];
rt = 1;
%% Reference Scaling
Ak = A-B*K;
Cref = -C*inv(Ak)*B;
r = inv(Cref)*r;

%% Init State
nuhat = [0 0 0]';

%% Main Loop
disp('Running Simulation...')
for it = 1:N
    Mea = O6.getMeasurement();
    ym = Mea.ym;
    num = ym(1:3);
    
    if(it==N/4), rt = rt+1; end
    if(it==3*N/4), rt = rt+1; end
    tau = -K*nuhat + r(:,rt);
    tau6 = [tau(1:2); zeros(3,1); tau(3)];
    
    % Input
    O6.controlAllocation(tau6);
    O6.step(Ts);
    
    % Observer
    dnuhat = A*nuhat + B*tau + L*(num - Cm*nuhat);
    nuhat = nuhat + Ts*dnuhat;
    
    % Save
    History.course(:,it) = Mea.Course;
    History.SOG(:,it) = Mea.SOG;
    History.tau_r(:,it) = tau;
    History.tau_a(:,it) = O6.Thrust([1 2 6]);
    
    History.num(:,it) = num;
    History.nuhat(:,it) = nuhat;
    History.nu(:,it) = O6.State([1 2 6]);
    
    History.phim(:,it) = ym(6);
    History.phi(:,it) = O6.State(12);
    History.posm(:,it) = ym(4:5);
    History.pos(:,it) = O6.State(7:8);
    
    % Time update
    T = [T t];
    t = t+Ts;
    
end
disp('Simulation done!')

%% Plotting
O6.plot(T)

disp('Press any key to show estimates'), pause
close all
% u
title = 'Linear Velocities';
names = ["$u_m$ ", "$\hat{u}$", "$u$"];
niceplot(T,toKnots([History.num(1,:); History.nuhat(1,:); History.nu(1,:)],'m/s'), names, title, [".","-","-"], ["time [s]", "[knot]"], 'northwest');

% v
title = 'Linear Velocities';
names = ["$v_m$", "$\hat{v}$", "$v$"];
niceplot(T,toKnots([History.num(2,:); History.nuhat(2,:); History.nu(2,:)],'m/s'), names, title, [".","-","-"], ["time [s]", "[knot]"], 'north');

% r
title = 'Angular Velocities';
names = ["$r_m$", "$\hat{r}$", "$r$"];
niceplot(T,[History.num(3,:); History.nuhat(3,:); History.nu(3,:)], names, title, [".","-","-"], ["time [s]", "[rad/s]"], 'northeast');

% tau
title = 'Thrust requested vs. applied';
names = ["$\tau_u $","$\tau_v$","$\tau_r$","$\tau_u $","$\tau_v$","$\tau_r$"];
niceplot(T, [History.tau_a; History.tau_r], names, title, ["-","--"], ["time [s]", "[N]"], 'southwest');
