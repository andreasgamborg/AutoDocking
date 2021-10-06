%% Clean
close all
clear all
clc

%% Init
Ts = 1/50;
N = 6000;
t = 0; % Start time
T = [];

%% Real Vessel
O6 = Otter6;
O6.UseProppeller = true;
%% Model
Model = 'Models/Primitive/otter3mtrx_lin.mat'
load(Model);

%% Controller
Q = diag([1 1 10])*1000;        % Stateerror cost
R = diag([0.1 10 1]);           % Input cost
[K,P,E] = lqr(A,B,Q,R);

%% Observer
Cm = eye(3);                    % Measurement matrix
Q = eye(3)*1000;                % Process noise
R = eye(3)*0.01;                % Measurement noise
[L,P,E] = lqe(A,B,Cm,Q,R)

%% Reference
r = [100 0 0]';
r = [[3 0 0]' [1 0 110]' [0 -1 0]'];
r = [[3 0 0]' [5 0 0]' [3 0 0]'];
rt = 1;

%% Reference Scaling
Ak = A-B*K;
Cref = -C*inv(Ak)*B;
r = inv(Cref)*r;

%% Init State
nuhat = [0 0 0]';

%% Control
nu = [0 0 0]';
tau = [0 0 0]';
Q = diag([1 1 1]);
R = diag([1 1 1]);
[K,P,E] = lqr(A,B,Q,R);
%K = eye(3);
%K = place(A,B,[-2 -2 -2]')
%K = place(A,B,eig(A)*1.1)

r = [[3 0 0]' [1 0 110]' [0 -1 0]'];
r = [[3 0 0]' [5 0 0]' [3 0 0]'];
r = [[3 0 0]' [0 1 0]' [3 0 0]'];

rt = 1;
%% Reference Scaling
Ak = A-B*K;
Cref = -C*inv(Ak)*B;
r = inv(Cref)*r;
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
    History.nu(:,it) = O6.State([1 2 6]);
    History.num(:,it) = num;
    History.phim(:,it) = ym(6);
    History.phi(:,it) = O6.State((12);
    History.posm(:,it) = ym(4:5);
    History.pos(:,it) = O6.State((7:8);
    
    % Time update
    T = [T t];
    t = t+Ts;
    
end
disp('Simulation done!')

O6.plot(T)

%%
title = 'Linear Velocities';
names = ["$u_m$ ", "$v_m$", "sog","$u$ ", "$v$ "];
niceplot(T,toKnots([History.num(1:2,:); History.SOG; History.nu(1:2,:)],'m/s'), names, title, ["s","s","-","-","-"], ["time [s]", "[knot]"], 'northeast');

title = 'Angular Velocities';
names = ["$r$ ", "$r_m$"];
niceplot(T,[History.nu(3,:); History.num(3,:)], names, title, ["-","s"], ["time [s]", "[rad/s]"], 'southeast');
%ylim([-pi/8 pi/8])

title = 'Heading';
names = ["$\phi_m$ ", "$\phi$", "$\theta_C$"];
niceplot(T,rad2deg([History.phim; History.phi; History.course]), names, title, ["s","-","-"], ["time [s]", "[deg]"], 'southeast');
ytickformat('%.0f°')

title = 'Position';
names = ["$x_m$","$y_m$","$x$","$y$"];
niceplot(T,[History.posm; History.pos], names, title, ["s","-"], ["time [s]", "[m]"], 'southeast');
