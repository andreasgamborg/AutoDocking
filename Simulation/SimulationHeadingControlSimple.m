%% Clean
close all
clear all
clc

%% Init
Ts = 1/50;
N = 24000;
t = 0; % Start time
T = [];

%% Real Vessel
state([7 8 12],1) = [0 0 0];
O6 = Otter6(state);
O6.UseProppeller = true;

%% Model
Model = 'Models/Primitive/otter3mtrx_lin.mat'
load(Model);

%% Controller
Q = diag([1000 1000 10000]);    % Stateerror cost
R = diag([0.1 10 1]);           % Input cost
[K,P,E] = lqr(A,B,Q,R);

%% Observer - Velocities
Cm = eye(3);                    % Measurement matrix
Q = diag([10000 10000 10000]);     % Process noise
R = diag([0.1 0.1 0.1]);          % Measurement noise
[L,P,E] = lqe(A,B,Cm,Q,R);

%% Observer - Position
Leta = eye(3);

%% Waypoints
WP = [[20 0 pi/4]' [20 -20 3*pi/4]' [0 -20 -3*pi/4]' [0 0 -pi/4]'...
    [0 20 pi/4]' [-20 20 3*pi/4]' [-20 0 -3*pi/4]' [0 0 -pi/4]'];


%% Init State
nuhat = [0 0 0]';
etahat = [0 0 0]';

iWP = 1;
acceptDistance = 1;           %[m]    Waypoint reached
cruiseSpeed = 1;              %[knot]

k1 = 100;


%% Main Loop
disp('Running Simulation...')
for it = 1:N
    Mea = O6.getMeasurement();
    ym = O6.measurementTransformation(Mea);
    num = ym(1:3);
    etam = ym(4:6);
    
    eta = O6.State([7 8 12]);
    nu = O6.State([1 2 6]);
    
    deltaG = WP(:,iWP)-etahat;
    
    if (norm(deltaG(1:2,:)) < acceptDistance)
        iWP = iWP + 1;
        if iWP > size(WP,2), break; end
        deltaG = WP(:,iWP)-etahat;
    end
    psi = -eta(3);
    R = [  cos(psi)   -sin(psi)
        sin(psi)    cos(psi)   ];
    
    deltaL = R * deltaG(1:2);
    
    wpb = atan2(deltaL(2),deltaL(1));
    wpb = wrapToPi(wpb);
    
    r = [cruiseSpeed*65; 0; k1*wpb];
    tau = -K*nuhat + r;
    
    % Input
    Tr([1 2 6],1) = tau;
    Ta = O6.controlAllocation(Tr,nuhat);
    O6.step(Ts);
    
    % Observer
    dnuhat = A*nuhat + B*Ta([1 2 6]) + L*(num - Cm*nuhat);
    nuhat = nuhat + Ts*dnuhat;
    
    detahat = Rot(etahat(3))*nuhat + Leta*(etam - etahat);
    etahat = etahat + Ts*detahat;
    etahat(3) = wrapToPi(etahat(3));
    
    % Save
    History.ang(:,it) = [wpb];
    History.deltaL(:,it) = [deltaL];
    
    % Time update
    T = [T t];
    t = t+Ts;
    
end
disp('Simulation done!')

%% Plotting
close all
O6.plot(T)
title = 'Control angles';
names = ["$wpb$ "];
niceplot(T,rad2deg([History.ang]), names, title, ["-"], ["time [s]", "[deg]"], 'southeast');
ytickformat('%.0f°')
title = 'Control distances';
names = ["$\delta_L x$", "$\delta_L y$"];
niceplot(T,[History.deltaL], names, title, ["-"], ["time [s]", "[m]"], 'southeast');




