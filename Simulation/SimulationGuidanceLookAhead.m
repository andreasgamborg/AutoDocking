%% Clean
close all
clear all
clc

%% Init
Ts = 1/50;
N = 30000;
t = 0; % Start time
T = [];

%% Real Vessel
state(12,1) = 0;
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
Q = diag([1000 1000 1000]);     % Process noise
R = diag([0.1 0.1 0.1]);          % Measurement noise
[L,P,E] = lqe(A,B,Cm,Q,R);

%% Observer - Position
Leta = diag([2 2 8]);
Ld = diag([2 2 8]);

%% Course
load('sinecurve.mat')
closestPoint = 1;
nP = length(P);
lookaheaddist = 8;
%% Init Observer State
nuhat = [0 0 0]';
etahat = [0 0 0]';
dhat = [0 0 0]';

%% Init Guidance System
target = 1;
%% Main Loop
disp('Running Simulation...')
for it = 1:N
    Mea = O6.getMeasurement();
    ym = O6.measurementTransformation(Mea);
    num = ym(1:3);
    etam = ym(4:6);
    
    Pdiff = P-etahat(1:2);

    dist = sqrt(sum(Pdiff.^2));
    closestPoint = find(dist==min(dist));
    
    for p = closestPoint+1:nP
       if dist(p) > lookaheaddist
           target = p-1;
           break;
       end
    end
    
    psi = -etahat(3);
    R = [  cos(psi)   -sin(psi)
        sin(psi)    cos(psi)   ];
    deltaL = R * Pdiff(:,target);
    alpha = atan2(deltaL(2),deltaL(1));
    alpha = wrapToPi(alpha);
    
    r = [65; 0; 100*alpha];

    tau = -K*nuhat + r;
    % Input
    Tr([1 2 6],1) = tau;
    Ta = O6.controlAllocation(Tr,nuhat);
    O6.step(Ts);
    
    
    % Observer
    dnuhat = A*nuhat + B*Ta([1 2 6]) + dhat + L*(num - Cm*nuhat);
    ddhat = Ld*(num - Cm*nuhat);
    nuhat = nuhat + Ts*dnuhat;    
    dhat = ddhat + Ts*ddhat;

    
    etae = etam - etahat;
    etae(3) = wrapToPi(etae(3));
    detahat = Rot(etahat(3))*nuhat + Leta*etae;
    etahat = etahat + Ts*detahat;
    etahat(3) = wrapToPi(etahat(3));
    
    
    % Save
    History.ang(:,it) = [alpha];
    History.course(:,it) = Mea.Course;
    History.SOG(:,it) = Mea.SOG;
    
    History.num(:,it) = num;
    History.nuhat(:,it) = nuhat;
    History.nu(:,it) = O6.State([1 2 6]);
    
    History.etam(:,it) = etam;
    History.etahat(:,it) = etahat;
    History.eta(:,it) = O6.State([7 8 12]);
    
    % Time update
    T = [T t];
    t = t+Ts;
    
end
disp('Simulation done!')

%% Plotting
close all
O6.plot(T)
title = 'Control angles';
names = ["$\alpha$ "];
niceplot(T,rad2deg([History.ang]), names, title, ["-"], ["time [s]", "[deg]"], 'southeast');
ytickformat('%.0f°')

disp('Press any key to show estimates'), pause
close all
% u
title = 'Linear Velocities';
names = ["$u_m$ ", "$\hat{u}$", "$u$"];
niceplot(T,toKnots([History.num(1,:); History.nuhat(1,:); History.nu(1,:)],'m/s'), names, title, [".","-","-"], ["time [s]", "[knot]"], 'northwest');

title = 'Position';
names = ["$x_m$", "$\hat{x}$", "$x$"];
niceplot(T,[History.etam(1,:); History.etahat(1,:); History.eta(1,:)], names, title, [".","-","-"], ["time [s]", "[m]"], 'southwest');
% v
title = 'Linear Velocities';
names = ["$v_m$", "$\hat{v}$", "$v$"];
niceplot(T,toKnots([History.num(2,:); History.nuhat(2,:); History.nu(2,:)],'m/s'), names, title, [".","-","-"], ["time [s]", "[knot]"], 'north');

title = 'Position';
names = ["$y_m$", "$\hat{y}$", "$y$"];
niceplot(T,[History.etam(2,:); History.etahat(2,:); History.eta(2,:)], names, title, [".","-","-"], ["time [s]", "[m]"], 'south');
% r
title = 'Angular Velocities';
names = ["$r_m$", "$\hat{r}$", "$r$"];
niceplot(T,[History.num(3,:); History.nuhat(3,:); History.nu(3,:)], names, title, [".","-","-"], ["time [s]", "[rad/s]"], 'northeast');

title = 'Heading';
names = ["$\phi_m$ ", "$\hat{\phi}$", "$\phi$"];
niceplot(T,rad2deg([History.etam(3,:); History.etahat(3,:); History.eta(3,:)]), names, title,...
    [".","-","-"], ["time [s]", "[deg]"], 'southeast');
ytickformat('%.0f°')


