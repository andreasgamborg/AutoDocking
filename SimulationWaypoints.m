%% Clean
close all
clear all
clc

%% Init
Ts = 1/50;
N = 12000;
t = 0; % Start time
T = [];

%% Real Vessel
state(12,1) = 0;
O6 = Otter6;
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
Leta = eye(3);
%% Waypoints
WP = [[20 10 0]' [40 -10 0]' [60 0 0]'];
%WP = [[10 0 -pi/4]' [10 10 -3*pi/4]' [0 10 3*pi/4]' [0 0 pi/4]'];
%WP = [[10 0 pi/2]' [10 10 0]' [20 10 0]'];
%WP = [[20 0 -pi/2]'];

%% Init State
nuhat = [0 0 0]';
etahat = [0 0 0]';

iWP = 1;
acceptDistance = 0.05;         %[m] Waypoint reached
cruiseSpeed = 2;              %[knot]
precisionDistance = 3;          %[m]
precisionSpeed = 0.5;              %[knot]

    k1 = -70;
    k2 = 100;
    k3 = -10;
%% Main Loop
disp('Running Simulation...')
for it = 1:N
    Mea = O6.getMeasurement();
    ym = Mea.ym;
    num = ym(1:3);
    etam = ym(4:6);
    
    deta = WP(1:2,iWP)-etahat(1:2);
    if norm(deta) < acceptDistance
        iWP = iWP + 1;
        if iWP > size(WP,2), break; end
        deta = WP(1:2,iWP)-etahat(1:2);
    end
    
    wpb = atan2(deta(2),deta(1)); wpb = wrapToPi(wpb);
    alpha = etahat(3)-wpb;     alpha = wrapToPi(alpha);
    beta = wpb-WP(3,iWP);     beta = wrapToPi(beta);
    theta = etahat(3)-WP(3,iWP);     theta = wrapToPi(theta);


    if norm(deta) < precisionDistance
        r = [precisionSpeed*65*cos(-alpha); 0.1*precisionSpeed*65*sin(-alpha); k3*theta];
    else
        r = [cruiseSpeed*65; 0; k1*alpha+k2*beta];
    end
    
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
    
    % Save
    History.ang(:,it) = [alpha;beta;theta]; 
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
O6.plot(T)
title = 'Control angles';
names = ["$\alpha$ ", "$\beta$", "$\theta$"];
niceplot(T,rad2deg([History.ang]), names, title,...
    [".","-","-"], ["time [s]", "[deg]"], 'southeast');
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


