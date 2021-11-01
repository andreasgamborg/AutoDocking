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

%% Waypoints
% WP = {WayPoint([20 0]',pi/2) WayPoint([20 20]' , pi) WayPoint([0 20]', -pi/2) WayPoint([0 0]', 0)};
%WP = {WayPoint([-10 0]',pi)};

   k = 30;
   ad = 0.2;
   aa = pi/36;
WP = {  WayPoint([k 0]', -pi/4, ad, aa) WayPoint([k -k]', -3*pi/4, ad, aa)  ...
        WayPoint([0 -k]', 3*pi/4, ad, aa) WayPoint([0 k]', 3*pi/4, ad, aa)  ...
        WayPoint([-k k]', -3*pi/4, ad, aa) WayPoint([-k 0]', 0, ad, aa)     ...
        WayPoint([0 0]', 0, ad, aa)    };
   %WP = {  WayPoint([k 0]', -pi/4) WayPoint([k -k]', -pi/2)  };
   %WP = {  WayPoint([20 5]', 0) WayPoint([k 0]', -pi/2) WayPoint([k -k]', -pi/2)  };

% WP{2}.setAccept(0.05,0.0175);
% WP{2}.precisionMode = true;
iWP = 1;
%% Init Observer State
nuhat = [0 0 0]';
etahat = [0 0 0]';
dhat = [0 0 0]';


%% Init Guidance System
% Velocity
umax = 2;              %[knot]
umin = 0.1;              %[knot]
k = 0.8;

% Rotation
k1 = 100;
k2 = 20;
k3 = 100;
speed = 1;

%% Main Loop
disp('Running Simulation...')
for it = 1:N
    Mea = O6.getMeasurement();
    ym = O6.measurementTransformation(Mea);
    num = ym(1:3);
    etam = ym(4:6);
    
    deltaG = WP{iWP}.pos-etahat(1:2);
    if (norm(deltaG) < WP{iWP}.accept.distance) && (abs(WP{iWP}.heading - etahat(3)) < WP{iWP}.accept.angle)
        iWP = iWP + 1;
        if iWP > size(WP,2),disp('Final WayPoint reached'); break; end
        deltaG = WP{iWP}.pos - etahat(1:2);
    end
    
    psi = -etam(3);
    R = [  cos(psi)   -sin(psi)
        sin(psi)    cos(psi)   ];
    deltaL = R * deltaG(1:2);
    
    bearG = atan2(deltaG(2),deltaG(1));
    bearL = atan2(deltaL(2),deltaL(1));
    
    alpha = bearL;                  alpha = wrapToPi(alpha);
    beta = bearG-WP{iWP}.heading;         beta = wrapToPi(beta);
    theta = WP{iWP}.heading-etahat(3);           theta = wrapToPi(theta);
    
    d = norm(deltaG(1:2,:));
    %speed = umin+(umax-umin)* 1/(1+exp(-k*(d-5)));

    
    
%     if WP{iWP}.precisionMode && norm(deltaG(1:2,:)) < 4% WP{iWP}.accept.distance
%         r = [speed*65*cos(alpha); speed*65*sin(alpha)/10; k3*theta];
%     else
%         r = [speed*65; 0; k1*alpha + k2*beta];
%     end
    
       
    
    speed = 0.1+0.9*umax*umin/(umin+(umax-umin)*exp(-k*(d-3)));
    speed = umin + (umax-umin)./(1+exp(-k*(d-3)));
    
        d = d/speed;

    k2 = 0.5*exp(-((d-8)/4).^2);
    k3 = exp(-0.8*(d));
    k1 = 1-k2-k3;

    r = [speed*65; speed*65*k3*sin(alpha); 100*wrapToPi(k1*alpha + k2*beta + k3*theta)];
    %r = [speed*65*cos(alpha); 0; 100*(k1*alpha + k2*beta + k3*theta)];

    
    tau = -K*nuhat + r;
    % Input
    Tr([1 2 6],1) = tau;
    Ta = O6.controlAllocation(Tr,nuhat);
    O6.step(Ts);
    
    % Observer
    if it > N/2, dhat = dhat*0; end
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
    History.ang(:,it) = [alpha;beta;theta];
    History.gains(:,it)  = [k1;k2;k3;d];
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
O6.plot(T,WP)
title = 'Control angles';
names = ["$\alpha$ ", "$\beta$", "$\theta$"];
niceplot(T,rad2deg([History.ang]), names, title, ["-"], ["time [s]", "[deg]"], 'southeast');
ytickformat('%.0f°')

title = 'gain parameter';
names = ["$k_1$","$k_2$","$k_3$","$d$"];
niceplot(T,[History.gains], names, title, ["-"], ["time [s]", " "], 'southwest');

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


