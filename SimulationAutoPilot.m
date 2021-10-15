%% Clean
close all
clear all
clc

%% Init

path = 'Rute luftfoto.png';
path = 'Rute søkort.png';
pixelsPerMeter = 3;
background = imread(path);
[N,M,~] = size(background);

close all
fig = figure('Name','Track','DefaultAxesFontSize',20,'OuterPosition', [100           100        1720        880]);
imagesc([-M/pixelsPerMeter M/pixelsPerMeter], [-N/pixelsPerMeter N/pixelsPerMeter], background); hold on
axis equal

%% Iit
Ts = 1/50;
N = 12000;
t = 0; % Start time
T = [];

%% Real Vessel
% state([7 8],1) = WP(1:2,1);
% state(12,1) = 0;
% O6 = Otter6(state);
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

%% Init Observer State
nuhat = [0 0 0]';
etahat = [0 0 0]';
%% Init Guidance System
% Velocity
umax = 3;              %[knot]
umin = 0.5;              %[knot]
k = 0.15;

% Rotation
k1 = 100;
k2 = 20;
k3 = -100;
%% Main Loop
lastindex = 1;
for jt = 1:4
    P = ginput(1);
    WP = WayPoint(P',nan);
    
    disp('Running Simulation...')
    for it = 1:N
        Mea = O6.getMeasurement();
        ym = O6.measurementTransformation(Mea);
        num = ym(1:3);
        etam = ym(4:6);
        
        deltaG = WP.pos-etahat(1:2);
        if (norm(deltaG) < WP.accept.distance)
            if(isnan(WP.heading))
                disp('WayPoint reached');
                break;
            elseif(abs(WP.heading - etahat(3)) < WP.accept.angle)
                disp('WayPoint reached');
                break;
            end
        end
        
        psi = -etam(3);
        R = [  cos(psi)   -sin(psi)
            sin(psi)    cos(psi)   ];
        deltaL = R * deltaG(1:2);
        
        bearG = atan2(deltaG(2),deltaG(1));
        bearL = atan2(deltaL(2),deltaL(1));
        
        alpha = bearL;                  alpha = wrapToPi(alpha);
        if(isnan(WP.heading))
            beta = 0;
            theta = 0;
        else
            beta = bearG-WP.heading;        beta = wrapToPi(beta);
            theta = etahat(3)-WP.heading;   theta = wrapToPi(theta);
        end
        
        d = norm(deltaG(1:2,:));
        speed = umax*umin/(umin+(umax-umin)*exp(-k*d));
        
        if WP.precisionMode && norm(deltaG(1:2,:)) < WP.accept.distance
            r = [speed*65*cos(alpha); speed*65*sin(alpha); k3*theta];
        else
            r = [speed*65; 0; k1*alpha + k2*beta];
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
    index = O6.t;
    period = lastindex : index;
    plot(O6.History.Pos(1,period),O6.History.Pos(2,period),'Linewidth',3);
    lastindex = index;
end
