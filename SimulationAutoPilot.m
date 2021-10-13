%% Clean
close all
clear all
clc

%% Init

path = 'Rute luftfoto.png';
pixelsPerMeter = 3;
background = imread(path);
[N,M,~] = size(background);

close all
fig = figure('Name','Track','DefaultAxesFontSize',20,'OuterPosition', [100           100        1720        880]);
imagesc([-M/pixelsPerMeter M/pixelsPerMeter], [-N/pixelsPerMeter N/pixelsPerMeter], background); hold on
axis equal

n= 4;
[x,y,button] = ginput(1)

P = ginput(n);
WP = [P' ; zeros(1,n)];

%% Iit
Ts = 1/50;
N = 12000;
t = 0; % Start time
T = [];

%% Real Vessel
state([7 8],1) = WP(1:2,1);
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
Leta = eye(3);

%% Init State
nuhat = [0 0 0]';
etahat = [-150 0 0]';

iWP = 2;
acceptDistance = 0.1;           %[m]    Waypoint reached

cruiseSpeed = 5;              %[knot]
precisionDistance = 2;          %[m]
precisionSpeed = 0.5;              %[knot]

k1 = -1000;
k2 = 800;
k3 = -1000;
%% Main Loop
disp('Running Simulation...')
for it = 1:N
    Mea = O6.getMeasurement();
    ym = O6.measurementTransformation(Mea);
    num = ym(1:3);
    etam = ym(4:6);
    
    
    deta = WP(1:2,iWP)-etahat(1:2);
    dpsi = WP(3,iWP)-etahat(3);
    if (norm(deta) < acceptDistance)
        iWP = iWP + 1;
        if iWP > size(WP,2), break; disp('Broke'); end
        deta = WP(1:2,iWP)-etahat(1:2);
    end
    
    wpb = atan2(deta(2),deta(1)); wpb = wrapToPi(wpb);
    alpha = etahat(3)-wpb;     alpha = wrapToPi(alpha);
    beta = wpb-WP(3,iWP);     beta = wrapToPi(beta);
    theta = etahat(3)-WP(3,iWP);     theta = wrapToPi(theta);
    
    
    if norm(deta) < precisionDistance
        r = [precisionSpeed*65*cos(-alpha); precisionSpeed*65*sin(-alpha); k3*theta];
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
    %History.ang(:,it) = [alpha;beta;theta];
    %     History.course(:,it) = Mea.Course;
    %     History.SOG(:,it) = Mea.SOG;
    %
    %     History.num(:,it) = num;
    %     History.nuhat(:,it) = nuhat;
    %     History.nu(:,it) = O6.State([1 2 6]);
    %
    %     History.etam(:,it) = etam;
    %     History.etahat(:,it) = etahat;
    %     History.eta(:,it) = O6.State([7 8 12]);
    %
    % Time update
    T = [T t];
    t = t+Ts;
    
end
disp('Simulation done!')

plot(O6.History.Pos(1,:),O6.History.Pos(2,:),'Linewidth',3);

