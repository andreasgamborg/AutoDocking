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
load('Models/Primitive/otter3mtrx_lin.mat');
% load('Models\Primitive\A3.mat')
load('Models\Primitive\A3_2nd_order_cf_fitting.mat')
%% Controller
Q1 = diag([1 1000 10000]);    % Stateerror cost
R1 = diag([0.01 1 1]);           % Input cost

%% Observer
Cm = eye(3);                    % Measurement matrix
Rvv = eye(3)*1;
Rww = diag([0.01 0.05 0.05]);
P = eye(3);
%% Reference
ref = [100 0 0]';
ref = [[3 0 0]' [1 0 110]' [0 -1 0]'];
ref = [[2 0 0]' [1 0 -110]' [2 0 0]'];
%ref = [[1 0 100]'];
rt = 1;


%% Init State
nuhat = [0 0 0]';
nustep = nuhat;

u = nustep(1);
v = nustep(2);
r = nustep(3);
A = eval(A3);
[F,G] = c2d(A,B,Ts);
[K] = dlqr(F,G,Q1,R1);
%% Reference Scaling
Ak = A-B*K;
Cref = -C*inv(Ak)*B;
sref = inv(Cref)*ref;
%% Main Loop
disp('Running Simulation...')
for it = 1:N
    Mea = O6.getMeasurement();
    ym = O6.measurementTransformation(Mea);
    num = ym(1:3);
    
    if (mod(it,100) == 0)
        disp('refitting model');
        nustep = nuhat;
        u = nustep(1);
        v = nustep(2);
        r = nustep(3);
        A = eval(A3);
        [F,G] = c2d(A,B,Ts);
        [K,P,E] = dlqr(F,G,Q1,R1);
        
        Fk = F-G*K;
        Cref = C*inv(eye(3)-Fk)*G;
        sref = inv(Cref)*ref;
    end
    
    if(it==N/4), rt = rt+1; end
    if(it==3*N/4), rt = rt+1; end
    tau = -K*nuhat + (sref(:,rt));

    % Input
    Tr([1 2 6],1) = tau ;
    Ta = O6.controlAllocation(Tr,nuhat);
    Ta = Ta([1 2 6]) ;
    O6.step(Ts);
    
    % Observer
    S       = Cm*P*Cm' + Rvv;
    kappa   = P*Cm'/S;
    epsilon = num - Cm*nuhat;
    nuhat   = nuhat + kappa*epsilon;
    P       = P - kappa*S * kappa';
    
    nuhat   = F*nuhat + G*tau;
    P       = F*P*F' + Rww;

    
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
