%% Clean
close all
clear all
clc

%% Init
Ts = 1/50;
N = 15000;
t = 0; % Start time
T = [];

%% Real Vessel
state(12,1) = 0;
O6 = Otter6(state);
O6.UseProppeller = false;
%% Model
syms u v w r p q

Model = 'Models\Primitive\otter6mtrx.mat'
load(Model);

M = MRB + MA;
Cs = CRB + CA;

Cs = subs(Cs, [w p q], [0 0 0]);

M = M([1 2 6],[1 2 6]);
Cs = Cs([1 2 6],[1 2 6]);

%%

K1 = diag([0.4, 0.4, 0.1]);
K2 = diag([5, 8, 6]);

Phis(1, 1:2) = [u^3 abs(u)*u];
Phis(2, 3:7) = [abs(v)*v abs(r)*v abs(r) abs(v)*r abs(r)*r];
Phis(3, 8:12) = [abs(v)*v abs(r)*v abs(r) abs(v)*r abs(r)*r];

Xu = 77.5544 ;
Yv = 40;
Nr = 45.2650;
gs = [Xu*u Yv*v Nr*r]';

load('Simulation\Data\thetahat.mat')
thetahat = zeros(12,1);
Gamma = diag([8, 8, 8, 8, 4, 8, 8, 8, 8, 4, 8, 8]);
%%
% 
% K1 = diag([1, 0.4, 0.1]);
% K2 = diag([5, 8, 6]);
% 
% 
% etat = [1 1 1]';
% 
% Phis(1, 1)      = [abs(u)*u];
% Phis(2, 2:3)    = [abs(v)*v abs(r)*r ];
% Phis(3, 4:5)   = [abs(v)*v abs(r)*r ];
% 
% Xu = 77.5544 ;
% Yv = 40;
% Nr =  45.2650;
%     gs = [Xu*u Yv*v Nr*r]';
% 
% load('Simulation\Data\thetahat.mat')
% thetahat = zeros(5,1);
% Gamma = diag(ones(1,5));


%% Course
load('sinecurve.mat')
%load('square.mat')

closestPoint = 1;
nP = length(P);
lookaheaddist = 8;
%% Main Loop
disp('Running Simulation...')
for it = 1:N
    nu = O6.State([1 2 6]);
    eta = O6.State([7 8 12]);
    u = nu(1);
    v= nu(2);
    r = nu(3);
    psi = eta(3);
    R = [  cos(psi)   -sin(psi) 0
        sin(psi)    cos(psi)   0
        0 0 1];
    
    
    
    %%
    Pdiff = P-eta(1:2);

    dist = sqrt(sum(Pdiff.^2));
    closestPoint = find(dist==min(dist));
    if(closestPoint+1 == nP), disp('Final point reached'); break;    end
    
    for p = closestPoint+1:nP
       if dist(p) > lookaheaddist
           target = p-1;
           break;
       end
    end
    
    deltaL = R(1:2,1:2)' * Pdiff(:,target);
    alpha = atan2(deltaL(2),deltaL(1));
    alpha = wrapToPi(alpha);
    
    etat = [P(:,target); alpha];

    %%
    
    C = eval(Cs);
    Phi = eval(Phis);
    g = eval(gs);
    
    z1 = R'*(eta-etat);
    alpha = K1 * z1;    % Missing term for detat
    z2 = nu - alpha;
    
    S(1,2) = -r;
    S(2,1) = r;
    S(3,3) = 0;
    
    dz1 = S'*z1 -K1*z1 + z2;
    dz2 = -M\(z1 + K2*z2);
    
    
    dalpha = K1 * dz1;
    
    tau = M*dalpha + C*nu - g - Phi*thetahat - z1 - K2*z2;

        dthetahat = Gamma*Phi'*z2;
    thetahat = thetahat + Ts*dthetahat;
    
    % Input
    Tr([1 2 6],1) = tau;
    %Ta = O6.controlAllocation(Tr,nu);
    O6.Thrust = Tr;
    O6.step(Ts);
    

    
    % Save
    History.tau_r(:,it) = tau;
    History.tau_a(:,it) = O6.Thrust([1 2 6]);
    
    History.nu(:,it) = O6.State([1 2 6]);
    
    History.phi(:,it) = O6.State(12);
    History.pos(:,it) = O6.State(7:8);
    History.thetahat(:,it) = thetahat;
    
    % Time update
    T = [T t];
    t = t+Ts;
    
end
disp('Simulation done!')

%% Plotting
close all

O6.plot(T,P)

%O6.plotPropeller(T)

title = 'thetahat';
names = "$\hat{\theta}_{"+[1:12]+"}$";
niceplot(T,History.thetahat, names, title, ["--"], ["time [s]", "[m]"], 'south');

save('Simulation/Data/thetahat.mat','thetahat')
