%% Clean
close all
clear all
clc

%% Init
Ts = 1/1000;
N = 50000;
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
%M = diag(diag(M));
Cs = Cs([1 2 6],[1 2 6]);

%%

K = diag([0.4, 0.2, 0.2]);

theta(1:2) = [0 0];
theta(3:8) = [0 0.0667 -0.9463 0 0.1143 0.3298];
theta(9:13)  = [0.0013 0.1699 0 -2.0239 -10.3735];
theta = theta';

Phis(1, 1:2) = [abs(u)*u u^3];
Phis(2, 3:8) = [v r abs(v)*v abs(r)*v abs(v)*r abs(r)*r];
Phis(3, 9:13) = [v abs(v)*v abs(r)*v abs(v)*r abs(r)*r];

Xu = 77.5544;
Nr = 49.3007;
d = diag([Xu 0 Nr])';


%% Adaption
Na = 12
thetahat = ones(Na,1)*0;

Gamma = eye(Na)*2;
%%
rnu = [1 0 0]';
drnu = [0 0 0]';
rx = 1:N;
%rnu = [cos(1/500 * rx)*0.5 + 0.5; sin(1/1500 * rx)*0.1; 0.1./(1+exp(0.005*(rx-N/2)))];
rnu = [sin(1/1000 * rx); zeros(2,N)];

%rnu = [0.5-0.5./(1+exp(0.005*(rx-N/2))); zeros(1,N); 0.1./(1+exp(0.005*(rx-N/2)))];

rnu = repmat([1;0;0],1,N);
%rnu = repmat([0;0.2;0.01],1,N);
%rnu = [0*rx; sin(1/500 * rx)*0.4; 0*rx];
%rnu = [repmat([1.2;0.2;-0.1],1,N/2) repmat([1.0;-0.2;0.1],1,N/2)]; 

%% Main Loop
disp('Running Simulation...')
for it = 1:N
    nu = O6.State([1 2 6]);
    eta = O6.State([7 8 12]);
    u = nu(1);
    v = nu(2);
    r = nu(3);
    psi = eta(3);
    R = [  cos(psi)   -sin(psi) 0
        sin(psi)    cos(psi)   0
        0 0 1];
    
    %%
%     Phi(1, 1:2) = [abs(u)*u u^3];
%     Phi(2, 3:8) = [v r abs(v)*v abs(r)*v abs(v)*r abs(r)*r];
%     Phi(3, 9:13) = [v abs(v)*v abs(r)*v abs(v)*r abs(r)*r];

    Phi(2, 1:6) = [abs(v) abs(r) abs(v)*v abs(r)*v abs(v)*r abs(r)*r];
    Phi(3, 7:12) = [abs(v) abs(r) abs(v)*v abs(r)*v abs(v)*r abs(r)*r];
    
    z = nu - rnu(:,it);

        
     C = [   - 11*r^2 - 137.5000*v*r
                 60.5000*r*u
                 11*r*u];
    
    if (it == 1 || it == N),  drnu = zeros(3,1); else, drnu = (rnu(:,it+1) - rnu(:,it-1))/Ts; end
        
    f = M\(d*nu - C);
    tau = M*(drnu -K*z - f) - Phi*thetahat;

    
    dthetahat = -Gamma * Phi' *inv(M)' * z;
    
    thetahat = thetahat + Ts*dthetahat;
    %thetahat = theta;
    %thetahat(thetahat<0) = 0;
    % Input
    Tr([1 2 6],1) = tau;
    Ta = O6.controlAllocation(Tr,nu);
    O6.Thrust = Tr;
    O6.step(Ts);
    

    
    % Save
    History.z(:,it) = z;
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
%%
save('Simulation/Data/thetahat.mat','thetahat')

%% Plotting
close all

O6.plot(T)

%O6.plotPropeller(T)
title = 'error';
names = ["$z_u$","$z_v$","$z_r$"];
niceplot(T,History.z, names, title, ["-"], ["time [s]", "$[\frac{m}{s}]$"], 'southeast');

title = 'Thrust requested vs. applied';
names = ["$\tau_u $","$\tau_v$","$\tau_r$","$\tau_u $","$\tau_v$","$\tau_r$"];
niceplot(T, [History.tau_a; History.tau_r], names, title, ["-","--"], ["time [s]", "[N]"], 'southwest');

disp('Press any key to show adaptation'), pause
close all
title = 'thetahat';
names = "$\hat{\theta}_"+[1:Na]+"$";
niceplot(T,History.thetahat, names, title, ["--"], ["time [s]", "[-]"], 'west');


