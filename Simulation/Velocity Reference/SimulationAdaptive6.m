%% Clean
close all
clear all
clc

%% Init
Ts = 1/50;
N = 20000;
t = 0; % Start time
T = [];

%% Real Vessel
%state(1,1) = 1;
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

K = diag([0.4, 0.2, 0, 0, 0, 0.2]);

theta(1:2) = [0 0];
theta(3:8) = [0 0.0667 -0.9463 0 0.1143 0.3298];
theta(9:13)  = [0.0013 0.1699 0 -2.0239 -10.3735];
theta = theta';

Phis(1, 1:2) = [abs(u)*u u^3];
Phis(2, 3:8) = [v r abs(v)*v abs(r)*v abs(v)*r abs(r)*r];
Phis(6, 9:13) = [v abs(v)*v abs(r)*v abs(v)*r abs(r)*r];

thetahat = ones(14,1)*0;
%thetahat = theta;
%thetahat = thetahat + randn(13,1)*10;
Gamma = eye(14)*8;
iz = 0;
%%
rnu = [1 0 0]';
drnu = [0 0 0]';
rx = 1:N;
%rnu = [sin(1/2000 * rx)*0.3+1; zeros(5,N)];

rnu = [0.5-0.5./(1+exp(0.005*(rx-N/2))); zeros(4,N); 0.1./(1+exp(0.005*(rx-N/2)))];

rnu = repmat([1;0;0;0;0;0],1,N);
%rnu = repmat([0;0.2;0.01],1,N);
%rnu = [0*rx; sin(1/500 * rx)*0.4; 0*rx];
%rnu = [repmat([1.2;0.2;-0.1],1,N/2) repmat([1.0;-0.2;0.1],1,N/2)]; 

%% Main Loop
disp('Running Simulation...')
for it = 1:N
    nu = O6.State([1:6]);
    eta = O6.State([7:12]);
       u = nu(1);
    v = nu(2);
    r = nu(6);
    [M,C,DL,DNL,G] = sysmtrx(nu);
    
    %%
    Phi(1, 1:2) = [abs(u)*u u];
    Phi(2, 3:8) = [v r abs(v)*v abs(r)*v abs(v)*r abs(r)*r];
    Phi(6, 9:14) = [v r abs(v)*v abs(r)*v abs(v)*r abs(r)*r];
    
     Phi(1, 1:2) = [abs(u)*u u];
    Phi(2, 3:8) = [v r 0 0 0 0];
    Phi(6, 9:14) = [v r 0 0 0 0];
    z = nu - rnu(:,it);
  
    if (it == 1 || it == N),  drnu = zeros(6,1); else, drnu = (rnu(:,it+1) - rnu(:,it-1))/Ts; end
        
    
    f = M\(- C*nu - DL*nu);    
    tau = M*(drnu -K*z - f) - Phi*thetahat;
    
%     f = M\(- C*nu;    
%     tau = M*(drnu -K*z - f) - Phi*thetahat;
    
    dthetahat = -Gamma * Phi' *inv(M)' * z;
    
    thetahat = thetahat + Ts*dthetahat;
    %thetahat = theta;
    %thetahat(thetahat<0) = 0;
    % Input
    tau([3 4 5]) = 0;
    %Ta = O6.controlAllocation(tau,nu);
    O6.Thrust = tau;
    O6.step(Ts);
    

    
    % Save
    History.z(:,it) = z;
    History.tau_r(:,it) = tau([1 2 6]);
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
%save('Simulation/Data/thetahat.mat','thetahat')

%% Plotting
close all

O6.plot(T)

%O6.plotPropeller(T)
title = 'error';
names = ["$z_u$","$z_v$","$z_r$"];
niceplot(T,History.z([1 2 6],:), names, title, ["-"], ["time [s]", "$[\frac{m}{s}]$"], 'southeast');

title = 'Thrust requested vs. applied';
names = ["$\tau_u $","$\tau_v$","$\tau_r$","$\tau_u $","$\tau_v$","$\tau_r$"];
niceplot(T, [History.tau_a; History.tau_r], names, title, ["-","--"], ["time [s]", "[N]"], 'southwest');

disp('Press any key to show adaptation'), pause
close all
maxtheta = max(History.thetahat,[],'all');
title = 'thetahat';
names = "$"+["X_{|u|u}" "X_{uuu}"]+"$";
niceplot(T,History.thetahat(1:2,:), names, title, ["--"], ["time [s]", "[-]"], 'west');
%ylim([0 maxtheta])
title = 'thetahat';
names = "$"+["Y_v" "Y_r" "Y_{|v|v}" "Y_{|r|v}" "Y_{|v|r}" "Y_{|r|r}"]+"$";
niceplot(T,History.thetahat(3:8,:), names, title, ["--"], ["time [s]", "[-]"], 'center');
%ylim([0 maxtheta])
title = 'thetahat';
names = "$"+["N_v" "N_r" "N_{|v|v}" "N_{|r|v}" "N_{|v|r}" "N_{|r|r}"]+"$";
niceplot(T,History.thetahat(9:14,:), names, title, ["--"], ["time [s]", "[-]"], 'east');
%ylim([0 maxtheta])

