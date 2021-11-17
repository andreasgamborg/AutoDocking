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
%O6.UseProppeller = false;
%% Model
syms u v w r p q

Model = 'Models\Primitive\otter6mtrx.mat'
load(Model);

%%
K = diag([0.8, 0.2, 0, 0, 0, 0.2]);

%%
rnu = [1 0 0]';
drnu = [0 0 0]';
rx = 1:N;
%rnu = [cos(1/500 * rx)*0.5 + 0.5; sin(1/1500 * rx)*0.1; 0.1./(1+exp(0.005*(rx-N/2)))];
rnu = [sin(1/2000 * rx)*0.3+1; zeros(5,N)];

%rnu = [0.5-0.5./(1+exp(0.005*(rx-N/2))); zeros(1,N); 0.1./(1+exp(0.005*(rx-N/2)))];

rnu = repmat([1;0;0;0;0;0],1,N);
%rnu = repmat([0;0.2;0.01],1,N);
%rnu = [0*rx; sin(1/500 * rx)*0.4; 0*rx];
%rnu = [repmat([1.2;0.2;-0.1],1,N/2) repmat([1.0;-0.2;0.1],1,N/2)]; 

%% Main Loop
disp('Running Simulation...')
for it = 1:N
    nu = O6.State([1:6]);
    eta = O6.State([7:12]);

    [M,C,D,G] = sysmtrx(nu);
    
    %%

    z = nu - rnu(:,it);
  
    if (it == 1 || it == N),  drnu = zeros(6,1); else, drnu = (rnu(:,it+1) - rnu(:,it-1))/Ts; end
        
    tau = M*(-K*z + drnu) + C*nu + D*nu; %Feedback linearization
    
    % Input
    tau([3 4 5]) = 0;
    Ta = O6.controlAllocation(tau,nu);
    %O6.Thrust = tau;
    O6.step(Ts);
    

    
    % Save
    History.z(:,it) = z;
    History.tau_r(:,it) = tau([1 2 6]);
    History.tau_a(:,it) = O6.Thrust([1 2 6]);
    
    History.nu(:,it) = O6.State([1 2 6]);
    
    History.phi(:,it) = O6.State(12);
    History.pos(:,it) = O6.State(7:8);
    
    % Time update
    T = [T t];
    t = t+Ts;
    
end
disp('Simulation done!')
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


