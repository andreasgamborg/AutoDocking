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
state(12,1) = 0;
O6 = Otter(state);
%O6.UseProppeller = false;

clear state
%% Model

Model = 'Models\Primitive\otter6mtrx.mat'
load(Model,'MRB');
load(Model,'MA');

% Known Constant parameters
M = MRB + MA;
M = M([1 2 6],[1 2 6]);

Xu = 77.5544;
Nr = 49.3007;
d = diag([Xu 0 Nr])';

clear MA MRB Xu Nr Model

%% Control gains
K = diag([0.4, 0.2, 0.2])*2;

%% Adaptation Init
Na = 13;
thetahat = ones(Na,1)*0;
Gamma = eye(Na)*2;


%% Course
%load('Course\square.mat','P')
load('Course\sinecurve.mat','P')
load('Course\circle.mat','P')

closestPoint = 1;
nP = length(P);
lookaheaddist = 4;
%% Main Loop
disp('Running Simulation...')
for it = 1:N
    % State
        nu = O6.State([1 2 6]);
        eta = O6.State([7 8 12]);
        u = nu(1);
        v = nu(2);
        r = nu(3);

    % Guidance
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

        psi = -eta(3);
        R = [  cos(psi)   -sin(psi)
            sin(psi)    cos(psi)   ];

        deltaL = R * Pdiff(:,target);

        alpha = atan2(deltaL(2),deltaL(1));
        alpha = wrapToPi(alpha);

        rnu = [1; 0; 0.4*alpha];
    % Error
        z = nu - rnu;
        if (it == 1 || it == N),  drnu = zeros(3,1); else, drnu = (rnu - lrnu)/Ts; end
        lrnu = rnu;
    % Adaptation
        Phi(1, 1:2) = [abs(u)*u u^3];
        Phi(2, 3:8) = [v r abs(v)*v abs(r)*v abs(v)*r abs(r)*r];
        Phi(3, 9:13) = [v abs(v)*v abs(r)*v abs(v)*r abs(r)*r];
    
        dthetahat = -Gamma * Phi' *inv(M)' * z; 
        thetahat = thetahat + Ts*dthetahat;
        
        
    % Control
        C = [   - 11*r^2 - 137.5000*v*r
            60.5000*r*u
            11*r*u];
        
        f = M\(-d*nu - C);
        tau = M*(drnu -K*z - f) - Phi*thetahat;
        
   
    % Input
        Tr([1 2 6],1) = tau;
        Ta = O6.controlAllocation(Tr,nu);
        %O6.Thrust = Tr;
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

clear u v r

%% Plotting
close all

O6.plot(T,P)

%O6.plotPropeller(T)
title = 'error';
names = ["$z_u$","$z_v$","$z_r$"];
niceplot(T,History.z, names, title, ["-"], ["time [s]", "$[\frac{m}{s}]$"], 'southeast');

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
names = "$"+["N_v" "N_{|v|v}" "N_{|r|v}" "N_{|v|r}" "N_{|r|r}"]+"$";
niceplot(T,History.thetahat(9:13,:), names, title, ["--"], ["time [s]", "[-]"], 'east');
%ylim([0 maxtheta])

