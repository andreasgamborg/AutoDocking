%% Clean
    close all
    clear all
    clc

%% Init
    Ts = 1/100;
    N = 20000;
    t = 0; % Start time
    T = [];

%% Real Vessel
    state(12,1) = 0;
    O6 = Otter(state);
    O6.UseProppeller = true;
    O6.setHarbour([6;5], 0, 4, 3)

    O6.setCurrent(0.1,-pi/2)
    clear state
%% Model

    Model = 'Models\Primitive\otter6mtrx.mat'
    load(Model,'MRB');
    load(Model,'MA');

    % Known Constant parameters
    M = MRB + MA;
    M = M([1 2 6],[1 2 6]);

    Xu = -77.5544;
    Nr = -49.3007;
    d = diag([Xu 0 Nr])';

    clear MA MRB Xu Nr Model

%% Control gains
    K1 = diag([2, 2, 1])*0.8;
    K2 = diag([1.3, 1, 1.1])*1.05;
    ftau = zeros(3,1);
    
%% Adaptation Init
    Na = 15;
    thetahat = zeros(Na,1);
    Gamma1 = eye(13)*0.1;
    %Gamma1 = diag([5 5 100 50000 100 100 100 100 50000 100 100 100 100]);

    Gamma2 = eye(2)*0.06;

    Gamma = [   Gamma1 zeros(13,2)
                zeros(2,13) Gamma2  ];
            
    Phi1 = zeros(3,Na);
    Phi1([1 2],[14 15]) = eye(2);
    
    iM = inv(M);

%% Reference
    reta1=[8 -2 -pi/4]';
    reta2=[20 -6 -pi/2]';
    
    reta1 = [1 0 0]';
    reta2 = [2 1 0]';
    
    reta1 = [4 5 pi/6]';
    reta2 = [8 5 0]';
    
    reta = reta1; 
    

    % Reference filter
    % Same filter is used for x, y and psi (position and heading)
    Ref.zeta = 1;
    Ref.wn = 0.1;
    Ref.K1 = diag([1 1 1])*Ref.wn^2;
    Ref.K2 = diag([1 1 1])*2*Ref.zeta*Ref.wn;

    Ref.r= [ 0 0 0 ]';
    Ref.dr= [ 0 0 0 ]';
    Ref.ddr= [ 0 0 0 ]';
%% Main Loop
disp('Running Simulation...'), tic;
for it = 1:N
    % State
        nu = O6.State([1 2 6]);
        if isnan(nu),            disp('nu is NAN');            break;        end
        eta = O6.State([7 8 12]);

        u = nu(1);
        v = nu(2);
        r = nu(3);
        psi = eta(3);
        
    % Rotation
        R = [   cos(psi)   -sin(psi)    0
                sin(psi)    cos(psi)    0
                0                  0    1   ];
        S = [   0   -r    0
                r    0    0
                0    0    0   ];
    
    % Regressor        
        Phi2(1, 1:2) = [abs(u)*u u^3];
        Phi2(2, 3:8) = [v r abs(v)*v abs(r)*v abs(v)*r abs(r)*r];
        Phi2(3, 9:15) = [v abs(v)*v abs(r)*v abs(v)*r abs(r)*r 0 0];
        
    % Error
        z1 = R'*(eta - Ref.r);
        z1(3) = wrapToPi(z1(3));
        alpha = -K1*z1 - R'*(Phi1*thetahat-Ref.dr);
        z2 = nu - alpha;
    
    % Adaptation
        dthetahat = Gamma * (Phi1'*R*(z1 + K1'*z2) + Phi2'*iM*z2);
        thetahat = thetahat + Ts*dthetahat;
        

%         currentBound = 1;
%         thetahat(14) = max(-currentBound, min(currentBound,thetahat(14)));
%         thetahat(15) = max(-currentBound, min(currentBound,thetahat(15)));

        
    % Control
        C = [   - 11*r^2 - 137.5000*v*r
            60.5000*r*u
            11*r*u];
        
        f = M\(d*nu - C);
        falpha = -K1*( S'*z1 + z2 - K1*z1 ) - R'*(S'*(Phi1*thetahat-Ref.dr) + Phi1*dthetahat - Ref.ddr);
        
        tau = M*(falpha-f-z1-K2*z2) - Phi2*thetahat;
        
    % Input
        Tr([1 2 6],1) = tau;
        Ta = O6.controlAllocation(Tr,nu);
        if(~O6.UseProppeller)
            O6.Thrust = Tr;
        end
        O6.step(Ts);
    % Reference
        if(it == N/2), reta = reta2; end
        % Reference dynamics
        
            Ref.ddr = -Ref.K2*Ref.dr + Ref.K1*(reta-Ref.r);
            Ref.dr = Ref.dr + Ts*Ref.ddr;
            Ref.r = Ref.r + Ts*Ref.dr;
        
    % Save
        History.z(:,it) = [z1;z2];
        History.tau_r(:,it) = tau;
        History.tau_a(:,it) = O6.Thrust([1 2 6]);

        History.nu(:,it) = O6.State([1 2 6]);

        History.phi(:,it) = O6.State(12);
        History.pos(:,it) = O6.State(7:8);
        History.thetahat(:,it) = thetahat;
        
        History.reta(:,it) = reta;
        History.freta(:,it) = Ref.r;

    % Time update
        T = [T t];
        t = t+Ts;
    
end
disp('Simulation done!'), toc;

clear u v r

%% Plotting
close all

O6.plot(T)

%O6.plotPropeller(T)

disp('Press any key to show error'), pause
close all

title = 'Thrust requested vs. applied';
names = ["$\tau_u $","$\tau_v$","$\tau_r$","$\tau_u $","$\tau_v$","$\tau_r$"];
niceplot(T, [History.tau_a; History.tau_r], names, title, ["-","--"], ["time [s]", "[N]"], 'northwest');

title = 'Reference';
names = "$"+["r_u","r_v","r_\psi","r_u","r_v","r_\psi"]+"$";
niceplot(T, [History.reta; History.freta], names, title, ["--","-"], ["time [s]", "[-]"], 'southwest');

title = 'Velocity error';
names = ["$z_u$","$z_v$","$z_r$"];
niceplot(T,History.z(4:6,:), names, title, ["-"], ["time [s]", "$[\frac{m}{s}]$"], 'center');
title = 'Position error';
names = ["$z_x$","$z_y$","$z_{\psi}$"];
niceplot(T,History.z(1:3,:), names, title, ["-"], ["time [s]", "$[\frac{m}{s}]$"], 'east');


disp('Press any key to show adaptation'), pause
close all
maxtheta = max(History.thetahat,[],'all');
title = 'thetahat';
names = "$"+["X_{|u|u}" "X_{uuu}"]+"$";
niceplot(T,History.thetahat(1:2,:), names, title, ["--"], ["time [s]", "[-]"], 'southwest');
title = 'thetahat';
names = "$"+["Y_v" "Y_r" "Y_{|v|v}" "Y_{|r|v}" "Y_{|v|r}" "Y_{|r|r}"]+"$";
niceplot(T,History.thetahat(3:8,:), names, title, ["--"], ["time [s]", "[-]"], 'south');
title = 'thetahat';
names = "$"+["N_v" "N_{|v|v}" "N_{|r|v}" "N_{|v|r}" "N_{|r|r}"]+"$";
niceplot(T,History.thetahat(9:13,:), names, title, ["--"], ["time [s]", "[-]"], 'southeast');

title = 'thetahat';
names = "$"+["\nu_c,x","\nu_c,y"]+"$";
niceplot(T,History.thetahat(14:15,:), names, title, ["--"], ["time [s]", "[-]"], 'north');