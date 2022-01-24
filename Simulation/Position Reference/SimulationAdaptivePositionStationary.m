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
    O6 = Otter();
    O6.UseProppeller = true;
   
    O6.setCurrent(0,pi/2)
    O6.setWind(0,0)
    clear state
%% Model

    Model = 'Models\Primitive\otter6mtrx.mat'
    load(Model,'MRB');
    load(Model,'MA');

    % Known Constant parameters
    M = MRB + MA;
    M = M([1 2 6],[1 2 6]);

    Xu = -77.5544;
    Yv = 0;
    Nr = -49.3007;
    d = diag([Xu Yv Nr])';

    clear MA MRB Xu Nr Model

%% Control gains
    K1 = diag([2, 2, 1])*0.8;
    K2 = diag([1.3, 1, 1.1])*1.05;
    ftau = zeros(3,1);
    
%% Adaptation Init
    Na = 14;
    thetahat = zeros(Na,1);
    Gamma1 = eye(12)*0.1;
    Gamma2 = eye(2)*0.06;

    Gamma = [   Gamma1 zeros(12,2)
                zeros(2,12) Gamma2  ];
            
    Phi1 = zeros(3,Na);
    Phi1([1 2],[13 14]) = eye(2);
    
    iM = inv(M);

%% Reference
    % Reference filter
    % Same filter is used for x, y and psi (position and heading)
    Ref.zeta = 1;
    Ref.wn = 0.2;
    Ref.K1 = diag([1 1 1])*Ref.wn^2;
    Ref.K2 = diag([1 1 1])*2*Ref.zeta*Ref.wn;

    Ref.reta =[ 0 0 0 ]';
    Ref.r= [ 0 0 0 ]';
    Ref.dr= [ 0 0 0 ]';
    Ref.ddr= [ 0 0 0 ]';
    
    reta = [ 0 0 0 ]';
    
%% Wind
    Wind = load('Wind\wind4.mat');
    Vc = 0.2;
    dirc = pi/4;
%% Main Loop
disp('Running Simulation...'), tic;
for it = 1:N
    % Wind & current
        itw = mod(it,7000)+1;
        %O6.setWind(Wind.speed(itw), Wind.direction(itw));
        O6.setCurrent(            Vc,           dirc);
    % State
        nu_c = O6.getWaterVelo();
        nu_r = O6.State([1 2 6]) - nu_c([1 2 6]) ;
        eta = O6.State([7 8 12]);

        u = nu_r(1);
        v = nu_r(2);
        r = nu_r(3);
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
        Phi2(2, 3:7) = [r abs(v)*v abs(r)*v abs(v)*r abs(r)*r];
        Phi2(3, 8:14) = [v abs(v)*v abs(r)*v abs(v)*r abs(r)*r 0 0];
        
    % Error
        z1 = R'*(eta - Ref.r);
        z1(3) = wrapToPi(z1(3));
        alpha = -K1*z1 - R'*(Phi1*thetahat-Ref.dr);
        z2 = nu_r - alpha;
    
    % Adaptation
        dthetahat = Gamma * (Phi1'*R*(z1 + K1'*z2) + Phi2'*iM*z2);
        thetahat = thetahat + Ts*dthetahat;
        
        
    % Control
        C = [   - 11*r^2 - 137.5000*v*r
            60.5000*r*u
            11*r*u];
        
        f = M\(d*nu_r - C);
        falpha = -K1*( S'*z1 + z2 - K1*z1 ) - R'*(S'*(Phi1*thetahat-Ref.dr) + Phi1*dthetahat - Ref.ddr);
        
        tau = M*(falpha-f-z1-K2*z2) - Phi2*thetahat;
        
    % Input
        Tr([1 2 6],1) = tau;
        Ta = O6.controlAllocation(Tr,nu_r);
        if(~O6.UseProppeller)
            O6.Thrust = Tr;
        end
        O6.step(Ts);

    % Reference dynamics
    
        Ref.reta = Ref.reta + Ts*1/4*(reta-Ref.reta);
        Ref.ddr = -Ref.K2*Ref.dr + Ref.K1*(Ref.reta-Ref.r);
        Ref.dr = Ref.dr + Ts*Ref.ddr;
        Ref.r = Ref.r + Ts*Ref.dr;
        
    % Save
        History.z(:,it) = [z1;z2];
        History.tau_r(:,it) = tau;
        History.tau_a(:,it) = O6.Thrust([1 2 6]);
        %History.tau_a(:,it) = Ta([1 2 6]);

        History.nu(:,it) = O6.State([1 2 6]);

        History.phi(:,it) = O6.State(12);
        History.pos(:,it) = O6.State(7:8);
        History.thetahat(:,it) = thetahat;
        
        History.reta(:,it) = reta;
        History.reta2(:,it) = Ref.reta;
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
names = "$"+["r_u","r_v","r_\psi","r_u","r_v","r_\psi","r_u","r_v","r_\psi"]+"$";
niceplot(T, [History.reta; History.reta2; History.freta], names, title, ["--","-.","-"], ["time [s]", "[-]"], 'southwest');

title = 'Velocity error';
names = ["$z_u$","$z_v$","$z_r$"];
niceplot(T,History.z(4:6,:), names, title, ["-"], ["time [s]", "$[\frac{m}{s}]$"], 'center');
title = 'Position error';
names = ["$z_x$","$z_y$","$z_{\psi}$"];
niceplot(T,History.z(1:3,:), names, title, ["-"], ["time [s]", "$[m]$"], 'northeast');

disp('Press any key to show adaptation'), pause
close all
maxtheta = max(History.thetahat,[],'all');
title = 'thetahat';
names = "$"+["X_{|u|u}" "X_{uuu}"]+"$";
niceplot(T,History.thetahat(1:2,:), names, title, ["--"], ["time [s]", "[-]"], 'southwest');
title = 'thetahat';
names = "$"+["Y_r" "Y_{|v|v}" "Y_{|r|v}" "Y_{|v|r}" "Y_{|r|r}"]+"$";
niceplot(T,History.thetahat(3:7,:), names, title, ["--"], ["time [s]", "[-]"], 'south');
title = 'thetahat';
names = "$"+["N_v" "N_{|v|v}" "N_{|r|v}" "N_{|v|r}" "N_{|r|r}"]+"$";
niceplot(T,History.thetahat(8:12,:), names, title, ["--"], ["time [s]", "[-]"], 'southeast');

title = 'thetahat';
names = "$"+["\nu_c,x","\nu_c,y"]+"$";
niceplot(T,History.thetahat(13:14,:), names, title, ["--"], ["time [s]", "[-]"], 'north');