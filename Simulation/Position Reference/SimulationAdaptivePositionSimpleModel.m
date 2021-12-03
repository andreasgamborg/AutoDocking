%% Clean
    close all
    clear all
    clc

%% Init
    Ts = 1/100;
    N = 10000;
    t = 0; % Start time
    T = [];
    
%% Model
    nu = zeros(3,1);
    eta = zeros(3,1);
    [M,C,D] = sysmtrx3DOF(nu);
    Xu = -77.5544;
    Yv = -122;
    Nr = -49.3007;
    d = diag([Xu Yv Nr])';

    clear MA MRB Xu Nr Model

%% Control gains
    K1 = diag([2, 2, 1])*0.8;
    K2 = diag([1.3, 1, 1.1])*1.05;
    
%% Adaptation Init
    Na = 14;
    thetahat = zeros(Na,1);
    Gamma1 = eye(12)*0.1;
    %Gamma1 = diag([5 5 100 50000 100 100 100 100 50000 100 100 100 100]);

    Gamma2 = eye(2)*0.06;

    Gamma = [   Gamma1 zeros(12,2)
                zeros(2,12) Gamma2  ];
            
    Phi1 = zeros(3,Na);
    Phi1([1 2],[13 14]) = eye(2);
    
    iM = inv(M);

%% Reference
%     reta1=[8 -2 -pi/4]';
%     reta2=[20 -6 -pi/2]';
    
%     reta1 = [1 1 0]';
%     reta2 = [2 1 0]';
%     
    reta1 = [1 1 0]';
    reta2 = [1.2 1 0]';

    reta = reta1;
    
    dreta = zeros(3,1); 
    ddreta = zeros(3,1); 

%% Main Loop
disp('Running Simulation...')
for it = 1:N
    % State
                
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
        Phi2(1, 1:2) =  [abs(u)*u u^3];
        Phi2(2, 3:7) =  [r abs(v)*v abs(r)*v abs(v)*r abs(r)*r];
        Phi2(3, 8:14) = [v abs(v)*v abs(r)*v abs(v)*r abs(r)*r 0 0];
        
    % Error
        z1 = R'*(eta - reta);
        %z1(3) = wrapToPi(z1(3));
        alpha = -K1*z1 - R'*(Phi1*thetahat-dreta);
        z2 = nu - alpha;
    
    % Adaptation
        dthetahat = Gamma * (Phi1'*R*(z1 + K1'*z2) + Phi2'*iM*z2);
        thetahat = thetahat + Ts*dthetahat;
        
    % Control
        
        f = M\(d*nu - C*nu);
        f = M\(-D*nu - C*nu);
        falpha = -K1*( S'*z1 + z2 - K1*z1 ) - R'*(S'*(Phi1*thetahat-dreta) + Phi1*dthetahat - ddreta);
        
        tau = M*(falpha-f-z1-K2*z2) - Phi2*thetahat;
        maxtau = 100;
        tau = max(-maxtau, min(maxtau,tau));

    % Model step
        [M,C,D] = sysmtrx3DOF(nu);
        dnu = M\(tau-C*nu-D*nu);
        deta = R*nu;
        
        nu = nu + Ts*dnu;
        eta = eta + Ts*deta;

    % Reference
        %if(norm(z1)<0.01), reta = reta2; end
        if(it == N/2), reta = reta2; end
    % Save
        History.z(:,it) = [z1;z2];
        History.tau_r(:,it) = tau;
        History.tau_a(:,it) = tau;

        History.nu(:,it) = nu;

        History.phi(:,it) = eta(3);
        History.pos(:,it) = eta(1:2);
        History.thetahat(:,it) = thetahat;
    
    % Time update
        T = [T t];
        t = t+Ts;
    
end
disp('Simulation done!')

clear u v r

%% Plotting
close all

title = 'Position';
names = ["$x$ north", "$y$ east"];
niceplot(T,History.pos(1:2,:), names, title, ["-"], ["time [s]", "[m]"], 'south');

disp('Press any key to show error'), pause
close all

title = 'Thrust requested vs. applied';
names = ["$\tau_u $","$\tau_v$","$\tau_r$","$\tau_u $","$\tau_v$","$\tau_r$"];
niceplot(T, [History.tau_a; History.tau_r], names, title, ["-","--"], ["time [s]", "[N]"], 'west');
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
names = "$"+["Y_r" "Y_{|v|v}" "Y_{|r|v}" "Y_{|v|r}" "Y_{|r|r}"]+"$";
niceplot(T,History.thetahat(3:7,:), names, title, ["--"], ["time [s]", "[-]"], 'south');
title = 'thetahat';
names = "$"+["N_v" "N_{|v|v}" "N_{|r|v}" "N_{|v|r}" "N_{|r|r}"]+"$";
niceplot(T,History.thetahat(8:12,:), names, title, ["--"], ["time [s]", "[-]"], 'southeast');

title = 'thetahat';
names = "$"+["\nu_c,x","\nu_c,y"]+"$";
niceplot(T,History.thetahat(13:14,:), names, title, ["--"], ["time [s]", "[-]"], 'north');