%% Clean
    close all
    clear all
    clc

%% Init
    Ts = 1/100;
    N = 30000;
    t = 0; % Start time
    T = [];

%% Real Vessel
    state(12,1) = pi/4;
    O6 = Otter(state);
    O6.UseProppeller = true;
    habourPos = [15;10];
    O6.setHarbour(habourPos, 0, 3, 2)
    
    
    weathercondition = "mild" % mild, fair, harsh or extreme

    switch weathercondition
        case "mild"
            O6.setWind(0, 0);
            O6.setCurrent(0.1/toKnots(1),pi/4);
        case "fair"
            O6.setWind(4, 0);
            O6.setCurrent(0.5/toKnots(1),pi/4);
        case "harsh"
            O6.setWind(8, 0);
            O6.setCurrent(1/toKnots(1),pi/4);
        case "extreme"
            O6.setWind(12, 0);
            O6.setCurrent(2/toKnots(1),pi/4);
        otherwise
            warning('No such wheather condition defined')
    end
    
    O6.setWind(0, 0);
    O6.setCurrent(0,pi/4);

    clear state
%% Model

    Model = 'Models/Primitive/otter3mtrx_lin.mat'
    load(Model);

 

%% Controller
    Q = diag([1000 1000 10000]);    % Stateerror cost
    R = diag([0.1 10 1]);           % Input cost
    [K,P,E] = lqr(A,B,Q,R);

    
%% Course & Reference
    lookaheaddist = 2;
    pt = 1;
    
    %[P1,P2] = dockingCouse(O6.getPosition,habourPos-[6;0.1],habourPos+[1.5;0.1]);
    [P1,P2] = dockingCouse([5;5],habourPos-[5;0],habourPos+[1.5;0]);

    P = P1;
    nP = length(P);

    % Reference filter
    % Same filter is used for x, y and psi (position and heading)
    Ref.zeta = 1;
    Ref.wn = 0.2;
    Ref.K1 = diag([1 1 1])*Ref.wn^2;
    Ref.K2 = diag([1 1 1])*2*Ref.zeta*Ref.wn;

    Ref.reta =[ 0 0 pi/4 ]';
    Ref.r= [ 0 0 pi/4 ]';
    Ref.dr= [ 0 0 0 ]';
    Ref.ddr= [ 0 0 0 ]';
    
    umax = 1;
    umin = 0;
    k = 0.8;

%% Main Loop
disp('Running Simulation...'), tic;
for it = 1:N
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
        
     % Guidance

        if pt < nP
            pos = eta(1:2);
            target = P(1:2,pt);            targetdist = norm(target-pos);
            next = P(1:2,pt+1);            nextdist = norm(next-pos);
            if nextdist < lookaheaddist
                pt = pt + 1;
                %excess = lookaheaddist-nextdist;
                excess = 0;
            else
                excess = lookaheaddist-targetdist;
            end
            dir = [next-target;0];          dir = dir/norm(dir);
            reta = P(:,pt)+excess*dir;
        end
    
        if(it == floor(N*2/3)) 
            P = P2;
            pt = 1;
            nP = length(P);
        end
        
        z1 = R'*(eta - Ref.r);
                z1(3) = wrapToPi(z1(3));

    ref = diag([0.1 0.1 1])*(z1);
    
    tau = -K*nu_r + ref;
        
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
        History.tau_r(:,it) = tau;
        History.tau_a(:,it) = O6.Thrust([1 2 6]);
        %History.tau_a(:,it) = Ta([1 2 6]);

        History.nu(:,it) = O6.State([1 2 6]);

        History.phi(:,it) = O6.State(12);
        History.pos(:,it) = O6.State(7:8);
        
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

O6.plot(T,[P1 P2])
return
%O6.plotPropeller(T)

%%
disp('Press any key to show error'), pause
close all

title = 'Thrust requested vs. applied';
names = ["$\tau_u $","$\tau_v$","$\tau_r$","$\tau_u $","$\tau_v$","$\tau_r$"];
niceplot(T, [History.tau_a; History.tau_r], names, title, ["-","--"], ["time [s]", "[N]"], 'northwest');

title = 'Reference';
names = "$"+["r_u","r_v","r_\psi","r_u","r_v","r_\psi","r_u","r_v","r_\psi"]+"$";
niceplot(T, [History.reta; History.reta2; History.freta], names, title, ["--","-.","-"], ["time [s]", "[-]"], 'southwest');
title = 'Reference';
names = "$"+["r_u","r_v","r_\psi","r_u","r_v","r_\psi"]+"$";
niceplot(T, [History.reta2; History.freta], names, title, ["--","-"], ["time [s]", "[-]"], 'southwest');

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
names = "$"+["Y_v" "Y_r" "Y_{|v|v}" "Y_{|r|v}" "Y_{|v|r}" "Y_{|r|r}"]+"$";
niceplot(T,History.thetahat(3:8,:), names, title, ["--"], ["time [s]", "[-]"], 'south');
title = 'thetahat';
names = "$"+["N_v" "N_{|v|v}" "N_{|r|v}" "N_{|v|r}" "N_{|r|r}"]+"$";
niceplot(T,History.thetahat(9:13,:), names, title, ["--"], ["time [s]", "[-]"], 'southeast');

title = 'thetahat';
names = "$"+["\nu_c,x","\nu_c,y"]+"$";
niceplot(T,History.thetahat(14:15,:), names, title, ["--"], ["time [s]", "[-]"], 'north');



%% Plot cummulativ error
close all

z = History.z;

Epos = sqrt(sum(z(1:2,:).^2));
Ehead = abs(z(3,:));
Elinvelo = sqrt(sum(z(4:5,:).^2));
Eyawrate = abs(z(6,:));

cum.Epos = cumsum(Epos*Ts);
cum.Ehead = cumsum(Ehead*Ts);
cum.Elinvelo = cumsum(Elinvelo*Ts);
cum.Eyawrate = cumsum(Eyawrate*Ts);

E = cumsum( [Epos;Ehead;Elinvelo;Eyawrate]*Ts ,2);

title = 'error';
names = "$"+["pos","velo"]+"$";
niceplot(T,[Epos;Elinvelo], names, title, ["--"], ["time [s]", "[-]"], 'north');
title = 'cummulativ error';
names = "$"+["pos","head","velo","\psi"]+"$";
niceplot(T,[cum.Epos;cum.Ehead;cum.Elinvelo;cum.Eyawrate ], names, title, ["--"], ["time [s]", "[-]"], 'northeast');
title = 'cummulativ error';
names = "$"+["pos","head","velo","\psi"]+"$";
niceplot(T,E, names, title, ["--"], ["time [s]", "[-]"], 'southeast');

Efinal = E(:,end)

save("Results/Docking-"+weathercondition+".mat",'Efinal');
