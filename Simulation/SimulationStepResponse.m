%% Clean
    close all
    clear all
    clc

%% Init
    Ts = 1/100;
    N = 5000;
    t = 0; % Start time
    T = [];

%% Real Vessel
    O6 = Otter6();
    O6.UseProppeller = false;
        tau = [0 0 0]';

%% Main Loop
disp('Running Simulation...')
for it = 1:N
 
    % Input
        if(it == 1/Ts)
            tau = [0 0 1]';
        end
        Tr([1 2 6],1) = tau;
%         Ta = O6.controlAllocation(Tr,nu);
        if(~O6.UseProppeller)
            O6.Thrust = Tr;
        end
        
        O6.step(Ts);

    % Time update
        T = [T t];
        t = t+Ts;
    
end
disp('Simulation done!')

%% Plotting
close all
O6.plot(T)
