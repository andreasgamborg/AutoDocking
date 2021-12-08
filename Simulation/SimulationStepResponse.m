%% Clean
    close all
    clear all
    %clc

%% Init
    Ts = 1/100;
    N = 10000;
    t = 0; % Start time
    T = [];

%% Real Vessel
    O6 = Otter();
    O6.UseProppeller = false;
    %O6.setCurrent(0.1,pi)
    %O6.setPayload(100,[1;0;0])
        tau = [0 0 0]';

%% Main Loop
disp('Running Simulation...'), tic;
for it = 1:N
 
    % Input
        if(it == 1000)
            tau = [0 1 0]';
            %O6.setPayload(100,[1;0;0])
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
disp('Simulation done!'), toc;

%% Plotting
close all
O6.plot(T)
