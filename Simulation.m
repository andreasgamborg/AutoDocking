%% Clean
close all
clear all
clc
%% Init
Ts = 1/50;
N = 12000;
t = 0; % Start time
T = [];

% Init State
State.x = zeros(12,1);
State.prop.velo = [30 10]';      % n = [ n_left n_right ]'
State.prop.xi   = [0 0]';      % n = [ n_left n_right ]'
State.mp = 0;              % payload mass (kg), max value 45 kg
State.rp = [0 0 0]';    % location of payload (m)
State.V_c = 0;              % current speed (m/s)
State.beta_c = 0; % current direction (rad);

% Init History
History.Pos = zeros(6,N);
History.Velo = zeros(6,N);

%% Main Loop
thrust_seq = makeref([20 30 20 10 20; 20 10 20 30 20], N/5);
thrust_seq = makeref([20 50 10; 20 50 10], N/3);

disp('Running Simulation...')
for i = 1:N
    State.prop_velo = thrust_seq(:,i);
    [x, info] = ferry(State);
    State.x = State.x + Ts * x;
    
    T = [T t];
    History.Velo(:,i) = State.x(1:6);
    History.Pos(:,i) = State.x(7:12);
    History.Propeller(:,i) = [State.prop_velo; info.Thrust];
    
    t = t+Ts;
end
disp('Simulation done!')

Plots
