%% Clean
% close all
% clear all
% clc

%% Init
Ts = 1/50;
N = 12000;
t = 0; % Start time
T = [];

% Init State
State.x = zeros(6,1);
State.prop.velo = [0; 0];           % velo = [ port starboard ]'
State.prop.xi   = [0; 0];           % xi = [ port starboard ]'
State.mp = 0;                       % payload mass (kg)
State.rp = [0 0]';                % location of payload (m)
State.V_c = 0;                      % current speed (m/s)
State.current.v = 0;                      % current speed (m/s)
State.beta_c = 0;                   % current direction (rad);
State.current.beta = 0;                   % current direction (rad);

% Init History
% History.Pos = zeros(6,N);
% History.Velo = zeros(6,N);

%% Main Loop
% Idle
%      thrust_seq = makeref([0; 0; 0; 0], N);
% Full - Break - Full
     %thrust_seq = makeref([30 0 30; 30 0 30; 0 0 0 ; 0 0 0], N/3);
% S-Line
%     thrust_seq = makeref([30 30 20 30; 30 20 30 30; 0 0 0 0; 0 0 0 0], N/4);
% Port Thruster half rotation
    %thrust_seq = makeref([20 20 20 20 20; 0 0 0 0 0 ; 0 pi/4 pi/2 3*pi/4 pi; 0 0 0 0 0], N/5);
% Starbord Thruster half rotation
    %thrust_seq = makeref([0 0 0 0 0; 20 20 20 20 20 ; 0 0 0 0 0; 0 -pi/4 -pi/2 -3*pi/4 -pi], N/5);
% Slalom by use of thruster angle
    a = 12;
    thrust_seq = makeref([20 20 20 20 20; 20 20 20 20 20 ; 0 -pi/a pi/a -pi/a pi/a; 0 -pi/a pi/a -pi/a pi/a], N/5);
    
disp('Running Simulation...')
for i = 1:N
    State.prop.velo = thrust_seq(1:2,i);
    State.prop.xi = thrust_seq(3:4,i);
    [x, info] = ferry3(State);
    State.x = State.x + Ts * x;
    
    T = [T t];
    History.Velo(:,i) = State.x(1:3);
    History.Pos(:,i) = State.x(4:6);
    History.Propeller(:,i) = [State.prop.xi; State.prop.velo; info.Thrust];
    
    t = t+Ts;
end
disp('Simulation done!')

Plots3
