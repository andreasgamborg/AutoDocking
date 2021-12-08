
close all
clear all
clc


Ts = 1/100;
N = 5000;
t = 0; % Start time
T = [];

x = zeros(12,1);

% Load condition
mp = 0;               % payload mass (kg), max value 45 kg
rp = [0 0 -0.0]';     % location of payload (m)
% Current
V_c = 0;               % current speed (m/s)
beta_c = 30 * pi/180;  % current direction (rad)

tau = zeros(6,1);

for it = 1:N
    
    if(it == 1000)
        tau(2) = 1;
    end
    if(it == 3000)
        tau(2) = 1;
    end
    x = x + Ts * otter(x,tau,mp,rp,V_c,beta_c);
    
    % Time update
    T = [T t];
    t = t+Ts;
    History.Velo(:,it) = x(1:6);
    History.Pos(:,it) = x(7:12);
end

% title = 'Position';
% names = "$"+["x", "y", "\psi"]+"$";
% niceplot(T,History.x([7 8 12],:), names, title, ["-"], ["time [s]", "m - rad"], 'north');
% title = 'Velocities';
% names = "$"+["u", "v", "r"]+"$";
% niceplot(T,History.x([1 2 6],:), names, title, ["-"], ["time [s]", "m/s - rad/s"], 'south');


title = 'Linear Velocities';
names = ["$u$", "$v$", "$w$"];
niceplot(T,toKnots(History.Velo(1:3,:)), names, title, ["-"], ["time [s]", "[knot]"], 'north');

title = 'Angular Velocities';
names = ["$p$", "$q$", "$r$"];
niceplot(T, rad2deg(History.Velo(4:6,:))*60, names, title, ["-"], ["time [s]", "[deg/min]"], 'northeast');
ytickformat('%.0f°')


title = 'Position';
names = ["$x$ north", "$y$ east", "$z$ down"];
niceplot(T,History.Pos(1:3,:), names, title, ["-"], ["time [s]", "[m]"], 'south');

title = 'Orientation';
names = ["$\phi$ roll", "$\theta$ pitch", "$\psi$ yaw"];
niceplot(T,rad2deg(History.Pos(4:6,:)), names, title, ["-"], ["time [s]", ""], 'southeast');
yticks(-180:30:180)
tl = [180:30:359 0:30:180]+"°"; tl(7) = "N"; tl(10) = "E"; tl([1,13]) = "S"; tl(4) = "W";
yticklabels(tl)





