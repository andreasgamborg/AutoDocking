%%
close all

disp('Making plots');

%% Plot data

title = 'Course';
niceplot(History.Pos(1,:),History.Pos(2,:), [], title, ["--"], ["x [m]", "y [m]"], 'north');
axis equal
grid

VesselPlot


title = 'Orientation';
names = ["$\phi$ roll", "$\theta$ pitch", "$\psi$ yaw"];
niceplot(T,History.Pos(4:6,:), names, title, ["--"], ["time [s]", "[rad]"], 'south');

title = 'Linear Velocities';
names = ["$u$ surge", "$v$ sway", "$w$ heave"];
niceplot(T,History.Velo(1:3,:), names, title, ["--"], ["time [s]", "[m/s]"], 'northeast');

title = 'Angular Velocities';
names = ["$p$ roll", "$q$ pitch", "$r$ yaw"];
niceplot(T,History.Velo(4:6,:), names, title, ["--"], ["time [s]", "[rad/s]"], 'southeast');

title = 'Propeller velocity';
names = ["P", "SB"];
niceplot(T,History.Propeller(3:4,:), names, title, ["-"], ["time [s]", "[rad/s]"], 'northwest');
title = 'Propeller Thrust';
names = ["P", "SB"];
niceplot(T,History.Propeller(5:6,:), names, title, ["-"], ["time [s]", "[N]"], 'southwest');

answer = questdlg('Would you like too close the plots?', ...
    'Plot tool', ...
    'Yes','No','Yes');
if strcmp(answer, 'Yes')
    close all
end



