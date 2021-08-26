%%
close all

disp('Making plots');

%% Plot data

title = 'Course';
niceplot(History.Pos(1,:),History.Pos(2,:), [], title, ["--"], ["x [m]", "y [m]"], 'north');
axis equal
grid
set(gca, 'YDir','reverse')

color = 'g-';
for i = [1:1000:N, N]
    
    vessel = vesselplot(History.Pos(6,i),History.Propeller(1:2,i));
    
    if i == N
        color = 'r-';
    end
    
    plot(vessel(1,:)+History.Pos(1,i), vessel(2,:)+History.Pos(2,i), color, 'LineWidth', 2);
    
    color = 'b-';
end


title = 'Linear Velocities';
names = ["$u$ surge", "$v$ sway", "$w$ heave"];
niceplot(T,History.Velo(1:3,:), names, title, ["--"], ["time [s]", "[m/s]"], 'northeast');

title = 'Orientation';
names = ["$\phi$ roll", "$\theta$ pitch", "$\psi$ yaw"];
if(1)
    niceplot(T,rad2deg(History.Pos(4:6,:)), names, title, ["--"], ["time [s]", ""], 'south');
    yticks(-180:10:180)
    yticklabels([-180:10:180]+"°")
else
    niceplot(T,History.Pos(4:6,:), names, title, ["--"], ["time [s]", "[rad]"], 'south');
end

title = 'Angular Velocities';
names = ["$p$ roll", "$q$ pitch", "$r$ yaw"];
if(1)
    niceplot(T,rad2deg(History.Velo(4:6,:)*60), names, title, ["--"], ["time [s]", "[deg/min]"], 'southeast');
    ytickformat('%.0f°')
else
    niceplot(T,History.Velo(4:6,:), names, title, ["--"], ["time [s]", "[rad/s]"], 'southeast');
end

title = 'Propeller velocity';
names = ["P", "SB"];
niceplot(T,History.Propeller(3:4,:), names, title, ["-"], ["time [s]", "[rad/s]"], 'northwest');
title = 'Propeller Thrust';
names = ["P", "SB"];
niceplot(T,History.Propeller(5:6,:), names, title, ["r-","g--"], ["time [s]", "[N]"], 'southwest');

answer = questdlg('Would you like too close the plots?', ...
    'Plot tool', ...
    'Yes','No','Yes');
if strcmp(answer, 'Yes')
    close all
end



