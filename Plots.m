close all
%% Plot data

title = 'Course';
niceplot(History.Pos(1,:),History.Pos(2,:), [], title, ["--"], ["x [m]", "y [m]"], 'north');
axis equal
    grid

 scale = [   min(History.Pos(:,1))  max(History.Pos(:,1));
        min(History.Pos(:,2))  max(History.Pos(:,2))
        ];
    m = max(max(abs(scale)));

    
    modelx = [2,5,2,-4,-4,-2,-2,-4,-4,2,5,2,2];
    modely = [1,2,3,3,1,1,-1,-1,-3,-3,-2,-1,1];
    model = [modelx;modely]*2/9;

    atSample = [1:1000:N, N];
    color = 'g-';
    for i = atSample
        R = [cos(-History.Pos(6,i)) sin(-History.Pos(6,i)); -sin(-History.Pos(6,i)) cos(-History.Pos(6,i))];
        modelR = R*model;
        
        if i == N    
            color = 'r-';        
        end
        
        plot(modelR(1,:)+History.Pos(1,i), modelR(2,:)+History.Pos(2,i), color, 'LineWidth', 2);
        color = 'b-';
    end
    


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
niceplot(T,History.Propeller(1:2,:), names, title, ["-"], ["time [s]", "[rad/s]"], 'northwest');
title = 'Propeller Thrust';
names = ["P", "SB"];
niceplot(T,History.Propeller(3:4,:), names, title, ["-"], ["time [s]", "[Nm]"], 'southwest');
%% Back up
% figure(1)
% plot(History.Pos(1,:),History.Pos(2,:),'linewidt',2)
% figure(2)
% plot(T,History.Pos(4:6,:),'linewidt',2)
% legend('phi: roll angle (rad)', 'theta: pitch angle (rad)', 'psi: yaw angle (rad)')
% figure(3)
% plot(T,History.Velo(1:3,:),'linewidt',2)
% legend('u: surge velocity (m/s)', 'v: sway velocity (m/s)', 'w: heave velocity (m/s)')
% figure(4)
% plot(T,History.Velo(4:6,:),'linewidt',2)
% legend('p: roll velocity (rad/s)', 'q: pitch velocity (rad/s)', 'r: yaw velocity (rad/s)')
% 
% figure(5)
% plot(T,[History.Pos(1:3,:);History.Velo(1:3,:)],'linewidt',2)
% title('All')
% legend('x:     position in x direction (m)', 'y:     position in y direction (m)', 'z:     position in z direction (m)', ...
% 'u: surge velocity (m/s)', 'v: sway velocity (m/s)', 'w: heave velocity (m/s)')
