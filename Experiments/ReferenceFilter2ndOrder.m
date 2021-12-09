clear all
close all

Ts = 1/100;
N = 600;
    t = 0; % Start time
    T = [];

    zeta = 1;
    wn = 3;
K1 = diag([1 1 1])*wn^2;
K2 = diag([1 1 1])*2*zeta*wn;

r= [ 0 0 0 ]';
dr= [ 0 0 0 ]';

u = [3 2 1]';
for it = 1:N
    h.r(:,it) = r;
    ddr = -K2*dr+K1*(u-r);
    dr = dr+Ts*ddr;
    r = r + Ts*dr;
        % Time update
        T = [T t];
        t = t+Ts;
end


title = 'Reference filter';
names = ["$r_u $","$r_v$","$r_\psi$"];
niceplot(T, h.r , names, title, ["-"], ["time [s]", "[N]"], 'northwest');