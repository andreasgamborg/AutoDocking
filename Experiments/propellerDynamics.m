close all
clear all
clc
Ts = 1/500;
N = 2000;
T = [];
t = 0;

b = -1;
m = 0.1;

A = [0 1;0 b/m];
B = [0; 1/m];
x = [0;0];
r = [90; 0];

K = place(A,B,[-10 -20])
eig(A-B*K)

umax = 80;
for it = 1:N
    u = -K*(x - r);
    u(u>umax) = umax;
    u(u<-umax) = -umax;

    dx = A*x + B*u;
    x = x + Ts*dx;
    H.x(:,it) = x;
        H.u(:,it) = u;
    
    T = [T t];
    t = t+Ts;
end

figure()
plot(T, H.x')
figure()
plot(T, H.u')