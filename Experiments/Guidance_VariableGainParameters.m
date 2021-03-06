clear all
close all
clc

d = 0:0.1:30;

a1 = 0.5;
k1 = 0.1+0.9*( 0.5./(0.5 + (1-0.5)*exp(-a1*(d-10))));
% k1 = 0.1+0.1./(0.1+exp(-a1*(d-5)))
k2 = 0.6*exp(-((d-8)/3).^2);
k3 = 0.8*exp(-a1*(d));
k1 = 1-k2-k3;

k2 = exp(-((d-6)/3).^2);
    k3 = 1./(1+exp(4*(d-2)));
    k1 = 1-k2-k3;
    
    
        k2 = 0.6*exp(-((d-6)/3).^2);
    k3 = exp(-0.8*(d));
    k1 = 1-k2-k3;
    
    
figure
plot(d,[k1; k2; k3])


%%
umin = 0.1;
umax = 3;
speed = umin + (umax-umin)./(1+exp(-0.8*(d-10)));

figure
plot(d,speed)

