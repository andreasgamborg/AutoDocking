close all
clear all

N = 5000;
sd = 5/2;                           % 5m approx. (2drms)
theta = rand(1,N)*2*pi;
R = [cos(theta); sin(theta)] * sd.*randn(1,N);

r = 2*sd;
theta = linspace(0,2*pi);
x = r*cos(theta);
y = r*sin(theta);

figure
plot(R(1,:),R(2,:),'.'), hold on
plot(x,y)
axis equal

count = 0;

for p = R
    if(norm(p) < r)
        count = count + 1;
    end
end

count/N