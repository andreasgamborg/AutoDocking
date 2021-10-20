close all
clear all
clc
load('Models/Primitive/A3.mat')
syms u v w p q r x y z phi theta psi    real
%%

A3 = subs(A3,v, 0)
%A3 = A3.*[1 0 0; 0 1 1; 0 1 1]
lambda = eig(A3); %lambda = simplify(lambda,100);

%%
u_list = -2:0.1:2;
uN = length(u_list);
r_list = -1:0.1:1;
rN = length(r_list);

HL = ones(rN,uN,3);
for ut = 1:uN
    for rt = 1:rN
        u = u_list(uN);
        r = r_list(rt);
        l = eval(lambda);
%         if  (real(l) > -eps)
%             disp('broke'); break
%         end
        HL(rt,ut,:) = real(l);
    end
end
%%
close all
figure
s1 = surf(u_list,r_list,HL(:,:,1)); s1.FaceColor = 'r'; s1.FaceAlpha = 0.6;
hold on
s2 = surf(u_list,r_list,HL(:,:,2)); s2.FaceColor = 'g'; s2.FaceAlpha = 0.6;
s3 = surf(u_list,r_list,HL(:,:,3)); s3.FaceColor = 'b'; s3.FaceAlpha = 0.6;
zlim([-5 1])
xlabel('u')
ylabel('r')
zlabel('RE(lambda)')


%%
syms u
A3 = subs(A3,u,3/toKnots(1));
lambda = eig(A3); %lambda = simplify(lambda,100);

r = [-10:0.1:10];
HL2 = eval(lambda);

RE = real(HL2);

figure
plot(r,RE')
