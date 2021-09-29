close all
clear all
clc
load('Models/Primitive/SysEq6DOF_lin.mat')
syms u v w p q r x y z phi theta psi    real

%% Find Jacobian
idx = [1 2 6];
A3 = jacobian(dnu(idx), nu(idx))
B3 = jacobian(dnu(idx), tau(idx))
%G3 = jacobian(dnu(idx), eta(idx))

%% Print
if(0)
    latexeq("A",A3);
    latexeq("B",B3);
end

%%
subs(deta,[w, p, q, phi, theta], zeros(1,5))
%% Reduction
% Linearization point
lp = [3, 0, 0, 0, 0, 0];
A = subs(A3,[u, v, w, p, q, r], lp);
B = subs(B3,[u, v, w, p, q, r], lp);

latexeq("A",A);
latexeq("B",B);

S.A = double(A);
S.B = double(B);

save('Models/Primitive/otter3mtrx_lin.mat','-struct','S')
