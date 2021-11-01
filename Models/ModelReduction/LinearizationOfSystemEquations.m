close all
clear all
clc
load('Models/Primitive/SysEq6DOF.mat')
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
lp = [3/toKnots(1), 0, 0, 0, 0, 0];
A = subs(A3,[u, v, w, p, q, r], lp);
B = subs(B3,[u, v, w, p, q, r], lp);

% Measurements
C(1,1) = toKnots(1);            % knot per m/s
C(2,2) = toKnots(1);            % knot per m/s
C(3,3) = 60*180/pi;             % deg/min per rad/s

Cm = C(1,:); 

latexeq("\bm{A}",A);
latexeq("\bm{B}",B);
latexeq("\bm{C}",C);

S.A = double(A);
S.B = double(B);
S.C = C;
S.Cm = Cm;
S.linarizationPoint = lp;

% save('Models/Primitive/otter3mtrx_lin.mat','-struct','S')
