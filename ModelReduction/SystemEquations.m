close all
clear all
clc
%%
load('Models/Primitive/otter6mtrx.mat')


syms u v w p q r x y z phi theta psi    real
x = [u v w p q r x y z phi theta psi]';
nu = x(1:6);
syms tau [6 1] real

M = MRB + MA;
C = CRB + CA;
A6 = -M\(C + D);
B6 = inv(M);
dnu = A6*nu + B6*tau;
dnu = simplify(dnu);

if(1)           %<-- Switch
    names = ["\dot{u}","\dot{v}","\dot{w}","\dot{p}","\dot{q}","\dot{r}"]
    for it = 1:6
        latexeq(names(it),dnu(it));
    end
end
%% Find Jacobian
idx = [1 2 6];
A = jacobian(dnu, nu(idx));
B = jacobian(dnu, tau(idx));

%% Truncate
latexeq("A",A(idx,idx));
latexeq("B",B(idx,idx));

%% Reduction
w = 0;
p = 0;
q = 0;

%A = subs(A,[w, p, q],[0, 0, 0]);
A = eval(A);

latexeq("A",A(idx,idx))
latexeq("B",B(idx,idx))
