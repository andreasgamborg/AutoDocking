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
dnu =A6*nu;
names = ["\dot{u}","\dot{v}","\dot{w}","\dot{p}","\dot{q}","\dot{r}"]
for it = 1:6
    latexeq(names(it),dnu(it));
end

%% Find Jacobian
A = jacobian(dnu, nu);
B = jacobian(dnu, tau);

%% Truncate
idx = [1 2 6];
latexeq("A",A(idx,idx));
